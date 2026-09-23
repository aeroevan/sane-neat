// SPDX-License-Identifier: GPL-2.0-or-later WITH SANE-exception
// Copyright (C) 2026 Evan McClain
/*
 * SANE backend "neat" for the Neat NM-1000 mobile sheetfed scanner.
 *
 * Each sane_start() scans one sheet: the page is read into memory until the
 * trailing edge is detected, cropped to the requested window and the sheet
 * ejected, so frontends always get exact parameters. With no sheet in the
 * feeder sane_start() returns SANE_STATUS_NO_DOCS, which ends an ADF batch.
 *
 * Debug output: SANE_DEBUG_NEAT=1..4
 */
#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nm1000.h"
#include <sane/sane.h>

#define BACKEND_BUILD 1
#define EXPORT __attribute__((visibility("default")))

#define MM_PER_INCH 25.4
#define MAX_WIDTH_MM 215.9    /* 8.5" sensor */
#define MAX_LENGTH_MM 900.0   /* long receipts; the scan stops at the trailing edge */

static int debug_level;

static void DBG(int level, const char *fmt, ...)
{
    va_list ap;
    if (level > debug_level)
        return;
    fprintf(stderr, "[neat] ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static void core_log(int level, const char *msg)
{
    DBG(level + 1, "%s\n", msg);
}

enum {
    OPT_NUM_OPTS,
    OPT_MODE_GROUP,
    OPT_MODE,
    OPT_RESOLUTION,
    OPT_SOURCE,
    OPT_GEOMETRY_GROUP,
    OPT_TL_X,
    OPT_TL_Y,
    OPT_BR_X,
    OPT_BR_Y,
    NUM_OPTIONS
};

static SANE_String_Const mode_list[] = {"Color", "Gray", NULL};
static SANE_String_Const source_list[] = {"ADF", NULL};
static SANE_Word resolution_list[] = {4, 150, 200, 300, 600};
static const SANE_Range x_range = {SANE_FIX(0), SANE_FIX(MAX_WIDTH_MM), 0};
static const SANE_Range y_range = {SANE_FIX(0), SANE_FIX(MAX_LENGTH_MM), 0};

struct scanner {
    struct scanner *next;
    char name[32];
    int bus, address;
    SANE_Device sane;

    struct nm1000 *dev;
    SANE_Option_Descriptor opt[NUM_OPTIONS];
    char mode[16];
    SANE_Word resolution;
    SANE_Fixed tl_x, tl_y, br_x, br_y;

    /* current page */
    SANE_Parameters params;
    uint8_t *page;
    size_t page_len, page_pos;
    int scanning;
    volatile int cancelled;
};

static struct scanner *scanners;
static const SANE_Device **device_list;

/* ------------------------------------------------------------------------ */

static SANE_Status map_status(int r)
{
    switch (r) {
    case NM1000_OK:
    case NM1000_EOF: return SANE_STATUS_GOOD;
    case NM1000_ERR_NOPAPER: return SANE_STATUS_NO_DOCS;
    case NM1000_ERR_NODEV: return SANE_STATUS_ACCESS_DENIED;
    case NM1000_ERR_BUSY: return SANE_STATUS_DEVICE_BUSY;
    case NM1000_ERR_NOMEM: return SANE_STATUS_NO_MEM;
    case NM1000_ERR_INVAL: return SANE_STATUS_INVAL;
    case NM1000_ERR_CANCELLED: return SANE_STATUS_CANCELLED;
    default: return SANE_STATUS_IO_ERROR;
    }
}

static void free_scanners(void)
{
    while (scanners) {
        struct scanner *s = scanners;
        scanners = s->next;
        free(s->page);
        free(s);
    }
    free(device_list);
    device_list = NULL;
}

static void probe_devices(void)
{
    libusb_context *ctx;
    libusb_device **list;
    ssize_t n;

    free_scanners();
    if (libusb_init(&ctx))
        return;
    n = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) || desc.idVendor != NM1000_VENDOR ||
            desc.idProduct != NM1000_PRODUCT)
            continue;
        struct scanner *s = calloc(1, sizeof(*s));
        if (!s)
            break;
        s->bus = libusb_get_bus_number(list[i]);
        s->address = libusb_get_device_address(list[i]);
        snprintf(s->name, sizeof(s->name), "libusb:%03d:%03d", s->bus, s->address);
        s->sane.name = s->name;
        s->sane.vendor = "Neat";
        s->sane.model = "NM-1000";
        s->sane.type = "sheetfed scanner";
        s->next = scanners;
        scanners = s;
        DBG(2, "found NM-1000 at %s\n", s->name);
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
}

static void init_options(struct scanner *s)
{
    SANE_Option_Descriptor *o = s->opt;

    for (int i = 0; i < NUM_OPTIONS; i++) {
        o[i].size = sizeof(SANE_Word);
        o[i].cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
    }

    o[OPT_NUM_OPTS].name = "";
    o[OPT_NUM_OPTS].title = "Number of options";
    o[OPT_NUM_OPTS].desc = "Read-only option that specifies how many options a specific device supports.";
    o[OPT_NUM_OPTS].type = SANE_TYPE_INT;
    o[OPT_NUM_OPTS].cap = SANE_CAP_SOFT_DETECT;

    o[OPT_MODE_GROUP].name = "";
    o[OPT_MODE_GROUP].title = "Scan Mode";
    o[OPT_MODE_GROUP].desc = "";
    o[OPT_MODE_GROUP].type = SANE_TYPE_GROUP;
    o[OPT_MODE_GROUP].cap = 0;
    o[OPT_MODE_GROUP].size = 0;

    o[OPT_MODE].name = "mode";
    o[OPT_MODE].title = "Scan mode";
    o[OPT_MODE].desc = "Selects the scan mode (e.g., lineart, monochrome, or color).";
    o[OPT_MODE].type = SANE_TYPE_STRING;
    o[OPT_MODE].size = sizeof(s->mode);
    o[OPT_MODE].constraint_type = SANE_CONSTRAINT_STRING_LIST;
    o[OPT_MODE].constraint.string_list = mode_list;
    snprintf(s->mode, sizeof(s->mode), "Color");

    o[OPT_RESOLUTION].name = "resolution";
    o[OPT_RESOLUTION].title = "Scan resolution";
    o[OPT_RESOLUTION].desc = "Sets the resolution of the scanned image.";
    o[OPT_RESOLUTION].type = SANE_TYPE_INT;
    o[OPT_RESOLUTION].unit = SANE_UNIT_DPI;
    o[OPT_RESOLUTION].constraint_type = SANE_CONSTRAINT_WORD_LIST;
    o[OPT_RESOLUTION].constraint.word_list = resolution_list;
    s->resolution = 300;

    o[OPT_SOURCE].name = "source";
    o[OPT_SOURCE].title = "Scan source";
    o[OPT_SOURCE].desc = "Selects the scan source (such as a document-feeder).";
    o[OPT_SOURCE].type = SANE_TYPE_STRING;
    o[OPT_SOURCE].size = 8;
    o[OPT_SOURCE].constraint_type = SANE_CONSTRAINT_STRING_LIST;
    o[OPT_SOURCE].constraint.string_list = source_list;

    o[OPT_GEOMETRY_GROUP].name = "";
    o[OPT_GEOMETRY_GROUP].title = "Geometry";
    o[OPT_GEOMETRY_GROUP].desc = "";
    o[OPT_GEOMETRY_GROUP].type = SANE_TYPE_GROUP;
    o[OPT_GEOMETRY_GROUP].cap = 0;
    o[OPT_GEOMETRY_GROUP].size = 0;

    static const struct {
        int idx;
        const char *name, *title, *desc;
        const SANE_Range *range;
    } geo[] = {
        {OPT_TL_X, "tl-x", "Top-left x", "Top-left x position of scan area.", &x_range},
        {OPT_TL_Y, "tl-y", "Top-left y", "Top-left y position of scan area.", &y_range},
        {OPT_BR_X, "br-x", "Bottom-right x", "Bottom-right x position of scan area.", &x_range},
        {OPT_BR_Y, "br-y", "Bottom-right y",
         "Bottom-right y position of scan area. The scan also ends at the end of the sheet.", &y_range},
    };
    for (size_t i = 0; i < sizeof(geo) / sizeof(geo[0]); i++) {
        SANE_Option_Descriptor *g = &o[geo[i].idx];
        g->name = geo[i].name;
        g->title = geo[i].title;
        g->desc = geo[i].desc;
        g->type = SANE_TYPE_FIXED;
        g->unit = SANE_UNIT_MM;
        g->constraint_type = SANE_CONSTRAINT_RANGE;
        g->constraint.range = geo[i].range;
    }
    s->tl_x = s->tl_y = 0;
    s->br_x = SANE_FIX(MAX_WIDTH_MM);
    s->br_y = SANE_FIX(MAX_LENGTH_MM);
}

static int is_color(struct scanner *s) { return !strcmp(s->mode, "Color"); }

static int mm_to_px(SANE_Fixed mm, int dpi) { return (int)(SANE_UNFIX(mm) * dpi / MM_PER_INCH + 0.5); }

/* Pixel window within the sensor line; the sensor spans the full 8.5". */
static void window(struct scanner *s, int sensor_px, int *x0, int *x1)
{
    *x0 = mm_to_px(s->tl_x, s->resolution);
    *x1 = mm_to_px(s->br_x, s->resolution);
    if (*x1 > sensor_px)
        *x1 = sensor_px;
    if (*x0 > *x1 - 1)
        *x0 = *x1 - 1;
    if (*x0 < 0)
        *x0 = 0;
}

static void estimate_params(struct scanner *s)
{
    int sensor_px = s->resolution * 85 / 10 & ~3, x0, x1;
    window(s, sensor_px, &x0, &x1);
    s->params.format = is_color(s) ? SANE_FRAME_RGB : SANE_FRAME_GRAY;
    s->params.last_frame = SANE_TRUE;
    s->params.depth = 8;
    s->params.pixels_per_line = x1 - x0;
    s->params.bytes_per_line = s->params.pixels_per_line * (is_color(s) ? 3 : 1);
    s->params.lines = mm_to_px(s->br_y - s->tl_y, s->resolution);
}

/* ------------------------------------------------------------------------ */
/* SANE API                                                                  */

EXPORT SANE_Status sane_neat_init(SANE_Int *version_code, SANE_Auth_Callback authorize)
{
    const char *lvl = getenv("SANE_DEBUG_NEAT");
    (void)authorize;
    debug_level = lvl ? atoi(lvl) : 0;
    nm1000_set_log(core_log);
    DBG(1, "sane_init: neat backend build %d\n", BACKEND_BUILD);
    if (version_code)
        *version_code = SANE_VERSION_CODE(SANE_CURRENT_MAJOR, 0, BACKEND_BUILD);
    return SANE_STATUS_GOOD;
}

EXPORT void sane_neat_exit(void)
{
    for (struct scanner *s = scanners; s; s = s->next)
        if (s->dev) {
            nm1000_close(s->dev);
            s->dev = NULL;
        }
    free_scanners();
}

EXPORT SANE_Status sane_neat_get_devices(const SANE_Device ***list, SANE_Bool local_only)
{
    int n = 0, i = 0;
    (void)local_only;

    /* Keep entries for open devices; re-probe only when none are open. */
    int any_open = 0;
    for (struct scanner *s = scanners; s; s = s->next)
        any_open |= s->dev != NULL;
    if (!any_open)
        probe_devices();

    for (struct scanner *s = scanners; s; s = s->next)
        n++;
    free(device_list);
    device_list = calloc(n + 1, sizeof(*device_list));
    if (!device_list)
        return SANE_STATUS_NO_MEM;
    for (struct scanner *s = scanners; s; s = s->next)
        device_list[i++] = &s->sane;
    *list = device_list;
    return SANE_STATUS_GOOD;
}

EXPORT SANE_Status sane_neat_open(SANE_String_Const name, SANE_Handle *handle)
{
    struct scanner *s;
    int r;

    if (!scanners)
        probe_devices();
    for (s = scanners; s; s = s->next)
        if (!name || !*name || !strcmp(name, s->name))
            break;
    if (!s)
        return SANE_STATUS_INVAL;
    if (s->dev)
        return SANE_STATUS_DEVICE_BUSY;

    DBG(2, "sane_open: %s\n", s->name);
    r = nm1000_open(&s->dev, s->bus, s->address);
    if (r) {
        DBG(1, "sane_open: %s\n", nm1000_strerror(r));
        s->dev = NULL;
        return map_status(r);
    }
    init_options(s);
    *handle = s;
    return SANE_STATUS_GOOD;
}

EXPORT void sane_neat_close(SANE_Handle h)
{
    struct scanner *s = h;
    if (s->dev) {
        nm1000_close(s->dev);
        s->dev = NULL;
    }
    free(s->page);
    s->page = NULL;
}

EXPORT const SANE_Option_Descriptor *sane_neat_get_option_descriptor(SANE_Handle h, SANE_Int n)
{
    struct scanner *s = h;
    if (n < 0 || n >= NUM_OPTIONS)
        return NULL;
    return &s->opt[n];
}

EXPORT SANE_Status sane_neat_control_option(SANE_Handle h, SANE_Int n, SANE_Action action, void *val,
                                             SANE_Int *info)
{
    struct scanner *s = h;
    SANE_Word *w = val;

    if (info)
        *info = 0;
    if (n < 0 || n >= NUM_OPTIONS || s->opt[n].type == SANE_TYPE_GROUP)
        return SANE_STATUS_INVAL;
    if (s->scanning)
        return SANE_STATUS_DEVICE_BUSY;

    if (action == SANE_ACTION_GET_VALUE) {
        switch (n) {
        case OPT_NUM_OPTS: *w = NUM_OPTIONS; break;
        case OPT_MODE: strcpy(val, s->mode); break;
        case OPT_RESOLUTION: *w = s->resolution; break;
        case OPT_SOURCE: strcpy(val, "ADF"); break;
        case OPT_TL_X: *w = s->tl_x; break;
        case OPT_TL_Y: *w = s->tl_y; break;
        case OPT_BR_X: *w = s->br_x; break;
        case OPT_BR_Y: *w = s->br_y; break;
        }
        return SANE_STATUS_GOOD;
    }

    if (action == SANE_ACTION_SET_AUTO)
        return SANE_STATUS_UNSUPPORTED;
    if (action != SANE_ACTION_SET_VALUE || n == OPT_NUM_OPTS)
        return SANE_STATUS_INVAL;

    switch (n) {
    case OPT_MODE:
        for (int i = 0; mode_list[i]; i++)
            if (!strcmp(val, mode_list[i])) {
                snprintf(s->mode, sizeof(s->mode), "%s", mode_list[i]);
                if (info)
                    *info |= SANE_INFO_RELOAD_PARAMS;
                return SANE_STATUS_GOOD;
            }
        return SANE_STATUS_INVAL;
    case OPT_SOURCE:
        return strcmp(val, "ADF") ? SANE_STATUS_INVAL : SANE_STATUS_GOOD;
    case OPT_RESOLUTION: {
        /* snap to the nearest supported resolution */
        SANE_Word best = resolution_list[1];
        for (int i = 1; i <= resolution_list[0]; i++)
            if (abs(resolution_list[i] - *w) < abs(best - *w))
                best = resolution_list[i];
        if (best != *w && info)
            *info |= SANE_INFO_INEXACT;
        s->resolution = best;
        *w = best;
        if (info)
            *info |= SANE_INFO_RELOAD_PARAMS;
        return SANE_STATUS_GOOD;
    }
    case OPT_TL_X: case OPT_TL_Y: case OPT_BR_X: case OPT_BR_Y: {
        const SANE_Range *r = s->opt[n].constraint.range;
        SANE_Fixed v = *w < r->min ? r->min : *w > r->max ? r->max : *w;
        if (v != *w && info)
            *info |= SANE_INFO_INEXACT;
        *w = v;
        if (n == OPT_TL_X) s->tl_x = v;
        if (n == OPT_TL_Y) s->tl_y = v;
        if (n == OPT_BR_X) s->br_x = v;
        if (n == OPT_BR_Y) s->br_y = v;
        if (info)
            *info |= SANE_INFO_RELOAD_PARAMS;
        return SANE_STATUS_GOOD;
    }
    }
    return SANE_STATUS_INVAL;
}

EXPORT SANE_Status sane_neat_get_parameters(SANE_Handle h, SANE_Parameters *p)
{
    struct scanner *s = h;
    if (!s->scanning)
        estimate_params(s);
    *p = s->params;
    return SANE_STATUS_GOOD;
}

EXPORT SANE_Status sane_neat_start(SANE_Handle h)
{
    struct scanner *s = h;
    struct nm1000_scan_info info = {0};
    uint8_t *raw = NULL;
    size_t cap = 0, used = 0;
    int r, lines = 0, x0, x1, y0, y1, ch;

    if (!s->dev)
        return SANE_STATUS_INVAL;
    if (s->br_x <= s->tl_x || s->br_y <= s->tl_y)
        return SANE_STATUS_INVAL;
    free(s->page);
    s->page = NULL;
    s->scanning = 0;
    s->cancelled = 0;

    ch = is_color(s) ? 3 : 1;
    y1 = mm_to_px(s->br_y, s->resolution);
    DBG(2, "sane_start: %s %d dpi\n", s->mode, s->resolution);
    r = nm1000_start(s->dev, s->resolution, is_color(s), (int)(SANE_UNFIX(s->br_y) + 1), &info);
    if (r) {
        DBG(1, "sane_start: %s\n", nm1000_strerror(r));
        return map_status(r);
    }

    /* Scan the whole sheet (it ends at the trailing edge or at br-y). */
    for (;;) {
        int n;
        if (s->cancelled) {
            r = NM1000_ERR_CANCELLED;
            break;
        }
        if (cap - used < (size_t)info.bytes_per_line * 64) {
            size_t ncap = cap ? cap * 2 : (size_t)info.bytes_per_line * 1024;
            uint8_t *nraw = realloc(raw, ncap);
            if (!nraw) {
                r = NM1000_ERR_NOMEM;
                break;
            }
            raw = nraw;
            cap = ncap;
        }
        r = nm1000_read(s->dev, raw + used, 64, &n);
        used += (size_t)n * info.bytes_per_line;
        lines += n;
        if (r || lines >= y1)
            break;
    }
    int fr = nm1000_finish(s->dev);
    if (r == NM1000_EOF || (r == NM1000_OK && lines >= y1))
        r = fr;
    if (r) {
        DBG(1, "sane_start: scan failed: %s\n", nm1000_strerror(r));
        free(raw);
        return map_status(r);
    }

    /* Crop to the requested window. */
    window(s, info.pixels, &x0, &x1);
    y0 = mm_to_px(s->tl_y, s->resolution);
    if (y1 > lines)
        y1 = lines;
    if (y0 > y1)
        y0 = y1;
    s->params.format = ch == 3 ? SANE_FRAME_RGB : SANE_FRAME_GRAY;
    s->params.last_frame = SANE_TRUE;
    s->params.depth = 8;
    s->params.pixels_per_line = x1 - x0;
    s->params.bytes_per_line = (x1 - x0) * ch;
    s->params.lines = y1 - y0;
    s->page_len = (size_t)s->params.bytes_per_line * s->params.lines;
    s->page = malloc(s->page_len ? s->page_len : 1);
    if (!s->page) {
        free(raw);
        return SANE_STATUS_NO_MEM;
    }
    for (int y = y0; y < y1; y++)
        memcpy(s->page + (size_t)(y - y0) * s->params.bytes_per_line,
               raw + (size_t)y * info.bytes_per_line + (size_t)x0 * ch, s->params.bytes_per_line);
    free(raw);
    s->page_pos = 0;
    s->scanning = 1;
    DBG(2, "sane_start: page %dx%d\n", s->params.pixels_per_line, s->params.lines);
    return SANE_STATUS_GOOD;
}

EXPORT SANE_Status sane_neat_read(SANE_Handle h, SANE_Byte *buf, SANE_Int max, SANE_Int *len)
{
    struct scanner *s = h;
    size_t n;

    *len = 0;
    if (!s->scanning)
        return s->cancelled ? SANE_STATUS_CANCELLED : SANE_STATUS_EOF;
    if (s->cancelled) {
        s->scanning = 0;
        return SANE_STATUS_CANCELLED;
    }
    if (s->page_pos >= s->page_len) {
        s->scanning = 0;
        return SANE_STATUS_EOF;
    }
    n = s->page_len - s->page_pos;
    if (n > (size_t)max)
        n = max;
    memcpy(buf, s->page + s->page_pos, n);
    s->page_pos += n;
    *len = n;
    return SANE_STATUS_GOOD;
}

EXPORT void sane_neat_cancel(SANE_Handle h)
{
    struct scanner *s = h;
    s->cancelled = 1;
    s->scanning = 0;
}

EXPORT SANE_Status sane_neat_set_io_mode(SANE_Handle h, SANE_Bool non_blocking)
{
    (void)h;
    return non_blocking ? SANE_STATUS_UNSUPPORTED : SANE_STATUS_GOOD;
}

EXPORT SANE_Status sane_neat_get_select_fd(SANE_Handle h, SANE_Int *fd)
{
    (void)h, (void)fd;
    return SANE_STATUS_UNSUPPORTED;
}

/* Plain sane_* names, for frontends that load this library directly. */
#define ALIAS(n) __attribute__((alias("sane_neat_" #n), visibility("default")))
SANE_Status sane_init(SANE_Int *, SANE_Auth_Callback) ALIAS(init);
void sane_exit(void) ALIAS(exit);
SANE_Status sane_get_devices(const SANE_Device ***, SANE_Bool) ALIAS(get_devices);
SANE_Status sane_open(SANE_String_Const, SANE_Handle *) ALIAS(open);
void sane_close(SANE_Handle) ALIAS(close);
const SANE_Option_Descriptor *sane_get_option_descriptor(SANE_Handle, SANE_Int) ALIAS(get_option_descriptor);
SANE_Status sane_control_option(SANE_Handle, SANE_Int, SANE_Action, void *, SANE_Int *) ALIAS(control_option);
SANE_Status sane_get_parameters(SANE_Handle, SANE_Parameters *) ALIAS(get_parameters);
SANE_Status sane_start(SANE_Handle) ALIAS(start);
SANE_Status sane_read(SANE_Handle, SANE_Byte *, SANE_Int, SANE_Int *) ALIAS(read);
void sane_cancel(SANE_Handle) ALIAS(cancel);
SANE_Status sane_set_io_mode(SANE_Handle, SANE_Bool) ALIAS(set_io_mode);
SANE_Status sane_get_select_fd(SANE_Handle, SANE_Int *) ALIAS(get_select_fd);

/*
 * Neat NM-1000 mobile sheetfed scanner — device driver core (see nm1000.h).
 *
 * USB protocol (GL847-family vendor requests, all on EP0 unless noted):
 *   register write  40 04/0c wValue=0x83  data = (reg, val) pairs
 *   register read   c0 04    wValue=0x8e  wIndex = 0x22 | reg << 8,
 *                                         returns n values + 0x55 status
 *   memory timing   40 0c    wValue=0x8c  wIndex = index, 1 byte
 *   bulk setup      40 04    wValue=0x82  wIndex = 0 (in) / 1 (out),
 *                                         data = addr32 LE, len32 LE
 *   then the data moves on bulk EP 0x81 (in) / 0x02 (out).
 * Bulk address space: 0x10000000 scan RAM (image FIFO, shading, motor
 * tables), 0x01000000 gamma tables, 0x03000000 SPI flash controller.
 * AFE registers are written through ASIC regs 0x51 (addr), 0x3a/0x3b (data).
 */
#define _GNU_SOURCE
#include "nm1000.h"

#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "nm1000_tables.h"

#define EP_IN 0x81
#define EP_OUT 0x02
#define CTRL_TIMEOUT 5000
#define BULK_TIMEOUT 30000
#define BULK_MAX 0xeff0

#define REQ_REGISTER 0x0c
#define REQ_BUFFER 0x04
#define VAL_BUFFER 0x82
#define VAL_SET_REGISTER 0x83
#define VAL_TIMING 0x8c
#define VAL_GET_REGISTER 0x8e

#define FLASH_CTRL 0x03000000u
#define SCAN_FIFO 0x10000000u

struct nm1000 {
    libusb_context *ctx;
    libusb_device_handle *h;
    char serial[32];

    /* calibration tables per flash region, loaded lazily */
    uint16_t *flash[4];
    size_t flash_words[4];

    /* current scan */
    const struct nm1000_mode *mode;
    int scanning;
    int bpl;
    int lines_max;       /* lines programmed into LINCNT */
    int lines_done;      /* lines delivered to the caller */
    int lines_total;     /* known once the trailing edge was seen, else -1 */
    uint8_t *chunk;      /* line-planar data from the FIFO */
    int chunk_lines;
    int data_seen;       /* the scan engine has produced data */
};

static nm1000_log_fn log_fn;

/* NM1000_TRACE=file logs all USB traffic in the same format as the
 * re/pe-harness tracer, so native and vendor sequences can be diffed. */
static FILE *trace_fp;
static int trace_init_done;

static FILE *trace_file(void)
{
    if (!trace_init_done) {
        const char *p = getenv("NM1000_TRACE");
        trace_init_done = 1;
        if (p && *p)
            trace_fp = fopen(p, "w");
    }
    return trace_fp;
}

static void trace_xfer(const char *kind, const char *head, const uint8_t *data, int len, int max)
{
    FILE *f = trace_file();
    if (!f)
        return;
    fprintf(f, "%s %s", kind, head);
    for (int i = 0; i < len && i < max; i++)
        fprintf(f, "%02x", data[i]);
    if (len > max)
        fprintf(f, "...(+%d)", len - max);
    fputc('\n', f);
    fflush(f);
}

void nm1000_set_log(nm1000_log_fn fn) { log_fn = fn; }

static void dbg(int level, const char *fmt, ...)
{
    char msg[512];
    va_list ap;
    if (!log_fn)
        return;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    log_fn(level, msg);
}

const char *nm1000_strerror(int s)
{
    switch (s) {
    case NM1000_OK: return "ok";
    case NM1000_EOF: return "end of page";
    case NM1000_ERR_IO: return "USB I/O error";
    case NM1000_ERR_NODEV: return "scanner not found";
    case NM1000_ERR_NOPAPER: return "no paper in the feeder";
    case NM1000_ERR_INVAL: return "invalid argument";
    case NM1000_ERR_NOMEM: return "out of memory";
    case NM1000_ERR_BUSY: return "device busy";
    case NM1000_ERR_CANCELLED: return "cancelled";
    }
    return "unknown error";
}

static void msleep(int ms)
{
    if (trace_file()) {
        fprintf(trace_fp, "Sleep %d\n", ms);
        fflush(trace_fp);
    }
    struct timespec ts = {ms / 1000, (ms % 1000) * 1000000L};
    while (nanosleep(&ts, &ts) && errno == EINTR)
        ;
}

static int trace_phase(const char *name, int cmd)
{
    if (trace_file()) {
        fprintf(trace_fp, "### SNCmd(0x%x %s p5=0 p6=0)\n", cmd, name);
        fflush(trace_fp);
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* USB primitives                                                            */

static int ctrl_out(struct nm1000 *d, uint16_t value, uint16_t index, const uint8_t *data, uint16_t len)
{
    uint8_t req = len > 1 ? REQ_BUFFER : REQ_REGISTER;
    char head[64];
    snprintf(head, sizeof(head), "%02x %04x %04x %u ", req, value, index, len);
    trace_xfer("CO", head, data, len, 1 << 20);
    int r = libusb_control_transfer(d->h, 0x40, req, value, index, (uint8_t *)data, len, CTRL_TIMEOUT);
    if (r != len) {
        dbg(1, "control out %04x/%04x len %u failed: %s", value, index, len,
            r < 0 ? libusb_error_name(r) : "short");
        return NM1000_ERR_IO;
    }
    return NM1000_OK;
}

/* Write (reg, val) pairs, at most 32 pairs per transfer like the vendor driver. */
static int write_pairs(struct nm1000 *d, const uint8_t *pairs, int n)
{
    while (n > 0) {
        int k = n > 32 ? 32 : n;
        int r = ctrl_out(d, VAL_SET_REGISTER, 0, pairs, k * 2);
        if (r)
            return r;
        pairs += k * 2;
        n -= k;
    }
    return NM1000_OK;
}

static int write_reg(struct nm1000 *d, uint8_t reg, uint8_t val)
{
    uint8_t p[2] = {reg, val};
    return write_pairs(d, p, 1);
}

static int read_regs(struct nm1000 *d, uint8_t reg, uint8_t *out, int n)
{
    uint8_t buf[65];
    if (n < 1 || n > 63)
        return NM1000_ERR_INVAL;
    int r = libusb_control_transfer(d->h, 0xc0, REQ_BUFFER, VAL_GET_REGISTER, 0x22 | (reg << 8), buf, n + 1,
                                    CTRL_TIMEOUT);
    char head[64];
    snprintf(head, sizeof(head), "04 008e %04x %d -> ", 0x22 | (reg << 8), n + 1);
    trace_xfer("CI", head, buf, r > 0 ? r : 0, 4096);
    if (r != n + 1 || buf[n] != 0x55) {
        dbg(1, "register read 0x%02x x%d failed: %s", reg, n, r < 0 ? libusb_error_name(r) : "bad status");
        return NM1000_ERR_IO;
    }
    memcpy(out, buf, n);
    return NM1000_OK;
}

static int read_reg(struct nm1000 *d, uint8_t reg, uint8_t *val)
{
    return read_regs(d, reg, val, 1);
}

static int write_timing(struct nm1000 *d, uint8_t index, uint8_t val)
{
    return ctrl_out(d, VAL_TIMING, index, &val, 1);
}

static int write_afe(struct nm1000 *d, uint8_t reg, uint8_t val)
{
    uint8_t p[6] = {0x51, reg, 0x3a, 0x00, 0x3b, val};
    return write_pairs(d, p, 3);
}

static int bulk_header(struct nm1000 *d, int out, uint32_t addr, uint32_t len)
{
    uint8_t h[8];
    for (int i = 0; i < 4; i++) {
        h[i] = addr >> (8 * i);
        h[4 + i] = len >> (8 * i);
    }
    return ctrl_out(d, VAL_BUFFER, out ? 1 : 0, h, 8);
}

static int bulk_write(struct nm1000 *d, uint32_t addr, const uint8_t *data, uint32_t len)
{
    int n = 0, r;
    if ((r = bulk_header(d, 1, addr, len)))
        return r;
    char head[32];
    snprintf(head, sizeof(head), "%u ", len);
    trace_xfer("BO", head, data, len, 1 << 20);
    r = libusb_bulk_transfer(d->h, EP_OUT, (uint8_t *)data, len, &n, BULK_TIMEOUT);
    if (r || (uint32_t)n != len) {
        dbg(1, "bulk write %u bytes to %08x failed: %s", len, addr, libusb_error_name(r));
        return NM1000_ERR_IO;
    }
    return NM1000_OK;
}

static int bulk_read(struct nm1000 *d, uint32_t addr, uint8_t *data, uint32_t len)
{
    int n = 0, r;
    if ((r = bulk_header(d, 0, addr, len)))
        return r;
    r = libusb_bulk_transfer(d->h, EP_IN, data, len, &n, BULK_TIMEOUT);
    char head[32];
    snprintf(head, sizeof(head), "%u -> %d ", len, n);
    trace_xfer("BI", head, data, n, 64);
    if (r || (uint32_t)n != len) {
        dbg(1, "bulk read %u bytes from %08x failed: %s (got %d)", len, addr, libusb_error_name(r), n);
        return NM1000_ERR_IO;
    }
    return NM1000_OK;
}

/* ------------------------------------------------------------------------ */
/* Register programs (nm1000_tables.h)                                       */

struct prog_ctx {
    int lincnt; /* value for OP_LINCNT */
};

static int afe_calibrate(struct nm1000 *d);
static int upload_shading(struct nm1000 *d, int ch, uint32_t addr, uint32_t len, uint32_t pad, int hdr_first);

/* Poll until the motor stops, nudging reg 0x02 like the vendor driver. */
static int wait_motor(struct nm1000 *d)
{
    uint8_t st[3];
    int r;
    for (int i = 0; i < 3000; i++) {
        if ((r = read_regs(d, 0x40, st, 3)))
            return r;
        if (!(st[1] & 0x01))
            return NM1000_OK;
        if ((r = write_reg(d, 0x02, 0x58)))
            return r;
        msleep(5);
    }
    dbg(1, "timeout waiting for the motor to stop");
    return NM1000_ERR_IO;
}

static int run_prog(struct nm1000 *d, const uint8_t *p, const struct prog_ctx *ctx)
{
    int r = 0;
    for (;;) {
        switch (*p++) {
        case OP_END:
            return NM1000_OK;
        case OP_W: {
            int n = *p++;
            r = write_pairs(d, p, n);
            p += 2 * n;
            break;
        }
        case OP_SLEEP:
            msleep(p[0] | p[1] << 8);
            p += 2;
            break;
        case OP_8C:
            r = write_timing(d, p[0], p[1]);
            p += 2;
            break;
        case OP_BULK: {
            uint32_t addr = p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
            uint16_t len = p[4] | p[5] << 8;
            r = bulk_write(d, addr, p + 6, len);
            p += 6 + len;
            break;
        }
        case OP_AFECAL:
            r = afe_calibrate(d);
            break;
        case OP_SHADING: {
            int ch = p[0];
            uint32_t addr = p[1] | p[2] << 8 | p[3] << 16 | (uint32_t)p[4] << 24;
            uint32_t len = p[5] | p[6] << 8 | p[7] << 16 | (uint32_t)p[8] << 24;
            uint32_t pad = p[9] | p[10] << 8 | p[11] << 16 | (uint32_t)p[12] << 24;
            r = upload_shading(d, ch, addr, len, pad, p[13]);
            p += 14;
            break;
        }
        case OP_WAITMOTOR:
            r = wait_motor(d);
            break;
        case OP_LINCNT: {
            uint8_t w[6] = {0x25, ctx->lincnt >> 16, 0x26, ctx->lincnt >> 8, 0x27, ctx->lincnt};
            r = write_pairs(d, w, 3);
            break;
        }
        default:
            dbg(1, "bad program opcode %d", p[-1]);
            return NM1000_ERR_INVAL;
        }
        if (r)
            return r;
    }
}

/* ------------------------------------------------------------------------ */
/* Calibration from SPI flash                                                */

/* Each flash region holds one mode's shading table per channel (1 for gray,
 * 3 for colour): [AFE offset, AFE gain, (dark, gain) per pixel ...]. */
static size_t region_channel_words(int region)
{
    return (region == 0 || region == 2) ? 10802 : 5402;
}

static int region_channels(int region)
{
    return (region == 0 || region == 1) ? 3 : 1;
}

static int flash_ctrl_write(struct nm1000 *d, uint8_t reg, const uint8_t *data, int len)
{
    return bulk_write(d, FLASH_CTRL | reg, data, len);
}

static int flash_ctrl_read(struct nm1000 *d, uint8_t reg, uint8_t *data, int len)
{
    return bulk_read(d, FLASH_CTRL | reg, data, len);
}

static int flash_read(struct nm1000 *d, uint32_t addr, uint16_t *words, size_t nwords)
{
    static const uint8_t init[][3] = {
        {0x00, 0x04, 0x00}, {0x02, 0x01, 0x00}, {0x08, 0x02, 0x08},
        {0x0a, 0x9f, 0x9f}, {0x0c, 0x9f, 0x9f}, {0x04, 0x10, 0x00},
    };
    uint8_t buf[256];
    int r;

    /* GPIOs 0xa6/0xa7 route the SPI flash to the ASIC's controller. */
    if ((r = write_reg(d, 0xa7, 0x01)) || (r = write_reg(d, 0xa6, 0x1d)))
        return r;
    for (size_t i = 0; i < sizeof(init) / sizeof(init[0]); i++)
        if ((r = flash_ctrl_write(d, init[i][0], init[i] + 1, 2)))
            return r;
    if ((r = flash_ctrl_read(d, 0x06, buf, 2)))
        return r;
    if ((r = flash_ctrl_write(d, 0x06, (const uint8_t[]){0x01, 0x00}, 2)))
        return r;
    for (int i = 0; i < 3; i++) /* JEDEC id; kept for parity with the vendor sequence */
        if ((r = flash_ctrl_read(d, 0x0e, buf, 2)))
            return r;

    /* 32 bytes per SPI READ (0x03); the controller returns each 16-bit word
     * in a 16-byte record. */
    for (size_t w = 0; w < nwords; w += 16) {
        uint32_t a = addr + w * 2;
        uint8_t cmd[8] = {0x01, 0x00, 0x0f, 0x20, a, a >> 8, a >> 16, 0x03};
        if ((r = flash_ctrl_write(d, 0x06, cmd, 8)) ||
            (r = flash_ctrl_write(d, 0x04, (const uint8_t[]){0x10, 0x08}, 2)) ||
            (r = flash_ctrl_read(d, 0x06, buf, 2)) ||
            (r = flash_ctrl_read(d, 0x0e, buf, 256)))
            return r;
        for (int k = 0; k < 16 && w + k < nwords; k++)
            words[w + k] = buf[k * 16] | buf[k * 16 + 1] << 8;
    }
    r = flash_ctrl_write(d, 0x06, (const uint8_t[]){0x01, 0x00}, 2);
    msleep(10);
    if (!r && !(r = write_reg(d, 0xa7, 0x00)))
        r = write_reg(d, 0xa6, 0x1c);
    return r;
}

static void cache_path(struct nm1000 *d, int region, char *out, size_t n)
{
    const char *base = getenv("XDG_CACHE_HOME");
    const char *home = getenv("HOME");
    if (base && *base)
        snprintf(out, n, "%s/sane-neat/%s-region%d.cal", base, d->serial, region);
    else if (home && *home)
        snprintf(out, n, "%s/.cache/sane-neat/%s-region%d.cal", home, d->serial, region);
    else
        out[0] = 0;
}

static void mkdir_parent(const char *path)
{
    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%s", path);
    for (char *q = tmp + 1; *q; q++)
        if (*q == '/') {
            *q = 0;
            mkdir(tmp, 0700);
            *q = '/';
        }
}

static int load_calibration(struct nm1000 *d, int region)
{
    char path[1024];
    size_t words = region_channel_words(region) * region_channels(region);
    uint16_t *t;
    FILE *f;
    int r;

    if (d->flash[region])
        return NM1000_OK;
    t = malloc(words * 2);
    if (!t)
        return NM1000_ERR_NOMEM;

    cache_path(d, region, path, sizeof(path));
    if (path[0] && (f = fopen(path, "rb"))) {
        size_t got = fread(t, 2, words, f);
        fclose(f);
        if (got == words) {
            dbg(3, "calibration region %d from cache %s", region, path);
            d->flash[region] = t;
            d->flash_words[region] = words;
            return NM1000_OK;
        }
    }

    dbg(2, "reading calibration region %d from scanner flash", region);
    if ((r = flash_read(d, (uint32_t)region << 16, t, words))) {
        free(t);
        return r;
    }
    size_t nonzero = 0;
    for (size_t i = 0; i < words; i++)
        nonzero += t[i] != 0;
    if (nonzero < words / 4) {
        dbg(1, "calibration region %d in flash looks empty; is the scanner calibrated?", region);
        free(t);
        return NM1000_ERR_IO;
    }
    d->flash[region] = t;
    d->flash_words[region] = words;
    if (path[0]) {
        mkdir_parent(path);
        if ((f = fopen(path, "wb"))) {
            fwrite(t, 2, words, f);
            fclose(f);
        }
    }
    return NM1000_OK;
}

static const uint16_t *cal_channel(struct nm1000 *d, int ch)
{
    int region = d->mode->flash_region;
    if (region_channels(region) == 1)
        ch = 0;
    return d->flash[region] + ch * region_channel_words(region);
}

/* AFE offset (0x20-0x22) and gain (0x28-0x2a) from each table's header,
 * sent as one transfer like the vendor driver. */
static int afe_calibrate(struct nm1000 *d)
{
    uint8_t p[36], *q = p;
    for (int ch = 0; ch < 3; ch++) {
        const uint16_t *c = cal_channel(d, ch);
        const uint8_t regs[2] = {0x20 + ch, 0x28 + ch}, vals[2] = {c[0], c[1]};
        for (int k = 0; k < 2; k++) {
            *q++ = 0x51, *q++ = regs[k];
            *q++ = 0x3a, *q++ = 0x00;
            *q++ = 0x3b, *q++ = vals[k];
        }
    }
    return write_pairs(d, p, 18);
}

static int upload_shading(struct nm1000 *d, int ch, uint32_t addr, uint32_t len, uint32_t pad, int hdr_first)
{
    size_t words = len / 2, cw = region_channel_words(d->mode->flash_region);
    const uint16_t *c = cal_channel(d, ch);
    uint8_t *buf = calloc(1, len);
    int r;

    if (!buf)
        return NM1000_ERR_NOMEM;
    if (hdr_first) {
        buf[0] = c[0];
        buf[1] = c[0] >> 8;
        buf[2] = c[1];
        buf[3] = c[1] >> 8;
    }
    buf[4] = pad;
    buf[5] = pad >> 8;
    buf[6] = pad >> 16;
    buf[7] = pad >> 24;
    for (size_t i = 4; i < words && i - 2 < cw; i++) {
        buf[2 * i] = c[i - 2];
        buf[2 * i + 1] = c[i - 2] >> 8;
    }
    r = bulk_write(d, addr, buf, len);
    free(buf);
    return r;
}

/* ------------------------------------------------------------------------ */
/* Device lifecycle                                                          */

/* Power-up sequence for a scanner that has just been plugged in (reg 0x41
 * bit 7 clear), from the vendor driver. */
static int cold_init(struct nm1000 *d)
{
    uint8_t v;
    int r, tries;

    dbg(2, "scanner is unpowered, running power-up sequence");
    if ((r = write_reg(d, 0x6e, 0x00)) || (r = write_reg(d, 0xa7, 0x00)) || (r = write_reg(d, 0x6f, 0x00)) ||
        (r = write_reg(d, 0x06, 0x50)))
        return r;
    msleep(100);
    if ((r = write_reg(d, 0x6c, 0x00)))
        return r;
    /* Switching this GPIO on can glitch the bus on bus-powered ports; the
     * write still takes effect, so tolerate one failure. */
    if (write_reg(d, 0x6e, 0x02)) {
        msleep(100);
        write_reg(d, 0x6e, 0x02);
    }
    if ((r = write_reg(d, 0x6c, 0x00)))
        return r;
    msleep(100);
    for (tries = 0; tries < 100; tries++) {
        if ((r = read_reg(d, 0x41, &v)))
            return r;
        if (!(v & 0x01))
            break;
        msleep(100);
    }
    msleep(100);
    if ((r = read_reg(d, 0x0b, &v)) || (r = write_reg(d, 0x0b, v & 0x1f)) || (r = write_timing(d, 0x10, 0x0a)) ||
        (r = write_timing(d, 0x13, 0x0e)) || (r = write_reg(d, 0xa6, 0x0a)) || (r = write_reg(d, 0xa7, 0x0a)) ||
        (r = write_reg(d, 0xa6, 0x0a)))
        return r;
    msleep(100);
    if ((r = write_afe(d, 0x04, 0x00)))
        return r;
    msleep(100);
    return write_reg(d, 0x0b, 0x01);
}

static int device_init(struct nm1000 *d)
{
    uint8_t v;
    int r;
    struct prog_ctx ctx = {0};

    if ((r = read_reg(d, 0x41, &v)))
        return r;
    if (!(v & 0x80) && (r = cold_init(d)))
        return r;
    return run_prog(d, prog_common_init, &ctx);
}

static int match_device(libusb_device *dev, int bus, int address)
{
    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev, &desc))
        return 0;
    if (desc.idVendor != NM1000_VENDOR || desc.idProduct != NM1000_PRODUCT)
        return 0;
    if (bus >= 0 && libusb_get_bus_number(dev) != bus)
        return 0;
    if (address >= 0 && libusb_get_device_address(dev) != address)
        return 0;
    return 1;
}

int nm1000_open(struct nm1000 **out, int bus, int address)
{
    struct nm1000 *d = calloc(1, sizeof(*d));
    libusb_device **list = NULL;
    libusb_device *found = NULL;
    struct libusb_device_descriptor desc;
    ssize_t n;
    int r;

    if (!d)
        return NM1000_ERR_NOMEM;
    if (libusb_init(&d->ctx)) {
        free(d);
        return NM1000_ERR_IO;
    }
    n = libusb_get_device_list(d->ctx, &list);
    for (ssize_t i = 0; i < n && !found; i++)
        if (match_device(list[i], bus, address))
            found = list[i];
    if (!found) {
        libusb_free_device_list(list, 1);
        libusb_exit(d->ctx);
        free(d);
        return NM1000_ERR_NODEV;
    }
    r = libusb_open(found, &d->h);
    libusb_get_device_descriptor(found, &desc);
    libusb_free_device_list(list, 1);
    if (r) {
        dbg(1, "cannot open scanner: %s", libusb_error_name(r));
        libusb_exit(d->ctx);
        free(d);
        return r == LIBUSB_ERROR_ACCESS ? NM1000_ERR_NODEV : NM1000_ERR_IO;
    }
    libusb_set_auto_detach_kernel_driver(d->h, 1);
    if ((r = libusb_claim_interface(d->h, 0))) {
        dbg(1, "cannot claim interface: %s", libusb_error_name(r));
        libusb_close(d->h);
        libusb_exit(d->ctx);
        free(d);
        return r == LIBUSB_ERROR_BUSY ? NM1000_ERR_BUSY : NM1000_ERR_IO;
    }
    if (desc.iSerialNumber <= 0 ||
        libusb_get_string_descriptor_ascii(d->h, desc.iSerialNumber, (uint8_t *)d->serial, sizeof(d->serial)) <= 0)
        snprintf(d->serial, sizeof(d->serial), "unknown");

    if ((r = device_init(d))) {
        nm1000_close(d);
        return r;
    }
    *out = d;
    return NM1000_OK;
}

void nm1000_close(struct nm1000 *d)
{
    static const uint8_t park[] = {0x03, 0x0f, 0x03, 0x8f, 0x6f, 0x00, 0x6d, 0x3f, 0x6b, 0x00};
    if (!d)
        return;
    if (d->scanning)
        nm1000_finish(d);
    write_pairs(d, park, sizeof(park) / 2);
    for (int i = 0; i < 4; i++)
        free(d->flash[i]);
    free(d->chunk);
    libusb_release_interface(d->h, 0);
    libusb_close(d->h);
    libusb_exit(d->ctx);
    free(d);
}

const char *nm1000_serial(struct nm1000 *d) { return d->serial; }

const int *nm1000_resolutions(void)
{
    static const int res[] = {150, 200, 300, 600, 0};
    return res;
}

int nm1000_paper_present(struct nm1000 *d)
{
    uint8_t v;
    int r;
    if ((r = write_reg(d, 0x0a, 0x20)) || (r = read_reg(d, 0x40, &v)))
        return r;
    return !(v & 0x40);
}

/* ------------------------------------------------------------------------ */
/* Scanning                                                                  */

static const struct nm1000_mode *find_mode(int dpi, int color)
{
    for (size_t i = 0; i < sizeof(nm1000_modes) / sizeof(nm1000_modes[0]); i++)
        if (nm1000_modes[i].dpi == dpi && nm1000_modes[i].color == !!color)
            return &nm1000_modes[i];
    return NULL;
}

int nm1000_prepare(struct nm1000 *d, int dpi, int color)
{
    const struct nm1000_mode *m = find_mode(dpi, color);
    if (!m)
        return NM1000_ERR_INVAL;
    return load_calibration(d, m->flash_region);
}

int nm1000_start(struct nm1000 *d, int dpi, int color, int max_height_mm, struct nm1000_scan_info *info)
{
    static const uint8_t lamp_off[] = {0x6e, 0x6f, 0x7e, 0x00};
    const struct nm1000_mode *m = find_mode(dpi, color);
    struct prog_ctx ctx;
    int r;

    if (!m || max_height_mm <= 0)
        return NM1000_ERR_INVAL;
    if (d->scanning)
        return NM1000_ERR_BUSY;

    r = nm1000_paper_present(d);
    if (r < 0)
        return r;
    if (!r && !getenv("NM1000_DEV_IGNORE_PAPER"))
        return NM1000_ERR_NOPAPER;

    d->mode = m;
    if ((r = load_calibration(d, m->flash_region)))
        return r;

    d->bpl = m->pixels * (m->color ? 3 : 1);
    d->lines_max = (int)((long)max_height_mm * dpi * 10 / 254);
    d->lines_done = 0;
    d->lines_total = -1;
    d->data_seen = 0;
    d->chunk_lines = BULK_MAX / d->bpl;
    free(d->chunk);
    d->chunk = malloc((size_t)d->chunk_lines * d->bpl);
    if (!d->chunk)
        return NM1000_ERR_NOMEM;

    /* The line counter runs per colour line in colour mode. */
    ctx.lincnt = d->lines_max * (m->color ? 3 : 1);
    if (ctx.lincnt > 0xffffff)
        return NM1000_ERR_INVAL;

    dbg(2, "start: %d dpi %s, %d px, up to %d lines", dpi, m->color ? "colour" : "gray", m->pixels,
        d->lines_max);
    if ((r = trace_phase("set-params", 2), run_prog(d, m->setparams, &ctx)) ||
        (r = trace_phase("load-calib", 4), run_prog(d, m->calib, &ctx)) ||
        (r = trace_phase("lamp", 9), run_prog(d, prog_lamp_on, &ctx)))
        return r;
    d->scanning = 1;
    trace_phase("start", 5);
    if ((r = run_prog(d, m->start, &ctx))) {
        write_pairs(d, lamp_off, 2);
        d->scanning = 0;
        return r;
    }
    if (info) {
        info->dpi = dpi;
        info->color = m->color;
        info->pixels = m->pixels;
        info->bytes_per_line = d->bpl;
        info->max_lines = d->lines_max;
    }
    return NM1000_OK;
}

static uint32_t be24(const uint8_t *p) { return (uint32_t)p[0] << 16 | p[1] << 8 | p[2]; }

/* Once the trailing edge leaves the paper sensor (reg 0x40 bit 6), the
 * remaining line count is the sensor-to-scan-line distance (regs 0x92/0x93)
 * plus the hardware line count at the event (0x4b-0x4d), as the vendor
 * driver computes it. Both are in colour sub-lines in colour mode. */
static int check_trailing_edge(struct nm1000 *d)
{
    uint8_t s, ev[3], dist[2];
    int r, color = d->mode->color;
    int32_t event, distance, adj = 0;

    if (d->lines_total >= 0)
        return NM1000_OK;
    if ((r = read_reg(d, 0x40, &s)))
        return r;
    if (!(s & 0x40))
        return NM1000_OK;
    if ((r = read_regs(d, 0x4b, ev, 3)) || (r = read_regs(d, 0x92, dist, 2)))
        return r;
    event = be24(ev);
    distance = dist[0] << 8 | dist[1];
    if (event % 3 == 1)
        adj = 2;
    else if (event % 3 == 2)
        adj = 1;
    if (color)
        d->lines_total = d->lines_done + (distance / 3 - d->lines_done) + event / 3 + adj;
    else
        d->lines_total = d->lines_done + (distance - d->lines_done) + event + adj;
    dbg(2, "trailing edge: event=%d distance=%d done=%d -> total %d lines", event, distance, d->lines_done,
        d->lines_total);
    if (d->lines_total > d->lines_max)
        d->lines_total = d->lines_max;
    return NM1000_OK;
}

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/* A bulk read the ASIC cannot satisfy wedges it until it is power-cycled
 * (even a USB reset does not recover it), so only read once the scan
 * engine reports data. This is the vendor driver's wait, applied before
 * every chunk rather than only the first. */
static int wait_for_data(struct nm1000 *d, int timeout_ms)
{
    long t0 = now_ms();
    uint8_t st[2], cnt[3];
    int r;

    for (;;) {
        if ((r = read_regs(d, 0x40, st, 2)))
            return r;
        if (!(st[1] & 0x40)) /* BUFEMPTY clear */
            break;
        if ((st[1] & 0x10) && d->data_seen) /* SCANFSH: engine done, buffer drained */
            return NM1000_EOF;
        if (now_ms() - t0 > timeout_ms)
            return d->data_seen ? NM1000_ERR_IO : NM1000_ERR_NOPAPER;
        msleep(10);
    }
    if (!d->data_seen) {
        do {
            if ((r = read_regs(d, 0x42, cnt, 3)))
                return r;
            if (cnt[0] | cnt[1] | cnt[2])
                break;
            if (now_ms() - t0 > timeout_ms)
                return NM1000_ERR_NOPAPER;
            msleep(10);
        } while (1);
        d->data_seen = 1;
    }
    return NM1000_OK;
}

int nm1000_read(struct nm1000 *d, uint8_t *buf, int max_lines, int *lines)
{
    int end, want, r, px;

    *lines = 0;
    if (!d->scanning)
        return NM1000_ERR_INVAL;
    end = d->lines_total >= 0 ? d->lines_total : d->lines_max;
    if (d->lines_done >= end)
        return NM1000_EOF;

    want = end - d->lines_done;
    if (want > max_lines)
        want = max_lines;
    if (want > d->chunk_lines)
        want = d->chunk_lines;
    if (want <= 0)
        return NM1000_ERR_INVAL;

    r = wait_for_data(d, d->data_seen ? 10000 : 20000);
    if (r == NM1000_EOF) {
        dbg(2, "scan engine finished after %d lines", d->lines_done);
        d->lines_total = d->lines_done;
        return NM1000_EOF;
    }
    if (r)
        return r;
    if ((r = bulk_read(d, SCAN_FIFO, d->chunk, (uint32_t)want * d->bpl)))
        return r;

    /* The FIFO delivers colour lines as R, G, B planes. */
    px = d->mode->pixels;
    for (int l = 0; l < want; l++) {
        const uint8_t *src = d->chunk + (size_t)l * d->bpl;
        uint8_t *dst = buf + (size_t)l * d->bpl;
        if (d->mode->color) {
            for (int x = 0; x < px; x++) {
                dst[3 * x] = src[x];
                dst[3 * x + 1] = src[px + x];
                dst[3 * x + 2] = src[2 * px + x];
            }
        } else {
            memcpy(dst, src, px);
        }
    }
    d->lines_done += want;
    *lines = want;
    return check_trailing_edge(d);
}

int nm1000_feed(struct nm1000 *d, int steps)
{
    static const uint8_t pre[] = {0x6f, 0x80, 0x6d, 0x9f, 0x6b, 0x81};
    static const uint8_t slope[] = {0xd6, 0x1e};
    int r;

    if (steps < 3 || steps > 0xffffff)
        return NM1000_ERR_INVAL;
    steps -= 2;
    uint8_t move[] = {0x02, 0x58, 0x6a, 0x01, 0x3d, steps >> 16, 0x3e, steps >> 8, 0x3f, steps};
    if ((r = write_pairs(d, pre, 3)))
        return r;
    msleep(20);
    if ((r = write_reg(d, 0x63, 0x00)) || (r = bulk_write(d, 0x1000c000, slope, 2)) ||
        (r = write_pairs(d, move, 5)) || (r = write_reg(d, 0x0f, 0x01)) || (r = write_reg(d, 0x6b, 0x01)))
        return r;
    return wait_motor(d);
}

int nm1000_finish(struct nm1000 *d)
{
    static const uint8_t lamp_off[] = {0x6e, 0x6f, 0x7e, 0x00};
    struct prog_ctx ctx = {0};
    int r;

    if (!d->scanning)
        return NM1000_OK;
    d->scanning = 0;
    r = run_prog(d, prog_stop, &ctx);
    /* Eject: keep feeding while the sensor still sees paper (at most ~40"). */
    for (int i = 0; !r && i < 12; i++) {
        int p = nm1000_paper_present(d);
        if (p <= 0) {
            r = p;
            break;
        }
        r = nm1000_feed(d, 1000);
    }
    write_pairs(d, lamp_off, 2);
    return r;
}

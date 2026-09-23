/*
 * neatcap: drive Neat's own driver DLL through its SNCmd() entry point, the
 * same way its TWAIN data source does, and record the USB traffic.
 *
 *   neatcap DLL status
 *   neatcap DLL scan RES BPP OUT.pnm [WIDTH_PX HEIGHT_PX]
 *   neatcap DLL calibrate
 *   neatcap DLL feed STEPS
 *   neatcap DLL raw CMD P5 P6
 *
 * Trace goes to $NEAT_TRACE (default trace.log).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pe_harness.h"

typedef int (*__attribute__((ms_abi)) sncmd_t)(uint32_t cmd, uint32_t *params, void *buf, float *fp,
                                                 int p5, uint32_t p6);
static sncmd_t SNCmd;

static int cmd(const char *what, uint32_t c, uint32_t *params, void *buf, float *fp, int p5, uint32_t p6)
{
    trace("### SNCmd(0x%x %s p5=%d p6=%u)\n", c, what, p5, p6);
    int r = SNCmd(c, params, buf, fp, p5, p6);
    trace("### SNCmd(0x%x %s) = 0x%x\n", c, what, r);
    fprintf(stderr, "SNCmd(0x%x %s) = 0x%x\n", c, what, r);
    return r;
}

static int paper_present(void)
{
    return cmd("paper?", 0x16, NULL, NULL, NULL, 0, 0) == 0;
}

static void eject(void)
{
    for (int i = 0; i < 12 && paper_present(); i++)
        cmd("feed", 0x13, NULL, NULL, NULL, 1, 1000);
}

static int do_scan(int res, int bpp, const char *out, int width, int height)
{
    static uint32_t p[0x210];
    int bpl, r;

    if (!paper_present() && !getenv("NEAT_NOPAPER")) {
        fprintf(stderr, "no paper detected; insert a sheet\n");
        return 1;
    }
    width &= bpp == 1 ? ~31 : ~3;
    memset(p, 0, sizeof(p));
    p[0] = 8;
    p[1] = bpp;
    p[2] = res;
    p[3] = res;
    p[4] = 0;
    p[5] = 0;
    p[6] = width;
    p[7] = height;
    double gamma = 1.6;
    memcpy(&p[8], &gamma, 8);
    p[10] = 0xe3f0 | (0x24u << 16);
    p[0xc] = 0x17f;
    p[0x20d] = 1;
    p[0x20f] = 4;
    r = cmd("set-params", 2, p, NULL, NULL, 0, 0);
    if (r >= 0xe000)
        return 1;

    float bc[3] = {1.4f, 0, 0};
    cmd("bc-enable", 0x19, NULL, NULL, NULL, 1, 0);
    bc[0] = 16 * 0.1f; /* DS default: gamma slider 16 * 0.1, contrast 10?, brightness -5? */
    bc[1] = 10;
    bc[2] = -5;
    cmd("bc-set", 0x1a, NULL, NULL, bc, 0, 0);

    r = cmd("load-calib", 4, NULL, NULL, NULL, 0, 0);
    cmd("lamp", 9, NULL, NULL, NULL, 1, 0);
    r = cmd("start", 5, NULL, NULL, NULL, 0, 0);
    if (r == 0xe009 || r == 0xe003)
        return 1;

    bpl = bpp == 1 ? (width + 7) / 8 : width * (bpp / 8);
    size_t total = (size_t)bpl * height;
    uint8_t *img = calloc(1, total);
    size_t got = 0;
    int lines_per_read = 64;
    while (got < total) {
        size_t want = (size_t)bpl * lines_per_read;
        if (want > total - got)
            want = total - got;
        r = cmd("read", 6, NULL, img + got, NULL, 0, (uint32_t)want);
        if (r != 0) {
            fprintf(stderr, "read returned 0x%x after %zu lines\n", r, got / bpl);
            break;
        }
        got += want;
    }
    cmd("stop", 7, NULL, NULL, NULL, 0, 0);
    eject();
    cmd("lamp-off", 9, NULL, NULL, NULL, 0, 0);

    int lines = got / bpl;
    FILE *f = fopen(out, "wb");
    if (bpp == 24)
        fprintf(f, "P6\n%d %d\n255\n", width, lines);
    else if (bpp == 8)
        fprintf(f, "P5\n%d %d\n255\n", width, lines);
    else if (bpp == 16)
        fprintf(f, "P5\n%d %d\n65535\n", width, lines);
    else
        fprintf(f, "P4\n%d %d\n", width, lines);
    fwrite(img, 1, (size_t)bpl * lines, f);
    fclose(f);
    fprintf(stderr, "wrote %s: %dx%d, %d bpp\n", out, width, lines, bpp);
    free(img);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s DLL status|scan|calibrate|feed ...\n", argv[0]);
        return 2;
    }
    const char *tp = getenv("NEAT_TRACE");
    trace_fp = fopen(tp ? tp : "trace.log", "w");
    bulk_dump_dir = getenv("NEAT_BULK_DIR");

    if (!pe_load(argv[1]))
        return 1;
    SNCmd = (sncmd_t)pe_export("SNCmd");
    if (!SNCmd) {
        fprintf(stderr, "SNCmd export not found\n");
        return 1;
    }

    int rc = 0;
    if (cmd("open", 1, NULL, NULL, NULL, 0, 0) != 0)
        return 1;
    cmd("connected?", 0x17, NULL, NULL, NULL, 0, 0);

    if (!strcmp(argv[2], "status")) {
        uint8_t buttons[2] = {0};
        cmd("buttons", 0x10, NULL, buttons, NULL, 0, 0);
        printf("paper=%d button0=%d button1=%d\n", paper_present(), buttons[0], buttons[1]);
    } else if (!strcmp(argv[2], "raw") && argc > 5) {
        /* raw CMD P5 P6: for commands that need no buffers */
        cmd("raw", strtoul(argv[3], NULL, 0), NULL, NULL, NULL, atoi(argv[4]), strtoul(argv[5], NULL, 0));
    } else if (!strcmp(argv[2], "feed") && argc > 3) {
        cmd("feed", 0x13, NULL, NULL, NULL, 1, atoi(argv[3]));
    } else if (!strcmp(argv[2], "calibrate")) {
        if (!paper_present()) {
            fprintf(stderr, "insert the calibration sheet first\n");
            rc = 1;
        } else {
            cmd("feed", 0x13, NULL, NULL, NULL, 1, 300);
            rc = cmd("calibrate", 3, NULL, NULL, NULL, 0, 0) != 0;
            cmd("lamp-off", 9, NULL, NULL, NULL, 0, 0);
            eject();
        }
    } else if (!strcmp(argv[2], "scan") && argc > 5) {
        int res = atoi(argv[3]), bpp = atoi(argv[4]);
        int w = argc > 6 ? atoi(argv[6]) : res * 85 / 10;
        int h = argc > 7 ? atoi(argv[7]) : res * 11;
        rc = do_scan(res, bpp, argv[5], w, h);
    } else {
        fprintf(stderr, "bad command\n");
        rc = 2;
    }
    cmd("close", 8, NULL, NULL, NULL, 0, 0);
    return rc;
}

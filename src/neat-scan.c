/*
 * neat-scan: command-line test tool for the NM-1000 driver core.
 *
 *   neat-scan info
 *   neat-scan prepare            read all calibration tables from flash
 *   neat-scan scan [-r DPI] [-g] [-o OUT.pnm]
 *   neat-scan feed STEPS
 */
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nm1000.h"

static int verbose;

static void log_cb(int level, const char *msg)
{
    if (level <= verbose)
        fprintf(stderr, "nm1000: %s\n", msg);
}

static int die(const char *what, int r)
{
    fprintf(stderr, "%s: %s\n", what, nm1000_strerror(r));
    return 1;
}

static int do_scan(struct nm1000 *d, int dpi, int color, const char *out)
{
    struct nm1000_scan_info info = {0};
    size_t cap = 0, used = 0;
    uint8_t *img = NULL;
    int r, lines = 0;

    r = nm1000_start(d, dpi, color, 900, &info);
    if (r)
        return die("start", r);
    for (;;) {
        int n;
        if (cap - used < (size_t)info.bytes_per_line * 64) {
            cap = cap ? cap * 2 : (size_t)info.bytes_per_line * 1024;
            img = realloc(img, cap);
            if (!img)
                return die("scan", NM1000_ERR_NOMEM);
        }
        r = nm1000_read(d, img + used, 64, &n);
        used += (size_t)n * info.bytes_per_line;
        lines += n;
        if (r == NM1000_EOF)
            break;
        if (r) {
            nm1000_finish(d);
            free(img);
            return die("read", r);
        }
    }
    r = nm1000_finish(d);
    if (r)
        fprintf(stderr, "finish: %s\n", nm1000_strerror(r));

    FILE *f = fopen(out, "wb");
    if (!f) {
        perror(out);
        free(img);
        return 1;
    }
    fprintf(f, "P%d\n%d %d\n255\n", color ? 6 : 5, info.pixels, lines);
    fwrite(img, 1, used, f);
    fclose(f);
    fprintf(stderr, "wrote %s: %dx%d @ %d dpi %s\n", out, info.pixels, lines, dpi, color ? "colour" : "gray");
    free(img);
    return 0;
}

int main(int argc, char **argv)
{
    int dpi = 300, color = 1, c, r, rc = 0;
    const char *out = "scan.pnm";
    struct nm1000 *d;

    while ((c = getopt(argc, argv, "r:go:v")) != -1) {
        switch (c) {
        case 'r': dpi = atoi(optarg); break;
        case 'g': color = 0; break;
        case 'o': out = optarg; break;
        case 'v': verbose++; break;
        default:
            fprintf(stderr, "usage: %s [-v] [-r dpi] [-g] [-o out.pnm] info|prepare|scan|feed N\n", argv[0]);
            return 2;
        }
    }
    if (optind >= argc) {
        fprintf(stderr, "missing command\n");
        return 2;
    }
    nm1000_set_log(log_cb);
    if ((r = nm1000_open(&d, -1, -1)))
        return die("open", r);

    const char *cmd = argv[optind];
    if (!strcmp(cmd, "info")) {
        r = nm1000_paper_present(d);
        printf("serial: %s\npaper: %s\n", nm1000_serial(d), r < 0 ? nm1000_strerror(r) : r ? "yes" : "no");
    } else if (!strcmp(cmd, "prepare")) {
        for (const int *res = nm1000_resolutions(); *res; res++)
            for (int col = 0; col < 2; col++)
                if ((r = nm1000_prepare(d, *res, col))) {
                    rc = die("prepare", r);
                    break;
                }
    } else if (!strcmp(cmd, "scan")) {
        rc = do_scan(d, dpi, color, out);
    } else if (!strcmp(cmd, "feed") && optind + 1 < argc) {
        if ((r = nm1000_feed(d, atoi(argv[optind + 1]))))
            rc = die("feed", r);
    } else {
        fprintf(stderr, "unknown command %s\n", cmd);
        rc = 2;
    }
    nm1000_close(d);
    return rc;
}

/*
 * Neat NM-1000 (USB 1f44:0001) mobile sheetfed scanner — device driver core.
 *
 * The scanner is built around a Genesys Logic GL123 (a GL847-family ASIC).
 * Register programs for each scan mode were captured from the vendor
 * driver and are replayed from nm1000_tables.h; per-device shading
 * calibration is read from the scanner's own SPI flash.
 */
#ifndef NM1000_H
#define NM1000_H

#include <stddef.h>
#include <stdint.h>

#define NM1000_VENDOR 0x1f44
#define NM1000_PRODUCT 0x0001

enum nm1000_status {
    NM1000_OK = 0,
    NM1000_EOF = 1,          /* end of page reached */
    NM1000_ERR_IO = -1,
    NM1000_ERR_NODEV = -2,
    NM1000_ERR_NOPAPER = -3,
    NM1000_ERR_INVAL = -4,
    NM1000_ERR_NOMEM = -5,
    NM1000_ERR_BUSY = -6,
    NM1000_ERR_CANCELLED = -7,
};

struct nm1000;

struct nm1000_scan_info {
    int dpi;
    int color;           /* 1: 24-bit RGB, 0: 8-bit gray */
    int pixels;          /* pixels per line */
    int bytes_per_line;  /* pixels * (color ? 3 : 1) */
    int max_lines;       /* upper bound of lines this scan can deliver */
};

typedef void (*nm1000_log_fn)(int level, const char *msg);
void nm1000_set_log(nm1000_log_fn fn);

/* Optional: restrict nm1000_open() to the device on this bus/address. */
int nm1000_open(struct nm1000 **dev, int bus, int address);
void nm1000_close(struct nm1000 *dev);
const char *nm1000_serial(struct nm1000 *dev);
const char *nm1000_strerror(int status);

/* 1 if a sheet is at the input sensor, 0 if not, <0 on error. */
int nm1000_paper_present(struct nm1000 *dev);

/* Supported resolutions (dpi), terminated by 0. */
const int *nm1000_resolutions(void);

/* Load (from cache or the scanner's flash) the calibration a mode needs.
 * nm1000_start() does this itself; calling it early just moves the delay. */
int nm1000_prepare(struct nm1000 *dev, int dpi, int color);

/* Program the scanner for a scan and start the feed. max_height_mm bounds
 * the page length; the scan normally ends earlier when the paper's trailing
 * edge is detected. */
int nm1000_start(struct nm1000 *dev, int dpi, int color, int max_height_mm,
                 struct nm1000_scan_info *info);

/* Read up to max_lines complete lines (RGB interleaved or gray) into buf.
 * Returns NM1000_OK with *lines set (possibly fewer than asked), or
 * NM1000_EOF once the page has ended and all lines were delivered. */
int nm1000_read(struct nm1000 *dev, uint8_t *buf, int max_lines, int *lines);

/* Finish or abort a scan: stop the scan engine, eject the sheet, lamp off. */
int nm1000_finish(struct nm1000 *dev);

/* Move the paper by `steps` motor steps (1000 is roughly 3.3 inches). */
int nm1000_feed(struct nm1000 *dev, int steps);

#endif

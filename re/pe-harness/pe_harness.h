#ifndef PE_HARNESS_H
#define PE_HARNESS_H

#include <stdint.h>
#include <stdio.h>

extern FILE *trace_fp;
extern int trace_bulk_in_full;
extern const char *bulk_dump_dir;

void trace(const char *fmt, ...);
void *pe_load(const char *path);
void *pe_export(const char *name);
uint8_t *image_base(void);

#endif

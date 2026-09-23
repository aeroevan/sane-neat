# Reverse-engineering notes

These tools are only needed to extend or re-derive the driver. The driver itself
contains no vendor code.

## Vendor driver

Neat's Windows driver (the same one for NC-1000 / ND-1000 / NM-1000):

    https://s3.amazonaws.com/scanner-drivers/Windows/NeatMobile/x64/Scanner.Install64.Neat.Mobile.MSI.msi
    sha256 cb006d10beb41cc20d14eb0b35cbdb866fb0ddba44335644bf42d66ff9cba794

`7z x` the MSI to get `neatmobilescanner_x64.dll`. It has a single export,
`int SNCmd(cmd, uint *params, void *buf, float *f, int p5, uint p6)`:

| cmd  | meaning                                          |
|------|--------------------------------------------------|
| 1/8  | open / close                                     |
| 2    | set parameters (0x840-byte block, see neatcap.c) |
| 3    | calibrate with the calibration sheet             |
| 4    | load calibration (flash, cached to AppData)      |
| 5    | start scan                                       |
| 6    | read `p6` bytes into `buf`                       |
| 7    | stop                                             |
| 9    | lamp on/off (`p5`)                               |
| 0x10 | buttons                                          |
| 0x13 | feed `p6` motor steps                            |
| 0x16 | paper present? (0 = yes)                         |

## pe-harness

`neatcap` loads the x64 DLL into a Linux process, stubs about 95 kernel32/user32
imports, maps `\\.\USBSCAN0` + `DeviceIoControl`/`ReadFile`/`WriteFile` onto
libusb, and logs every transfer:

    make -C re/pe-harness
    NEAT_TRACE=trace.log re/pe-harness/neatcap neatmobilescanner_x64.dll scan 300 24 out.ppm

`NEAT_NOPAPER=1` skips the paper check, which is enough to capture register
programming. It creates a `winroot/` sandbox for the files the DLL writes.

## Protocol summary

- Register write: `40 04|0c 0083 0000`, data = (reg, val) pairs.
- Register read: `c0 04 008e (0x22 | reg<<8)`, returns n values + `0x55`.
- Bulk: `40 04 0082 {0=in,1=out}` with addr32/len32 LE, then EP 0x81 / 0x02.
  - `0x10000000` image FIFO (colour lines arrive as R, G, B planes)
  - `0x10014000` etc. shading tables, `0x01000000` gamma, `0x1000c000` motor
  - `0x03000000` SPI flash controller; set GPIO `a7=01 a6=1d` first and
    `a7=00 a6=1c` afterwards
- AFE: reg 0x51 = address, 0x3a/0x3b = data.
- Flash: calibration regions 0x00xxxx (600 colour), 0x01xxxx (300 colour),
  0x02xxxx (600 gray), 0x03xxxx (300 gray). 150/200 dpi reuse the 300 tables.
  Each table is `[AFE offset, AFE gain, (dark, gain) per pixel ...]`.
- Paper: write `0a=20`, then reg 0x40 bit 6 = no paper. After the trailing edge,
  the remaining lines = (regs 0x92:0x93 + regs 0x4b:0x4d) / 3 in colour
  (/1 in gray).
- Power-up (reg 0x41 bit 7 clear): see `cold_init()` in `src/nm1000.c`.

## Regenerating tables

    mkdir /tmp/t && cp re/traces/*.xz /tmp/t && xz -d /tmp/t/*.xz
    re/tools/gen_tables.py /tmp/t /tmp/t/warm_open.log /tmp/t/scan_300_color.log > src/nm1000_tables.h

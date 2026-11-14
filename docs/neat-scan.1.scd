NEAT-SCAN(1)

# NAME

neat-scan - utility program for the NM-1000 Neat Receipts scanner

# SYNOPSIS

*neat-scan* [_-v_] [_-r_ _dpi_] [_-g_] [_-o_ _out.pnm_] _info_|_prepare_|_scan_|_feed N_

# DESCRIPTION

A command-line tool to initialize, validate, and scan documents using the
NM-1000 Neat Receipts scanner.

# OPTIONS

*-v*
	Verbose output

# COMMANDS

*info*
	Returns the device serial number and paper status

*scan*
	Scan a document. The document must be against the sensor.

*-r* _dpi_
	Set the resolution in dots-per-inch. Valid resolutions are 150, 200, 300,
	600. Default: 300

*-g*
	Scan in color

*-o* _filename_
	The name of the output image file

*prepare*
	Initializes the device calibration cache. The device will do this
	automatically on *scan* if the calibration files are not present, but
	*prepare* will generate the calibration files without requiring there be
	paper in the device.

	The calibration files are kept in _$HOME/.cache/sane-neat/_

*feed*
	Low-level diagnostic tool to runs the scanner's feed motor forward a number
	of steps. Used to test the feed reader.

# AUTHORS

Maintained by Evan McClain, <https://github.com/aeroevan>.  Latest sources can be
found at https://github.com/aeroevan/sane-neat.

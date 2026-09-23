#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later WITH SANE-exception
# Copyright (C) 2026 Evan McClain
# Build the RPM on a Fedora system that has the build dependencies installed
# (see the spec). Used by CI and by build-rpm.sh inside a container.
# Output: dist/*.rpm (binary and source).
set -eu
cd "$(dirname "$0")/.."
NAME=sane-backends-neat
VERSION=$(sed -n 's/^Version: *//p' packaging/$NAME.spec)
TOP=$(mktemp -d)
mkdir -p dist "$TOP/SOURCES"
tar czf "$TOP/SOURCES/$NAME-$VERSION.tar.gz" --transform "s,^,$NAME-$VERSION/," \
    Makefile README.md LICENSE COPYING src udev packaging
rpmbuild -ba --quiet --define "_topdir $TOP" packaging/$NAME.spec
cp "$TOP"/RPMS/*/*.rpm "$TOP"/SRPMS/*.rpm dist/
rm -rf "$TOP"
ls -1 dist/*.rpm

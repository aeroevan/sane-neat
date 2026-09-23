#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later WITH SANE-exception
# Copyright (C) 2026 Evan McClain
# Build the RPM in a throwaway Fedora container; output lands in dist/.
#   FEDORA=43 ./packaging/build-rpm.sh
set -eu
cd "$(dirname "$0")/.."
FEDORA=${FEDORA:-44}
PODMAN=podman
# Inside a toolbox, use the host's podman (nested containers do not work).
[ -e /run/.toolboxenv ] && PODMAN="flatpak-spawn --host podman"
$PODMAN run --rm -v "$PWD:/src:Z" -w /src registry.fedoraproject.org/fedora:$FEDORA sh -euc "
    dnf -y -q install rpm-build gcc make tar 'pkgconfig(libusb-1.0)' sane-backends-devel >/dev/null
    ./packaging/rpmbuild.sh
"

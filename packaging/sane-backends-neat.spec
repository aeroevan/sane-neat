Name:           sane-backends-neat
Version:        0.1.0
Release:        1%{?dist}
Summary:        SANE backend for the Neat NM-1000 mobile sheetfed scanner
License:        GPL-2.0-or-later WITH SANE-exception
Source0:        %{name}-%{version}.tar.gz

BuildRequires:  gcc
BuildRequires:  make
BuildRequires:  pkgconfig(libusb-1.0)
BuildRequires:  sane-backends-devel
Requires:       sane-backends

%description
A SANE backend ("neat") and a small command-line tool (neat-scan) for the
Neat NM-1000 / Neat Receipts mobile scanner (USB 1f44:0001), a Genesys Logic
GL123-based sheetfed scanner. Scans in colour or gray at 150-600 dpi using
the factory calibration stored in the scanner's own flash.

%prep
%autosetup

%build
%make_build CFLAGS="%{optflags}" LDFLAGS="%{build_ldflags}"

%install
%make_install LIBDIR=%{_libdir} PREFIX=%{_prefix} UDEVDIR=%{_udevrulesdir}

%files
%license COPYING LICENSE
%doc README.md
%{_libdir}/sane/libsane-neat.so.1
%{_libdir}/sane/libsane-neat.so
%{_bindir}/neat-scan
%config(noreplace) %{_sysconfdir}/sane.d/dll.d/neat
%{_udevrulesdir}/70-neat-nm1000.rules

%changelog
* Wed Sep 23 2026 Evan <aeroevan@gmail.com> - 0.1.0-1
- Initial package

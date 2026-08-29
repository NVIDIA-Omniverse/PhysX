#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Fetch the released ovstage wheel for the current platform. The wheel is
# self-contained: it carries the C++ package (ovstage/include,
# ovstage/bin/libovstage.so + plugins, ovstage/lib/cmake) and the importable python
# package. It is resolved from the PEP 503 simple index in INDEX_URL; switch that
# index with scripts/sync_ovstage_version.py --source {internal,public}.
# --dest extracts the wheel's ovstage/ tree (OVSTAGE_DIR, resolved by
# OvstageDependency.cmake) and mirrors the package notices to dest/. --wheel-dest
# also places the wheel there for the python tests.
#
# Stdlib only (urllib + zipfile + hashlib) so it can run early in dependency
# fetching, before cmake or a full python environment is available.

import argparse
import hashlib
import os
import re
import shutil
import sys
import urllib.parse
import urllib.request
import zipfile

# --- Release coordinates --------------------------------------------------------
# The pinned ovstage version (a numeric pip/wheel version). Run
# scripts/sync_ovstage_version.py to propagate it into python/pyproject.toml and
# the python-test lockfile.
OVSTAGE_VERSION = "0.1.1.355824"

# The PEP 503 simple index the ovstage wheel is fetched from. Switch it with
# scripts/sync_ovstage_version.py --source {internal,public}.
INDEX_URL = "https://pypi.org/simple/ovstage/"


def _numeric_version(version):
    """The pip/wheel version: drop any trailing git-hash component (the optional 5th).

    Dropped by position because a short hash can occasionally be all digits.
    """
    parts = version.split(".")
    if len(parts) >= 5:
        parts = parts[:4]
    elif parts and not parts[-1].isdigit():
        parts = parts[:-1]
    return ".".join(parts)


# The numeric version is the wheel version, and what _canonical_wheel_name validates
# a downloaded wheel against.
EXPECTED_WHEEL_VERSION = _numeric_version(OVSTAGE_VERSION)


def _wheel_plat(platform):
    if platform == "windows-x86_64":
        return "win_amd64"
    if platform == "windows-arm64":
        return "win_arm64"
    return platform


def _wheel_name(version, platform):
    return "ovstage-%s-py3-none-%s.whl" % (_numeric_version(version), _wheel_plat(platform))


def _pypi_wheel_url(index_url, wheel_name):
    """Resolve wheel_name to (url, sha256) from a PEP 503 simple index."""
    with urllib.request.urlopen(urllib.request.Request(index_url), timeout=120) as resp:
        html = resp.read().decode("utf-8", "replace")
    for href in re.findall(r'href="([^"]+)"', html):
        file_part = href.split("#", 1)[0].rstrip("/").rsplit("/", 1)[-1]
        if file_part != wheel_name:
            continue
        url = urllib.parse.urljoin(index_url, href.split("#", 1)[0])
        m = re.search(r"#sha256=([0-9a-fA-F]{64})", href)
        return url, (m.group(1).lower() if m else None)
    sys.exit("fetch_ovstage_release: %s not found in index %s" % (wheel_name, index_url))


def _resolve_wheel(version, platform):
    """Return (url, wheel_name, sha256) for the ovstage wheel, resolved from INDEX_URL."""
    wheel = _wheel_name(version, platform)
    url, sha = _pypi_wheel_url(INDEX_URL, wheel)
    return url, wheel, sha


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _download(url, dest_path, expected_sha256=None):
    print("  ovstage: downloading %s" % os.path.basename(dest_path))
    with urllib.request.urlopen(urllib.request.Request(url), timeout=600) as resp, \
            open(dest_path, "wb") as out:
        shutil.copyfileobj(resp, out)
    size = os.path.getsize(dest_path)
    if size < 4096:  # a tiny body is a server error page
        with open(dest_path, "rb") as f:
            body = f.read(512)
        sys.exit("fetch_ovstage_release: %s looks invalid (size=%d): %r"
                 % (os.path.basename(dest_path), size, body))
    if expected_sha256:
        want = expected_sha256.split(":", 1)[-1].strip().lower()  # tolerate a 'sha256:' prefix
        actual = _sha256(dest_path)
        if actual != want:
            sys.exit("fetch_ovstage_release: %s sha256 %s does not match expected %s"
                     % (os.path.basename(dest_path), actual, want))


def _wheel_metadata_version(whl_path):
    with zipfile.ZipFile(whl_path) as zf:
        metadata_files = [name for name in zf.namelist() if name.endswith(".dist-info/METADATA")]
        if len(metadata_files) != 1:
            sys.exit("fetch_ovstage_release: expected one METADATA file in %s, found %d"
                     % (whl_path, len(metadata_files)))
        metadata = zf.read(metadata_files[0]).decode("utf-8", "replace")

    for line in metadata.splitlines():
        if line.startswith("Version:"):
            version = line.split(":", 1)[1].strip()
            if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+!]*", version):
                return version
            sys.exit("fetch_ovstage_release: invalid wheel Version %r in %s" % (version, whl_path))
    sys.exit("fetch_ovstage_release: no Version field in %s" % whl_path)


def _canonical_wheel_name(whl_path, platform):
    version = _wheel_metadata_version(whl_path)
    if version != EXPECTED_WHEEL_VERSION:
        sys.exit("fetch_ovstage_release: wheel version %s does not match pinned release %s"
                 % (version, EXPECTED_WHEEL_VERSION))
    return "ovstage-%s-py3-none-%s.whl" % (version, _wheel_plat(platform))


def _extract_wheel(whl_path, dest):
    """Extract the wheel's top-level ovstage/ tree into OVSTAGE_DIR (dest/ovstage/...).

    The sibling ovstage-<version>.dist-info/ is python metadata and is skipped. The
    package notices are mirrored to the dest root for the SDK install's notices check.
    """
    if os.path.islink(dest) or os.path.isfile(dest):
        os.remove(dest)
    elif os.path.isdir(dest):
        shutil.rmtree(dest)
    os.makedirs(dest)

    dest_abs = os.path.abspath(dest)
    with zipfile.ZipFile(whl_path) as zf:
        for info in zf.infolist():
            name = info.filename
            if not name.startswith("ovstage/") or name.endswith("/"):
                continue
            out_path = os.path.abspath(os.path.join(dest, name))
            if not out_path.startswith(dest_abs + os.sep):
                sys.exit("fetch_ovstage_release: wheel entry escapes dest: %r" % name)
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            with zf.open(info) as src, open(out_path, "wb") as out:
                shutil.copyfileobj(src, out)
            mode = (info.external_attr >> 16) & 0o777
            if mode:
                os.chmod(out_path, mode)

    pkg_dir = os.path.join(dest, "ovstage")
    if not os.path.isdir(os.path.join(pkg_dir, "include")) or not os.path.isdir(os.path.join(pkg_dir, "bin")):
        sys.exit("fetch_ovstage_release: extracted ovstage at %s is missing ovstage/include or "
                 "ovstage/bin (unexpected wheel layout)." % dest)

    src_notice = os.path.join(pkg_dir, "THIRD-PARTY-NOTICES.txt")
    if os.path.isfile(src_notice):
        shutil.copyfile(src_notice, os.path.join(dest, "THIRD-PARTY-NOTICES.txt"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True,
                    help="manylinux_2_35_x86_64 | manylinux_2_35_aarch64 | windows-x86_64 | windows-arm64")
    ap.add_argument("--dest", required=True, help="extract the ovstage package here (OVSTAGE_DIR)")
    ap.add_argument("--wheel-dest", default="", help="also place the python wheel here (optional; for python tests)")
    args = ap.parse_args()

    # Offline / pre-staged escape hatch: when OVSTAGE_SKIP_FETCH is set, trust that
    # --dest already holds a valid ovstage and do not download (e.g. building against
    # a locally-staged ovstage that is not yet published).
    if os.environ.get("OVSTAGE_SKIP_FETCH"):
        dest = os.path.abspath(args.dest)
        has_pkg = (os.path.isdir(os.path.join(dest, "ovstage"))
                   or os.path.isdir(os.path.join(dest, "include")))
        if not has_pkg:
            sys.exit("fetch_ovstage_release: OVSTAGE_SKIP_FETCH set but %s has no "
                     "staged ovstage (expected ovstage/ or include/)." % dest)
        print("  ovstage: OVSTAGE_SKIP_FETCH set -- using pre-staged %s" % dest)
        return

    url, wheel_name, sha = _resolve_wheel(OVSTAGE_VERSION, args.platform)
    print("  ovstage: version=%s index=%s" % (_numeric_version(OVSTAGE_VERSION), INDEX_URL))

    dest = os.path.abspath(args.dest)
    parent = os.path.dirname(dest)
    if not os.path.isdir(parent):
        os.makedirs(parent)
    wheel_tmp = os.path.join(parent, wheel_name)
    _download(url, wheel_tmp, sha)

    # Validate the embedded build version before trusting the payload.
    canonical_name = _canonical_wheel_name(wheel_tmp, args.platform)

    _extract_wheel(wheel_tmp, dest)
    print("  ovstage: package extracted to %s" % dest)

    if args.wheel_dest:
        wdir = os.path.abspath(args.wheel_dest)
        if not os.path.isdir(wdir):
            os.makedirs(wdir)
        for f in os.listdir(wdir):
            if f.startswith("ovstage-") and f.endswith(".whl"):
                os.remove(os.path.join(wdir, f))
        shutil.copyfile(wheel_tmp, os.path.join(wdir, canonical_name))
        print("  ovstage: wheel placed at %s/%s" % (wdir, canonical_name))

    os.remove(wheel_tmp)


if __name__ == "__main__":
    main()

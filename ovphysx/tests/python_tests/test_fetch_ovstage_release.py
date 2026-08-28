# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Regression tests for the ovstage release fetcher."""

import hashlib
import sys
import zipfile
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import fetch_ovstage_release as m  # noqa: E402

_PINNED = m.EXPECTED_WHEEL_VERSION


def test_canonical_wheel_name_uses_embedded_build_version(tmp_path):
    wheel = tmp_path / "ovstage-0.1.0-py3-none-manylinux_2_35_x86_64.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr(
            f"ovstage-{_PINNED}.dist-info/METADATA",
            f"Metadata-Version: 2.4\nName: ovstage\nVersion: {_PINNED}\n",
        )
    assert m._canonical_wheel_name(str(wheel), "manylinux_2_35_x86_64") == \
        f"ovstage-{_PINNED}-py3-none-manylinux_2_35_x86_64.whl"


def test_canonical_wheel_name_rejects_release_mismatch(tmp_path):
    wheel = tmp_path / "ovstage-0.1.0-py3-none-manylinux_2_35_x86_64.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr(
            "ovstage-0.1.0.335995.dist-info/METADATA",
            "Metadata-Version: 2.4\nName: ovstage\nVersion: 0.1.0.335995\n",
        )
    with pytest.raises(SystemExit, match=rf"wheel version 0.1.0.335995 does not match pinned release {_PINNED}"):
        m._canonical_wheel_name(str(wheel), "manylinux_2_35_x86_64")


def test_wheel_plat_maps_windows_tags():
    assert m._wheel_plat("windows-x86_64") == "win_amd64"
    assert m._wheel_plat("windows-arm64") == "win_arm64"
    assert m._wheel_plat("manylinux_2_35_aarch64") == "manylinux_2_35_aarch64"


def test_numeric_version_drops_trailing_git_hash():
    assert m._numeric_version("0.1.0.999999.deadbeef") == "0.1.0.999999"  # M.m.p.build.hash
    assert m._numeric_version("0.1.0.999999") == "0.1.0.999999"           # M.m.p.build
    assert m._numeric_version("0.1.0") == "0.1.0"                         # M.m.p
    # An all-digit hash is still the 5th component, dropped by position.
    assert m._numeric_version("0.1.0.888888.12345678") == "0.1.0.888888"


def test_wheel_name_uses_numeric_version_and_wheel_tag():
    assert m._wheel_name("0.1.0.999999.deadbeef", "windows-x86_64") == \
        "ovstage-0.1.0.999999-py3-none-win_amd64.whl"
    assert m._wheel_name("0.1.0.999999", "manylinux_2_35_x86_64") == \
        "ovstage-0.1.0.999999-py3-none-manylinux_2_35_x86_64.whl"


def test_resolve_wheel_uses_index_url(monkeypatch):
    seen = {}

    def fake_lookup(index_url, wheel_name):
        seen["index"] = index_url
        return "https://files.example.invalid/" + wheel_name, "a" * 64

    monkeypatch.setattr(m, "_pypi_wheel_url", fake_lookup)
    url, name, sha = m._resolve_wheel("0.1.0.999999", "win_amd64")
    assert seen["index"] == m.INDEX_URL
    assert name == "ovstage-0.1.0.999999-py3-none-win_amd64.whl"
    assert sha == "a" * 64
    assert url.endswith(name)


def test_pypi_wheel_url_parses_index(monkeypatch):
    index_html = (
        '<a href="https://files.pythonhosted.org/packages/ab/cd/'
        'ovstage-0.1.0.888888-py3-none-manylinux_2_35_x86_64.whl#sha256='
        + "d" * 64 + '">ovstage-0.1.0.888888-py3-none-manylinux_2_35_x86_64.whl</a>\n'
        '<a href="https://files.pythonhosted.org/packages/ef/01/'
        'ovstage-0.1.0.888888-py3-none-win_amd64.whl#sha256=' + "0" * 64 + '">win</a>\n'
    )

    class _Resp:
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def read(self): return index_html.encode()

    monkeypatch.setattr(m.urllib.request, "urlopen", lambda *a, **k: _Resp())
    url, sha = m._pypi_wheel_url("https://pypi.org/simple/ovstage/",
                                 "ovstage-0.1.0.888888-py3-none-manylinux_2_35_x86_64.whl")
    assert url == ("https://files.pythonhosted.org/packages/ab/cd/"
                   "ovstage-0.1.0.888888-py3-none-manylinux_2_35_x86_64.whl")
    assert sha == "d" * 64


def test_download_verifies_sha256(tmp_path):
    src = tmp_path / "blob.bin"
    src.write_bytes(b"x" * 5000)  # over the 4096 size floor
    good = hashlib.sha256(src.read_bytes()).hexdigest()
    m._download(src.as_uri(), str(tmp_path / "ok.bin"), good)
    m._download(src.as_uri(), str(tmp_path / "ok2.bin"), "sha256:" + good)  # 'sha256:' prefix tolerated
    with pytest.raises(SystemExit, match="sha256"):
        m._download(src.as_uri(), str(tmp_path / "bad.bin"), "0" * 64)


def test_extract_wheel_preserves_ovstage_prefix(tmp_path):
    wheel = tmp_path / "ovstage-py3-none-manylinux_2_35_x86_64.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("ovstage/__init__.py", "# package\n")
        archive.writestr("ovstage/include/ovstage/ovstage.h", "// header\n")
        archive.writestr("ovstage/bin/libovstage.so", "ELF-ish\n")
        archive.writestr("ovstage/lib/cmake/ovstage/ovstageConfig.cmake", "# cmake\n")
        archive.writestr("ovstage/THIRD-PARTY-NOTICES.txt", "notices\n")
        archive.writestr("ovstage-0.1.0.dist-info/METADATA", "Name: ovstage\nVersion: 0.1.0\n")

    dest = tmp_path / "ovstage"
    m._extract_wheel(str(wheel), str(dest))

    assert (dest / "ovstage" / "include" / "ovstage" / "ovstage.h").is_file()
    assert (dest / "ovstage" / "bin" / "libovstage.so").is_file()
    # Notices mirrored to the OVSTAGE_DIR root for the SDK notices install rule.
    assert (dest / "THIRD-PARTY-NOTICES.txt").is_file()
    # The python dist-info must not leak into OVSTAGE_DIR.
    assert not list(dest.glob("*.dist-info"))


def test_extract_wheel_rejects_layout_without_include_or_bin(tmp_path):
    wheel = tmp_path / "ovstage-py3-none-manylinux_2_35_x86_64.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("ovstage/lib/cmake/ovstage/ovstageConfig.cmake", "# cmake\n")
    with pytest.raises(SystemExit, match="missing ovstage/include or ovstage/bin"):
        m._extract_wheel(str(wheel), str(tmp_path / "ovstage"))


def test_extract_wheel_rejects_zip_slip(tmp_path):
    wheel = tmp_path / "ovstage-py3-none-manylinux_2_35_x86_64.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("ovstage/include/x.h", "ok\n")
        archive.writestr("ovstage/bin/y", "ok\n")
        archive.writestr("ovstage/../../escape.txt", "pwned\n")
    with pytest.raises(SystemExit, match="escapes dest"):
        m._extract_wheel(str(wheel), str(tmp_path / "ovstage"))

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""strip_unstripped_elf_binaries uses readelf, not the libmagic file tool.

ELF symbol tables and compiler comments are stripped, including comments in
already-stripped dependencies. Clean ELFs stay unchanged. Missing readelf fails.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="ELF stripping is Linux-only")

OVPHYSX_DIR = Path(__file__).resolve().parents[2]
BUILD_COMMON = OVPHYSX_DIR / "scripts" / "build_common.cmake"
CMAKE = shutil.which("cmake") or "cmake"

_DRIVER = """
cmake_minimum_required(VERSION 3.16)
include("${BUILD_COMMON}")
strip_unstripped_elf_binaries("${ROOT}" "test tree")
"""


def _has_symtab(path: Path) -> bool:
    result = subprocess.run(
        ["readelf", "-S", "-W", str(path)],
        check=True,
        capture_output=True,
        text=True,
    )
    return " SYMTAB " in result.stdout


def _has_compiler_comment(path: Path) -> bool:
    result = subprocess.run(
        ["readelf", "-S", "-W", str(path)],
        check=True, capture_output=True, text=True,
    )
    return " .comment " in result.stdout


def _make_libs(tmp: Path) -> tuple[Path, Path]:
    root = tmp / "tree"
    root.mkdir()
    source = tmp / "symbol.c"
    source.write_text("int ovphysx_strip_probe(void) { return 1; }\n", encoding="ascii")
    unstripped = root / "unstripped.so"
    stripped = root / "stripped.so"
    subprocess.run(
        ["gcc", "-shared", "-fPIC", "-o", str(unstripped), str(source)],
        check=True,
    )
    shutil.copyfile(unstripped, stripped)
    subprocess.run(["strip", "--strip-unneeded", str(stripped)], check=True)
    (root / "notes.so").write_text("not an elf\n", encoding="ascii")
    return unstripped, stripped


def _run_strip(root: Path, path: str | None = None) -> subprocess.CompletedProcess[str]:
    driver = root.parent / "driver.cmake"
    driver.write_text(_DRIVER, encoding="ascii")
    env = os.environ.copy()
    if path is not None:
        env["PATH"] = path
    return subprocess.run(
        [
            CMAKE,
            f"-DBUILD_COMMON={BUILD_COMMON}",
            f"-DROOT={root}",
            "-P",
            str(driver),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )


def test_symbols_and_compiler_comments_are_removed_from_both_elf_states(tmp_path: Path):
    unstripped, stripped = _make_libs(tmp_path)
    assert _has_symtab(unstripped)
    assert not _has_symtab(stripped)
    assert _has_compiler_comment(unstripped)
    assert _has_compiler_comment(stripped)

    result = _run_strip(unstripped.parent)
    assert result.returncode == 0, result.stderr
    assert "Stripping 2 libraries (out of 2 ELF libraries)" in result.stdout
    for library in (unstripped, stripped):
        assert not _has_symtab(library)
        assert not _has_compiler_comment(library)

    before = [library.read_bytes() for library in (unstripped, stripped)]
    result = _run_strip(unstripped.parent)
    assert result.returncode == 0, result.stderr
    assert "All ELF libraries are already stripped (2 checked)" in result.stdout
    assert [library.read_bytes() for library in (unstripped, stripped)] == before


def test_missing_readelf_fails_the_strip_step(tmp_path: Path):
    unstripped, _stripped = _make_libs(tmp_path)
    empty_bin = tmp_path / "empty-bin"
    empty_bin.mkdir()
    # uname is how script-mode CMake learns the host arch. readelf stays absent.
    os.symlink(shutil.which("uname") or "/usr/bin/uname", empty_bin / "uname")

    result = _run_strip(unstripped.parent, path=str(empty_bin))
    combined = result.stdout + result.stderr
    assert result.returncode != 0
    assert "readelf not found" in combined
    assert "apt-get install file" not in combined
    assert _has_symtab(unstripped)

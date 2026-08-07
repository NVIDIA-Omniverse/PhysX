#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""
Verify that the shipped namespaced ovphysx package is py-less and USD-isolated.

The ovphysx wheel must work across Python 3.x minor versions, so the native
artifacts inside _install/ or the wheel staging tree must not contain Python
runtime pieces or classic OpenUSD libraries. Namespaced ovphysx ships the
isolated monolithic USD library (libov_<ver>usd_ms); pulling in classic modular
USD (libusd_tf, libusd_sdf, usd_tf.dll, etc.) would break the split.

  - Bundled libpython (pins the wheel to one minor version)
  - libusd_python (the Python-flavored USD slice)
  - Python schema binding modules (_physxSchema)
  - Classic USD shared libraries (libusd_*.so*, usd_*.dll)
  - Core Carbonite (libcarb.so, libcarb.so.*, carb.dll) -- ovphysx is static-carb,
    so carb is linked into libovphysx.so and must not ship separately
  - Build-tree Python paths baked into RPATH/RUNPATH
  - ELF DT_NEEDED / SONAME entries or Windows PE imports that still depend on
    Python, classic USD, or core Carbonite

This script scans a directory tree and exits non-zero if any violations are
found. It is meant to be called from install.cmake and build_wheel.cmake as
a hard gate, and also from Python tests for fast offline policy checks.

Usage:
    python scripts/verify_pyless_closure.py --dir _install
    python scripts/verify_pyless_closure.py --dir _build/python_wheel_staging/ovphysx
"""

import argparse
import fnmatch
import platform
import re
import struct
import subprocess
import sys
from pathlib import Path

# Shared library files whose presence in the shipped package is forbidden.
FORBIDDEN_FILENAMES = [
    "libpython3*.so*",
    "libpython3.so",
    "python3*.dll",
    "python3.dll",
    "libusd_python.so",
    "usd_python.dll",
    "libusd_*.so*",
    "usd_*.dll",
    "_physxSchema.*",
    # Physics-owned runtime plugin artifacts have been folded into static
    # ovphysx/ovruntime links. The former engine-agnostic tensor plugin was
    # deleted too; TensorApi now lives in the static PhysX runtime.
    "libomni.physx.plugin.so",
    "omni.physx.plugin.dll",
    # Negative guard: the Fabric plugin is deleted, but keep its names forbidden. Packaging copies
    # shared libs from incremental ovruntime output, so a stale build artifact could otherwise
    # re-enter the closure -- deleting the producer does not scrub prior build outputs.
    "libomni.physx.fabric.plugin.so",
    "omni.physx.fabric.plugin.dll",
    "libomni.physics.tensors.plugin.so",
    "omni.physics.tensors.plugin.dll",
    "libomni.physx.tensors.plugin.so",
    "omni.physx.tensors.plugin.dll",
    "libomni.physx.cooking.plugin.so",
    "omni.physx.cooking.plugin.dll",
    "libomni.physx.foundation.plugin.so",
    "omni.physx.foundation.plugin.dll",
    # Core Carbonite must never ship: ovphysx is static-carb, carb is
    # linked into libovphysx.so. A loose core libcarb would reintroduce the
    # duplicate-libcarb SONAME clash (IsaacLab LD_PRELOAD/OVPHYSX_CARB hack).
    # The per-plugin no-libcarb shims (libcarb.<name>.plugin.so /
    # carb.<name>.plugin.dll) are allowed; only the bare core lib is not.
    "libcarb.so",
    "libcarb.so.*",
    "carb.dll",
]

# Substrings in RUNPATH/RPATH entries that indicate stale build-tree paths.
FORBIDDEN_RPATH_SUBSTRINGS = [
    "target-deps/python",
]

# ELF DT_NEEDED entries that must not appear in a py-less namespaced package.
FORBIDDEN_DT_NEEDED = [
    "libpython3*.so*",
    "libpython3.so",
    "libusd_python.so*",
    "libusd_*.so*",
    # No shipped binary may import core Carbonite (static-carb invariant).
    "libcarb.so",
    "libcarb.so.*",
]

# ELF DT_SONAME entries that must not appear in a py-less namespaced package.
FORBIDDEN_ELF_SONAME = [
    # No shipped binary may advertise itself as core Carbonite.
    "libcarb.so",
    "libcarb.so.*",
]

# Windows import table entries that must not appear in a py-less namespaced
# package. Checked on Windows CI; filename checks above catch bundled DLLs on
# any host.
FORBIDDEN_PE_IMPORTS = [
    "python3*.dll",
    "python3.dll",
    "usd_python.dll",
    "usd_*.dll",
    # No shipped binary may import core Carbonite (static-carb invariant).
    "carb.dll",
]


def _is_shared_lib(name: str) -> bool:
    return name.endswith((".dll", ".pyd", ".so")) or (".so." in name)


def _is_elf_shared_lib(name: str) -> bool:
    return name.endswith(".so") or (".so." in name)


def _matches_any(name: str, patterns: list[str]) -> bool:
    return any(fnmatch.fnmatch(name, p) for p in patterns)


def _relative_path(root: Path, path: Path) -> str:
    return path.relative_to(root).as_posix()


def _check_forbidden_files(root: Path) -> list[str]:
    """Return list of forbidden files found under root."""
    violations = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            if _is_shared_lib(path.name) and _matches_any(path.name, FORBIDDEN_FILENAMES):
                violations.append(f"forbidden file: {_relative_path(root, path)}")
            continue
        if not path.is_file():
            continue
        if _is_shared_lib(path.name) and _matches_any(path.name, FORBIDDEN_FILENAMES):
            violations.append(f"forbidden file: {_relative_path(root, path)}")
    return violations


def _check_elf_dynamic_linux(root: Path) -> list[str]:
    """On Linux, scan ELF dynamic entries for forbidden RPATH, imports, and SONAME.

    Fail closed: if readelf is missing or cannot inspect an ELF, report that as
    a violation instead of silently skipping the file.
    """
    if platform.system() != "Linux":
        return []
    violations = []
    needed_pattern = re.compile(r"\(NEEDED\).*\[([^\]]+)\]")
    soname_pattern = re.compile(r"\(SONAME\).*\[([^\]]+)\]")
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.is_symlink():
            continue
        if not _is_elf_shared_lib(path.name):
            continue
        try:
            result = subprocess.run(
                ["readelf", "-d", str(path)],
                capture_output=True, text=True, timeout=10,
            )
        except FileNotFoundError:
            return ["readelf not found on Linux; cannot verify ELF dynamic entries"]
        except subprocess.TimeoutExpired:
            rel = _relative_path(root, path)
            violations.append(f"readelf timed out while inspecting {rel}")
            continue
        rel = _relative_path(root, path)
        if result.returncode != 0:
            stderr = (result.stderr or "").strip() or "<no stderr>"
            violations.append(f"readelf failed for {rel}: {stderr}")
            continue
        for line in result.stdout.splitlines():
            if "RUNPATH" in line or "RPATH" in line:
                for forbidden in FORBIDDEN_RPATH_SUBSTRINGS:
                    if forbidden in line:
                        violations.append(
                            f"forbidden RPATH in {rel}: contains '{forbidden}'"
                        )
            match = needed_pattern.search(line)
            if match:
                needed_name = match.group(1)
                for pattern in FORBIDDEN_DT_NEEDED:
                    if fnmatch.fnmatch(needed_name, pattern):
                        violations.append(
                            f"forbidden DT_NEEDED in {rel}: {needed_name}"
                        )
            match = soname_pattern.search(line)
            if match:
                soname_name = match.group(1)
                for pattern in FORBIDDEN_ELF_SONAME:
                    if fnmatch.fnmatch(soname_name, pattern):
                        violations.append(
                            f"forbidden SONAME in {rel}: {soname_name}"
                        )
    return violations


def read_pe_imports(path: Path) -> list[str]:
    """Return imported DLL names from a PE file using a minimal stdlib parser.

    Reads just enough of the PE structure to walk the Import Directory; avoids
    requiring `pefile` or `dumpbin` so the verifier is usable from any CI job
    that has a Python runtime. PE32 and PE32+ are both supported.
    """
    data = path.read_bytes()

    if len(data) < 0x40 or data[:2] != b"MZ":
        raise RuntimeError(f"{path}: not a PE file (bad DOS signature)")

    e_lfanew = struct.unpack_from("<I", data, 0x3C)[0]
    if e_lfanew + 24 > len(data) or data[e_lfanew:e_lfanew + 4] != b"PE\x00\x00":
        raise RuntimeError(f"{path}: missing PE signature")

    coff_off = e_lfanew + 4
    num_sections = struct.unpack_from("<H", data, coff_off + 2)[0]
    size_of_opt_header = struct.unpack_from("<H", data, coff_off + 16)[0]
    opt_off = coff_off + 20
    if opt_off + 2 > len(data):
        raise RuntimeError(f"{path}: truncated optional header")

    magic = struct.unpack_from("<H", data, opt_off)[0]
    if magic == 0x10B:  # PE32
        data_dirs_off = opt_off + 96
    elif magic == 0x20B:  # PE32+
        data_dirs_off = opt_off + 112
    else:
        raise RuntimeError(f"{path}: unknown optional header magic 0x{magic:x}")

    # Data directory index 1 is the Import Table.
    import_rva, import_size = struct.unpack_from("<II", data, data_dirs_off + 8)
    if import_rva == 0 or import_size == 0:
        return []

    section_table_off = opt_off + size_of_opt_header
    sections: list[tuple[int, int, int, int]] = []  # (vaddr, vsize, raw_off, raw_size)
    for i in range(num_sections):
        s_off = section_table_off + i * 40
        if s_off + 40 > len(data):
            raise RuntimeError(f"{path}: truncated section header")
        vsize, vaddr, raw_size, raw_off = struct.unpack_from("<IIII", data, s_off + 8)
        sections.append((vaddr, vsize, raw_off, raw_size))

    def _rva_to_file_offset(rva: int) -> int:
        for vaddr, vsize, raw_off, raw_size in sections:
            if vaddr <= rva < vaddr + max(vsize, raw_size):
                return raw_off + (rva - vaddr)
        raise RuntimeError(f"{path}: cannot map RVA 0x{rva:x} to file offset")

    imports: list[str] = []
    cursor = _rva_to_file_offset(import_rva)
    while True:
        if cursor + 20 > len(data):
            raise RuntimeError(f"{path}: truncated import descriptor")
        descriptor = data[cursor:cursor + 20]
        if descriptor == b"\x00" * 20:
            break
        name_rva = struct.unpack_from("<I", descriptor, 12)[0]
        if name_rva == 0:
            break
        name_off = _rva_to_file_offset(name_rva)
        end = data.find(b"\x00", name_off)
        if end < 0:
            raise RuntimeError(f"{path}: unterminated import name")
        imports.append(data[name_off:end].decode("ascii", errors="replace"))
        cursor += 20
    return imports


def _check_pe_imports_windows(root: Path) -> list[str]:
    """On Windows, scan PE import tables for forbidden runtime DLLs."""
    if platform.system() != "Windows":
        return []

    violations = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.is_symlink():
            continue
        if not path.name.endswith((".dll", ".pyd")):
            continue
        try:
            imports = read_pe_imports(path)
        except RuntimeError as exc:
            rel = _relative_path(root, path)
            violations.append(f"PE import scan failed for {rel}: {exc}")
            continue
        rel = _relative_path(root, path)
        for imported_name in imports:
            for pattern in FORBIDDEN_PE_IMPORTS:
                if fnmatch.fnmatch(imported_name.lower(), pattern):
                    violations.append(
                        f"forbidden PE import in {rel}: {imported_name}"
                    )
    return violations


def verify(directory: Path) -> list[str]:
    """Run all namespaced package isolation checks and return violations."""
    violations = []
    violations.extend(_check_forbidden_files(directory))
    violations.extend(_check_elf_dynamic_linux(directory))
    violations.extend(_check_pe_imports_windows(directory))
    return violations


def main():
    parser = argparse.ArgumentParser(
        description="Verify py-less, namespaced ovphysx package contents"
    )
    parser.add_argument("--dir", type=Path, required=True,
                        help="Root directory to scan (_install or wheel staging)")
    args = parser.parse_args()

    if not args.dir.is_dir():
        print(f"[ERROR] Directory not found: {args.dir}", file=sys.stderr)
        sys.exit(1)

    violations = verify(args.dir)
    if violations:
        print(f"\n[FAIL] {len(violations)} package isolation violation(s) in {args.dir}:\n")
        for v in violations:
            print(f"  - {v}")
        print()
        sys.exit(1)
    else:
        print(f"[PASS] py-less namespaced package contents verified: {args.dir}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-USDFREE-001
# @covers AC-2 AC-4 AC-7
# @implements REQ-PACKAGING-OMNICLIENT-001
# @covers AC-1 AC-2
# @implements REQ-PACKAGING-CLOSURE-001
# @covers AC-1 AC-3

"""
Verify that the shipped ovphysx package is py-less and USD-free.

The ovphysx wheel must work across Python 3.x minor versions and ships no
OpenUSD at all: ovstage brings its own namespaced USD runtime and the
application owns any USD it authors with. The native artifacts inside _install/
or the wheel staging tree must therefore not contain:

  - Bundled libpython (pins the wheel to one minor version)
  - Any OpenUSD library: the namespaced monolith (libov_<ver>usd_ms), classic
    modular USD (libusd_tf, libusd_sdf, usd_tf.dll, ...), libusd_python, or the
    Omniverse USD resolver
  - USD's dependency closure (TBB, MaterialX, Alembic, Imath, OpenSubdiv, draco,
    hdStorm). It belongs to the USD runtime ovstage ships
  - A USD plugin registry directory (plugins/usd)
  - Python schema binding modules (_physxSchema)
  - Core Carbonite (libcarb.so, libcarb.so.*, carb.dll). ovphysx is static-carb,
    so carb is linked into libovphysx.so and must not ship separately
  - Retired Fabric-era Cubric and Carbonite GPU-compute plugins
  - OmniClient and its connection library, which belong to OVStage
  - Build-tree Python paths baked into RPATH/RUNPATH
  - ELF DT_NEEDED / SONAME entries or Windows PE imports that still depend on
    Python, OpenUSD, or core Carbonite

With --require-schemas the tree must also carry the codeless PhysX USD schemas
(schemas/physx/plugInfo.json plus at least one <Module>/resources/plugInfo.json),
which ovphysx ships as data for the application to register.

This script scans a directory tree and exits non-zero if any violations are
found. It is meant to be called from install.cmake and build_wheel.cmake as
a hard gate, and also from Python tests for fast offline policy checks.

Usage:
    python scripts/verify_pyless_closure.py --dir _install --require-schemas
    python scripts/verify_pyless_closure.py --dir _build/python_wheel_staging/ovphysx --require-schemas
"""

import argparse
import fnmatch
import json
import platform
import re
import struct
import subprocess
import sys
from pathlib import Path

# Core and physics-facing OpenUSD modules as a stock (non-monolithic) build
# names them: lib<module>.so on Linux, <module>.dll on Windows.
UPSTREAM_USD_MODULES = [
    "arch", "tf", "js", "gf", "trace", "work", "plug", "vt", "ar", "kind", "sdf",
    "pcp", "ndr", "sdr", "usd", "usdGeom", "usdShade", "usdLux", "usdPhysics",
    "usdSkel", "usdUtils", "hf", "hd", "hio", "glf", "garch", "hgi", "usdImaging",
]

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
    # The namespaced OpenUSD monolith and the Omniverse USD resolver ship with
    # ovstage only. A second copy here would register USD's process-wide
    # singletons twice. ovphysx neither loads nor links them.
    "libov_*usd_ms.so*",
    "ov_*usd_ms.dll",
    "libomni_usd_resolver.so*",
    "omni_usd_resolver.dll",
    # The application-supplied OVStage package owns asset loading. ovphysx has
    # no OmniClient API and must neither bundle nor directly link this pair.
    "libomniclient.so*",
    "omniclient.dll",
    "libomniverse_connection.so*",
    "omniverse_connection.dll",
    "ovstage-omniclient.version",
    # USD's own dependency closure. These libraries are the monolith's leaf
    # dependencies and belong to the runtime ovstage ships.
    "libtbb*.so*",
    "tbb*.dll",
    "libMaterialX*.so*",
    "MaterialX*.dll",
    "libAlembic*.so*",
    "Alembic*.dll",
    "libImath*.so*",
    "Imath*.dll",
    "libosd*.so*",
    "osd*.dll",
    "libdraco*.so*",
    "draco*.dll",
    "libhdStorm*.so*",
    "hdStorm*.dll",
    "_physxSchema.*",
    # Physics-owned runtime plugins are linked statically into ovphysx/ovruntime,
    # and TensorApi lives in the static PhysX runtime. None of them ship as a
    # separate plugin.
    "libomni.physx.plugin.so",
    "omni.physx.plugin.dll",
    # Negative guard: the Fabric plugin no longer exists, but its names stay forbidden. Packaging
    # copies shared libs from incremental ovruntime output, so a stale build artifact could
    # otherwise re-enter the closure.
    "libomni.physx.fabric.plugin.so",
    "omni.physx.fabric.plugin.dll",
    "libomni.cubric.plugin.so",
    "omni.cubric.plugin.dll",
    "libomni.gpucompute-cuda.plugin.so",
    "omni.gpucompute-cuda.plugin.dll",
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
    # duplicate-libcarb SONAME clash (IsaacLab LD_PRELOAD/OVPHYSX_CARB workaround).
    # The per-plugin no-libcarb shims (libcarb.<name>.plugin.so /
    # carb.<name>.plugin.dll) are allowed. Only the bare core lib is not.
    "libcarb.so",
    "libcarb.so.*",
    "carb.dll",
] + [
    # Upstream modular OpenUSD library names (libtf.so, usdGeom.dll, ...): the
    # NVIDIA builds above use a libusd_<module> prefix, a stock build does not.
    pattern
    for module in UPSTREAM_USD_MODULES
    for pattern in (f"lib{module}.so*", f"{module}.dll")
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
    # No shipped ovphysx binary may import OpenUSD directly. Only libovstage,
    # which ovphysx does not ship, links the namespaced monolith.
    "libov_*usd_ms.so*",
    "libomniclient.so*",
    "libomniverse_connection.so*",
    # No shipped binary may import core Carbonite (static-carb invariant).
    "libcarb.so",
    "libcarb.so.*",
] + [
    f"lib{module}.so*" for module in UPSTREAM_USD_MODULES
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
    # No shipped ovphysx binary may import OpenUSD directly (see FORBIDDEN_DT_NEEDED).
    "ov_*usd_ms.dll",
    "omni_usd_resolver.dll",
    "omniclient.dll",
    "omniverse_connection.dll",
    # No shipped binary may import core Carbonite (static-carb invariant).
    "carb.dll",
] + [
    # Import names are compared lowercased (see _check_pe_imports_windows).
    f"{module.lower()}.dll" for module in UPSTREAM_USD_MODULES
]


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
            if _matches_any(path.name, FORBIDDEN_FILENAMES):
                violations.append(f"forbidden file: {_relative_path(root, path)}")
            continue
        if not path.is_file():
            continue
        if _matches_any(path.name, FORBIDDEN_FILENAMES):
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

    Reads just enough of the PE structure to walk the Import Directory and avoids
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


def _check_no_usd_plugin_registry(root: Path) -> list[str]:
    """The USD plugin registry (plugins/usd) belonged to ovphysx's own USD runtime.

    ovphysx no longer has one, so any plugInfo.json under plugins/ is a stale or
    misrouted USD payload. The codeless schemas live under schemas/physx instead.
    """
    plugins_usd = root / "plugins" / "usd"
    if plugins_usd.exists():
        return [f"USD plugin registry shipped: {_relative_path(root, plugins_usd)} (schemas belong in schemas/physx)"]
    return []


def _check_no_usd_runtime_config(root: Path) -> list[str]:
    """config.toml carried ovphysx's USD version and schema configuration.

    ovphysx no longer has a USD runtime to configure, so the file must not ship
    in any layout.
    """
    return [
        f"USD runtime configuration shipped: {_relative_path(root, path)}"
        for path in sorted(root.rglob("config.toml"))
        if path.is_file()
    ]


def _load_plug_info(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _check_codeless_schemas(root: Path) -> list[str]:
    """The codeless PhysX schema tree must be registrable as shipped: a root
    plugInfo.json that includes the module resources, and at least one
    <Module>/resources/ holding a resource-type plugInfo.json with declared
    schema types and a generatedSchema.usda."""
    schema_root = root / "schemas" / "physx"
    rel_root = _relative_path(root, schema_root)
    violations = []
    root_plug_info = schema_root / "plugInfo.json"
    if not root_plug_info.is_file():
        violations.append(f"missing codeless schema root registry: {rel_root}/plugInfo.json")
    else:
        root_data = _load_plug_info(root_plug_info)
        includes = root_data.get("Includes") if isinstance(root_data, dict) else None
        if not isinstance(includes, list) or "*/resources/" not in includes:
            violations.append(
                f"codeless schema root registry does not include the module resources: "
                f'{rel_root}/plugInfo.json (expected "Includes": ["*/resources/"])'
            )
    modules = sorted(schema_root.glob("*/resources/plugInfo.json")) if schema_root.is_dir() else []
    if not modules:
        violations.append(f"no codeless schema module under {rel_root} (expected <Module>/resources/plugInfo.json)")
    for plug_info in modules:
        rel_module = _relative_path(root, plug_info.parent)
        if not (plug_info.parent / "generatedSchema.usda").is_file():
            violations.append(f"codeless schema module without generatedSchema.usda: {rel_module}")
        data = _load_plug_info(plug_info)
        plugins = data.get("Plugins") if isinstance(data, dict) else None
        if not isinstance(plugins, list) or not plugins:
            violations.append(f"codeless schema module without a Plugins entry: {rel_module}/plugInfo.json")
            continue
        for plugin in plugins:
            if not isinstance(plugin, dict):
                violations.append(f"codeless schema module with a malformed Plugins entry: {rel_module}/plugInfo.json")
                continue
            if plugin.get("Type") != "resource":
                violations.append(
                    f"codeless schema module is not a resource plugin: {rel_module}/plugInfo.json "
                    f"(Type {plugin.get('Type')!r})"
                )
            if "LibraryPath" in plugin:
                violations.append(f"codeless schema module names a LibraryPath: {rel_module}/plugInfo.json")
            if not (plugin.get("Info") or {}).get("Types"):
                violations.append(f"codeless schema module declares no schema types: {rel_module}/plugInfo.json")
    return violations


def verify(directory: Path, require_schemas: bool = False) -> list[str]:
    """Run all py-less, USD-free package checks and return violations."""
    violations = []
    violations.extend(_check_forbidden_files(directory))
    violations.extend(_check_no_usd_plugin_registry(directory))
    violations.extend(_check_no_usd_runtime_config(directory))
    violations.extend(_check_elf_dynamic_linux(directory))
    violations.extend(_check_pe_imports_windows(directory))
    if require_schemas:
        violations.extend(_check_codeless_schemas(directory))
    return violations


def main():
    parser = argparse.ArgumentParser(
        description="Verify py-less, USD-free ovphysx package contents"
    )
    parser.add_argument("--dir", type=Path, required=True,
                        help="Root directory to scan (_install or wheel staging)")
    parser.add_argument("--require-schemas", action="store_true",
                        help="Also require the codeless PhysX schema tree under schemas/physx")
    args = parser.parse_args()

    if not args.dir.is_dir():
        print(f"[ERROR] Directory not found: {args.dir}", file=sys.stderr)
        sys.exit(1)

    violations = verify(args.dir, require_schemas=args.require_schemas)
    if violations:
        print(f"\n[FAIL] {len(violations)} package isolation violation(s) in {args.dir}:\n")
        for v in violations:
            print(f"  - {v}")
        print()
        sys.exit(1)
    else:
        print(f"[PASS] py-less, USD-free package contents verified: {args.dir}")


if __name__ == "__main__":
    main()

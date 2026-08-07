#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""
Package dependencies into FLAT plugins/ structure for ovphysx SDK.

Sources (kitless -- no extsPhysics/exts trees):
  - ovruntime _install/<cfg>/bin/     : PhysX runtime .plugin.so + PhysXGpu
  - target-deps/usd/<config>/lib/     : USD core shared libraries
  - carb_sdk_static/_build/<cfg>/      : no-libcarb Carbonite plugin shims
  - ovruntime_deps_<config>/.../plugins/ : datastore, UJITSO, blobkey, Cubric, GPU compute, and USD support
  - usd_ext_physics/                  : codeless PhysX schema (data, headers, Python; config-neutral)
  - exact OVSTAGE_DIR/bin/plugins/    : matched resolver, registry, OmniClient,
                                        and omniverse_connection runtime set

Creates:
  _install/
      +-- lib/
      |   +-- libovphysx.so
      +-- plugins/           <- ALL .so files here (flat!)
          +-- libomni.physx.*.so
          +-- usd/           <- USD plugInfo.json registry
          +-- bin/deps/      <- (currently unused, reserved)

Usage:
    python scripts/package_deps.py --build-dir=_build --install-dir=_install \
        --ovruntime-install-dir=<path> --ovstage-dir=<resolved-root> \
        --ovstage-runtime-dir=<resolved-runtime> [--verbose]
"""

import argparse
import ctypes
import fnmatch
import json
import os
import platform
import shutil
import sys
import zipfile
from pathlib import Path

try:
    import tomllib
except ImportError:
    try:
        import tomli as tomllib
    except ImportError:
        print("[ERROR] No TOML parser available. Install tomli: pip install tomli", file=sys.stderr)
        sys.exit(1)


PLATFORM_LINUX = os.name != "nt"
SO_EXT = ".dll" if not PLATFORM_LINUX else ".so"


def _is_shared_lib(name: str) -> bool:
    return name.endswith(SO_EXT) or (f"{SO_EXT}." in name)


def _matches_any(name: str, patterns: list[str]) -> bool:
    return any(fnmatch.fnmatch(name, p) for p in patterns)


def load_manifest(manifest_path: Path) -> dict:
    if not manifest_path.exists():
        print(f"[Error] Manifest not found: {manifest_path}")
        return {}
    with open(manifest_path, "rb") as f:
        return tomllib.load(f)


def _normalize_json_like(source: str) -> str:
    """Normalize USD plugInfo JSON extensions outside quoted strings.

    USD plugInfo files are parsed by USD's permissive JSON reader and may
    contain `#`, `//`, or `/* ... */` comments plus trailing commas. Python's
    `json` module does not accept those extensions, so packaging normalizes the
    file just enough to inspect and rewrite `LibraryPath` entries.
    """
    result: list[str] = []
    in_string = False
    escaped = False
    pending_comma = False
    pending_whitespace: list[str] = []
    i = 0

    def flush_pending_comma() -> None:
        nonlocal pending_comma, pending_whitespace
        if pending_comma:
            result.append(",")
            result.extend(pending_whitespace)
            pending_comma = False
            pending_whitespace = []

    while i < len(source):
        ch = source[i]
        nxt = source[i + 1] if i + 1 < len(source) else ""
        if in_string:
            result.append(ch)
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                in_string = False
            i += 1
            continue
        if pending_comma:
            if ch in " \t\r\n":
                pending_whitespace.append(ch)
                i += 1
                continue
            if ch == "#" or (ch == "/" and nxt == "/"):
                while i < len(source) and source[i] not in "\r\n":
                    i += 1
                continue
            if ch == "/" and nxt == "*":
                end = source.find("*/", i + 2)
                if end < 0:
                    break
                i = end + 2
                continue
            if ch in "}]":
                result.extend(pending_whitespace)
                pending_comma = False
                pending_whitespace = []
            else:
                flush_pending_comma()
        if ch == '"':
            flush_pending_comma()
            in_string = True
            result.append(ch)
            i += 1
            continue
        if ch == "#" or (ch == "/" and nxt == "/"):
            while i < len(source) and source[i] not in "\r\n":
                i += 1
            continue
        if ch == "/" and nxt == "*":
            end = source.find("*/", i + 2)
            if end < 0:
                break
            i = end + 2
            continue
        if ch == ",":
            pending_comma = True
            pending_whitespace = []
            i += 1
            continue
        result.append(ch)
        i += 1
    flush_pending_comma()
    return "".join(result)


def _load_json_like(path: Path) -> dict | None:
    """Read and normalize a USD plugInfo file. Returns None if the file is
    missing or unparseable.

    Note: an unterminated `/* ... */` block comment causes _normalize_json_like
    to break out of its state machine early (see the `if end < 0: break`
    branches), and any trailing comma immediately before the unterminated
    comment is preserved rather than stripped. json.loads then raises
    JSONDecodeError, which we swallow here -- so a malformed plugInfo file
    silently disappears from packaging without a diagnostic. Callers that need
    to distinguish "absent" from "malformed" should inspect the file directly.
    """
    try:
        text = path.read_text()
    except OSError:
        return None

    normalized = _normalize_json_like(text)
    try:
        return json.loads(normalized)
    except json.JSONDecodeError:
        return None


def _preferred_platform_build() -> str:
    if not PLATFORM_LINUX:
        machine = platform.machine().lower()
        if machine in ("arm64", "aarch64"):
            return "windows-arm64"
        return "windows-x86_64"

    machine = platform.machine().lower()
    if machine in ("arm64", "aarch64"):
        return "linux-aarch64"
    return "linux-x86_64"


def _resolve_platform_build(build_root: Path) -> str:
    preferred = _preferred_platform_build()
    if (build_root / preferred).exists():
        return preferred

    if build_root.exists():
        candidates = sorted(
            child.name
            for child in build_root.iterdir()
            if child.is_dir() and child.name.startswith(("linux-", "windows-"))
        )
        if len(candidates) == 1:
            return candidates[0]

    return preferred


def _resolve_ovstage_plugins_dir(ovstage_runtime_dir: Path) -> Path:
    """Validate the plugin tree beside CMake's exact resolved OVStage runtime."""
    plugins_dir = ovstage_runtime_dir / "plugins"
    if not (plugins_dir / "omni.client.lib").is_dir() or not (plugins_dir / "omni.usd_resolver").is_dir():
        raise RuntimeError(
            f"OVStage Release runtime plugin tree is missing beside the configured runtime: " f"{plugins_dir}"
        )
    return plugins_dir


def _copy_required_files(source_dir: Path, dest_dir: Path, names: list[str], label: str) -> int:
    missing = [name for name in names if not (source_dir / name).is_file()]
    if missing:
        raise RuntimeError(f"{label} payload is incomplete under {source_dir}; missing: {', '.join(missing)}")

    dest_dir.mkdir(parents=True, exist_ok=True)
    for name in names:
        shutil.copy2(source_dir / name, dest_dir / name)
    return len(names)


def _read_omniclient_version(client_library: Path) -> str:
    """Read the provider's exported version for runtime provenance checks."""
    try:
        if PLATFORM_LINUX:
            library = ctypes.CDLL(str(client_library))
        else:
            with os.add_dll_directory(str(client_library.parent)):
                library = ctypes.WinDLL(str(client_library))
        version_fn = library.omniClientGetVersionString
        version_fn.argtypes = []
        version_fn.restype = ctypes.c_char_p
        version_bytes = version_fn()
    except (AttributeError, OSError) as exc:
        raise RuntimeError(
            f"Cannot read omniClientGetVersionString from OVStage library {client_library}: {exc}"
        ) from exc

    if not version_bytes:
        raise RuntimeError(f"OVStage library returned an empty OmniClient version: {client_library}")
    return version_bytes.decode("utf-8", errors="strict")


def merge_ovstage_notices(
    ovstage_dir: Path,
    ovstage_runtime_dir: Path,
    license_archive: Path,
    install_dir: Path,
    license_archive_required: bool = True,
) -> list[str]:
    """Preserve the exact OVStage-provided notice payload in ovphysx packages.

    New releases publish one complete ``THIRD-PARTY-NOTICES.txt``. Older
    releases publish a text index plus the complete ``rendering-LICENSES.zip``.
    Replace any previous ``ovstage/`` archive entries so rerunning installation
    is deterministic and never accumulates duplicate zip members.
    """
    payloads: list[tuple[Path, str]] = []
    visible_notice: Path | None = None
    notice_roots = []
    if ovstage_runtime_dir.name == "bin":
        notice_roots.append(ovstage_runtime_dir.parent)
    notice_roots.append(ovstage_dir)

    for notice_root in dict.fromkeys(notice_roots):
        root_notice = notice_root / "THIRD-PARTY-NOTICES.txt"
        if root_notice.is_file():
            visible_notice = root_notice
            payloads.append((root_notice, "ovstage/THIRD-PARTY-NOTICES.txt"))
            break

        for legacy_dir in (
            notice_root / "PACKAGE-LICENSES",
            notice_root / "_build" / "PACKAGE-LICENSES",
        ):
            legacy_text = legacy_dir / "rendering-LICENSES.txt"
            legacy_zip = legacy_dir / "rendering-LICENSES.zip"
            if legacy_text.is_file() and legacy_zip.is_file():
                visible_notice = legacy_text
                payloads.extend(
                    [
                        (legacy_text, "ovstage/rendering-LICENSES.txt"),
                        (legacy_zip, "ovstage/rendering-LICENSES.zip"),
                    ]
                )
                break
        if payloads:
            break

    if not payloads or visible_notice is None or not visible_notice.is_file():
        raise RuntimeError(
            f"OVStage license provenance is missing under {ovstage_dir}; expected "
            "THIRD-PARTY-NOTICES.txt or PACKAGE-LICENSES/rendering-LICENSES.{txt,zip}"
        )
    empty = [str(source) for source, _ in payloads if source.stat().st_size == 0]
    if empty:
        raise RuntimeError(f"OVStage notice payload contains empty files: {', '.join(empty)}")
    visible_dest = install_dir / "ovstage-THIRD-PARTY-NOTICES.txt"
    shutil.copy2(visible_notice, visible_dest)

    if not license_archive.is_file():
        if license_archive_required:
            raise RuntimeError(f"ovphysx license archive not found: {license_archive}")
        # Public source drop: license gathering is skipped (repo_licensing is
        # not shipped), so there is no archive to merge the notices into. The
        # visible OVStage notice file above is still installed.
        print("  [note] ovphysx-LICENSES.zip not present (license gathering skipped); OVStage notices not merged")
        return [visible_dest.name]

    temp_archive = license_archive.with_name(f".{license_archive.name}.tmp")
    if temp_archive.exists():
        temp_archive.unlink()
    try:
        with (
            zipfile.ZipFile(license_archive, "r") as source_zip,
            zipfile.ZipFile(temp_archive, "w", compression=zipfile.ZIP_DEFLATED) as dest_zip,
        ):
            for info in source_zip.infolist():
                if not info.filename.startswith("ovstage/"):
                    dest_zip.writestr(info, source_zip.read(info.filename))
            for source, archive_name in payloads:
                dest_zip.write(source, archive_name)
        os.replace(temp_archive, license_archive)
    finally:
        if temp_archive.exists():
            temp_archive.unlink()

    return [archive_name for _, archive_name in payloads]


def _load_schema_verifier():
    try:
        from verify_schema_package import verify_schema_package

        return verify_schema_package
    except ImportError:
        # Allow importing package_deps.py from contexts where the scripts dir
        # is not on sys.path by loading the verifier directly.
        import importlib.util

        script_dir = Path(__file__).resolve().parent
        spec = importlib.util.spec_from_file_location("verify_schema_package", script_dir / "verify_schema_package.py")
        if spec is None or spec.loader is None:  # pragma: no cover - defensive
            raise RuntimeError("Could not load verify_schema_package.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.verify_schema_package


def _verify_codeless_schema_package(schema_dir: Path) -> list[str]:
    verify_schema_package = _load_schema_verifier()
    return verify_schema_package(schema_dir)


def _select_schema_package_dir(
    omni_physics_dir: Path,
    omni_physics_tdeps: Path,
    prefer_local_schema: bool = False,
) -> Path:
    """Choose the codeless schema package tree for ovphysx packaging.

    Defaults to the prebuilt codeless usd_ext_physics package pulled by
    ovruntime. --devschema is an explicit developer override that swaps in a
    local schemas/physx build; either way the chosen tree is verified as a
    valid codeless schema before being accepted, failing fast otherwise.
    """
    # omni_physics_dir is a symlink to ovruntime; resolve to get the real ovphysx/ parent.
    ovruntime_real = omni_physics_dir.resolve()

    if prefer_local_schema:
        # Devschema path: accept a local schema tree only when it verifies as a
        # valid codeless schema. schemas/physx is codeless: build.sh stages a
        # single flat, config- and platform-neutral tree at _build/schema.
        local_schema_base = ovruntime_real.parent.parent / "schemas" / "physx" / "_build"
        local_schema_candidates: list[Path] = [local_schema_base / "schema"]

        for candidate in local_schema_candidates:
            if not candidate.exists():
                continue
            errors = _verify_codeless_schema_package(candidate)
            if not errors:
                print(f"  Using local (devschema) schema: {candidate}")
                return candidate
        searched = ", ".join(str(p) for p in local_schema_candidates) or str(local_schema_base)
        raise RuntimeError(
            "A local schema build was explicitly requested (--devschema), but no valid codeless local schema tree "
            f"was found. Checked: {searched}. Rebuild schemas/physx or retry without --devschema."
        )

    packman_schema_dir = omni_physics_tdeps / "usd_ext_physics"
    if not packman_schema_dir.exists():
        raise RuntimeError(
            f"Schema package not found at {packman_schema_dir}. Re-run "
            "ovruntime/pull_dependencies, "
            "or rerun packaging with --devschema if you intentionally want a local schema build."
        )

    # This is the second line of defense after build.cmake's post-pull check and
    # protects direct `cmake -P scripts/install.cmake` invocations too.
    errors = _verify_codeless_schema_package(packman_schema_dir)
    if errors:
        joined = "\n  - ".join(errors)
        raise RuntimeError(f"Schema package at {packman_schema_dir} is not a valid codeless schema:\n  - {joined}")

    print(f"  Using packman schema (codeless): {packman_schema_dir}")
    return packman_schema_dir


# ---------------------------------------------------------------------------
# Copy helpers
# ---------------------------------------------------------------------------


def _copy_shared_libs(
    source_dir: Path,
    dest_dir: Path,
    skip_patterns: list[str] | None = None,
    include_patterns: list[str] | None = None,
    verbose: bool = False,
    label: str = "",
) -> int:
    """Copy all .so/.dll files from *source_dir* (non-recursive) into *dest_dir*.

    When *include_patterns* is provided, only files whose name matches at
    least one pattern are copied. This is used to stage the schema package
    through a strict allowlist so future package growth cannot silently
    ship extra payload. *skip_patterns* is always applied on top as a
    defense-in-depth denylist (see deps_manifest.toml [packaging]).

    Preserves full versioned symlink chains
    (e.g. libFoo.so -> libFoo.so.1 -> libFoo.so.1.2.3).
    Returns count of entries created.
    """
    if not source_dir.exists():
        if verbose:
            print(f"  [skip] {label}: source not found: {source_dir}")
        return 0

    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = 0

    # Separate real files from symlinks so we can copy real files first,
    # then recreate symlinks pointing at their immediate targets.
    regular_files: list[Path] = []
    symlinks: list[Path] = []

    for entry in sorted(source_dir.iterdir()):
        if entry.is_dir():
            continue
        name = entry.name
        if not _is_shared_lib(name):
            continue
        if include_patterns and not _matches_any(name, include_patterns):
            if verbose:
                print(f"  [skip] {name} (not in allowlist)")
            continue
        if skip_patterns and _matches_any(name, skip_patterns):
            if verbose:
                print(f"  [skip] {name} (packaging skip)")
            continue
        if PLATFORM_LINUX and os.path.islink(entry):
            symlinks.append(entry)
        else:
            regular_files.append(entry)

    # Pass 1: copy real files
    for entry in regular_files:
        dest_file = dest_dir / entry.name
        if dest_file.exists() or os.path.lexists(dest_file):
            continue
        shutil.copy2(entry, dest_file)
        copied += 1
        if verbose:
            print(f"  [copy] {entry.name} <- {label}")

    # Pass 2: recreate symlinks (immediate targets already copied above
    # or will be created as symlinks themselves in this pass)
    for entry in symlinks:
        dest_file = dest_dir / entry.name
        if dest_file.exists() or os.path.lexists(dest_file):
            continue
        link_target = os.readlink(entry)
        target_name = Path(link_target).name
        # Ensure the ultimate real file is present in dest
        target_src = (entry.parent / link_target).resolve()
        real_name = target_src.name
        real_dest = dest_dir / real_name
        if target_src.exists() and target_src.is_file() and not real_dest.exists():
            shutil.copy2(target_src, real_dest)
            copied += 1
            if verbose:
                print(f"  [copy] {real_name} <- {label}")
        if not os.path.lexists(dest_file):
            try:
                os.symlink(target_name, dest_file)
                copied += 1
            except FileExistsError:
                pass
            if verbose:
                print(f"  [link] {entry.name} -> {target_name}")

    return copied


def _copy_plugin_libs(
    source_dir: Path,
    dest_dir: Path,
    skip_patterns: list[str] | None = None,
    verbose: bool = False,
    label: str = "",
) -> int:
    """Copy all shared libraries from *source_dir* (recursive) into *dest_dir*.

    Copies every .so/.dll found under the directory tree.  The caller is
    responsible for scoping *source_dir* to a curated subdirectory (e.g. a
    specific plugin like ``carb.datastore``) so that unrelated support
    libraries (gstreamer, vulkan, slang) from sibling directories are not
    pulled in.

    Carbonite plugins (*.plugin.so / *.plugin.dll) and their companion
    runtime shared libraries both need to be present for DLL loading to
    succeed on Windows, where there is no RTLD_GLOBAL equivalent.

    Preserves versioned symlinks (e.g. libFoo.so -> libFoo.so.1.2.3).
    """
    if not source_dir.exists():
        return 0
    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = 0

    # Collect all shared-lib entries (files + symlinks) first so we can
    # handle symlinks after their targets have been copied.
    regular_files: list[Path] = []
    symlinks: list[Path] = []

    for root, _, files in os.walk(source_dir, followlinks=False):
        for fname in sorted(files):
            entry = Path(root) / fname
            if not _is_shared_lib(fname):
                continue
            if skip_patterns and _matches_any(fname, skip_patterns):
                if verbose:
                    print(f"  [skip] {fname} (packaging skip)")
                continue
            if PLATFORM_LINUX and os.path.islink(entry):
                symlinks.append(entry)
            else:
                regular_files.append(entry)

    # Copy regular files first
    for entry in regular_files:
        dest_file = dest_dir / entry.name
        if dest_file.exists() or os.path.lexists(dest_file):
            continue
        shutil.copy2(entry, dest_file)
        copied += 1
        if verbose:
            print(f"  [copy] {entry.name} <- {label}")

    # Recreate symlinks, copying their ultimate target if not yet present
    for entry in symlinks:
        dest_file = dest_dir / entry.name
        if dest_file.exists() or os.path.lexists(dest_file):
            continue
        link_target = os.readlink(entry)
        target_name = Path(link_target).name
        target_src = (entry.parent / link_target).resolve()
        target_dest = dest_dir / target_name
        if target_src.exists() and target_src.is_file() and not target_dest.exists():
            shutil.copy2(target_src, target_dest)
            copied += 1
            if verbose:
                print(f"  [copy] {target_name} <- {label}")
        if not os.path.lexists(dest_file):
            try:
                os.symlink(target_name, dest_file)
                copied += 1
            except FileExistsError:
                pass
            if verbose:
                print(f"  [link] {entry.name} -> {target_name}")

    return copied


# ---------------------------------------------------------------------------
# Carbonite bootstrap plugins
# ---------------------------------------------------------------------------


def copy_carbonite_plugins(
    source_dir: Path,
    plugins_dest: Path,
    plugin_names: list[str],
    verbose: bool = False,
) -> int:
    if not source_dir.exists():
        raise RuntimeError(f"Static Carbonite plugin shim directory not found: {source_dir}")
    plugins_dest.mkdir(parents=True, exist_ok=True)
    prefix = "" if not PLATFORM_LINUX else "lib"
    suffix = ".dll" if not PLATFORM_LINUX else ".so"
    copied = 0
    missing_plugins = []
    for base_name in plugin_names:
        plugin_file = f"{prefix}{base_name}{suffix}"
        src_path = source_dir / plugin_file
        if src_path.exists():
            dest_path = plugins_dest / plugin_file
            if not dest_path.exists():
                shutil.copy2(src_path, dest_path)
                copied += 1
                if verbose:
                    print(f"  [copy] {plugin_file}")
        else:
            if verbose:
                print(f"  [missing] {plugin_file}")
            missing_plugins.append(plugin_file)
    if missing_plugins:
        raise RuntimeError(
            "Missing required Carbonite bootstrap plugin shims from " f"{source_dir}: {', '.join(missing_plugins)}"
        )
    return copied


# ---------------------------------------------------------------------------
# USD plugInfo.json registry
# ---------------------------------------------------------------------------


def copy_usd_registry(
    usd_lib_dir: Path,
    usd_plugin_dir: Path,
    schema_share_dir: Path | None,
    plugins_dest: Path,
    verbose: bool = False,
) -> int:
    """Merge USD plugInfo.json registry trees into plugins/usd/.

    Sources:
      - usd_lib_dir/usd/     (core USD modules -- ar, sdf, usd, usdGeom, ...)
      - usd_plugin_dir/usd/  (USD plugin modules -- hdStorm, sdrGlslfx, usdAbc, ...)
      - schema_share_dir/     (PhysxSchema, OmniUsdPhysicsDeformableSchema, ...)
    """
    usd_dest = plugins_dest / "usd"
    if usd_dest.exists():
        shutil.rmtree(usd_dest)
    usd_dest.mkdir(parents=True, exist_ok=True)

    def _merge_tree(src: Path, label: str) -> int:
        if not src.exists():
            return 0
        count = 0
        for root, _, files in os.walk(src, followlinks=True):
            for fname in files:
                src_path = Path(root) / fname
                if src_path.suffix in (".so", ".dll"):
                    continue
                rel = src_path.relative_to(src)
                dst_path = usd_dest / rel
                dst_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src_path, dst_path)
                if fname == "plugInfo.json":
                    count += 1
        if verbose and count:
            print(f"  [merge] {count} plugInfo.json from {label}")
        return count

    total = 0
    usd_lib_usd = usd_lib_dir / "usd"
    if usd_lib_usd.exists():
        total += _merge_tree(usd_lib_usd, "usd/release/lib/usd")

    usd_plugin_usd = usd_plugin_dir / "usd"
    if usd_plugin_usd.exists():
        total += _merge_tree(usd_plugin_usd, "usd/release/plugin/usd")

    if schema_share_dir and schema_share_dir.exists():
        total += _merge_tree(schema_share_dir, "usd_ext_physics/share/usd/plugins")

    fixed = fixup_usd_plugin_info_library_paths(plugins_dest, verbose)
    if fixed and verbose:
        print(f"  [fixup] Rewrote LibraryPath in {fixed} plugInfo.json file(s)")

    return total


def fixup_usd_plugin_info_library_paths(plugins_dest: Path, verbose: bool = False) -> int:
    """Rewrite LibraryPath entries in plugins_dest/usd/**/plugInfo.json to point
    at the flat plugins/ layout when the original relative path does not resolve.
    """
    usd_dest = plugins_dest / "usd"
    if not usd_dest.exists():
        return 0
    fixed = 0
    for plug_info in usd_dest.rglob("plugInfo.json"):
        data = _load_json_like(plug_info)
        if data is None:
            continue

        changed = False
        for plugin in data.get("Plugins", []):
            lib_path = plugin.get("LibraryPath", "")
            if not lib_path:
                continue

            # USD resolves LibraryPath relative to Root (default ".."),
            # not relative to the plugInfo.json file itself.
            root_rel = plugin.get("Root", "..")
            root_dir = (plug_info.parent / root_rel).resolve()

            resolved = (root_dir / lib_path).resolve()
            if resolved.exists():
                continue

            flat_candidate = plugins_dest / Path(lib_path).name
            if not flat_candidate.exists():
                print(f"  [warn] {plug_info.relative_to(usd_dest)}: " f"{lib_path} -> unresolved (no flat candidate)")
                continue

            new_path = os.path.relpath(flat_candidate, root_dir)
            plugin["LibraryPath"] = new_path
            changed = True
            if verbose:
                print(f"  [fixup] {plug_info.relative_to(usd_dest)}: " f"{lib_path} -> {new_path}")

        if changed:
            plug_info.write_text(json.dumps(data, indent=4) + "\n")
            fixed += 1
    return fixed


# ---------------------------------------------------------------------------
# Main packaging entry point
# ---------------------------------------------------------------------------


def package_deps(
    manifest_path: Path,
    build_dir: Path,
    install_dir: Path,
    ovruntime_install_dir: Path,
    ovstage_dir: Path,
    ovstage_runtime_dir: Path,
    config: str = "release",
    devschema: bool = False,
    verbose: bool = False,
    license_archive_required: bool = True,
) -> bool:
    print("\n" + "=" * 70)
    print("Packaging Dependencies (FLAT plugins/ structure)")
    print("=" * 70)

    manifest = load_manifest(manifest_path)
    if not manifest:
        return False

    print(f"[Config] Manifest: {manifest_path}")
    print(f"[Config] Build dir: {build_dir}")
    print(f"[Config] Install dir: {install_dir}")
    print(f"[Config] ovruntime install: {ovruntime_install_dir}")
    print(f"[Config] OVStage root: {ovstage_dir}")
    print(f"[Config] OVStage runtime: {ovstage_runtime_dir}")
    print("[Config] USD mode: namespaced")
    print("[Config] Static Carbonite: ON")
    print(f"[Config] Devschema override: {'ON' if devschema else 'OFF'}")

    plugins_dest = install_dir / "plugins"
    if plugins_dest.exists():
        print(f"\n[Clean] Removing existing plugins: {plugins_dest}")
        shutil.rmtree(plugins_dest)
    plugins_dest.mkdir(parents=True, exist_ok=True)

    target_deps = build_dir / "target-deps"
    ovstage_plugins_dir = _resolve_ovstage_plugins_dir(ovstage_runtime_dir)
    print(f"[Config] OVStage Release plugins: {ovstage_plugins_dir}")
    skip_lib_patterns = list(manifest.get("packaging", {}).get("skip_shared_libs", []))
    skip_lib_patterns.extend(["libcarb.so", "libcarb.so.*", "carb.dll", "carb.lib", "carb.pdb"])

    # omni_physics symlink -> ovruntime; used to reach infrastructure plugins
    # and usd_ext_physics.
    omni_physics_dir = target_deps / "omni_physics"
    omni_physics_tdeps = omni_physics_dir / "_build" / "target-deps"
    # infra_pkg / infra_plugins_dir are config-scoped: some plugin binaries
    # (e.g. libomni.tbb.globalcontrol.plugin.so) encode config in their
    # DT_NEEDED (release needs libtbb.so.12, debug needs libtbb_debug.so.12),
    # so copying the wrong variant leaves a dangling libtbb SONAME.
    infra_pkg = f"ovruntime_deps_{config}"
    infra_label = "ovruntime_deps"
    platform_build = _resolve_platform_build(omni_physics_tdeps / infra_pkg / "_build")
    infra_plugins_dir = omni_physics_tdeps / infra_pkg / "_build" / platform_build / config / "plugins"
    usd_ext_physics_dir = _select_schema_package_dir(
        omni_physics_dir,
        omni_physics_tdeps,
        prefer_local_schema=devschema,
    )

    # ------------------------------------------------------------------
    # libcarb is statically linked into ovphysx. The remaining Carbonite
    # payload comes from the no-libcarb shims in carb_sdk_static.
    print("\nSkipping core library (libcarb) -- statically linked into ovphysx")

    print("\nCopying carbonite bootstrap plugins...")
    carb_plugins = manifest.get("carbonite_plugins", {}).get("include", [])
    # These Carbonite plugins are already linked into libovphysx; do not ship
    # their no-libcarb shim libraries as separate runtime plugins.
    statically_linked_plugins = {
        "carb.assets.plugin",
        "carb.datasource-file.plugin",
        "carb.dictionary.plugin",
        "carb.dictionary.serializer-json.plugin",
        "carb.dictionary.serializer-toml.plugin",
        "carb.eventdispatcher.plugin",
        "carb.events.plugin",
        "carb.settings.plugin",
        "carb.tasking.plugin",
        "carb.tokens.plugin",
        "carb.variant.plugin",
    }
    carb_plugins = [p for p in carb_plugins if p not in statically_linked_plugins]
    carb_static_build_root = target_deps / "carb_sdk_static" / "_build"
    carbonite_plugins_src = carb_static_build_root / _resolve_platform_build(carb_static_build_root) / config
    if not carbonite_plugins_src.exists() and config != "release":
        release_src = carb_static_build_root / _resolve_platform_build(carb_static_build_root) / "release"
        if release_src.exists():
            print(f"  (static Carbonite: {carbonite_plugins_src} not found, using {release_src})")
            carbonite_plugins_src = release_src
    print(f"  (static Carbonite: sourcing no-libcarb plugin shims from {carbonite_plugins_src})")
    print(f"  (static Carbonite: filtered {len(statically_linked_plugins)} statically-linked plugins)")
    count = copy_carbonite_plugins(carbonite_plugins_src, plugins_dest, carb_plugins, verbose)
    print(f"  [OK] Copied {count} carbonite plugins")

    # ------------------------------------------------------------------
    # PhysX runtime plugins from ovruntime install
    # ------------------------------------------------------------------
    print("\nCopying PhysX runtime plugins from ovruntime install...")
    # In CMake subproject mode, .so files land directly in the output dir (e.g. _build/release/).
    # In standalone mode, they're under <install>/bin/. Accept both layouts.
    ovr_bin = ovruntime_install_dir / "bin"
    if not ovr_bin.exists():
        ovr_bin = ovruntime_install_dir
    if not ovr_bin.exists():
        print(f"[Error] ovruntime install dir not found: {ovruntime_install_dir}")
        return False
    count = _copy_shared_libs(
        ovr_bin, plugins_dest, skip_patterns=skip_lib_patterns, verbose=verbose, label="ovruntime install"
    )
    print(f"  [OK] Copied {count} physics plugin/binding files")

    # ------------------------------------------------------------------
    # USD core libs: prefer ovruntime's namespaced USD over ovphysx's own
    # kit-sdk-imported copy.
    # ------------------------------------------------------------------
    print("\nCopying USD core libs (full, unfiltered)...")
    ovruntime_usd_dir = omni_physics_dir / "_build" / "target-deps" / "usd" / config / "lib"
    local_usd_dir = target_deps / "usd" / config / "lib"
    if ovruntime_usd_dir.exists():
        usd_lib_dir = ovruntime_usd_dir
        print(f"  Using ovruntime USD: {usd_lib_dir}")
    else:
        usd_lib_dir = local_usd_dir
        print(f"  Using local USD: {usd_lib_dir}")
    count = _copy_shared_libs(
        usd_lib_dir, plugins_dest, skip_patterns=skip_lib_patterns, verbose=verbose, label=f"usd/{config}/lib"
    )
    usd_base_dir = usd_lib_dir.parent  # .../usd/release
    usd_bin_dir = usd_base_dir / "bin"
    count += _copy_shared_libs(
        usd_bin_dir, plugins_dest, skip_patterns=skip_lib_patterns, verbose=verbose, label=f"usd/{config}/bin"
    )
    # USD plugin .so files (sdrGlslfx, usdAbc, usdDraco, usdShaders, hdStorm, etc.)
    usd_plugin_dir = usd_base_dir / "plugin" / "usd"
    if usd_plugin_dir.exists():
        for so_file in sorted(usd_plugin_dir.glob("*.so")):
            if so_file.is_file():
                dest_file = plugins_dest / so_file.name
                if not dest_file.exists():
                    shutil.copy2(so_file, dest_file)
                    count += 1
                    if verbose:
                        print(f"  [copy] {so_file.name} <- usd/{config}/plugin/usd")
        for dll_file in sorted(usd_plugin_dir.glob("*.dll")):
            if dll_file.is_file():
                dest_file = plugins_dest / dll_file.name
                if not dest_file.exists():
                    shutil.copy2(dll_file, dest_file)
                    count += 1
    print(f"  [OK] Copied {count} USD lib files")

    # ------------------------------------------------------------------
    # Infrastructure plugins
    # ------------------------------------------------------------------
    # usd_ext_physics is codeless -- no native schema libs to stage here. Its USD
    # plugin data (share/) is copied later by copy_usd_registry; ovphysx is
    # py-less and does not ship the schema's Python.
    print(f"\nCopying {infra_label} plugins...")

    # Infrastructure subdirectories still owned by ovphysx. Fabric, USDRT,
    # and scenegraph are intentionally absent: ovstage ships and loads its own
    # matching population runtime from its module-relative plugin tree.
    infra_subdirs_to_copy = [
        "carb.datastore",
        "omni.blobkey",
        "carb.ujitsoagent",
        "carb.ujitso.default",
        "omni.cubric",
        "gpucompute",
        "omni.usd",
    ]
    total_infra = 0
    found_infra_subdirs = 0
    for subdir_name in infra_subdirs_to_copy:
        subdir_path = infra_plugins_dir / subdir_name
        if not subdir_path.exists():
            if verbose:
                print(f"  [skip] {infra_label}/{subdir_name}: not found")
            continue
        found_infra_subdirs += 1
        c = _copy_plugin_libs(
            subdir_path,
            plugins_dest,
            skip_patterns=skip_lib_patterns,
            verbose=verbose,
            label=f"{infra_label}/{subdir_name}",
        )
        total_infra += c
        if verbose:
            print(f"  [OK] {infra_label}/{subdir_name}: {c}")
    if found_infra_subdirs == 0:
        raise RuntimeError(
            f"{infra_label} infrastructure plugin payload is empty or missing under {infra_plugins_dir}. "
            "Re-run ovruntime/pull_dependencies and verify the expected package was pulled."
        )
    print(f"  [OK] {infra_label} plugins: {total_infra}")

    # Keep the resolver, OmniClient, and omniverse_connection as one matched
    # OVStage-provided Release runtime set. The flat copies remain canonical for
    # PhysX-first startup; OVStage-first reuses the already-loaded SONAME.
    client_lib_dir = ovstage_plugins_dir / "omni.client.lib"
    client_lib_names = (
        ["libomniclient.so", "libomniverse_connection.so"]
        if PLATFORM_LINUX
        else ["omniclient.dll", "omniverse_connection.dll"]
    )
    c = _copy_required_files(client_lib_dir, plugins_dest, client_lib_names, "OVStage OmniClient")
    client_version = _read_omniclient_version(client_lib_dir / client_lib_names[0])
    (plugins_dest / "ovstage-omniclient.version").write_bytes((client_version + "\n").encode("utf-8"))
    print(f"  [OK] OVStage OmniClient runtime: {c}")
    print(f"  [OK] OVStage OmniClient version: {client_version}")

    # The resolver is NOT staged: it registers the OMNI_USD_RESOLVER TF debug symbol,
    # and a second copy beside the application's OVStage aborts the process. Its
    # plugInfo goes with it (an orphaned registry fails the plugin load). Same drop
    # build_wheel.cmake makes. Still validated, so a mismatched OVStage is caught.
    resolver_lib_dir = ovstage_plugins_dir / "omni.usd_resolver"
    resolver_lib_name = "libomni_usd_resolver.so" if PLATFORM_LINUX else "omni_usd_resolver.dll"
    if not (resolver_lib_dir / resolver_lib_name).is_file():
        raise RuntimeError(f"OVStage USD resolver payload is incomplete: missing {resolver_lib_dir / resolver_lib_name}")
    resolver_plug_info = resolver_lib_dir / "usd" / "omni_usd_resolver" / "resources" / "plugInfo.json"
    if not resolver_plug_info.is_file():
        raise RuntimeError(f"OVStage USD resolver registry is missing: {resolver_plug_info}")
    print("  [OK] OVStage USD resolver: validated, not staged (provided by the application's OVStage)")

    # ------------------------------------------------------------------
    # USD plugInfo.json registry
    # ------------------------------------------------------------------
    print("\nCopying USD plugInfo registry...")
    usd_plugin_base_dir = usd_base_dir / "plugin"
    schema_share_dir = usd_ext_physics_dir / "share" / "usd" / "plugins"
    count = copy_usd_registry(usd_lib_dir, usd_plugin_base_dir, schema_share_dir, plugins_dest, verbose)
    print(f"  [OK] Merged {count} plugInfo.json files")

    # Sweep an orphaned resolver registry left by a previous install.
    stale_resolver_registry = plugins_dest / "usd" / "omni_usd_resolver"
    if stale_resolver_registry.exists():
        shutil.rmtree(stale_resolver_registry)
        print("  [OK] Removed stale OVStage resolver registry from a previous install")
    fixup_usd_plugin_info_library_paths(plugins_dest, verbose)

    license_archive = build_dir / "PACKAGE-LICENSES" / "ovphysx-LICENSES.zip"
    notice_entries = merge_ovstage_notices(
        ovstage_dir, ovstage_runtime_dir, license_archive, install_dir, license_archive_required
    )
    print(f"  [OK] Preserved OVStage notices: {', '.join(notice_entries)}")

    print("\nSkipping Python runtime libs for namespaced py-less packaging...")

    # ------------------------------------------------------------------
    # Ensure all shared libraries have execute permission
    # ------------------------------------------------------------------
    if PLATFORM_LINUX:
        for f in plugins_dest.rglob("*"):
            if f.is_file() and _is_shared_lib(f.name) and not f.is_symlink():
                f.chmod(0o755)

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    total_files = sum(1 for _ in plugins_dest.rglob("*") if _.is_file())
    total_size = sum(f.stat().st_size for f in plugins_dest.rglob("*") if f.is_file())

    print("\n" + "=" * 70)
    print("[SUCCESS] Flat plugins/ structure created!")
    print("=" * 70)
    print(f"  Location: {plugins_dest}")
    print(f"  Files: {total_files}")
    print(f"  Size: {total_size / (1024**3):.2f} GB")
    print("=" * 70 + "\n")

    return True


def main():
    parser = argparse.ArgumentParser(description="Package dependencies into flat plugins/ structure")

    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path("_build"),
        help="Build directory containing target-deps (used as input only)",
    )
    parser.add_argument(
        "--install-dir",
        type=Path,
        default=Path("_install"),
        help="Installation directory",
    )
    parser.add_argument(
        "--ovruntime-install-dir",
        type=Path,
        default=None,
        help="Path to ovruntime install output (e.g. ovruntime/_install/ovruntime/release)",
    )
    parser.add_argument(
        "--ovstage-dir",
        type=Path,
        required=True,
        help="Exact OVStage root resolved by the configured build",
    )
    parser.add_argument(
        "--ovstage-runtime-dir",
        type=Path,
        required=True,
        help="Exact OVStage runtime directory resolved by the configured build",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="Path to deps_manifest.toml",
    )
    parser.add_argument(
        "--config",
        default="release",
        help="Build configuration name for packman deps (default: release)",
    )
    parser.add_argument(
        "--devschema",
        action="store_true",
        help="Use a local schemas/physx build as the schema source instead of the packman package.",
    )
    parser.add_argument(
        "--no-license-archive",
        action="store_true",
        help="Tolerate a missing ovphysx-LICENSES.zip (public source drop: "
        "repo_licensing is not shipped, so license gathering is skipped).",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Verbose output",
    )
    args = parser.parse_args()

    if args.manifest:
        manifest_path = args.manifest
    else:
        script_dir = Path(__file__).parent
        manifest_path = script_dir.parent / "deps_manifest.toml"

    ovruntime_install_dir = args.ovruntime_install_dir
    if ovruntime_install_dir is None:
        # Default: subproject output in _build/<config>/
        ovruntime_install_dir = Path(__file__).resolve().parent.parent / "_build" / args.config
        print(f"[Info] --ovruntime-install-dir not specified, defaulting to: {ovruntime_install_dir}")

    success = package_deps(
        manifest_path=manifest_path,
        build_dir=args.build_dir,
        install_dir=args.install_dir,
        ovruntime_install_dir=ovruntime_install_dir,
        ovstage_dir=args.ovstage_dir,
        ovstage_runtime_dir=args.ovstage_runtime_dir,
        config=args.config,
        devschema=args.devschema,
        verbose=args.verbose,
        license_archive_required=not args.no_license_archive,
    )
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

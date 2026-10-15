#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fetch pip packages defined in a TOML dependency file.

Standalone script that processes pip dependency TOML files (the same format
used by repo_build's ``fetch.pip.files_to_pull``).  Can be invoked from
pull_dependencies scripts or from build.py without requiring the full
repo_man / repo_build / packman toolchain.

Usage:
    python pip_fetch.py <toml_file> [--python <dir>] [--target-deps <dir>]

The optional ``--python`` override replaces the ``python`` field in every
``[[dependency]]`` block (useful when the calling project provides its own
Python).  Similarly ``--target-deps`` replaces the target directory prefix.
An optional ``constraint_file`` resolves relative to the TOML file. Locked
installs are reused only while the dependency settings and lock are unchanged.
"""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import logging
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    try:
        import tomli as tomllib  # type: ignore[no-redef]
    except ModuleNotFoundError:
        tomllib = None  # type: ignore[assignment]

logger = logging.getLogger("pip_fetch")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _get_platform_target() -> str:
    """Return a platform target string like ``linux-x86_64`` or ``windows-x86_64``."""
    machine = platform.machine().lower()
    if machine in ("amd64", "x86_64"):
        machine = "x86_64"
    elif machine in ("arm64", "aarch64"):
        machine = "aarch64"
    if sys.platform == "win32":
        return f"windows-{machine}"
    elif sys.platform == "darwin":
        return f"macos-{machine}"
    else:
        return f"linux-{machine}"


def _resolve_path(config_file: str, path: str) -> str:
    """Resolve *path* relative to the directory containing *config_file*."""
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(os.path.dirname(config_file), path))


def _find_python_exe(python_dir: str) -> str | None:
    """Locate a Python executable inside *python_dir*."""
    candidates = (
        ["python.exe", "python3.exe", "Scripts/python.exe"]
        if sys.platform == "win32"
        else ["python", "python3", "bin/python3", "bin/python"]
    )
    for c in candidates:
        p = os.path.join(python_dir, c)
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


def _install_locked(cmd: list[str], target_dir: str, fingerprint: str) -> bool:
    """Publish a complete locked install; preserve the previous cache on failure."""
    target = Path(target_dir).absolute()
    stamp = target / ".pip-fetch-lock"
    try:
        if target.is_symlink() or target.resolve() != target.parent.resolve() / target.name:
            raise OSError(f"Locked pip target must not be a link: {target}")
        if stamp.is_file() and stamp.read_text() == fingerprint:
            logger.info("Locked target is current: %s", target)
            return True

        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".pip-", dir=target.parent) as scratch:
            staged = Path(scratch) / "package"
            result = subprocess.run([*cmd, f"--target={staged}"])
            if result.returncode != 0:
                logger.error("Locked pip install failed (exit %d): %s", result.returncode, target)
                return False
            (staged / stamp.name).write_text(fingerprint)
            previous = Path(scratch) / "previous"
            if target.exists():
                if target.resolve() != target.parent.resolve() / target.name:
                    raise OSError(f"Locked pip target escapes its parent: {target}")
                target.rename(previous)
            try:
                staged.rename(target)
            except OSError:
                if previous.exists():
                    previous.rename(target)
                raise
        return True
    except OSError as error:
        logger.error("Cannot provision locked pip target %s: %s", target, error)
        return False


# ---------------------------------------------------------------------------
# Core fetch logic
# ---------------------------------------------------------------------------


def fetch(
    config_file: str,
    platform_target: str | None = None,
    python_override: str | None = None,
    target_deps_override: str | None = None,
) -> bool:
    """Process *config_file* and pip-install every matching ``[[dependency]]``.

    Returns ``True`` if all installs succeeded (or were skipped), ``False``
    on any failure.
    """
    config_file = os.path.abspath(config_file)
    if not os.path.isfile(config_file):
        logger.error("Config file not found: %s", config_file)
        return False

    if tomllib is None:
        logger.error("No TOML parser available (need Python 3.11+ or 'tomli' package)")
        return False

    with open(config_file, "rb") as f:
        config = tomllib.load(f)

    if platform_target is None:
        platform_target = _get_platform_target()

    ok = True
    for dep in config.get("dependency", []):
        # Platform filter
        platforms = dep.get("platforms", ["*"])
        if not any(fnmatch.fnmatch(platform_target, p) for p in platforms):
            logger.info("Skipping dependency (platform %s not in %s)", platform_target, platforms)
            continue

        # Resolve python executable
        if python_override:
            python_dir = python_override
        else:
            python_dir = _resolve_path(config_file, dep.get("python", ""))

        python_exe = _find_python_exe(python_dir)
        if not python_exe:
            logger.warning("Python executable not found in %s — skipping", python_dir)
            continue

        # Resolve target directory
        raw_target = dep["target"]
        if target_deps_override:
            # Replace only the leaf directory name under the override base
            target_dir = os.path.join(target_deps_override, os.path.basename(raw_target))
        else:
            target_dir = _resolve_path(config_file, raw_target)

        packages = dep.get("packages", [])
        if not packages:
            continue

        append = dep.get("append_to_install_folder", False)
        download_only = dep.get("download_only", False)
        constraint_file = dep.get("constraint_file")
        fingerprint = None
        if constraint_file:
            if append or download_only:
                logger.error("Locked dependencies require a separate install target")
                ok = False
                continue
            constraint_file = _resolve_path(config_file, constraint_file)
            try:
                lock = Path(constraint_file).read_bytes()
            except OSError as error:
                logger.error("Cannot read pip lock %s: %s", constraint_file, error)
                ok = False
                continue
            inputs = json.dumps([dep, platform_target, python_exe], sort_keys=True).encode()
            fingerprint = hashlib.sha256(inputs + b"\0" + lock).hexdigest()
        elif os.path.isdir(target_dir) and not append:
            logger.info("Target already exists, skipping: %s", target_dir)
            continue

        # Build pip command — mirrors repo_build pippull.py _build_install_cmd()
        install_deps = dep.get("install_dependencies", True)
        extra_args = dep.get("extra_args", [])

        cmd = [python_exe, "-m", "pip"]
        if download_only:
            cmd.extend(["download", f"--destination-directory={target_dir}"])
        else:
            cmd.append("install")
            if not constraint_file:
                cmd.append(f"--target={target_dir}")

        cmd.extend(["--disable-pip-version-check", "--no-input", "--timeout=60", "--retries=5"])

        if not install_deps:
            cmd.append("--no-deps")

        cmd.extend(extra_args)
        if constraint_file:
            cmd.extend(["--constraint", constraint_file])
        cmd.extend(packages)

        pkg_summary = ", ".join(packages[:3])
        if len(packages) > 3:
            pkg_summary += f" ... ({len(packages)} total)"
        logger.info("Installing into '%s': %s", os.path.basename(target_dir), pkg_summary)

        if fingerprint is not None:
            if not _install_locked(cmd, target_dir, fingerprint):
                ok = False
            continue

        result = subprocess.run(cmd)
        if result.returncode != 0:
            logger.error("pip install failed (exit %d) for target %s", result.returncode, target_dir)
            ok = False

    return ok


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description="Fetch pip packages from a TOML dependency file.")
    parser.add_argument("config", help="Path to the pip dependency TOML file")
    parser.add_argument("--python", default=None, help="Override Python directory for all dependencies")
    parser.add_argument("--target-deps", default=None, help="Override target-deps base directory")
    parser.add_argument("--platform", default=None, help="Platform target (e.g. linux-x86_64)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    ok = fetch(
        args.config, platform_target=args.platform, python_override=args.python, target_deps_override=args.target_deps
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

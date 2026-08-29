# SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#

import contextlib
import json
import logging
import os
import platform
import re
import sys
from functools import lru_cache
from pathlib import Path

import packmanapi

logger = logging.getLogger(__name__)

if sys.version_info < (3, 10):  # noqa: UP036 - Keep version check for user-facing warning
    logger.warning("This version of repo_man currently requires Python 3.10 or later.")

REPO_ROOT = os.path.join(os.path.dirname(os.path.normpath(__file__)), "../..")
REPO_CACHE_FILE = os.path.join(REPO_ROOT, "repo-cache.json")


def repoman_bootstrap():
    _path_checks()
    _prep_cache_paths()
    _pull_optional_deps()
    _check_ai_skills_stale()


def _check_ai_skills_stale(repo_root: str | None = None):
    """Set ``_REPO_AI_SKILLS_STALE=1`` if installed AI skills look out of date.

    Compares the mtime of the resolved main and optional packman dep files
    against the AI skills manifest. If either deps file is newer (e.g. the
    user ran ``repo update``, switched branches, or hand-edited a deps file)
    the env var is set so ``omni.repo.man.entry`` can refresh skills after
    the tool finishes. Side-effect-only by design: returns nothing because
    the contract with ``entry.py`` is the env var.

    Lives in the bootstrapper rather than in ``repoman.py`` so that customer
    repos auto-pick-up new staleness logic via ``repo update`` (the update
    pipeline refreshes the bootstrapper but does not rewrite ``repoman.py``).

    Hygiene: any inherited ``_REPO_AI_SKILLS_STALE`` from a parent process
    is cleared before deciding. This prevents a stale value from a previous
    ``repo`` invocation in the same shell (or from an outer build wrapper)
    from triggering an unintended refresh in this one. We then make our own
    independent decision based on the local mtime check.

    Docker / container relaunch: when ``LINBUILD_EMBEDDED`` is set, we are
    running inside a re-launched build container -- the outer process will
    still run its own bootstrap and post-tool refresh after we exit, and
    that outer process operates on the same mounted repo. Skipping the
    check here avoids two refreshes touching identical files, which is
    purely wasteful in an already-slow flow.

    Cost: 1 ``os.path.exists`` + 1-2 ``isfile`` + 1-3 ``stat`` calls. No
    network, no imports beyond stdlib. Returns silently if the manifest
    doesn't exist (skills never installed) or if any check raises.
    """
    # Always start from a clean slate; never carry an inherited value forward.
    os.environ.pop("_REPO_AI_SKILLS_STALE", None)

    if os.environ.get("LINBUILD_EMBEDDED"):
        # Running inside a re-launched build container; outer process handles refresh.
        return

    if repo_root is None:
        repo_root = REPO_ROOT

    manifest_path = os.path.join(repo_root, ".repo-ai-manifest.json")
    if not os.path.exists(manifest_path):
        return  # Skills never installed -- no staleness to flag.

    try:
        manifest_mtime = os.path.getmtime(manifest_path)
        main_deps, opt_deps = _find_deps_files(repo_root)
        stale = (main_deps is not None and os.path.getmtime(main_deps) > manifest_mtime) or (
            opt_deps is not None and os.path.getmtime(opt_deps) > manifest_mtime
        )
        # Also flag stale when the manifest references files that no longer
        # exist on disk -- e.g. a fresh clone where the consumer commits the
        # manifest but gitignores the generated skill files, or any case where
        # a user has deleted a skill file by hand. One stat per manifest entry,
        # short-circuits on first miss, no network. Reading the JSON is the
        # only extra cost vs the previous mtime-only check.
        if not stale:
            try:
                with open(manifest_path, encoding="utf-8") as mf:
                    manifest = json.load(mf)
                for entry in manifest.get("files", []):
                    rel = entry.get("path")
                    if rel and not os.path.exists(os.path.join(repo_root, rel)):
                        stale = True
                        break
            except Exception as e:
                logger.debug(f"AI skills file-presence check failed (non-fatal): {e}")
        if stale:
            os.environ["_REPO_AI_SKILLS_STALE"] = "1"
    except Exception as e:
        logger.debug(f"AI skills staleness check failed (non-fatal): {e}")


def _pull_optional_deps():
    """
    Pull optional dependencies if repo-deps-<suffix> exists as determined by _opt_deps_suffix()
    """
    _, opt_deps_file = _find_deps_files(REPO_ROOT)
    if opt_deps_file is None:
        return
    deps = None
    with contextlib.suppress(packmanapi.PackmanErrorFileNotFound):
        deps = packmanapi.pull(opt_deps_file)
        for dep_path in deps.values():
            if dep_path not in sys.path:
                sys.path.append(dep_path)
    if deps is None:
        logger.debug(
            f"Failed to pull optional dependencies in {opt_deps_file}. This can be normal depending on configuration and context.",
        )


def _path_checks():
    """Check for problematic path conditions and warn appropriately."""
    cwd = os.getcwd()
    if " " in cwd:
        logger.warning(
            "Current working directory: %s contains whitespace which may cause issues with some tooling such as premake within repo_build. It is recommended to move your project to a path without spaces.",
            cwd,
        )

    # Check if current working directory is within a OneDrive folder
    if platform.system() == "Windows":
        onedrive_path = os.getenv("OneDrive")  # For personal OneDrive
        onedrive_business_path = os.getenv("OneDriveCommercial")  # For business accounts

        if not onedrive_path and not onedrive_business_path:
            # OneDrive is not installed or synced
            return

        if (onedrive_path and cwd.startswith(onedrive_path)) or (
            onedrive_business_path and cwd.startswith(onedrive_business_path)
        ):
            logger.warning(
                "Current working directory: %s appears to be within a OneDrive folder. This may cause filesystem issues with Packman linking dependencies. It is recommended to move your project outside of OneDrive.",
                cwd,
            )


def _prep_cache_paths():
    """
    There are several environment variables that repo_man can optionally set to control where various caches are placed. They will all be relative to the repository root.
    - PM_PACKAGES_ROOT: this is where Packman stores its package cache
    - PIP_CACHE_DIR: this is where pip stores its wheel cache
    - UV_CACHE_DIR: this is where uv stores its wheel and package cache

    There are several gating flags as well to prevent repo_man from using the pip/uv default cache dir envvars unless explicitly set by us.
    - OM_PIP_CACHE: gating pip cache dir flag for omni.repo.man.deps.pip_install_requirements
    - OM_UV_CACHE: gating uv cache dir flag for omni.repo.man.deps._uv_requirements_load
    """

    repo_cache_file = Path(REPO_CACHE_FILE)
    if repo_cache_file.is_file():
        # cache file is present, read it in and set environment variables.
        cache_path_data = json.loads(repo_cache_file.read_text(encoding="utf-8"))
        # resolve REPO_ROOT rather than relative path to avoid any chdir shenanigans.
        resolved_root = Path(REPO_ROOT).resolve()

        for cache, cache_path in cache_path_data.items():
            # Expand $HOME and ~
            resolved_path = Path(os.path.expandvars(os.path.expanduser(cache_path)))
            if not resolved_path.is_dir():
                # Relative path to current working directory or absolute path is not present.
                # It's possible repo was somehow executed outside of the repository root.
                resolved_path = resolved_root / cache_path

            # Fully resolve path to avoid weird dir popping in some workflows.
            os.environ[cache] = resolved_path.resolve().as_posix()
            resolved_path.mkdir(parents=True, exist_ok=True)

            # Set repo_man breadcrumb to respect PIP_CACHE_DIR and UV_CACHE_DIR.
            # Unset OMNI_REPO_ROOT to force the caching of installed Python deps
            # in the packman cache dir.
            if cache == "PIP_CACHE_DIR":
                os.environ["OM_PIP_CACHE"] = "1"
                os.environ["OMNI_REPO_ROOT"] = ""
            elif cache == "UV_CACHE_DIR":
                os.environ["OM_UV_CACHE"] = "1"
                os.environ["OMNI_REPO_ROOT"] = ""


def _find_deps_files(repo_root):
    """Locate this repo's main and optional packman dep files.

    Resolution order, each step taking precedence over the next:

    1. ``REPO_DEPS_FILE`` and ``OPT_DEPS_FILE`` module constants on the
       running ``repoman.py`` (i.e. ``sys.modules['__main__']``). Repos that
       relocate their deps directory surface the change as module-level
       constants in their ``tools/repoman/repoman.py``; honouring those is
       how downstream consumers stay in sync without each having to teach
       us a new path.
    2. The conventional ``deps/`` directory under ``repo_root``. The
       optional file name is derived from :func:`_opt_deps_suffix` so the
       suffix stays parameter-driven from the repo's ``repo.toml``.

    Either return value may be ``None`` if the corresponding file does not
    exist on disk. ``REPO_DEPS_FILE`` / ``OPT_DEPS_FILE`` may be ``str`` or
    :class:`pathlib.Path`; both are normalised through :func:`os.fspath`.
    """
    main_path = None
    opt_path = None

    main_module = sys.modules.get("__main__")
    if main_module is not None:
        repo_deps_const = getattr(main_module, "REPO_DEPS_FILE", None)
        if repo_deps_const is not None:
            main_path = os.fspath(repo_deps_const)
        opt_deps_const = getattr(main_module, "OPT_DEPS_FILE", None)
        if opt_deps_const is not None:
            opt_path = os.fspath(opt_deps_const)

    # Fallback for the main file: the conventional ``deps/`` location.
    if main_path is None:
        main_path = os.path.join(repo_root, "deps", "repo-deps.packman.xml")

    # Fallback for the optional file: same directory as the main file,
    # filename built from the configured suffix. This handles repos that
    # only override REPO_DEPS_FILE, and repos that override neither.
    if opt_path is None:
        suffix = _opt_deps_suffix()
        deps_dir = os.path.dirname(main_path)
        opt_path = os.path.join(deps_dir, f"repo-deps-{suffix}.packman.xml")

    main_path = main_path if os.path.isfile(main_path) else None
    opt_path = opt_path if os.path.isfile(opt_path) else None
    return main_path, opt_path


@lru_cache  # Called by both _pull_optional_deps() and repoman.py's staleness check
def _opt_deps_suffix():
    """
    We want a general ability to specify an optional set of repo-tool dependencies for internal use.
    Since this config must be checked for before repo_man has been bootstrapped, accessing with toml
    is not an option. This is invoked on every tool startup, and needs to be fast, so a very simple
    load-and-search is used. No config-resolution occurs at this point.

    This is only accessed in the repoman.py entrypoint, and should not be used anywhere else.

    If this value is needed later in other contexts, it can instead be gotten from the resolved config.
    """

    opt_deps_suffix = "nv"
    repo_toml = Path(REPO_ROOT, "repo.toml")
    if repo_toml.is_file():
        # repo.toml is UTF-8 per TOML spec; do not rely on the host's preferred encoding
        # (e.g. cp932 on Japanese Windows) which would raise UnicodeDecodeError on any
        # non-ASCII byte.
        with open(repo_toml, encoding="utf-8") as f:
            for line in f.readlines():
                line = line.lstrip()
                if line.startswith("optional_deps_suffix"):
                    match = re.search(r"""optional_deps_suffix.=.['"](.+?)['"]""", line)
                    if match:
                        opt_deps_suffix = match.group(1)
                        break
    return opt_deps_suffix

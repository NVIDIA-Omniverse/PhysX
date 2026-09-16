#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

set -e

# Set OMNI_REPO_ROOT early so `repo` bootstrapping can target the repository
# root when writing out Python dependencies.
export OMNI_REPO_ROOT="$( cd "$(dirname "$0")" ; pwd -P )"

# Force Python's UTF-8 Mode (PEP 540) for every interpreter launched by repo_man.
# Without this, text-mode open() defaults to locale.getpreferredencoding(), which
# can be non-UTF-8 on hosts with non-UTF-8 system locales and raises
# UnicodeDecodeError on TOML/JSON/config files containing bytes that aren't valid
# in that codepage. This env var is exported only for the duration of this script's
# child processes (the `exec` below). The parent shell is not affected.
export PYTHONUTF8=1

# By default custom caching is disabled in repo_man. But if a repo-cache.json
# caching configuration file is generated via the `repo cache` command, its
# presence will trigger the configuration of custom caching.
if [[ -f "${OMNI_REPO_ROOT}/repo-cache.json" ]]; then
    PM_PACKAGES_ROOT=$(grep '"PM_PACKAGES_ROOT"' "${OMNI_REPO_ROOT}/repo-cache.json" | sed 's/.*"PM_PACKAGES_ROOT": "\(.*\)".*/\1/')

    # PM_PACKAGES_ROOT is present in the config file. It is set early
    # so Packman will reference the cached package repository.
    if [[ -n "${PM_PACKAGES_ROOT}" ]]; then
        # Expand a leading ~ to $HOME without `eval`. PM_PACKAGES_ROOT comes
        # from repo-cache.json, and running it through the shell would let a
        # crafted cache file execute arbitrary commands.
        RESOLVED_PACKAGES_ROOT="${PM_PACKAGES_ROOT/#\~/$HOME}"

        if [[ "${RESOLVED_PACKAGES_ROOT}" != /* ]]; then
            # PM_PACKAGES_ROOT is not an abs path, assumption is then
            # that it is a relative path to the repository root.
            PM_PACKAGES_ROOT="${OMNI_REPO_ROOT}/${RESOLVED_PACKAGES_ROOT}"
        else
            PM_PACKAGES_ROOT=${RESOLVED_PACKAGES_ROOT}
        fi
        export PM_PACKAGES_ROOT
    fi
fi

# Use "exec" to ensure that environment variables don't accidentally affect other processes.
exec "${OMNI_REPO_ROOT}/tools/packman/python.sh" "${OMNI_REPO_ROOT}/tools/repoman/repoman.py" "$@"

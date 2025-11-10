#!/bin/bash

set -e

# Set OMNI_REPO_ROOT early so `repo` bootstrapping can target the repository
# root when writing out Python dependencies.
export OMNI_REPO_ROOT="$( cd "$(dirname "$0")" ; pwd -P )"

# By default custom caching is disabled in repo_man. But if a repo-cache.json
# caching configuration file is generated via the `repo cache` command, it's
# presence will trigger the configuration of custom caching.
if [[ -f "${OMNI_REPO_ROOT}/repo-cache.json" ]]; then
    PM_PACKAGES_ROOT=$(grep '"PM_PACKAGES_ROOT"' "${OMNI_REPO_ROOT}/repo-cache.json" | sed 's/.*"PM_PACKAGES_ROOT": "\(.*\)".*/\1/')

    # PM_PACKAGES_ROOT is present in the config file. We set this early
    # so Packman will reference our cached package repository.
    if [[ -n "${PM_PACKAGES_ROOT}" ]]; then
        # Use eval to resolve ~ and perform parameter expansion
        RESOLVED_PACKAGES_ROOT=$(eval echo "$PM_PACKAGES_ROOT")

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

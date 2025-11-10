#!/bin/bash

set -e

# Set OMNI_REPO_ROOT early so `repo` bootstrapping can target the repository
# root when writing out Python dependencies.
export OMNI_REPO_ROOT="$( cd "$(dirname "$0")" ; pwd -P )"

SCRIPT_DIR=$(dirname ${BASH_SOURCE})
cd "$SCRIPT_DIR"

# Use "exec" to ensure that envrionment variables don't accidentally affect other processes.
exec "../tools/packman/python.sh" tools/repoman/repoman.py "$@"

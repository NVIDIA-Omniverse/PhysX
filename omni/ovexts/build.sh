#!/bin/bash
set -e
"$(dirname "${BASH_SOURCE}")/tools/packman/python.sh" "$(dirname "${BASH_SOURCE}")/tools/build.py" "$@"

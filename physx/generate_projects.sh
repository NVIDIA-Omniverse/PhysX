#!/bin/bash +x

if [ -n "${BASH_SOURCE[0]}" ]; then
    SCRIPT_DIR=$(dirname "${BASH_SOURCE[0]}")
else
    SCRIPT_DIR=$(dirname "$0")
fi

PACKMAN_CMD="$SCRIPT_DIR/buildtools/packman/packman"
if [ ! -f "$PACKMAN_CMD" ]; then
    PACKMAN_CMD="${PACKMAN_CMD}.sh"
fi
source "$PACKMAN_CMD" init

if [[ $# -eq 0 ]] ; then
    exec "$SCRIPT_DIR/buildtools/packman/python.sh" "$SCRIPT_DIR/buildtools/cmake_generate_projects.py"
    exit 1
fi

echo Running packman in preparation for cmake ...
cutName=${1%%.*}
export targetPlatform=$1

if [ "$1" = "$cutName" ] ; then
    source "$PACKMAN_CMD" pull "$SCRIPT_DIR/dependencies.xml" --platform $1
    exec "$SCRIPT_DIR/buildtools/packman/python.sh" "$SCRIPT_DIR/buildtools/cmake_generate_projects.py" "$targetPlatform"
else
    source "$PACKMAN_CMD" pull "$SCRIPT_DIR/dependencies.xml" --platform $cutName
    exec "$SCRIPT_DIR/buildtools/packman/python.sh" "$SCRIPT_DIR/buildtools/cmake_generate_projects.py" "$targetPlatform"
fi

status=$?
if [ "$status" -ne "0" ]; then
 echo "Error $status"
 exit 1
fi
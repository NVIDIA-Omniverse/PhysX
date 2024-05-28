#! /bin/bash

PACKMAN_CMD="$SCRIPT_DIR/buildtools/packman/packman"
if [ ! -f "$PACKMAN_CMD" ]; then
    PACKMAN_CMD="${PACKMAN_CMD}.sh"
fi
source "$PACKMAN_CMD" init
source "$PACKMAN_CMD" pull "$SCRIPT_DIR/dependencies.xml" -p linux-$(arch)

./generate_projects_aarch64.sh

cd _compiler/gmake2

make config=debug_aarch64
make config=release_aarch64

cd ../..

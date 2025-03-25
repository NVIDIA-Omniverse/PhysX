#! /bin/bash

SCRIPT_DIR=$(dirname ${BASH_SOURCE})
source "$SCRIPT_DIR/buildtools/packman/packman" pull "$SCRIPT_DIR/dependencies.xml" -p linux-$(arch)

./generate_projects_aarch64.sh

cd _compiler/gmake2

make config=debug_aarch64
make config=release_aarch64

cd ../..

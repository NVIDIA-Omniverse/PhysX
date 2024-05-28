#! /bin/bash
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
source "$SCRIPT_DIR/buildtools/packman/packman" pull "$SCRIPT_DIR/dependencies.xml" -p linux-$(arch)

./generate_projects.sh

cd _compiler/gmake2

make config=debug_x86_64
make config=release_x86_64

if [ $? -ne 0 ]; then
    echo "Error: building nvflow failed" >&2
    exit 1
fi

cd ../..

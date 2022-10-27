#! /bin/bash

./generate_projects_aarch64.sh

cd _compiler/gmake2

make config=debug_aarch64
make config=release_aarch64

cd ../..

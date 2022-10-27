#! /bin/bash

./generate_projects.sh

cd _compiler/gmake2

make config=debug_x86_64
make config=release_x86_64

cd ../..

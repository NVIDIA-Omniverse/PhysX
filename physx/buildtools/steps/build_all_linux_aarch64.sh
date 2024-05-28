#!/bin/bash +x

set -e

# get number of CPU cores
if [ -f /proc/cpuinfo ]; then
    CPUS=`grep processor /proc/cpuinfo | wc -l`
else
    CPUS=1
fi

# Stackoverflow suggests jobs count of (CPU cores + 1) as a respctively good number!
JOBS=`expr $CPUS + 1`

# run make for all configs
pushd "$(dirname "$0")/../../compiler/linux-aarch64-debug"
make -j$JOBS || exit 1
popd

pushd "$(dirname "$0")/../../compiler/linux-aarch64-release"
make -j$JOBS || exit 1
popd

pushd "$(dirname "$0")/../../compiler/linux-aarch64-checked"
make -j$JOBS || exit 1
popd

pushd "$(dirname "$0")/../../compiler/linux-aarch64-profile"
make -j$JOBS || exit 1
popd

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
pushd "$(dirname "$0")/../../compiler/linux-debug"
make -j$JOBS || exit 1
popd

pushd "$(dirname "$0")/../../compiler/linux-release"
make -j$JOBS || exit 1
popd

pushd "$(dirname "$0")/../../compiler/linux-checked"
make -j$JOBS || exit 1
popd

pushd "$(dirname "$0")/../../compiler/linux-profile"
make -j$JOBS || exit 1
popd

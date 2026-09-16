## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#!/bin/bash +x

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PHYSX_ROOT_DIR=$SCRIPT_DIR/../..

export GEN_META_DATA_PARAMETER=$1

PACKMAN_CMD="$PHYSX_ROOT_DIR/buildtools/packman/packman"
if [ ! -f "$PACKMAN_CMD" ]; then
    PACKMAN_CMD="${PACKMAN_CMD}.sh"
fi
export PYTHONPATH="${PM_MODULE_DIR}:${PYTHONPATH}"


if [ -f "$PACKMAN_CMD" ] ; then
    source "$PACKMAN_CMD" init
    "${PM_PYTHON}" -m unittest discover -s "$SCRIPT_DIR/tests" -p "test_*.py"
    if [ "$?" -ne "0" ]; then
        exit 1
    fi
    source "$PACKMAN_CMD" pull "$PHYSX_ROOT_DIR/dependencies.xml" --platform linux --include-tag=requiredForMetaGen
    "${PM_PYTHON}"  $SCRIPT_DIR/generateMetaData.py $GEN_META_DATA_PARAMETER
fi

status=$? 
if [ "$status" -ne "0" ]; then
 echo "Error $status"
 exit 1
fi

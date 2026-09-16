## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#!/bin/bash

set -e

python generateMetaData.py $GEN_META_DATA_PARAMETER
status=$? 
if [ "$status" -ne "0" ]; then
 echo "Error $status"
 exit 1
fi

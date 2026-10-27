#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

set -e
"$(dirname "${BASH_SOURCE}")/tools/packman/python.sh" "$(dirname "${BASH_SOURCE}")/tools/build.py" "$@"

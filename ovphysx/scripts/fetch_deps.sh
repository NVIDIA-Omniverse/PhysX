#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

#
# Thin wrapper to invoke centralized fetch_deps.cmake script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Forward environment variables and invoke CMake script
exec cmake -P "$SCRIPT_DIR/fetch_deps.cmake"

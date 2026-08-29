@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: BSD-3-Clause

REM Thin wrapper to invoke centralized fetch_deps.cmake script
cmake -P "%~dp0fetch_deps.cmake"

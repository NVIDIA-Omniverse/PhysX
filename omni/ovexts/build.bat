@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: Apache-2.0

setlocal enableextensions
call "%~dp0tools\packman\python.bat" "%~dp0tools\build.py" %*

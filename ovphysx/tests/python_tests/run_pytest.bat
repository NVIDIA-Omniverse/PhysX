@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: Apache-2.0

setlocal enableextensions
pushd "%~dp0"
py -3 "..\..\scripts\run_uv_pytest.py" "%CD%" %*
set "EXIT_CODE=%ERRORLEVEL%"
popd
exit /b %EXIT_CODE%

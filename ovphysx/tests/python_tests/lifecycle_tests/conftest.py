# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Pytest configuration for lifecycle tests.

These tests exercise the PhysX create/release cycle. Carbonite and the embedded
Python interpreter cannot be cleanly finalized and re-initialized in the same
process, so each test FILE in this directory gets its own subprocess invocation
from test_python.cmake.  Within a single file, only ONE create+release cycle
is permitted — all assertions share that single instance.
"""

# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Pytest configuration for lifecycle tests.

These tests exercise the PhysX create/destroy cycle. Carbonite and the embedded
Python interpreter cannot be cleanly finalized and re-initialized in the same
process, so each test FILE in this directory gets its own subprocess invocation
from scripts/test_python_runtime.cmake. Within a single file, only ONE
create+destroy cycle is permitted. All assertions share that single instance.
"""

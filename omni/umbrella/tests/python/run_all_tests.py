#!/usr/bin/env python
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

"""
Run all Physics Umbrella Python tests.

This script initializes the Carbonite framework and runs all test modules.
For pytest-based execution, use: pytest tests/python/
The conftest.py handles Carbonite initialization automatically.
"""

import sys
import os

import _carb_setup  # noqa: F401 - initializes Carbonite framework and loads plugins

import unittest

loader = unittest.TestLoader()
start_dir = os.path.dirname(__file__)
suite = loader.discover(start_dir, pattern="test*.py")

runner = unittest.TextTestRunner(verbosity=2)
result = runner.run(suite)

sys.exit(0 if result.wasSuccessful() else 1)

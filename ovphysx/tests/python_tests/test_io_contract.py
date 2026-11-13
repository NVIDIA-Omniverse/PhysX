# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-IO-CONTRACT-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Expose the standalone codegen regression suite to public Python test runners."""
import importlib.util
from pathlib import Path

_contract_tests_path = (Path(__file__).resolve().parents[2] / 'ovruntime' /
                        'physics_io_domain/tests/test_contract.py')
_spec = importlib.util.spec_from_file_location('_io_contract_tests', _contract_tests_path)
_contract_tests = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_contract_tests)


class TestRuntimeIOContract(_contract_tests.ContractTest):
    """Collect the native-free generator and committed-artifact regression suite."""

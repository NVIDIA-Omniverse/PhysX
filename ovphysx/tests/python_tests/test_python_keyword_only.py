# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-KEYWORD-001
# @covers AC-1 AC-2

"""Verify the positional and keyword-only split of selected Python APIs."""

from __future__ import annotations

import ast
import inspect
from pathlib import Path

import ovphysx
from ovphysx.api import PhysX, enable_python_logging


_EXPECTED_SIGNATURES = {
    "PhysX.__init__": (
        ("self",),
        ("config", "ignore_version_mismatch", "active_cuda_gpus"),
    ),
    "PhysX.wait_op": (("self", "op_index"), ("timeout_ns",)),
    "PhysX.wait_all": (("self",), ("timeout_ns",)),
    "PhysX.attach_ovstage": (("self", "stage"), ("read_ordinal",)),
    "PhysX.read": (("self", "object_type", "attribute_names"), ("scope",)),
    "PhysX.read_tokens": (("self", "object_type", "attribute_tokens"), ("scope",)),
    "PhysX.get_contact_report": (("self",), ("include_friction_anchors", "copy")),
    "enable_python_logging": (("logger_name",), ("min_severity", "channel_filter")),
}


def _runtime_callables():
    return {
        "PhysX.__init__": PhysX.__init__,
        "PhysX.wait_op": PhysX.wait_op,
        "PhysX.wait_all": PhysX.wait_all,
        "PhysX.attach_ovstage": PhysX.attach_ovstage,
        "PhysX.read": PhysX.read,
        "PhysX.read_tokens": PhysX.read_tokens,
        "PhysX.get_contact_report": PhysX.get_contact_report,
        "enable_python_logging": enable_python_logging,
    }


def _parameter_split(
    function: ast.FunctionDef,
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    assert not function.args.posonlyargs
    assert function.args.vararg is None
    assert function.args.kwarg is None
    positional = tuple(arg.arg for arg in function.args.args)
    keyword_only = tuple(arg.arg for arg in function.args.kwonlyargs)
    return positional, keyword_only


def _stub_signatures() -> dict[str, tuple[tuple[str, ...], tuple[str, ...]]]:
    package_dir = Path(ovphysx.__file__).resolve().parent
    module = ast.parse((package_dir / "api.pyi").read_text(encoding="utf-8"))
    signatures = {}
    for node in module.body:
        if isinstance(node, ast.FunctionDef) and node.name == "enable_python_logging":
            signatures[node.name] = _parameter_split(node)
        if isinstance(node, ast.ClassDef) and node.name == "PhysX":
            for child in node.body:
                if isinstance(child, ast.FunctionDef):
                    key = f"PhysX.{child.name}"
                    if key in _EXPECTED_SIGNATURES:
                        signatures[key] = _parameter_split(child)
    return signatures


def test_runtime_keyword_only_signatures():
    """Runtime signatures expose only primary operands positionally."""
    for function in _runtime_callables().values():
        assert all(
            parameter.kind
            in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
            for parameter in inspect.signature(function).parameters.values()
        )

    signatures = {
        name: (
            tuple(
                parameter.name
                for parameter in inspect.signature(function).parameters.values()
                if parameter.kind is inspect.Parameter.POSITIONAL_OR_KEYWORD
            ),
            tuple(
                parameter.name
                for parameter in inspect.signature(function).parameters.values()
                if parameter.kind is inspect.Parameter.KEYWORD_ONLY
            ),
        )
        for name, function in _runtime_callables().items()
    }
    assert signatures == _EXPECTED_SIGNATURES


def test_stub_keyword_only_signatures():
    """The PEP 561 stub matches the runtime positional split."""
    assert _stub_signatures() == _EXPECTED_SIGNATURES

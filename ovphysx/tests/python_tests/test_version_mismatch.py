# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-BUILDNUM-001
# @covers AC-7

import logging

import pytest

from ovphysx import PhysX, _bindings, api


@pytest.mark.parametrize("python_version,native_version", [
    ("0.6.3", "0.6.3.12345678"),
    ("0.6.3.12345678", "0.6.3"),
    ("0.6.3.12345678", "0.6.3.12345679"),
    ("0.6.3.12345678+trunk.abc12345", "0.6.3.12345679"),
])
def test_same_release_different_build_warns(monkeypatch, caplog, python_version, native_version):
    monkeypatch.setattr(api, "_python_version", python_version)
    monkeypatch.setattr(_bindings, "get_native_version_string", lambda: native_version)

    with caplog.at_level(logging.WARNING, logger=api.__name__):
        api._check_version_match()

    records = [record for record in caplog.records if record.name == api.__name__]
    assert len(records) == 1
    assert records[0].levelno == logging.WARNING
    message = records[0].getMessage()
    assert "same release but different builds" in message
    assert python_version in message
    assert native_version in message


@pytest.mark.parametrize("python_version,native_version", [
    ("0.6.3", "0.6.3"),
    ("0.6.3.12345678", "0.6.3.12345678"),
    ("0.6.3.12345678+trunk.abc12345", "0.6.3.12345678"),
])
def test_same_release_and_build_do_not_warn(monkeypatch, caplog, python_version, native_version):
    monkeypatch.setattr(api, "_python_version", python_version)
    monkeypatch.setattr(_bindings, "get_native_version_string", lambda: native_version)

    with caplog.at_level(logging.WARNING, logger=api.__name__):
        api._check_version_match()

    assert not [record for record in caplog.records if record.name == api.__name__]


@pytest.mark.parametrize("python_version,native_version", [
    ("0.6.3", "0.6.4"),
    ("0.6.3.12345678", "0.6.4.12345678"),
    ("0.6.3.12345678", "0.7.3.12345678"),
    ("0.6.3.12345678", "1.6.3.12345678"),
])
def test_different_release_raises(monkeypatch, python_version, native_version):
    monkeypatch.setattr(api, "_python_version", python_version)
    monkeypatch.setattr(_bindings, "get_native_version_string", lambda: native_version)

    with pytest.raises(RuntimeError, match="version does not match"):
        api._check_version_match()


def test_version_mismatch_raises(monkeypatch):
    monkeypatch.setattr(_bindings, "get_native_version_string", lambda: "9.9.9")

    with pytest.raises(RuntimeError, match="version does not match"):
        PhysX()


def test_shared_fixture_available(physx_sdk):
    """Smoke check that the shared physx_sdk fixture is alive and holds a valid handle.

    The ignore_version_mismatch create/destroy path is tested in
    lifecycle_tests/test_version_mismatch.py (separate subprocess).
    """
    assert physx_sdk is not None
    assert physx_sdk.handle != 0

# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause


# Tests for remote USD loading. No test needs credentials or a live bucket.
# Live-S3 tests were removed — they depended on a bucket and credentials this
# repo does not own, so any drift there failed the blocking test jobs.

import ctypes
import logging
import os

import pytest
from test_utils import load_usd_with_ovstage

log = logging.getLogger(__name__)


def _configure_omniclient_s3(host, bucket, region, access_key_id, secret_access_key, session_token):
    """Set S3 credentials directly on OmniClient — the asset layer the application
    owns. ovphysx no longer wraps this (it only consumes an already-populated
    Stage); a real app configures the USD resolver itself before populating
    ovstage. Requires libomniclient loaded in-process (creating a PhysX instance
    loads it). Mirrors the C signature of omniClientSetS3Configuration2.
    """
    fn = None
    library_names = ("omniclient.dll",) if os.name == "nt" else (None, "libomniclient.so")
    for library_name in library_names:
        try:
            lib = ctypes.CDLL(library_name)
        except OSError:
            continue
        fn = getattr(lib, "omniClientSetS3Configuration2", None)
        if fn:
            break
    if fn is None:
        pytest.skip("omniClientSetS3Configuration2 not available in process")
    fn.restype = ctypes.c_int
    fn.argtypes = [ctypes.c_char_p] * 6 + [ctypes.c_char_p, ctypes.c_bool, ctypes.c_bool]

    def enc(s):
        return s.encode("utf-8") if s else None

    rc = fn(enc(host), enc(bucket), enc(region), enc(access_key_id),
            enc(secret_access_key), enc(session_token), None, False, False)
    if rc != 0:
        raise RuntimeError(f"omniClientSetS3Configuration2 failed (result={rc})")


def test_configure_omniclient_s3_uses_platform_library(monkeypatch):
    loaded_names = []
    calls = []

    def fake_set_s3_configuration(*args):
        calls.append(args)
        return 0

    class FakeLibrary:
        pass

    def fake_cdll(name):
        loaded_names.append(name)
        library = FakeLibrary()
        library.omniClientSetS3Configuration2 = fake_set_s3_configuration
        return library

    monkeypatch.setattr(ctypes, "CDLL", fake_cdll)

    _configure_omniclient_s3(
        "bucket.s3.us-west-2.amazonaws.com",
        "bucket",
        "us-west-2",
        "access-key",
        "secret-key",
        "session-token",
    )

    expected_library = "omniclient.dll" if os.name == "nt" else None
    assert loaded_names == [expected_library]
    assert calls == [
        (
            b"bucket.s3.us-west-2.amazonaws.com",
            b"bucket",
            b"us-west-2",
            b"access-key",
            b"secret-key",
            b"session-token",
            None,
            False,
            False,
        )
    ]


@pytest.mark.skipif(os.name != "nt", reason="Windows OmniClient loader regression")
def test_windows_omniclient_s3_symbol_available(physx_sdk):
    del physx_sdk  # The fixture ensures CarboniteLoader has pinned OmniClient.
    library = ctypes.CDLL("omniclient.dll")
    assert getattr(library, "omniClientSetS3Configuration2", None) is not None


def test_ovstage_invalid_remote_uri(physx_sdk, monkeypatch):
    """Verify ovstage population fails gracefully for a nonexistent remote URI.

    Uses the ``physx_sdk`` fixture from conftest.py (no S3 credentials needed).
    The ``.invalid`` TLD is reserved by RFC 2606 and guaranteed to never resolve,
    so no real server is ever contacted.

    The host still costs a DNS/HTTP attempt, and omniclient retries failed HTTP
    requests with quadratic backoff (default cap 120s) — for a guaranteed-dead
    host that turned a ~0.5s failure into a ~30s one. ``OMNICLIENT_HTTP_RETRIES``
    is read per request, so disabling retries here (auto-restored by monkeypatch)
    keeps this negative test fast.
    """
    monkeypatch.setenv("OMNICLIENT_HTTP_RETRIES", "0")
    bad_uri = "https://nonexistent-bucket.s3.us-east-1.invalid/no_such_file.usd"
    log.info("Testing ovstage population with invalid remote URI: %s", bad_uri)
    with pytest.raises(RuntimeError, match=r"Failed"):
        op = load_usd_with_ovstage(physx_sdk, bad_uri)
        physx_sdk.wait_op(op)
    log.info("Invalid remote URI correctly rejected")

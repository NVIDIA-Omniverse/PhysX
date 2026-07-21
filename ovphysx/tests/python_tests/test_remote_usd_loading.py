# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#


# Tests for remote USD loading via S3.
#
# All S3 tests are skipped unless the following environment variables are set:
#   OVPHYSX_S3_TEST_URI          — full HTTPS URL to a .usd/.usda asset
#                                  (virtual-hosted style, e.g.
#                                  https://bucket.s3.region.amazonaws.com/path/scene.usda)
#   OVPHYSX_AWS_ACCESS_KEY_ID    — AWS access key
#   OVPHYSX_AWS_SECRET_ACCESS_KEY — AWS secret key
#   OVPHYSX_AWS_SESSION_TOKEN    — (optional) STS session token
#
# Host, bucket, and region are parsed from the URI automatically.
# CI sets these via pipeline variables; contributors can point at their own bucket.

import ctypes
import logging
import os
import re

import pytest
from test_utils import destroy_ovstage_test_attachments, load_usd_with_ovstage

log = logging.getLogger(__name__)

S3_TEST_ASSET = os.environ.get("OVPHYSX_S3_TEST_URI", "")

_has_s3_config = bool(
    S3_TEST_ASSET and os.environ.get("OVPHYSX_AWS_ACCESS_KEY_ID") and os.environ.get("OVPHYSX_AWS_SECRET_ACCESS_KEY")
)
requires_aws = pytest.mark.skipif(
    not _has_s3_config,
    reason="OVPHYSX_S3_TEST_URI and OVPHYSX_AWS_ACCESS_KEY_ID / OVPHYSX_AWS_SECRET_ACCESS_KEY not set",
)

_VH_PATTERN = re.compile(r"^https://(?P<bucket>[^.]+)\.s3\.(?P<region>[^.]+)\.amazonaws\.com/")


def _parse_s3_uri(uri: str) -> tuple[str, str, str]:
    """Extract (host, bucket, region) from an HTTPS virtual-hosted S3 URL."""
    m = _VH_PATTERN.match(uri)
    if not m:
        pytest.skip(f"OVPHYSX_S3_TEST_URI is not a virtual-hosted S3 URL: {uri}")
    bucket = m.group("bucket")
    region = m.group("region")
    host = f"{bucket}.s3.{region}.amazonaws.com"
    return host, bucket, region


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


@requires_aws
def test_remote_usd_resolver_is_active(_gpu_session_instance):
    """The S3 tests require OmniUsdResolver to win the one-time Ar selection."""
    del _gpu_session_instance  # The fixture initializes Carbonite, then activates the resolver.
    from pxr import Ar, Plug, Tf

    resolver_type = Tf.Type.FindByName("OmniUsdResolver")
    assert not resolver_type.isUnknown
    plugin = Plug.Registry().GetPluginForType(resolver_type)
    assert plugin is not None
    resolver = Ar.GetUnderlyingResolver()
    assert plugin.isLoaded
    assert not isinstance(resolver, Ar.DefaultResolver)
    print(
        "[test] remote USD resolver: "
        f"plugin={plugin.path} loaded={plugin.isLoaded} default=False",
        flush=True,
    )


@pytest.fixture(scope="session")
def s3_physx(_gpu_session_instance):
    """Session-scoped PhysX instance with S3 credentials configured.

    Uses session scope to avoid the Carbonite re-init limitation.
    """
    del _gpu_session_instance  # Ensure resolver activation precedes this instance's first stage load.
    import ovphysx

    log.info("Creating PhysX instance for S3 tests")
    physx = ovphysx.PhysX()

    host, bucket, region = _parse_s3_uri(S3_TEST_ASSET)

    access_key_id = os.environ["OVPHYSX_AWS_ACCESS_KEY_ID"]
    secret_access_key = os.environ["OVPHYSX_AWS_SECRET_ACCESS_KEY"]
    session_token = os.environ.get("OVPHYSX_AWS_SESSION_TOKEN")

    log.info(
        "Configuring S3: host=%s bucket=%s region=%s session_token=%s",
        host,
        bucket,
        region,
        "set" if session_token else "not set",
    )
    # The application configures the asset layer (OmniClient) itself — ovphysx
    # does not wrap credential setup in the ovstage model.
    _configure_omniclient_s3(host, bucket, region, access_key_id, secret_access_key, session_token)
    log.info("S3 credentials configured successfully")

    yield physx
    destroy_ovstage_test_attachments(physx)
    physx.release()
    log.info("S3 PhysX instance released")


@requires_aws
def test_load_s3_usd(s3_physx):
    """Load a USD scene from S3 and verify a valid stage is returned."""
    log.info("Loading S3 asset: %s", S3_TEST_ASSET)
    op = load_usd_with_ovstage(s3_physx, S3_TEST_ASSET)
    s3_physx.wait_op(op)
    s3_physx.wait_all()
    log.info("S3 asset loaded successfully")

    # A successful attach must allow stepping the simulation without crashing.
    s3_physx.step_sync(1.0 / 60.0)

    destroy_ovstage_test_attachments(s3_physx)
    s3_physx.reset_stage()
    s3_physx.wait_all()
    log.info("S3 asset removed and cleanup complete")


@requires_aws
def test_simulate_s3_usd(s3_physx):
    """Load a USD scene from S3, step simulation, and verify it completes."""
    log.info("Loading S3 asset for simulation: %s", S3_TEST_ASSET)
    op = load_usd_with_ovstage(s3_physx, S3_TEST_ASSET)
    s3_physx.wait_op(op)
    log.info("S3 asset loaded, running 5 simulation steps")

    dt = 1.0 / 60.0
    for i in range(5):
        step_op = s3_physx.step(dt)
        s3_physx.wait_op(step_op)

    log.info("Simulation steps completed successfully")
    destroy_ovstage_test_attachments(s3_physx)
    s3_physx.reset_stage()
    s3_physx.wait_all()
    log.info("S3 simulation test cleanup complete")


def test_ovstage_invalid_remote_uri(physx_sdk, monkeypatch):
    """Verify ovstage population fails gracefully for a nonexistent remote URI.

    Uses the ``physx_sdk`` fixture from conftest.py (no S3 credentials needed).
    The ``.invalid`` TLD is reserved by RFC 2606 and guaranteed to never resolve,
    so no real server is ever contacted.

    The host still costs a DNS/HTTP attempt, and omniclient retries failed HTTP
    requests with quadratic backoff (default cap 120s) — for a guaranteed-dead
    host that turned a ~0.5s failure into a ~30s one. ``OMNICLIENT_HTTP_RETRIES``
    is read per request, so disabling retries here (auto-restored by monkeypatch)
    keeps this negative test fast without weakening retry behaviour for the real
    S3 integration tests above.
    """
    monkeypatch.setenv("OMNICLIENT_HTTP_RETRIES", "0")
    bad_uri = "https://nonexistent-bucket.s3.us-east-1.invalid/no_such_file.usd"
    log.info("Testing ovstage population with invalid remote URI: %s", bad_uri)
    with pytest.raises(RuntimeError, match=r"Failed"):
        op = load_usd_with_ovstage(physx_sdk, bad_uri)
        physx_sdk.wait_op(op)
    log.info("Invalid remote URI correctly rejected")

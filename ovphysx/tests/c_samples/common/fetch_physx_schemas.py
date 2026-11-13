#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-1 AC-2 AC-3 AC-6

"""Install the external schema provider for native samples (stdlib only).

The wheel is a platform-independent data archive here. Python is needed for
this installation step only, never by the native application at runtime.
"""

import argparse
from email.parser import Parser
import hashlib
import http.client
import io
import json
import os
from pathlib import Path, PurePosixPath
import ssl
import sys
import tempfile
import time
import urllib.error
import urllib.request
import zipfile

_LOCK = json.loads(Path(__file__).with_name("physx_schemas.lock.json").read_text(encoding="utf-8"))
SHA256 = _LOCK["sha256"]
URL = _LOCK["url"]
DOWNLOAD_TIMEOUT_SECONDS = 60
RETRY_DELAY_SECONDS = 60


def _filesystem_path(path):
    path = str(Path(path).resolve())
    if os.name == "nt" and not path.startswith("\\\\?\\"):
        path = "\\\\?\\UNC\\" + path[2:] if path.startswith("\\\\") else "\\\\?\\" + path
    return Path(path)


def _download(retry_timeout=0):
    """Fetch the locked wheel, retrying transient failures for retry_timeout seconds.

    Same rules as the OVStage native archive fetcher: an active transfer may finish after the
    window and no new attempt starts after it; authentication, certificate and local file
    errors fail immediately; a temporary repository 404 is retried for the same locked URL.
    Zero is a single attempt with the original socket timeout.
    """
    deadline = time.monotonic() + retry_timeout
    attempt = 0
    while True:
        attempt += 1
        timeout = DOWNLOAD_TIMEOUT_SECONDS
        if retry_timeout:
            timeout = min(timeout, max(1, deadline - time.monotonic()))
        try:
            with urllib.request.urlopen(URL, timeout=timeout) as response:
                return response.read()
        except (urllib.error.URLError, TimeoutError, ConnectionError,
                http.client.IncompleteRead, ssl.SSLEOFError) as error:
            if not retry_timeout:
                raise
            if isinstance(error, urllib.error.HTTPError):
                if error.code not in (404, 408, 429) and not 500 <= error.code < 600:
                    raise
                error.close()
            elif isinstance(error, urllib.error.URLError):
                if isinstance(error.reason, (ssl.SSLCertVerificationError, FileNotFoundError,
                                             PermissionError, IsADirectoryError,
                                             NotADirectoryError)):
                    raise
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise
            delay = min(RETRY_DELAY_SECONDS, remaining)
            print(f"  schemas: download attempt {attempt} failed ({error}); waiting {delay:.0f}s "
                  f"({remaining:.0f}s retry window remaining)", file=sys.stderr, flush=True)
            time.sleep(delay)
            if time.monotonic() >= deadline:
                raise


def validate_provider(root):
    root = _filesystem_path(root)
    metadata_files = list(root.glob("physx_usd_schemas-*.dist-info/METADATA"))
    if len(metadata_files) != 1:
        raise RuntimeError("Incompatible schema provider: expected one physx-usd-schemas distribution")
    metadata = Parser().parsestr(metadata_files[0].read_text(encoding="utf-8"))
    if metadata.get("Name") != "physx-usd-schemas" or not metadata.get("Version"):
        raise RuntimeError("Incompatible schema provider: invalid physx-usd-schemas package metadata")
    for module in ("PhysxSchema", "OmniUsdPhysicsDeformableSchema"):
        for filename in ("plugInfo.json", "generatedSchema.usda"):
            path = root / "physx_usd_schemas" / module / "resources" / filename
            if not path.is_file():
                raise RuntimeError(f"Incomplete schema provider: missing {path.relative_to(root)}")


def install(destination, wheel=None, sha256=SHA256, retry_timeout=0):
    if sha256 != SHA256 and wheel is None:
        raise ValueError("A custom SHA-256 is only allowed with an explicitly supplied local wheel")
    destination = Path(destination).resolve()
    target = _filesystem_path(destination)
    marker = target / ".provider-sha256"
    if marker.is_file() and marker.read_text().strip() == sha256:
        validate_provider(destination)
        return destination
    if target.exists():
        raise RuntimeError(f"Incomplete or incompatible schema provider at {destination}; choose a new empty destination.")
    if wheel:
        payload = _filesystem_path(wheel).read_bytes()
    else:
        payload = _download(retry_timeout)
    if hashlib.sha256(payload).hexdigest() != sha256:
        raise RuntimeError("physx-usd-schemas wheel SHA-256 mismatch; installation refused")
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="physx-schemas-", dir=target.parent) as temporary:
        staged = Path(temporary) / "provider"
        staged.mkdir()
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            for entry in archive.infolist():
                path = PurePosixPath(entry.filename)
                if (path.is_absolute() or ".." in path.parts or "\\" in entry.filename
                        or ":" in entry.filename):
                    raise RuntimeError(f"Unsafe wheel entry: {entry.filename}")
            archive.extractall(staged)
        validate_provider(staged)
        (staged / marker.name).write_text(sha256 + "\n", encoding="utf-8")
        staged.rename(target)
    return destination


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    location = parser.add_mutually_exclusive_group(required=True)
    location.add_argument("--dest", help="Separate application-owned schema directory")
    location.add_argument("--cache-dir", help="Cache root; creates a provider directory named by its digest")
    parser.add_argument("--wheel", help="Use an already downloaded or locally built wheel")
    parser.add_argument("--sha256", default=SHA256,
                        help="Expected digest for --wheel (defaults to the published provider digest)")
    parser.add_argument("--retry-timeout", type=int,
                        default=os.environ.get("OVSTAGE_FETCH_RETRY_TIMEOUT") or "0",  # empty means unset
                        help="retry transient download failures for this many seconds "
                             "(default: OVSTAGE_FETCH_RETRY_TIMEOUT, or 0 for a single attempt)")
    args = parser.parse_args(argv)
    if args.retry_timeout < 0:
        parser.error("--retry-timeout must be nonnegative")
    destination = args.dest or Path(args.cache_dir) / "physx-usd-schemas" / args.sha256
    print(install(destination, args.wheel, args.sha256, args.retry_timeout))


if __name__ == "__main__":
    main()

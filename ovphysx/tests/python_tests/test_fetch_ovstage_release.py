# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-OVSTAGE-001
# @covers AC-1, AC-2

"""OVStage archive identity, extraction, and retry regressions."""

import errno
import hashlib
import http.client
import io
import os
import ssl
import sys
import urllib.error
import zipfile
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import fetch_ovstage_release as m  # noqa: E402

_VERSION = "0.2.0.999999.deadbeef"


class _Clock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        assert seconds > 0
        self.sleeps.append(seconds)
        self.now += seconds


@pytest.fixture
def retry_clock(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(m.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(m.time, "sleep", clock.sleep)
    return clock


class _Response(io.BytesIO):
    def __init__(self, body, content_length=None):
        super().__init__(body)
        self.headers = {"Content-Length": str(len(body) if content_length is None else content_length)}


class _DisconnectedResponse(_Response):
    def read(self, size=-1):
        chunk = super().read(size)
        if not chunk:
            raise ConnectionResetError("connection lost while reading response")
        return chunk


def test_retry_window_covers_thirty_minutes(retry_clock):
    attempts = []
    failure = urllib.error.HTTPError("https://example.invalid/", 404, "Not Found", {}, None)

    def unavailable(timeout):
        attempts.append((retry_clock.now, timeout))
        raise failure

    with pytest.raises(urllib.error.HTTPError) as exc:
        m._retry_network(unavailable, "test archive", 120, retry_timeout=1800)

    assert exc.value is failure
    assert [when for when, _ in attempts] == list(range(0, 1800, 60))
    assert all(0 < timeout <= min(120, 1800 - when) for when, timeout in attempts)
    assert retry_clock.now == 1800
    assert retry_clock.sleeps == [60] * 30


def test_retry_recovers_after_more_than_fifteen_minutes(retry_clock):
    attempts = []

    def eventually_available(timeout):
        attempts.append(retry_clock.now)
        if retry_clock.now < 16 * 60:
            raise urllib.error.URLError("temporary DNS failure")
        return "resolved pinned archive"

    assert m._retry_network(eventually_available, "test archive", 120, retry_timeout=1800) == "resolved pinned archive"
    assert attempts == list(range(0, 16 * 60 + 1, 60))


def test_retry_counts_request_time_and_limits_last_timeout(retry_clock):
    attempts = []

    def unavailable(timeout):
        attempts.append((retry_clock.now, timeout))
        retry_clock.now += 1660 if len(attempts) == 1 else timeout
        raise TimeoutError("request timed out")

    with pytest.raises(TimeoutError):
        m._retry_network(unavailable, "test archive", 120, retry_timeout=1800)

    assert attempts == [(0, 120), (1720, 80)]
    assert retry_clock.now == 1800
    assert retry_clock.sleeps == [60]


@pytest.mark.parametrize("status", [404, 408, 429, 500, 502, 503, 504, 599])
def test_retry_recovers_from_transient_http_status(status, retry_clock):
    attempts = []

    def request(timeout):
        attempts.append(timeout)
        if len(attempts) == 1:
            raise urllib.error.HTTPError("https://example.invalid/", status, "unavailable", {}, None)
        return b"response"

    assert m._retry_network(request, "test request", 120, retry_timeout=1800) == b"response"
    assert len(attempts) == 2
    assert retry_clock.sleeps == [60]


@pytest.mark.parametrize("failure", [
    urllib.error.URLError("temporary DNS failure"),
    TimeoutError("timed out"),
    ConnectionResetError("connection reset"),
    http.client.IncompleteRead(b"partial", 100),
    ssl.SSLEOFError("TLS connection closed"),
])
def test_retry_recovers_from_network_errors(failure, retry_clock):
    attempts = []

    def request(timeout):
        attempts.append(timeout)
        if len(attempts) == 1:
            raise failure
        return b"response"

    assert m._retry_network(request, "test request", 120, retry_timeout=1800) == b"response"
    assert len(attempts) == 2
    assert retry_clock.sleeps == [60]


@pytest.mark.parametrize("failure", [
    urllib.error.HTTPError("https://example.invalid/", 400, "Bad Request", {}, None),
    urllib.error.HTTPError("https://example.invalid/", 401, "Unauthorized", {}, None),
    urllib.error.HTTPError("https://example.invalid/", 403, "Forbidden", {}, None),
    ssl.SSLCertVerificationError("certificate verify failed"),
    urllib.error.URLError(ssl.SSLCertVerificationError("certificate verify failed")),
    urllib.error.URLError(FileNotFoundError(errno.ENOENT, "missing local file")),
    urllib.error.URLError(PermissionError(errno.EACCES, "local file denied")),
])
def test_retry_rejects_permanent_errors_immediately(failure, retry_clock):
    attempts = []

    def request(timeout):
        attempts.append(timeout)
        raise failure

    with pytest.raises(type(failure)) as exc:
        m._retry_network(request, "test request", 120, retry_timeout=1800)

    assert exc.value is failure
    assert len(attempts) == 1
    assert retry_clock.sleeps == []


@pytest.mark.parametrize("failure_mode", ["disconnect", "short_read"])
def test_download_retries_and_replaces_partial_file(failure_mode, tmp_path, monkeypatch, retry_clock):
    partial_body = b"incomplete" * 1000
    final_body = b"complete" * 625
    attempts = []
    responses = []
    destination = tmp_path / "archive.zip"

    def urlopen(request, timeout):
        attempts.append((request.full_url, timeout))
        if len(attempts) == 1:
            response = (_DisconnectedResponse(partial_body) if failure_mode == "disconnect"
                        else _Response(partial_body, len(partial_body) + 100))
        else:
            assert destination.read_bytes() == partial_body
            response = _Response(final_body)
        responses.append(response)
        return response

    monkeypatch.setattr(m.urllib.request, "urlopen", urlopen)
    m._download("https://example.invalid/archive.zip", str(destination), hashlib.sha256(final_body).hexdigest(),
                retry_timeout=1800)
    assert destination.read_bytes() == final_body
    assert len(attempts) == 2
    assert all(response.closed for response in responses)
    assert retry_clock.sleeps == [60]


@pytest.mark.parametrize("error_number", [errno.ENOSPC, errno.EACCES])
def test_download_does_not_retry_local_write_errors(error_number, tmp_path, monkeypatch, retry_clock):
    attempts = []
    failure = OSError(error_number, "local write failed")

    def urlopen(request, timeout):
        attempts.append(timeout)
        return _Response(b"x" * 5000)

    class UnwritableOutput:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def write(self, data):
            raise failure

    monkeypatch.setattr(m.urllib.request, "urlopen", urlopen)
    monkeypatch.setattr(m, "open", lambda *a, **kw: UnwritableOutput(), raising=False)
    with pytest.raises(OSError) as exc:
        m._download("https://example.invalid/archive.zip", str(tmp_path / "archive.zip"), "a" * 64, retry_timeout=1800)
    assert exc.value is failure
    assert len(attempts) == 1
    assert retry_clock.sleeps == []


def test_default_download_preserves_content_length_behavior(tmp_path, monkeypatch, retry_clock):
    body = b"x" * 5000
    monkeypatch.setattr(m.urllib.request, "urlopen", lambda *a, **kw: _Response(body, len(body) + 100))
    destination = tmp_path / "archive.zip"
    m._download("https://example.invalid/", str(destination), hashlib.sha256(body).hexdigest())
    assert destination.read_bytes() == body
    assert retry_clock.sleeps == []


def test_default_download_fails_once_with_original_timeout(tmp_path, monkeypatch, retry_clock):
    attempts = []

    def unavailable(request, timeout):
        attempts.append(timeout)
        raise urllib.error.HTTPError(request.full_url, 404, "Not Found", {}, None)

    monkeypatch.setattr(m.urllib.request, "urlopen", unavailable)
    destination = tmp_path / "archive.zip"
    with pytest.raises(SystemExit, match="not published.*HTTP 404"):
        m._download("https://example.invalid/archive.zip", str(destination), "a" * 64)
    assert attempts == [600]
    assert retry_clock.sleeps == []
    assert not destination.exists()


def test_download_discards_partial_archive_when_retry_window_expires(tmp_path, monkeypatch, retry_clock):
    attempts = []

    def truncated(request, timeout):
        attempts.append(timeout)
        return _Response(b"x" * 5000, content_length=6000)

    monkeypatch.setattr(m.urllib.request, "urlopen", truncated)
    destination = tmp_path / "archive.zip"
    with pytest.raises(http.client.IncompleteRead):
        m._download("https://example.invalid/archive.zip", str(destination), "a" * 64, retry_timeout=120)
    assert attempts == [120, 60]
    assert retry_clock.sleeps == [60, 60]
    assert not destination.exists()


@pytest.mark.parametrize("environment,argument,expected", [
    (None, None, 0),
    ("1800", None, 1800),
    (None, "1800", 1800),
    ("1800", "0", 0),
    ("invalid", "1800", 1800),
])
def test_main_forwards_opt_in_retry_timeout(environment, argument, expected, tmp_path, monkeypatch):
    seen = []
    monkeypatch.delenv("OVSTAGE_SKIP_FETCH", raising=False)
    monkeypatch.delenv("OVSTAGE_FETCH_RETRY_TIMEOUT", raising=False)
    if environment is not None:
        monkeypatch.setenv("OVSTAGE_FETCH_RETRY_TIMEOUT", environment)
    argv = ["fetch_ovstage_release.py", "--platform", "windows-x86_64", "--dest", str(tmp_path / "ovstage")]
    if argument is not None:
        argv.extend(["--retry-timeout", argument])
    monkeypatch.setattr(sys, "argv", argv)

    class DownloadReached(Exception):
        pass

    def download(url, destination, sha, retry_timeout=0):
        seen.append(("download", retry_timeout))
        raise DownloadReached

    monkeypatch.setattr(m, "_download", download)
    with pytest.raises(DownloadReached):
        m.main()
    assert seen == [("download", expected)]


@pytest.mark.parametrize("source,value", [("environment", "-1"), ("environment", "invalid"),
                                          ("argument", "-1"), ("argument", "invalid")])
def test_main_rejects_invalid_retry_timeout_before_fetch(source, value, tmp_path, monkeypatch):
    monkeypatch.delenv("OVSTAGE_FETCH_RETRY_TIMEOUT", raising=False)
    argv = ["fetch_ovstage_release.py", "--platform", "windows-x86_64", "--dest", str(tmp_path / "ovstage")]
    if source == "environment":
        monkeypatch.setenv("OVSTAGE_FETCH_RETRY_TIMEOUT", value)
    else:
        argv.extend(["--retry-timeout", value])
    monkeypatch.setattr(sys, "argv", argv)
    monkeypatch.setattr(m, "_download", lambda *a, **kw: pytest.fail("invalid timeout started a fetch"))
    with pytest.raises(SystemExit) as exc:
        m.main()
    assert exc.value.code == 2


def _add_symlink(archive, name, target):
    """Add a symlink entry the way the real archive stores .so version chains."""
    info = zipfile.ZipInfo(name)
    info.create_system = 3  # unix
    info.external_attr = (0o120777 << 16)  # S_IFLNK | 0777
    archive.writestr(info, target)


def _native_zip(path, *, notices=True, include=True, bin_dir=True, extra=None, symlinks=None):
    """Build a stand-in for the OVStage native archive (root-level package layout)."""
    with zipfile.ZipFile(path, "w") as archive:
        if include:
            archive.writestr("include/ovstage/ovstage.h", "// header\n")
        if bin_dir:
            archive.writestr("bin/libovstage.so", "ELF-ish\n")
        archive.writestr("lib/cmake/ovstage/ovstageConfig.cmake", "# cmake\n")
        archive.writestr("python/ovstage/__init__.py", "# package\n")
        if notices:
            archive.writestr("THIRD-PARTY-NOTICES.txt", "notices\n")
        for name, body in (extra or {}).items():
            archive.writestr(name, body)
        for name, target in (symlinks or {}).items():
            _add_symlink(archive, name, target)
    return path


def test_main_retries_pinned_native_archive_then_verifies_and_extracts(tmp_path, monkeypatch, retry_clock):
    body = _native_zip(tmp_path / "source.zip", extra={"bin/padding": "x" * 5000}).read_bytes()
    platform = "windows-x86_64"
    digest = hashlib.sha256(body).hexdigest()
    monkeypatch.setattr(m, "OVSTAGE_DIGESTS", {platform: digest})
    monkeypatch.setenv("OVSTAGE_FETCH_RETRY_TIMEOUT", "1800")
    monkeypatch.delenv("OVSTAGE_SKIP_FETCH", raising=False)
    destination = tmp_path / "sdk"
    monkeypatch.setattr(sys, "argv", ["fetch_ovstage_release.py", "--platform", platform,
                                     "--dest", str(destination)])
    attempts = []

    def eventually_available(request, timeout):
        attempts.append(request.full_url)
        if len(attempts) == 1:
            raise urllib.error.HTTPError(request.full_url, 404, "Not Found", {}, None)
        if len(attempts) == 2:
            return _Response(body[:5000], content_length=len(body))
        return _Response(body)

    monkeypatch.setattr(m.urllib.request, "urlopen", eventually_available)
    m.main()
    assert attempts == [m._zip_url(m.OVSTAGE_VERSION, platform)] * 3
    assert retry_clock.sleeps == [60, 60]
    assert (destination / "include/ovstage/ovstage.h").is_file()
    assert (destination / "THIRD-PARTY-NOTICES.txt").read_text() == "notices\n"
    assert not (tmp_path / m._zip_name(m.OVSTAGE_VERSION, platform)).exists()


def test_numeric_version_drops_trailing_git_hash():
    assert m._numeric_version("0.1.0.999999.deadbeef") == "0.1.0.999999"  # M.m.p.build.hash
    assert m._numeric_version("0.1.0.999999") == "0.1.0.999999"           # M.m.p.build
    assert m._numeric_version("0.1.0") == "0.1.0"                         # M.m.p
    # An all-digit hash is still the 5th component, dropped by position.
    assert m._numeric_version("0.1.0.888888.12345678") == "0.1.0.888888"


def test_semver_and_githash_split_by_position():
    assert m._semver(_VERSION) == "0.2.0"
    assert m._githash(_VERSION) == "deadbeef"
    assert m._githash("0.2.0.999999") is None
    # All-digit hashes are taken by position, like _numeric_version.
    assert m._githash("0.1.0.888888.12345678") == "12345678"


def test_zip_name_matches_the_published_archive_convention():
    assert m._zip_name(_VERSION, "manylinux_2_35_x86_64") == \
        "ovstage@0.2.0.999999.deadbeef.manylinux_2_35_x86_64.zip"
    # The zip uses its own platform tokens -- no win_amd64 mapping, unlike the wheel.
    assert m._zip_name(_VERSION, "windows-x86_64") == \
        "ovstage@0.2.0.999999.deadbeef.windows-x86_64.zip"


def test_zip_name_requires_a_githash():
    with pytest.raises(SystemExit, match="no githash component"):
        m._zip_name("0.2.0.999999", "windows-x86_64")


def test_zip_url_renders_both_mirror_shapes(monkeypatch):
    monkeypatch.setattr(
        m, "OVSTAGE_ZIP_URL",
        "https://artifactory.invalid/artifactory/ct-omniverse-pypi-local/ovstage/{numeric}/{archive}")
    assert m._zip_url(_VERSION, "windows-x86_64") == (
        "https://artifactory.invalid/artifactory/ct-omniverse-pypi-local/ovstage/"
        "0.2.0.999999/ovstage@0.2.0.999999.deadbeef.windows-x86_64.zip")

    # The public mirror keys off the release tag (v<semver>), not the build number.
    monkeypatch.setattr(
        m, "OVSTAGE_ZIP_URL",
        "https://github.invalid/NVIDIA-Omniverse/ovstage/releases/download/v{semver}/{archive}")
    assert m._zip_url(_VERSION, "windows-x86_64") == (
        "https://github.invalid/NVIDIA-Omniverse/ovstage/releases/download/v0.2.0/"
        "ovstage@0.2.0.999999.deadbeef.windows-x86_64.zip")


def test_digest_lookup_reports_an_unpinned_platform(monkeypatch):
    monkeypatch.setattr(m, "OVSTAGE_DIGESTS", {"windows-x86_64": "a" * 64})
    assert m._digest("windows-x86_64") == "a" * 64
    # A platform with no entry points the caller at the tool that regenerates the table.
    with pytest.raises(SystemExit, match="sync_ovstage_version"):
        m._digest("manylinux_2_35_x86_64")


@pytest.mark.parametrize("retry_timeout", [0, 1800])
def test_download_verifies_sha256(tmp_path, retry_timeout, retry_clock):
    src = tmp_path / "blob.bin"
    src.write_bytes(b"x" * 5000)  # over the 4096 size floor
    good = hashlib.sha256(src.read_bytes()).hexdigest()
    m._download(src.as_uri(), str(tmp_path / "ok.bin"), good, retry_timeout=retry_timeout)
    m._download(src.as_uri(), str(tmp_path / "ok2.bin"), "sha256:" + good, retry_timeout=retry_timeout)  # prefix tolerated

    bad = tmp_path / "bad.bin"
    with pytest.raises(SystemExit, match="does not match pinned"):
        m._download(src.as_uri(), str(bad), "0" * 64, retry_timeout=retry_timeout)
    # An unverified archive must not survive under the pinned artifact name.
    assert not bad.exists()
    assert retry_clock.sleeps == []


@pytest.mark.parametrize("retry_timeout", [0, 1800])
def test_download_discards_a_short_error_page(tmp_path, retry_timeout, retry_clock):
    src = tmp_path / "tiny.bin"
    src.write_bytes(b"<html>404</html>")
    out = tmp_path / "out.bin"
    with pytest.raises(SystemExit, match="looks invalid"):
        m._download(src.as_uri(), str(out), "0" * 64, retry_timeout=retry_timeout)
    assert not out.exists()
    assert retry_clock.sleeps == []


def test_extract_zip_lands_the_package_at_the_root(tmp_path):
    archive = _native_zip(tmp_path / "ovstage.zip")
    dest = tmp_path / "ovstage"
    m._extract_zip(str(archive), str(dest))

    assert (dest / "include" / "ovstage" / "ovstage.h").is_file()
    assert (dest / "bin" / "libovstage.so").is_file()
    assert (dest / "lib" / "cmake" / "ovstage" / "ovstageConfig.cmake").is_file()
    assert (dest / "python" / "ovstage" / "__init__.py").is_file()
    assert not (dest / "ovstage").exists()


def _symlinks_available(tmp_path):
    """Whether this process may create symlinks (Windows needs developer mode)."""
    probe = tmp_path / "_symlink_probe"
    try:
        os.symlink("target", str(probe))
    except (OSError, NotImplementedError, AttributeError):
        return False
    os.remove(str(probe))
    return True


def test_extract_zip_recreates_symlinks(tmp_path):
    archive = _native_zip(
        tmp_path / "ovstage.zip",
        extra={"bin/plugins/libtbb.so.12.13": "ELF" + "x" * 500},
        symlinks={"bin/plugins/libtbb.so.12": "libtbb.so.12.13"},
    )
    dest = tmp_path / "ovstage"
    m._extract_zip(str(archive), str(dest))

    link = dest / "bin" / "plugins" / "libtbb.so.12"
    # What has to hold everywhere: the entry is the real library, not the text stub.
    assert link.stat().st_size > 400
    if _symlinks_available(tmp_path):
        assert link.is_symlink(), "version-chain entry was written as a regular file"
        assert os.readlink(str(link)) == "libtbb.so.12.13"
    else:
        # Windows without developer mode: _extract_symlink copies the target instead.
        assert not link.is_symlink()


def test_extract_zip_rejects_a_symlink_escaping_dest(tmp_path):
    archive = _native_zip(
        tmp_path / "ovstage.zip",
        symlinks={"bin/evil.so": "../../../../etc/passwd"},
    )
    with pytest.raises(SystemExit, match="escapes dest"):
        m._extract_zip(str(archive), str(tmp_path / "ovstage"))


def test_extract_zip_requires_the_third_party_notices(tmp_path):
    archive = _native_zip(tmp_path / "ovstage.zip", notices=False)
    with pytest.raises(SystemExit, match="THIRD-PARTY-NOTICES"):
        m._extract_zip(str(archive), str(tmp_path / "ovstage"))


def test_extract_zip_rejects_layout_without_include_or_bin(tmp_path):
    archive = _native_zip(tmp_path / "ovstage.zip", include=False)
    with pytest.raises(SystemExit, match="missing include/ or bin/"):
        m._extract_zip(str(archive), str(tmp_path / "ovstage"))


def test_extract_zip_rejects_zip_slip(tmp_path):
    archive = _native_zip(tmp_path / "ovstage.zip", extra={"../../escape.txt": "pwned\n"})
    with pytest.raises(SystemExit, match="escapes dest"):
        m._extract_zip(str(archive), str(tmp_path / "ovstage"))


def test_extract_zip_replaces_a_previous_ovstage_tree(tmp_path):
    dest = tmp_path / "ovstage"
    (dest / "include" / "ovstage").mkdir(parents=True)
    (dest / "include" / "ovstage" / "stale.h").write_text("// from an older pin\n")
    (dest / "bin").mkdir()
    metadata = {"VERSION.md": "0.3.0.1791455901\n", "ovstage-build-info.json": "{}\n"}
    for name in metadata:
        (dest / name).write_text("from an older pin\n")
    archive = _native_zip(tmp_path / "ovstage.zip", extra=metadata)
    m._extract_zip(str(archive), str(dest))
    assert not (dest / "include" / "ovstage" / "stale.h").exists()
    assert (dest / "include" / "ovstage" / "ovstage.h").is_file()
    for name, contents in metadata.items():
        assert (dest / name).read_text() == contents


def test_extract_zip_refuses_to_delete_an_unrelated_directory(tmp_path):
    dest = tmp_path / "my_deps"
    dest.mkdir()
    (dest / "important.txt").write_text("someone else's tree\n")
    archive = _native_zip(tmp_path / "ovstage.zip")
    with pytest.raises(SystemExit, match="refusing to overwrite"):
        m._extract_zip(str(archive), str(dest))
    assert (dest / "important.txt").is_file()


def test_extract_zip_accepts_an_empty_directory(tmp_path):
    dest = tmp_path / "empty"
    dest.mkdir()
    archive = _native_zip(tmp_path / "ovstage.zip")
    m._extract_zip(str(archive), str(dest))
    assert (dest / "include" / "ovstage" / "ovstage.h").is_file()


def test_supported_platforms_match_the_published_archives():
    # The build system passes PLATFORM straight through, so these must be the archive tokens.
    assert m.SUPPORTED_PLATFORMS == (
        "manylinux_2_35_x86_64", "manylinux_2_35_aarch64", "windows-x86_64")


def test_main_preserves_a_neighboring_archive(tmp_path, monkeypatch):
    platform = "windows-x86_64"
    neighbor = tmp_path / m._zip_name(m.OVSTAGE_VERSION, platform)
    neighbor.write_bytes(b"another download owns this file")
    archive = _native_zip(tmp_path / "fixture.zip", extra={"bin/padding": "x" * 5000})
    monkeypatch.setattr(m, "OVSTAGE_ZIP_URL", archive.as_uri())
    monkeypatch.setattr(m, "OVSTAGE_DIGESTS", {platform: m._sha256(archive)})
    monkeypatch.delenv("OVSTAGE_SKIP_FETCH", raising=False)
    monkeypatch.setattr(sys, "argv", ["fetch_ovstage_release.py", "--platform", platform,
                                     "--dest", str(tmp_path / "sdk")])
    m.main()
    assert (tmp_path / "sdk/include/ovstage/ovstage.h").is_file()
    assert neighbor.read_bytes() == b"another download owns this file"
    assert not list(tmp_path.glob(".ovstage-*"))

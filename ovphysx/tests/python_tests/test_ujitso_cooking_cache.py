# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""UJITSO cooked-collider cache: end-to-end behavior through the ovphysx loader.

Regression coverage for NVBugs 6262606. Without the UJITSO plugins loaded, the cooking
service finds no ``carb::ujitso::IRegistry`` and silently cooks uncached, re-cooking every
collider on every launch. The loader therefore loads the UJITSO plugins (local, in-process,
no Hub/Nucleus/GRPC) and persists cooked colliders to a cache directory the application
provides.

The cache location is passed to ovphysx as application config
(``PhysXConfig(cooked_collider_cache_dir=...)``). ovphysx does not read environment variables
and does not choose a persistent location on the app's behalf. With the field unset it cooks to
a process-private temp directory that is discarded at shutdown (NVBugs 6504275).

Each "launch" is a separate ovphysx process (mirrors IsaacLab launching the same
workflow repeatedly): process-global PhysX/Carbonite state is fresh, and the only
thing shared between launches is the on-disk cache directory. The assertions use the
content-addressed datastore's on-disk footprint because ovphysx does not surface
the per-cook ``resultSource`` hit/miss flag to Python.

The scene ``data/boxes_falling_on_groundplane.usda`` has a dozen ``Mesh`` prims with
``physxConvexHullCollisionAPI`` (``physics:approximation = "convexHull"``), so each
cold launch performs many convex-hull cooks.
"""

# @implements REQ-CAPI-CACHE-001
# @covers AC-1 AC-5
# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-8

import argparse
import os
import signal
import subprocess
import sys

import pytest
from test_utils import load_usd_with_ovstage

_COOK_OK = "OVPHYSX_UJITSO_COOK_OK"
_PYTHON_ATEXIT_OK = "OVPHYSX_PYTHON_ATEXIT_OK"
_NATIVE_SHUTDOWN_OK = "OVPHYSX_NATIVE_SHUTDOWN_OK"
_WINDOWS_CONTROL_C_EXIT = 0xC000013A
_WORKER_KEEPALIVE = None

# data/ lives in tests/, one directory up from tests/python_tests/.
_DATA_USD = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "data",
    "boxes_falling_on_groundplane.usda",
)


def _dir_size(path: str) -> int:
    """Total bytes of all files under ``path`` (the UJITSO datastore footprint)."""
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def _cook_launch(cache_dir: str | None, enable_cooking: bool, env: dict | None = None,
                 mode: str = "cook", second_cache_dir: str | None = None,
                 cwd: str | None = None,
                 expect_returncodes: tuple[int, ...] = (0,)) -> subprocess.CompletedProcess:
    """Run one ovphysx 'launch' (the cook worker) in its own process and require the
    cook to complete and the process to exit cleanly. The cooked-collider cache
    directory is handed to the worker on the command line and passed into ovphysx as
    app config (``PhysXConfig.cooked_collider_cache_dir``). Pass ``cache_dir=None`` to
    leave the field unset and exercise the process-private fallback. Pass
    ``expect_returncodes`` to accept a deliberate non-zero exit (KeyboardInterrupt)."""
    if mode == "reconfigure" and (cache_dir is None or second_cache_dir is None):
        raise ValueError("reconfigure mode needs both cache_dir and second_cache_dir")

    argv = [sys.executable, os.path.abspath(__file__), "--worker",
            "--enable", "1" if enable_cooking else "0", "--mode", mode]
    if cache_dir is not None:
        argv += ["--cache-dir", cache_dir]
    if second_cache_dir is not None:
        argv += ["--second-cache-dir", second_cache_dir]
    proc = subprocess.run(
        argv,
        capture_output=True,
        text=True,
        timeout=600,
        env=env,
        cwd=cwd,
    )
    # The worker must reach completion (COOK_OK) and exit cleanly. A missing marker means
    # PhysX()/ovstage attach actually failed (e.g. UJITSO did not load and the loader
    # hard-errored). A nonzero exit code means the process did not shut down cleanly with
    # the UJITSO worker threads running (agent validation/scheduler + datastore write-back).
    assert _COOK_OK in proc.stdout, (
        f"cook worker did not complete (UJITSO may have failed to load; rc={proc.returncode}).\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    assert proc.returncode in expect_returncodes, (
        f"cook worker exited with rc={proc.returncode}, expected one of {expect_returncodes}.\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    return proc


def _private_cache_env(temp_root: os.PathLike[str] | str) -> dict[str, str]:
    """Environment with the OS temp dir redirected to ``temp_root``.

    Isolates the process-private cache so leftover assertions see only this run's
    directories. ``temp_root`` must already exist."""
    env = dict(os.environ)
    for var in ("TMPDIR", "TMP", "TEMP"):
        env[var] = str(temp_root)
    return env


def test_cooked_colliders_persist_and_are_reused(tmp_path):
    """Cold launch cooks and persists to disk. Warm launch reuses it (no re-cook)."""
    cache = str(tmp_path / "ujitso_cache")
    os.makedirs(cache, exist_ok=True)
    base = _dir_size(cache)

    _cook_launch(cache, enable_cooking=True)
    after_first = _dir_size(cache)
    assert after_first > base, (
        "first launch must write cooked-collider data to the persistent on-disk cache; "
        "if this fails, the UJITSO registry is not wired or the datastore degraded to "
        "memory-only (e.g. unwritable cache dir)"
    )

    cooked_bytes = after_first - base
    _cook_launch(cache, enable_cooking=True)
    after_second = _dir_size(cache)
    # A warm launch is served from the content-addressed cache, so it must not
    # re-cook: any growth is minor datastore bookkeeping, far below the cold
    # launch's cooked-collider payload.
    grew = after_second - after_first
    assert grew <= cooked_bytes // 2, (
        f"second launch should reuse the cache, not re-cook "
        f"(cold wrote {cooked_bytes} B; warm added {grew} B)"
    )


def test_cache_only_written_when_cooking_enabled(tmp_path):
    """Enabled cooking persists cooked-collider blobs. Disabled persists none."""
    on = str(tmp_path / "on")
    off = str(tmp_path / "off")
    os.makedirs(on, exist_ok=True)
    os.makedirs(off, exist_ok=True)

    _cook_launch(on, enable_cooking=True)
    _cook_launch(off, enable_cooking=False)

    on_bytes = _dir_size(on)
    off_bytes = _dir_size(off)
    assert on_bytes > off_bytes, (
        f"enabled cooking should persist cooked colliders ({on_bytes} B) far more than "
        f"disabled cooking ({off_bytes} B)"
    )


def _datastore_errors(proc) -> list:
    """Datastore error lines in a worker's output. Matched on the channel plus severity rather
    than the exact ``[Error] [omni.datastore]`` prefix, which carb owns and could reformat.
    Pinning the literal would let the assertion pass vacuously."""
    return [
        line for line in (proc.stdout + proc.stderr).splitlines()
        if "datastore" in line.lower() and "error" in line.lower()
    ]


def test_unset_cache_dir_is_quiet_and_persists_nothing(tmp_path):
    """No cache dir configured: no datastore errors, and nothing left behind.

    Regression coverage for NVBugs 6504275, where an unset field let carb.ujitso.default
    fall back to ``<interpreter dir>/cache/DerivedDataCache`` (the non-writable /usr/bin on a
    Linux venv) and log two [Error] lines per scene attach. ovphysx cooks to a
    process-private temp directory that is discarded at shutdown, so the documented
    "nothing is persisted" contract still holds.
    """
    temp_root = tmp_path / "tmp"
    temp_root.mkdir()
    proc = _cook_launch(None, enable_cooking=True, env=_private_cache_env(temp_root))

    assert _PYTHON_ATEXIT_OK in proc.stdout, (
        "the Python atexit handler did not destroy the live instance before interpreter "
        f"finalization\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    errors = _datastore_errors(proc)
    assert not errors, (
        f"an unset cooked_collider_cache_dir must not produce datastore errors: {errors}\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    leftovers = list(temp_root.glob("ovphysx-cache-*"))
    if sys.platform == "win32":
        # Windows refuses to unlink files the datastore may still have open, so cleanup is
        # best-effort there and the OS reaps the temp tree instead. POSIX unlinks regardless.
        return
    assert not leftovers, f"process-private cache was not cleaned up at shutdown: {leftovers}"


def test_unset_cache_dir_shuts_down_with_live_read_array(tmp_path):
    """A globally reachable read() array still reaches native ovphysx_shutdown()."""
    temp_root = tmp_path / "tmp"
    temp_root.mkdir()

    proc = _cook_launch(
        None,
        enable_cooking=True,
        env=_private_cache_env(temp_root),
        mode="read-array-exit",
    )

    assert _PYTHON_ATEXIT_OK in proc.stdout, (
        "Python atexit did not finish after destroying the instance with a live read array\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    assert _NATIVE_SHUTDOWN_OK in proc.stdout, (
        "native ovphysx_shutdown() was not invoked while a read() array remained reachable; "
        "cache removal alone does not prove direct-runtime teardown\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    if sys.platform != "win32":
        leftovers = list(temp_root.glob("ovphysx-cache-*"))
        assert not leftovers, f"live read-array exit left cache dirs behind: {leftovers}"


def test_unset_cache_dir_is_removed_after_keyboardinterrupt(tmp_path):
    """An uncaught KeyboardInterrupt still runs Python atexit before finalization."""
    temp_root = tmp_path / "tmp"
    temp_root.mkdir()
    expected_returncodes = (1, -signal.SIGINT, 128 + signal.SIGINT)
    if sys.platform == "win32":
        expected_returncodes += (_WINDOWS_CONTROL_C_EXIT,)

    proc = _cook_launch(
        None,
        enable_cooking=True,
        env=_private_cache_env(temp_root),
        mode="keyboardinterrupt",
        expect_returncodes=expected_returncodes,
    )

    assert _PYTHON_ATEXIT_OK in proc.stdout, (
        f"Python atexit did not run after KeyboardInterrupt\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    if sys.platform != "win32":
        leftovers = list(temp_root.glob("ovphysx-cache-*"))
        assert not leftovers, f"KeyboardInterrupt left process-private cache dirs behind: {leftovers}"


def test_unset_cache_dir_destroys_all_live_instances_at_exit(tmp_path):
    """Process exit drains every Python lifecycle token before cache removal."""
    temp_root = tmp_path / "tmp"
    temp_root.mkdir()

    proc = _cook_launch(
        None,
        enable_cooking=True,
        env=_private_cache_env(temp_root),
        mode="two-live-exit",
    )

    assert _PYTHON_ATEXIT_OK in proc.stdout, (
        "Python atexit did not finish destroying the live instances\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    # POSIX only: on Windows the datastore may still hold cache files open, so
    # removal is documented best-effort and leftovers are not a failure.
    if sys.platform != "win32":
        leftovers = list(temp_root.glob("ovphysx-cache-*"))
        assert not leftovers, f"multi-instance exit left cache dirs behind: {leftovers}"


def test_unwritable_configured_dir_falls_back_without_errors(tmp_path):
    """An unwritable configured directory degrades to the process-private cache, quietly.

    Warning and then letting carb retry the same unusable path would emit the very [Error]
    lines the warning claims are a graceful degradation.
    """
    if sys.platform == "win32":
        pytest.skip("chmod-based unwritability is not meaningful on Windows")
    if os.geteuid() == 0:
        pytest.skip("root ignores directory permissions")

    unwritable = tmp_path / "unwritable"
    unwritable.mkdir(mode=0o500)
    try:
        proc = _cook_launch(str(unwritable / "cache"), enable_cooking=True)
        errors = _datastore_errors(proc)
        assert not errors, (
            f"an unwritable configured cache dir must not produce datastore errors: {errors}\n"
            f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
        )
    finally:
        unwritable.chmod(0o700)


def test_configured_cache_dir_is_used_not_the_fallback(tmp_path):
    """An app-provided cooked_collider_cache_dir is used, and no temp cache is created."""
    temp_root = tmp_path / "tmp"
    temp_root.mkdir()
    configured = tmp_path / "configured"
    configured.mkdir()
    _cook_launch(str(configured), enable_cooking=True, env=_private_cache_env(temp_root))

    assert _dir_size(str(configured)) > 0, "cooked colliders were not persisted to the configured dir"
    assert not list(temp_root.glob("ovphysx-cache-*")), (
        "the process-private fallback must not be used when cooked_collider_cache_dir is set"
    )


def test_releasing_one_instance_keeps_the_shared_cache(tmp_path):
    """Releasing an instance must not remove the process-wide cache under a surviving one.

    The cache directory, the runtime and the UJITSO datastore are all process-wide, but each
    instance owns a CarboniteLoader. Cleanup from a per-instance shutdown would pull the
    directory out from under any instance still cooking.
    """
    temp_root = tmp_path / "tmp"
    temp_root.mkdir()
    proc = _cook_launch(
        None, enable_cooking=True, env=_private_cache_env(temp_root), mode="release-then-cook"
    )

    errors = _datastore_errors(proc)
    assert not errors, (
        f"cooking through the surviving instance hit the datastore: {errors}\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )


def test_relative_temp_dir_is_not_used(tmp_path):
    """A relative TMPDIR must not place the cache under the launch directory."""
    if sys.platform == "win32":
        pytest.skip("TMPDIR is not the temp-root selector on Windows")

    cwd = tmp_path / "cwd"
    cwd.mkdir()
    env = dict(os.environ)
    env["TMPDIR"] = "relative-temp"

    proc = _cook_launch(None, enable_cooking=True, env=env, cwd=str(cwd))

    assert not (cwd / "relative-temp").exists(), (
        "a relative TMPDIR must be rejected, not resolved against the launch cwd"
    )
    errors = _datastore_errors(proc)
    assert not errors, f"rejecting the temp root must not surface datastore errors: {errors}"


def test_preexisting_probe_entry_is_not_truncated(tmp_path):
    """A pre-existing entry at the old predictable probe path must survive untouched.

    A fixed ``.ovphysx_write_probe`` opened with a truncating ofstream would let a symlink
    planted there destroy the target. The probe name is unpredictable and the file is created
    with exclusive, no-follow semantics.
    """
    if sys.platform == "win32":
        pytest.skip("symlink creation needs elevation on Windows")

    configured = tmp_path / "configured"
    configured.mkdir()
    victim = tmp_path / "victim"
    victim.write_bytes(b"x" * 128)
    (configured / ".ovphysx_write_probe").symlink_to(victim)

    _cook_launch(str(configured), enable_cooking=True)

    assert victim.read_bytes() == b"x" * 128, "the symlink target was truncated"
    assert (configured / ".ovphysx_write_probe").is_symlink(), "the planted symlink was removed"


def test_non_ascii_cache_dir(tmp_path):
    """A configured cache directory outside ASCII must round-trip to the datastore."""
    configured = tmp_path / "cache-Muller-日本語"
    configured.mkdir()

    proc = _cook_launch(str(configured), enable_cooking=True)

    errors = _datastore_errors(proc)
    assert not errors, f"a non-ASCII cache dir must not surface datastore errors: {errors}"
    assert _dir_size(str(configured)) > 0, (
        f"cooked colliders were not persisted to the non-ASCII cache dir {configured}"
    )


def test_cache_dir_is_not_reconfigurable_in_process(tmp_path):
    """The configured directory is applied at first runtime bootstrap only.

    Documented behavior, pinned here so it cannot change silently: the datastore is built once,
    so a second PhysXConfig in the same process does not move the cache.
    """
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()

    _cook_launch(str(first), enable_cooking=True, mode="reconfigure", second_cache_dir=str(second))

    assert _dir_size(str(first)) > 0, "the first configured dir should hold the cooked colliders"
    assert _dir_size(str(second)) == 0, (
        "a later cooked_collider_cache_dir must not take effect in the same process; if this "
        "fails the datastore became reconfigurable and the documented restriction is stale"
    )


def _retain_for_atexit(*objects) -> None:
    """Keep objects reachable until interpreter exit so atexit, not GC, destroys them."""
    global _WORKER_KEEPALIVE
    _WORKER_KEEPALIVE = objects


def _print_python_atexit_ok() -> None:
    """Witness that the library atexit callback already ran (atexit is LIFO)."""
    import ovphysx.api as api

    _ = _WORKER_KEEPALIVE
    if api._PROCESS_LIFECYCLE_REFCOUNT != 0:
        sys.stderr.write(
            "process-lifecycle refcount still "
            f"{api._PROCESS_LIFECYCLE_REFCOUNT} after library atexit\n"
        )
        return
    print(_PYTHON_ATEXIT_OK, flush=True)


def _install_python_atexit_ok_marker() -> None:
    """Register the witness before the first PhysX() so LIFO runs the library first.

    Does not unregister ``_destroy_process_instances_at_exit``. Printing after
    that callback attests that ``_track_process_instance`` registered it.
    """
    import atexit

    atexit.register(_print_python_atexit_ok)


def _install_native_shutdown_witness() -> None:
    """Print when the ctypes ovphysx_shutdown entry is actually invoked."""
    import ovphysx.api as api

    original_shutdown = api._lib.ovphysx_shutdown

    def _witness_shutdown(*args, **kwargs):
        result = original_shutdown(*args, **kwargs)
        print(_NATIVE_SHUTDOWN_OK, flush=True)
        return result

    api._lib.ovphysx_shutdown = _witness_shutdown


def _worker(cache_dir: str | None, enable_cooking: bool, interrupt: bool = False) -> None:
    """Child process: one ovphysx launch that parses + cooks a convex-hull mesh scene.

    The cache directory is supplied to ovphysx through app config
    (``PhysXConfig.cooked_collider_cache_dir``), or omitted to use the process-private fallback.
    Prints the completion marker once cooking is done, then lets the process
    exit so ovphysx flushes the cooked-collider cache to disk. For the
    process-private fallback, the instance is retained until Python atexit to
    exercise interpreter-exit cleanup rather than ordinary garbage collection.
    """
    from ovphysx import PhysX, PhysXConfig

    config = PhysXConfig(
        cooked_collider_cache_dir=cache_dir,
        carbonite_overrides={"/physics/cooking/ujitsoCollisionCooking": enable_cooking},
    )
    PhysX.set_cpu_mode(True)
    if cache_dir is None:
        _install_python_atexit_ok_marker()
    physx = PhysX(config=config)
    load_usd_with_ovstage(physx, _DATA_USD)
    physx.wait_all()  # drain async collision cooking
    if cache_dir is None:
        # Keep this instance alive until process exit. Without this reference,
        # CPython may run __del__ as soon as _worker returns, which would test
        # ordinary garbage collection rather than interpreter-exit cleanup.
        _retain_for_atexit(physx)
    print(_COOK_OK, flush=True)
    if interrupt:
        raise KeyboardInterrupt


def _worker_two_live_exit() -> None:
    """Cook with one instance and retain two live instances until atexit."""
    from ovphysx import PhysX, PhysXConfig

    PhysX.set_cpu_mode(True)
    _install_python_atexit_ok_marker()
    first = PhysX(config=PhysXConfig())
    second = PhysX(config=PhysXConfig())
    load_usd_with_ovstage(first, _DATA_USD)
    first.wait_all()
    _retain_for_atexit(first, second)
    print(_COOK_OK, flush=True)


def _worker_read_array_exit() -> None:
    """Retain a read() Warp array through exit so shutdown cannot wait for its deleter."""
    from ovphysx import PhysX, PhysXConfig
    from ovphysx.types import SimObjectType

    PhysX.set_cpu_mode(True)
    _install_python_atexit_ok_marker()
    _install_native_shutdown_witness()
    physx = PhysX(config=PhysXConfig())
    load_usd_with_ovstage(physx, _DATA_USD)
    physx.wait_all()
    retained_arrays = []
    with physx.read(SimObjectType.RIGID_BODY, ["position"]) as result:
        if not result.groups:
            raise RuntimeError("read() returned no groups; cannot retain a process-lifecycle borrow")
        for group in result.groups:
            retained_arrays.extend(group.tensors)
    if not retained_arrays:
        raise RuntimeError("read() returned empty tensors; cannot retain a process-lifecycle borrow")
    _retain_for_atexit(physx, retained_arrays)
    print(_COOK_OK, flush=True)


def _worker_release_then_cook() -> None:
    """Release one instance, then cook through a surviving one.

    The process-private cache is process-wide, like the runtime and the UJITSO datastore.
    Releasing an instance must not remove it while another instance can still cook against it.
    The parent locates the directory by redirecting the temp root.
    """
    from ovphysx import PhysX, PhysXConfig

    PhysX.set_cpu_mode(True)
    first = PhysX(config=PhysXConfig())
    second = PhysX(config=PhysXConfig())
    load_usd_with_ovstage(first, _DATA_USD)
    first.wait_all()

    first.destroy()  # must not take the shared cache down with it

    load_usd_with_ovstage(second, _DATA_USD)
    second.wait_all()  # would hit a removed datastore path if destroy() had cleaned up
    print(_COOK_OK, flush=True)


def _worker_reconfigure(first_dir: str, second_dir: str) -> None:
    """Configure cache dir A, cook, release, then configure B in the same process and cook.

    Locks in the documented first-bootstrap-only behavior: the datastore is built once, so B is
    not adopted. The parent asserts A received the data and B stayed empty.
    """
    from ovphysx import PhysX, PhysXConfig

    PhysX.set_cpu_mode(True)
    first = PhysX(config=PhysXConfig(cooked_collider_cache_dir=first_dir))
    load_usd_with_ovstage(first, _DATA_USD)
    first.wait_all()
    first.destroy()

    second = PhysX(config=PhysXConfig(cooked_collider_cache_dir=second_dir))
    load_usd_with_ovstage(second, _DATA_USD)
    second.wait_all()
    print(_COOK_OK, flush=True)


if __name__ == "__main__":
    if "--worker" in sys.argv:
        parser = argparse.ArgumentParser()
        parser.add_argument("--worker", action="store_true")
        parser.add_argument("--cache-dir", default=None)
        parser.add_argument("--enable", choices=["0", "1"], default="1")
        parser.add_argument(
            "--mode",
            choices=[
                "cook",
                "release-then-cook",
                "reconfigure",
                "keyboardinterrupt",
                "two-live-exit",
                "read-array-exit",
            ],
            default="cook",
        )
        parser.add_argument("--second-cache-dir", default=None)
        ns = parser.parse_args()
        if ns.mode == "release-then-cook":
            _worker_release_then_cook()
        elif ns.mode == "two-live-exit":
            _worker_two_live_exit()
        elif ns.mode == "read-array-exit":
            _worker_read_array_exit()
        elif ns.mode == "reconfigure":
            if ns.cache_dir is None or ns.second_cache_dir is None:
                parser.error("--mode reconfigure requires --cache-dir and --second-cache-dir")
            _worker_reconfigure(ns.cache_dir, ns.second_cache_dir)
        else:
            _worker(
                ns.cache_dir,
                ns.enable == "1",
                interrupt=ns.mode == "keyboardinterrupt",
            )  # process exit flushes the cache to disk
    else:
        raise SystemExit("this module is a pytest test module; run it via pytest")

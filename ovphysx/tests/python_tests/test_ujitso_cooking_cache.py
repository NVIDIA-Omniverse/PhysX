# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""UJITSO cooked-collider cache: end-to-end behavior through the ovphysx loader.

Regression coverage for NVBugs 6262606. The kitless loader historically never
loaded the UJITSO plugins, so the cooking service found no ``carb::ujitso::IRegistry``
and silently cooked uncached -- re-cooking every collider on every launch. The fix
loads the UJITSO plugins (local, in-process; no Hub/Nucleus/GRPC) and persists cooked
colliders to a cache directory the application provides.

The cache location is passed to ovphysx as application config
(``PhysXConfig(cooked_collider_cache_dir=...)``) -- ovphysx does not read environment
variables and does not choose a location on the app's behalf.

Each "launch" is a separate ovphysx process (mirrors IsaacLab launching the same
workflow repeatedly): process-global PhysX/Carbonite state is fresh, and the only
thing shared between launches is the on-disk cache directory. We assert on the
content-addressed datastore's on-disk footprint because ovphysx does not surface
the per-cook ``resultSource`` hit/miss flag to Python.

The scene ``data/boxes_falling_on_groundplane.usda`` has a dozen ``Mesh`` prims with
``physxConvexHullCollisionAPI`` (``physics:approximation = "convexHull"``), so each
cold launch performs many convex-hull cooks.
"""

import argparse
import os
import subprocess
import sys
from test_utils import load_usd_with_ovstage

_COOK_OK = "OVPHYSX_UJITSO_COOK_OK"

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


def _cook_launch(cache_dir: str, enable_cooking: bool) -> None:
    """Run one ovphysx 'launch' (the cook worker) in its own process and require the
    cook to complete and the process to exit cleanly. The cooked-collider cache
    directory is handed to the worker on the command line and passed into ovphysx as
    app config (``PhysXConfig.cooked_collider_cache_dir``) -- no environment variable
    selects the cache location."""
    proc = subprocess.run(
        [
            sys.executable, os.path.abspath(__file__), "--worker",
            "--cache-dir", cache_dir,
            "--enable", "1" if enable_cooking else "0",
        ],
        capture_output=True,
        text=True,
        timeout=600,
    )
    # The worker must reach completion (COOK_OK) and exit cleanly. A missing marker means
    # PhysX()/ovstage attach actually failed (e.g. UJITSO did not load and the loader
    # hard-errored); a nonzero exit code means the process did not shut down cleanly with
    # the UJITSO worker threads running (agent validation/scheduler + datastore write-back).
    assert _COOK_OK in proc.stdout, (
        f"cook worker did not complete (UJITSO may have failed to load; rc={proc.returncode}).\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    assert proc.returncode == 0, (
        f"cook worker did not exit cleanly (rc={proc.returncode}).\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )


def test_cooked_colliders_persist_and_are_reused(tmp_path):
    """Cold launch cooks + persists to disk; warm launch reuses it (no re-cook)."""
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
    """Enabled cooking persists cooked-collider blobs; disabled persists none."""
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


def _worker(cache_dir: str, enable_cooking: bool) -> None:
    """Child process: one ovphysx launch that parses + cooks a convex-hull mesh scene.

    The cache directory is supplied to ovphysx purely through app config
    (``PhysXConfig.cooked_collider_cache_dir``). Prints the completion marker once
    cooking is done, then lets the process exit so ovphysx flushes the cooked-collider
    cache to disk. The instance is intentionally not released (mirrors
    cpu_tests/conftest.py: "Carbonite shutdown can hang; the OS reclaims at process
    exit")."""
    from ovphysx import PhysX, PhysXConfig

    config = PhysXConfig(
        cooked_collider_cache_dir=cache_dir,
        carbonite_overrides={"/physics/cooking/ujitsoCollisionCooking": enable_cooking},
    )
    PhysX.set_cpu_mode(True)
    physx = PhysX(config=config)
    load_usd_with_ovstage(physx, _DATA_USD)
    physx.wait_all()  # drain async collision cooking
    print(_COOK_OK, flush=True)


if __name__ == "__main__":
    if "--worker" in sys.argv:
        parser = argparse.ArgumentParser()
        parser.add_argument("--worker", action="store_true")
        parser.add_argument("--cache-dir", required=True)
        parser.add_argument("--enable", choices=["0", "1"], required=True)
        ns = parser.parse_args()
        _worker(ns.cache_dir, ns.enable == "1")  # process exit flushes the cache to disk
    else:
        raise SystemExit("this module is a pytest test module; run it via pytest")

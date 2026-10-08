# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""What ``ovphysx_set_cpu_mode(true)`` does and does not keep away from the CUDA driver.

ovphysx's own code touches no driver under CPU mode. TWO dependencies outside ovphysx can still
open it, and this file is scoped to that boundary: it asserts what ovphysx controls and records
what it does not.

1. **Warp** (Python read and write paths). ``read()`` and ``write()`` expose ``warp.array``, and
   BUILDING an array, on any device including ``cpu``, initializes the Warp runtime, which
   loads the driver on a CUDA-enabled Warp build. Importing Warp is not what does it; constructing
   the first array is. There is no runtime switch: ``WARP_DISABLE_CUDA`` does not exist, and
   ``CUDA_VISIBLE_DEVICES=""`` hides the devices but still loads the driver. ``wp_is_cuda_enabled()``
   is a build-time property of the native library, so the answer is decided by which Warp is
   INSTALLED.

2. **ovstage** (stage load). Loading a stage pulls in ovstage's own
   ``bin/plugins/gpucompute/libomni.gpucompute-cuda.plugin.so``, which ``dlopen``s ``libcuda.so.1``
   at runtime -- it carries no ``DT_NEEDED`` on the driver, so this is the plugin's doing, not link
   time. ovphysx neither ships nor loads that plugin, and every
   ``cudaShim::isCudaAvailable()`` caller on the create/attach path sits behind ``isCpuMode()``.

WHY THESE ASSERT DELTAS, NOT ABSOLUTES. An absolute "the driver was never loaded" check is a test
of ovstage, not of ovphysx. It fails on the stage load, before reaching the step it means to
cover, and reads as an ovphysx defect. Each case therefore samples ``/proc/self/maps`` before and
after the operation it owns, so a driver opened by a dependency cannot mask or manufacture a
result. ``test_startup_and_instance_creation_are_driverless`` keeps one absolute check, over the
only span ovphysx fully owns: process start through instance creation.

Pinned in FRESH subprocesses, because whether the driver was ever opened is a once-per-process fact
that any earlier Warp or ovstage use in the same interpreter would destroy.
"""

# @implements REQ-CAPI-CPU-001
# @covers AC-5
# @maps_to TEST-CAPI-CPU-001

import os
import subprocess
import sys

import pytest


_TEST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

pytestmark = pytest.mark.skipif(
    not sys.platform.startswith("linux"),
    reason="reads /proc/self/maps to observe whether the driver was ever loaded",
)


_CHILD = r"""
import os, sys
sys.path.insert(0, {test_dir!r})
from ovphysx import PhysX
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def driver_loaded():
    with open("/proc/self/maps") as maps:
        return "libcuda.so" in maps.read()


def _warp_build_has_cuda():
    # Whether the INSTALLED Warp was compiled with CUDA -- which is what decides the expected
    # answer below, and is not something ovphysx controls. Read straight off the native library,
    # which does not initialize the runtime and so does not itself open the driver.
    import ctypes
    name = "warp.dll" if sys.platform == "win32" else "libwarp.dylib" if sys.platform == "darwin" else "warp.so"
    try:
        import warp
        lib = ctypes.CDLL(os.path.join(os.path.dirname(warp.__file__), "bin", name))
        lib.wp_is_cuda_enabled.restype = ctypes.c_int
        lib.wp_is_cuda_enabled.argtypes = None
        return bool(lib.wp_is_cuda_enabled())
    except Exception:
        return None


# Sampled at each boundary so a case can attribute a driver load to the step that caused it
# rather than to whichever step happens to run last.
print("DRIVER_AT_START=%s" % driver_loaded())

PhysX.set_cpu_mode(True)
sdk = PhysX()
print("DRIVER_AFTER_INSTANCE=%s" % driver_loaded())

# data_path, not a path built from the test directory: the fixtures live in tests/data,
# one level ABOVE python_tests, so joining "data" onto the test directory named a
# directory that has never existed and the child died before any driver check ran.
load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
sdk.wait_all()
sdk.step(1.0 / 60.0)
sdk.wait_all()
print("DRIVER_BEFORE_READ=%s" % driver_loaded())

{body}

print("DRIVER_AFTER_READ=%s" % driver_loaded())
print("WARP_IMPORTED=%s" % ("warp" in sys.modules))
print("WARP_HAS_CUDA=%s" % _warp_build_has_cuda())
"""


def _run_child(body):
    script = _CHILD.format(test_dir=_TEST_DIR, body=body)
    proc = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=600,
        cwd=_TEST_DIR,
    )
    assert proc.returncode == 0, f"child failed:\n{proc.stdout}\n{proc.stderr}"
    _KEYS = (
        "DRIVER_AT_START=",
        "DRIVER_AFTER_INSTANCE=",
        "DRIVER_BEFORE_READ=",
        "DRIVER_AFTER_READ=",
        "WARP_IMPORTED=",
        "WARP_HAS_CUDA=",
    )
    out = dict(line.split("=", 1) for line in proc.stdout.splitlines() if line.startswith(_KEYS))
    assert out, f"child produced no verdict:\n{proc.stdout}\n{proc.stderr}"
    return out


def test_startup_and_instance_creation_are_driverless():
    """The span ovphysx fully owns, and the only one an absolute check belongs on.

    Process start through ``PhysX()`` is all ovphysx: every ``isCudaAvailable()`` caller on the
    create path is behind ``isCpuMode()``. Nothing external has run yet, so if the driver is mapped
    here it is ovphysx's doing and nobody else's.
    """
    out = _run_child("pass\n")
    assert out["DRIVER_AT_START"] == "False", "the driver was already mapped before ovphysx ran"
    assert out["DRIVER_AFTER_INSTANCE"] == "False", (
        "CPU mode must not open the CUDA driver during startup or instance creation"
    )


def test_cpu_mode_read_that_resolves_nothing_never_opens_the_cuda_driver():
    """The guarantee holds as long as the read has no array to build.

    A read that drains to END_OF_ITERATION with no group never reaches array construction, so the
    Warp runtime is never initialized and the READ opens no driver.

    Scoped to the read: the stage load before it may already have opened the driver via ovstage's
    gpucompute plugin, which ovphysx neither ships nor loads. Asserting an absolute here would fail
    on that load, before this case reaches the step it exists to cover.
    """
    # An articulation-joint read on a scene holding only rigid bodies: the session drains to
    # END_OF_ITERATION with no group, which is the shape that must stay driverless.
    out = _run_child(
        'with sdk.read(SimObjectType.ARTICULATION_JOINT, ["jointPosition"],\n'
        '               scope=ObjectScope.ALL) as r:\n'
        "    assert not r.groups, r.groups\n"
    )
    # The driver is the contract. The import is only the mechanism. Importing Warp without
    # building an array does not open the driver, so this must not be asserted the other way round.
    assert out["DRIVER_AFTER_READ"] == out["DRIVER_BEFORE_READ"], (
        "an empty read must not change whether the CUDA driver is loaded "
        f"(before={out['DRIVER_BEFORE_READ']}, after={out['DRIVER_AFTER_READ']})"
    )


def test_cpu_mode_non_empty_read_is_driverless_exactly_when_warp_is_built_without_cuda():
    """The guarantee is keepable; what it depends on is the Warp BUILD, not ovphysx.

    Asserted against the requirement rather than against today's default wheel. Whether the
    driver opens is decided by `wp_is_cuda_enabled()`, a compile-time constant in warp.so:

      - Warp built WITHOUT CUDA -> no driver, ever. That is the driverless deployment, and it
        is the case that must never regress.
      - Warp built WITH CUDA -> the driver opens when the first array is built. ovphysx cannot
        prevent it, so its obligation is to SAY so rather than let it be found in a trace.

    Pinning "the driver loads" unconditionally would have encoded the defect as the contract:
    a Warp fix, or a CI runner switching to the conda CPU-only build, would fail the test for
    doing the right thing.
    """
    out = _run_child(
        'with sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as r:\n'
        "    assert r.groups and r.groups[0].tensors\n"
    )
    assert out["WARP_IMPORTED"] == "True"

    has_cuda = out.get("WARP_HAS_CUDA")
    if has_cuda == "False":
        assert out["DRIVER_AFTER_READ"] == out["DRIVER_BEFORE_READ"], (
            "Warp is built without CUDA, so a non-empty read must not change whether the driver "
            f"is loaded (before={out['DRIVER_BEFORE_READ']}, after={out['DRIVER_AFTER_READ']})"
        )
    elif has_cuda == "True":
        if out["DRIVER_BEFORE_READ"] == "True":
            # ovstage's gpucompute plugin already opened it during the stage load, so Warp's own
            # contribution is no longer observable. Skipped rather than asserted: "still loaded"
            # would pass whatever Warp did, which is worse than admitting the arm did not run.
            pytest.skip("driver already loaded before the read (ovstage); Warp's effect not observable")
        assert out["DRIVER_AFTER_READ"] == "True", (
            "Warp is built with CUDA but building the first array did not open the driver -- Warp "
            "may have gained a driverless path. Re-check ADR-0023 and the ovphysx_set_cpu_mode "
            "wording; this is good news, not a failure of ovphysx."
        )
    else:
        pytest.skip("could not determine whether the installed Warp was built with CUDA")


def test_cpu_mode_non_empty_write_is_driverless_exactly_when_warp_is_built_without_cuda():
    """Building the first CPU write array has the same Warp-build boundary as read."""
    out = _run_child(
        "import warp as wp\n"
        'with sdk.write(SimObjectType.RIGID_BODY, "position", scope=ObjectScope.ALL) as w:\n'
        "    tensors = [t for g in w.groups for t in g.tensors]\n"
        "    assert tensors and all(t.size > 0 for t in tensors)\n"
        "    assert all(isinstance(t, wp.array) and t.device.is_cpu for t in tensors)\n"
    )
    assert out["WARP_IMPORTED"] == "True"

    has_cuda = out.get("WARP_HAS_CUDA")
    if has_cuda == "False":
        assert out["DRIVER_AFTER_READ"] == out["DRIVER_BEFORE_READ"], (
            "Warp is built without CUDA, so a non-empty write must not change whether the driver "
            f"is loaded (before={out['DRIVER_BEFORE_READ']}, after={out['DRIVER_AFTER_READ']})"
        )
    elif has_cuda == "True":
        if out["DRIVER_BEFORE_READ"] == "True":
            pytest.skip("driver already loaded before the write (ovstage); Warp's effect not observable")
        assert out["DRIVER_AFTER_READ"] == "True", (
            "Warp is built with CUDA but building the first write array did not open the driver -- "
            "Warp may have gained a driverless path. Re-check the CPU-mode wording."
        )
    else:
        pytest.skip("could not determine whether the installed Warp was built with CUDA")

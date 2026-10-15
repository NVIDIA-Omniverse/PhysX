# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""
Profiling sample -- publish an ovphysx run's profile zones to a consumer.

This sample demonstrates how to:
1. Select a profiler backend before the PhysX instance is created
2. Confirm the selection through the public config getter
3. Run a simulation and collect the resulting chrome trace

It defaults to the trace backend, which needs no external tool and leaves a JSON
file you can open in Perfetto or chrome://tracing, or parse from a script. Select
the NVTX backend instead to capture with Nsight Systems:

    OVPHYSX_PROFILER=nvtx nsys profile -t nvtx,cuda python profiling.py

With no backend selected the zones cost an atomic load and a branch, so an
unprofiled run pays almost nothing.
"""

# [tutorial-start]
# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1
# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-4
# @implements REQ-PYTHON-SAMPLE-002
# @covers AC-1 AC-2

import tempfile
from pathlib import Path

import ovphysx
from ovphysx import GlobalConfigKey, PhysX, ProfilerBackend, get_global_config, newton_schema_root, set_global_config
from ovphysx.samples.python_samples.schema_support import physx_schema_paths


def attach_scene(physx, usd_path, stage_name):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    # Register the external schemas before population builds USD's registry.
    ovstage.population.register_usd_schemas(
        [*[str(path) for path in physx_schema_paths()], str(newton_schema_root())]
    )
    stage = ovstage.Stage(stage_name)
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=ordinal))
        return stage
    except Exception:
        if physx.is_stage_attached(stage):
            physx.wait_task(physx.detach_ovstage())
        stage.destroy()
        raise

# Select a profiler backend.
#
# IMPORTANT: this has to be set at initialization, before the physics engine is
# created internally, because that is when the backend is bound and the PhysX
# profiler callback installed. There is no way to select one later in the process.
#
# Setting OVPHYSX_PROFILER=trace in the environment does the same thing without any
# code change, which is the usual way to profile an application you did not write.
# The environment variable overrides a backend selected in code, so an application
# that hard-codes one -- as this sample does -- can still be profiled another way.
trace_dir = Path(tempfile.mkdtemp(prefix="ovphysx-profiling-"))
trace_path = trace_dir / "ovphysx_sample.json"

set_global_config(GlobalConfigKey.PROFILER_BACKEND, ProfilerBackend.TRACE)
set_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH, str(trace_path))

# Prefer package data so a copied sample works. Fall back to the checked-in
# sample's adjacent data directory when package data is absent. Resolve it
# before creating an instance so a missing scene does not skip destroy().
usd_path = Path(ovphysx.__file__).resolve().parent / "samples" / "data" / "links_chain_sample.usda"
if not usd_path.is_file():
    usd_path = Path(__file__).resolve().parent.parent / "data" / "links_chain_sample.usda"
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

physx = PhysX()

# The effective selection is readable, so a harness can log what it is doing rather
# than guess from an empty capture. Read the destination back the same way: like the
# backend, OVPHYSX_PROFILER_OUTPUT overrides what this sample asked for, and the file
# is checked below.
backend = ProfilerBackend(get_global_config(GlobalConfigKey.PROFILER_BACKEND))
resolved_trace_path = Path(get_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH) or trace_path)
print(f"Profiler backend: {backend.name}")
print(f"Loading USD scene: {usd_path}")

# Teardown runs even if attach or stepping raises: leaving a stage attached or an
# instance alive would leak the runtime singleton and mask the original error.
stage = None
try:
    stage = attach_scene(physx, usd_path, "ovphysx-profiling-sample")
    physx.wait_all()

    # Every call below opens a profile zone, and the runtime's host-side zones and
    # the PhysX SDK zones nest inside them.
    dt = 1.0 / 60.0
    n_steps = 120  # 2 seconds at 60 Hz
    print(f"Simulating {n_steps} steps...")

    for i in range(n_steps):
        physx.step_sync(dt)

    print("Simulation complete.")
finally:
    if stage is not None:
        physx.wait_task(physx.detach_ovstage())
        stage.destroy()
    # The trace is written here, not earlier: the profiler shuts down when the last
    # instance goes away, and that is what flushes the file.
    physx.destroy()
    print("Cleanup complete")

# Only the trace backend leaves a file to check. Under NVTX the zones went to the
# capturing tool instead, and under none they were dropped, so the checks below are
# scoped to the backend that actually ran rather than to the one selected in code.
if backend == ProfilerBackend.TRACE:
    # The trace is the whole point of selecting this backend, so a missing or empty
    # file is a failure rather than a remark. This sample doubles as a test.
    if not resolved_trace_path.exists():
        raise SystemExit(f"the trace backend wrote no file to {resolved_trace_path}")
    trace_bytes = resolved_trace_path.stat().st_size
    if trace_bytes == 0:
        raise SystemExit(f"the trace backend wrote an empty file to {resolved_trace_path}")

    print(f"Chrome trace written: {resolved_trace_path} ({trace_bytes / 1e6:.1f} MB for {n_steps} steps)")
    print("Open a trace in Perfetto (https://ui.perfetto.dev) or chrome://tracing.")

# This sample cleans up after itself, because it runs in CI and traces are large.
# To keep one, point the backend at a path of your own instead:
#     OVPHYSX_PROFILER=trace OVPHYSX_PROFILER_OUTPUT=my_run.json python profiling.py
for leftover in trace_dir.glob("*"):
    leftover.unlink()
trace_dir.rmdir()

print(
    "\nTo capture with Nsight Systems instead of writing a trace:\n"
    "  OVPHYSX_PROFILER=nvtx nsys profile -t nvtx,cuda python profiling.py\n"
    "The timeline then shows the ovphysx_* API zones, the runtime's host-side\n"
    "zones and the PhysX SDK zones, correlated with CUDA work. The SDK's twelve\n"
    "cross-thread phases (Basic.simulate and its siblings) are not reported under\n"
    "any backend: carb::profiler cannot express a zone that closes on another\n"
    "thread."
)
# [tutorial-end]

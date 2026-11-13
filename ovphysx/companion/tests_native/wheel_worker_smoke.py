# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2
# @implements REQ-COMP-VEHICLE-001
# @covers AC-1
"""Exercise stopped sample workers against installed Linux wheels.

Runs without customer pxr, GPU initialization, or inherited native search paths.
"""
import argparse
from dataclasses import asdict
import importlib.util
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--companion", type=Path,
                        default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    if sys.platform != "linux":
        parser.error("This wheel regression is qualified on Linux.")
    companion = args.companion.resolve()
    spec = importlib.util.find_spec("ovphysx")
    if spec is None or spec.origin is None:
        raise RuntimeError("Install the ovphysx wheel before running this test")
    library = Path(spec.origin).parent / "lib/libovphysx.so"
    if not library.is_file():
        raise RuntimeError(f"Installed wheel library missing: {library}")
    env = {key: value for key, value in os.environ.items()
           if key not in ("LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH",
                          "OVSTAGE_LIBRARY_PATH_HINT", "OVPHYSX_LIB")}
    env["OVPHYSX_LIB"] = str(library)
    env["PYTHONPATH"] = str(companion / "python")
    sys.path.insert(0, str(companion / "python"))
    from ovphysx_companion._worker import run_worker
    from ovphysx_companion.mass import Request, Shape, ShapeKind

    request = asdict(Request((Shape("/Box", ShapeKind.BOX, (1, 2, 3), 10),)))
    request.pop("source")
    result = run_worker(companion / "samples/stopped_mass_editor.py", request, env=env)
    assert result["mass"] == 480, result
    assert result["diagonal_inertia"] == [2080, 1600, 800], result
    print("PASS stopped mass worker", flush=True)

    result = run_worker(companion / "samples/vehicle_authoring.py", {
        "points": [[-1, 0, -2], [1, 0, -2], [-1, 0, 2], [1, 0, 2]],
        "mass": 1200,
        "body_pose": [1, 0, 0, 0, 0, 0, 1],
        "wheel_pose": [2, 0, 3, 0, 0, 0, 1],
    }, env=env)
    assert result["masses"] == [300] * 4, result
    assert result["frame"] == [1, 0, 3, 0, 0, 0, 1], result
    print("PASS vehicle authoring worker", flush=True)

    result = subprocess.run(
        [sys.executable, str(companion / "samples/vehicle_control.py"), "--math-only"],
        env=env, text=True, capture_output=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    print("PASS vehicle math process", flush=True)

    maps = Path("/proc/self/maps").read_text()
    assert "libovphysx.so" not in maps and "libovstage.so" not in maps
    assert "pxr" not in sys.modules and "ovstage" not in sys.modules
    assert "ovphysx._bindings" not in sys.modules
    print("PASS parent native isolation", flush=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        print(error.stderr, file=sys.stderr)
        raise

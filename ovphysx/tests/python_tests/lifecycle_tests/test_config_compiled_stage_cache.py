# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Compiled ovstage cache mode: validation, lifecycle and per-attach resolution.

@implements REQ-CAPI-COMPILED-CACHE-001
@covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6

Runs in its own subprocess (see test_python_runtime.cmake). The test process runs
one create+destroy cycle. Each resolution case runs in a nested child process,
because the cooked-collider directory and the cooking setting are fixed when the
first instance starts the runtime.
"""

import ctypes
import stat
import subprocess
import sys
from pathlib import Path

import pytest

from ovphysx import CompiledStageCacheMode, GlobalConfigKey, PhysX, get_global_config, set_global_config
from test_utils import get_cuda_driver

_TIMEOUT_S = 120
_PYTHON_TESTS_DIR = Path(__file__).resolve().parents[1]
# test_utils imports the schema recipe from the samples tree, as conftest.py arranges.
_PYTHON_SAMPLES_DIR = _PYTHON_TESTS_DIR.parent / "python_samples"

# Attaches one small scene with INFO logging and prints the cache log line and the
# datastore directory. argv: cooked-collider cache directory ("" leaves it unset),
# CompiledStageCacheMode name, "on" or "off" for UJITSO collision cooking, "cpu" or
# "gpu", and the scene file in tests/data.
_ATTACH_CHILD = """
import logging
import sys

sys.path[:0] = [{python_tests_dir!r}, {python_samples_dir!r}]

from ovphysx import (
    CompiledStageCacheMode,
    GlobalConfigKey,
    LogLevel,
    PhysX,
    PhysXConfig,
    enable_python_logging,
    flush_log,
    get_global_config,
    set_global_config,
)
from test_utils import data_path, destroy_ovstage_test_attachments, load_usd_with_ovstage

cache_dir, mode, cooking, device, scene = sys.argv[1:6]
set_global_config(GlobalConfigKey.CPU_ONLY, device == "cpu")
if cache_dir:
    set_global_config(GlobalConfigKey.COOKED_COLLIDER_CACHE_DIRECTORY, cache_dir)
set_global_config(GlobalConfigKey.COMPILED_STAGE_CACHE, CompiledStageCacheMode[mode])

lines = []


class Collector(logging.Handler):
    def emit(self, record):
        message = record.getMessage()
        if "compiled stage cache:" in message:
            lines.append(message)


logger = logging.getLogger("ovphysx")
logger.setLevel(logging.DEBUG)
logger.addHandler(Collector())
enable_python_logging()
# The cooking setting has no global key; Carbonite overrides still go through PhysXConfig.
config = PhysXConfig(
    carbonite_overrides={{"/physics/cooking/ujitsoCollisionCooking": cooking == "on"}}
)
physx = PhysX(config=config)
# Set once the instance exists: the runtime's log sources take the level then.
set_global_config(GlobalConfigKey.LOG_LEVEL, LogLevel.INFO)
try:
    load_usd_with_ovstage(physx, data_path(scene))
    flush_log()
    for line in lines:
        print("CACHE_LOG=" + line, flush=True)
    print("CACHE_DIR=" + (get_global_config(GlobalConfigKey.COOKED_COLLIDER_CACHE_DIRECTORY) or ""), flush=True)
finally:
    destroy_ovstage_test_attachments(physx)
    physx.destroy()
"""


def test_compiled_stage_cache_round_trip_rejection_and_live_writes():
    key = GlobalConfigKey.COMPILED_STAGE_CACHE
    assert int(key) == 0x0000000E
    assert get_global_config(key) == CompiledStageCacheMode.AUTOMATIC

    for mode in CompiledStageCacheMode:
        set_global_config(key, mode)
        assert get_global_config(key) == mode

    set_global_config(key, CompiledStageCacheMode.READ_ONLY)
    for invalid in (-1, 4, 2**31, True, 2.0, "READ_ONLY"):
        with pytest.raises(ValueError):
            set_global_config(key, invalid)
        assert get_global_config(key) == CompiledStageCacheMode.READ_ONLY

    # A live instance does not freeze the key: each later attach reads it.
    set_global_config(GlobalConfigKey.CPU_ONLY, True)
    physx = PhysX()
    try:
        set_global_config(key, CompiledStageCacheMode.READ_WRITE)
        assert get_global_config(key) == CompiledStageCacheMode.READ_WRITE
    finally:
        physx.destroy()

    # The configured value outlives the last instance, like other global keys.
    assert get_global_config(key) == CompiledStageCacheMode.READ_WRITE
    set_global_config(key, CompiledStageCacheMode.AUTOMATIC)
    assert get_global_config(key) == CompiledStageCacheMode.AUTOMATIC


def _attach(cache_dir, mode, cooking="on", device="cpu", scene="minimal_scene.usda"):
    """The attach's cache log line and the datastore directory the SDK reports."""
    script = _ATTACH_CHILD.format(
        python_tests_dir=str(_PYTHON_TESTS_DIR), python_samples_dir=str(_PYTHON_SAMPLES_DIR)
    )
    completed = subprocess.run(
        [sys.executable, "-c", script, cache_dir, mode, cooking, device, scene],
        check=False,
        capture_output=True,
        text=True,
        timeout=_TIMEOUT_S,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    lines = [line for line in completed.stdout.splitlines() if line.startswith("CACHE_LOG=")]
    assert len(lines) == 1, completed.stdout + completed.stderr
    dirs = [line[len("CACHE_DIR="):] for line in completed.stdout.splitlines() if line.startswith("CACHE_DIR=")]
    assert len(dirs) == 1, completed.stdout + completed.stderr
    return lines[0], dirs[0]


def _attach_cache_line(cache_dir, mode, cooking="on", device="cpu", scene="minimal_scene.usda"):
    return _attach(cache_dir, mode, cooking, device, scene)[0]


@pytest.mark.parametrize(
    "configure_cache_dir, mode, cooking, resolved",
    [
        # AUTOMATIC stores only into an application-configured cooked-collider
        # directory, and only while UJITSO collision cooking is on.
        (False, "AUTOMATIC", "on", "disabled"),
        (True, "AUTOMATIC", "on", "read-write"),
        (True, "AUTOMATIC", "off", "disabled"),
        # Explicit modes apply with or without a configured directory.
        (True, "DISABLED", "on", "disabled"),
        (False, "READ_ONLY", "on", "read-only"),
        (False, "READ_WRITE", "on", "read-write"),
    ],
)
def test_attach_resolves_and_logs_the_configured_mode(tmp_path, configure_cache_dir, mode, cooking, resolved):
    cache_dir = str(tmp_path / "cooked") if configure_cache_dir else ""
    line = _attach_cache_line(cache_dir, mode, cooking)
    assert f"(mode {resolved}" in line, line
    assert (", automatic)" in line) == (mode == "AUTOMATIC"), line
    # A disabled mode never consults the cache; any other mode reports an outcome.
    assert ("compiled stage cache: disabled " in line) == (resolved == "disabled"), line


def test_automatic_stores_then_hits_through_the_configured_directory(tmp_path):
    # Two processes share one cooked-collider directory: the first stores into the
    # real datastore, and the second restores from it.
    cache_dir = str(tmp_path / "cache" / "cooked")
    first = _attach_cache_line(cache_dir, "AUTOMATIC")
    assert "compiled stage cache: stored (mode read-write, automatic)" in first, first
    # Each directory ovphysx created is its user's alone, whatever the umask.
    if sys.platform != "win32":
        for created in (Path(cache_dir), Path(cache_dir).parent):
            assert stat.S_IMODE(created.stat().st_mode) == 0o700, created
    second = _attach_cache_line(cache_dir, "AUTOMATIC")
    assert "compiled stage cache: hit (mode read-write, automatic)" in second, second


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX ownership and permission bits")
@pytest.mark.parametrize(
    "layout, trusted",
    [
        ("group-writable", False),
        ("world-writable", False),
        ("readable by others", False),
        ("symlink", False),
        ("group-writable parent", False),
        ("sticky world-writable parent", True),
    ],
)
def test_automatic_uses_only_a_private_directory(tmp_path, layout, trusted):
    # A hit deserializes PhysX binary data from the directory, so AUTOMATIC uses only one
    # that no other user can reach into or replace: owned by this user, closed to its group
    # and others, in parent directories that no one else can change, or sticky ones (as
    # /tmp is). READ_WRITE uses any configured directory.
    parent = tmp_path / "parent"
    parent.mkdir()
    parent_modes = {"group-writable parent": 0o770, "sticky world-writable parent": 0o1777}
    parent.chmod(parent_modes.get(layout, 0o700))
    cache_dir = parent / "cooked"
    cache_dir.mkdir()
    modes = {"group-writable": 0o770, "world-writable": 0o703, "readable by others": 0o755}
    cache_dir.chmod(modes.get(layout, 0o700))
    if layout == "symlink":
        link = parent / "link"
        link.symlink_to(cache_dir, target_is_directory=True)
        cache_dir = link
    line = _attach_cache_line(str(cache_dir), "AUTOMATIC")
    if trusted:
        assert "compiled stage cache: stored (mode read-write, automatic)" in line, line
        return
    assert "compiled stage cache: disabled (mode disabled, automatic)" in line, line
    line = _attach_cache_line(str(cache_dir), "READ_WRITE")
    assert "compiled stage cache: stored (mode read-write)" in line, line


# Two live instances in one process: the first configures the cooked-collider directory
# argv[1] and the second argv[2], and each attaches one small scene with the default
# AUTOMATIC mode. Prints each attach's cache log line and the reason AUTOMATIC gives.
_TWO_INSTANCES_CHILD = """
import logging
import sys

sys.path[:0] = [{python_tests_dir!r}, {python_samples_dir!r}]

from ovphysx import (
    GlobalConfigKey,
    LogLevel,
    PhysX,
    PhysXConfig,
    enable_python_logging,
    flush_log,
    set_global_config,
)
from test_utils import data_path, destroy_ovstage_test_attachments, load_usd_with_ovstage

lines = []


class Collector(logging.Handler):
    def emit(self, record):
        message = record.getMessage()
        if "compiled stage cache:" in message or "AUTOMATIC leaves" in message:
            lines.append(message)


logger = logging.getLogger("ovphysx")
logger.setLevel(logging.DEBUG)
logger.addHandler(Collector())
enable_python_logging()
set_global_config(GlobalConfigKey.CPU_ONLY, True)
instances = []
try:
    for cache_dir in sys.argv[1:3]:
        instances.append(PhysX(config=PhysXConfig(cooked_collider_cache_dir=cache_dir)))
        set_global_config(GlobalConfigKey.LOG_LEVEL, LogLevel.INFO)
        load_usd_with_ovstage(instances[-1], data_path("minimal_scene.usda"))
        flush_log()
        print("ATTACH=" + " | ".join(lines), flush=True)
        lines.clear()
finally:
    for physx in instances:
        destroy_ovstage_test_attachments(physx)
        physx.destroy()
"""


def test_automatic_judges_the_directory_the_datastore_uses(tmp_path):
    # The datastore keeps the directory it was built from while any instance lives, so a
    # second instance that names a private directory does not make AUTOMATIC trust the
    # shared one the datastore still reads.
    shared = tmp_path / "shared"
    private = tmp_path / "private"
    for directory, mode in ((shared, 0o770), (private, 0o700)):
        directory.mkdir()
        directory.chmod(mode)
    script = _TWO_INSTANCES_CHILD.format(
        python_tests_dir=str(_PYTHON_TESTS_DIR), python_samples_dir=str(_PYTHON_SAMPLES_DIR)
    )
    completed = subprocess.run(
        [sys.executable, "-c", script, str(shared), str(private)],
        check=False,
        capture_output=True,
        text=True,
        timeout=_TIMEOUT_S,
    )
    output = completed.stdout + completed.stderr
    assert completed.returncode == 0, output
    attaches = [line[len("ATTACH="):] for line in completed.stdout.splitlines() if line.startswith("ATTACH=")]
    assert len(attaches) == 2, output
    # Windows has no ownership or permission check, so there the shared directory is used.
    first = "stored (mode read-write" if sys.platform == "win32" else "disabled (mode disabled"
    assert f"compiled stage cache: {first}, automatic)" in attaches[0], attaches[0]
    assert "compiled stage cache: disabled (mode disabled, automatic)" in attaches[1], attaches[1]
    reason = f"datastore was not built from the cooked-collider cache directory '{private}'"
    assert reason in attaches[1], attaches[1]


@pytest.mark.parametrize("configure_cache_dir, outcome", [(False, "unavailable"), (True, "stored")])
def test_cooking_off_sets_up_no_cache_directory(tmp_path, configure_cache_dir, outcome):
    # With collision cooking off ovphysx sets up no cache directory and leaves the setting
    # alone. An explicit mode stores into a directory the application configured. Without
    # one the datastore would fall back to Carbonite's application directory, so local
    # caching stays off and the cache reports itself unavailable.
    cache_dir = str(tmp_path / "cooked") if configure_cache_dir else ""
    line, directory = _attach(cache_dir, "READ_WRITE", "off")
    assert f"compiled stage cache: {outcome} (mode read-write)" in line, line
    if configure_cache_dir:
        assert directory == cache_dir, directory
    else:
        # The setting holds carb.ujitso.default's own fallback, which nothing uses.
        assert not Path(directory).name.startswith("ovphysx-cache-"), directory


def _cuda_device_count():
    try:
        cuda = get_cuda_driver()
    except OSError:
        return 0
    count = ctypes.c_int(0)
    if cuda.cuInit(0) != 0 or cuda.cuDeviceGetCount(ctypes.byref(count)) != 0:
        return 0
    return count.value


# A dynamic body with an SDF mesh collider.
_SDF_BODY = """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene"
    {
    }

    def Mesh "Body" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"]
    )
    {
        uniform token physics:approximation = "sdf"
        uniform int physxSDFMeshCollision:sdfResolution = 16
        uniform int physxSDFMeshCollision:sdfSubgridResolution = 0
        point3f[] points = [
            (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5),
            (-0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, 0.5, 0.5), (-0.5, -0.5, 0.5)
        ]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 1, 2, 3, 1, 5, 6, 2, 3, 2, 6, 7, 0, 3, 7, 4, 1, 0, 4, 5, 5, 4, 7, 6]
        double3 xformOp:translate = (0, 0, 2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""


def test_sdf_meshes_cooked_on_another_device_are_not_restored(tmp_path):
    # SDF meshes cook on the GPU when a CUDA device is usable and on the CPU otherwise,
    # and the two cooks differ. A GPU process sharing the directory with a CPU-only
    # process stores its own result instead of restoring the CPU-only one's.
    if _cuda_device_count() == 0:
        pytest.skip("needs a CUDA device")
    scene = tmp_path / "sdf_body.usda"
    scene.write_text(_SDF_BODY, encoding="utf-8")
    cache_dir = str(tmp_path / "cooked")
    for device, result in (("cpu", "stored"), ("cpu", "hit"), ("gpu", "stored"), ("gpu", "hit")):
        line = _attach_cache_line(cache_dir, "AUTOMATIC", device=device, scene=str(scene))
        assert f"compiled stage cache: {result} (mode read-write, automatic)" in line, (device, line)


def _boxes_on_triangle_mesh(vertices_per_side=256, boxes_per_side=4, half_extent=8.0):
    """Boxes resting on a static triangle-mesh grid. The grid's vertex-to-triangle table, which
    cooking with GPU data adds, takes more than a megabyte: an allocation of its own."""
    n = vertices_per_side
    step = 2 * half_extent / (n - 1)
    points = ", ".join(
        f"({-half_extent + i * step:.6g}, {-half_extent + j * step:.6g}, 0)" for j in range(n) for i in range(n)
    )
    indices = []
    for j in range(n - 1):
        for i in range(n - 1):
            a, b, c, d = j * n + i, j * n + i + 1, (j + 1) * n + i, (j + 1) * n + i + 1
            indices += [a, b, d, a, d, c]
    counts = ", ".join(["3"] * (len(indices) // 3))
    faces = ", ".join(map(str, indices))
    spacing = 2 * half_extent / (boxes_per_side + 1)
    boxes = "".join(
        f"""
    def Cube "Box_{y}_{x}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
    )
    {{
        double size = 0.5
        float physics:mass = 1
        double3 xformOp:translate = ({-half_extent + (x + 1) * spacing:.6g}, {-half_extent + (y + 1) * spacing:.6g}, 0.3)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}"""
        for y in range(boxes_per_side)
        for x in range(boxes_per_side)
    )
    return f"""#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{{
    def PhysicsScene "PhysicsScene"
    {{
    }}

    def Mesh "Ground" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
    )
    {{
        uniform token physics:approximation = "none"
        point3f[] points = [{points}]
        int[] faceVertexCounts = [{counts}]
        int[] faceVertexIndices = [{faces}]
    }}
{boxes}
}}
"""


# Attaches a scene DirectGPU with INFO logging and simulates a second. Prints the attach's cache
# log line, every error logged, and the bodies' poses as hex. argv: cooked-collider cache
# directory ("" leaves it unset), CompiledStageCacheMode name, and the scene file.
_SIMULATE_GPU_CHILD = """
import logging
import sys
import warnings

sys.path[:0] = [{python_tests_dir!r}, {python_samples_dir!r}]

from ovphysx import (
    CompiledStageCacheMode,
    GlobalConfigKey,
    LogLevel,
    PhysX,
    PhysXConfig,
    enable_python_logging,
    flush_log,
    set_global_config,
)
from test_utils import destroy_ovstage_test_attachments, load_usd_with_ovstage, read_rigid_body_poses

cache_dir, mode, scene = sys.argv[1:4]
if cache_dir:
    set_global_config(GlobalConfigKey.COOKED_COLLIDER_CACHE_DIRECTORY, cache_dir)
set_global_config(GlobalConfigKey.COMPILED_STAGE_CACHE, CompiledStageCacheMode[mode])

lines = []


class Collector(logging.Handler):
    def emit(self, record):
        message = record.getMessage()
        if "compiled stage cache:" in message:
            lines.append("CACHE_LOG=" + message)
        elif record.levelno >= logging.ERROR:
            lines.append("ERROR=" + message)


logger = logging.getLogger("ovphysx")
logger.setLevel(logging.DEBUG)
logger.addHandler(Collector())
enable_python_logging()
# DirectGPU has no global key; Carbonite overrides still go through PhysXConfig.
with warnings.catch_warnings():
    warnings.simplefilter("ignore", DeprecationWarning)
    physx = PhysX(config=PhysXConfig(carbonite_overrides={{"/physics/suppressReadback": True}}))
set_global_config(GlobalConfigKey.LOG_LEVEL, LogLevel.INFO)
try:
    load_usd_with_ovstage(physx, scene)
    for _ in range(60):
        physx.step_sync(1.0 / 60.0)
    poses = read_rigid_body_poses(physx)
    flush_log()
    for line in lines:
        print(line, flush=True)
    print("POSES=" + poses.tobytes().hex(), flush=True)
finally:
    destroy_ovstage_test_attachments(physx)
    physx.destroy()
"""


def _simulate_gpu(cache_dir, mode, scene):
    """The attach's cache log lines, the errors logged and the bodies' poses after a second."""
    script = _SIMULATE_GPU_CHILD.format(
        python_tests_dir=str(_PYTHON_TESTS_DIR), python_samples_dir=str(_PYTHON_SAMPLES_DIR)
    )
    completed = subprocess.run(
        [sys.executable, "-c", script, cache_dir, mode, str(scene)],
        check=False,
        capture_output=True,
        text=True,
        timeout=_TIMEOUT_S,
    )
    output = completed.stdout + completed.stderr
    assert completed.returncode == 0, output
    stdout = completed.stdout.splitlines()
    lines = [line[len("CACHE_LOG="):] for line in stdout if line.startswith("CACHE_LOG=")]
    errors = [line[len("ERROR="):] for line in stdout if line.startswith("ERROR=")]
    errors += [line for line in output.splitlines() if "PhysX error" in line and not line.startswith("ERROR=")]
    poses = [line[len("POSES="):] for line in stdout if line.startswith("POSES=")]
    assert len(poses) == 1, output
    return lines, errors, poses[0]


def test_triangle_meshes_restored_by_another_process_simulate_on_the_gpu(tmp_path):
    # PhysX binary serialization leaves the vertex-to-triangle tables of a triangle mesh cooked
    # with GPU data at the addresses they had in the process that stored the bundle. The GPU
    # narrowphase copies them when it uploads the mesh and indexes them for contacts at the
    # mesh's vertices, so a process restoring the bundle crashed in its first step or failed
    # with CUDA error 700. A bundle carries each triangle mesh as its cooked stream instead:
    # every restore simulates exactly as the scene loaded without the cache.
    if _cuda_device_count() == 0:
        pytest.skip("needs a CUDA device")
    scene = tmp_path / "boxes_on_triangle_mesh.usda"
    scene.write_text(_boxes_on_triangle_mesh(), encoding="utf-8")
    cache_dir = str(tmp_path / "cooked")
    _, errors, ordinary = _simulate_gpu("", "DISABLED", scene)
    assert not errors, errors
    for result in ("stored", "hit", "hit"):
        lines, errors, poses = _simulate_gpu(cache_dir, "READ_WRITE", scene)
        assert any(f"compiled stage cache: {result} (mode read-write)" in line for line in lines), (result, lines)
        assert not errors, (result, errors)
        assert poses == ordinary, result


# Two lifecycles in one process, each attaching one scene with READ_WRITE: on Linux the
# last destroy shuts Carbonite down and the next instance starts it again. argv:
# cooked-collider cache directory.
_RESTART_CHILD = """
import sys

sys.path[:0] = [{python_tests_dir!r}, {python_samples_dir!r}]

from ovphysx import CompiledStageCacheMode, GlobalConfigKey, LogLevel, PhysX, set_global_config
from test_utils import data_path, destroy_ovstage_test_attachments, load_usd_with_ovstage

set_global_config(GlobalConfigKey.CPU_ONLY, True)
set_global_config(GlobalConfigKey.COOKED_COLLIDER_CACHE_DIRECTORY, sys.argv[1])
set_global_config(GlobalConfigKey.COMPILED_STAGE_CACHE, CompiledStageCacheMode.READ_WRITE)
for scene in ("minimal_scene.usda", "simple_physics_scene.usda"):
    physx = PhysX()
    set_global_config(GlobalConfigKey.LOG_LEVEL, LogLevel.INFO)
    try:
        load_usd_with_ovstage(physx, data_path(scene))
    finally:
        destroy_ovstage_test_attachments(physx)
        physx.destroy()
"""


def test_stores_again_after_the_runtime_restarts(tmp_path):
    # The store's write lock waits through whichever tasking instance is current, so
    # the second lifecycle's store needs nothing the first lifecycle's Carbonite owned.
    # A later process finds what each lifecycle stored.
    cache_dir = str(tmp_path / "cooked")
    script = _RESTART_CHILD.format(
        python_tests_dir=str(_PYTHON_TESTS_DIR), python_samples_dir=str(_PYTHON_SAMPLES_DIR)
    )
    completed = subprocess.run(
        [sys.executable, "-c", script, cache_dir],
        check=False,
        capture_output=True,
        text=True,
        timeout=_TIMEOUT_S,
    )
    output = completed.stdout + completed.stderr
    assert completed.returncode == 0, output
    if sys.platform.startswith("linux"):
        # Windows keeps the framework until process exit.
        assert "[CarboniteLoader] Shutdown complete" in output, output
    for scene in ("minimal_scene.usda", "simple_physics_scene.usda"):
        line = _attach_cache_line(cache_dir, "READ_ONLY", scene=scene)
        assert "compiled stage cache: hit (mode read-only)" in line, (scene, line)

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for apply_build_number() in scripts/crossplatform_helpers.cmake.

The build number reaches the wheel name, the SDK archive name and the
Artifactory publish path through four separate scripts. They agree only
because they all call this one function, so it is worth pinning directly
rather than inferring it from a built artifact.
"""

import os
import shutil
import subprocess

import pytest

from pathlib import Path

HELPERS = Path(__file__).resolve().parents[2] / "scripts" / "crossplatform_helpers.cmake"

# Everything apply_build_number() reads. Cleared for every probe so a real value
# in the caller's shell -- CI_PIPELINE_ID is always set on a runner -- cannot
# turn a negative case green.
BUILD_NUMBER_VARS = ("OVPHYSX_BUILD_NUMBER", "CI_PIPELINE_ID")

pytestmark = pytest.mark.skipif(shutil.which("cmake") is None, reason="cmake not on PATH")


def _probe_env(overrides=None):
    """The real environment minus the variables under test, plus overrides.

    Inherit rather than start empty: cmake locates its own modules through the
    environment, and running it with none fails with `Could not find
    CMAKE_ROOT` before it ever reaches the script.
    """
    env = {k: v for k, v in os.environ.items() if k not in BUILD_NUMBER_VARS}
    env.update(overrides or {})
    return env


def _run_probe(script, cache=None, env=None):
    cmd = ["cmake"]
    for key, value in (cache or {}).items():
        cmd.append(f"-D{key}={value}")
    cmd += ["-P", str(script)]
    proc = subprocess.run(cmd, capture_output=True, text=True, env=_probe_env(env), check=True)
    # cmake writes message() to stderr, so both streams are searched.
    for line in (proc.stdout + proc.stderr).splitlines():
        if line.startswith("RESULT="):
            return line[len("RESULT="):].strip()
    raise AssertionError(f"no RESULT in cmake output:\n{proc.stdout}\n{proc.stderr}")


def _apply(tmp_path, base, env=None, cache=None):
    """Run apply_build_number(base) in cmake script mode and return the result."""
    script = tmp_path / "probe.cmake"
    script.write_text(
        f'include("{HELPERS.as_posix()}")\n'
        f'apply_build_number("{base}" OUT)\n'
        'message("RESULT=${OUT}")\n'
    )
    return _run_probe(script, cache=cache, env=env)


def test_no_build_number_leaves_version_bare(tmp_path):
    """AC-1: a developer build keeps the VERSION file value."""
    assert _apply(tmp_path, "0.6.3") == "0.6.3"


def test_ci_pipeline_id_is_appended(tmp_path):
    """AC-1: CI_PIPELINE_ID is the fallback source."""
    assert _apply(tmp_path, "0.6.3", env={"CI_PIPELINE_ID": "1234567"}) == "0.6.3.1234567"


def test_explicit_env_build_number_wins_over_pipeline_id(tmp_path):
    """AC-1: OVPHYSX_BUILD_NUMBER takes precedence.

    The parent pipeline forwards its own id this way, so the child pipeline's
    CI_PIPELINE_ID must not win.
    """
    env = {"OVPHYSX_BUILD_NUMBER": "555", "CI_PIPELINE_ID": "1234567"}
    assert _apply(tmp_path, "0.6.3", env=env) == "0.6.3.555"


def test_cache_variable_wins_over_environment(tmp_path):
    """AC-1: the CMake variable is consulted before either env var."""
    result = _apply(tmp_path, "0.6.3",
                    env={"CI_PIPELINE_ID": "1234567"},
                    cache={"OVPHYSX_BUILD_NUMBER": "42"})
    assert result == "0.6.3.42"


@pytest.mark.parametrize("value", ["abc", "12a", "1.2"])
def test_non_numeric_build_number_is_ignored(tmp_path, value):
    """AC-2: a build number that is not a PEP 440 release component is dropped."""
    assert _apply(tmp_path, "0.6.3", env={"OVPHYSX_BUILD_NUMBER": value}) == "0.6.3"


@pytest.mark.parametrize("base", ["0.6.3.rc1", "0.6.3-rc1", "0.6", "0.6.3.1234567"])
def test_non_release_base_version_is_returned_unchanged(tmp_path, base):
    """AC-2: only a bare X.Y.Z takes a build number.

    The last case matters most: applying twice would produce a five-component
    version, so the function has to be idempotent against its own output.
    """
    assert _apply(tmp_path, base, env={"CI_PIPELINE_ID": "1234567"}) == base


def test_build_number_precedes_the_local_segment(tmp_path):
    """AC-3: PEP 440 requires the local '+' segment to come last."""
    script = tmp_path / "order.cmake"
    script.write_text(
        f'include("{HELPERS.as_posix()}")\n'
        'apply_build_number("0.6.3" NUMBERED)\n'
        'set(ENV{CI_COMMIT_REF_NAME} "dev/someone/thing")\n'
        'set(ENV{CI_COMMIT_SHORT_SHA} "abc12345")\n'
        'append_branch_local_version("${NUMBERED}" "." OUT)\n'
        'message("RESULT=${OUT}")\n'
    )
    out = _run_probe(script, env={"CI_PIPELINE_ID": "1234567"})
    assert out == "0.6.3.1234567+dev.someone.thing.abc12345"

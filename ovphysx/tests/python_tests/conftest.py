# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Pytest configuration for PhysX tests.

Provides a session-scoped shared PhysX GPU instance for test isolation.
Carbonite and the embedded Python interpreter cannot be cleanly finalized and
re-initialized in the same process (DLL/SO init routines fail on reload), so a
single long-lived instance is created once per pytest session and reused across
all tests.  Simulation state is reset between tests via ``PhysX.reset_stage()`` +
``PhysX.wait_all()`` to provide per-test isolation without the costly
destroy/recreate cycle.

Note: Process-wide CPU/GPU mode is controlled via PhysX.set_cpu_mode(True), which
must be called before any instances are created. Tests that need CPU-only mode
run in a separate subprocess (cpu_tests/) so the process-level setting is clean.
"""

import importlib.util
import logging
import os
import sys
from collections import defaultdict
from datetime import datetime

import pytest


_remote_resolver_dll_directory_handles = []


def _get_pkg_dir():
    """Return the ovphysx package directory without importing it."""
    spec = importlib.util.find_spec("ovphysx")
    if spec and spec.origin:
        return os.path.dirname(spec.origin)
    return None


def _is_wheel_mode():
    """Check if the ovphysx package has bundled native libs (wheel / build_wheel layout).

    Uses importlib to locate the package without importing it, avoiding premature
    native bootstrap at pytest_configure time.
    """
    pkg_dir = _get_pkg_dir()
    if pkg_dir is None:
        return False
    lib_name = "ovphysx.dll" if sys.platform == "win32" else "libovphysx.so"
    return os.path.exists(os.path.join(pkg_dir, "lib", lib_name))


def _get_install_dir():
    """Return the _install/ directory path if it exists, else None."""
    test_dir = os.path.dirname(os.path.abspath(__file__))
    install_dir = os.path.normpath(os.path.join(test_dir, "..", "..", "_install"))
    if os.path.isdir(install_dir):
        return install_dir
    return None


def _remote_usd_tests_enabled():
    return bool(
        os.environ.get("OVPHYSX_S3_TEST_URI")
        and os.environ.get("OVPHYSX_AWS_ACCESS_KEY_ID")
        and os.environ.get("OVPHYSX_AWS_SECRET_ACCESS_KEY")
    )


def _prepare_remote_usd_resolver(install_dir):
    """Register and select OmniUsdResolver before OpenUSD caches a resolver."""
    plugins_dir = os.path.join(install_dir, "plugins")
    resources_dir = os.path.join(plugins_dir, "usd", "omni_usd_resolver", "resources")
    if not os.path.isdir(resources_dir):
        raise pytest.UsageError(f"OmniUsdResolver resources not found: {resources_dir}")

    if sys.platform == "win32":
        for directory in (os.path.join(install_dir, "bin"), plugins_dir):
            if not os.path.isdir(directory):
                raise pytest.UsageError(f"Remote USD DLL directory not found: {directory}")
            try:
                _remote_resolver_dll_directory_handles.append(os.add_dll_directory(directory))
            except OSError as exc:
                raise pytest.UsageError(f"Failed to add remote USD DLL directory {directory}: {exc}") from exc
        os.environ.setdefault("OVSTAGE_LIBRARY_PATH_HINT", os.path.join(install_dir, "bin"))

    import ovphysx

    # Publish the PhysX schema path before constructing Plug.Registry below.
    # This is pure Python environment setup and does not load native Carbonite.
    ovphysx.register_schema_paths()

    from pxr import Ar, Plug, Tf

    registry = Plug.Registry()
    registry.RegisterPlugins(resources_dir)
    resolver_type = Tf.Type.FindByName("OmniUsdResolver")
    if resolver_type.isUnknown:
        raise pytest.UsageError(f"OmniUsdResolver type was not registered from {resources_dir}")

    plugin = registry.GetPluginForType(resolver_type)
    if plugin is None:
        raise pytest.UsageError("No USD plugin owns the OmniUsdResolver type")

    # This must happen before GetUnderlyingResolver/Stage.Open: Ar chooses once.
    # Do not load the plugin or access the resolver here. Debug ovphysx and the
    # release-built resolver carry different Carbonite frameworks; eager loading
    # the resolver before PhysX initialization makes those frameworks collide.
    Ar.SetPreferredResolver("OmniUsdResolver")
    print(
        "[conftest] remote USD resolver: "
        f"preferred=OmniUsdResolver plugin={plugin.path} activation=pending",
        flush=True,
    )


def _activate_remote_usd_resolver():
    """Activate OmniUsdResolver after PhysX has initialized Carbonite."""
    from pxr import Ar, Plug, Tf

    resolver_type = Tf.Type.FindByName("OmniUsdResolver")
    if resolver_type.isUnknown:
        raise pytest.UsageError("OmniUsdResolver type is not registered")

    plugin = Plug.Registry().GetPluginForType(resolver_type)
    if plugin is None:
        raise pytest.UsageError("No USD plugin owns the OmniUsdResolver type")
    if not plugin.isLoaded and not plugin.Load():
        raise pytest.UsageError(f"Failed to load OmniUsdResolver plugin: {plugin.path}")

    resolver = Ar.GetUnderlyingResolver()
    using_default = isinstance(resolver, Ar.DefaultResolver)
    print(
        "[conftest] remote USD resolver: "
        f"plugin={plugin.path} loaded={plugin.isLoaded} default={using_default}",
        flush=True,
    )
    if using_default:
        raise pytest.UsageError("OpenUSD selected ArDefaultResolver instead of OmniUsdResolver")


def pytest_configure(config):
    """Auto-detect ovphysx environment and prepare native library loading.

    Wheel mode (lib/ovphysx.dll exists in the package directory):
      _bindings.py handles DLL search paths and library loading internally.
      No additional pytest-time setup is required.

    Install mode (_install/ exists, no bundled libs):
      Sets OVPHYSX_LIB so _bindings._load_library() finds the _install/ build.
      On Linux, preloads libpython with RTLD_GLOBAL so native plugins can
      resolve symbols without setting PYTHONHOME.
    """
    if _is_wheel_mode():
        return

    install_dir = _get_install_dir()
    if install_dir is None:
        return

    lib_dir = os.path.join(install_dir, "bin" if sys.platform == "win32" else "lib")
    lib_name = "ovphysx.dll" if sys.platform == "win32" else "libovphysx.so"
    lib_path = os.path.join(lib_dir, lib_name)

    if os.path.exists(lib_path):
        existing = os.environ.get("OVPHYSX_LIB")
        if existing:
            print(f"[conftest] OVPHYSX_LIB already set: {existing} (not overriding with {lib_path})")
        else:
            os.environ["OVPHYSX_LIB"] = lib_path

    # Preload libpython with RTLD_GLOBAL so Python-facing runtime dependencies
    # can resolve Python symbols at dlopen time.
    import ctypes

    if not hasattr(sys, "_omni_physx_preloaded_libs"):
        sys._omni_physx_preloaded_libs = []

    def _preload_required_library(lib_path: str, label: str) -> None:
        try:
            lib = ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL | RTLD_NODELETE)
        except OSError as exc:
            raise RuntimeError(f"Failed to preload {label} from {lib_path}: {exc}") from exc
        sys._omni_physx_preloaded_libs.append(lib)

    if sys.platform != "win32":
        RTLD_NODELETE = 0x01000

        # Preload libpython for dependencies that may resolve PyObject_* dynamically.
        import sysconfig

        cfg = sysconfig.get_config_vars()
        py_lib_name = cfg.get("LDLIBRARY") or cfg.get("DLLLIBRARY")
        if py_lib_name:
            # LDLIBRARY can be a static archive (e.g. conda's default
            # libpython3.X.a), which cannot be dlopen'd. Prefer a sibling shared
            # object when the reported name is a static archive.
            candidates = [py_lib_name]
            if py_lib_name.endswith(".a"):
                so_name = py_lib_name[: -len(".a")] + ".so"
                candidates = [so_name, so_name + ".1.0", py_lib_name]
            preloaded = False
            for d in filter(None, [cfg.get("LIBDIR"), cfg.get("LIBPL")]):
                for name in candidates:
                    full = os.path.join(d, name)
                    if os.path.exists(full):
                        _preload_required_library(full, name)
                        preloaded = True
                        break
                if preloaded:
                    break

    if _remote_usd_tests_enabled():
        _prepare_remote_usd_resolver(install_dir)


DETAILED_LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "detailed_log.txt")

ERROR_MARKERS = (
    # "[Error]",
    # "Runtime Error:",
    # "Coding Error:",
    # "ModuleNotFoundError:",
    "No CUDA context manager available",
    "[ErrorMarkerSentinel]",  # used by test_error_markers_catch_native_warnings
)

# ---------------------------------------------------------------------------
# Native log capture: route Carbonite CARB_LOG_* messages into a list so that
# ERROR_MARKERS can detect warnings/errors from the C++ runtime.
# A raw C-level callback is registered at session start; the makereport hook
# checks the collected records after each test.
# ---------------------------------------------------------------------------
_native_log_records: list = []

# Global test results tracking for session summary
_test_results = {}
_session_start_time = None
_session_end_time = None
_STATUS_PRIORITY = {"PASSED": 0, "SKIPPED": 1, "FAILED": 2}


def _first_error_marker(output: str):
    if not output:
        return None
    for marker in ERROR_MARKERS:
        if marker in output:
            return marker
    return None


def _append_detailed_log(lines):
    with open(DETAILED_LOG_PATH, "a", encoding="utf-8") as log_file:
        for line in lines:
            log_file.write(f"{line}\n")


def pytest_sessionstart(session):
    global _session_start_time, _test_results
    _session_start_time = datetime.now()
    _test_results = {}

    if os.path.exists(DETAILED_LOG_PATH):
        os.remove(DETAILED_LOG_PATH)

    # Native log callback registration is deferred to the _register_native_log_callback
    # fixture (below) because Carbonite must be initialized first (requires a PhysX instance).


def pytest_runtest_logstart(nodeid, location):
    _append_detailed_log(
        [
            "====================",
            f"TEST STARTED: {nodeid}",
            "====================",
            "",
        ]
    )


def pytest_runtest_logfinish(nodeid, location):
    _append_detailed_log(
        [
            "====================",
            f"TEST FINISHED: {nodeid}",
            "====================",
            "",
        ]
    )


def _record_test_result(nodeid, status, duration, error=None, skipped_reason=None):
    existing = _test_results.get(nodeid)
    if existing is None:
        _test_results[nodeid] = {
            "nodeid": nodeid,
            "status": status,
            "duration": duration,
            "error": error,
            "skipped_reason": skipped_reason,
        }
        return

    existing["duration"] += duration
    if _STATUS_PRIORITY[status] >= _STATUS_PRIORITY[existing["status"]]:
        existing["status"] = status
        existing["error"] = error if status == "FAILED" else existing["error"]
        existing["skipped_reason"] = skipped_reason if status == "SKIPPED" else existing["skipped_reason"]


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    nodeid = item.nodeid
    phase = report.when
    status = "PASSED"
    combined_output = ""
    if report.capstdout:
        combined_output += report.capstdout
    if report.capstderr:
        combined_output += report.capstderr
    # Include native Carbonite log messages captured via enable_python_logging()
    if _native_log_records:
        combined_output += "\n".join(_native_log_records)

    matched_marker = _first_error_marker(combined_output)
    if matched_marker and not report.failed:
        report.outcome = "failed"
        report.longrepr = (
            f"Test emitted native runtime warning matching ERROR_MARKERS: '{matched_marker}'\n"
            "See captured native log output above."
        )
    # Clear native log records after the final phase (teardown) so each test starts clean
    if phase == "teardown":
        _native_log_records.clear()

    if report.skipped:
        status = "SKIPPED"
    elif report.failed:
        status = "FAILED"

    lines = [
        f"STATUS={status}",
        f"TEST={nodeid}",
        f"PHASE={phase}",
    ]

    if report.skipped:
        lines.append(f"REASON={report.longrepr}")
    elif report.failed:
        lines.append(f"ERROR={report.longreprtext}")
        if matched_marker:
            lines.append(f"MATCHED_MARKER={matched_marker}")

    if report.capstdout:
        lines.append("STDOUT_BEGIN")
        lines.append(report.capstdout.rstrip())
        lines.append("STDOUT_END")
    if report.capstderr:
        lines.append("STDERR_BEGIN")
        lines.append(report.capstderr.rstrip())
        lines.append("STDERR_END")

    lines.append("")
    _append_detailed_log(lines)

    _record_test_result(
        nodeid=nodeid,
        status=status,
        duration=report.duration,
        error=report.longreprtext if report.failed else None,
        skipped_reason=str(report.longrepr) if report.skipped else None,
    )


def pytest_sessionfinish(session, exitstatus):
    """Generate summary table at the end of test session."""
    global _session_end_time
    _session_end_time = datetime.now()

    if not _test_results:
        return

    # Calculate statistics
    results = list(_test_results.values())
    total = len(results)
    passed = sum(1 for r in results if r["status"] == "PASSED")
    failed = sum(1 for r in results if r["status"] == "FAILED")
    skipped = sum(1 for r in results if r["status"] == "SKIPPED")
    total_duration = sum(r["duration"] for r in results)

    # Sort by duration for slowest tests
    sorted_by_duration = sorted(results, key=lambda x: x["duration"], reverse=True)

    # Categorize tests (basic heuristic based on test name)
    categories = defaultdict(lambda: {"passed": 0, "failed": 0, "skipped": 0, "tests": []})
    for result in results:
        name = result["nodeid"].split("::")[-1].lower()

        if any(x in name for x in ["smoke", "init", "default", "get_", "basic"]):
            cat = "Basic API Calls"
        elif any(x in name for x in ["variants", "permutation", "parametrize", "value_types"]):
            cat = "Parameter Permutations"
        elif any(x in name for x in ["error", "invalid", "raises", "fails", "boundary"]):
            cat = "Error Handling"
        elif any(x in name for x in ["integration", "multiple_usd", "clone_with", "reset_with"]):
            cat = "Integration Tests"
        elif any(x in name for x in ["lifecycle", "release", "use_after", "context_manager"]):
            cat = "Lifecycle Tests"
        elif any(x in name for x in ["stress", "large_array", "multiple_bindings", "rapid"]):
            cat = "Stress Tests"
        else:
            cat = "Other Tests"

        categories[cat]["tests"].append(result)
        if result["status"] == "PASSED":
            categories[cat]["passed"] += 1
        elif result["status"] == "FAILED":
            categories[cat]["failed"] += 1
        elif result["status"] == "SKIPPED":
            categories[cat]["skipped"] += 1

    # Build summary
    summary_lines = [
        "",
        "=" * 80,
        "#" * 80,
        "#" + " " * 78 + "#",
        "#" + "                          TEST EXECUTION SUMMARY".center(78) + "#",
        "#" + " " * 78 + "#",
        "#" * 80,
        "",
        f"Session Start Time: {_session_start_time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"Session End Time:   {_session_end_time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"Total Duration:     {total_duration:.2f} seconds",
        "",
        "=" * 80,
        "                              OVERALL STATISTICS",
        "=" * 80,
        f"Total Tests:    {total}",
        f"Passed:         {passed:<4} ({passed/total*100:.2f}%)" if total > 0 else "Passed:         0",
        f"Failed:         {failed:<4} ({failed/total*100:.2f}%)" if total > 0 else "Failed:         0",
        f"Skipped:        {skipped:<4} ({skipped/total*100:.2f}%)" if total > 0 else "Skipped:        0",
        "=" * 80,
        "",
        "=" * 80,
        "                           ALL TESTS WITH RESULTS",
        "=" * 80,
        f"{'#':<5} | {'Status':<8} | {'Duration':<9} | Test Name",
        "=" * 80,
    ]

    # Add all tests
    for idx, result in enumerate(results, 1):
        test_name = result["nodeid"]
        status = result["status"]
        duration = f"{result['duration']:.3f}s"
        summary_lines.append(f"{idx:<5} | {status:<8} | {duration:<9} | {test_name}")

    summary_lines.extend(
        [
            "=" * 80,
            "",
        ]
    )

    # Failed tests details
    failed_tests = [r for r in results if r["status"] == "FAILED"]
    if failed_tests:
        summary_lines.extend(
            [
                "=" * 80,
                "                              FAILED TESTS DETAILS",
                "=" * 80,
            ]
        )
        for idx, result in enumerate(results, 1):
            if result["status"] == "FAILED":
                error_lines = result["error"].split("\n") if result["error"] else ["No error message"]
                # Get first meaningful error line
                error_msg = next((line.strip() for line in error_lines if line.strip()), "Unknown error")
                summary_lines.extend(
                    [
                        f"[FAILED #{idx:03d}] {result['nodeid']}",
                        f"    Error: {error_msg[:100]}{'...' if len(error_msg) > 100 else ''}",
                        f"    Duration: {result['duration']:.3f}s",
                        "",
                    ]
                )
        summary_lines.extend(
            [
                "=" * 80,
                "",
            ]
        )

    # Skipped tests details
    skipped_tests = [r for r in results if r["status"] == "SKIPPED"]
    if skipped_tests:
        summary_lines.extend(
            [
                "=" * 80,
                "                             SKIPPED TESTS DETAILS",
                "=" * 80,
            ]
        )
        for idx, result in enumerate(results, 1):
            if result["status"] == "SKIPPED":
                summary_lines.extend(
                    [
                        f"[SKIPPED #{idx:03d}] {result['nodeid']}",
                        f"    Reason: {result['skipped_reason'][:100] if result['skipped_reason'] else 'Unknown'}",
                        "",
                    ]
                )
        summary_lines.extend(
            [
                "=" * 80,
                "",
            ]
        )

    # Category breakdown
    summary_lines.extend(
        [
            "=" * 80,
            "                         TESTS GROUPED BY CATEGORY",
            "=" * 80,
            "",
        ]
    )

    for cat_name in sorted(categories.keys()):
        cat_data = categories[cat_name]
        total_cat = cat_data["passed"] + cat_data["failed"] + cat_data["skipped"]

        if cat_data["failed"] == 0 and cat_data["skipped"] == 0:
            status_icon = "[PASS]"
            status_text = "All PASSED"
        elif cat_data["failed"] > 0:
            status_icon = "[WARN]"
            status_text = f"{cat_data['passed']} PASSED, {cat_data['failed']} FAILED"
        else:
            status_icon = "[WARN]"
            status_text = f"{cat_data['passed']} PASSED, {cat_data['skipped']} SKIPPED"

        summary_lines.append(f"{status_icon} {cat_name.upper()} ({total_cat} tests)")
        summary_lines.append(f"   - {status_text}")

        # List failed tests in category
        failed_in_cat = [r for r in cat_data["tests"] if r["status"] == "FAILED"]
        if failed_in_cat:
            for result in failed_in_cat:
                test_name = result["nodeid"].split("::")[-1]
                summary_lines.append(f"     - {test_name}")

        summary_lines.append("")

    summary_lines.extend(
        [
            "=" * 80,
            "",
        ]
    )

    # Slowest tests
    summary_lines.extend(
        [
            "=" * 80,
            "                            SLOWEST 10 TESTS",
            "=" * 80,
        ]
    )

    for idx, result in enumerate(sorted_by_duration[:10], 1):
        test_name = result["nodeid"].split("::")[-1]
        status = result["status"]
        duration = result["duration"]
        summary_lines.append(f"{idx:2}. {test_name:<50} {duration:6.3f}s  {status}")

    summary_lines.extend(
        [
            "=" * 80,
            "",
        ]
    )

    # Quick reference
    summary_lines.extend(
        [
            "=" * 80,
            "                         QUICK REFERENCE COMMANDS",
            "=" * 80,
            "Rerun all tests:",
            "  pytest python_tests/",
            "",
        ]
    )

    if failed_tests:
        summary_lines.extend(
            [
                "Rerun only failed tests:",
                "  pytest --lf python_tests/",
                "",
            ]
        )
        if len(failed_tests) == 1:
            failed_nodeid = failed_tests[0]["nodeid"]
            summary_lines.extend(
                [
                    "Rerun specific failed test:",
                    f"  pytest {failed_nodeid}",
                    "",
                ]
            )

    summary_lines.extend(
        [
            "Run with verbose output:",
            "  pytest -vv python_tests/",
            "=" * 80,
            "",
            "#" * 80,
            "#" + " " * 78 + "#",
            "#" + "                            END OF TEST SUMMARY".center(78) + "#",
            "#" + " " * 78 + "#",
            "#" * 80,
            "",
        ]
    )

    _append_detailed_log(summary_lines)


_native_log_cb_registered = False
_conftest_native_log_cb = None  # prevent GC of the ctypes callback


@pytest.fixture(autouse=True)
def _register_native_log_callback():
    """Register Carbonite log callback once Carbonite is initialized.

    This autouse fixture ensures that native CARB_LOG_* messages are captured
    into _native_log_records for ERROR_MARKERS detection.  Registration is
    deferred to test time (not session start) because Carbonite must be
    initialized first — which only happens when a PhysX instance is created.

    Uses ovphysx_register_log_callback directly (not enable_python_logging)
    to avoid interfering with tests that assert on the Python bridge state.
    """
    global _native_log_cb_registered, _conftest_native_log_cb
    if _native_log_cb_registered:
        return
    try:
        from ovphysx._bindings import _lib, ovphysx_log_fn

        @ovphysx_log_fn
        def _cb(level, message, user_data):
            text = message.decode("utf-8", errors="replace") if message else ""
            if level >= 2:  # WARNING or above
                _native_log_records.append(text)

        _conftest_native_log_cb = _cb
        _lib.ovphysx_register_log_callback(_cb, None)
        _native_log_cb_registered = True
    except Exception:
        pass


@pytest.fixture(scope="session")
def _gpu_session_instance():
    """Session-wide GPU PhysX instance (internal).

    Created once, reused across all tests. Carbonite/Python cannot be
    re-initialized after destroy, so one long-lived instance avoids the
    problematic destroy/recreate cycle.

    We intentionally do NOT call release() at session end — Carbonite
    shutdown can hang when plugins fail to re-initialize during
    teardown.  The OS reclaims all resources when the process exits.

    The GPU tensor-binding tests rely on GPU-resident state, so this
    session opts into DirectGPU (PxSceneFlag::eENABLE_DIRECT_GPU_API).
    Since 0.4.x ovphysx no longer auto-enables /physics/suppressReadback;
    DirectGPU is workflow-specific (Isaac-Lab-style tensor pipelines)
    and must be opted into explicitly via Carbonite settings.
    """
    from ovphysx import PhysX, PhysXConfig

    physx = PhysX(
        config=PhysXConfig(
            carbonite_overrides={
                "/physics/suppressReadback": True,
            }
        ),
    )
    if _remote_usd_tests_enabled():
        _activate_remote_usd_resolver()
    yield physx
    try:
        from test_utils import destroy_ovstage_test_attachments

        destroy_ovstage_test_attachments(physx)
    except Exception:
        pass
    try:
        physx.reset_stage()
        physx.wait_all()
    except RuntimeError:
        logging.getLogger(__name__).warning(
            "physx session teardown: reset_stage() failed; process shutdown will reclaim remaining resources"
        )


@pytest.fixture(scope="function")
def physx_sdk(_gpu_session_instance):
    """Provide the shared GPU-mode PhysX instance for each test.

    Reuses the session-scoped instance and resets simulation state
    (USD stages, bindings) after each test for isolation.

    Teardown is deliberately resilient: if a test leaves the instance
    with failed pending operations (e.g. error-path tests), we drain
    and reset on a best-effort basis to prevent cascading failures.
    """
    yield _gpu_session_instance
    try:
        _gpu_session_instance.wait_all()
    except RuntimeError:
        pass
    try:
        from test_utils import destroy_ovstage_test_attachments

        destroy_ovstage_test_attachments(_gpu_session_instance)
    except Exception:
        pass
    try:
        _gpu_session_instance.reset_stage()
        _gpu_session_instance.wait_all()
    except RuntimeError:
        logging.getLogger(__name__).warning(
            "physx_sdk teardown: reset_stage() failed — instance may be degraded for subsequent tests"
        )
    try:
        from test_utils import destroy_ovstage_test_attachments

        destroy_ovstage_test_attachments(_gpu_session_instance)
    except Exception:
        pass

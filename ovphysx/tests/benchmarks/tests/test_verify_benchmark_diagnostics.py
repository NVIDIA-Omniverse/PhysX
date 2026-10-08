# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# DEPRECATED (tensor-binding-deprecation): the binding-create-probe validators retire with the binding; generic ones stay.

# @implements REQ-CAPI-BENCHMARK-002
# @covers AC-1 AC-2 AC-3 AC-4
#
# @implements REQ-CAPI-BENCHMARK-003
# @covers AC-1 AC-2

import json
import tempfile
import unittest
from pathlib import Path

import verify_benchmark_diagnostics as vbd

OVPHYSX_ROOT = Path(__file__).resolve().parents[3]
EXPECTED_CMD = "Smoke.no_op_2T"
REPORT = (
    "Name                                      Avg (us)           Memory\n"
    "----                                      --------           ------\n"
    "Smoke.no_op_2T                              1060 (+/-12)         0\n"
)
DIAGNOSTICS_ROW = {
    "schema_version": 1,
    "cmd": EXPECTED_CMD,
    "all_steps_count": 100,
    "all_steps_mean_us": 1062.5,
    "all_steps_sd_us": 14.25,
    "all_steps_min_us": 1020,
    "all_steps_max_us": 1120,
}


def _jsonl(row):
    return json.dumps(row, separators=(",", ":")) + "\n"


def _binding_source(cuda_body=""):
    if not cuda_body:
        cuda_body = """
            uint32_t getNbSteps() const override { return 1; }
            uint32_t getNbRuns() const override {
                // Preserve first use.
                return 0;
            }
            Time::Second timedStep() override {
                if (mMeasured) {
                    throw std::runtime_error("once");
                }
                mMeasured = true;
                Time timer;
                for (size_t bindingIndex = 0; bindingIndex < kCartpoleBindingCases.size(); ++bindingIndex) {
                    const CartpoleBindingCase& bindingCase = kCartpoleBindingCases[bindingIndex];
                    requireSuccess(
                        mPhysX->createTensorBinding(
                            mBindings[bindingIndex], kArticulationPattern, bindingCase.tensorType),
                        bindingCase.name);
                    requireSuccess(
                        mBindings[bindingIndex].spec(mSpecs[bindingIndex]), bindingCase.name);
                }
                const Time::Second elapsed = timer.getElapsedSeconds();
                validateBindings();
                return elapsed;
            }
        """
    return f"""
        constexpr std::array<CartpoleBindingCase, 5> kCartpoleBindingCases = {{{{
            {{OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32, "actuation force"}},
            {{OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32, "position target"}},
            {{OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32, "velocity target"}},
            {{OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32, "DOF position"}},
            {{OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32, "DOF velocity"}},
        }}}};
        class CartpoleTensorBindingCreate : public BmBenchmark {{
        public:
            {cuda_body}
        }};
        class CartpoleTensorBindingCreate : public BmBenchmark {{
        public:
            uint32_t getNbSteps() const override {{ return 1; }}
            uint32_t getNbRuns() const override {{ return 0; }}
        }};
        Register<CartpoleTensorBindingCreate, true> registration(
            "Probe.cartpole_4096_tensor_binding_create");
    """


def _harness_source(index_expression="s * 3"):
    return f"""
        const bool singleActualRun = runCount == 0;
        else {{
            times.push_back(stepTimeMS);
            times.push_back(stepTimeMS);
            times.push_back(stepTimeMS);
            runCount = 3;
        }}
        if (singleActualRun) {{
            actualTimes.clear();
            for (uint32_t s = 0; s < stepCount; ++s) {{
                actualTimes.push_back(times[{index_expression}]);
            }}
            timingDiagnostics[i] = computeTimingDiagnostics(actualTimes);
        }}
    """


class ReportContractTests(unittest.TestCase):
    def test_accepts_exact_four_field_report(self):
        parsed = vbd.parse_single_regenerate_row(REPORT, EXPECTED_CMD)
        self.assertEqual(parsed, {"avg_us": 1060, "std_dev": 12, "memory_bytes": 0})

    def test_rejects_a_fifth_result_field(self):
        report = REPORT.replace("(+/-12)         0", "(+/-12)         0  extra")
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_regenerate_row(report, EXPECTED_CMD)

    def test_rejects_wrong_decorated_name(self):
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_regenerate_row(REPORT, "Smoke.no_op")

    def test_rejects_an_extra_result_row(self):
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_regenerate_row(REPORT + REPORT.splitlines()[2] + "\n", EXPECTED_CMD)

    def test_no_result_accepts_missing_or_header_only_report(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "report.txt"
            vbd.verify_no_report_row(str(path), EXPECTED_CMD)
            path.write_text(REPORT.rsplit("\n", 2)[0] + "\n", encoding="utf-8")
            vbd.verify_no_report_row(str(path), EXPECTED_CMD)

    def test_no_result_rejects_exact_decorated_row(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "report.txt"
            path.write_text(REPORT, encoding="utf-8")
            with self.assertRaises(vbd.DiagnosticsContractError):
                vbd.verify_no_report_row(str(path), EXPECTED_CMD)


class JsonlContractTests(unittest.TestCase):
    def test_accepts_exact_bounded_schema_and_ranges(self):
        parsed = vbd.parse_single_diagnostic_row(_jsonl(DIAGNOSTICS_ROW), EXPECTED_CMD, 100)
        self.assertEqual(parsed["all_steps_count"], 100)

    def test_rejects_raw_samples_or_any_extra_key(self):
        row = dict(DIAGNOSTICS_ROW)
        row["raw_samples"] = [1, 2, 3]
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_diagnostic_row(_jsonl(row), EXPECTED_CMD, 100)

    def test_rejects_float_schema_version(self):
        row = dict(DIAGNOSTICS_ROW, schema_version=1.0)
        with self.assertRaisesRegex(vbd.DiagnosticsContractError, "integer 1"):
            vbd.parse_single_diagnostic_row(_jsonl(row), EXPECTED_CMD, 100)

    def test_rejects_wrong_sample_count(self):
        row = dict(DIAGNOSTICS_ROW, all_steps_count=99)
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_diagnostic_row(_jsonl(row), EXPECTED_CMD, 100)

    def test_rejects_boolean_numeric_value(self):
        row = dict(DIAGNOSTICS_ROW, all_steps_sd_us=True)
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_diagnostic_row(_jsonl(row), EXPECTED_CMD, 100)

    def test_rejects_mean_outside_min_max(self):
        row = dict(DIAGNOSTICS_ROW, all_steps_mean_us=1200)
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_diagnostic_row(_jsonl(row), EXPECTED_CMD, 100)

    def test_rejects_population_sd_above_half_the_observed_range(self):
        row = dict(DIAGNOSTICS_ROW, all_steps_sd_us=50.001)
        with self.assertRaisesRegex(vbd.DiagnosticsContractError, "exceeds half"):
            vbd.parse_single_diagnostic_row(_jsonl(row), EXPECTED_CMD, 100)

    def test_rejects_multiple_jsonl_rows(self):
        text = _jsonl(DIAGNOSTICS_ROW) + _jsonl(DIAGNOSTICS_ROW)
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.parse_single_diagnostic_row(text, EXPECTED_CMD, 100)


class EmptySidecarTests(unittest.TestCase):
    def test_requires_existing_byte_empty_file(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "empty.jsonl"
            path.write_bytes(b"")
            vbd.verify_empty_sidecar(str(path))
            path.write_text("\n", encoding="utf-8")
            with self.assertRaises(vbd.DiagnosticsContractError):
                vbd.verify_empty_sidecar(str(path))

    def test_rejects_missing_file(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaises(vbd.DiagnosticsContractError):
                vbd.verify_empty_sidecar(str(Path(temp_dir) / "missing.jsonl"))


class BindingContractTests(unittest.TestCase):
    def test_accepts_exact_hidden_one_shot_structure(self):
        vbd.verify_hidden_binding_list("Probe.cartpole_4096_tensor_binding_create\n (hidden)\n")
        vbd.verify_binding_source(_binding_source(), _harness_source())

    def test_rejects_unhidden_list_row(self):
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.verify_hidden_binding_list("Probe.cartpole_4096_tensor_binding_create\n\n")

    def test_rejects_warm_default_runs(self):
        source = _binding_source().replace(
            "uint32_t getNbRuns() const override { return 0; }",
            "uint32_t getNbRuns() const override { return 5; }",
            1,
        )
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.verify_binding_source(source, _harness_source())

    def test_rejects_guard_after_timer_start(self):
        cuda_body = """
            uint32_t getNbSteps() const override { return 1; }
            uint32_t getNbRuns() const override { return 0; }
            Time::Second timedStep() override {
                Time timer;
                if (mMeasured) { throw std::runtime_error("once"); }
                mMeasured = true;
                return timer.getElapsedSeconds();
            }
        """
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.verify_binding_source(_binding_source(cuda_body), _harness_source())

    def test_rejects_non_failing_second_measurement_guard(self):
        cuda_body = """
            uint32_t getNbSteps() const override { return 1; }
            uint32_t getNbRuns() const override { return 0; }
            Time::Second timedStep() override {
                if (mMeasured) { return 0.0; }
                mMeasured = true;
                Time timer;
                return timer.getElapsedSeconds();
            }
        """
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.verify_binding_source(_binding_source(cuda_body), _harness_source())

    def test_rejects_extra_work_inside_binding_timer(self):
        source = _binding_source().replace(
            "const CartpoleBindingCase& bindingCase = kCartpoleBindingCases[bindingIndex];",
            "const CartpoleBindingCase& bindingCase = kCartpoleBindingCases[bindingIndex];\n"
            "consumeCpuTime();",
            1,
        )
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.verify_binding_source(source, _harness_source())

    def test_rejects_legacy_triplication_as_three_actual_samples(self):
        with self.assertRaises(vbd.DiagnosticsContractError):
            vbd.verify_binding_source(_binding_source(), _harness_source("s"))


class BuildConfigurationTests(unittest.TestCase):
    def test_parent_owns_required_benchmark_cuda_configuration(self):
        parent = (OVPHYSX_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        child = (OVPHYSX_ROOT / "tests" / "benchmarks" / "CMakeLists.txt").read_text(encoding="utf-8")
        windows_wrapper = (OVPHYSX_ROOT / "build.bat").read_text(encoding="utf-8")

        for token in (
            "OVPHYSX_REQUIRE_BENCHMARK_CUDA",
            "set(ENV{CUDACXX}",
            "check_language(CUDA)",
            "enable_language(CUDA)",
            "find_package(CUDAToolkit QUIET)",
            "add_library(ovphysx_benchmark_cuda INTERFACE)",
            "TARGET CUDA::cudart_static",
            "bin/nvcc",
            "lib64/libcudart_static.a",
            "lib/x64/cudart_static.lib",
            "Refusing to fall back to a host CUDA installation.",
        ):
            self.assertIn(token, parent)
        self.assertLess(
            parent.index("find_package(CUDAToolkit QUIET)"),
            parent.index('add_subdirectory("${OVRUNTIME_DIR}"'),
        )
        cuda_block = parent.split(
            "if(NOT WIN32 AND _OVPHYSX_BENCHMARK_CUDA_STAGED_OK)", 1
        )[1]
        self.assertLess(
            cuda_block.index("set(ENV{CUDACXX}"),
            cuda_block.index("check_language(CUDA)"),
        )
        self.assertLess(
            cuda_block.index("check_language(CUDA)"),
            cuda_block.index("enable_language(CUDA)"),
        )
        self.assertNotIn("find_package(CUDAToolkit", child)
        self.assertIn(
            "target_link_libraries(ovphysx_benchmarks PRIVATE ovphysx_benchmark_cuda)",
            child,
        )
        self.assertIn("--require-benchmark-cuda", windows_wrapper)
        self.assertIn(
            "-DOVPHYSX_REQUIRE_BENCHMARK_CUDA=ON",
            windows_wrapper,
        )

if __name__ == "__main__":
    unittest.main()

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-OVD-INVESTIGATION-001
maps_to: REQ-PYTHON-OVD-INVESTIGATION-001
type: integration
---

## Scenario

Run the installed wheel sample with an installed inspector from a separate working
directory. `tests/ovd_workflow/test_workflow.py` exercises real captures and errors.

## Given

- An installed OvPhysX wheel, its samples/data, and an explicit SDK `ovd-inspect` executable.
- A new artifact directory; CPU and GPU instances run in separate subprocesses.
- The external `physx-usd-schemas` provider, with both resource directories; Newton schemas are optional for this fixture.
- GPU runs require CUDA-backed live reads. A local host without CUDA reports a GPU skip; `--require-gpu` makes absence a failure.
- Real CPU capture and inspector responses are used for negative checks. Each case changes only a test-owned copy or one returned JSON document.

## When

- Execute `python tests/ovd_workflow/test_workflow.py --inspector <installed-executable> --sample <installed-package>/samples/python_samples/omnipvd_inspection.py --output-root <new-artifacts> --require-gpu`.
- For focused CPU iteration use `--cpu-only` instead. Pass both flags in an isolated argument test, with no runtime or inspector launch.
- Execute the sample CMake driver with offline dependency/process stand-ins in `tests/python_tests/test_ovd_workflow_mode.py`. Cross explicit/environment GPU-skip selection with required CUDA, including explicit OFF overriding environment ON.
- Exercise copied-Ninja fixture selection with a hostile PATH-first shim, no PATH Ninja, and absent repository-provisioned Ninja. Probe the copied canonical executable with `--version` in the same cleaned environment used by CMake; isolate a failed probe to check source/copied paths, exit and stdout/stderr diagnostics.
- Make `physx_schema_paths` raise `FileNotFoundError` with Newton present and absent; observe stage creation, schema registration, population and attachment actions.
- Make the sample's schema preflight fail with a new output path, before importing the runtime. Pass absolute and relative inspector overrides, including paths with spaces and a launch directory outside the project, through the CMake driver; also supply a missing override.
- Exercise an existing output directory, a file used as the parent directory, a missing inspector, a non-inspector executable, and `finalize_and_inspect` on a real instance with no active recording.
- Independently perturb component 1 by 1e-4 for each live endpoint (steps 1/10), each of the three bodies, and each of position/linearVelocity (12 subtests). Perturb Cube1 step 1 position in each component with both signs at twice the 1e-5 bound (six more subtests). Use a fresh output directory per case. Check optional Newton-root registration arguments and registration-before-population ordering with and without that package.
- Wrap the real inspector process, then change only its successful JSON response for duplicate scene context, incomplete frame 20, Carried dynamic pose, wrong source frame, or non-Carried kinematic pose. Also change a middle live kinematic step, make the dynamic final position match its first position and the returned pose, or append a byte to the copied capture after inspection.
- Check a short vector against a longer vector. Make `stop_recording` return while `is_recording` remains true, and fail if the inspector runs.
- Call the real `read_live` with constrained current-runtime `read` and `read_sync` signatures and dense ReadGroup fixtures for all three named bodies and both live vector attributes.
- Call `run` with constrained global-configuration and instance signatures in CPU/GPU modes. Collect returned attach/detach tasks; independently fail attach collection with and without a retained attachment, and fail detach collection.
- Independently supply a sparse index_map, a sparse prim_index_map, and duplicate groups with matching body names, keeping all other ReadGroup fields and vector/name fixtures valid.
- Call `inspect_json` with byte subprocess results for `--version`, summary, inspect and both comparison modes. Keep all other fixture fields valid while independently replacing the command's version with boolean, float, string, null or unsupported integers, or its tool/format with a wrong value. Exercise malformed, non-object and trailing JSON for every command family. No inspector process runs for these isolated guards.
- Encode a non-ASCII identity with `ensure_ascii=False` and UTF-8; a subprocess-boundary fake models Windows locale decoding when text mode is requested. Independently supply malformed UTF-8 stdout on success and malformed UTF-8 stderr on tool exit 2.
- Call the real `inspect_json` helper with isolated inspector process responses for legacy version-1 and generic version-2 Different comparisons, retaining distinct identities and numeric values. Exercise exit 1 with a wrong command, status, evidence flag or format; empty, malformed and non-object JSON; unsupported formats or versions; and exit 2 with valid Equal evidence.
- Return a negative subprocess exit code (-11) with otherwise valid summary JSON, swap the valid summary and inspect sibling formats, and return exit 2 with an empty stderr and a stdout-only error document.
- Return exit -9 with empty stderr and truncated UTF-8 stdout; request an unknown command with otherwise valid summary JSON.
- Cross version-1 and version-2 responses with the opposite legacy/generic comparison mode for both Equal and Different exits. Request `--attribute` or `--distance` alone, and supply a boolean version or nonstandard `NaN`, `Infinity` or `-Infinity` comparison value.
- Load the sibling runtime-free `test_inspection_helper.py` cases through the normal `test_workflow.py` runner, while the real workflow keeps using the sample resolved from the installed wheel.
- Compare the CPU/GPU captures' named dynamic body at ten aligned post-step frames using the installed SDK comparator. Save its optional annotation sidecar separately from the sample's `evidence.json` manifest.
- With the installed generic comparator, compare Cube1 and Cube2 positions at recorded frame 2 of the real CPU capture. Their distinct recorded poses must yield exit-1 Different evidence through `inspect_json`, with version-2 values and identities intact; preserve the annotation sidecar and capture hash.

## Then

- Stop precedes inspection; a post-inspection live read shows both bodies continuing to fall before teardown, and a real failed stop prevents inspection. The phase/step labels are format-only checks, not independent lifecycle evidence. Loading the sample with `runpy.run_path` creates no competing runtime (REQ AC-1).
- Global recording capability is enabled before instance creation, with the requested readback override retained. Both external schema roots are registered before population. Attach completion precedes recording; successful detach completion precedes stage destruction. A failed attach that still owns the stage is detached before release; failed detach never destroys the borrowed stage (REQ AC-1, AC-5).
- Both dynamic bodies fall by more than 0.1 distance units and finish with Y velocity below -1. Recorded frames 2/20 match live steps 1/10 for both bodies' position and velocity within 1e-5. The fixed kinematic body's ten live poses equal `(0,2,0)` and velocities equal zero. CPU evidence records CPU mode and CPU read devices without GPU scene settings; GPU evidence records the opposite mode, CUDA read devices and GPU scene settings (REQ AC-2).
- Dynamic samples are Exact with requested/source frame 2 or 20 in the summary's sole scene-mapped context/segment. Each body's stable resolved identity matches its exact name and PxRigidDynamic class; IDs differ between the three bodies. Command ordinals are positive and bounded by the summary, increase between dynamic endpoints, and remain unchanged for carried kinematic values. This producer's raw context 1 and null raw frame are not conflated with mapped scene clocks. The sample verifies both kinematic pose and velocity at requested frames 2/20 are Carried, pre-membership and without a source frame before claiming an initial snapshot. The report preserves the installed inspector tool/build/SDK/evidence version (REQ AC-3).
- Capture bytes hash to the report's SHA-256, existing capture bytes remain intact, and directory/tool/stop/mismatch failures exit unsuccessfully without a success report. Optional Newton schema routing precedes population and permits a codeless-only scene without the package (REQ AC-4, AC-5).
- All 18 perturbations fail with recorded/live disagreement and leave no `evidence.json`. The six component/sign cases detect missing Z, Y-only, one-sided and 9e-5 tolerance comparisons. Omitting velocity, an endpoint, a body, components beyond 0, or relaxing tolerance to 0.5 is also detected (REQ AC-2, AC-5).
- Every real-inspector JSON perturbation and live/capture mutation fails with its specific context, frame, Exact-write, source-frame, Carried-state, kinematic-motion, dynamic-fall or capture-hash diagnostic and leaves no `evidence.json`. Different vector lengths compare false. Recording still active after stop prevents inspector invocation and leaves no report (REQ AC-3, AC-4, AC-5).
- Live snapshots use the current synchronous type-based read API and preserve both vector attributes for the two dynamic bodies and fixed kinematic body, with the read device recorded; obsolete type/scope arguments to query-based `read` fail the focused regression (REQ AC-2, AC-3).
- Either sparse map is rejected with the dense-group diagnostic; duplicate matching-body rows are rejected as ambiguous rather than silently overwriting live state (REQ AC-2, AC-3).
- Unsupported tool-metadata and inspect versions, plus inspector exit 2 with otherwise valid comparison JSON, raise explicit errors. Conflicting CPU-only/required-GPU flags produce the final argument-parser conflict diagnostic before path resolution or runtime startup (REQ AC-5).
- A missing required PhysX schema provider propagates its original `FileNotFoundError` before any stage/register/populate/attach action, regardless of optional Newton availability (REQ AC-5).
- A schema preflight failure leaves the requested output path absent so installation can be repaired and the command retried. Relative inspector overrides reach the workflow relative to the original launch directory; a missing override requests correcting or unsetting that override (REQ AC-4, AC-5).
- The CMake driver selects only `--cpu-only` when the effective GPU-skip setting is true, otherwise only `--require-gpu` for required CUDA, otherwise neither. An explicit CMake OFF overrides environment ON (REQ AC-5).
- Offline provisioning regressions in `test_ovd_workflow_mode.py` execute the production custom target and C++ sample driver with real CMake configure/build/install operations. Both install the companion inspector in the selected build directory, including paths with spaces; configured/explicit directories override the environment, environment-only and default selection work, and external-inspector or run-only sample passes do not provision a tool (REQ AC-5).
- The relocation fixture prefers the original repository's provisioned Ninja before PATH fallback and gives a clear failed-copy probe diagnostic. Production `inspector_build_env` still prefers PATH Ninja; selection tests use inert bytes with mocked process launch, while the real provisioning case runs the audited standalone executable (REQ AC-5).
- Valid compare exit 1 returns unmodified Different version-1 or version-2 evidence, including values and identities; it is not a workflow success report. Wrong-command or wrong-status exit 1, absent comparison evidence, exit 2 even with Equal JSON, malformed/non-object output, and unsupported format/version all fail. Valid exit-0 legacy commands and version-2 generic comparison remain readable (REQ AC-6).
- Every command requires an exact supported integer version and its own correct tool/format; otherwise valid fixtures prevent one guard masking another. Comparison evidence must match its requested mode, paired generic flags are required, exit 0 requires Equal, and all three nonstandard numeric JSON constants fail decoding. Version-2 fixtures retain the actual nested measure, numeric component count, completion flags and decimal-string counters (REQ AC-6).
- Negative subprocess return codes remain failures despite valid JSON; swapped summary/inspect sibling formats are rejected. A stdout-only failed-process diagnostic is retained as the fallback when stderr is empty (REQ AC-6).
- Invalid UTF-8 in the failed-process stdout fallback becomes readable replacement diagnostics without hiding the negative exit; unknown commands reject otherwise valid summary evidence (REQ AC-6).
- UTF-8 identities survive without locale decoding or lossy replacement. Malformed UTF-8 stdout fails; malformed stderr becomes readable replacement diagnostics while preserving the tool's exit-2 failure (REQ AC-3, AC-6).
- The normal installed-wheel workflow gate runs the focused helper cases as well as the real capture cases; a helper failure fails that gate (REQ AC-6).
- The CPU/GPU comparison is Equal within 0.0001 m/s at ten samples. Its optional SDK annotation JSON equals the CLI result without changing either recording; it is not the sample manifest or a viewer import requirement (REQ AC-2, AC-4).
- The real CPU capture's Cube1/Cube2 position comparison reports Different with version 2, the matching recorded identity and position values, a nonzero distance, an equal annotation sidecar, and unchanged capture bytes (REQ AC-6).

The Python sample gate resolves the sample from the installed wheel and preserves
the captures/reports under `_build/test_results/ovd-workflow`.

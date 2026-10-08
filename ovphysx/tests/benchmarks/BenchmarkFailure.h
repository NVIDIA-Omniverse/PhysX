// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Explicit benchmark failure channel.
//
// The harness records a sample for every step it runs and emits it
// unconditionally. Without this channel a benchmark whose setup, operation
// sequence, correctness gate or baseline comparison failed could publish an
// unusually fast number and still exit successfully.
//
// A row calls bmRecordFailure() instead of only printing a diagnostic. The
// harness suppresses operationally failed rows before emit(), so failed work
// publishes no number rather than a misleading one. A local non-regenerate
// baseline breach is recorded after emit() reports its failed comparison. The
// process then exits non-zero, which the CMake driver turns into a hard
// failure (`FATAL_ERROR` on a non-zero pass exit code).
//
// FrameCore benchmark runs use --regenerate, so they never perform that local
// baseline comparison.
//
// Ordinary device gating is not a failure. A row that does not belong to the
// current pass returns false from isValid() and is reported Skipped, which is
// the convention for the Step.*_cpu / Step.*_gpu split. A suite may still fail
// closed when an explicitly unsupported global mode would make its device
// label or measured API invalid. ContactReport under DirectGPU is one such
// negative-control contract.

#ifndef BENCHMARK_FAILURE_H
#define BENCHMARK_FAILURE_H

#include <cstdint>

// Records a failure against `row`. `row` should be one exact registered
// benchmark name. Matching also accepts only the postfixes Harness.cpp appends:
// `_GPU`, `_<N>T`, or `_<N>T_GPU`.
void bmRecordFailure(const char* row, const char* format, ...);

// True when `registeredName` (postfix included) has a recorded failure.
bool bmRowHasFailure(const char* registeredName);

uint32_t bmFailureCount();

// Prints every recorded failure. No-op when there are none.
void bmPrintFailureSummary();

#endif

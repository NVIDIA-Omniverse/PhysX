// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace ovphysx {
namespace internal {

// Returns true when the host CPU and OS expose AVX (ymm) state.
// Always returns true on non-x86_64 builds (for example Linux aarch64).
bool cpuSupportsAvx();

} // namespace internal
} // namespace ovphysx

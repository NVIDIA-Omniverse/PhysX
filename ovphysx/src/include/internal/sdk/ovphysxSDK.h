// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Internal/private C API header (C, not C++).
// This header is NOT part of the public SDK; it should NOT be installed.

#pragma once

#include "ovphysx/ovphysx_export.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Private C API: load tensor plugins from a collection root directory.
OVPHYSX_API int32_t ovphysx_load_tensor_plugins(void);

#ifdef __cplusplus
}
#endif

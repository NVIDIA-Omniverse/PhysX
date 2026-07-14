// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary

// Force-included (MSVC /FI) before every nopy-compatible USD plugin translation unit.
//
// Saturates the pxr.h include-guard first, then un-defines PXR_PYTHON_SUPPORT_ENABLED.
// Any subsequent #include <pxr/pxr.h> in a source file hits the guard and skips the
// body, leaving PXR_PYTHON_SUPPORT_ENABLED undefined for the rest of the translation unit.
//
// Effect: the DLL has no pxr_boost::python imports and is binary-compatible with both
// usd.py312 and usd.nopy builds of ov_25.11usd_ms.dll.
// Python extension modules (_physxSchema, _physicsSchemaTools, …) are compiled separately
// and are unaffected — they do not use this force-include.

#include <pxr/pxr.h>
#ifdef PXR_PYTHON_SUPPORT_ENABLED
#undef PXR_PYTHON_SUPPORT_ENABLED
#endif

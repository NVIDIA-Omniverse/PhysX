// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


// Force pvdruntime symbols to be regular (non-exported) when compiled into pvddom.
// Include this AFTER any pvdruntime headers to undo the dllexport decoration.
#pragma once

#ifdef OMNI_PVD_EXPORT
#undef OMNI_PVD_EXPORT
#endif
#define OMNI_PVD_EXPORT

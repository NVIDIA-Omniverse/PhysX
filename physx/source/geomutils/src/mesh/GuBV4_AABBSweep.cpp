// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuBV4.h"
using namespace physx;
using namespace Gu;

#define SWEEP_AABB_IMPL
#include "foundation/PxVecMath.h"
using namespace aos;
#include "GuBV4_BoxSweep_Internal.h"

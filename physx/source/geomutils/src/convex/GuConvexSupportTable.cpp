// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuVecBox.h"

namespace physx
{
	const aos::BoolV boxVertexTable[8] = {
										aos::BFFFF(),//---
										aos::BTFFF(),//+--
										aos::BFTFF(),//-+-
										aos::BTTFF(),//++-
										aos::BFFTF(),//--+
										aos::BTFTF(),//+-+
										aos::BFTTF(),//-++
										aos::BTTTF(),//+++
									}; 
}

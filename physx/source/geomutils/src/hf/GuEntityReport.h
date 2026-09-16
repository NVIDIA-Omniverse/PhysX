// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_ENTITY_REPORT_H
#define GU_ENTITY_REPORT_H

#include "PxQueryReport.h"

namespace physx
{
namespace Gu
{
	class EntityReport
	{
		public:

		virtual			~EntityReport()	{}

		virtual	bool	onEvent(PxU32 nbEntities, const PxU32* entities) = 0;
	};

	class OverlapReport
	{
		public:

		virtual			~OverlapReport()	{}

		virtual	bool	reportTouchedTris(PxU32 nbEntities, const PxU32* entities) = 0;
	};

}  // namespace Gu

}

#endif

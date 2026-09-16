// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_REPX_UPGRADER_H
#define SN_REPX_UPGRADER_H

#include "foundation/PxSimpleTypes.h"

namespace physx { namespace Sn {
	class RepXCollection;

	class RepXUpgrader
	{
	public:
		//If a new collection is created, the source collection is destroyed.
		//Thus you only need to release the new collection.
		//This holds for all of the upgrade functions.
		//So be aware, that the argument to these functions may not be valid
		//after they are called, but the return value always will be valid.
		static RepXCollection& upgradeCollection( RepXCollection& src );
		static RepXCollection& upgrade10CollectionTo3_1Collection( RepXCollection& src );
		static RepXCollection& upgrade3_1CollectionTo3_2Collection( RepXCollection& src );
		static RepXCollection& upgrade3_2CollectionTo3_3Collection( RepXCollection& src );
		static RepXCollection& upgrade3_3CollectionTo3_4Collection( RepXCollection& src );
		static RepXCollection& upgrade3_4CollectionTo4_0Collection( RepXCollection& src );
	};
} }

#endif

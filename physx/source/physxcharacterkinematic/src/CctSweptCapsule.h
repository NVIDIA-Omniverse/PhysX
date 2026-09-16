// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CCT_SWEPT_CAPSULE
#define CCT_SWEPT_CAPSULE

#include "CctSweptVolume.h"

namespace physx
{
namespace Cct
{
	class SweptCapsule : public SweptVolume
	{
	public:
							SweptCapsule();
		virtual				~SweptCapsule();

		virtual		void	computeTemporalBox(const SweepTest&, PxExtendedBounds3& box, const PxExtendedVec3& center, const PxVec3& direction)	const	PX_OVERRIDE	PX_FINAL;

					PxF32	mRadius;
					PxF32	mHeight;
	};

} // namespace Cct

}

#endif

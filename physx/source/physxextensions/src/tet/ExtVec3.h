// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_VEC3_H
#define EXT_VEC3_H

#include "foundation/PxMath.h"
#include "foundation/PxVec3.h"

namespace physx
{
	namespace Ext
	{
		// ---------------------------------------------------------------------------------
		struct Bounds3 {
			Bounds3() {}
			Bounds3(const PxVec3d &min, const PxVec3d &max) : minimum(min), maximum(max) {}

			void setEmpty() {
				minimum = PxVec3d(PX_MAX_F64, PX_MAX_F64, PX_MAX_F64);
				maximum = PxVec3d(-PX_MAX_F64, -PX_MAX_F64, -PX_MAX_F64);
			}

			PxVec3d getDimensions() const {
				return maximum - minimum;
			}

			void include(const PxVec3d &p) {
				minimum = minimum.minimum(p);
				maximum = maximum.maximum(p);
			}

			void include(const Bounds3 &b) {
				minimum = minimum.minimum(b.minimum);
				maximum = maximum.maximum(b.maximum);
			}
			void expand(double d) {
				minimum -= PxVec3d(d, d, d);
				maximum += PxVec3d(d, d, d);
			}
			PxVec3d minimum, maximum;
		};
	}
}

#endif


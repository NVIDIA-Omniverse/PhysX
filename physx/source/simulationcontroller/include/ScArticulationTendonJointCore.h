// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef SC_TENDON_JOINT_CORE_H
#define SC_TENDON_JOINT_CORE_H

#include "foundation/PxVec3.h"
#include "solver/PxSolverDefs.h"

namespace physx
{
	namespace Sc
	{
		class ArticulationFixedTendonSim;

		class ArticulationTendonJointCore
		{
		public:

			// PX_SERIALIZATION
			ArticulationTendonJointCore(const PxEMPTY) : mTendonSim(NULL) {}
			void preExportDataReset() { }
			//~PX_SERIALIZATION

			ArticulationTendonJointCore()
			{
				coefficient = PX_MAX_F32;
				recipCoefficient = PX_MAX_F32;
			}

			PX_FORCE_INLINE PxArticulationAxis::Enum getAxis()
			{
				return axis;
			}

			PX_FORCE_INLINE void getCoefficient(PxArticulationAxis::Enum& axis_, PxReal& coefficient_, PxReal& recipCoefficient_) const
			{
				axis_ = axis;
				coefficient_ = coefficient;
				recipCoefficient_ = recipCoefficient;
			}

			void setCoefficient(PxArticulationAxis::Enum axis_, const PxReal coefficient_, const PxReal recipCoefficient_);


			PxArticulationAxis::Enum			axis;
			PxReal								coefficient;
			PxReal								recipCoefficient;
			PxU32								mLLLinkIndex;
			ArticulationTendonJointCore*		mParent;
			PxU32								mLLTendonJointIndex;
			Sc::ArticulationFixedTendonSim*		mTendonSim;
		};
	}//namespace Sc
}//namespace physx

#endif

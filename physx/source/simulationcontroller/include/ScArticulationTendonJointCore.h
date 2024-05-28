// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


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
			static void getBinaryMetaData(PxOutputStream& stream);
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

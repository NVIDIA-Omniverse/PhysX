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


#ifndef PX_PHYSICS_SCP_ARTICULATION_TENDON_CORE
#define PX_PHYSICS_SCP_ARTICULATION_TENDON_CORE

#include "DyArticulationTendon.h"

namespace physx
{
namespace Sc
{
	class ArticulationSpatialTendonSim;
	class ArticulationFixedTendonSim;

	class ArticulationTendonCore
	{
	public:

// PX_SERIALIZATION
															ArticulationTendonCore(const PxEMPTY) {}
						void								preExportDataReset() { }
		static			void								getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

		ArticulationTendonCore() : mStiffness(0.f), mDamping(0.f), mOffset(0.f), mLimitStiffness(0.f)
		{

		}
		PxReal								mStiffness;
		PxReal								mDamping;
		PxReal								mOffset;
		PxReal								mLimitStiffness;
	};

	class ArticulationSpatialTendonCore : public ArticulationTendonCore
	{
	public:

// PX_SERIALIZATION
															ArticulationSpatialTendonCore(const PxEMPTY) : ArticulationTendonCore(PxEmpty), mSim(NULL) {}
						void								preExportDataReset() { }
		static			void								getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

		ArticulationSpatialTendonCore() : ArticulationTendonCore() { mSim = NULL; }
		~ArticulationSpatialTendonCore() {}

		void							setStiffness(const PxReal stiffness);
		PxReal							getStiffness() const;

		void							setDamping(const PxReal damping);
		PxReal							getDamping() const;

		void							setLimitStiffness(const PxReal stiffness);
		PxReal							getLimitStiffness() const;

		void							setOffset(const PxReal offset);
		PxReal							getOffset() const;


		PX_FORCE_INLINE	void							setSim(ArticulationSpatialTendonSim* sim)
		{
			PX_ASSERT((sim == 0) ^ (mSim == 0));
			mSim = sim;
		}

		PX_FORCE_INLINE	ArticulationSpatialTendonSim*			getSim()			const { return mSim; }

	
		ArticulationSpatialTendonSim*			mSim;
	};

	class ArticulationFixedTendonCore : public ArticulationTendonCore
	{
	public:
		// PX_SERIALIZATION
		ArticulationFixedTendonCore(const PxEMPTY) : ArticulationTendonCore(PxEmpty), mSim(NULL) {}
		void preExportDataReset() {}
		static void getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION

		ArticulationFixedTendonCore() : ArticulationTendonCore(), mLowLimit(PX_MAX_F32), mHighLimit(-PX_MAX_F32), mRestLength(0.f)
		{ mSim = NULL; }
		~ArticulationFixedTendonCore() {}

		void							setStiffness(const PxReal stiffness);
		PxReal							getStiffness() const;

		void							setDamping(const PxReal damping);
		PxReal							getDamping() const;

		void							setLimitStiffness(const PxReal stiffness);
		PxReal							getLimitStiffness() const;


		void							setOffset(const PxReal offset);
		PxReal							getOffset() const;

		void							setSpringRestLength(const PxReal restLength);
		PxReal							getSpringRestLength() const;

		void							setLimitRange(const PxReal lowLimit, const PxReal highLimit);
		void							getLimitRange(PxReal& lowLimit, PxReal& highLimit) const;

		PX_FORCE_INLINE	void							setSim(ArticulationFixedTendonSim* sim)
		{
			PX_ASSERT((sim == 0) ^ (mSim == 0));
			mSim = sim;
		}
		PX_FORCE_INLINE	ArticulationFixedTendonSim*			getSim()			const { return mSim; }

		PxReal								mLowLimit;
		PxReal								mHighLimit;
		PxReal								mRestLength;

		ArticulationFixedTendonSim*			mSim;
	};


}//namespace Sc
} //namespace physx

#endif

// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


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

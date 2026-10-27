// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_ARTICULATION_TENDON_SIM_H
#define SC_ARTICULATION_TENDON_SIM_H

#include "foundation/PxUserAllocated.h"
#include "DyArticulationTendon.h"

namespace physx
{
namespace Sc
{

	class ArticulationFixedTendonCore;
	class ArticulationTendonJointCore;
	class ArticulationSpatialTendonCore;
	class ArticulationAttachmentCore;
	class Scene;
	class ArticulationJointCore;
	class ArticulationSim;

	class ArticulationSpatialTendonSim : public PxUserAllocated
	{
		PX_NOCOPY(ArticulationSpatialTendonSim)
	public:
		ArticulationSpatialTendonSim(ArticulationSpatialTendonCore& tendon, Scene& scene);

		virtual ~ArticulationSpatialTendonSim();

		void							setStiffness(const PxReal stiffness);
		PxReal							getStiffness() const;

		void							setDamping(const PxReal damping);
		PxReal							getDamping() const;

		void							setLimitStiffness(const PxReal stiffness);
		PxReal							getLimitStiffness() const;

		void							setOffset(const PxReal offset);
		PxReal							getOffset() const;


		void							setAttachmentCoefficient(ArticulationAttachmentCore& core, const PxReal coefficient);
		void							setAttachmentRelativeOffset(ArticulationAttachmentCore& core, const PxVec3& offset);
		void							setAttachmentLimits(ArticulationAttachmentCore& core, const PxReal lowLimit, const PxReal highLimit);
		void							setAttachmentRestLength(ArticulationAttachmentCore& core, const PxReal restLength);
		void addAttachment(ArticulationAttachmentCore& core);
		void removeAttachment(ArticulationAttachmentCore& core);

		Dy::ArticulationSpatialTendon						mLLTendon;
		ArticulationSpatialTendonCore&						mTendonCore;

		ArticulationSim*									mArtiSim;
		Scene& mScene;

	};

	class ArticulationFixedTendonSim : public PxUserAllocated
	{
		PX_NOCOPY(ArticulationFixedTendonSim)
	public:
		ArticulationFixedTendonSim(ArticulationFixedTendonCore& tendon, Scene& scene);

		virtual ~ArticulationFixedTendonSim();

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

		void addTendonJoint(ArticulationTendonJointCore& tendonJointCore);
		void removeTendonJoint(ArticulationTendonJointCore& core);

		void							setTendonJointCoefficient(ArticulationTendonJointCore& core, const PxArticulationAxis::Enum axis, const float coefficient, const float recipCoefficient);

		Dy::ArticulationFixedTendon						mLLTendon; 
		ArticulationFixedTendonCore&					mTendonCore;

		ArticulationSim*								mArtiSim;
		Scene& mScene;
	};
}//namespace Sc
}//namespace physx
#endif

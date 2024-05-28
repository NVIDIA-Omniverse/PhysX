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

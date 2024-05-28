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

#ifndef PXS_PARTICLE_BUFFER_H
#define PXS_PARTICLE_BUFFER_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxUserAllocated.h"
#include "PxParticleSystemFlag.h"

namespace physx
{
	class PxCudaContextManager;
	struct PxParticleVolume;
	struct PxParticleRigidFilterPair;
	struct PxParticleRigidAttachment;

	class PxsParticleBuffer
	{
	public:
		virtual void release() = 0;

		virtual PxVec4* getPositionInvMassesD() const = 0;
		virtual PxVec4* getVelocitiesD() const = 0;
		virtual PxU32* getPhasesD() const = 0;
		virtual PxParticleVolume* getParticleVolumesD() const = 0;

		virtual PxVec4* getPositionInvMassesH() const = 0;
		virtual PxVec4* getVelocitiesH() const = 0;
		virtual PxU32* getPhasesH() const = 0;
		virtual PxParticleVolume* getParticleVolumesH() const = 0;

		virtual void setNbActiveParticles(PxU32 nbActiveParticles) = 0;
		virtual PxU32 getNbActiveParticles() const = 0;
		virtual PxU32 getMaxParticles() const = 0;
		virtual PxU32 getNbParticleVolumes() const = 0;
		virtual void setNbParticleVolumes(PxU32 nbParticleVolumes) = 0;
		virtual PxU32 getMaxParticleVolumes() const = 0;
		virtual void setRigidFilters(PxParticleRigidFilterPair* filters, PxU32 nbFilters) = 0;
		virtual void setRigidAttachments(PxParticleRigidAttachment* attachments, PxU32 nbAttachments) = 0;
		virtual PxU32 getFlatListStartIndex() const = 0;
		virtual void raiseFlags(PxParticleBufferFlag::Enum flags) = 0;
		virtual PxU32 getUniqueId() const = 0;
		virtual void allocHostBuffers() = 0;

	protected:
		virtual ~PxsParticleBuffer() {}
	};

	class PxsParticleAndDiffuseBuffer : public PxsParticleBuffer
	{
	public:
		virtual PxVec4* getDiffusePositionLifeTimeD() const = 0;
		virtual PxVec4* getDiffuseVelocitiesD() const = 0;
		virtual PxU32 getNbActiveDiffuseParticles() const = 0;
		virtual void setMaxActiveDiffuseParticles(PxU32 maxActiveDiffuseParticles) = 0;
		virtual PxU32 getMaxDiffuseParticles() const = 0;
		virtual void setDiffuseParticleParams(const PxDiffuseParticleParams& params) = 0;
		virtual const PxDiffuseParticleParams& getDiffuseParticleParams() const = 0;

	protected:
		virtual ~PxsParticleAndDiffuseBuffer() {}
	};

	class PxsParticleClothBuffer : public PxsParticleBuffer
	{
	public:
		virtual PxVec4* getRestPositionsD() = 0;
		virtual PxU32* getTrianglesD() const = 0;
		virtual void setNbTriangles(PxU32 nbTriangles) = 0;
		virtual PxU32 getNbTriangles() const = 0;
		virtual PxU32 getNbSprings() const = 0;
		virtual PxParticleSpring* getSpringsD() = 0;
		virtual void setCloths(PxPartitionedParticleCloth& cloths) = 0;

	protected:
		virtual ~PxsParticleClothBuffer() {}
	};

	class PxsParticleRigidBuffer : public PxsParticleBuffer
	{
	public:
		virtual PxU32* getRigidOffsetsD() const = 0;
		virtual PxReal* getRigidCoefficientsD() const = 0;
		virtual PxVec4* getRigidLocalPositionsD() const = 0;
		virtual PxVec4* getRigidTranslationsD() const = 0;
		virtual PxVec4* getRigidRotationsD() const = 0;
		virtual PxVec4* getRigidLocalNormalsD() const = 0;
		virtual void setNbRigids(PxU32 nbRigids) = 0;
		virtual PxU32 getNbRigids() const = 0;

	protected:
		virtual ~PxsParticleRigidBuffer() {}
	};

}
#endif
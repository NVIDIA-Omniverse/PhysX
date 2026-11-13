// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_PARTICLE_BUFFER_H
#define PXS_PARTICLE_BUFFER_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxUserAllocated.h"
#include "PxParticleSystemFlag.h"

namespace physx
{
	class PxCudaContextManager;
	class PxsParticleBuffer
	{
	public:
		virtual void release() = 0;

		virtual PxVec4* getPositionInvMassesD() const = 0;
		virtual PxVec4* getVelocitiesD() const = 0;
		virtual PxU32* getPhasesD() const = 0;

		virtual PxVec4* getPositionInvMassesH() const = 0;
		virtual PxVec4* getVelocitiesH() const = 0;
		virtual PxU32* getPhasesH() const = 0;

		virtual void setNbActiveParticles(PxU32 nbActiveParticles) = 0;
		virtual PxU32 getNbActiveParticles() const = 0;
		virtual PxU32 getMaxParticles() const = 0;
		virtual PxU32 getFlatListStartIndex() const = 0;
		virtual void raiseFlags(PxParticleBufferFlag::Enum flags) = 0;
		virtual PxU32 getUniqueId() const = 0;

#if PX_SUPPORT_OMNI_PVD
		enum OmniPvdReadFlag { ePOSITION = 1, eVELOCITY = 2, ePHASE = 4, eDIFFUSE_POSITION = 8, eDIFFUSE_VELOCITY = 16 };
		virtual bool readOmniPvdData(PxU32 flags, PxU64& transferredBytes) = 0;
		virtual void trimOmniPvdData(PxU32 flags) = 0;
		virtual bool getOmniPvdPhasesChanged() const = 0;
		virtual void setOmniPvdPhasesChanged(bool changed) = 0;
#endif

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

#if PX_SUPPORT_OMNI_PVD
		virtual PxVec4* getDiffusePositionLifeTimeH() const = 0;
		virtual PxVec4* getDiffuseVelocitiesH() const = 0;
		// Entries copied by the last successful readOmniPvdData().
		virtual PxU32 getOmniPvdDiffuseCount() const = 0;
#endif

	protected:
		virtual ~PxsParticleAndDiffuseBuffer() {}
	};

}
#endif

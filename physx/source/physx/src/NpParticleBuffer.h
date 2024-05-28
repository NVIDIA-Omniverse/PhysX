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

#ifndef NP_PARTICLE_BUFFER_H
#define NP_PARTICLE_BUFFER_H

#include "PxParticleBuffer.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxArray.h"
#include "NpBase.h"
#include "NpFactory.h"
#include "PxsParticleBuffer.h"
#include "omnipvd/OmniPvdPxSampler.h"

namespace physx
{
	class PxCudaContextManager;

	class NpParticleClothPreProcessor : public PxParticleClothPreProcessor, public PxUserAllocated
	{
	public:
		NpParticleClothPreProcessor(PxCudaContextManager* cudaContextManager) : mCudaContextManager(cudaContextManager), mNbPartitions(0){}
		virtual ~NpParticleClothPreProcessor() {}

		virtual	void release();

		virtual void partitionSprings(const PxParticleClothDesc& clothDesc, PxPartitionedParticleCloth& output) PX_OVERRIDE;

	private:
		PxU32 computeSpringPartition(const PxParticleSpring& springs, const PxU32 partitionStartIndex, PxU32* partitionProgresses);

		PxU32* partitions(const PxParticleSpring* springs, PxU32* orderedSpringIndices);

		PxU32 combinePartitions(const PxParticleSpring* springs, const PxU32* orderedSpringIndices, const PxU32* accumulatedSpringsPerPartition,
			PxU32* accumulatedSpringsPerCombinedPartitions, PxParticleSpring* orderedSprings, PxU32* accumulatedCopiesPerParticles, PxU32* remapOutput);

		void classifySprings(const PxParticleSpring* springs, PxU32* partitionProgresses, PxU32* tempSprings, physx::PxArray<PxU32>& springsPerPartition);

		void writeSprings(const PxParticleSpring* springs, PxU32* partitionProgresses, PxU32* tempSprings, PxU32* orderedSprings,
			PxU32* accumulatedSpringsPerPartition);


		PxCudaContextManager* mCudaContextManager;

		PxU32 mNumSprings;
		PxU32 mNbPartitions;
		PxU32 mNumParticles;
		PxU32 mMaxSpringsPerPartition;
	};

	template<class APIClass>
	class NpParticleBufferBase : public APIClass, public NpBase
	{
	public:
		NpParticleBufferBase(PxType type)
		: APIClass(type)
		, NpBase(NpType::eUNDEFINED)
		, mGpuBuffer(NULL)
		, mParticleSystem(NULL) 
		, mBufferIndex(0xffffffff)
		{}

		virtual ~NpParticleBufferBase() PX_OVERRIDE { NpFactory::getInstance().onParticleBufferRelease(this); }

		virtual PxVec4* getPositionInvMasses() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getPositionInvMassesD();
		}

		virtual PxVec4* getVelocities() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getVelocitiesD();
		}

		virtual PxU32* getPhases() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getPhasesD();
		}

		virtual	PxParticleVolume* getParticleVolumes() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getParticleVolumesD();
		}

		virtual void setNbActiveParticles(PxU32 numActiveParticles) PX_OVERRIDE PX_FINAL
		{
			mGpuBuffer->setNbActiveParticles(numActiveParticles);
		}

		virtual PxU32 getNbActiveParticles() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getNbActiveParticles();
		}

		virtual PxU32 getMaxParticles() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getMaxParticles();
		}

		virtual PxU32 getNbParticleVolumes() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getNbParticleVolumes();
		}

		virtual PxU32 getMaxParticleVolumes() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getMaxParticleVolumes();
		}

		virtual void setNbParticleVolumes(PxU32 numVolumes) PX_OVERRIDE PX_FINAL
		{
			mGpuBuffer->setNbParticleVolumes(numVolumes);
		}

		virtual PxU32 getFlatListStartIndex() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getFlatListStartIndex();
		}

		virtual void raiseFlags(PxParticleBufferFlag::Enum flags) PX_OVERRIDE PX_FINAL
		{
			mGpuBuffer->raiseFlags(flags);
		}

		virtual void	setRigidFilters(PxParticleRigidFilterPair* filters, PxU32 numFilters) PX_OVERRIDE PX_FINAL
		{
			mGpuBuffer->setRigidFilters(filters, numFilters);
		}

		virtual void setRigidAttachments(PxParticleRigidAttachment* attachments, PxU32 numAttachments) PX_OVERRIDE PX_FINAL
		{
			mGpuBuffer->setRigidAttachments(attachments, numAttachments);
		}

		virtual PxU32	getUniqueId() const PX_OVERRIDE PX_FINAL
		{
			return mGpuBuffer->getUniqueId();
		}

		void setParticleSystem(NpPBDParticleSystem* particleSystem)
		{
			mParticleSystem = particleSystem;
		}

		void allocHostBuffers()
		{
			mGpuBuffer->allocHostBuffers();
		}

		PxsParticleBuffer*		mGpuBuffer;
		NpPBDParticleSystem*	mParticleSystem;
		PxU32					mBufferIndex;
	};

	class NpParticleBuffer : public NpParticleBufferBase<PxParticleBuffer>
	{
	public:
		NpParticleBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxCudaContextManager& cudaContextManager);
		virtual void release() PX_OVERRIDE PX_FINAL;

		virtual	bool isKindOf(const char* name) const PX_OVERRIDE PX_FINAL
		{
			PX_IS_KIND_OF(name, "PxParticleBuffer", PxBase);
		}

		virtual	const char* getConcreteTypeName() const PX_OVERRIDE PX_FINAL
		{
			return "PxParticleBuffer";
		}
	};

	class NpParticleAndDiffuseBuffer : public NpParticleBufferBase<PxParticleAndDiffuseBuffer>
	{
	public:
		NpParticleAndDiffuseBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumDiffuseParticles, PxCudaContextManager& cudaContextManager);
		virtual void release() PX_OVERRIDE PX_FINAL;

		virtual PxVec4* getDiffusePositionLifeTime() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleAndDiffuseBuffer*>(mGpuBuffer)->getDiffusePositionLifeTimeD();
		}

		virtual PxVec4* getDiffuseVelocities() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleAndDiffuseBuffer*>(mGpuBuffer)->getDiffuseVelocitiesD();
		}

		virtual PxU32 getNbActiveDiffuseParticles() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleAndDiffuseBuffer*>(mGpuBuffer)->getNbActiveDiffuseParticles();
		}

		virtual void setMaxActiveDiffuseParticles(PxU32 maxActiveDiffuseParticles) PX_OVERRIDE PX_FINAL
		{
			static_cast<PxsParticleAndDiffuseBuffer*>(mGpuBuffer)->setMaxActiveDiffuseParticles(maxActiveDiffuseParticles);
		}

		virtual PxU32 getMaxDiffuseParticles() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleAndDiffuseBuffer*>(mGpuBuffer)->getMaxDiffuseParticles();
		}

		virtual void setDiffuseParticleParams(const PxDiffuseParticleParams& params) PX_OVERRIDE PX_FINAL
		{
			static_cast<PxsParticleAndDiffuseBuffer*>(mGpuBuffer)->setDiffuseParticleParams(params);
#if PX_SUPPORT_OMNI_PVD
			streamDiffuseParticleParamsAttributes(getDiffuseParticleParamsRef());
#endif
		}

		virtual PxDiffuseParticleParams getDiffuseParticleParams() const PX_OVERRIDE PX_FINAL
		{
			return getDiffuseParticleParamsRef();
		}

		virtual	bool isKindOf(const char* name) const PX_OVERRIDE PX_FINAL
		{
			PX_IS_KIND_OF(name, "PxParticleAndDiffuseBuffer", PxParticleBuffer);
		}

		virtual	const char* getConcreteTypeName() const PX_OVERRIDE PX_FINAL
		{
			return "PxParticleAndDiffuseBuffer";
		}

		const PxDiffuseParticleParams& getDiffuseParticleParamsRef() const
		{
			return static_cast<PxsParticleAndDiffuseBuffer*>(mGpuBuffer)->getDiffuseParticleParams();
		}
	};

	class NpParticleClothBuffer : public NpParticleBufferBase<PxParticleClothBuffer>
	{
	public:
		NpParticleClothBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, 
			PxU32 maxNumSprings, PxCudaContextManager& cudaContextManager);
		virtual void release() PX_OVERRIDE PX_FINAL;

		virtual PxVec4* getRestPositions() PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleClothBuffer*>(mGpuBuffer)->getRestPositionsD();
		}

		virtual PxU32* getTriangles() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleClothBuffer*>(mGpuBuffer)->getTrianglesD();
		}

		virtual void setNbTriangles(PxU32 nbTriangles) PX_OVERRIDE PX_FINAL
		{
			static_cast<PxsParticleClothBuffer*>(mGpuBuffer)->setNbTriangles(nbTriangles);
		}

		virtual PxU32 getNbTriangles() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleClothBuffer*>(mGpuBuffer)->getNbTriangles();
		}

		virtual PxU32 getNbSprings() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleClothBuffer*>(mGpuBuffer)->getNbSprings();
		}

		virtual PxParticleSpring* getSprings() PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleClothBuffer*>(mGpuBuffer)->getSpringsD();
		}

		virtual void setCloths(PxPartitionedParticleCloth& cloths) PX_OVERRIDE PX_FINAL
		{
			static_cast<PxsParticleClothBuffer*>(mGpuBuffer)->setCloths(cloths);
		}

		virtual	bool isKindOf(const char* name) const PX_OVERRIDE PX_FINAL
		{
			PX_IS_KIND_OF(name, "PxParticleClothBuffer", PxParticleBuffer);
		}

		virtual	const char* getConcreteTypeName() const PX_OVERRIDE PX_FINAL
		{
			return "PxParticleClothBuffer";
		}
	};

	class NpParticleRigidBuffer : public NpParticleBufferBase<PxParticleRigidBuffer>
	{
	public:
		NpParticleRigidBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumRigids, PxCudaContextManager& cudaContextManager);
		virtual void release() PX_OVERRIDE PX_FINAL;

		virtual PxU32* getRigidOffsets() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->getRigidOffsetsD();
		}

		virtual PxReal* getRigidCoefficients() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->getRigidCoefficientsD();
		}

		virtual PxVec4* getRigidLocalPositions() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->getRigidLocalPositionsD();
		}

		virtual PxVec4* getRigidTranslations() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->getRigidTranslationsD();
		}

		virtual PxVec4* getRigidRotations() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->getRigidRotationsD();
		}

		virtual PxVec4* getRigidLocalNormals() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->getRigidLocalNormalsD();
		}

		virtual void setNbRigids(PxU32 nbRigids) PX_OVERRIDE PX_FINAL
		{
			static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->setNbRigids(nbRigids);
		}

		virtual PxU32 getNbRigids() const PX_OVERRIDE PX_FINAL
		{
			return static_cast<PxsParticleRigidBuffer*>(mGpuBuffer)->getNbRigids();
		}

		virtual	bool isKindOf(const char* name) const PX_OVERRIDE PX_FINAL
		{
			PX_IS_KIND_OF(name, "PxParticleRigidBuffer", PxParticleBuffer);
		}

		virtual	const char* getConcreteTypeName() const PX_OVERRIDE PX_FINAL
		{
			return "PxParticleRigidBuffer";
		}
	};

}
#endif

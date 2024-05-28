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

#ifndef NP_HAIR_SYSTEM_H
#define NP_HAIR_SYSTEM_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxHairSystem.h"
#include "ScHairSystemCore.h"
#include "NpActorTemplate.h"

namespace physx
{
	class NpScene;
	class NpShape;

	class NpHairSystem : public NpActorTemplate<PxHairSystem>
	{
	public:
		template <class T>
		class MemoryWithAlloc
		{
			PX_NOCOPY(MemoryWithAlloc)
		public:
			MemoryWithAlloc() : mData(NULL), mSize(0), mAlloc(NULL) {}

			~MemoryWithAlloc()
			{
				deallocate();
			}

			void allocate(uint32_t size)
			{
				if (size == mSize)
					return;

				deallocate();
				if (size > 0)
				{
					mData = reinterpret_cast<T*>(mAlloc->allocate(sizeof(T) * size, 0, PX_FL));
					if (mData != NULL)
						mSize = size;
				}
				else
				{
					mData = NULL;
					mSize = 0;
				}
			}

			void setAllocatorCallback(physx::PxVirtualAllocatorCallback* cb)
			{
				if (mSize > 0)
				{
					deallocate();
				}
				mAlloc = cb;
			}

			void reset()
			{
				if (mSize > 0)
				{
					deallocate();
				}
			}

			void swap(MemoryWithAlloc<T>& other)
			{
				physx::PxSwap(mData, other.mData);
				physx::PxSwap(mSize, other.mSize);
				physx::PxSwap(mAlloc, other.mAlloc);
			}

			uint32_t size() const
			{
				return mSize;
			}

			const T& operator[](uint32_t i) const
			{
				PX_ASSERT(i < mSize);
				return mData[i];
			}

			T& operator[](uint32_t i)
			{
				PX_ASSERT(i < mSize);
				return mData[i];
			}

			const T* begin() const
			{
				return mData;
			}

			T* begin()
			{
				return mData;
			}

		private:
			void deallocate()
			{
				if (mSize > 0)
				{
					mAlloc->deallocate(mData);
					mSize = 0;
				}
			}

		private:
			T* mData;
			uint32_t mSize;
			physx::PxVirtualAllocatorCallback* mAlloc;
		};

	public:
		NpHairSystem(PxCudaContextManager& cudaContextManager);
		NpHairSystem(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager);

		virtual ~NpHairSystem() PX_OVERRIDE;

		// external API
		virtual void				release() PX_OVERRIDE;

		virtual PxActorType::Enum	getType() const PX_OVERRIDE { return PxActorType::eHAIRSYSTEM; }

		virtual PxBounds3			getWorldBounds(float inflation = 1.01f) const PX_OVERRIDE;

		virtual void				setHairSystemFlag(PxHairSystemFlag::Enum flag, bool val) PX_OVERRIDE;
		virtual void				setHairSystemFlags(PxHairSystemFlags flags) PX_OVERRIDE { mCore.setFlags(flags); }
		virtual PxHairSystemFlags	getHairSystemFlags() const PX_OVERRIDE { return mCore.getFlags(); }

		virtual void				setReadRequestFlag(PxHairSystemData::Enum flag, bool val) PX_OVERRIDE;
		virtual void				setReadRequestFlags(PxHairSystemDataFlags flags) PX_OVERRIDE;
		virtual PxHairSystemDataFlags getReadRequestFlags() const PX_OVERRIDE;

		virtual void				setBendingRestAngles(const PxReal* bendingRestAngles, PxReal bendingCompliance) PX_OVERRIDE;

		virtual void				setTwistingCompliance(PxReal twistingCompliance) PX_OVERRIDE;
		virtual void				getTwistingRestPositions(PxReal* buffer) PX_OVERRIDE;

		virtual PxCudaContextManager*	getCudaContextManager() const PX_OVERRIDE { return mCudaContextManager; }

		virtual void				setWakeCounter(PxReal wakeCounterValue) PX_OVERRIDE;
		virtual PxReal				getWakeCounter() const PX_OVERRIDE;
		virtual bool				isSleeping() const PX_OVERRIDE;
		
		virtual void				setRestPositions(PxVec4* restPos, bool isGpuPtr) PX_OVERRIDE;
		virtual PxVec4*				getRestPositionsGpu() PX_OVERRIDE;

		virtual void				addRigidAttachment(const PxRigidBody& rigidBody) PX_OVERRIDE;
		virtual void				removeRigidAttachment(const PxRigidBody& rigidBody) PX_OVERRIDE;
		virtual void				setRigidAttachments(PxParticleRigidAttachment* attachments, PxU32 numAttachments, bool isGpuPtr) PX_OVERRIDE;
		virtual PxParticleRigidAttachment* getRigidAttachmentsGpu(PxU32* numAttachments = NULL) PX_OVERRIDE;

		virtual PxU32				addSoftbodyAttachment(const PxSoftBody& softbody, const PxU32* tetIds, const PxVec4* tetmeshBarycentrics,
			const PxU32* hairVertices, PxConeLimitedConstraint* constraints, PxReal* constraintOffsets, PxU32 numAttachments) PX_OVERRIDE;
		virtual void				removeSoftbodyAttachment(const PxSoftBody& softbody, PxU32 handle) PX_OVERRIDE;

		virtual void				initFromDesc(const PxHairSystemDesc& desc) PX_OVERRIDE;

		virtual void				setTopology(PxVec4* vertexPositionsInvMass, PxVec4* vertexVelocities,
			const PxU32* strandPastEndIndices, PxReal segmentLength, PxReal segmentRadius, PxU32 numVertices,
			PxU32 numStrands, const PxBounds3& bounds) PX_OVERRIDE;

		virtual void				setLevelOfDetailGradations(const PxReal* proportionOfStrands, const PxReal* proportionOfVertices, PxU32 numLevels) PX_OVERRIDE;
		virtual void				setLevelOfDetail(PxU32 level) PX_OVERRIDE;
		virtual PxU32				getLevelOfDetail(PxU32* numLevels) const PX_OVERRIDE;

		virtual void				setPositionsInvMass(PxVec4* vertexPositionsInvMass, const PxBounds3& bounds) PX_OVERRIDE;
		virtual PxVec4*				getPositionInvMass() PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mPositionInvMass; }
		virtual const PxVec4*		getPositionInvMass() const PX_OVERRIDE{ return mCore.getShapeCore().getLLCore().mPositionInvMass; }
		virtual const PxVec4*		getPositionInvMassGpuSim() const PX_OVERRIDE{ return mCore.getShapeCore().getLLCore().mPositionInvMassGpuSim; }

		virtual void				setVelocities(PxVec4* vertexVelocities) PX_OVERRIDE;
		virtual PxVec4*				getVelocities() PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mVelocity; }
		virtual const PxVec4*		getVelocities() const PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mVelocity; }

		virtual PxU32				getNumVertices() const PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mNumVertices; }
		virtual PxU32				getNumStrands() const PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mNumStrands; }
		virtual const PxU32*		getStrandPastEndIndices() const PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mStrandPastEndIndices; }
		virtual const PxU32*		getStrandPastEndIndicesGpuSim() const PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mStrandPastEndIndicesGpuSim; }

		virtual void				setSolverIterationCounts(PxU32 iters) PX_OVERRIDE;
		virtual PxU32				getSolverIterationCounts() const PX_OVERRIDE;
		virtual void				setWind(const PxVec3& wind) PX_OVERRIDE;
		virtual PxVec3				getWind() const PX_OVERRIDE;
		virtual void				setAerodynamicDrag(PxReal dragCoefficient) PX_OVERRIDE;
		virtual PxReal				getAerodynamicDrag() const PX_OVERRIDE;
		virtual void				setAerodynamicLift(PxReal liftCoefficient) PX_OVERRIDE;
		virtual PxReal				getAerodynamicLift() const PX_OVERRIDE;

		virtual void				setSegmentRadius(PxReal radius) PX_OVERRIDE;
		virtual void				getSegmentDimensions(PxReal& length, PxReal& radius) const PX_OVERRIDE;
		virtual void				setFrictionParameters(PxReal interHairVelDamping, PxReal frictionCoeff) PX_OVERRIDE;
		virtual void				getFrictionParameters(PxReal& interHairVelDamping, PxReal& frictionCoeff) const PX_OVERRIDE;
		virtual void				setMaxDepenetrationVelocity(PxReal maxDepenetrationVelocity) PX_OVERRIDE;
		virtual PxReal				getMaxDepenetrationVelocity() const PX_OVERRIDE;
		virtual void				setShapeCompliance(PxReal startCompliance, PxReal strandRatio) PX_OVERRIDE;
		virtual void				getShapeCompliance(PxReal& startCompliance, PxReal& strandRatio) const PX_OVERRIDE;
		virtual void				setInterHairRepulsion(PxReal repulsion) PX_OVERRIDE;
		virtual PxReal				getInterHairRepulsion() const PX_OVERRIDE;
		virtual void				setSelfCollisionRelaxation(PxReal relaxation) PX_OVERRIDE;
		virtual PxReal				getSelfCollisionRelaxation() const PX_OVERRIDE;
		virtual void				setStretchingRelaxation(PxReal relaxation) PX_OVERRIDE;
		virtual PxReal				getStretchingRelaxation() const PX_OVERRIDE;
		virtual void				setContactOffset(PxReal contactOffset) PX_OVERRIDE;
		virtual PxReal				getContactOffset() const PX_OVERRIDE;
		virtual void				setHairContactOffset(PxReal hairContactOffset) PX_OVERRIDE;
		virtual PxReal				getHairContactOffset() const PX_OVERRIDE;
		virtual void				setShapeMatchingParameters(PxReal compliance, PxReal linearStretching, PxU16 numVerticesPerGroup, PxU16 numVerticesOverlap) PX_OVERRIDE;
		virtual PxReal				getShapeMatchingCompliance() const PX_OVERRIDE;

#if PX_ENABLE_DEBUG_VISUALIZATION
		void visualize(PxRenderOutput& out, NpScene& npScene)	const;
#endif

		PX_FORCE_INLINE	const	Sc::HairSystemCore&		getCore() const	{ return mCore; }
		PX_FORCE_INLINE			Sc::HairSystemCore&		getCore()		{ return mCore; }
		static PX_FORCE_INLINE size_t					getCoreOffset()	{ return PX_OFFSET_OF_RT(NpHairSystem, mCore); }

	private:
		void init();
		void setLlGridSize(const PxBounds3& bounds);
		void createAllocator();
		void releaseAllocator();

	private:
		Sc::HairSystemCore			mCore;
		PxHairSystemGeometry		mGeometry;
		PxCudaContextManager*		mCudaContextManager;

		PxsMemoryManager*			mMemoryManager;
		PxVirtualAllocatorCallback*	mHostMemoryAllocator;
		PxVirtualAllocatorCallback*	mDeviceMemoryAllocator;

		// internal buffers populated when hair system initialized from desc instead of user-provided buffers
		MemoryWithAlloc<PxVec4>	mPosInvMassInternal;
		MemoryWithAlloc<PxVec4>	mVelInternal;
		MemoryWithAlloc<PxU32>	mStrandPastEndIndicesInternal;

		// internal buffers in case user gives us CPU-buffers
		MemoryWithAlloc<PxParticleRigidAttachment> mParticleRigidAttachmentsInternal;
		MemoryWithAlloc<PxVec4> mRestPositionsInternal;

		PxArray<PxReal> mLodProportionOfStrands;
		PxArray<PxReal> mLodProportionOfVertices;

		PxHashMap<PxU32, PxPair<PxU32, PxU32>> mSoftbodyAttachmentsOffsets; // map from handle to {offset, size}
		PxPinnedArray<Dy::SoftbodyHairAttachment> mSoftbodyAttachments;
	};
}

#endif
#endif
#endif

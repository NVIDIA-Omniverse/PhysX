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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef NP_PARTICLE_SYSTEM_H
#define NP_PARTICLE_SYSTEM_H

#include "foundation/PxBounds3.h"
#include "foundation/PxErrors.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxVec3.h"

#include "common/PxBase.h"
#include "common/PxRenderOutput.h"

#include "PxActor.h"
#include "PxFiltering.h"
#include "PxParticleBuffer.h"
//#include "PxParticlePhase.h"
#include "PxParticleSolverType.h"
#include "PxParticleSystem.h"
#include "PxPBDParticleSystem.h"
#include "PxPBDMaterial.h"
#include "PxSceneDesc.h"
#include "PxSparseGridParams.h"

#include "DyParticleSystem.h"

#include "NpActor.h"
#include "NpActorTemplate.h"
#include "NpBase.h"
#include "NpMaterialManager.h"
#include "NpPhysics.h"

#include "ScParticleSystemSim.h"

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxFLIPParticleSystem.h"
#include "PxFLIPMaterial.h"
#include "PxMPMParticleSystem.h"
#include "PxMPMMaterial.h"
#endif

namespace physx
{
	class NpScene;

	class PxCudaContextManager;
	class PxParticleMaterial;
	class PxRigidActor;
	class PxSerializationContext;

	namespace Sc
	{
		class ParticleSystemSim;
	}

	template<class APIClass>
	class NpParticleSystem : public NpActorTemplate<APIClass>
	{
	public:
		NpParticleSystem(PxCudaContextManager& contextManager, PxType concreteType, NpType::Enum npType, PxActorType::Enum actorType) :
			NpActorTemplate<APIClass>(concreteType, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, npType),
			mCore(actorType),
			mCudaContextManager(&contextManager),
			mNextPhaseGroupID(0)
		{

		}
									NpParticleSystem(PxBaseFlags baseFlags) : NpActorTemplate<APIClass>(baseFlags)
									{}

		virtual						~NpParticleSystem()
		{
			//TODO!!!!! Do this correctly
			//sParticleSystemIdPool.tryRemoveID(mID);
		}
		virtual void				exportData(PxSerializationContext& /*context*/) const {}
		//external API

		virtual	PxBounds3			getWorldBounds(float inflation = 1.01f) const
		{
			NP_READ_CHECK(NpBase::getNpScene());

			if (!NpBase::getNpScene())
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Querying bounds of a PxParticleSystem which is not part of a PxScene is not supported.");
				return PxBounds3::empty();
			}

			const Sc::ParticleSystemSim* sim = mCore.getSim();
			PX_ASSERT(sim);

			PX_SIMD_GUARD;

			PxBounds3 bounds = sim->getBounds();
			PX_ASSERT(bounds.isValid());

			// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
			const PxVec3 center = bounds.getCenter();
			const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
			return PxBounds3::centerExtents(center, inflatedExtents);
		}

		virtual	void setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters = 1)
		{
			NpScene* npScene = NpBase::getNpScene();
			NP_WRITE_CHECK(npScene);
			PX_CHECK_AND_RETURN(minPositionIters > 0, "NpParticleSystem::setSolverIterationCounts: positionIters must be more than zero!");
			PX_CHECK_AND_RETURN(minPositionIters <= 255, "NpParticleSystem::setSolverIterationCounts: positionIters must be no greater than 255!");
			PX_CHECK_AND_RETURN(minVelocityIters <= 255, "NpParticleSystem::setSolverIterationCounts: velocityIters must be no greater than 255!");

			PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxParticleSystem::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.")

			mCore.setSolverIterationCounts((minVelocityIters & 0xff) << 8 | (minPositionIters & 0xff));

			//UPDATE_PVD_PROPERTY
		}

		virtual	void getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const
		{
			NP_READ_CHECK(NpBase::getNpScene());

			PxU16 x = mCore.getSolverIterationCounts();
			minVelocityIters = PxU32(x >> 8);
			minPositionIters = PxU32(x & 0xff);
		}
		
		virtual	PxCudaContextManager*	getCudaContextManager() const { return mCudaContextManager; }

		virtual void				setRestOffset(PxReal restOffset) { scSetRestOffset(restOffset); }
		virtual	PxReal				getRestOffset() const { return mCore.getRestOffset(); }

		virtual	void				setContactOffset(PxReal contactOffset) { scSetContactOffset(contactOffset); }
		virtual	PxReal				getContactOffset() const { return mCore.getContactOffset(); }

		virtual void				setParticleContactOffset(PxReal particleContactOffset) { scSetParticleContactOffset(particleContactOffset); }
		virtual	PxReal				getParticleContactOffset() const { return mCore.getParticleContactOffset(); }

		virtual void				setSolidRestOffset(PxReal solidRestOffset) { scSetSolidRestOffset(solidRestOffset); }
		virtual	PxReal				getSolidRestOffset() const { return mCore.getSolidRestOffset(); }

		virtual	void				setMaxDepenetrationVelocity(PxReal v) {scSetMaxDepenetrationVelocity(v); }
		virtual	PxReal				getMaxDepenetrationVelocity(){ return mCore.getMaxDepenetrationVelocity(); }

		virtual	void				setMaxVelocity(PxReal v) { scSetMaxVelocity(v); }
		virtual	PxReal				getMaxVelocity() { return mCore.getMaxVelocity(); }

		virtual void						setParticleSystemCallback(PxParticleSystemCallback* callback) { mCore.setParticleSystemCallback(callback); }
		virtual PxParticleSystemCallback*	getParticleSystemCallback() const { return mCore.getParticleSystemCallback(); }
		// TOFIX
		virtual	void				enableCCD(bool enable) { scSnableCCD(enable); }

		virtual PxU32				createPhase(PxParticleMaterial* material, PxParticlePhaseFlags flags) = 0;

		virtual	PxFilterData		getSimulationFilterData() const
		{
			return mCore.getShapeCore().getSimulationFilterData();
		}

		virtual	void				setSimulationFilterData(const PxFilterData& data)
		{
			mCore.getShapeCore().setSimulationFilterData(data);
		}

		virtual	void				setParticleFlag(PxParticleFlag::Enum flag, bool val)
		{
			PxParticleFlags flags = mCore.getFlags();
			if (val)
				flags.raise(flag);
			else
				flags.clear(flag);

			mCore.setFlags(flags);
			scSetDirtyFlag();
		}

		virtual	void				setParticleFlags(PxParticleFlags flags)
		{
			mCore.setFlags(flags);
			scSetDirtyFlag();
		}

		virtual	PxParticleFlags		getParticleFlags() const
		{
			return mCore.getFlags();
		}

		
		void						setSolverType(const PxParticleSolverType::Enum solverType) { scSetSolverType(solverType); }
		virtual PxParticleSolverType::Enum	getSolverType() const { return mCore.getSolverType(); }

		virtual PxU32 getNbParticleMaterials() const 
		{
			const Sc::ParticleSystemShapeCore& shapeCore = mCore.getShapeCore();
			const Dy::ParticleSystemCore& core = shapeCore.getLLCore();
			return core.mUniqueMaterialHandles.size();
		}

		virtual PxU32 getParticleMaterials(PxParticleMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

		virtual void addParticleBuffer(PxParticleBuffer* userBuffer) = 0;
		virtual void removeParticleBuffer(PxParticleBuffer* userBuffer) = 0;

		virtual PxU32 getGpuParticleSystemIndex()
		{
			NP_READ_CHECK(NpBase::getNpScene());
			PX_CHECK_AND_RETURN_VAL(NpBase::getNpScene(), "NpParticleSystem::getGpuParticleSystemIndex: particle system must be in a scene.", 0xffffffff);

			if (NpBase::getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)
				return mCore.getSim()->getLowLevelParticleSystem()->getGpuRemapId();
			return 0xffffffff;
		}

		PX_FORCE_INLINE	const Sc::ParticleSystemCore&	getCore()	const	{ return mCore; }
		PX_FORCE_INLINE	Sc::ParticleSystemCore&			getCore()			{ return mCore; }
		static PX_FORCE_INLINE size_t					getCoreOffset()		{ return PX_OFFSET_OF_RT(NpParticleSystem, mCore); }

		PX_INLINE void scSetDirtyFlag()
		{
			NP_READ_CHECK(NpBase::getNpScene());
			NpScene* scene = NpBase::getNpScene();
			if (scene)
			{
				mCore.getSim()->getLowLevelParticleSystem()->mFlag |= Dy::ParticleSystemFlag::eUPDATE_PARAMS;
			}
		}

		PX_INLINE void scSetSleepThreshold(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setSleepThreshold(v);
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetRestOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setRestOffset(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetContactOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setContactOffset(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetParticleContactOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setParticleContactOffset(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetSolidRestOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setSolidRestOffset(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetFluidRestOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setFluidRestOffset(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetGridSizeX(const PxU32 v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setGridSizeX(v);
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetGridSizeY(const PxU32 v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setGridSizeY(v);
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetGridSizeZ(const PxU32 v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setGridSizeZ(v);
			//UPDATE_PVD_PROPERTY
		}
		
		PX_INLINE void scSnableCCD(const bool enable)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.enableCCD(enable);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetWind(const PxVec3& v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setWind(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetMaxDepenetrationVelocity(const PxReal v)
		{
			PX_CHECK_AND_RETURN(v > 0.f, "PxParticleSystem::setMaxDepenetrationVelocity: Max Depenetration Velocity must be > 0!");
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setMaxDepenetrationVelocity(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetMaxVelocity(const PxReal v)
		{
			PX_CHECK_AND_RETURN(v > 0.f, "PxParticleSystem::setMaxVelocity: Max Velocity must be > 0!");
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setMaxVelocity(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}

		PX_INLINE void scSetSolverType(const PxParticleSolverType::Enum solverType)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setSolverType(solverType);
			//UPDATE_PVD_PROPERTY
		}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
		PX_INLINE void scSetSparseGridParams(const PxSparseGridParams& params)
		{
			PX_CHECK_AND_RETURN(params.subgridSizeX > 1 && params.subgridSizeX % 2 == 0, "PxParticleSystem::setSparseGridParams: Sparse grid subgridSizeX must be > 1 and even number!");
			PX_CHECK_AND_RETURN(params.subgridSizeY > 1 && params.subgridSizeY % 2 == 0, "PxParticleSystem::setSparseGridParams: Sparse grid subgridSizeY must be > 1 and even number!");
			PX_CHECK_AND_RETURN(params.subgridSizeZ > 1 && params.subgridSizeZ % 2 == 0, "PxParticleSystem::setSparseGridParams: Sparse grid subgridSizeZ must be > 1 and even number!");
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setSparseGridParams(params);
			//UPDATE_PVD_PROPERTY
		}
#endif

#if PX_ENABLE_DEBUG_VISUALIZATION
		virtual void visualize(PxRenderOutput& out, NpScene& npScene)	const = 0;
#else
		PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
									// Debug name
		void						setName(const char* debugName)
		{
			NP_WRITE_CHECK(NpBase::getNpScene());
			NpActor::mName = debugName;
		}

		const char*					getName() const
		{
			NP_READ_CHECK(NpBase::getNpScene());
			return NpActor::mName;
		}

		virtual	void				setGridSizeX(PxU32 gridSizeX) { scSetGridSizeX(gridSizeX); }
		virtual	void				setGridSizeY(PxU32 gridSizeY) { scSetGridSizeY(gridSizeY); }
		virtual	void				setGridSizeZ(PxU32 gridSizeZ) { scSetGridSizeZ(gridSizeZ); }

		template<typename ParticleMaterialType>
		PxU32 getParticleMaterialsInternal(PxParticleMaterial** userBuffer, PxU32 bufferSize,
	                                       PxU32 startIndex = 0) const
	    {
		    const Sc::ParticleSystemShapeCore& shapeCore = mCore.getShapeCore();
		    const Dy::ParticleSystemCore& core = shapeCore.getLLCore();

		    NpMaterialManager<ParticleMaterialType>& matManager =
		        NpMaterialAccessor<ParticleMaterialType>::getMaterialManager(NpPhysics::getInstance());

		    PxU32 size = core.mUniqueMaterialHandles.size();
		    const PxU32 remainder = PxU32(PxMax<PxI32>(PxI32(size - startIndex), 0));
		    const PxU32 writeCount = PxMin(remainder, bufferSize);
		    for(PxU32 i = 0; i < writeCount; i++)
		    {
			    userBuffer[i] = matManager.getMaterial(core.mUniqueMaterialHandles[startIndex + i]);
		    }
		    return writeCount;
	    }


	protected:
		Sc::ParticleSystemCore		mCore;
		PxCudaContextManager*		mCudaContextManager;
		PxU32						mNextPhaseGroupID;
	};


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


	class NpPBDParticleSystem : public NpParticleSystem<PxPBDParticleSystem>
	{
	public:

		NpPBDParticleSystem(PxU32 maxNeighborhood, PxCudaContextManager& contextManager);
	
		virtual						~NpPBDParticleSystem(){}

		virtual	PxU32				createPhase(PxParticleMaterial* material, PxParticlePhaseFlags flags) PX_OVERRIDE;
	
		virtual void				release();


		virtual	void				setWind(const PxVec3& wind) { scSetWind(wind); }
		virtual	PxVec3				getWind() const { return mCore.getWind(); }

		virtual void				setFluidBoundaryDensityScale(PxReal fluidBoundaryDensityScale) { scSetFluidBoundaryDensityScale(fluidBoundaryDensityScale); }
		virtual	PxReal				getFluidBoundaryDensityScale() const { return mCore.getFluidBoundaryDensityScale(); }
		
		virtual void				setFluidRestOffset(PxReal fluidRestOffset) { scSetFluidRestOffset(fluidRestOffset); }
		virtual	PxReal				getFluidRestOffset() const { return mCore.getFluidRestOffset(); }


#if PX_ENABLE_DEBUG_VISUALIZATION
		virtual void				visualize(PxRenderOutput& out, NpScene& npScene)	const;
#else
		PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif	

		//external API
		virtual PxActorType::Enum	getType() const { return PxActorType::ePBD_PARTICLESYSTEM; }

		virtual PxU32				getParticleMaterials(PxParticleMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const PX_OVERRIDE;

		virtual void				addParticleBuffer(PxParticleBuffer* particleBuffer);
		virtual void				removeParticleBuffer(PxParticleBuffer* particleBuffer);

		virtual void				addRigidAttachment(PxRigidActor* actor);
		virtual void				removeRigidAttachment(PxRigidActor* actor);

	private:

		PX_FORCE_INLINE void scSetFluidBoundaryDensityScale(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setFluidBoundaryDensityScale(v);
			scSetDirtyFlag();
			//UPDATE_PVD_PROPERTY
		}
	};

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	class NpFLIPParticleSystem : public NpParticleSystem<PxFLIPParticleSystem>
	{
	public:

		NpFLIPParticleSystem(PxCudaContextManager& contextManager);

		virtual						~NpFLIPParticleSystem() {}

		virtual	PxU32				createPhase(PxParticleMaterial* material, PxParticlePhaseFlags flags) PX_OVERRIDE;
		virtual void				release();

		virtual void				setSparseGridParams(const PxSparseGridParams& params) { scSetSparseGridParams(params); }
		virtual PxSparseGridParams	getSparseGridParams() const { return mCore.getSparseGridParams(); }

		virtual void*				getSparseGridDataPointer(PxSparseGridDataFlag::Enum flags);

		virtual void				getSparseGridCoord(PxI32& x, PxI32& y, PxI32& z, PxU32 id);

		virtual void				setFLIPParams(const PxFLIPParams& params) { scSetFLIPParams(params); }
		virtual PxFLIPParams		getFLIPParams() const { return mCore.getFLIPParams(); }

#if PX_ENABLE_DEBUG_VISUALIZATION
		virtual void				visualize(PxRenderOutput& out, NpScene& npScene)	const;
#else
		PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

		virtual PxU32				getParticleMaterials(PxParticleMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const PX_OVERRIDE;

		virtual void				addParticleBuffer(PxParticleBuffer* particleBuffer);
		virtual void				removeParticleBuffer(PxParticleBuffer* particleBuffer);

		virtual void				addRigidAttachment(PxRigidActor* /*actor*/) {}
		virtual void				removeRigidAttachment(PxRigidActor* /*actor*/) {}
		
		//external API
		virtual						PxActorType::Enum	getType() const { return PxActorType::eFLIP_PARTICLESYSTEM; }


	
	private:

		PX_FORCE_INLINE void scSetFLIPParams(const PxFLIPParams& params)
		{
			PX_CHECK_AND_RETURN(params.blendingFactor >= 0.f && params.blendingFactor <= 1.f, "PxParticleSystem::setFLIPParams: FLIP blending factor must be >= 0 and <= 1!");
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setFLIPParams(params);
			//UPDATE_PVD_PROPERTY
		}

	};

	class NpMPMParticleSystem :public NpParticleSystem<PxMPMParticleSystem>
	{
	public:

		NpMPMParticleSystem(PxCudaContextManager& contextManager);

		virtual						~NpMPMParticleSystem() {}

		virtual	PxU32				createPhase(PxParticleMaterial* material, PxParticlePhaseFlags flags) PX_OVERRIDE;
		virtual void				release();

		virtual void				setSparseGridParams(const PxSparseGridParams& params) { scSetSparseGridParams(params); }
		virtual PxSparseGridParams	getSparseGridParams() const { return mCore.getSparseGridParams(); }

		virtual void*				getSparseGridDataPointer(PxSparseGridDataFlag::Enum flags);

		virtual void				getSparseGridCoord(PxI32& x, PxI32& y, PxI32& z, PxU32 id);

		virtual void				setMPMParams(const PxMPMParams& params) { scSetMPMParams(params); }
		virtual PxMPMParams			getMPMParams() const { return mCore.getMPMParams(); }

		virtual		void*			getMPMDataPointer(PxMPMParticleDataFlag::Enum flags);
#if PX_ENABLE_DEBUG_VISUALIZATION
		virtual void				visualize(PxRenderOutput& out, NpScene& npScene)	const;
#else
		PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif	

		virtual PxU32				getParticleMaterials(PxParticleMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const PX_OVERRIDE;

		virtual void				addParticleBuffer(PxParticleBuffer* particleBuffer);
		virtual void				removeParticleBuffer(PxParticleBuffer* particleBuffer);

		virtual void				addRigidAttachment(PxRigidActor* actor);
		virtual void				removeRigidAttachment(PxRigidActor* actor);
		
		//external API
		virtual						PxActorType::Enum	getType() const { return PxActorType::eMPM_PARTICLESYSTEM; }
	
	private:
		PX_INLINE void scSetMPMParams(const PxMPMParams& params)
		{
			mCore.setMPMParams(params);
			//UPDATE_PVD_PROPERTY
		}
	};
#endif

}
#endif

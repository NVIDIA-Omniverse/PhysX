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

#ifndef NP_PBD_PARTICLE_SYSTEM_H
#define NP_PBD_PARTICLE_SYSTEM_H

#include "PxPBDParticleSystem.h"
#include "PxParticleMaterial.h"
#include "PxSparseGridParams.h"
#include "NpActorTemplate.h"
#include "ScParticleSystemSim.h"

namespace physx
{
	class PxCudaContextManager;
	class PxRigidActor;
	class PxSerializationContext;
	class PxParticleBuffer;
	class NpScene;
	class NpParticleBuffer;
	class NpParticleAndDiffuseBuffer;
	class NpParticleClothBuffer;
	class NpParticleRigidBuffer;

	class NpPBDParticleSystem : public NpActorTemplate<PxPBDParticleSystem>
	{
	public:
		NpPBDParticleSystem(PxU32 maxNeighborhood, PxReal neighborhoodScale, PxCudaContextManager& contextManager);
		NpPBDParticleSystem(PxBaseFlags baseFlags);
	
		virtual						~NpPBDParticleSystem(){}

		void						exportData(PxSerializationContext& /*context*/) const {}

		// PxBase
		virtual void				release() PX_OVERRIDE;
		//~PxBase

		// PxActor
		virtual PxActorType::Enum	getType() const PX_OVERRIDE { return PxActorType::ePBD_PARTICLESYSTEM; }
		//~PxActor

		// PxPBDParticleSystem
		virtual	PxCudaContextManager* getCudaContextManager() const PX_OVERRIDE { return mCudaContextManager; }

		virtual	PxBounds3			getWorldBounds(PxReal inflation = 1.01f) const PX_OVERRIDE;

		virtual	void				setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters = 1) PX_OVERRIDE;
		virtual	void				getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const PX_OVERRIDE;

		virtual void				setRestOffset(PxReal restOffset) PX_OVERRIDE { scSetRestOffset(restOffset); }
		virtual	PxReal				getRestOffset() const PX_OVERRIDE { return mCore.getRestOffset(); }

		virtual	void				setContactOffset(PxReal contactOffset) PX_OVERRIDE { scSetContactOffset(contactOffset); }
		virtual	PxReal				getContactOffset() const PX_OVERRIDE { return mCore.getContactOffset(); }

		virtual void				setParticleContactOffset(PxReal particleContactOffset) PX_OVERRIDE { scSetParticleContactOffset(particleContactOffset); }
		virtual	PxReal				getParticleContactOffset() const PX_OVERRIDE { return mCore.getParticleContactOffset(); }

		virtual void				setSolidRestOffset(PxReal solidRestOffset) PX_OVERRIDE { scSetSolidRestOffset(solidRestOffset); }
		virtual	PxReal				getSolidRestOffset() const PX_OVERRIDE { return mCore.getSolidRestOffset(); }

		virtual	void				setMaxDepenetrationVelocity(PxReal v) PX_OVERRIDE { scSetMaxDepenetrationVelocity(v); }
		virtual	PxReal				getMaxDepenetrationVelocity() const PX_OVERRIDE { return mCore.getMaxDepenetrationVelocity(); }

		virtual	void				setMaxVelocity(PxReal v) PX_OVERRIDE { scSetMaxVelocity(v); }
		virtual	PxReal				getMaxVelocity() const PX_OVERRIDE { return mCore.getMaxVelocity(); }

		virtual void				setParticleSystemCallback(PxParticleSystemCallback* callback) PX_OVERRIDE { mCore.setParticleSystemCallback(callback); }
		virtual PxParticleSystemCallback* getParticleSystemCallback() const PX_OVERRIDE { return mCore.getParticleSystemCallback(); }

		//deprecated
		virtual	void				enableCCD(bool enable) PX_OVERRIDE { scEnableCCD(enable); }

		virtual PxParticleLockFlags getParticleLockFlags() const PX_OVERRIDE { return mCore.getLockFlags(); }
		virtual void				setParticleLockFlag(PxParticleLockFlag::Enum flag, bool value) PX_OVERRIDE { scSetParticleLockFlag(flag, value); }
		virtual void				setParticleLockFlags(PxParticleLockFlags flags) PX_OVERRIDE { scSetParticleLockFlags(flags); }

		virtual	PxFilterData		getSimulationFilterData() const PX_OVERRIDE { return mCore.getShapeCore().getSimulationFilterData(); }
		virtual	void				setSimulationFilterData(const PxFilterData& data) PX_OVERRIDE { scSetSimulationFilterData(data); }

		virtual	void				setParticleFlag(PxParticleFlag::Enum flag, bool val) PX_OVERRIDE { scSetParticleFlag(flag, val); }
		virtual	void				setParticleFlags(PxParticleFlags flags) PX_OVERRIDE { scSetParticleFlags(flags); }
		virtual	PxParticleFlags		getParticleFlags() const PX_OVERRIDE { return mCore.getFlags(); }

		virtual	PxU32				createPhase(PxParticleMaterial* material, PxParticlePhaseFlags flags) PX_OVERRIDE;
	
		virtual	void				setWind(const PxVec3& wind) PX_OVERRIDE { scSetWind(wind); }
		virtual	PxVec3				getWind() const PX_OVERRIDE { return mCore.getWind(); }

		virtual void				setFluidBoundaryDensityScale(PxReal fluidBoundaryDensityScale) PX_OVERRIDE { scSetFluidBoundaryDensityScale(fluidBoundaryDensityScale); }
		virtual	PxReal				getFluidBoundaryDensityScale() const PX_OVERRIDE { return mCore.getFluidBoundaryDensityScale(); }
		
		virtual void				setFluidRestOffset(PxReal fluidRestOffset) PX_OVERRIDE { scSetFluidRestOffset(fluidRestOffset); }
		virtual	PxReal				getFluidRestOffset() const PX_OVERRIDE { return mCore.getFluidRestOffset(); }

		virtual	void				setGridSizeX(PxU32 gridSizeX) PX_OVERRIDE { scSetGridSizeX(gridSizeX); }
		virtual	PxU32				getGridSizeX() const { return mCore.getGridSizeX(); }

		virtual	void				setGridSizeY(PxU32 gridSizeY) PX_OVERRIDE { scSetGridSizeY(gridSizeY); }
		virtual	PxU32				getGridSizeY() const PX_OVERRIDE { return mCore.getGridSizeY(); }

		virtual	void				setGridSizeZ(PxU32 gridSizeZ) PX_OVERRIDE { scSetGridSizeZ(gridSizeZ); }
		virtual	PxU32				getGridSizeZ() const PX_OVERRIDE { return mCore.getGridSizeZ(); }

		virtual PxU32				getParticleMaterials(PxParticleMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const PX_OVERRIDE;
		virtual PxU32				getNbParticleMaterials() const PX_OVERRIDE { return mCore.getShapeCore().getLLCore().mUniqueMaterialHandles.size(); }

		virtual void				addParticleBuffer(PxParticleBuffer* particleBuffer) PX_OVERRIDE;
		virtual void				removeParticleBuffer(PxParticleBuffer* particleBuffer) PX_OVERRIDE;

		virtual void				addRigidAttachment(PxRigidActor* actor) PX_OVERRIDE;
		virtual void				removeRigidAttachment(PxRigidActor* actor) PX_OVERRIDE;

		virtual PxU32				getGpuParticleSystemIndex() PX_OVERRIDE;
		//~PxPBDParticleSystem

	public:
		PX_FORCE_INLINE	const Sc::ParticleSystemCore& getCore()	const { return mCore; }
		PX_FORCE_INLINE	Sc::ParticleSystemCore& getCore() { return mCore; }
		static PX_FORCE_INLINE size_t getCoreOffset() { return PX_OFFSET_OF_RT(NpPBDParticleSystem, mCore); }

#if PX_ENABLE_DEBUG_VISUALIZATION
		void visualize(PxRenderOutput& out, NpScene& npScene) const;
#else
		PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	private:

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
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, restOffset, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetContactOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setContactOffset(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, contactOffset, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetParticleContactOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setParticleContactOffset(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleContactOffset, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetSolidRestOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setSolidRestOffset(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, solidRestOffset, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetFluidRestOffset(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setFluidRestOffset(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, fluidRestOffset, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scEnableCCD(bool enable)
		{
			PxParticleFlags flags = mCore.getFlags();
			if (enable)
				flags.raise(PxParticleFlag::eENABLE_SPECULATIVE_CCD);
			else
				flags.clear(PxParticleFlag::eENABLE_SPECULATIVE_CCD);

			mCore.setFlags(flags);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleFlags, static_cast<PxPBDParticleSystem&>(*this), flags);
		}

		PX_INLINE void scSetParticleLockFlag(const PxParticleLockFlag::Enum flag, bool value)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			PxParticleLockFlags flags = mCore.getLockFlags();
			if (value)
				flags = flags | flag;
			else
				flags = flags & (~flag);

			scSetParticleLockFlags(flags);
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleLockFlags, static_cast<PxPBDParticleSystem&>(*this), flags);
		}

		PX_INLINE void scSetParticleLockFlags(const PxParticleLockFlags flags)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setLockFlags(flags);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleLockFlags, static_cast<PxPBDParticleSystem&>(*this), flags);
		}

		PX_INLINE void scSetSimulationFilterData(const PxFilterData& data)
		{
			mCore.getShapeCore().setSimulationFilterData(data);
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, simulationFilterData, static_cast<PxPBDParticleSystem&>(*this), data);
		}

		PX_INLINE void scSetParticleFlag(PxParticleFlag::Enum flag, bool val)
		{
			PxParticleFlags flags = mCore.getFlags();
			if (val)
				flags.raise(flag);
			else
				flags.clear(flag);

			mCore.setFlags(flags);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleFlags, static_cast<PxPBDParticleSystem&>(*this), flags);
		}

		PX_INLINE void scSetParticleFlags(PxParticleFlags flags)
		{
			mCore.setFlags(flags);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleFlags, static_cast<PxPBDParticleSystem&>(*this), flags);
		}

		PX_INLINE void scSetWind(const PxVec3& v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setWind(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, wind, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetMaxDepenetrationVelocity(const PxReal v)
		{
			PX_CHECK_AND_RETURN(v > 0.f, "PxParticleSystem::setMaxDepenetrationVelocity: Max Depenetration Velocity must be > 0!");
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setMaxDepenetrationVelocity(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, maxDepenetrationVelocity, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetMaxVelocity(const PxReal v)
		{
			PX_CHECK_AND_RETURN(v > 0.f, "PxParticleSystem::setMaxVelocity: Max Velocity must be > 0!");
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setMaxVelocity(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, maxVelocity, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_FORCE_INLINE void scSetFluidBoundaryDensityScale(const PxReal v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setFluidBoundaryDensityScale(v);
			scSetDirtyFlag();
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, fluidBoundaryDensityScale, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetGridSizeX(const PxU32 v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setGridSizeX(v);
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeX, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetGridSizeY(const PxU32 v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setGridSizeY(v);
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeY, static_cast<PxPBDParticleSystem&>(*this), v);
		}

		PX_INLINE void scSetGridSizeZ(const PxU32 v)
		{
			PX_ASSERT(!NpBase::isAPIWriteForbidden());
			mCore.setGridSizeZ(v);
			OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeZ, static_cast<PxPBDParticleSystem&>(*this), v);
		}

	public:
		PxArray<NpParticleBuffer*>				mParticleBuffers;
		PxArray<NpParticleAndDiffuseBuffer*>	mParticleDiffuseBuffers;
		PxArray<NpParticleClothBuffer*>			mParticleClothBuffers;
		PxArray<NpParticleRigidBuffer*>			mParticleRigidBuffers;

	private:
		Sc::ParticleSystemCore mCore;
		PxCudaContextManager* mCudaContextManager;
		PxU32 mNextPhaseGroupID;
	};

}
#endif

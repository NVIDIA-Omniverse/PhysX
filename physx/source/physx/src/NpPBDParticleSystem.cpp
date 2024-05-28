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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "NpPBDParticleSystem.h"
#include "NpParticleBuffer.h"
#include "NpPhysics.h"

#include "ScParticleSystemSim.h"
#include "CmVisualization.h"

using namespace physx;

namespace physx
{
#if PX_ENABLE_DEBUG_VISUALIZATION
	static void visualizeParticleSystem(PxRenderOutput& out, NpScene& npScene, const Sc::ParticleSystemCore& core)
	{
		if (!(core.getActorFlags() & PxActorFlag::eVISUALIZATION))
			return;

		const Sc::Scene& scScene = npScene.getScScene();

		const bool visualizeAABBs = scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_AABBS) != 0.0f;
		if (visualizeAABBs)
		{
			out << PxU32(PxDebugColor::eARGB_YELLOW) << PxMat44(PxIdentity);
			Cm::renderOutputDebugBox(out, core.getSim()->getBounds());
		}
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	////////////////////////////////////////////////////////////////////////////////////////

	NpPBDParticleSystem::NpPBDParticleSystem(PxU32 maxNeighborhood, PxReal neighborhoodScale, PxCudaContextManager& cudaContextManager) :
		NpActorTemplate<PxPBDParticleSystem>(PxConcreteType::ePBD_PARTICLESYSTEM, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::ePBD_PARTICLESYSTEM),
		mCore(PxActorType::ePBD_PARTICLESYSTEM),
		mCudaContextManager(&cudaContextManager),
		mNextPhaseGroupID(0)
	{
		mCore.getShapeCore().initializeLLCoreData(maxNeighborhood, neighborhoodScale);
	}

	NpPBDParticleSystem::NpPBDParticleSystem(PxBaseFlags baseFlags) : 
		NpActorTemplate<PxPBDParticleSystem>(baseFlags),
		mCore(PxEmpty),
		mCudaContextManager(NULL)
	{}
	
	PxBounds3 NpPBDParticleSystem::getWorldBounds(PxReal inflation) const
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

	void NpPBDParticleSystem::setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters)
	{
		NpScene* npScene = NpBase::getNpScene();
		NP_WRITE_CHECK(npScene);
		PX_CHECK_AND_RETURN(minPositionIters > 0, "NpParticleSystem::setSolverIterationCounts: positionIters must be more than zero!");
		PX_CHECK_AND_RETURN(minPositionIters <= 255, "NpParticleSystem::setSolverIterationCounts: positionIters must be no greater than 255!");
		PX_CHECK_AND_RETURN(minVelocityIters <= 255, "NpParticleSystem::setSolverIterationCounts: velocityIters must be no greater than 255!");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxParticleSystem::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.")

			mCore.setSolverIterationCounts((minVelocityIters & 0xff) << 8 | (minPositionIters & 0xff));
		OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, positionIterations, static_cast<PxPBDParticleSystem&>(*this), minPositionIters); // @@@
		OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, velocityIterations, static_cast<PxPBDParticleSystem&>(*this), minVelocityIters); // @@@
	}

	void NpPBDParticleSystem::getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const
	{
		NP_READ_CHECK(NpBase::getNpScene());

		PxU16 x = mCore.getSolverIterationCounts();
		minVelocityIters = PxU32(x >> 8);
		minPositionIters = PxU32(x & 0xff);
	}

	PxU32 NpPBDParticleSystem::createPhase(PxParticleMaterial* material, const PxParticlePhaseFlags flags)
	{
		if (material->getConcreteType() == PxConcreteType::ePBD_MATERIAL)
		{
			Sc::ParticleSystemShapeCore& shapeCore = mCore.getShapeCore();
			Dy::ParticleSystemCore& core = shapeCore.getLLCore();

			PxU16 materialHandle = static_cast<NpPBDMaterial*>(material)->mMaterial.mMaterialIndex;
		
			const PxU32 groupID = mNextPhaseGroupID++;

			core.mPhaseGroupToMaterialHandle.pushBack(materialHandle);
			PxU16* foundHandle = core.mUniqueMaterialHandles.find(materialHandle);
			if(foundHandle == core.mUniqueMaterialHandles.end())
			{
				core.mUniqueMaterialHandles.pushBack(materialHandle);
			}

			if (mCore.getSim())
				mCore.getSim()->getLowLevelParticleSystem()->mFlag |= Dy::ParticleSystemFlag::eUPDATE_PHASE;

			return (groupID & PxParticlePhaseFlag::eParticlePhaseGroupMask)
				| (PxU32(flags) & PxParticlePhaseFlag::eParticlePhaseFlagsMask);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPBDParticleSystem:createPhase(): the provided material is not supported by this type of particle system.");
			return 0;
		}
	}

	void NpPBDParticleSystem::release()
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		for (PxU32 i = 0; i < mParticleBuffers.size(); ++i)
		{
			NpParticleBuffer* npBuffer = mParticleBuffers[i];
			npBuffer->setParticleSystem(NULL);
		}

		for (PxU32 i = 0; i < mParticleDiffuseBuffers.size(); ++i)
		{
			NpParticleAndDiffuseBuffer* npBuffer = mParticleDiffuseBuffers[i];
			npBuffer->setParticleSystem(NULL);
		}

		for (PxU32 i = 0; i < mParticleClothBuffers.size(); ++i)
		{
			NpParticleClothBuffer* npBuffer = mParticleClothBuffers[i];
			npBuffer->setParticleSystem(NULL);
		}

		for (PxU32 i = 0; i < mParticleRigidBuffers.size(); ++i)
		{
			NpParticleRigidBuffer* npBuffer = mParticleRigidBuffers[i];
			npBuffer->setParticleSystem(NULL);
		}

		//	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

		if (npScene)
		{
			npScene->scRemoveParticleSystem(*this);
			npScene->removeFromParticleSystemList(*this);
		}

		PX_ASSERT(!isAPIWriteForbidden());
		NpDestroyParticleSystem(this);
	}

	PxU32 NpPBDParticleSystem::getParticleMaterials(PxParticleMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
	{
		const Sc::ParticleSystemShapeCore& shapeCore = mCore.getShapeCore();
		const Dy::ParticleSystemCore& core = shapeCore.getLLCore();

		NpMaterialManager<NpPBDMaterial>& matManager = NpMaterialAccessor<NpPBDMaterial>::getMaterialManager(NpPhysics::getInstance());

		PxU32 size = core.mUniqueMaterialHandles.size();
		const PxU32 remainder = PxU32(PxMax<PxI32>(PxI32(size - startIndex), 0));
		const PxU32 writeCount = PxMin(remainder, bufferSize);
		for (PxU32 i = 0; i < writeCount; i++)
		{
			userBuffer[i] = matManager.getMaterial(core.mUniqueMaterialHandles[startIndex + i]);
		}
		return writeCount;
	}

	PxU32 NpPBDParticleSystem::getGpuParticleSystemIndex()
	{
		NP_READ_CHECK(NpBase::getNpScene());
		PX_CHECK_AND_RETURN_VAL(NpBase::getNpScene(), "NpParticleSystem::getGpuParticleSystemIndex: particle system must be in a scene.", 0xffffffff);

		if (NpBase::getNpScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)
			return mCore.getSim()->getLowLevelParticleSystem()->getGpuRemapId();
		return 0xffffffff;
	}

	namespace
	{
		template<class NpBufferType>
		bool addParticleBufferT(NpPBDParticleSystem* npPS, PxArray<NpBufferType*>& npBuffers,
			PxArray<PxsParticleBuffer*>& pxsBuffers, bool& updateFlag, PxParticleBuffer* pxBuffer)
		{
			NpBufferType* npBuffer = static_cast<NpBufferType*>(pxBuffer);
			PX_CHECK_AND_RETURN_VAL(npBuffer->mParticleSystem == NULL,
				"NpPBDParticleSystem::addParticleBuffer: buffer was already added to particle system!",
				false);
			PX_ASSERT(npBuffer->mBufferIndex == 0xffffffff);
			PX_ASSERT(pxsBuffers.size() == npBuffers.size());
			npBuffer->mBufferIndex = pxsBuffers.size();
			npBuffer->mParticleSystem = npPS;
			npBuffers.pushBack(npBuffer);
			pxsBuffers.pushBack(npBuffer->mGpuBuffer);
			updateFlag |= true;
			return true;
		}

		template<class NpBufferType>
		bool removeParticleBufferT(NpPBDParticleSystem* npPS, PxArray<NpBufferType*>& npBuffers,
			PxArray<PxsParticleBuffer*>& pxsBuffers, bool& updateFlag, PxParticleBuffer* pxBuffer)
		{
			PX_UNUSED(npPS);
			NpBufferType* npBuffer = static_cast<NpBufferType*>(pxBuffer);
			PX_CHECK_AND_RETURN_VAL(npBuffer->mParticleSystem == npPS,
				"NpPBDParticleSystem::removeParticleBuffer: buffer was not added to this particle system!",
				false);
			PX_ASSERT(npBuffer->mBufferIndex != 0xffffffff);
			const PxU32 index = npBuffer->mBufferIndex;

			if (index < npBuffers.size())
			{
				npBuffers.replaceWithLast(index);
				pxsBuffers.replaceWithLast(index);
				if (npBuffers.size() > index)
				{
					NpBufferType& movedNpBuffer = static_cast<NpBufferType&>(*npBuffers[index]);
					movedNpBuffer.mBufferIndex = index;
				}
				npBuffer->mBufferIndex = 0xffffffff;
				npBuffer->mParticleSystem = NULL;
				updateFlag |= true;
				return true;
			}
			return false;
		}
	}

	void NpPBDParticleSystem::addParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(particleBuffer != NULL, "NpPBDParticleSystem::addParticleBuffer: particle buffer is NULL!");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpPBDParticleSystem::addParticleBuffer: this function cannot be called when the particle system is not inserted into the scene!");
			
		Dy::ParticleSystemCore& llCore = mCore.getShapeCore().getLLCore();
		bool added = false;
		switch (particleBuffer->getConcreteType())
		{
			case (PxConcreteType::ePARTICLE_BUFFER):
			{
				added = addParticleBufferT<NpParticleBuffer>(this, mParticleBuffers,
					llCore.mParticleBuffers, llCore.mParticleBufferUpdate, particleBuffer);
				break;
			}
			case (PxConcreteType::ePARTICLE_DIFFUSE_BUFFER):
			{
				added = addParticleBufferT<NpParticleAndDiffuseBuffer>(this, mParticleDiffuseBuffers,
					llCore.mParticleDiffuseBuffers, llCore.mParticleDiffuseBufferUpdate, particleBuffer);
				break;
			}
			case (PxConcreteType::ePARTICLE_CLOTH_BUFFER):
			{
				added = addParticleBufferT<NpParticleClothBuffer>(this, mParticleClothBuffers,
					llCore.mParticleClothBuffers, llCore.mParticleClothBufferUpdate, particleBuffer);
				break;
			}
			case (PxConcreteType::ePARTICLE_RIGID_BUFFER):
			{
				added = addParticleBufferT<NpParticleRigidBuffer>(this, mParticleRigidBuffers,
					llCore.mParticleRigidBuffers, llCore.mParticleRigidBufferUpdate, particleBuffer);
				break;
			}
			default:
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "NpPBDParticleSystem::addParticleBuffer: Error, this buffer does not have a valid type!");
				break;
			}
		}

		if (added)
		{
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers,
				static_cast<const PxPBDParticleSystem&>(*this), *particleBuffer);
		}
	}

	void NpPBDParticleSystem::removeParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		PX_CHECK_AND_RETURN(particleBuffer != NULL, "NpPBDParticleSystem::removeParticleBuffer: particle buffer is NULL!");

		Dy::ParticleSystemCore& llCore = mCore.getShapeCore().getLLCore();
		bool removed = false;
		switch (particleBuffer->getConcreteType())
		{
			case (PxConcreteType::ePARTICLE_BUFFER):
			{
				removed = removeParticleBufferT<NpParticleBuffer>(this, mParticleBuffers, 
					llCore.mParticleBuffers, llCore.mParticleBufferUpdate, particleBuffer);
				break;
			}
			case (PxConcreteType::ePARTICLE_DIFFUSE_BUFFER):
			{
				removed = removeParticleBufferT<NpParticleAndDiffuseBuffer>(this, mParticleDiffuseBuffers, 
					llCore.mParticleDiffuseBuffers, llCore.mParticleDiffuseBufferUpdate, particleBuffer);
				break;
			}
			case (PxConcreteType::ePARTICLE_CLOTH_BUFFER):
			{
				removed = removeParticleBufferT<NpParticleClothBuffer>(this, mParticleClothBuffers, 
					llCore.mParticleClothBuffers, llCore.mParticleClothBufferUpdate, particleBuffer);
				break;
			}
			case (PxConcreteType::ePARTICLE_RIGID_BUFFER):
			{
				removed = removeParticleBufferT<NpParticleRigidBuffer>(this, mParticleRigidBuffers, 
					llCore.mParticleRigidBuffers, llCore.mParticleRigidBufferUpdate, particleBuffer);
				break;
			}
			default:
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "NpPBDParticleSystem::removeParticleBuffer: Error, this buffer does not have a valid type!");
				break;
			}
		}

		if (removed)
		{
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers,
				static_cast<const PxPBDParticleSystem&>(*this), *particleBuffer);
		}
	}

#if PX_ENABLE_DEBUG_VISUALIZATION
	void NpPBDParticleSystem::visualize(PxRenderOutput& out, NpScene& npScene)	const
	{
		visualizeParticleSystem(out, npScene, mCore);
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	static void internalAddRigidAttachment(PxRigidActor* actor, Sc::ParticleSystemCore& psCore)
	{
		Sc::BodyCore* core = getBodyCore(actor);

		psCore.addRigidAttachment(core);
	}

	static void internalRemoveRigidAttachment(PxRigidActor* actor, Sc::ParticleSystemCore& psCore)
	{
		Sc::BodyCore* core = getBodyCore(actor);
		psCore.removeRigidAttachment(core);
	}

	void NpPBDParticleSystem::addRigidAttachment(PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpPBDParticleSystem::addRigidAttachment: particleSystem must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpPBDParticleSystem::addRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpPBDParticleSystem::addRigidAttachment: Illegal to call while simulation is running.");

		internalAddRigidAttachment(actor, mCore);
	}

	void NpPBDParticleSystem::removeRigidAttachment(PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpPBDParticleSystem::removeRigidAttachment: particleSystem must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpPBDParticleSystem::removeRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpPBDParticleSystem::removeRigidAttachment: Illegal to call while simulation is running.");

		internalRemoveRigidAttachment(actor, mCore);
	}

}

#endif //PX_SUPPORT_GPU_PHYSX

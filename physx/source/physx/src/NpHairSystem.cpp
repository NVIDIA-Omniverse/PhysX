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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "NpHairSystem.h"
#include "NpCheck.h"
#include "NpScene.h"
#include "ScHairSystemSim.h"
#include "NpFactory.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "ScBodyCore.h"
#include "ScBodySim.h"
#include "geometry/PxHairSystemDesc.h"
#include "PxsMemoryManager.h"
#include "NpSoftBody.h"

#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "GuTetrahedronMeshUtils.h"

using namespace physx;

namespace physx
{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	NpHairSystem::NpHairSystem(PxCudaContextManager& cudaContextManager) :
		NpActorTemplate(PxConcreteType::eHAIR_SYSTEM, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::eHAIRSYSTEM),
		mCudaContextManager(&cudaContextManager),
		mMemoryManager(NULL),
		mHostMemoryAllocator(NULL),
		mDeviceMemoryAllocator(NULL),
		mRestPositionActor(NULL)
	{
		init();
	}


	NpHairSystem::NpHairSystem(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager) :
		NpActorTemplate(baseFlags),
		mCudaContextManager(&cudaContextManager),
		mMemoryManager(NULL),
		mHostMemoryAllocator(NULL),
		mDeviceMemoryAllocator(NULL),
		mRestPositionActor(NULL)
	{
		init();
	}

	NpHairSystem::~NpHairSystem()
	{
		releaseAllocator();
	}

	void NpHairSystem::init()
	{
		mCore.getShapeCore().createBuffers(mCudaContextManager);
		createAllocator();

		mSoftbodyAttachments.getAllocator().setCallback(mHostMemoryAllocator);
	}

	void NpHairSystem::releaseAllocator()
	{
		// destroy internal buffers if they exist before destroying allocators
		if(mStrandPastEndIndicesInternal.size() > 0)
			mStrandPastEndIndicesInternal.reset();
		if (mPosInvMassInternal.size() > 0)
			mPosInvMassInternal.reset();
		if (mVelInternal.size() > 0)
			mVelInternal.reset();
		if (mParticleRigidAttachmentsInternal.size() > 0)
			mParticleRigidAttachmentsInternal.reset();
		if (mRestPositionsInternal.size() > 0)
			mRestPositionsInternal.reset();

		if (mRestPositionTransformPinnedBuf.size() > 0)
		{
			mRestPositionTransformPinnedBuf.reset();
			mCore.getShapeCore().getLLCore().mRestPositionsTransform = NULL;
		}

		mSoftbodyAttachments.reset();

		if (mMemoryManager != NULL)
		{
			mMemoryManager->~PxsMemoryManager();
			mHostMemoryAllocator = NULL; // released by memory manager
			mDeviceMemoryAllocator = NULL; // released by memory manager
			PX_FREE(mMemoryManager);
		}
	}

	PxBounds3 NpHairSystem::getWorldBounds(float inflation) const
	{
		NP_READ_CHECK(getNpScene());

		if (!getNpScene())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Querying bounds of a PxHairSystem which is not part of a PxScene is not supported.");
			return PxBounds3::empty();
		}

		const Sc::HairSystemSim* sim = mCore.getSim();
		PX_ASSERT(sim);

		PX_SIMD_GUARD;

		PxBounds3 bounds = sim->getBounds();
		PX_ASSERT(bounds.isValid());

		// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
		const PxVec3 center = bounds.getCenter();
		const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
		return PxBounds3::centerExtents(center, inflatedExtents);
	}

#if PX_ENABLE_DEBUG_VISUALIZATION
	void NpHairSystem::visualize(PxRenderOutput& out, NpScene& scene) const
	{
		if(!(mCore.getActorFlags() & PxActorFlag::eVISUALIZATION))
			return;

		const Sc::Scene& scScene = scene.getScScene();

		const bool visualizeAABBs = scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_AABBS) != 0.0f;	
		if(visualizeAABBs)
		{
			out << PxU32(PxDebugColor::eARGB_YELLOW) << PxMat44(PxIdentity);
			Cm::renderOutputDebugBox(out, mCore.getSim()->getBounds());
		}
	}
#endif


	void NpHairSystem::setHairSystemFlag(PxHairSystemFlag::Enum flag, bool val)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxHairSystem::setHairSystemFlag() not allowed while simulation is running. Call will be ignored.");

		PxHairSystemFlags flags = mCore.getFlags();
		if (val)
		{
			flags.raise(flag);
		}
		else
		{
			flags.clear(flag);
		}
		mCore.setFlags(flags);
		mCore.getShapeCore().getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	void NpHairSystem::setReadRequestFlag(PxHairSystemData::Enum flag, bool val)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxHairSystem::setReadRequestFlag() not allowed while simulation is running. Call will be ignored.");

		if (val)
		{
			mCore.getShapeCore().getLLCore().mReadRequests.raise(flag);
		}
		else
		{
			mCore.getShapeCore().getLLCore().mReadRequests.clear(flag);
		}
	}

	void NpHairSystem::setReadRequestFlags(PxHairSystemDataFlags flags)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxHairSystem::setReadRequestFlag() not allowed while simulation is running. Call will be ignored.");

		mCore.getShapeCore().getLLCore().mReadRequests = flags;
	}

	PxHairSystemDataFlags NpHairSystem::getReadRequestFlags() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.getShapeCore().getLLCore().mReadRequests;
	}

	void NpHairSystem::setPositionsInvMass(PxVec4* vertexPositionsInvMass, const PxBounds3& bounds)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(vertexPositionsInvMass != NULL, "PxHairSystem::setPositionsInvMass vertexPositionsInvMass must not be NULL.")
		PX_CHECK_AND_RETURN(!((uintptr_t)static_cast<const void *>(vertexPositionsInvMass) & 15), "PxHairSystem::setPositionsInvMass vertexPositionInvMass not aligned to 16 bytes");
		PX_CHECK_AND_RETURN(!bounds.isEmpty(), "PxHairSystem::setPositionsInvMass bounds must not be empty");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();

		PX_CHECK_AND_RETURN(llCore.mNumVertices > 0, "PxHairSystem::setPositionsInvMass numVertices must be greater than zero. Use setTopology().")
		PX_CHECK_AND_RETURN(llCore.mNumStrands > 0, "PxHairSystem::setPositionsInvMass numStrands must be greater than zero. Use setTopology().")
		PX_CHECK_AND_RETURN(llCore.mStrandPastEndIndices != NULL, "PxHairSystem::setPositionsInvMass StrandPastEndIndices are not set. Use setTopology().")

		setLlGridSize(bounds);

		llCore.mPositionInvMass = vertexPositionsInvMass;

		// release internal buffers if they're not needed anymore
		if (vertexPositionsInvMass != mPosInvMassInternal.begin())
			mPosInvMassInternal.reset();

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePOSITIONS_VELOCITIES_MASS |
			Dy::HairSystemDirtyFlag::eGRID_SIZE;
	}

	void NpHairSystem::setVelocities(PxVec4* vertexVelocities)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(vertexVelocities != NULL, "PxHairSystem::setVelocities vertexVelocities must not be NULL.")
		PX_CHECK_AND_RETURN(!((uintptr_t)static_cast<const void *>(vertexVelocities) & 15), "PxHairSystem::setVelocities vertexVelocities not aligned to 16 bytes");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();

		PX_CHECK_AND_RETURN(llCore.mNumVertices > 0, "PxHairSystem::setPositionsInvMass numVertices must be greater than zero. Use setTopology().")

		// release internal buffers if they're not needed anymore
		if (vertexVelocities != mVelInternal.begin())
			mVelInternal.reset();

		llCore.mVelocity = vertexVelocities;
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePOSITIONS_VELOCITIES_MASS;
	}

	void NpHairSystem::setBendingRestAngles(const PxReal* bendingRestAngles, PxReal bendingCompliance)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		PX_CHECK_AND_RETURN(bendingRestAngles || llCore.mRestPositions, "PxHairSystem::setBendingRestAngles() NULL bendingRestAngles only allowed if restPositions have been set.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxHairSystem::setBendingRestAngles() not allowed while simulation is running. Call will be ignored.");

		llCore.mBendingRestAngles = bendingRestAngles;
		llCore.mParams.mBendingCompliance = bendingCompliance;
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eBENDING_REST_ANGLES | Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	void NpHairSystem::setTwistingCompliance(PxReal twistingCompliance)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxHairSystem::setTwistingCompliance() not allowed while simulation is running. Call will be ignored.")

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mParams.mTwistingCompliance = twistingCompliance;
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	void NpHairSystem::getTwistingRestPositions(PxReal* buffer)
	{
		NP_READ_CHECK(getNpScene());

		PxScopedCudaLock lock(*mCudaContextManager);

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		mCudaContextManager->getCudaContext()->memcpyDtoH(buffer,
			reinterpret_cast<CUdeviceptr>(llCore.mTwistingRestPositionsGpuSim),
			sizeof(float) * llCore.mNumVertices);
	}

	void NpHairSystem::setWakeCounter(PxReal wakeCounterValue)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxHairSystem::setWakeCounter() not allowed while simulation is running. Call will be ignored.")

		mCore.setWakeCounter(wakeCounterValue);
		mCore.getShapeCore().getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	PxReal NpHairSystem::getWakeCounter() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.getWakeCounter();
	}

	bool NpHairSystem::isSleeping() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.isSleeping();
	}

	void NpHairSystem::setSolverIterationCounts(PxU32 iters)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);
		PX_CHECK_AND_RETURN(iters > 0, "PxHairSystem::setSolverIterationCounts: positionIters must be more than zero!");
		PX_CHECK_AND_RETURN(iters <= 255, "PxHairSystem::setSolverIterationCounts: positionIters must be no greater than 255!");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxHairSystem::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.")

		mCore.setSolverIterationCounts(static_cast<PxU16>(iters));
	}

	PxU32 NpHairSystem::getSolverIterationCounts() const
	{
		NP_READ_CHECK(getNpScene());

		PxU16 x = mCore.getSolverIterationCounts();
		return x;
	}

	void NpHairSystem::release()
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

	//	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

		if(mRestPositionActor)
		{
			// Careful: If the restPositionActor is already destroyed, we must not call removeRigidAttachment
			PxU32 numActors = npScene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC);
			PxArray<PxActor*> userBuffer(numActors);
			numActors = npScene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, &userBuffer[0], userBuffer.size(), 0);

			for(PxU32 i = 0; i < numActors; i++)
			{
				if(userBuffer[i]->getType() == PxActorType::eRIGID_STATIC || userBuffer[i]->getType() == PxActorType::eRIGID_DYNAMIC)
				{
					const PxRigidActor* rigidActor = static_cast<PxRigidActor*>(userBuffer[i]);
					if(rigidActor == mRestPositionActor)
					{
						removeRigidAttachment(mRestPositionActor);
						break;
					}
				}
			}
			mRestPositionActor = NULL;
		}

		if (npScene)
		{
			npScene->scRemoveHairSystem(*this);
			npScene->removeFromHairSystemList(*this);
		}

		// detachShape();

		PX_ASSERT(!isAPIWriteForbidden());

		mCore.getShapeCore().releaseBuffers();
		releaseAllocator();
		NpDestroyHairSystem(this);
	}
	
	void NpHairSystem::addRigidAttachment(const PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxHairSystem::addRigidAttachment: Hair system must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor != NULL && actor->getScene() != NULL), "PxHairSystem::addRigidAttachment: Actor invalid or not part of a scene.");
		PX_CHECK_AND_RETURN(getScene() == actor->getScene(), "PxHairSystem::addRigidAttachment: Actor and hair must be part of the same scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxHairSystem::addRigidAttachment: Illegal to call while simulation is running.");

		if(actor->getConcreteType() == PxConcreteType::eRIGID_STATIC)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
				"PxHairSystem::addRigidAttachment does not support attachments to static actors. Use attachment to world (NULL) instead.");
			return;
		}

		const Sc::BodyCore* attachmentBodyCore = getBodyCore(actor);

		PX_CHECK_AND_RETURN(attachmentBodyCore != NULL, "PxHairSystem::addRigidAttachment: Attachment body must be rigid dynamic or articulation.");

		if(attachmentBodyCore != NULL)
		{
			const Sc::BodySim* bodySim = attachmentBodyCore->getSim();
			if(bodySim)
				mCore.addAttachment(*bodySim);
		}
	}

	void NpHairSystem::removeRigidAttachment(const PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxHairSystem::removeRigidAttachment: Hair system must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor != NULL && actor->getScene() != NULL), "PxHairSystem::removeRigidAttachment: Actor invalid or not part of a scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxHairSystem::removeRigidAttachment: Illegal to call while simulation is running.");

		const Sc::BodyCore* attachmentBodyCore = getBodyCore(actor);
		if(attachmentBodyCore != NULL)
		{
			const Sc::BodySim* bodySim = attachmentBodyCore->getSim();
			if(bodySim)
				mCore.removeAttachment(*bodySim);
		}
	}

	void NpHairSystem::setRigidAttachments(PxParticleRigidAttachment* attachments, PxU32 numAttachments, bool isGpuPtr)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxHairSystem::setRigidAttachments: Hair system must be inserted into the scene.");
		PX_CHECK_AND_RETURN(attachments != NULL || numAttachments == 0, "PxHairSystem::setRigidAttachments: attachments must not be NULL if numAttachments > 0.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxHairSystem::setRigidAttachments: Illegal to call while simulation is running.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mNumRigidAttachments = numAttachments;

		if(!isGpuPtr)
		{
			// create fresh buffer
			mParticleRigidAttachmentsInternal.reset();
			mParticleRigidAttachmentsInternal.setAllocatorCallback(mDeviceMemoryAllocator);
			mParticleRigidAttachmentsInternal.allocate(numAttachments);
			llCore.mRigidAttachments = mParticleRigidAttachmentsInternal.begin();

			// copy H2D
			PxScopedCudaLock lock(*mCudaContextManager);
			PxCUresult result = mCudaContextManager->getCudaContext()->memcpyHtoD(reinterpret_cast<CUdeviceptr>(llCore.mRigidAttachments), attachments,
				sizeof(PxParticleRigidAttachment) * numAttachments);

			PX_ASSERT(result == 0);
			PX_UNUSED(result);
		}
		else if (mParticleRigidAttachmentsInternal.begin() != attachments)
		{
			mParticleRigidAttachmentsInternal.reset();
			llCore.mRigidAttachments = attachments;
		}
		// Don't clear mParticleRigidAttachmentsInternal if isGpuPtr==true and user has passed in the same pointer again

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eRIGID_ATTACHMENTS;
	}

	PxParticleRigidAttachment* NpHairSystem::getRigidAttachmentsGpu(PxU32* numAttachments)
	{
		NP_READ_CHECK(getNpScene());

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		if(numAttachments)
			*numAttachments = llCore.mNumRigidAttachments;
		return llCore.mRigidAttachments;
	}

	PxU32 NpHairSystem::addSoftbodyAttachment(const PxSoftBody& softbody, const PxU32* tetIds,
		const PxVec4* tetmeshBarycentrics, const PxU32* hairVertices, PxU32 numAttachments)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxHairSystem::addSoftbodyAttachment: Illegal to call while simulation is running.", 0xffFFffFF);
		PX_CHECK_AND_RETURN_VAL(tetIds != NULL, "PxHairSystem::addSoftbodyAttachment: tetIds must not be null.", 0xffFFffFF);
		PX_CHECK_AND_RETURN_VAL(tetmeshBarycentrics != NULL, "PxHairSystem::addSoftbodyAttachment: tetmeshBarycentrics must not be null.", 0xffFFffFF);
		PX_CHECK_AND_RETURN_VAL(hairVertices != NULL, "PxHairSystem::addSoftbodyAttachment: hairVertices must not be null.", 0xffFFffFF);
		PX_CHECK_AND_RETURN_VAL(softbody.getScene() != NULL, "PxHairSystem::addSoftbodyAttachment: Softbody must be inserted into the scene.", 0xffFFffFF);
		PX_CHECK_AND_RETURN_VAL(getScene() == softbody.getScene(), "PxHairSystem::addSoftbodyAttachment: Softbody and hair must be part of the same scene.", 0xffFFffFF);

		if(numAttachments == 0)
			return 0xffFFffFF;

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();

		const NpSoftBody& npSoftbody = static_cast<const NpSoftBody&>(softbody);
		const PxU32 softbodyIdx = npSoftbody.getCore().getGpuSoftBodyIndex();

		const PxTetrahedronMesh& simMesh = *softbody.getSimulationMesh();

		const PxSoftBodyAuxData* auxData = softbody.getSoftBodyAuxData();
		PX_CHECK_AND_RETURN_VAL(auxData->getConcreteType() == PxConcreteType::eSOFT_BODY_STATE, "PxHairSystem::addSoftbodyAttachment: The softbodies aux data must be of type Gu::SoftBodyAuxData.", 0xffFFffFF);
		const Gu::SoftBodyAuxData* guAuxData = static_cast<const Gu::SoftBodyAuxData*>(auxData);

		// Gu::BVTetrahedronMesh does not have a concrete type
		const PxTetrahedronMesh* collisionMesh = softbody.getCollisionMesh();
		// PX_CHECK_AND_RETURN_VAL(collisionMesh->getConcreteType() == ???, "PxHairSystem::addSoftbodyAttachment: The softbodies collision mesh must be of type Gu::BVTetrahedronMesh.", 0xffFFffFF);
		const Gu::BVTetrahedronMesh* bvCollisionMesh = static_cast<const Gu::BVTetrahedronMesh*>(collisionMesh);

		// assign handle by finding the first nonexistent key in the hashmap
		PxU32 handle = PX_MAX_U32;
		for(PxU32 i = 0; i < PX_MAX_U32; i++)
		{
			if(mSoftbodyAttachmentsOffsets.find(i) == NULL)
			{
				handle = i;
				break;
			}
		}
		PX_ASSERT(handle != PX_MAX_U32);

		// new attachments will be added to the end of the contiguous array
		mSoftbodyAttachmentsOffsets[handle] = PxPair<PxU32, PxU32>(mSoftbodyAttachments.size(), numAttachments);
		
		mSoftbodyAttachments.reserve(mSoftbodyAttachments.size() + numAttachments);
		for(PxU32 i=0; i<numAttachments; i++)
		{
			// convert to sim mesh tets/barycentrics
			PxU32 simTetId;
			PxVec4 simBarycentric;
			Gu::convertSoftbodyCollisionToSimMeshTets(simMesh, *guAuxData, *bvCollisionMesh,
				tetIds[i], tetmeshBarycentrics[i], simTetId, simBarycentric);

			Dy::SoftbodyHairAttachment att;
			att.hairVtxIdx= hairVertices[i];
			att.softbodyNodeIdx = softbodyIdx;
			att.tetBarycentric = simBarycentric;
			att.tetId = simTetId;

			mSoftbodyAttachments.pushBack(att);
		}

		mCore.addAttachment(*npSoftbody.getCore().getSim()); // add edge for island generation

		llCore.mSoftbodyAttachments = mSoftbodyAttachments.begin();
		llCore.mNumSoftbodyAttachments = mSoftbodyAttachments.size();

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eSOFTBODY_ATTACHMENTS;
		return handle;
	}

	void NpHairSystem::removeSoftbodyAttachment(const PxSoftBody& softbody, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxHairSystem::removeSoftbodyAttachment: Illegal to call while simulation is running.");
		PX_CHECK_AND_RETURN(softbody.getScene() != NULL, "PxHairSystem::removeSoftbodyAttachment: Softbody must still be inserted into the scene.");

		const PxPair<const PxU32, PxPair<PxU32, PxU32>>* handleOffsetSize = mSoftbodyAttachmentsOffsets.find(handle);
		PX_ASSERT(handleOffsetSize != NULL);
		if(handleOffsetSize == NULL)
			return;

		PX_ASSERT(handle == handleOffsetSize->first);
		const PxU32 offset = handleOffsetSize->second.first;
		const PxU32 numRemoved = handleOffsetSize->second.second;
		const PxU32 totSize = mSoftbodyAttachments.size();

		// shift all subsequent elements in the attachment array to the left
		for(PxU32 i=offset; i + numRemoved < totSize; i++)
		{
			mSoftbodyAttachments[i] = mSoftbodyAttachments[i+numRemoved];
		}
		mSoftbodyAttachments.resize(mSoftbodyAttachments.size() - numRemoved);

		// correct all offsets of the still existing attachments and delete the current handle
		mSoftbodyAttachmentsOffsets.erase(handle);
		
		for(PxHashMap<PxU32, PxPair<PxU32, PxU32>>::Iterator it = mSoftbodyAttachmentsOffsets.getIterator();
				!it.done(); it++)
		{
			if(it->second.first > offset)
				it->second.first -= numRemoved;
		}

		const NpSoftBody& npSoftbody = static_cast<const NpSoftBody&>(softbody);
		mCore.removeAttachment(*npSoftbody.getCore().getSim()); // remove edge for island generation

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mSoftbodyAttachments = mSoftbodyAttachments.begin();
		llCore.mNumSoftbodyAttachments = mSoftbodyAttachments.size();

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eSOFTBODY_ATTACHMENTS;
	}


	void NpHairSystem::setRestPositions(PxVec4* restPos, bool isGpuPtr, const PxTransform& transf, const PxRigidBody* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "PxHairSystem::setRestPositions: Actor invalid or not part of a scene.");
		PX_CHECK_AND_RETURN((actor == NULL || getScene() == actor->getScene()), "PxHairSystem::setRestPositions: Actor and hair must be part of the same scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxHairSystem::setRestPositions: Illegal to call while simulation is running.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();

		if(restPos)
		{
			if(!isGpuPtr)
			{
				// create fresh buffer
				mRestPositionsInternal.reset();
				mRestPositionsInternal.setAllocatorCallback(mDeviceMemoryAllocator);
				mRestPositionsInternal.allocate(llCore.mNumVertices);
				llCore.mRestPositions = mRestPositionsInternal.begin();

				// copy H2D
				PxScopedCudaLock lock(*mCudaContextManager);
				mCudaContextManager->getCudaContext()->memcpyHtoD(reinterpret_cast<CUdeviceptr>(llCore.mRestPositions), restPos,
					sizeof(PxVec4) * llCore.mNumVertices);
			}
			else if (mRestPositionsInternal.begin() != restPos)
			{
				mRestPositionsInternal.reset();
				llCore.mRestPositions = restPos;
			}
			// Don't clear mRestPositionsInternal if isGpuPtr==true and user has passed in the same pointer again

			llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eREST_POSITIONS;
		}

		if(llCore.mRestPositionsTransform == NULL)
		{
			mRestPositionTransformPinnedBuf.setAllocatorCallback(mHostMemoryAllocator);
			mRestPositionTransformPinnedBuf.allocate(1);
			llCore.mRestPositionsTransform = mRestPositionTransformPinnedBuf.begin();
		}
		*llCore.mRestPositionsTransform = transf;

		// update actor relative to which the positions are specified
		if(mRestPositionActor && actor != mRestPositionActor)
		{
			removeRigidAttachment(mRestPositionActor);
			mRestPositionActor = NULL;
			llCore.mRestPositionBodyNodeIdx = PxNodeIndex().getInd();
		}
		if(actor && actor != mRestPositionActor)
		{
			addRigidAttachment(actor); // ensure actor is in the same island
			mRestPositionActor = actor;
			llCore.mRestPositionBodyNodeIdx = actor->getInternalIslandNodeIndex().getInd();
		}

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eREST_POSITION_TRANSFORM;
	}

	PxVec4* NpHairSystem::getRestPositionsGpu(PxTransform* transf)
	{
		NP_READ_CHECK(getNpScene());

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		if(transf)
			*transf = *llCore.mRestPositionsTransform;
		return llCore.mRestPositions;
	}

	void NpHairSystem::setLlGridSize(const PxBounds3& bounds)
	{
		// give system some space to expand: create a grid to accomodate at least 1.5 times the initial size
		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		const PxVec3 dimensions = bounds.getDimensions();
		const PxReal maxXZ = PxMax(dimensions.x, dimensions.z); // rotations in plane perpendicular to gravity are very likely
		const PxReal cellSize = llCore.mParams.getCellSize();

		const PxU32 tightGridSizeXZ = static_cast<PxU32>(maxXZ / cellSize) + 1;
		const PxU32 tightGridSizeY = static_cast<PxU32>(dimensions.y / cellSize) + 1;

		PxU32 gridSizeXZ = PxNextPowerOfTwo(tightGridSizeXZ);
		PxU32 gridSizeY = PxNextPowerOfTwo(tightGridSizeY);

		if(gridSizeXZ < 1.5f * tightGridSizeXZ)
			gridSizeXZ *= 2;
		if(gridSizeY < 1.5f * tightGridSizeY)
			gridSizeY *= 2;

		llCore.mParams.mGridSize[0] = gridSizeXZ;
		llCore.mParams.mGridSize[1] = gridSizeY;
		llCore.mParams.mGridSize[2] = llCore.mParams.mGridSize[0];

		if (static_cast<PxU32>(dimensions.maxElement() / cellSize) > 512)
			PxGetFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
				"Grid of hair system appears very large (%i by %i by %i). Double check ratio of segment"
				" length (%f) to extent of the hair system defined by the vertices (%f, %f, %f).",
				llCore.mParams.mGridSize[0], llCore.mParams.mGridSize[1], llCore.mParams.mGridSize[2],
				static_cast<double>(llCore.mParams.mSegmentLength), static_cast<double>(dimensions.x),
				static_cast<double>(dimensions.y), static_cast<double>(dimensions.z));
	}

	void NpHairSystem::createAllocator()
	{
		if (!mMemoryManager)
		{
			PxPhysXGpu* physXGpu = PxvGetPhysXGpu(true);
			PX_ASSERT(physXGpu != NULL);

			mMemoryManager = physXGpu->createGpuMemoryManager(mCudaContextManager);
			mHostMemoryAllocator = mMemoryManager->getHostMemoryAllocator();
			mDeviceMemoryAllocator = mMemoryManager->getDeviceMemoryAllocator();
		}
		PX_ASSERT(mMemoryManager != NULL);
		PX_ASSERT(mHostMemoryAllocator != NULL);
		PX_ASSERT(mDeviceMemoryAllocator != NULL);
	}

	void NpHairSystem::initFromDesc(const PxHairSystemDesc& desc)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(desc.isValid(), "PxHairSystem::initFromDesc desc is not valid.");
		PX_ASSERT(mCudaContextManager != NULL);

		// destroy internal buffers if they exist because we might switch allocators
		if(mStrandPastEndIndicesInternal.size() > 0)
			mStrandPastEndIndicesInternal.reset();
		if (mPosInvMassInternal.size() > 0)
			mPosInvMassInternal.reset();
		if (mVelInternal.size() > 0)
			mVelInternal.reset();

		// create internal buffers
		if (desc.flags.isSet(PxHairSystemDescFlag::eDEVICE_MEMORY))
		{
			mPosInvMassInternal.setAllocatorCallback(mDeviceMemoryAllocator);
			mVelInternal.setAllocatorCallback(mDeviceMemoryAllocator);
			mStrandPastEndIndicesInternal.setAllocatorCallback(mDeviceMemoryAllocator);
		}
		else
		{
			mPosInvMassInternal.setAllocatorCallback(mHostMemoryAllocator);
			mVelInternal.setAllocatorCallback(mHostMemoryAllocator);
			mStrandPastEndIndicesInternal.setAllocatorCallback(mHostMemoryAllocator);
		}

		PxScopedCudaLock lock(*mCudaContextManager);

		// StrandPastEndIndices
		MemoryWithAlloc<PxU32> strandPastEndIndicesLocal;
		strandPastEndIndicesLocal.setAllocatorCallback(mHostMemoryAllocator);
		strandPastEndIndicesLocal.allocate(desc.numStrands);
		const PxU32* hostStrandPastEndIndices = strandPastEndIndicesLocal.begin();
		PxU32 numVertices = 0;
		for (PxU32 strandIdx = 0; strandIdx < desc.numStrands; strandIdx++)
		{
			numVertices += desc.numVerticesPerStrand.at<PxU32>(strandIdx);
			strandPastEndIndicesLocal[strandIdx] = numVertices;
		}
		if (desc.flags.isSet(PxHairSystemDescFlag::eDEVICE_MEMORY))
		{
			mStrandPastEndIndicesInternal.allocate(desc.numStrands);
			mCudaContextManager->getCudaContext()->memcpyHtoD(
				reinterpret_cast<CUdeviceptr>(mStrandPastEndIndicesInternal.begin()),
				strandPastEndIndicesLocal.begin(), desc.numStrands * sizeof(PxU32));
		}
		else
		{
			strandPastEndIndicesLocal.swap(mStrandPastEndIndicesInternal);
		}

		// positions, velocities
		const bool onlyRootPositionsGiven = desc.numStrands == desc.vertices.count;
		const bool velocitiesGiven = desc.velocities.count > 0;

		MemoryWithAlloc<PxVec4> posInvMassLocal, velLocal;
		posInvMassLocal.setAllocatorCallback(mHostMemoryAllocator);
		posInvMassLocal.allocate(numVertices);
		velLocal.setAllocatorCallback(mHostMemoryAllocator);
		velLocal.allocate(numVertices);

		PxBounds3 bounds = PxBounds3::empty();
		PxU32 overallVertexIdx = 0;
		for (PxU32 strandIdx = 0; strandIdx < desc.numStrands; strandIdx++)
		{
			for (; overallVertexIdx < hostStrandPastEndIndices[strandIdx]; overallVertexIdx++)
			{
				const PxU32 inputVertexIdx = onlyRootPositionsGiven ? strandIdx : overallVertexIdx;
				posInvMassLocal[overallVertexIdx] = desc.vertices.at<PxVec4>(inputVertexIdx);
				velLocal[overallVertexIdx] = velocitiesGiven ? desc.velocities.at<PxVec4>(overallVertexIdx) : PxVec4(PxZero);
				bounds.include(desc.vertices.at<PxVec4>(inputVertexIdx).getXYZ());
			}
		}
		PX_ASSERT(overallVertexIdx == numVertices);
		if (desc.flags.isSet(PxHairSystemDescFlag::eDEVICE_MEMORY))
		{
			mPosInvMassInternal.allocate(numVertices);
			mVelInternal.allocate(numVertices);

			mCudaContextManager->getCudaContext()->memcpyHtoD(
				reinterpret_cast<CUdeviceptr>(mPosInvMassInternal.begin()),
				posInvMassLocal.begin(), numVertices * sizeof(PxVec4));
			mCudaContextManager->getCudaContext()->memcpyHtoD(
				reinterpret_cast<CUdeviceptr>(mVelInternal.begin()),
				velLocal.begin(), numVertices * sizeof(PxVec4));
		}
		else
		{
			posInvMassLocal.swap(mPosInvMassInternal);
			velLocal.swap(mVelInternal);
		}

		setTopology(mPosInvMassInternal.begin(), mVelInternal.begin(), mStrandPastEndIndicesInternal.begin(),
			desc.segmentLength, desc.segmentRadius, numVertices, desc.numStrands, bounds);
	}

	void NpHairSystem::setTopology(PxVec4* vertexPositionsInvMass, PxVec4* vertexVelocities, const PxU32* strandPastEndIndices,
		PxReal segmentLength, PxReal segmentRadius, PxU32 numVertices, PxU32 numStrands, const PxBounds3& bounds)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(numVertices > 0, "PxHairSystem::setTopology numVertices must be greater than zero");
		PX_CHECK_AND_RETURN(numStrands > 0, "PxHairSystem::setTopology numStrands must be greater than zero");
		PX_CHECK_AND_RETURN(segmentLength > 0.0f, "PxHairSystem::setTopology segmentLength must be greater than zero");
		PX_CHECK_AND_RETURN(segmentRadius > 0.0f, "PxHairSystem::setTopology segmentRadius must be greater than zero");
		PX_CHECK_AND_RETURN(2.0f * segmentRadius < segmentLength, "PxHairSystem::setTopology segmentRadius must be smaller than half of segmentLength. Call ignored");
		PX_CHECK_AND_RETURN(!bounds.isEmpty(), "PxHairSystem::setTopology bounds must not be empty");
		PX_CHECK_AND_RETURN((vertexPositionsInvMass != NULL && vertexVelocities != NULL), "PxHairSystem::setTopology positions and velocities must not be NULL")
		PX_CHECK_AND_RETURN(!((uintptr_t)static_cast<const void *>(vertexPositionsInvMass) & 15), "PxHairSystem::setTopology vertexPositionInvMass not aligned to 16 bytes");
		PX_CHECK_AND_RETURN(!((uintptr_t)static_cast<const void *>(vertexVelocities) & 15), "PxHairSystem::setTopology vertexVelocities not aligned to 16 bytes");
		PX_CHECK_AND_RETURN(numVertices < (1 << 24), "PxHairSystem::setTopology numVertices must be smaller than 1<<24"); // due to encoding compressedVtxIndex with hairsystem index

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();

		// release internal buffers if they're not needed anymore
		if (vertexPositionsInvMass != mPosInvMassInternal.begin())
			mPosInvMassInternal.reset();
		if (vertexVelocities != mVelInternal.begin())
			mVelInternal.reset();
		if (strandPastEndIndices != mStrandPastEndIndicesInternal.begin())
			mStrandPastEndIndicesInternal.reset();

		llCore.mParams.mSegmentLength = segmentLength;
		setSegmentRadius(segmentRadius);
		llCore.mParams.mSegmentRadius = segmentRadius;
		llCore.mNumVertices = numVertices;
		llCore.mNumStrands = numStrands;
		llCore.mStrandPastEndIndices = strandPastEndIndices;
		llCore.mPositionInvMass = vertexPositionsInvMass;
		llCore.mVelocity = vertexVelocities;

		// reset rest positions
		llCore.mRestPositions = NULL;
		setRestPositions(NULL, false, PxTransform(PxIdentity), NULL);

		setLlGridSize(bounds);

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eNUM_STRANDS_OR_VERTS |
			Dy::HairSystemDirtyFlag::ePOSITIONS_VELOCITIES_MASS | Dy::HairSystemDirtyFlag::ePARAMETERS |
			Dy::HairSystemDirtyFlag::eSTRAND_LENGTHS | Dy::HairSystemDirtyFlag::eGRID_SIZE;
	}

	void NpHairSystem::setLevelOfDetailGradations(const PxReal* proportionOfStrands, const PxReal* proportionOfVertices,
		PxU32 numLevels)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(numLevels > 0, "PxHairSystem::defineLevelOfDetailGradations Must specify at least one level.");
		for (PxU32 i = 0; i < numLevels; i++)
		{
			PX_CHECK_AND_RETURN(proportionOfStrands[i] <= 1.0f && proportionOfStrands[i] > 0.0f, "PxHairSystem::defineLevelOfDetailGradations proportionOfStrands must be in (0, 1] ");
			PX_CHECK_AND_RETURN(proportionOfVertices[i] <= 1.0f && proportionOfVertices[i] > 0.0f, "PxHairSystem::defineLevelOfDetailGradations proportionOfStrands must be in (0, 1] ");
		}

		// deep copy because user buffers may go out of scope / deallocate
		mLodProportionOfStrands.assign(proportionOfStrands, proportionOfStrands + numLevels);
		mLodProportionOfVertices.assign(proportionOfVertices, proportionOfVertices + numLevels);

		// TODO(jcarius) check that hair system is initialized already
		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mLodNumLevels = numLevels;
		llCore.mLodProportionOfStrands = mLodProportionOfStrands.begin();
		llCore.mLodProportionOfVertices = mLodProportionOfVertices.begin();

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eLOD_DATA;

		// in case the level that we were on got removed switch to closest one
		if(llCore.mLodLevel > numLevels)
		{
			llCore.mLodLevel = numLevels;
			llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eLOD_SWITCH;
		}
	}

	void NpHairSystem::setLevelOfDetail(PxU32 level)
	{
		NP_WRITE_CHECK(getNpScene());
		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		PX_CHECK_AND_RETURN(level == 0 || llCore.mLodNumLevels >= level, "PxHairSystem::setLevelOfDetail Invalid level given");

		llCore.mLodLevel = level;

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eLOD_SWITCH;
	}

	PxU32 NpHairSystem::getLevelOfDetail(PxU32* numLevels) const
	{
		NP_READ_CHECK(getNpScene());
		if(numLevels != NULL)
		{
			*numLevels = mCore.getShapeCore().getLLCore().mLodNumLevels;
		}
		return mCore.getShapeCore().getLLCore().mLodLevel;
	}

	void NpHairSystem::setName(const char* debugName)
	{
		NP_WRITE_CHECK(getNpScene());
		mName = debugName;
	}

	const char* NpHairSystem::getName() const
	{
		NP_READ_CHECK(getNpScene());
		return mName;
	}

	void NpHairSystem::getSegmentDimensions(PxReal& length, PxReal& radius) const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		length = llCore.mParams.mSegmentLength;
		radius = llCore.mParams.mSegmentRadius;
	}

	void NpHairSystem::setSegmentRadius(PxReal radius)
	{
		NP_WRITE_CHECK(getNpScene());
		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		PX_CHECK_AND_RETURN(2.0f * radius < llCore.mParams.mSegmentLength, "PxHairSystem::setSegmentRadius radius must be smaller than half of segment length. Call ignored");

		llCore.mParams.mSegmentRadius = radius;
		mCore.setContactOffset(2.0f * radius); // sensible default
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	void NpHairSystem::setWind(const PxVec3& wind)
	{
		NP_WRITE_CHECK(getNpScene());
		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();

		PxVec3 windNormalized = wind;
		const PxReal magnitude = windNormalized.normalize();
		llCore.mWind = PxVec4(windNormalized, magnitude);
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	PxVec3 NpHairSystem::getWind() const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		return llCore.mWind.getXYZ() * llCore.mWind.w;
	}

	void NpHairSystem::setFrictionParameters(PxReal interHairVelDamping, PxReal frictionCoeff)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(interHairVelDamping >= 0.0f, "PxHairSystem::setFrictionParameters interHairVelDamping must be greater or equal zero.");
		PX_CHECK_AND_RETURN(interHairVelDamping <= 1.0f, "PxHairSystem::setFrictionParameters interHairVelDamping must be smaller or equal one.");
		PX_CHECK_AND_RETURN(frictionCoeff >= 0.0f, "PxHairSystem::setFrictionParameters frictionCoeff must be greater or equal zero.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();

		llCore.mParams.mInterHairVelocityDamping = interHairVelDamping;
		llCore.mParams.mFrictionCoeff = frictionCoeff;

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	void NpHairSystem::getFrictionParameters(PxReal& interHairVelDamping, PxReal& frictionCoeff) const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		interHairVelDamping = llCore.mParams.mInterHairVelocityDamping;
		frictionCoeff = llCore.mParams.mFrictionCoeff;
	}

	void NpHairSystem::setShapeCompliance(PxReal startCompliance, PxReal strandRatio)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(strandRatio >= 0.0f, "PxHairSystem::setShapeCompliance strandRatio must not be negative.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		PX_CHECK_AND_RETURN(llCore.mRestPositions, "PxHairSystem::setShapeCompliance restPositions must be set before enabling shape compliance");

		llCore.mParams.mShapeCompliance[0] = startCompliance;
		llCore.mParams.mShapeCompliance[1] = strandRatio;

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	void NpHairSystem::getShapeCompliance(PxReal& startCompliance, PxReal& strandRatio) const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		startCompliance = llCore.mParams.mShapeCompliance[0];
		strandRatio = llCore.mParams.mShapeCompliance[1];
	}

	void NpHairSystem::setInterHairRepulsion(PxReal repulsion)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(repulsion >= 0.0f, "PxHairSystem::setInterHairRepulsion repulsion must not be negative.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mParams.mInterHairRepulsion = repulsion;
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	PxReal NpHairSystem::getInterHairRepulsion() const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		return llCore.mParams.mInterHairRepulsion;
	}

	void NpHairSystem::setSelfCollisionRelaxation(PxReal relaxation)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(relaxation > 0.0f, "PxHairSystem::setSelfCollisionRelaxation relaxation must be greater zero.");
		PX_CHECK_AND_RETURN(relaxation <= 1.0f, "PxHairSystem::setSelfCollisionRelaxation relaxation must not be greater 1.0.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mParams.mSelfCollisionRelaxation = relaxation;
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	PxReal NpHairSystem::getSelfCollisionRelaxation() const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		return llCore.mParams.mSelfCollisionRelaxation;
	}

	void NpHairSystem::setStretchingRelaxation(PxReal relaxation)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(relaxation > 0.0f, "PxHairSystem::setStretchingRelaxation relaxation must be greater zero.");
		PX_CHECK_AND_RETURN(relaxation <= 1.0f, "PxHairSystem::setStretchingRelaxation relaxation must not be greater 1.0.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mParams.mLraRelaxation = relaxation;
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	PxReal NpHairSystem::getStretchingRelaxation() const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		return llCore.mParams.mLraRelaxation;
	}

	void NpHairSystem::setContactOffset(PxReal contactOffset)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(contactOffset >= 0.0f, "PxHairSystem::setContactOffset contactOffset must not be negative.");

		mCore.setContactOffset(contactOffset);

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	PxReal NpHairSystem::getContactOffset() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.getContactOffset();
	}

	void NpHairSystem::setHairContactOffset(PxReal hairContactOffset)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(hairContactOffset >= 1.0f, "PxHairSystem::setHairContactOffset hairContactOffset must not be below 1.0.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		llCore.mParams.mSelfCollisionContactDist = hairContactOffset;
		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
	}

	PxReal NpHairSystem::getHairContactOffset() const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		return llCore.mParams.mSelfCollisionContactDist;
	}

	void NpHairSystem::setShapeMatchingParameters(PxReal compliance, PxReal linearStretching,
		PxU16 numVerticesPerGroup, PxU16 numVerticesOverlap)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(linearStretching >= 0.0f, "PxHairSystem::setShapeMatchingParameters linearStretching must not be below 0.0.");
		PX_CHECK_AND_RETURN(linearStretching <= 1.0f, "PxHairSystem::setShapeMatchingParameters linearStretching must not be above 1.0.");
		PX_CHECK_AND_RETURN(numVerticesPerGroup > 1, "PxHairSystem::setShapeMatchingParameters numVerticesPerGroup must be greater 1.");
		PX_CHECK_AND_RETURN(numVerticesPerGroup >= 2 * numVerticesOverlap, "PxHairSystem::setShapeMatchingParameters numVerticesOverlap must at most be numVerticesPerGroup/2.");

		Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		PX_CHECK_AND_RETURN(llCore.mRestPositions, "PxHairSystem::setShapeMatchingParameters restPositions must be set before enabling shape compliance");

		llCore.mParams.mShapeMatchingCompliance = compliance;
		llCore.mParams.mShapeMatchingBeta = linearStretching;
		llCore.mParams.mShapeMatchingNumVertsPerGroup = numVerticesPerGroup;
		llCore.mParams.mShapeMatchingNumVertsOverlap = numVerticesOverlap;

		llCore.mDirtyFlags |= Dy::HairSystemDirtyFlag::eSHAPE_MATCHING_SIZES;
	}

	PxReal NpHairSystem::getShapeMatchingCompliance() const
	{
		NP_READ_CHECK(getNpScene());
		const Dy::HairSystemCore& llCore = mCore.getShapeCore().getLLCore();
		return llCore.mParams.mShapeMatchingCompliance;
	}
#endif
} // namespace physx

#endif //PX_SUPPORT_GPU_PHYSX

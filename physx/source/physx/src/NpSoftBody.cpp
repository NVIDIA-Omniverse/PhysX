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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpSoftBody.h"
#include "NpParticleSystem.h"
#include "NpCheck.h"
#include "NpScene.h"
#include "NpShape.h"
#include "geometry/PxTetrahedronMesh.h"
#include "geometry/PxTetrahedronMeshGeometry.h"
#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "GuTetrahedronMesh.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpArticulationLink.h"
#include "ScSoftBodySim.h"
#include "NpFEMSoftBodyMaterial.h"

using namespace physx;

namespace physx
{
	NpSoftBody::NpSoftBody(PxCudaContextManager& cudaContextManager) :
		NpActorTemplate	(PxConcreteType::eSOFT_BODY, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::eSOFTBODY),
		mShape			(NULL),
		mCudaContextManager(&cudaContextManager)
	{
	}

	NpSoftBody::NpSoftBody(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager) :
		NpActorTemplate	(baseFlags), 
		mShape			(NULL),
		mCudaContextManager(&cudaContextManager)
	{
	}

	PxBounds3 NpSoftBody::getWorldBounds(float inflation) const
	{
		NP_READ_CHECK(getNpScene());

		if (!getNpScene())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Querying bounds of a PxSoftBody which is not part of a PxScene is not supported.");
			return PxBounds3::empty();
		}

		const Sc::SoftBodySim* sim = mCore.getSim();
		PX_ASSERT(sim);

		PX_SIMD_GUARD;

		PxBounds3 bounds = sim->getBounds();
		PX_ASSERT(bounds.isValid());

		// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
		const PxVec3 center = bounds.getCenter();
		const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
		return PxBounds3::centerExtents(center, inflatedExtents);
	}

	PxU32 NpSoftBody::getGpuSoftBodyIndex()
	{
		NP_READ_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(getNpScene(), "PxSoftBody::getGpuSoftBodyIndex: Soft body must be in a scene.", 0xffffffff);

		return mCore.getGpuSoftBodyIndex();
	}

	void NpSoftBody::setSoftBodyFlag(PxSoftBodyFlag::Enum flag, bool val)
	{
		PxSoftBodyFlags flags = mCore.getFlags();
		if (val)
			flags.raise(flag);
		else
			flags.clear(flag);

		mCore.setFlags(flags);
	}

	void NpSoftBody::setSoftBodyFlags(PxSoftBodyFlags flags)
	{
		mCore.setFlags(flags);
	}

	PxSoftBodyFlags NpSoftBody::getSoftBodyFlag() const
	{
		return mCore.getFlags();
	}

	void NpSoftBody::setParameter(const PxFEMParameters paramters)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxSoftBody::setInternalIterationCount() not allowed while simulation is running. Call will be ignored.")

		mCore.setParameter(paramters);
		UPDATE_PVD_PROPERTY
	}

	PxFEMParameters NpSoftBody::getParameter() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.getParameter();
	}

	PxBuffer* NpSoftBody::getBufferFromFlag(PxSoftBodyData::Enum flags)
	{
		PxBuffer* buf = NULL;
		Dy::SoftBodyCore& core = mCore.getCore();
		PX_UNUSED(core);
		switch (flags)
		{
		case PxSoftBodyData::ePOSITION_INVMASS:
			buf = core.mPositionInvMass;
			break;
		case PxSoftBodyData::eSIM_POSITION_INVMASS:
			buf = core.mSimPositionInvMass;
			break;
		case PxSoftBodyData::eSIM_VELOCITY:
			buf = core.mSimVelocityInvMass;
			break;
		case PxSoftBodyData::eSIM_KINEMATIC_TARGET:
			buf = core.mKinematicTarget;
			break;
		case PxSoftBodyData::eNONE:
		case PxSoftBodyData::eALL:
		default:
			PX_ASSERT(0);
		}
		return buf;
	}

	PxBuffer* NpSoftBody::getBufferHostFromFlag(PxSoftBodyData::Enum flags)
	{
		PxBuffer* buf = NULL;
		Dy::SoftBodyCore& core = mCore.getCore();
		PX_UNUSED(core);
		switch (flags)
		{
		case PxSoftBodyData::ePOSITION_INVMASS:
			buf = core.mPositionInvMassCPU;
			break;
		case PxSoftBodyData::eSIM_POSITION_INVMASS:
			buf = core.mSimPositionInvMassCPU;
			break;
		case PxSoftBodyData::eSIM_VELOCITY:
			buf = core.mSimVelocityInvMassCPU;
			break;
		case PxSoftBodyData::eSIM_KINEMATIC_TARGET:
			buf = core.mKinematicTargetCPU;
			break;
		case PxSoftBodyData::eNONE:
		case PxSoftBodyData::eALL:
		default:
			PX_ASSERT(0);
		}
		return buf;
	}

	void NpSoftBody::readData(PxSoftBodyData::Enum flags, PxBuffer& buffer, bool flush)
	{
		PxBuffer* sourceBuffer = getBufferFromFlag(flags);
		if (!sourceBuffer)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxSoftBody::readData, source buffer hasn't been allocated.");
			return;
		}

		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
		PX_ASSERT(physxGpu);

		physxGpu->addCopyCommand(buffer, *sourceBuffer, flush);
	}

	void NpSoftBody::readData(PxSoftBodyData::Enum flags, bool flush)
	{
		readData(flags, *getBufferHostFromFlag(flags), flush);
	}

	void NpSoftBody::writeData(PxSoftBodyData::Enum flags, PxBuffer& buffer, bool flush)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxSoftBody::writeData() not allowed while simulation is running. Call will be ignored.")

		if (flags == PxSoftBodyData::ePOSITION_INVMASS)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "NpSoftBody::writeData, ePOSITION_INVMASS is immutable.");
			return;
		}
		PxBuffer* targetBuffer = getBufferFromFlag(flags);
		if (!targetBuffer)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "NpSoftBody::writeData, target buffer hasn't been allocated.");
			return;
		}

		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
		PX_ASSERT(physxGpu);

		physxGpu->addCopyCommand(*targetBuffer, buffer, flush);
	}

	void NpSoftBody::writeData(PxSoftBodyData::Enum flags, bool flush)
	{
		writeData(flags, *getBufferHostFromFlag(flags), flush);
	}

	PxCudaContextManager* NpSoftBody::getCudaContextManager() const
	{
		return mCudaContextManager;
	}

	void NpSoftBody::setWakeCounter(PxReal wakeCounterValue)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxSoftBody::setWakeCounter() not allowed while simulation is running. Call will be ignored.")

		mCore.setWakeCounter(wakeCounterValue);
		//UPDATE_PVD_PROPERTIES_OBJECT()
	}

	PxReal NpSoftBody::getWakeCounter() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.getWakeCounter();
	}

	bool NpSoftBody::isSleeping() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.isSleeping();
	}

	void NpSoftBody::setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);
		PX_CHECK_AND_RETURN(positionIters > 0, "NpSoftBody::setSolverIterationCounts: positionIters must be more than zero!");
		PX_CHECK_AND_RETURN(positionIters <= 255, "NpSoftBody::setSolverIterationCounts: positionIters must be no greater than 255!");
		PX_CHECK_AND_RETURN(velocityIters <= 255, "NpSoftBody::setSolverIterationCounts: velocityIters must be no greater than 255!");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxSoftBody::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.")

		mCore.setSolverIterationCounts((velocityIters & 0xff) << 8 | (positionIters & 0xff));
	}

	void NpSoftBody::getSolverIterationCounts(PxU32& positionIters, PxU32& velocityIters) const
	{
		NP_READ_CHECK(getNpScene());

		PxU16 x = mCore.getSolverIterationCounts();
		velocityIters = PxU32(x >> 8);
		positionIters = PxU32(x & 0xff);
	}

	PxShape* NpSoftBody::getShape()
	{ 
		return mShape; 
	}

	PxTetrahedronMesh* NpSoftBody::getCollisionMesh()
	{
		PxTetrahedronMeshGeometry tetMeshGeom;
		mShape->getTetrahedronMeshGeometry(tetMeshGeom);
		return tetMeshGeom.tetrahedronMesh;
	}

	PxTetrahedronMesh* NpSoftBody::getSimulationMesh()
	{
		return mSimulationMesh;
	}

	PxSoftBodyAuxData* NpSoftBody::getSoftBodyAuxData()
	{
		return mSoftBodyAuxData;
	}

	bool NpSoftBody::attachSimulationMesh(PxTetrahedronMesh& simulationMesh, PxSoftBodyAuxData& softBodyAuxData)
	{
		Dy::SoftBodyCore& core = mCore.getCore();

		PX_CHECK_AND_RETURN_NULL(core.mSimPositionInvMassCPU == NULL, "NpSoftBody::attachSimulationMesh: mSimPositionInvMassCPU already exists, overwrite not allowed, call detachSimulationMesh first");
		PX_CHECK_AND_RETURN_NULL(core.mSimVelocityInvMassCPU == NULL, "NpSoftBody::attachSimulationMesh: mSimVelocityInvMassCPU already exists, overwrite not allowed, call detachSimulationMesh first");

		PX_CHECK_AND_RETURN_NULL(core.mSimPositionInvMass == NULL, "NpSoftBody::attachSimulationMesh: mSimPositionInvMass already exists, overwrite not allowed, call detachSimulationMesh first");
		PX_CHECK_AND_RETURN_NULL(core.mSimVelocityInvMass == NULL, "NpSoftBody::attachSimulationMesh: mSimVelocityInvMass already exists, overwrite not allowed, call detachSimulationMesh first");

		mSimulationMesh = static_cast<Gu::TetrahedronMesh*>(&simulationMesh);
		mSoftBodyAuxData = static_cast<Gu::SoftBodyAuxData*>(&softBodyAuxData);

		//const PxTetrahedronMeshGeometry& tetGeometry = mSimShape->getGeometry().tetMesh();
		Gu::TetrahedronMesh* tetMesh = static_cast<Gu::TetrahedronMesh*>(&simulationMesh);
		
		const PxU32 numVertsGM = tetMesh->getNbVerticesFast();

		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);

		core.mSimPositionInvMassCPU = physxGpu->createBuffer(numVertsGM * sizeof(PxVec4), PxBufferType::eHOST, mCudaContextManager, &mCore.getGpuMemStat());
		core.mSimVelocityInvMassCPU = physxGpu->createBuffer(numVertsGM * sizeof(PxVec4), PxBufferType::eHOST, mCudaContextManager, &mCore.getGpuMemStat());
		core.mKinematicTargetCPU = physxGpu->createBuffer(numVertsGM * sizeof(PxVec4), PxBufferType::eHOST, mCudaContextManager, &mCore.getGpuMemStat());

		core.mSimPositionInvMass = physxGpu->createBuffer(numVertsGM * sizeof(PxVec4), PxBufferType::eDEVICE, mCudaContextManager, &mCore.getGpuMemStat());
		core.mSimVelocityInvMass = physxGpu->createBuffer(numVertsGM * sizeof(PxVec4), PxBufferType::eDEVICE, mCudaContextManager, &mCore.getGpuMemStat());
		core.mKinematicTarget = physxGpu->createBuffer(numVertsGM * sizeof(PxVec4), PxBufferType::eDEVICE, mCudaContextManager, &mCore.getGpuMemStat());
		
		const Gu::TetrahedronMesh* meshData = static_cast<const Gu::TetrahedronMesh*>(tetMesh);
		PxVec3* positions = meshData->getVerticesFast();
		PxVec4* positionInv = reinterpret_cast<PxVec4*>(core.mSimPositionInvMassCPU->map());
		PxVec4* velocityInv = reinterpret_cast<PxVec4*>(core.mSimVelocityInvMassCPU->map());
		PxVec4* kinematicTarget = reinterpret_cast<PxVec4*>(core.mKinematicTargetCPU->map());
		const Gu::SoftBodyAuxData* s = static_cast<const Gu::SoftBodyAuxData*>(&softBodyAuxData);
		const float* invMassGM = s->mGridModelInvMass;
		for (PxU32 i = 0; i < numVertsGM; ++i)
		{
			const PxVec3 vert = positions[i];
			PxReal invMass = invMassGM ? invMassGM[i] : 1.0f;
			positionInv[i] = PxVec4(vert.x, vert.y, vert.z, invMass);
			velocityInv[i] = PxVec4(0.f, 0.f, 0.f, invMass);
			kinematicTarget[i] = PxVec4(vert.x, vert.y, vert.z, 0.f);
		}
		core.mSimPositionInvMassCPU->unmap();
		core.mSimVelocityInvMassCPU->unmap();
		core.mKinematicTargetCPU->unmap();

		return true;
	}

	void NpSoftBody::updateMaterials()
	{
		Dy::SoftBodyCore& core = mCore.getCore();
		core.clearMaterials();
		for (PxU32 i = 0; i < mShape->getNbMaterials(); ++i)
		{
			PxFEMSoftBodyMaterial* material;
			mShape->getSoftBodyMaterials(&material, 1, i);
			core.setMaterial(static_cast<NpFEMSoftBodyMaterial*>(material)->mMaterial.mMaterialIndex);
		}
	}

	bool NpSoftBody::attachShape(PxShape& shape)
	{
		NpShape* npShape = static_cast<NpShape*>(&shape);

		PX_CHECK_AND_RETURN_NULL(npShape->getGeometryTypeFast() == PxGeometryType::eTETRAHEDRONMESH, "NpSoftBody::attachShape: Geometry type must be tetrahedron mesh geometry");
		PX_CHECK_AND_RETURN_NULL(mShape == NULL, "NpSoftBody::attachShape: soft body can just have one shape");
		PX_CHECK_AND_RETURN_NULL(shape.isExclusive(), "NpSoftBody::attachShape: shape must be exclusive");
		PX_CHECK_AND_RETURN_NULL(npShape->getCore().getCore().mShapeCoreFlags & PxShapeCoreFlag::eSOFT_BODY_SHAPE, "NpSoftBody::attachShape: shape must be a soft body shape!");
		
		Dy::SoftBodyCore& core = mCore.getCore();

		PX_CHECK_AND_RETURN_NULL(core.mPositionInvMassCPU == NULL, "NpSoftBody::attachShape: mPositionInvMassCPU already exists, overwrite not allowed, call detachShape first");
		
		PX_CHECK_AND_RETURN_NULL(core.mPositionInvMass == NULL, "NpSoftBody::attachShape: mPositionInvMass already exists, overwrite not allowed, call detachShape first");
		
		mShape = npShape;

		PX_ASSERT(shape.getActor() == NULL);
		npShape->onActorAttach(*this);
			
		const PxGeometryHolder gh(mShape->getGeometry());	// PT: TODO: avoid that copy
		const PxTetrahedronMeshGeometry& tetGeometry = gh.tetMesh();
		Gu::BVTetrahedronMesh* tetMesh = static_cast<Gu::BVTetrahedronMesh*>(tetGeometry.tetrahedronMesh);
		const PxU32 numVerts = tetMesh->getNbVerticesFast();

		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);

		core.mPositionInvMassCPU = physxGpu->createBuffer(numVerts * sizeof(PxVec4), PxBufferType::eHOST, mCudaContextManager, &mCore.getGpuMemStat());
		core.mRestPositionInvMassCPU = physxGpu->createBuffer(numVerts * sizeof(PxVec4), PxBufferType::eHOST, mCudaContextManager, &mCore.getGpuMemStat());
		
		core.mPositionInvMass = physxGpu->createBuffer(numVerts * sizeof(PxVec4), PxBufferType::eDEVICE, mCudaContextManager, &mCore.getGpuMemStat());
		
		const Gu::TetrahedronMesh* meshData = static_cast<const Gu::TetrahedronMesh*>(tetMesh);
		PxVec3* positions = meshData->getVerticesFast();
		PxVec4* positionInv = reinterpret_cast<PxVec4*>(core.mPositionInvMassCPU->map());
		PxVec4* restPositionInv = reinterpret_cast<PxVec4*>(core.mRestPositionInvMassCPU->map());
		for (PxU32 i = 0; i < numVerts; ++i)
		{
			const PxVec3 vert = positions[i];
			positionInv[i] = PxVec4(vert.x, vert.y, vert.z, 1.0f);
			restPositionInv[i] = PxVec4(vert.x, vert.y, vert.z, 1.f);
		}
		core.mPositionInvMassCPU->unmap();
		core.mRestPositionInvMassCPU->unmap();

		updateMaterials();

		return true;
	}

	void NpSoftBody::detachShape()
	{
		PX_CHECK_MSG(getNpSceneFromActor(*this) == NULL, "Detaching a shape from a softbody is currenly only allowed as long as it is not part of a scene. Please remove the softbody from its scene first.");

		Dy::SoftBodyCore& core = mCore.getCore();
		if (core.mPositionInvMassCPU)
		{
			core.mPositionInvMassCPU->release();
			core.mPositionInvMassCPU = NULL;
		}
		if (core.mRestPositionInvMassCPU)
		{
			core.mRestPositionInvMassCPU->release();
			core.mRestPositionInvMassCPU = NULL;
		}
		if (core.mPositionInvMass)
		{
			core.mPositionInvMass->release();
			core.mPositionInvMass = NULL;
		}

		if (core.mKinematicTargetCPU)
		{
			core.mKinematicTargetCPU->release();
			core.mKinematicTargetCPU = NULL;
		}

		if (core.mKinematicTarget)
		{
			core.mKinematicTarget->release();
			core.mKinematicTarget = NULL;
		}

		if (mShape)
			mShape->onActorDetach();
		mShape = NULL;
	}

	void NpSoftBody::detachSimulationMesh()
	{
		Dy::SoftBodyCore& core = mCore.getCore();
		if (core.mSimPositionInvMassCPU) 
		{
			core.mSimPositionInvMassCPU->release();
			core.mSimPositionInvMassCPU = NULL;
		}
		if (core.mSimPositionInvMass)
		{
			core.mSimPositionInvMass->release();
			core.mSimPositionInvMass = NULL;
		}

		if (core.mSimVelocityInvMassCPU)
		{
			core.mSimVelocityInvMassCPU->release();
			core.mSimVelocityInvMassCPU = NULL;
		}
		if (core.mSimVelocityInvMass)
		{
			core.mSimVelocityInvMass->release();
			core.mSimVelocityInvMass = NULL;
		}

		if (core.mKinematicTargetCPU)
		{
			core.mKinematicTargetCPU->release();
			core.mKinematicTargetCPU = NULL;
		}
		if (core.mKinematicTarget)
		{
			core.mKinematicTarget->release();
			core.mKinematicTarget = NULL;
		}

		mSimulationMesh = NULL;
		mSoftBodyAuxData = NULL;
	}

	void NpSoftBody::release()
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

	//	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

		if (npScene)
		{
			npScene->scRemoveSoftBody(*this);
			npScene->removeFromSoftBodyList(*this);
		}

		detachShape();

		PX_ASSERT(!isAPIWriteForbidden());

		NpDestroySoftBody(this);
	}

	void NpSoftBody::setName(const char* debugName)
	{
		NP_WRITE_CHECK(getNpScene());
		mName = debugName;
	}

	const char* NpSoftBody::getName() const
	{
		NP_READ_CHECK(getNpScene());
		return mName;
	}

	void NpSoftBody::addParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::addParticleFilter: Soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((particlesystem == NULL || particlesystem->getScene() != NULL), "NpSoftBody::addParticleFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::addParticleFilter: Illegal to call while simulation is running.");

		NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
		Sc::ParticleSystemCore& core = npParticleSystem->getCore();
		mCore.addParticleFilter(&core, particleId, buffer ? buffer->bufferUniqueId : 0, tetId);
	}

	void NpSoftBody::removeParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeParticleFilter: Soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((particlesystem == NULL || particlesystem->getScene() != NULL), "NpSoftBody::removeParticleFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeParticleFilter: Illegal to call while simulation is running.");

		NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
		Sc::ParticleSystemCore& core = npParticleSystem->getCore();
		mCore.removeParticleFilter(&core, particleId, buffer ? buffer->bufferUniqueId : 0, tetId);
	}

	PxU32 NpSoftBody::addParticleAttachment(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId, const PxVec4& barycentric)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpSoftBody::addParticleAttachment: Soft body must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL((particlesystem == NULL || particlesystem->getScene() != NULL), "NpSoftBody::addParticleAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpSoftBody::addParticleAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
		Sc::ParticleSystemCore& core = npParticleSystem->getCore();
		return mCore.addParticleAttachment(&core, particleId, buffer ? buffer->bufferUniqueId : 0, tetId, barycentric);
	}

	void NpSoftBody::removeParticleAttachment(PxPBDParticleSystem* particlesystem, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::addParticleAttachment: Soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((particlesystem == NULL || particlesystem->getScene() != NULL), "NpSoftBody::addParticleAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::addParticleAttachment: Illegal to call while simulation is running.");

		NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
		Sc::ParticleSystemCore& core = npParticleSystem->getCore();
		mCore.removeParticleAttachment(&core, handle);
	}

	void NpSoftBody::addRigidFilter(PxRigidActor* actor, PxU32 vertId)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::addRigidFilter: Soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpSoftBody::addRigidFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::addRigidFilter: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);
		mCore.addRigidFilter(core, vertId);
	}

	void NpSoftBody::removeRigidFilter(PxRigidActor* actor, PxU32 vertId)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeRigidFilter: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpSoftBody::removeRigidFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeRigidFilter: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);
		mCore.removeRigidFilter(core, vertId);
	}

	PxU32 NpSoftBody::addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpSoftBody::addRigidAttachment: Soft body must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL((actor == NULL || actor->getScene() != NULL), "NpSoftBody::addRigidAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpSoftBody::addRigidAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		Sc::BodyCore* core = getBodyCore(actor);

		PxVec3 aPose = actorSpacePose;
		if(actor && actor->getConcreteType()==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor);
			aPose = stat->getGlobalPose().transform(aPose);
		}

		return mCore.addRigidAttachment(core, vertId, aPose, constraint);
	}

	void NpSoftBody::removeRigidAttachment(PxRigidActor* actor, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeRigidAttachment: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpSoftBody::removeRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeRigidAttachment: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);
		mCore.removeRigidAttachment(core, handle);
	}

	void NpSoftBody::addTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::addTetRigidFilter: Soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpSoftBody::addTetRigidFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::addTetRigidFilter: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);
		return mCore.addTetRigidFilter(core, tetIdx);
	}

	void NpSoftBody::removeTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeTetRigidFilter: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpSoftBody::removeTetRigidFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeTetRigidFilter: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);
		mCore.removeTetRigidFilter(core, tetIdx);
	}

	PxU32 NpSoftBody::addTetRigidAttachment(PxRigidActor* actor, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpSoftBody::addTetRigidAttachment: Soft body must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL((actor == NULL || actor->getScene() != NULL), "NpSoftBody::addTetRigidAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpSoftBody::addTetRigidAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		Sc::BodyCore* core = getBodyCore(actor);

		PxVec3 aPose = actorSpacePose;
		if(actor && actor->getConcreteType()==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor);
			aPose = stat->getGlobalPose().transform(aPose);
		}

		return mCore.addTetRigidAttachment(core, tetIdx, barycentric, aPose, constraint);
	}

	void NpSoftBody::addSoftBodyFilter(PxSoftBody* softbody0, PxU32 tetIdx0, PxU32 tetIdx1)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(softbody0 != NULL, "NpSoftBody::addSoftBodyFilter: soft body must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::addSoftBodyFilter: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "NpSoftBody::addSoftBodyFilter: soft body must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::addSoftBodyFilter: Illegal to call while simulation is running.");

		NpSoftBody* dyn = static_cast<NpSoftBody*>(softbody0);
		Sc::SoftBodyCore* core = &dyn->getCore();

		mCore.addSoftBodyFilter(*core, tetIdx0, tetIdx1);
	}

	void NpSoftBody::removeSoftBodyFilter(PxSoftBody* softbody0, PxU32 tetIdx0, PxU32 tetIdx1)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(softbody0 != NULL, "NpSoftBody::removeSoftBodyFilter: soft body must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeSoftBodyFilter: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "NpSoftBody::removeSoftBodyFilter: soft body must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeSoftBodyFilter: Illegal to call while simulation is running.");

		NpSoftBody* dyn = static_cast<NpSoftBody*>(softbody0);
		Sc::SoftBodyCore* core = &dyn->getCore();

		mCore.removeSoftBodyFilter(*core, tetIdx0, tetIdx1);
	}

	void NpSoftBody::addSoftBodyFilters(PxSoftBody* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(softbody0 != NULL, "NpSoftBody::addSoftBodyFilter: soft body must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::addSoftBodyFilter: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "NpSoftBody::addSoftBodyFilter: soft body must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::addSoftBodyFilter: Illegal to call while simulation is running.");

		NpSoftBody* dyn = static_cast<NpSoftBody*>(softbody0);
		Sc::SoftBodyCore* core = &dyn->getCore();

		mCore.addSoftBodyFilters(*core, tetIndices0, tetIndices1, tetIndicesSize);
	}

	void NpSoftBody::removeSoftBodyFilters(PxSoftBody* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(softbody0 != NULL, "NpSoftBody::removeSoftBodyFilter: soft body must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeSoftBodyFilter: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "NpSoftBody::removeSoftBodyFilter: soft body must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeSoftBodyFilter: Illegal to call while simulation is running.");

		NpSoftBody* dyn = static_cast<NpSoftBody*>(softbody0);
		Sc::SoftBodyCore* core = &dyn->getCore();

		mCore.removeSoftBodyFilters(*core, tetIndices0, tetIndices1, tetIndicesSize);
	}

	PxU32 NpSoftBody::addSoftBodyAttachment(PxSoftBody* softbody0, PxU32 tetIdx0, const PxVec4& tetBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1, PxConeLimitedConstraint* constraint)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(softbody0 != NULL, "NpSoftBody::addSoftBodyAttachment: soft body must not be null", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpSoftBody::addSoftBodyAttachment: soft body must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL(softbody0->getScene() != NULL, "NpSoftBody::addSoftBodyAttachment: soft body must be inserted into the scene.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpSoftBody::addSoftBodyAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		NpSoftBody* dyn = static_cast<NpSoftBody*>(softbody0);
		Sc::SoftBodyCore* core = &dyn->getCore();

		return mCore.addSoftBodyAttachment(*core, tetIdx0, tetBarycentric0, tetIdx1, tetBarycentric1, constraint);
	}

	void NpSoftBody::removeSoftBodyAttachment(PxSoftBody* softbody0, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(softbody0 != NULL, "NpSoftBody::removeSoftBodyAttachment: soft body must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeSoftBodyAttachment: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "NpSoftBody::removeSoftBodyAttachment: soft body must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeSoftBodyAttachment: Illegal to call while simulation is running.");

		NpSoftBody* dyn = static_cast<NpSoftBody*>(softbody0);
		Sc::SoftBodyCore* core = &dyn->getCore();

		mCore.removeSoftBodyAttachment(*core, handle);
	}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	void NpSoftBody::addClothFilter(PxFEMCloth* cloth, PxU32 triIdx, PxU32 tetIdx)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(cloth != NULL, "NpSoftBody::addClothFilter: actor must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::addClothFilter: Soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(cloth->getScene() != NULL, "NpSoftBody::addClothFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::addClothFilter: Illegal to call while simulation is running.");

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(cloth);
		Sc::FEMClothCore* core = &dyn->getCore();

		return mCore.addClothFilter(*core, triIdx, tetIdx);
	}

	void NpSoftBody::removeClothFilter(PxFEMCloth* cloth, PxU32 triIdx, PxU32 tetIdx)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(cloth != NULL, "NpSoftBody::removeClothFilter: actor must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::removeClothFilter: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(cloth->getScene() != NULL, "NpSoftBody::removeClothFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::removeClothFilter: Illegal to call while simulation is running.");

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(cloth);
		Sc::FEMClothCore* core = &dyn->getCore();

		mCore.removeClothFilter(*core, triIdx, tetIdx);
	}

	PxU32 NpSoftBody::addClothAttachment(PxFEMCloth* cloth, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric, PxConeLimitedConstraint* constraint)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(cloth != NULL, "NpSoftBody::addClothAttachment: actor must not be null", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpSoftBody::addClothAttachment: Soft body must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL(cloth->getScene() != NULL, "NpSoftBody::addClothAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpSoftBody::addClothAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(cloth);
		Sc::FEMClothCore* core = &dyn->getCore();
		
		return mCore.addClothAttachment(*core, triIdx, triBarycentric, tetIdx, tetBarycentric, constraint);
	}

	void NpSoftBody::removeClothAttachment(PxFEMCloth* cloth, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(cloth != NULL, "NpSoftBody::releaseClothAttachment: actor must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpSoftBody::releaseClothAttachment: soft body must be inserted into the scene.");
		PX_CHECK_AND_RETURN(cloth->getScene() != NULL, "NpSoftBody::releaseClothAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpSoftBody::releaseClothAttachment: Illegal to call while simulation is running.");

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(cloth);
		Sc::FEMClothCore* core = &dyn->getCore();

		mCore.removeClothAttachment(*core, handle);
	}
#endif
}

#endif //PX_SUPPORT_GPU_PHYSX

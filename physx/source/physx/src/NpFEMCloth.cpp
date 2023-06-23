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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpFEMCloth.h"
#include "NpCheck.h"
#include "NpScene.h"
#include "NpShape.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "GuTriangleMesh.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpArticulationLink.h"

#include "GuTriangleMesh.h"
#include "ScFEMClothSim.h"

using namespace physx;

namespace physx
{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	NpFEMCloth::NpFEMCloth(PxCudaContextManager& cudaContextManager)
		:
		NpActorTemplate(PxConcreteType::eFEM_CLOTH, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::eFEMCLOTH),
		mShape(NULL),
		mCudaContextManager(&cudaContextManager)
	{
	}

	NpFEMCloth::NpFEMCloth(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager) :
		NpActorTemplate(baseFlags),
		mShape(NULL),
		mCudaContextManager(&cudaContextManager)
	{
	}

	PxBounds3 NpFEMCloth::getWorldBounds(float inflation) const
	{
		NP_READ_CHECK(getNpScene());

		if (!getNpScene())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Querying bounds of a PxFEMCloth which is not part of a PxScene is not supported.");
			return PxBounds3::empty();
		}

		const Sc::FEMClothSim* sim = mCore.getSim();
		PX_ASSERT(sim);

		PX_SIMD_GUARD;

		const PxBounds3 bounds = sim->getBounds();
		PX_ASSERT(bounds.isValid());

		// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
		const PxVec3 center = bounds.getCenter();
		const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
		return PxBounds3::centerExtents(center, inflatedExtents);	
	}

	void NpFEMCloth::setFEMClothFlag(PxFEMClothFlag::Enum flag, bool val)
	{
		PxFEMClothFlags flags = mCore.getFlags();
		if (val)
			flags.raise(flag);
		else
			flags.clear(flag);

		mCore.setFlags(flags);
	}

	void NpFEMCloth::setFEMClothFlags(PxFEMClothFlags flags)
	{
		mCore.setFlags(flags);
	}

	PxFEMClothFlags NpFEMCloth::getFEMClothFlag() const
	{
		return mCore.getFlags();
	}

#if 0 // disabled until future use.
	void NpFEMCloth::setDrag(const PxReal drag)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setDrag() not allowed while simulation is running. Call will be ignored.")

		mCore.setDrag(drag);
		UPDATE_PVD_PROPERTY
	}

	PxReal NpFEMCloth::getDrag() const
	{
		return mCore.getDrag();
	}

	void NpFEMCloth::setLift(const PxReal lift)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setLift() not allowed while simulation is running. Call will be ignored.")

		mCore.setLift(lift);
		UPDATE_PVD_PROPERTY
	}

	PxReal NpFEMCloth::getLift() const
	{
		return mCore.getLift();
	}

	void NpFEMCloth::setWind(const PxVec3& wind)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setWind() not allowed while simulation is running. Call will be ignored.")

		mCore.setWind(wind);
		UPDATE_PVD_PROPERTY
	}

    PxVec3 NpFEMCloth::getWind() const
	{
		return mCore.getWind();
	}

    void NpFEMCloth::setAirDensity(const PxReal airDensity)
    {
	    NpScene* npScene = getNpScene();
	    NP_WRITE_CHECK(npScene);

	    PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setAirDensity() not allowed while simulation is running. Call will be ignored.")

	    mCore.setAirDensity(airDensity);
	    UPDATE_PVD_PROPERTY
    }

	float NpFEMCloth::getAirDensity() const 
	{ 
		return mCore.getAirDensity(); 
	}

	void NpFEMCloth::setBendingActivationAngle(const PxReal angle)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setBendingActivationAngle() not allowed while simulation is running. Call will be ignored.")

			mCore.setBendingActivationAngle(angle);
		UPDATE_PVD_PROPERTY
	}

	PxReal NpFEMCloth::getBendingActivationAngle() const
	{
		return mCore.getBendingActivationAngle();
	}
#endif

	void NpFEMCloth::setParameter(const PxFEMParameters& paramters)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setParameter() not allowed while simulation is running. Call will be ignored.")

		mCore.setParameter(paramters);
		UPDATE_PVD_PROPERTY
	}

	PxFEMParameters NpFEMCloth::getParameter() const
	{
		NP_READ_CHECK(getNpScene());
		return mCore.getParameter();
	}

	void NpFEMCloth::setBendingScales(const PxReal* const bendingScales, PxU32 nbElements)
    {
	    NpScene* npScene = getNpScene();
	    NP_WRITE_CHECK(npScene);

	    PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setBendingScales() not allowed while simulation is running. Call will be ignored.")

	    mCore.setBendingScales(bendingScales, nbElements);
	    UPDATE_PVD_PROPERTY
    }

    const PxReal* NpFEMCloth::getBendingScales() const
    {
	    NP_READ_CHECK(getNpScene());
	    return mCore.getBendingScales();
    }

    PxU32 NpFEMCloth::getNbBendingScales() const
    {
	    NP_READ_CHECK(getNpScene());
	    return mCore.getNbBendingScales();
    }

	void NpFEMCloth::setMaxVelocity(const PxReal v) 
	{ 
	    NpScene* npScene = getNpScene();
	    NP_WRITE_CHECK(npScene);

	    PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setMaxVelocity() not allowed while simulation is running. Call will be ignored.")

	    mCore.setMaxVelocity(v);
	    UPDATE_PVD_PROPERTY
	}

    PxReal NpFEMCloth::getMaxVelocity() const 
	{ 
		return mCore.getMaxVelocity(); 
	}

	void NpFEMCloth::setMaxDepenetrationVelocity(const PxReal v)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setMaxDepenetrationVelocity() not allowed while simulation is running. Call will be ignored.")

		mCore.setMaxDepenetrationVelocity(v);
		UPDATE_PVD_PROPERTY
	}

	PxReal NpFEMCloth::getMaxDepenetrationVelocity() const
	{
		return mCore.getMaxDepenetrationVelocity();
	}

	void NpFEMCloth::setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setNbCollisionPairUpdatesPerTimestep() not allowed while simulation is running. Call will be ignored.")

		mCore.setNbCollisionPairUpdatesPerTimestep(frequency);
		UPDATE_PVD_PROPERTY
	}

	PxU32 NpFEMCloth::getNbCollisionPairUpdatesPerTimestep() const
	{
		return mCore.getNbCollisionPairUpdatesPerTimestep();
	}

	void NpFEMCloth::setNbCollisionSubsteps(const PxU32 frequency)
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setNbCollisionSubsteps() not allowed while simulation is running. Call will be ignored.")

		mCore.setNbCollisionSubsteps(frequency);
		UPDATE_PVD_PROPERTY
	}

	PxU32 NpFEMCloth::getNbCollisionSubsteps() const
	{
		return mCore.getNbCollisionSubsteps();
	}

	PxVec4* NpFEMCloth::getPositionInvMassBufferD()
	{
		PX_CHECK_AND_RETURN_NULL(mShape != NULL, " NpFEMCloth::getPositionInvMassBufferD: FEM cloth does not have a shape, attach shape first.");

		Dy::FEMClothCore& core = mCore.getCore();
		return core.mPositionInvMass;
	}

	PxVec4* NpFEMCloth::getVelocityBufferD()
	{
		PX_CHECK_AND_RETURN_NULL(mShape != NULL, " NpFEMCloth::getVelocityBufferD: FEM cloth does not have a shape, attach shape first.");

		Dy::FEMClothCore& core = mCore.getCore();
		return core.mVelocity;
	}

	PxVec4* NpFEMCloth::getRestPositionBufferD()
	{
		PX_CHECK_AND_RETURN_NULL(mShape != NULL, " NpFEMCloth::getRestPositionBufferD: FEM cloth does not have a shape, attach shape first.");
		
		Dy::FEMClothCore& core = mCore.getCore();
		return core.mRestPosition;
	}

	void NpFEMCloth::markDirty(PxFEMClothDataFlags flags)
	{
		NP_WRITE_CHECK(getNpScene());

		Dy::FEMClothCore& core = mCore.getCore();
		core.mDirtyFlags |= flags;
	}

	PxCudaContextManager* NpFEMCloth::getCudaContextManager() const
	{
		return mCudaContextManager;
	}

	void NpFEMCloth::setCudaContextManager(PxCudaContextManager* cudaContextManager)
	{
		mCudaContextManager = cudaContextManager;
	}

	void NpFEMCloth::setSolverIterationCounts(PxU32 positionIters)  // maybe use PxU16?
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);
		PX_CHECK_AND_RETURN(positionIters > 0, "NpFEMCloth::setSolverIterationCounts: positionIters must be more than zero!");
		PX_CHECK_AND_RETURN(positionIters <= 255, "NpFEMCloth::setSolverIterationCounts: positionIters must be no greater than 255!");
		//PX_CHECK_AND_RETURN(velocityIters > 0, "NpFEMCloth::setSolverIterationCounts: velocityIters must be more than zero!");
		//PX_CHECK_AND_RETURN(velocityIters <= 255, "NpFEMCloth::setSolverIterationCounts: velocityIters must be no greater than 255!");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxFEMCloth::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.")

		//mCore.setSolverIterationCounts((velocityIters & 0xff) << 8 | (positionIters & 0xff));
		mCore.setSolverIterationCounts(positionIters & 0xff);
	}

	void NpFEMCloth::getSolverIterationCounts(PxU32& positionIters) const
	{
		NP_READ_CHECK(getNpScene());

		PxU16 x = mCore.getSolverIterationCounts();
		//velocityIters = PxU32(x >> 8);
		positionIters = PxU32(x & 0xff);
	}

	PxShape* NpFEMCloth::getShape()
	{
		return mShape;
	}

	bool NpFEMCloth::attachShape(PxShape& shape)
	{
		NpShape* npShape = static_cast<NpShape*>(&shape);

		PX_CHECK_AND_RETURN_NULL(npShape->getGeometryTypeFast() == PxGeometryType::eTRIANGLEMESH, "NpFEMCloth::attachShape: Geometry type must be triangle mesh geometry");
		PX_CHECK_AND_RETURN_NULL(mShape == NULL, "NpFEMCloth::attachShape: FEM-cloth can just have one shape");
		PX_CHECK_AND_RETURN_NULL(shape.isExclusive(), "NpFEMCloth::attachShape: shape must be exclusive");

		Dy::FEMClothCore& core = mCore.getCore();
		PX_CHECK_AND_RETURN_NULL(core.mPositionInvMass == NULL, "NpFEMCloth::attachShape: mPositionInvMass already exists, overwrite not allowed, call detachShape first");
		PX_CHECK_AND_RETURN_NULL(core.mVelocity == NULL, "NpFEMCloth::attachShape: mClothVelocity already exists, overwrite not allowed, call detachShape first");
		PX_CHECK_AND_RETURN_NULL(core.mRestPosition == NULL, "NpFEMCloth::attachShape: mClothRestPosition already exists, overwrite not allowed, call detachShape first");

		const PxTriangleMeshGeometry& geom = static_cast<const PxTriangleMeshGeometry&>(npShape->getGeometry());
		Gu::TriangleMesh* guMesh = static_cast<Gu::TriangleMesh*>(geom.triangleMesh);

#if PX_CHECKED
		const PxU32 triangleReference = guMesh->getNbTriangleReferences();
		PX_CHECK_AND_RETURN_NULL(triangleReference > 0, "NpFEMCloth::attachShape: cloth triangle mesh has cooked with eENABLE_VERT_MAPPING");
#endif		

		mShape = npShape;

		PX_ASSERT(shape.getActor() == NULL);
		npShape->onActorAttach(*this);

		updateMaterials();

		const PxU32 numVerts = guMesh->getNbVerticesFast();

		core.mPositionInvMass = PX_DEVICE_ALLOC_T(PxVec4, mCudaContextManager, numVerts);
		core.mVelocity = PX_DEVICE_ALLOC_T(PxVec4, mCudaContextManager, numVerts);
		core.mRestPosition = PX_DEVICE_ALLOC_T(PxVec4, mCudaContextManager, numVerts);

		return true;
	}
	
	void NpFEMCloth::addRigidFilter(PxRigidActor* actor, PxU32 vertId)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::addRigidFilter: Cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::addRigidFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::addRigidFilter: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);

		return mCore.addRigidFilter(core, vertId);
	}

	void NpFEMCloth::removeRigidFilter(PxRigidActor* actor, PxU32 vertId)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::removeRigidFilter: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::removeRigidFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::removeRigidFilter: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);

		mCore.removeRigidFilter(core, vertId);
	}

	PxU32 NpFEMCloth::addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpFEMCloth::addRigidAttachment: cloth must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::addRigidAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL(constraint == NULL || constraint->isValid(), "NpFEMCloth::addRigidAttachment: PxConeLimitedConstraint needs to be valid if specified.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpFEMCloth::addRigidAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		Sc::BodyCore* core = getBodyCore(actor);

		PxVec3 aPose = actorSpacePose;
		if(actor && actor->getConcreteType()==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor);
			aPose = stat->getGlobalPose().transform(aPose);
		}

		return mCore.addRigidAttachment(core, vertId, aPose, constraint);
	}

	void NpFEMCloth::removeRigidAttachment(PxRigidActor* actor, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::removeRigidAttachment: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::removeRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::removeRigidAttachment: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);

		mCore.removeRigidAttachment(core, handle);
	}

	void NpFEMCloth::addTriRigidFilter(PxRigidActor* actor, PxU32 triangleIdx)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::addTriRigidFilter: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::addTriRigidFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::addTriRigidFilter: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);

		return mCore.addTriRigidFilter(core, triangleIdx);
	}

	void NpFEMCloth::removeTriRigidFilter(PxRigidActor* actor, PxU32 triangleId)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::releaseTriRigidAttachment: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::releaseTriRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::releaseTriRigidAttachment: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);

		mCore.removeTriRigidFilter(core, triangleId);
	}

	PxU32 NpFEMCloth::addTriRigidAttachment(PxRigidActor* actor, PxU32 triangleIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpFEMCloth::addTriRigidAttachment: cloth must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::addTriRigidAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL(constraint == NULL || constraint->isValid(), "NpFEMCloth::addTriRigidAttachment: PxConeLimitedConstraint needs to be valid if specified.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpFEMCloth::addTriRigidAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		Sc::BodyCore* core = getBodyCore(actor);

		PxVec3 aPose = actorSpacePose;
		if(actor && actor->getConcreteType()==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor);
			aPose = stat->getGlobalPose().transform(aPose);
		}

		return mCore.addTriRigidAttachment(core, triangleIdx, barycentric, aPose, constraint);
	}

	void NpFEMCloth::removeTriRigidAttachment(PxRigidActor* actor, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::releaseTriRigidAttachment: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpFEMCloth::releaseTriRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::releaseTriRigidAttachment: Illegal to call while simulation is running.");

		Sc::BodyCore* core = getBodyCore(actor);

		mCore.removeTriRigidAttachment(core, handle);
	}

	void NpFEMCloth::addClothFilter(PxFEMCloth* otherCloth, PxU32 otherTriIdx, PxU32 triIdx)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::addClothAttachment: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN((otherCloth == NULL || otherCloth->getScene() != NULL), "NpFEMCloth::addClothFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::addClothFilter: Illegal to call while simulation is running.");

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(otherCloth);
		Sc::FEMClothCore* otherCore = &dyn->getCore();

		return mCore.addClothFilter(otherCore, otherTriIdx, triIdx);
	}

	void NpFEMCloth::removeClothFilter(PxFEMCloth* otherCloth, PxU32 otherTriIdx, PxU32 triIdx)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(otherCloth != NULL, "NpFEMCloth::removeClothFilter: actor must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::removeClothFilter: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN(otherCloth->getScene() != NULL, "NpFEMCloth::removeClothFilter: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::removeClothFilter: Illegal to call while simulation is running.");

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(otherCloth);
		Sc::FEMClothCore* otherCore = &dyn->getCore();

		mCore.removeClothFilter(otherCore, otherTriIdx, triIdx);
	}

	PxU32 NpFEMCloth::addClothAttachment(PxFEMCloth* otherCloth, PxU32 otherTriIdx, const PxVec4& otherTriBarycentric, PxU32 triIdx, const PxVec4& triBarycentric)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "NpFEMCloth::addClothAttachment: cloth must be inserted into the scene.", 0xFFFFFFFF);
		PX_CHECK_AND_RETURN_VAL((otherCloth == NULL || otherCloth->getScene() != NULL), "NpFEMCloth::addClothAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "NpFEMCloth::addClothAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(otherCloth);
		Sc::FEMClothCore* otherCore = &dyn->getCore();

		return mCore.addClothAttachment(otherCore, otherTriIdx, otherTriBarycentric, triIdx, triBarycentric);
	}

	void NpFEMCloth::removeClothAttachment(PxFEMCloth* otherCloth, PxU32 handle)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(otherCloth != NULL, "NpFEMCloth::releaseClothAttachment: actor must not be null");
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFEMCloth::releaseClothAttachment: cloth must be inserted into the scene.");
		PX_CHECK_AND_RETURN(otherCloth->getScene() != NULL, "NpFEMCloth::releaseClothAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpFEMCloth::releaseClothAttachment: Illegal to call while simulation is running.");

		NpFEMCloth* dyn = static_cast<NpFEMCloth*>(otherCloth);
		Sc::FEMClothCore* otherCore = &dyn->getCore();

		mCore.removeClothAttachment(otherCore, handle);
	}

	void NpFEMCloth::detachShape()
	{
		Dy::FEMClothCore& core = mCore.getCore();

		if (core.mPositionInvMass)
		{
			PX_DEVICE_FREE(mCudaContextManager, core.mPositionInvMass);
			core.mPositionInvMass = NULL;
		}

		if (core.mVelocity)
		{
			PX_DEVICE_FREE(mCudaContextManager, core.mVelocity);
			core.mVelocity = NULL;
		}

		if (core.mRestPosition)
		{
			PX_DEVICE_FREE(mCudaContextManager, core.mRestPosition);
			core.mRestPosition = NULL;
		}

		if (mShape)
			mShape->onActorDetach();
		mShape = NULL;
	}

	void NpFEMCloth::release()
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		if (npScene)
		{
			npScene->scRemoveFEMCloth(*this);
			npScene->removeFromFEMClothList(*this);
		}

		detachShape();

		PX_ASSERT(!isAPIWriteForbidden());
		NpDestroyFEMCloth(this);
	}

	void NpFEMCloth::setName(const char* debugName)
	{
		NP_WRITE_CHECK(getNpScene());
		mName = debugName;
	}

	const char* NpFEMCloth::getName() const
	{
		NP_READ_CHECK(getNpScene());
		return mName;
	}

	bool NpFEMCloth::isSleeping() const
	{
		Sc::FEMClothSim* sim = mCore.getSim();
		if (sim)
		{
			return sim->isSleeping();
		}
		return true;
	}

	void NpFEMCloth::updateMaterials()
	{
		Dy::FEMClothCore& core = mCore.getCore();
		core.clearMaterials();
		for (PxU32 i = 0; i < mShape->getNbMaterials(); ++i)
		{
			PxFEMClothMaterial* material;
			mShape->getClothMaterials(&material, 1, i);
			core.setMaterial(static_cast<NpFEMClothMaterial*>(material)->mMaterial.mMaterialIndex);
		}
	}
#endif
}

#endif //PX_SUPPORT_GPU_PHYSX

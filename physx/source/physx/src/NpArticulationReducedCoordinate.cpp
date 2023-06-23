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

#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationTendon.h"
#include "NpArticulationSensor.h"

#include "DyFeatherstoneArticulation.h"
#include "ScArticulationSim.h"
#include "ScConstraintSim.h"

#include "foundation/PxAlignedMalloc.h"
#include "foundation/PxPool.h"

#include "PxPvdDataStream.h"
#include "NpAggregate.h"

using namespace physx;

void PxArticulationCache::release()
{
	PxcScratchAllocator* scratchAlloc = reinterpret_cast<PxcScratchAllocator*>(scratchAllocator);
	PX_DELETE(scratchAlloc);
	scratchAllocator = NULL;

	PX_FREE(scratchMemory);

	PX_FREE_THIS;
}

// PX_SERIALIZATION
NpArticulationReducedCoordinate* NpArticulationReducedCoordinate::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationReducedCoordinate* obj = PX_PLACEMENT_NEW(address, NpArticulationReducedCoordinate(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpArticulationReducedCoordinate);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void NpArticulationReducedCoordinate::preExportDataReset()
{
	//for now, no support for loop joint serialization
	PxArray<NpConstraint*> emptyLoopJoints;
	PxMemCopy(&mLoopJoints, &emptyLoopJoints, sizeof(PxArray<NpConstraint*>));
}

//~PX_SERIALIZATION

NpArticulationReducedCoordinate::NpArticulationReducedCoordinate()
	: PxArticulationReducedCoordinate(PxConcreteType::eARTICULATION_REDUCED_COORDINATE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE), 
	NpBase(NpType::eARTICULATION), mNumShapes(0), mAggregate(NULL), mName(NULL), mCacheVersion(0), mTopologyChanged(false)
{
}

void NpArticulationReducedCoordinate::setArticulationFlags(PxArticulationFlags flags)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setArticulationFlags() not allowed while simulation is running. Call will be ignored.");

	scSetArticulationFlags(flags);

	OMNI_PVD_SET(PxArticulationReducedCoordinate, articulationFlags, static_cast<const PxArticulationReducedCoordinate&>(*this), flags);
}

void NpArticulationReducedCoordinate::setArticulationFlag(PxArticulationFlag::Enum flag, bool value)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setArticulationFlag() not allowed while simulation is running. Call will be ignored.");

	PxArticulationFlags flags = mCore.getArticulationFlags();

	if(value)
		flags |= flag;
	else
		flags &= (~flag);

	scSetArticulationFlags(flags);

	OMNI_PVD_SET(PxArticulationReducedCoordinate, articulationFlags, static_cast<const PxArticulationReducedCoordinate&>(*this), flags);
}

PxArticulationFlags	NpArticulationReducedCoordinate::getArticulationFlags() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getArticulationFlags();
}

PxU32 NpArticulationReducedCoordinate::getDofs() const
{
	NP_READ_CHECK(getNpScene());
	// core will check if in scene and return 0xFFFFFFFF if not.
	return mCore.getDofs();
}

PxArticulationCache* NpArticulationReducedCoordinate::createCache() const
{
	NP_READ_CHECK(getNpScene());	// doesn't modify the scene, only reads
	PX_CHECK_AND_RETURN_NULL(getNpScene(), "PxArticulationReducedCoordinate::createCache: Articulation must be in a scene.");

	PxArticulationCache* cache = mCore.createCache();
	if (cache)
		cache->version = mCacheVersion;

	return cache;
}

PxU32 NpArticulationReducedCoordinate::getCacheDataSize() const
{
	NP_READ_CHECK(getNpScene());	// doesn't modify the scene, only reads
	// core will check if in scene and return 0xFFFFFFFF if not.
	return mCore.getCacheDataSize();
}

void NpArticulationReducedCoordinate::zeroCache(PxArticulationCache& cache) const
{
	NP_READ_CHECK(getNpScene());	// doesn't modify the scene, only reads
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::zeroCache: Articulation must be in a scene.");
	// need to check cache version as correct cache size is required for zeroing
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::zeroCache: cache is invalid, articulation configuration has changed! ");

	return mCore.zeroCache(cache);
}

void NpArticulationReducedCoordinate::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flags, bool autowake)
{
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::applyCache: Articulation must be in a scene.");

	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::applyCache: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_AND_RETURN(!(getScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API), "PxArticulationReducedCoordinate::applyCache : it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");

	//if we try to do a bulk op when sim is running, return with error
	if (getNpScene()->getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
			"PxArticulationReducedCoordinate::applyCache() not allowed while simulation is running. Call will be ignored.");
		return;
	}

	if (!(getScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API))
	{
		const bool forceWake = mCore.applyCache(cache, flags);

		if (flags & (PxArticulationCacheFlag::ePOSITION | PxArticulationCacheFlag::eROOT_TRANSFORM))
		{
			const PxU32 linkCount = mArticulationLinks.size();

			//KS - the below code forces contact managers to be updated/cached data to be dropped and
			//shape transforms to be updated.
			for (PxU32 i = 0; i < linkCount; ++i)
			{
				NpArticulationLink* link = mArticulationLinks[i];
				//in the lowlevel articulation, we have already updated bodyCore's body2World
				const PxTransform internalPose = link->getCore().getBody2World();
				link->scSetBody2World(internalPose);
			}
		}

		wakeUpInternal(forceWake, autowake);
	}
}

void NpArticulationReducedCoordinate::copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flags) const
{
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::copyInternalStateToCache: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::copyInternalStateToCache: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::copyInternalStateToCache() not allowed while simulation is running. Call will be ignored.");

	PX_CHECK_AND_RETURN(!(getScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API), "PxArticulationReducedCoordinate::copyInternalStateToCache : it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");

	const bool isGpuSimEnabled = getNpScene()->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;

	mCore.copyInternalStateToCache(cache, flags, isGpuSimEnabled);
}

void NpArticulationReducedCoordinate::packJointData(const PxReal* maximum, PxReal* reduced) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::packJointData: Articulation must be in a scene.");

	mCore.packJointData(maximum, reduced);
}

void NpArticulationReducedCoordinate::unpackJointData(const PxReal* reduced, PxReal* maximum) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::unpackJointData: Articulation must be in a scene.");

	mCore.unpackJointData(reduced, maximum);
}

void NpArticulationReducedCoordinate::commonInit() const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::commonInit: Articulation must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::commonInit() not allowed while simulation is running. Call will be ignored.");

	mCore.commonInit();
}

void NpArticulationReducedCoordinate::computeGeneralizedGravityForce(PxArticulationCache& cache) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::computeGeneralizedGravityForce: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version ==mCacheVersion, "PxArticulationReducedCoordinate::computeGeneralizedGravityForce: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::computeGeneralizedGravityForce() not allowed while simulation is running. Call will be ignored.");

	mCore.computeGeneralizedGravityForce(cache);
}

void NpArticulationReducedCoordinate::computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::computeCoriolisAndCentrifugalForce: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::computeCoriolisAndCentrifugalForce: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::computeCoriolisAndCentrifugalForce() not allowed while simulation is running. Call will be ignored.");

	mCore.computeCoriolisAndCentrifugalForce(cache);
}

void NpArticulationReducedCoordinate::computeGeneralizedExternalForce(PxArticulationCache& cache) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::computeGeneralizedExternalForce: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::computeGeneralizedExternalForce: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::computeGeneralizedExternalForce() not allowed while simulation is running. Call will be ignored.");

	mCore.computeGeneralizedExternalForce(cache);
}

void NpArticulationReducedCoordinate::computeJointAcceleration(PxArticulationCache& cache) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::computeJointAcceleration: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::computeJointAcceleration: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::computeJointAcceleration() not allowed while simulation is running. Call will be ignored.");

	mCore.computeJointAcceleration(cache);
}

void NpArticulationReducedCoordinate::computeJointForce(PxArticulationCache& cache) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::computeJointForce: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::computeJointForce: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::computeJointForce() not allowed while simulation is running. Call will be ignored.");

	mCore.computeJointForce(cache);
}

void NpArticulationReducedCoordinate::computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::computeDenseJacobian: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::computeDenseJacobian: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::computeDenseJacobian() not allowed while simulation is running. Call will be ignored.");

	mCore.computeDenseJacobian(cache, nRows, nCols);
}

void NpArticulationReducedCoordinate::computeCoefficientMatrix(PxArticulationCache& cache) const
{
	NpScene* npScene = getNpScene();
	NP_READ_CHECK(npScene);
	PX_CHECK_AND_RETURN(npScene, "PxArticulationReducedCoordinate::computeCoefficientMatrix: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::computeCoefficientMatrix: cache is invalid, articulation configuration has changed! ");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationReducedCoordinate::computeCoefficientMatrix() not allowed while simulation is running. Call will be ignored.");

	npScene->updateConstants(mLoopJoints);

	mCore.computeCoefficientMatrix(cache);
}

bool NpArticulationReducedCoordinate::computeLambda(PxArticulationCache& cache, PxArticulationCache& initialState, const PxReal* const jointTorque, const PxU32 maxIter) const
{
	if (!getNpScene())
		return PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
								"PxArticulationReducedCoordinate::computeLambda: Articulation must be in a scene.");
	NP_READ_CHECK(getNpScene());
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::computeLambda() not allowed while simulation is running. Call will be ignored.", false);

	if (cache.version != mCacheVersion)
		return PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
								"PxArticulationReducedCoordinate::computeLambda: cache is invalid, articulation configuration has changed!");

	return mCore.computeLambda(cache, initialState, jointTorque, getScene()->getGravity(), maxIter);
}

void NpArticulationReducedCoordinate::computeGeneralizedMassMatrix(PxArticulationCache& cache) const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::computeGeneralizedMassMatrix: Articulation must be in a scene.");
	PX_CHECK_AND_RETURN(cache.version == mCacheVersion, "PxArticulationReducedCoordinate::computeGeneralizedMassMatrix: cache is invalid, articulation configuration has changed!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::computeGeneralizedMassMatrix() not allowed while simulation is running. Call will be ignored.");

	mCore.computeGeneralizedMassMatrix(cache);
}

void NpArticulationReducedCoordinate::addLoopJoint(PxConstraint* joint)
{
	NP_WRITE_CHECK(getNpScene());

#if PX_CHECKED
	PxRigidActor* actor0;
	PxRigidActor* actor1;

	joint->getActors(actor0, actor1);

	PxArticulationLink* link0 = NULL;
	PxArticulationLink* link1 = NULL;

	if(actor0 && actor0->getConcreteType()==PxConcreteType::eARTICULATION_LINK)
		link0 = static_cast<PxArticulationLink*>(actor0);

	if(actor1 && actor1->getConcreteType()==PxConcreteType::eARTICULATION_LINK)
		link1 = static_cast<PxArticulationLink*>(actor1);

	PX_CHECK_AND_RETURN((link0 || link1), "PxArticulationReducedCoordinate::addLoopJoint : at least one of the PxRigidActors need to be PxArticulationLink!");

	PxArticulationReducedCoordinate* base0 = NULL;
	PxArticulationReducedCoordinate* base1 = NULL;
	if (link0)
		base0 = &link0->getArticulation();

	if (link1)
		base1 = &link1->getArticulation();

	PX_CHECK_AND_RETURN((base0 == this || base1 == this), "PxArticulationReducedCoordinate::addLoopJoint : at least one of the PxArticulationLink belongs to this articulation!");
#endif

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::addLoopJoint() not allowed while simulation is running. Call will be ignored.")

	const PxU32 size = mLoopJoints.size();
	if (size >= mLoopJoints.capacity())
		mLoopJoints.reserve(size * 2 + 1);

	NpConstraint* constraint = static_cast<NpConstraint*>(joint);
	mLoopJoints.pushBack(constraint);

	Sc::ArticulationSim* scArtSim = mCore.getSim();

	Sc::ConstraintSim* cSim = constraint->getCore().getSim();
	if(scArtSim)
		scArtSim->addLoopConstraint(cSim);
}

void NpArticulationReducedCoordinate::removeLoopJoint(PxConstraint* joint)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::removeLoopJoint() not allowed while simulation is running. Call will be ignored.")

	NpConstraint* constraint = static_cast<NpConstraint*>(joint);
	mLoopJoints.findAndReplaceWithLast(constraint);

	Sc::ArticulationSim* scArtSim = mCore.getSim();

	Sc::ConstraintSim* cSim = constraint->getCore().getSim();
	scArtSim->removeLoopConstraint(cSim);
}

PxU32 NpArticulationReducedCoordinate::getNbLoopJoints() const
{
	NP_READ_CHECK(getNpScene());

	return mLoopJoints.size();
}

PxU32 NpArticulationReducedCoordinate::getLoopJoints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());

	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mLoopJoints.begin(), mLoopJoints.size());
}

PxU32 NpArticulationReducedCoordinate::getCoefficientMatrixSize() const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_NULL(getNpScene(), "PxArticulationReducedCoordinate::getCoefficientMatrixSize: Articulation must be in a scene.");
	
	// core will check if in scene and return 0xFFFFFFFF if not.
	return mCore.getCoefficientMatrixSize();
}

void NpArticulationReducedCoordinate::setRootGlobalPose(const PxTransform& pose, bool autowake)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(!mArticulationLinks.empty(), "PxArticulationReducedCoordinate::setRootGlobalPose() called on empty articulation.");
	PX_CHECK_AND_RETURN(pose.isValid(), "PxArticulationReducedCoordinate::setRootGlobalPose pose is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setRootGlobalPose() not allowed while simulation is running. Call will be ignored.");

	NpArticulationLink* root = mArticulationLinks[0];
	root->setGlobalPoseInternal(pose, autowake);
}

PxTransform	NpArticulationReducedCoordinate::getRootGlobalPose() const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(!mArticulationLinks.empty(), "PxArticulationReducedCoordinate::getRootGlobalPose() called on empty articulation.", PxTransform(PxIdentity));

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getRootGlobalPose() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", PxTransform(PxIdentity));

	NpArticulationLink* root = mArticulationLinks[0];
	return root->getGlobalPose();
}

void NpArticulationReducedCoordinate::setRootLinearVelocity(const PxVec3& linearVelocity, bool autowake)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(!mArticulationLinks.empty(), "PxArticulationReducedCoordinate::setRootLinearVelocity() called on empty articulation.");
	PX_CHECK_AND_RETURN(linearVelocity.isFinite(), "PxArticulationReducedCoordinate::setRootLinearVelocity velocity is not finite.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationReducedCoordinate::setRootLinearVelocity() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance(). Call will be ignored.");

	NpArticulationLink* root = mArticulationLinks[0];
	root->scSetLinearVelocity(linearVelocity);
	if(getNpScene())
	{
		const bool forceWakeup = !(linearVelocity.isZero());
		wakeUpInternal(forceWakeup, autowake);
	}
}
 
void NpArticulationReducedCoordinate::setRootAngularVelocity(const PxVec3& angularVelocity, bool autowake)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(!mArticulationLinks.empty(), "PxArticulationReducedCoordinate::setRootAngularVelocity() called on empty articulation.");
	PX_CHECK_AND_RETURN(angularVelocity.isFinite(), "PxArticulationReducedCoordinate::setRootAngularVelocity velocity is not finite.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationReducedCoordinate::setRootAngularVelocity() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance(). Call will be ignored.");

	NpArticulationLink* root = mArticulationLinks[0];
	root->scSetAngularVelocity(angularVelocity);
	if (getNpScene())
	{
		const bool forceWakeup = !(angularVelocity.isZero());
		wakeUpInternal(forceWakeup, autowake);
	}
}

PxVec3 NpArticulationReducedCoordinate::getRootLinearVelocity() const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(!mArticulationLinks.empty(), "PxArticulationReducedCoordinate::getRootLinearVelocity() called on empty articulation.", PxVec3(0.0f));

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getRootLinearVelocity() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", PxVec3(0.f));

	NpArticulationLink* root = mArticulationLinks[0];
	return root->getLinearVelocity();
}

PxVec3 NpArticulationReducedCoordinate::getRootAngularVelocity() const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(!mArticulationLinks.empty(), "PxArticulationReducedCoordinate::getRootAngularVelocity() called on empty articulation.", PxVec3(0.0f));

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getRootAngularVelocity() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", PxVec3(0.f));

	NpArticulationLink* root = mArticulationLinks[0];
	return root->getAngularVelocity();
}

PxSpatialVelocity NpArticulationReducedCoordinate::getLinkAcceleration(const PxU32 linkId)
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getLinkAcceleration: Articulation must be in a scene.", PxSpatialVelocity());
	PX_CHECK_AND_RETURN_VAL(linkId < 64, "PxArticulationReducedCoordinate::getLinkAcceleration index is not valid.", PxSpatialVelocity());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getLinkAcceleration() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", PxSpatialVelocity());

	const bool isGpuSimEnabled = (getNpScene()->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS) ? true : false;

	return mCore.getLinkAcceleration(linkId, isGpuSimEnabled);
}

PxU32 NpArticulationReducedCoordinate::getGpuArticulationIndex()
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getGpuArticulationIndex: Articulation must be in a scene.", 0xffffffff);
	
	if (getScene()->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)
		return mCore.getGpuArticulationIndex();
	return 0xffffffff;
}

PxArticulationSpatialTendon* NpArticulationReducedCoordinate::createSpatialTendon()
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
								"PxArticulationReducedCoordinate::createSpatialTendon() not allowed while the articulation is in a scene. Call will be ignored.");
		return NULL;
	}

	void* tendonMem = PX_ALLOC(sizeof(NpArticulationSpatialTendon), "NpArticulationSpatialTendon");
	PxMarkSerializedMemory(tendonMem, sizeof(NpArticulationSpatialTendon));
	NpArticulationSpatialTendon* tendon = PX_PLACEMENT_NEW(tendonMem, NpArticulationSpatialTendon)(this);

	tendon->setHandle(mSpatialTendons.size());
	mSpatialTendons.pushBack(tendon);
	return tendon;
}

void NpArticulationReducedCoordinate::removeSpatialTendonInternal(NpArticulationSpatialTendon* npTendon)
{
	//we don't need to remove low-level tendon from the articulation sim because the only case the tendon can be removed is
	//when the whole articulation is removed from the scene and the ArticulationSim get destroyed
	getNpScene()->scRemoveArticulationSpatialTendon(*npTendon);
}

PxArticulationFixedTendon* NpArticulationReducedCoordinate::createFixedTendon()
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationReducedCoordinate::createFixedTendon() not allowed while the articulation is in a scene. Call will be ignored.");
		return NULL;
	}

	void* tendonMem = PX_ALLOC(sizeof(NpArticulationFixedTendon), "NpArticulationFixedTendon");
	PxMarkSerializedMemory(tendonMem, sizeof(NpArticulationFixedTendon));
	NpArticulationFixedTendon* tendon = PX_PLACEMENT_NEW(tendonMem, NpArticulationFixedTendon)(this);

	tendon->setHandle(mFixedTendons.size());
	mFixedTendons.pushBack(tendon);
	return tendon;
}

void NpArticulationReducedCoordinate::removeFixedTendonInternal(NpArticulationFixedTendon* npTendon)
{
	//we don't need to remove low-level tendon from the articulation sim because the only case the tendon can be removed is
	//when the whole articulation is removed from the scene and the ArticulationSim get destroyed
	getNpScene()->scRemoveArticulationFixedTendon(*npTendon);
}

void NpArticulationReducedCoordinate::removeSensorInternal(NpArticulationSensor* npSensor)
{
	//we don't need to remove low-level sensor from the articulation sim because the only case the tendon can be removed is
	//when the whole articulation is removed from the scene and the ArticulationSim get destroyed
	getNpScene()->scRemoveArticulationSensor(*npSensor);
}

PxArticulationSensor* NpArticulationReducedCoordinate::createSensor(PxArticulationLink* link, const PxTransform& relativePose)
{
	if (getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationReducedCoordinate::createSensor() not allowed while the articulation is in a scene. Call will be ignored.");
		return NULL;
	}

	void* sensorMem = PX_ALLOC(sizeof(NpArticulationSensor), "NpArticulationSensor");
	PxMarkSerializedMemory(sensorMem, sizeof(NpArticulationSensor));
	NpArticulationSensor* sensor = PX_PLACEMENT_NEW(sensorMem, NpArticulationSensor)(link, relativePose);

	sensor->setHandle(mSensors.size());
	mSensors.pushBack(sensor);

	mTopologyChanged = true;
	return sensor;
}

void NpArticulationReducedCoordinate::releaseSensor(PxArticulationSensor& sensor)
{
	if (getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationReducedCoordinate::releaseSensor() not allowed while the articulation is in a scene. Call will be ignored.");
		return;
	}

	NpArticulationSensor* npSensor = static_cast<NpArticulationSensor*>(&sensor);

	const PxU32 handle = npSensor->getHandle();

	PX_CHECK_AND_RETURN(handle < mSensors.size() && mSensors[handle] == npSensor,
		"PxArticulationReducedCoordinate::releaseSensor: Attempt to release sensor that is not part of this articulation.");

	mSensors.back()->setHandle(handle);
	mSensors.replaceWithLast(handle);
	npSensor->~NpArticulationSensor();

	if (npSensor->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		PX_FREE(npSensor);

	mTopologyChanged = true;
}

PxU32 NpArticulationReducedCoordinate::getSensors(PxArticulationSensor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mSensors.begin(), mSensors.size());
}

PxU32 NpArticulationReducedCoordinate::getNbSensors()
{
	return mSensors.size();
}

NpArticulationSensor* NpArticulationReducedCoordinate::getSensor(const PxU32 index) const
{
	return mSensors[index];
}

PxU32 NpArticulationReducedCoordinate::getSpatialTendons(PxArticulationSpatialTendon** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mSpatialTendons.begin(), mSpatialTendons.size());
}

PxU32 NpArticulationReducedCoordinate::getNbSpatialTendons()
{
	return mSpatialTendons.size();
}

NpArticulationSpatialTendon* NpArticulationReducedCoordinate::getSpatialTendon(const PxU32 index) const
{
	return mSpatialTendons[index];
}

PxU32 NpArticulationReducedCoordinate::getFixedTendons(PxArticulationFixedTendon** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mFixedTendons.begin(), mFixedTendons.size());
}

PxU32 NpArticulationReducedCoordinate::getNbFixedTendons()
{
	return mFixedTendons.size();
}

NpArticulationFixedTendon* NpArticulationReducedCoordinate::getFixedTendon(const PxU32 index) const
{
	return mFixedTendons[index];
}

void NpArticulationReducedCoordinate::updateKinematic(PxArticulationKinematicFlags flags)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::updateKinematic: Articulation must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::updateKinematic() not allowed while simulation is running. Call will be ignored.");

	if(getNpScene())
	{
		mCore.updateKinematic(flags);

		const PxU32 linkCount = mArticulationLinks.size();

		//KS - the below code forces contact managers to be updated/cached data to be dropped and
		//shape transforms to be updated.
		for(PxU32 i = 0; i < linkCount; ++i)
		{
			NpArticulationLink* link = mArticulationLinks[i];
			//in the lowlevel articulation, we have already updated bodyCore's body2World
			const PxTransform internalPose = link->getCore().getBody2World();
			link->scSetBody2World(internalPose);
		}
	}
}

NpArticulationReducedCoordinate::~NpArticulationReducedCoordinate()
{
	//release tendons
	for (PxU32 i = 0; i < mSpatialTendons.size(); ++i)
	{
		if (mSpatialTendons[i])
		{
			mSpatialTendons[i]->~NpArticulationSpatialTendon();
			if(mSpatialTendons[i]->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
				PX_FREE(mSpatialTendons[i]);
		}
	}

	for (PxU32 i = 0; i < mFixedTendons.size(); ++i)
	{
		if (mFixedTendons[i])
		{
			mFixedTendons[i]->~NpArticulationFixedTendon();
			if(mFixedTendons[i]->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
				PX_FREE(mFixedTendons[i]);
		}
	}

	for (PxU32 i = 0; i < mSensors.size(); ++i)
	{
		if (mSensors[i])
		{
			mSensors[i]->~NpArticulationSensor();
			if(mSensors[i]->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
				PX_FREE(mSensors[i]);
		}
	}

	NpFactory::getInstance().onArticulationRelease(this);
}

PxArticulationJointReducedCoordinate* NpArticulationReducedCoordinate::createArticulationJoint(PxArticulationLink& parent,
	const PxTransform& parentFrame,
	PxArticulationLink& child,
	const PxTransform& childFrame)
{	
	return NpFactory::getInstance().createNpArticulationJointRC(static_cast<NpArticulationLink&>(parent), parentFrame, static_cast<NpArticulationLink&>(child), childFrame);
}

void NpArticulationReducedCoordinate::recomputeLinkIDs()
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::recomputeLinkIDs: Articulation must be in a scene.");

	if (!isAPIWriteForbidden())
	{
		Sc::ArticulationSim* scArtSim = getCore().getSim();

		if (scArtSim)
		{

			physx::NpArticulationLink*const* links = getLinks();

			const PxU32 nbLinks = getNbLinks();
			for (PxU32 i = 1; i < nbLinks; ++i)
			{
				physx::NpArticulationLink* link = links[i];
				PxU32 cHandle = scArtSim->findBodyIndex(*link->getCore().getSim());
				link->setLLIndex(cHandle);
			}
		}
	}
}

// PX_SERIALIZATION
void NpArticulationReducedCoordinate::requiresObjects(PxProcessPxBaseCallback& c)
{
	// Collect articulation links
	const PxU32 nbLinks = mArticulationLinks.size();
	for (PxU32 i = 0; i < nbLinks; i++)
		c.process(*mArticulationLinks[i]);

	const PxU32 nbSensors = mSensors.size();
	for (PxU32 i = 0; i < nbSensors; i++)
		c.process(*mSensors[i]);

	const PxU32 nbSpatialTendons = mSpatialTendons.size();
	for (PxU32 i = 0; i < nbSpatialTendons; i++)
		c.process(*mSpatialTendons[i]);

	const PxU32 nbFixedTendons = mFixedTendons.size();
	for (PxU32 i = 0; i < nbFixedTendons; i++)
		c.process(*mFixedTendons[i]);

}

void NpArticulationReducedCoordinate::exportExtraData(PxSerializationContext& stream)
{
	Cm::exportInlineArray(mArticulationLinks, stream);
	Cm::exportArray(mSpatialTendons, stream);
	Cm::exportArray(mFixedTendons, stream);
	Cm::exportArray(mSensors, stream);

	stream.writeName(mName);
}

void NpArticulationReducedCoordinate::importExtraData(PxDeserializationContext& context)
{
	Cm::importInlineArray(mArticulationLinks, context);
	Cm::importArray(mSpatialTendons, context);
	Cm::importArray(mFixedTendons, context);
	Cm::importArray(mSensors, context);

	context.readName(mName);
}

void NpArticulationReducedCoordinate::resolveReferences(PxDeserializationContext& context)
{
	const PxU32 nbLinks = mArticulationLinks.size();
	for (PxU32 i = 0; i < nbLinks; i++)
	{
		NpArticulationLink*& link = mArticulationLinks[i];
		context.translatePxBase(link);
	}

	const PxU32 nbSensors = mSensors.size();
	for (PxU32 i = 0; i < nbSensors; i++)
	{
		NpArticulationSensor*& sensor = mSensors[i];
		context.translatePxBase(sensor);
	}

	const PxU32 nbSpatialTendons = mSpatialTendons.size();
	for (PxU32 i = 0; i < nbSpatialTendons; i++)
	{
		NpArticulationSpatialTendon*& spatialTendon = mSpatialTendons[i];
		context.translatePxBase(spatialTendon);
	}

	const PxU32 nbFixedTendons = mFixedTendons.size();
	for (PxU32 i = 0; i < nbFixedTendons; i++)
	{
		NpArticulationFixedTendon*& fixedTendon = mFixedTendons[i];
		context.translatePxBase(fixedTendon);
	}

	mAggregate = NULL;
}
// ~PX_SERIALIZATION


void NpArticulationReducedCoordinate::release()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationReducedCoordinate::release() not allowed while simulation is running. Call will be ignored.");

	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationReducedCoordinate::userData);

	//!!!AL TODO: Order should not matter in this case. Optimize by having a path which does not restrict release to leaf links or
	//      by using a more advanced data structure
	PxU32 idx = 0;
	while (mArticulationLinks.size())
	{
		idx = idx % mArticulationLinks.size();

		if (mArticulationLinks[idx]->getNbChildren() == 0)
		{
			mArticulationLinks[idx]->releaseInternal();  // deletes joint, link and removes link from list
		}
		else
		{
			idx++;
		}
	}

	if (npScene)
	{
		npScene->removeArticulationTendons(*this);
		npScene->removeArticulationSensors(*this);
		npScene->scRemoveArticulation(*this);
		npScene->removeFromArticulationList(*this);
	}

	mArticulationLinks.clear();

	NpDestroyArticulation(this);
}


PxArticulationLink*	 NpArticulationReducedCoordinate::createLink(PxArticulationLink* parent, const PxTransform& pose)
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationReducedCoordinate::createLink() not allowed while the articulation is in a scene. Call will be ignored.");
		return NULL;
	}
	PX_CHECK_AND_RETURN_NULL(pose.isSane(), "PxArticulationReducedCoordinate::createLink: pose is not valid.");
	
	if (parent && mArticulationLinks.empty())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
								"PxArticulationReducedCoordinate::createLink: Root articulation link must have NULL parent pointer!");
		return NULL;
	}

	// Check if parent is in same articulation is done internally for checked builds
	if (!parent && !mArticulationLinks.empty())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationReducedCoordinate::createLink: Non-root articulation link must have valid parent pointer!");
		return NULL;
	}

	NpArticulationLink* parentLink = static_cast<NpArticulationLink*>(parent);

	NpArticulationLink* link = static_cast<NpArticulationLink*>(NpFactory::getInstance().createArticulationLink(*this, parentLink, pose.getNormalized()));

	if (link)
	{
		addToLinkList(*link);
		mTopologyChanged = true;
	}

	return link;
}

void NpArticulationReducedCoordinate::setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(positionIters > 0, "PxArticulationReducedCoordinate::setSolverIterationCount: positionIters must be more than zero!");
	PX_CHECK_AND_RETURN(positionIters <= 255, "PxArticulationReducedCoordinate::setSolverIterationCount: positionIters must be no greater than 255!");
	PX_CHECK_AND_RETURN(velocityIters <= 255, "PxArticulationReducedCoordinate::setSolverIterationCount: velocityIters must be no greater than 255!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.");

	scSetSolverIterationCounts((velocityIters & 0xff) << 8 | (positionIters & 0xff));

	OMNI_PVD_SET(PxArticulationReducedCoordinate, positionIterations, static_cast<const PxArticulationReducedCoordinate&>(*this), positionIters);
	OMNI_PVD_SET(PxArticulationReducedCoordinate, velocityIterations, static_cast<const PxArticulationReducedCoordinate&>(*this), velocityIters);
}

void NpArticulationReducedCoordinate::getSolverIterationCounts(PxU32& positionIters, PxU32& velocityIters) const
{
	NP_READ_CHECK(getNpScene());

	const PxU16 x = mCore.getSolverIterationCounts();
	velocityIters = PxU32(x >> 8);
	positionIters = PxU32(x & 0xff);
}

void NpArticulationReducedCoordinate::setGlobalPose()
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::setGlobalPose: Articulation must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setGlobalPose() not allowed while simulation is running. Call will be ignored.");

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.setGlobalPose();

	//This code is force PVD to update other links position
	{
		physx::NpArticulationLink*const* links = getLinks();

		const PxU32 nbLinks = getNbLinks();
		for (PxU32 i = 1; i < nbLinks; ++i)
		{
			physx::NpArticulationLink* link = links[i];
			//in the lowlevel articulation, we have already updated bodyCore's body2World
			const PxTransform internalPose = link->getCore().getBody2World();
			link->scSetBody2World(internalPose);
		}
	}
}

bool NpArticulationReducedCoordinate::isSleeping() const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::isSleeping: Articulation must be in a scene.", true);

	PX_CHECK_SCENE_API_READ_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::isSleeping() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().", true);

	return mCore.isSleeping();
}

void NpArticulationReducedCoordinate::setSleepThreshold(PxReal threshold)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setSleepThreshold() not allowed while simulation is running. Call will be ignored.");

	scSetSleepThreshold(threshold);

	OMNI_PVD_SET(PxArticulationReducedCoordinate, sleepThreshold, static_cast<const PxArticulationReducedCoordinate&>(*this), threshold);
}

PxReal NpArticulationReducedCoordinate::getSleepThreshold() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getSleepThreshold();
}

void NpArticulationReducedCoordinate::setStabilizationThreshold(PxReal threshold)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setStabilizationThreshold() not allowed while simulation is running. Call will be ignored.");

	scSetFreezeThreshold(threshold);

	OMNI_PVD_SET(PxArticulationReducedCoordinate, stabilizationThreshold, static_cast<const PxArticulationReducedCoordinate&>(*this), threshold);
}

PxReal NpArticulationReducedCoordinate::getStabilizationThreshold() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getFreezeThreshold();
}

void NpArticulationReducedCoordinate::setWakeCounter(PxReal wakeCounterValue)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationReducedCoordinate::setWakeCounter() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance(). Call will be ignored.");

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		mArticulationLinks[i]->scSetWakeCounter(wakeCounterValue);
	}

	scSetWakeCounter(wakeCounterValue);

	OMNI_PVD_SET(PxArticulationReducedCoordinate, wakeCounter, static_cast<const PxArticulationReducedCoordinate&>(*this), wakeCounterValue);
}

PxReal NpArticulationReducedCoordinate::getWakeCounter() const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getWakeCounter() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().", 0.0f);

	return mCore.getWakeCounter();
}

// follows D6 wakeup logic and is used for joint and tendon autowake
void NpArticulationReducedCoordinate::autoWakeInternal(void)
{
	PxReal wakeCounter = mCore.getWakeCounter();
	if (wakeCounter < getNpScene()->getWakeCounterResetValueInternal())
	{
		wakeCounter = getNpScene()->getWakeCounterResetValueInternal();
		for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
		{
			mArticulationLinks[i]->scWakeUpInternal(wakeCounter);
		}

		scWakeUpInternal(wakeCounter);
	}
}

// Follows RB wakeup logic. If autowake is true, increase wakeup counter to at least the scene reset valu
// If forceWakeUp is true, wakeup and leave wakeup counter unchanged, so that articulation goes to sleep
// again if wakecounter was zero at forceWakeup. The value of forceWakeup has no effect if autowake is true.
void NpArticulationReducedCoordinate::wakeUpInternal(bool forceWakeUp, bool autowake)
{
	PX_ASSERT(getNpScene());
	PxReal wakeCounterResetValue = getNpScene()->getWakeCounterResetValueInternal();

	PxReal wakeCounter = mCore.getWakeCounter();
	bool needsWakingUp = isSleeping() && (autowake || forceWakeUp);
	if (autowake && (wakeCounter < wakeCounterResetValue))
	{
		wakeCounter = wakeCounterResetValue;
		needsWakingUp = true;
	}

	if (needsWakingUp)
	{
		for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
		{
			mArticulationLinks[i]->scWakeUpInternal(wakeCounter);
		}

		scWakeUpInternal(wakeCounter);
	}
}

void NpArticulationReducedCoordinate::wakeUp()
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::wakeUp: Articulation must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationReducedCoordinate::wakeUp() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance(). Call will be ignored.");

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		mArticulationLinks[i]->scWakeUpInternal(getNpScene()->getWakeCounterResetValueInternal());
	}

	PX_ASSERT(getNpScene());  // only allowed for an object in a scene
	scWakeUpInternal(getNpScene()->getWakeCounterResetValueInternal());
}

void NpArticulationReducedCoordinate::putToSleep()
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationReducedCoordinate::putToSleep: Articulation must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::putToSleep() not allowed while simulation is running. Call will be ignored.");

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		mArticulationLinks[i]->scPutToSleepInternal();
	}

	PX_ASSERT(!isAPIWriteForbidden());
	mCore.putToSleep();
}

void NpArticulationReducedCoordinate::setMaxCOMLinearVelocity(const PxReal maxLinearVelocity)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setMaxCOMLinearVelocity() not allowed while simulation is running. Call will be ignored.");

	scSetMaxLinearVelocity(maxLinearVelocity);

	OMNI_PVD_SET(PxArticulationReducedCoordinate, maxLinearVelocity, static_cast<const PxArticulationReducedCoordinate&>(*this), maxLinearVelocity);
}

PxReal NpArticulationReducedCoordinate::getMaxCOMLinearVelocity() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getMaxLinearVelocity();
}

void NpArticulationReducedCoordinate::setMaxCOMAngularVelocity(const PxReal maxAngularVelocity)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationReducedCoordinate::setMaxCOMAngularVelocity() not allowed while simulation is running. Call will be ignored.");

	scSetMaxAngularVelocity(maxAngularVelocity);

	OMNI_PVD_SET(PxArticulationReducedCoordinate, maxAngularVelocity, static_cast<const PxArticulationReducedCoordinate&>(*this), maxAngularVelocity);
}

PxReal NpArticulationReducedCoordinate::getMaxCOMAngularVelocity() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getMaxAngularVelocity();
}

PxU32 NpArticulationReducedCoordinate::getNbLinks() const
{
	NP_READ_CHECK(getNpScene());
	return mArticulationLinks.size();
}

PxU32 NpArticulationReducedCoordinate::getLinks(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mArticulationLinks.begin(), mArticulationLinks.size());
}

PxU32 NpArticulationReducedCoordinate::getNbShapes() const
{
	NP_READ_CHECK(getNpScene());
	return mNumShapes;
}

PxBounds3 NpArticulationReducedCoordinate::getWorldBounds(float inflation) const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationReducedCoordinate::getWorldBounds() not allowed while simulation is running, except in a split simulation during PxScene::collide() and up to PxScene::advance().", PxBounds3::empty());

	PxBounds3 bounds = PxBounds3::empty();

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		bounds.include(mArticulationLinks[i]->getWorldBounds());
	}
	PX_ASSERT(bounds.isValid());

	// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
	const PxVec3 center = bounds.getCenter();
	const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
	return PxBounds3::centerExtents(center, inflatedExtents);
}

PxAggregate* NpArticulationReducedCoordinate::getAggregate() const
{
	NP_READ_CHECK(getNpScene());
	return mAggregate;
}

void NpArticulationReducedCoordinate::setName(const char* debugName)
{
	NP_WRITE_CHECK(getNpScene());
	mName = debugName;
}

const char* NpArticulationReducedCoordinate::getName() const
{
	NP_READ_CHECK(getNpScene());
	return mName;
}

NpArticulationLink* NpArticulationReducedCoordinate::getRoot()
{
	if (!mArticulationLinks.size())
		return NULL;

	PX_ASSERT(mArticulationLinks[0]->getInboundJoint() == NULL);
	return mArticulationLinks[0];
}

void NpArticulationReducedCoordinate::setAggregate(PxAggregate* a) 
{ 
	mAggregate = static_cast<NpAggregate*>(a); 
}


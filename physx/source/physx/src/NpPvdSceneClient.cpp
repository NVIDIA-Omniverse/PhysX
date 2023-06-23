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
#if PX_SUPPORT_PVD
#include "common/PxProfileZone.h"
#include "common/PxRenderBuffer.h"
#include "PxParticleSystem.h"
#include "PxPBDParticleSystem.h"
//#include "PxFLIPParticleSystem.h"
//#include "PxMPMParticleSystem.h"
#include "PxPhysics.h"
#include "PxConstraintDesc.h"
#include "NpPvdSceneClient.h"
#include "ScBodySim.h"
#include "ScConstraintSim.h"
#include "ScConstraintCore.h"
#include "PxsMaterialManager.h"
#include "PvdTypeNames.h"
#include "PxPvdUserRenderer.h"
#include "PxvNphaseImplementationContext.h"
#include "NpConstraint.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "NpSoftBody.h"
//#include "NpFEMCloth.h"
#include "NpHairSystem.h"
#include "NpAggregate.h"
#include "NpScene.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationReducedCoordinate.h"

using namespace physx;
using namespace physx::Vd;
using namespace physx::pvdsdk;

namespace
{
	PX_FORCE_INLINE	PxU64	getContextId(NpScene& scene) { return scene.getScScene().getContextId(); }

	///////////////////////////////////////////////////////////////////////////////

	// Sc-to-Np
	PX_FORCE_INLINE static NpConstraint* getNpConstraint(Sc::ConstraintCore* scConstraint)
	{
		return reinterpret_cast<NpConstraint*>(reinterpret_cast<char*>(scConstraint) - NpConstraint::getCoreOffset());
	}

	///////////////////////////////////////////////////////////////////////////////

	PX_FORCE_INLINE static const PxActor* getPxActor(const NpActor* scActor)
	{
		return scActor->getPxActor();
	}

	struct CreateOp
	{
		CreateOp& operator=(const CreateOp&);
		physx::pvdsdk::PvdDataStream& mStream;
		PvdMetaDataBinding& mBinding;
		PsPvd* mPvd;
		PxScene& mScene;
		CreateOp(physx::pvdsdk::PvdDataStream& str, PvdMetaDataBinding& bind, PsPvd* pvd, PxScene& scene)
			: mStream(str), mBinding(bind), mPvd(pvd), mScene(scene)
		{
		}
		template <typename TDataType>
		void operator()(const TDataType& dtype)
		{
			mBinding.createInstance(mStream, dtype, mScene, PxGetPhysics(), mPvd);
		}
		void operator()(const PxArticulationLink&)
		{
		}
	};

	struct UpdateOp
	{
		UpdateOp& operator=(const UpdateOp&);
		physx::pvdsdk::PvdDataStream& mStream;
		PvdMetaDataBinding& mBinding;
		UpdateOp(physx::pvdsdk::PvdDataStream& str, PvdMetaDataBinding& bind) : mStream(str), mBinding(bind)
		{
		}
		template <typename TDataType>
		void operator()(const TDataType& dtype)
		{
			mBinding.sendAllProperties(mStream, dtype);
		}
	};

	struct DestroyOp
	{
		DestroyOp& operator=(const DestroyOp&);
		physx::pvdsdk::PvdDataStream& mStream;
		PvdMetaDataBinding& mBinding;
		PxScene& mScene;
		DestroyOp(physx::pvdsdk::PvdDataStream& str, PvdMetaDataBinding& bind, PxScene& scene)
			: mStream(str), mBinding(bind), mScene(scene)
		{
		}
		template <typename TDataType>
		void operator()(const TDataType& dtype)
		{
			mBinding.destroyInstance(mStream, dtype, mScene);
		}
		void operator()(const PxArticulationLink& dtype)
		{
			mBinding.destroyInstance(mStream, dtype);
		}
	};

	template <typename TOperator>
	inline void BodyTypeOperation(const NpActor* scBody, TOperator op)
	{
//		const bool isArticulationLink = scBody->getActorType_() == PxActorType::eARTICULATION_LINK;
		const bool isArticulationLink = scBody->getNpType() == NpType::eBODY_FROM_ARTICULATION_LINK;
		if(isArticulationLink)
		{
			const NpArticulationLink* link = static_cast<const NpArticulationLink*>(scBody);
			op(*static_cast<const PxArticulationLink*>(link));
		}
		else
		{
			const NpRigidDynamic* npRigidDynamic = static_cast<const NpRigidDynamic*>(scBody);
			op(*static_cast<const PxRigidDynamic*>(npRigidDynamic));
		}
	}

	template <typename TOperator>
	inline void ActorTypeOperation(const PxActor* actor, TOperator op)
	{
		switch(actor->getType())
		{
		case PxActorType::eRIGID_STATIC:
			op(*static_cast<const PxRigidStatic*>(actor));
			break;
		case PxActorType::eRIGID_DYNAMIC:
			op(*static_cast<const PxRigidDynamic*>(actor));
			break;
		case PxActorType::eARTICULATION_LINK:
			op(*static_cast<const PxArticulationLink*>(actor));
			break;
		case PxActorType::eSOFTBODY:
#if PX_SUPPORT_GPU_PHYSX
			op(*static_cast<const PxSoftBody*>(actor));
#endif
			break;
		case PxActorType::eFEMCLOTH:
			//op(*static_cast<const PxFEMCloth*>(actor));
			break;
		case PxActorType::ePBD_PARTICLESYSTEM:
			op(*static_cast<const PxPBDParticleSystem*>(actor));
			break;
		case PxActorType::eFLIP_PARTICLESYSTEM:
			//op(*static_cast<const PxFLIPParticleSystem*>(actor));
			break;
		case PxActorType::eMPM_PARTICLESYSTEM:
			//op(*static_cast<const PxMPMParticleSystem*>(actor));
			break;
		case PxActorType::eHAIRSYSTEM:
			//op(*static_cast<const PxHairSystem*>(actor));
			break;
		case PxActorType::eACTOR_COUNT:
		case PxActorType::eACTOR_FORCE_DWORD:
			PX_ASSERT(false);
			break;
		};
	}

	namespace
	{
		struct PvdConstraintVisualizer : public PxConstraintVisualizer
		{
			PX_NOCOPY(PvdConstraintVisualizer)
		public:
			physx::pvdsdk::PvdUserRenderer& mRenderer;

			PvdConstraintVisualizer(const void* id, physx::pvdsdk::PvdUserRenderer& r) : mRenderer(r)
			{
				mRenderer.setInstanceId(id);
			}

			virtual void visualizeJointFrames(const PxTransform& parent, const PxTransform& child)	PX_OVERRIDE
			{
				mRenderer.visualizeJointFrames(parent, child);
			}

			virtual void visualizeLinearLimit(const PxTransform& t0, const PxTransform& t1, PxReal value)	PX_OVERRIDE
			{
				mRenderer.visualizeLinearLimit(t0, t1, PxF32(value));
			}

			virtual void visualizeAngularLimit(const PxTransform& t0, PxReal lower, PxReal upper)	PX_OVERRIDE
			{
				mRenderer.visualizeAngularLimit(t0, PxF32(lower), PxF32(upper));
			}

			virtual void visualizeLimitCone(const PxTransform& t, PxReal tanQSwingY, PxReal tanQSwingZ)	PX_OVERRIDE
			{
				mRenderer.visualizeLimitCone(t, PxF32(tanQSwingY), PxF32(tanQSwingZ));
			}

			virtual void visualizeDoubleCone(const PxTransform& t, PxReal angle)	PX_OVERRIDE
			{
				mRenderer.visualizeDoubleCone(t, PxF32(angle));
			}

			virtual void visualizeLine( const PxVec3& p0, const PxVec3& p1, PxU32 color)	PX_OVERRIDE
			{
				const PxDebugLine line(p0, p1, color);
				mRenderer.drawLines(&line, 1);
			}
		};
	}

	class SceneRendererClient : public RendererEventClient, public physx::PxUserAllocated
	{
		PX_NOCOPY(SceneRendererClient)
	public:
		SceneRendererClient(PvdUserRenderer* renderer, PxPvd* pvd):mRenderer(renderer)
		{
			mStream = PvdDataStream::create(pvd); 
			mStream->createInstance(renderer);
		}

		~SceneRendererClient()
		{
			mStream->destroyInstance(mRenderer);
			mStream->release();
		}

		virtual void handleBufferFlush(const uint8_t* inData, uint32_t inLength)
		{
			mStream->setPropertyValue(mRenderer, "events", inData, inLength);
		}

	private:

		PvdUserRenderer* mRenderer;
		PvdDataStream* mStream;
	};

} // namespace

PvdSceneClient::PvdSceneClient(NpScene& scene) :
	mPvd			(NULL),
	mScene			(scene),
	mPvdDataStream	(NULL),
	mUserRender		(NULL),
	mRenderClient	(NULL),
	mIsConnected	(false)
{
}

PvdSceneClient::~PvdSceneClient()
{
	if(mPvd)
		mPvd->removeClient(this);
}

void PvdSceneClient::updateCamera(const char* name, const PxVec3& origin, const PxVec3& up, const PxVec3& target)
{
	if(mIsConnected)
		mPvdDataStream->updateCamera(name, origin, up, target);
}

void PvdSceneClient::drawPoints(const PxDebugPoint* points, PxU32 count)
{
	if(mUserRender)
		mUserRender->drawPoints(points, count);
}

void PvdSceneClient::drawLines(const PxDebugLine* lines, PxU32 count)
{
	if(mUserRender)
		mUserRender->drawLines(lines, count);
}

void PvdSceneClient::drawTriangles(const PxDebugTriangle* triangles, PxU32 count)
{
	if(mUserRender)
		mUserRender->drawTriangles(triangles, count);
}

void PvdSceneClient::drawText(const PxDebugText& text)
{
	if(mUserRender)
		mUserRender->drawText(text);
}

void PvdSceneClient::setScenePvdFlag(PxPvdSceneFlag::Enum flag, bool value)
{
	if(value)
		mFlags |= flag;
	else
		mFlags &= ~flag;
}

void PvdSceneClient::onPvdConnected()
{
	if(mIsConnected || !mPvd)
		return;

	mIsConnected = true;

	mPvdDataStream = PvdDataStream::create(mPvd);

	mUserRender = PvdUserRenderer::create();
	mRenderClient = PX_NEW(SceneRendererClient)(mUserRender, mPvd);	
	mUserRender->setClient(mRenderClient);

	sendEntireScene();
}

void PvdSceneClient::onPvdDisconnected()
{
	if(!mIsConnected)
		return;
	mIsConnected = false;

	PX_DELETE(mRenderClient);
	mUserRender->release();
	mUserRender = NULL;
	mPvdDataStream->release();
	mPvdDataStream = NULL;
}

void PvdSceneClient::updatePvdProperties()
{
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, mScene);
}

void PvdSceneClient::releasePvdInstance()
{
	if(mPvdDataStream)
	{		
		PxScene* theScene = &mScene;
		// remove from parent	
		mPvdDataStream->removeObjectRef(&PxGetPhysics(), "Scenes", theScene);
		mPvdDataStream->destroyInstance(theScene);
	}
}

// PT: this is only called once, from "onPvdConnected"
void PvdSceneClient::sendEntireScene()
{
	NpScene* npScene = &mScene;

	if(npScene->getFlagsFast() & PxSceneFlag::eREQUIRE_RW_LOCK) // getFlagsFast() will trigger a warning of lock check
		npScene->lockRead(__FILE__, __LINE__);

	PxPhysics& physics = PxGetPhysics();
	{
		PxScene* theScene = &mScene;
		mPvdDataStream->createInstance(theScene);
		updatePvdProperties();

		// Create parent/child relationship.
		mPvdDataStream->setPropertyValue(theScene, "Physics", reinterpret_cast<const void*>(&physics));
		mPvdDataStream->pushBackObjectRef(&physics, "Scenes", theScene);
	}

	// materials:
	{
		PxsMaterialManager& manager = mScene.getScScene().getMaterialManager();
		PxsMaterialManagerIterator<PxsMaterialCore> iter(manager);
		PxsMaterialCore* mat;
		while(iter.getNextMaterial(mat))
		{
			const PxMaterial* theMaterial = mat->mMaterial;
			if(mPvd->registerObject(theMaterial))
				mMetaDataBinding.createInstance(*mPvdDataStream, *theMaterial, physics);
		};
	}

	if(mPvd->getInstrumentationFlags() & PxPvdInstrumentationFlag::eDEBUG)
	{
		PxArray<PxActor*> actorArray;

		// RBs
		// static:
		{
			PxU32 numActors = npScene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC);
			actorArray.resize(numActors);
			npScene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, actorArray.begin(), actorArray.size());
			for(PxU32 i = 0; i < numActors; i++)
			{
				PxActor* pxActor = actorArray[i];
				if(pxActor->getConcreteType()==PxConcreteType::eRIGID_STATIC)
					mMetaDataBinding.createInstance(*mPvdDataStream, *static_cast<PxRigidStatic*>(pxActor), *npScene, physics, mPvd);
				else
					mMetaDataBinding.createInstance(*mPvdDataStream, *static_cast<PxRigidDynamic*>(pxActor), *npScene, physics, mPvd);
			}
		}
		// articulations & links
		{
			PxArray<PxArticulationReducedCoordinate*> articulations;
			PxU32 numArticulations = npScene->getNbArticulations();
			articulations.resize(numArticulations);
			npScene->getArticulations(articulations.begin(), articulations.size());
			for(PxU32 i = 0; i < numArticulations; i++)
				mMetaDataBinding.createInstance(*mPvdDataStream, *articulations[i], *npScene, physics, mPvd);
		}

		// joints
		{
			Sc::ConstraintCore*const * constraints = mScene.getScScene().getConstraints();
			PxU32 nbConstraints = mScene.getScScene().getNbConstraints();
			for(PxU32 i = 0; i < nbConstraints; i++)
			{
				updateConstraint(*constraints[i], PxPvdUpdateType::CREATE_INSTANCE);
				updateConstraint(*constraints[i], PxPvdUpdateType::UPDATE_ALL_PROPERTIES);
			}
		}
	}

	if(npScene->getFlagsFast() & PxSceneFlag::eREQUIRE_RW_LOCK)
		npScene->unlockRead();
}

void PvdSceneClient::updateConstraint(const Sc::ConstraintCore& scConstraint, PxU32 updateType)
{
	PxConstraintConnector* conn = scConstraint.getPxConnector();
	if(conn && checkPvdDebugFlag())	
		conn->updatePvdProperties(*mPvdDataStream, scConstraint.getPxConstraint(), PxPvdUpdateType::Enum(updateType));
}

void PvdSceneClient::createPvdInstance(const PxActor* actor)
{
	if(checkPvdDebugFlag())	
		ActorTypeOperation(actor, CreateOp(*mPvdDataStream, mMetaDataBinding, mPvd, mScene));
}

void PvdSceneClient::updatePvdProperties(const PxActor* actor)
{
	if(checkPvdDebugFlag())	
		ActorTypeOperation(actor, UpdateOp(*mPvdDataStream, mMetaDataBinding));
}

void PvdSceneClient::releasePvdInstance(const PxActor* actor)
{
	if(checkPvdDebugFlag())	
		ActorTypeOperation(actor, DestroyOp(*mPvdDataStream, mMetaDataBinding, mScene));
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpActor* actor)
{
	// PT: why not UPDATE_PVD_PROPERTIES_CHECK() here?
	createPvdInstance(getPxActor(actor));
}

void PvdSceneClient::updatePvdProperties(const NpActor* actor)
{
	// PT: why not UPDATE_PVD_PROPERTIES_CHECK() here?
	updatePvdProperties(getPxActor(actor));
}

void PvdSceneClient::releasePvdInstance(const NpActor* actor)
{
	// PT: why not UPDATE_PVD_PROPERTIES_CHECK() here?
	releasePvdInstance(getPxActor(actor));
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpRigidDynamic* body)
{
	if(checkPvdDebugFlag() && body->getNpType() != NpType::eBODY_FROM_ARTICULATION_LINK)
		BodyTypeOperation(body, CreateOp(*mPvdDataStream, mMetaDataBinding, mPvd, mScene));
}

void PvdSceneClient::createPvdInstance(const NpArticulationLink* body)
{
	if(checkPvdDebugFlag() && body->getNpType() != NpType::eBODY_FROM_ARTICULATION_LINK)
		BodyTypeOperation(body, CreateOp(*mPvdDataStream, mMetaDataBinding, mPvd, mScene));
}

void PvdSceneClient::releasePvdInstance(const NpRigidDynamic* body)
{
	releasePvdInstance(getPxActor(body));
}

void PvdSceneClient::releasePvdInstance(const NpArticulationLink* body)
{
	releasePvdInstance(getPxActor(body));
}

void PvdSceneClient::updateBodyPvdProperties(const NpActor* body)
{
	if(checkPvdDebugFlag())
	{
		if(body->getNpType() == NpType::eBODY_FROM_ARTICULATION_LINK)
			updatePvdProperties(static_cast<const NpArticulationLink*>(body));
		else if(body->getNpType() == NpType::eBODY)
			updatePvdProperties(static_cast<const NpRigidDynamic*>(body));
		else PX_ASSERT(0);
	}
}

void PvdSceneClient::updatePvdProperties(const NpRigidDynamic* body)
{
	if(checkPvdDebugFlag())	
		BodyTypeOperation(body, UpdateOp(*mPvdDataStream, mMetaDataBinding));
}

void PvdSceneClient::updatePvdProperties(const NpArticulationLink* body)
{
	if(checkPvdDebugFlag())	
		BodyTypeOperation(body, UpdateOp(*mPvdDataStream, mMetaDataBinding));
}

void PvdSceneClient::updateKinematicTarget(const NpActor* body, const PxTransform& p)
{
	if(checkPvdDebugFlag())
	{
		if(body->getNpType() == NpType::eBODY_FROM_ARTICULATION_LINK)
			mPvdDataStream->setPropertyValue(static_cast<const NpArticulationLink*>(body), "KinematicTarget", p);
		else if(body->getNpType() == NpType::eBODY)
			mPvdDataStream->setPropertyValue(static_cast<const NpRigidDynamic*>(body), "KinematicTarget", p);
		else PX_ASSERT(0);
	}
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::releasePvdInstance(const NpRigidStatic* rigidStatic)
{
	releasePvdInstance(static_cast<const NpActor*>(rigidStatic));
}

void PvdSceneClient::createPvdInstance(const NpRigidStatic* rigidStatic)
{	
	if(checkPvdDebugFlag())	
		mMetaDataBinding.createInstance(*mPvdDataStream, *rigidStatic, mScene, PxGetPhysics(), mPvd);
}

void PvdSceneClient::updatePvdProperties(const NpRigidStatic* rigidStatic)
{
	if(checkPvdDebugFlag())	
		mMetaDataBinding.sendAllProperties(*mPvdDataStream, *rigidStatic);
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpConstraint* constraint)
{
	if(checkPvdDebugFlag())	
		updateConstraint(constraint->getCore(), PxPvdUpdateType::CREATE_INSTANCE);
}

void PvdSceneClient::updatePvdProperties(const NpConstraint* constraint)
{
	if(checkPvdDebugFlag())	
		updateConstraint(constraint->getCore(), PxPvdUpdateType::UPDATE_ALL_PROPERTIES);
}

void PvdSceneClient::releasePvdInstance(const NpConstraint* constraint)
{	
	const Sc::ConstraintCore& scConstraint = constraint->getCore();
	PxConstraintConnector* conn;
	if(checkPvdDebugFlag() && (conn = scConstraint.getPxConnector()) != NULL)
		conn->updatePvdProperties(*mPvdDataStream, scConstraint.getPxConstraint(), PxPvdUpdateType::RELEASE_INSTANCE);
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpArticulationReducedCoordinate* articulation)
{
	if (checkPvdDebugFlag())
	{
		mMetaDataBinding.createInstance(*mPvdDataStream, *articulation, mScene, PxGetPhysics(), mPvd);
	}
}

void PvdSceneClient::updatePvdProperties(const NpArticulationReducedCoordinate* articulation)
{
	if(checkPvdDebugFlag())
	{
		mMetaDataBinding.sendAllProperties(*mPvdDataStream, *articulation);
	}
}

void PvdSceneClient::releasePvdInstance(const NpArticulationReducedCoordinate* articulation)
{
	if (checkPvdDebugFlag())
	{
		mMetaDataBinding.destroyInstance(*mPvdDataStream, *articulation, mScene);
	}
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpArticulationJointReducedCoordinate* articulationJoint)
{
	PX_UNUSED(articulationJoint);
}

void PvdSceneClient::updatePvdProperties(const NpArticulationJointReducedCoordinate* articulationJoint)
{
	if (checkPvdDebugFlag())
	{
		mMetaDataBinding.sendAllProperties(*mPvdDataStream, *articulationJoint);
	}
}

void PvdSceneClient::releasePvdInstance(const NpArticulationJointReducedCoordinate* articulationJoint)
{
	PX_UNUSED(articulationJoint);
}
/////////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpArticulationSpatialTendon* articulationTendon)
{
	PX_UNUSED(articulationTendon);
}

void PvdSceneClient::updatePvdProperties(const NpArticulationSpatialTendon* articulationTendon)
{
	PX_UNUSED(articulationTendon);
}

void PvdSceneClient::releasePvdInstance(const NpArticulationSpatialTendon* articulationTendon)
{
	PX_UNUSED(articulationTendon);
}

/////////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpArticulationFixedTendon* articulationTendon)
{
	PX_UNUSED(articulationTendon);
}

void PvdSceneClient::updatePvdProperties(const NpArticulationFixedTendon* articulationTendon)
{
	PX_UNUSED(articulationTendon);
}

void PvdSceneClient::releasePvdInstance(const NpArticulationFixedTendon* articulationTendon)
{
	PX_UNUSED(articulationTendon);
}

/////////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpArticulationSensor* sensor)
{
	PX_UNUSED(sensor);
}

void PvdSceneClient::updatePvdProperties(const NpArticulationSensor* sensor)
{
	PX_UNUSED(sensor);
}

void PvdSceneClient::releasePvdInstance(const NpArticulationSensor* sensor)
{
	PX_UNUSED(sensor);
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const PxsMaterialCore* materialCore)
{	
	if(checkPvdDebugFlag())	
	{
		const PxMaterial* theMaterial = materialCore->mMaterial;
		if(mPvd->registerObject(theMaterial))
			mMetaDataBinding.createInstance(*mPvdDataStream, *theMaterial, PxGetPhysics());
	}
}

void PvdSceneClient::updatePvdProperties(const PxsMaterialCore* materialCore)
{
	if(checkPvdDebugFlag())	
		mMetaDataBinding.sendAllProperties(*mPvdDataStream, *materialCore->mMaterial);
}

void PvdSceneClient::releasePvdInstance(const PxsMaterialCore* materialCore)
{
	if(checkPvdDebugFlag() && mPvd->unRegisterObject(materialCore->mMaterial))
		mMetaDataBinding.destroyInstance(*mPvdDataStream, *materialCore->mMaterial, PxGetPhysics());
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const PxsFEMSoftBodyMaterialCore* materialCore)
{
	if (checkPvdDebugFlag())
	{
		const PxFEMSoftBodyMaterial* theMaterial = materialCore->mMaterial;
		if (mPvd->registerObject(theMaterial))
			mMetaDataBinding.createInstance(*mPvdDataStream, *theMaterial, PxGetPhysics());
	}
}

void PvdSceneClient::updatePvdProperties(const PxsFEMSoftBodyMaterialCore* materialCore)
{
	if (checkPvdDebugFlag())
		mMetaDataBinding.sendAllProperties(*mPvdDataStream, *materialCore->mMaterial);
}

void PvdSceneClient::releasePvdInstance(const PxsFEMSoftBodyMaterialCore* materialCore)
{
	if (checkPvdDebugFlag() && mPvd->unRegisterObject(materialCore->mMaterial))
		mMetaDataBinding.destroyInstance(*mPvdDataStream, *materialCore->mMaterial, PxGetPhysics());
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const PxsFEMClothMaterialCore*)
{
	// no PVD support but method is "needed" since macro code is shared among all material types
}

void PvdSceneClient::updatePvdProperties(const PxsFEMClothMaterialCore*)
{
	// no PVD support but method is "needed" since macro code is shared among all material types
}

void PvdSceneClient::releasePvdInstance(const PxsFEMClothMaterialCore*)
{
	// no PVD support but method is "needed" since macro code is shared among all material types
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const PxsPBDMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::updatePvdProperties(const PxsPBDMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::releasePvdInstance(const PxsPBDMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::createPvdInstance(const PxsFLIPMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::updatePvdProperties(const PxsFLIPMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::releasePvdInstance(const PxsFLIPMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::createPvdInstance(const PxsMPMMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::updatePvdProperties(const PxsMPMMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

void PvdSceneClient::releasePvdInstance(const PxsMPMMaterialCore* /*materialCore*/)
{
//	PX_ASSERT(0);
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpShape* npShape, PxActor& owner)
{
	if(checkPvdDebugFlag())
	{
		PX_PROFILE_ZONE("PVD.createPVDInstance", getContextId(mScene));
		mMetaDataBinding.createInstance(*mPvdDataStream, *npShape, static_cast<PxRigidActor&>(owner), PxGetPhysics(), mPvd);
	}
}

static void addShapesToPvd(PxU32 nbShapes, NpShape* const* shapes, PxActor& pxActor, PsPvd* pvd, PvdDataStream& stream, PvdMetaDataBinding& binding)
{
	PxPhysics& physics = PxGetPhysics();
	for(PxU32 i=0;i<nbShapes;i++)
	{
		const NpShape* npShape = shapes[i];
		binding.createInstance(stream, *npShape, static_cast<PxRigidActor&>(pxActor), physics, pvd);
	}
}

void PvdSceneClient::addBodyAndShapesToPvd(NpRigidDynamic& b)
{
	if(checkPvdDebugFlag())
	{
		PX_PROFILE_ZONE("PVD.createPVDInstance", getContextId(mScene));
		createPvdInstance(&b);

		PxActor& pxActor = *b.getCore().getPxActor();
		NpShape* const* shapes;
		const PxU32 nbShapes = NpRigidDynamicGetShapes(b, shapes);
		addShapesToPvd(nbShapes, shapes, pxActor, mPvd, *mPvdDataStream, mMetaDataBinding);
	}
}

void PvdSceneClient::addStaticAndShapesToPvd(NpRigidStatic& s)
{
	if(checkPvdDebugFlag())
	{
		PX_PROFILE_ZONE("PVD.createPVDInstance", getContextId(mScene));
		createPvdInstance(&s);

		PxActor& pxActor = static_cast<PxRigidStatic&>(s);
		NpShape* const* shapes;
		const PxU32 nbShapes = NpRigidStaticGetShapes(s, shapes);
		addShapesToPvd(nbShapes, shapes, pxActor, mPvd, *mPvdDataStream, mMetaDataBinding);
	}
}

void PvdSceneClient::updateMaterials(const NpShape* npShape)
{
	if(checkPvdDebugFlag())
		mMetaDataBinding.updateMaterials(*mPvdDataStream, *npShape, mPvd);
}

void PvdSceneClient::updatePvdProperties(const NpShape* npShape)
{
	if(checkPvdDebugFlag())
		mMetaDataBinding.sendAllProperties(*mPvdDataStream, *npShape);
}

void PvdSceneClient::releaseAndRecreateGeometry(const NpShape* npShape)
{
	if(checkPvdDebugFlag())
		mMetaDataBinding.releaseAndRecreateGeometry(*mPvdDataStream, *npShape, NpPhysics::getInstance(), mPvd);
}

void PvdSceneClient::releasePvdInstance(const NpShape* npShape, PxActor& owner)
{
	if(checkPvdDebugFlag())
	{
		PX_PROFILE_ZONE("PVD.releasePVDInstance", getContextId(mScene));

		mMetaDataBinding.destroyInstance(*mPvdDataStream, *npShape, static_cast<PxRigidActor&>(owner));

		const PxU32 numMaterials = npShape->getNbMaterials();
		PX_ALLOCA(materialPtr, PxMaterial*, numMaterials);
		npShape->getMaterials(materialPtr, numMaterials);

		for(PxU32 idx = 0; idx < numMaterials; ++idx)
			releasePvdInstance(&(static_cast<NpMaterial*>(materialPtr[idx])->mMaterial));
	}
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::originShift(PxVec3 shift)
{
	mMetaDataBinding.originShift(*mPvdDataStream, &mScene, shift);
}

void PvdSceneClient::frameStart(PxReal simulateElapsedTime)
{	
	PX_PROFILE_ZONE("Basic.pvdFrameStart", mScene.getScScene().getContextId());

	if(!mIsConnected)
		return;

	mPvdDataStream->flushPvdCommand();
	mMetaDataBinding.sendBeginFrame(*mPvdDataStream, &mScene, simulateElapsedTime);
}

void PvdSceneClient::frameEnd()
{
	PX_PROFILE_ZONE("Basic.pvdFrameEnd", mScene.getScScene().getContextId());

	if(!mIsConnected)
	{
		if(mPvd)
			mPvd->flush(); // Even if we aren't connected, we may need to flush buffered events.
		return;
	}

	PxScene* theScene = &mScene;
	
	mMetaDataBinding.sendStats(*mPvdDataStream, theScene);

	// flush our data to the main connection
	mPvd->flush();

	// End the frame *before* we send the dynamic object current data.
	// This ensures that contacts end up synced with the rest of the system.
	// Note that contacts were sent much earler in NpScene::fetchResults.
	mMetaDataBinding.sendEndFrame(*mPvdDataStream, &mScene);

	if(mPvd->getInstrumentationFlags() & PxPvdInstrumentationFlag::eDEBUG)
	{
		PX_PROFILE_ZONE("PVD.sceneUpdate", getContextId(mScene));

		PvdVisualizer* vizualizer = NULL;
		const bool visualizeJoints = getScenePvdFlagsFast() & PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS;
		if(visualizeJoints)
			vizualizer = this;

		mMetaDataBinding.updateDynamicActorsAndArticulations(*mPvdDataStream, theScene, vizualizer);
	}

	// frame end moved to update contacts to have them in the previous frame.
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpAggregate* npAggregate)
{
	if(checkPvdDebugFlag())
	{
		PX_PROFILE_ZONE("PVD.createPVDInstance", getContextId(mScene));
		mMetaDataBinding.createInstance(*mPvdDataStream, *npAggregate, mScene);
	}
}

void PvdSceneClient::updatePvdProperties(const NpAggregate* npAggregate)
{
	if(checkPvdDebugFlag())
		mMetaDataBinding.sendAllProperties(*mPvdDataStream, *npAggregate);
}

void PvdSceneClient::attachAggregateActor(const NpAggregate* npAggregate, NpActor* actor)
{
	if(checkPvdDebugFlag())
		mMetaDataBinding.attachAggregateActor(*mPvdDataStream, *npAggregate, *getPxActor(actor));
}

void PvdSceneClient::detachAggregateActor(const NpAggregate* npAggregate, NpActor* actor)
{
	if(checkPvdDebugFlag())
		mMetaDataBinding.detachAggregateActor(*mPvdDataStream, *npAggregate, *getPxActor(actor));
}

void PvdSceneClient::releasePvdInstance(const NpAggregate* npAggregate)
{
	if(checkPvdDebugFlag())
	{
		PX_PROFILE_ZONE("PVD.releasePVDInstance", getContextId(mScene));
		mMetaDataBinding.destroyInstance(*mPvdDataStream, *npAggregate, mScene);
	}
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_GPU_PHYSX
void PvdSceneClient::createPvdInstance(const NpSoftBody* softBody)
{
	PX_UNUSED(softBody);
	//Todo
}

void PvdSceneClient::updatePvdProperties(const NpSoftBody* softBody)
{
	PX_UNUSED(softBody);
	//Todo
}

void PvdSceneClient::attachAggregateActor(const NpSoftBody* softBody, NpActor* actor)
{
	PX_UNUSED(softBody);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::detachAggregateActor(const NpSoftBody* softBody, NpActor* actor)
{
	PX_UNUSED(softBody);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::releasePvdInstance(const NpSoftBody* softBody)
{
	PX_UNUSED(softBody);
	//Todo
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpFEMCloth* femCloth)
{
	PX_UNUSED(femCloth);
	//Todo
}

void PvdSceneClient::updatePvdProperties(const NpFEMCloth* femCloth)
{
	PX_UNUSED(femCloth);
	//Todo
}

void PvdSceneClient::attachAggregateActor(const NpFEMCloth* femCloth, NpActor* actor)
{
	PX_UNUSED(femCloth);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::detachAggregateActor(const NpFEMCloth* femCloth, NpActor* actor)
{
	PX_UNUSED(femCloth);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::releasePvdInstance(const NpFEMCloth* femCloth)
{
	PX_UNUSED(femCloth);
	//Todo
}

///////////////////////////////////////////////////////////////////////////////
void PvdSceneClient::createPvdInstance(const NpPBDParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

void PvdSceneClient::updatePvdProperties(const NpPBDParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

void PvdSceneClient::attachAggregateActor(const NpPBDParticleSystem* particleSystem, NpActor* actor)
{
	PX_UNUSED(particleSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::detachAggregateActor(const NpPBDParticleSystem* particleSystem, NpActor* actor)
{
	PX_UNUSED(particleSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::releasePvdInstance(const NpPBDParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

///////////////////////////////////////////////////////////////////////////////
void PvdSceneClient::createPvdInstance(const NpFLIPParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

void PvdSceneClient::updatePvdProperties(const NpFLIPParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

void PvdSceneClient::attachAggregateActor(const NpFLIPParticleSystem* particleSystem, NpActor* actor)
{
	PX_UNUSED(particleSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::detachAggregateActor(const NpFLIPParticleSystem* particleSystem, NpActor* actor)
{
	PX_UNUSED(particleSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::releasePvdInstance(const NpFLIPParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

///////////////////////////////////////////////////////////////////////////////
void PvdSceneClient::createPvdInstance(const NpMPMParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

void PvdSceneClient::updatePvdProperties(const NpMPMParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

void PvdSceneClient::attachAggregateActor(const NpMPMParticleSystem* particleSystem, NpActor* actor)
{
	PX_UNUSED(particleSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::detachAggregateActor(const NpMPMParticleSystem* particleSystem, NpActor* actor)
{
	PX_UNUSED(particleSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::releasePvdInstance(const NpMPMParticleSystem* particleSystem)
{
	PX_UNUSED(particleSystem);
	//Todo
}

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::createPvdInstance(const NpHairSystem* hairSystem)
{
	PX_UNUSED(hairSystem);
	//Todo
}

void PvdSceneClient::updatePvdProperties(const NpHairSystem* hairSystem)
{
	PX_UNUSED(hairSystem);
	//Todo
}

void PvdSceneClient::attachAggregateActor(const NpHairSystem* hairSystem, NpActor* actor)
{
	PX_UNUSED(hairSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::detachAggregateActor(const NpHairSystem* hairSystem, NpActor* actor)
{
	PX_UNUSED(hairSystem);
	PX_UNUSED(actor);
	//Todo
}

void PvdSceneClient::releasePvdInstance(const NpHairSystem* hairSystem)
{
	PX_UNUSED(hairSystem);
	//Todo
}

#endif

///////////////////////////////////////////////////////////////////////////////

void PvdSceneClient::updateJoints()
{
	if(checkPvdDebugFlag())
	{
		PX_PROFILE_ZONE("PVD.updateJoints", getContextId(mScene));

		const bool visualizeJoints = getScenePvdFlagsFast() & PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS;

		Sc::ConstraintCore*const * constraints = mScene.getScScene().getConstraints();
		const PxU32 nbConstraints = mScene.getScScene().getNbConstraints();
		PxI64 constraintCount = 0;

		for(PxU32 i=0; i<nbConstraints; i++)
		{
			Sc::ConstraintCore* constraint = constraints[i];
			PxPvdUpdateType::Enum updateType = getNpConstraint(constraint)->isDirty()
				? PxPvdUpdateType::UPDATE_ALL_PROPERTIES
				: PxPvdUpdateType::UPDATE_SIM_PROPERTIES;
			updateConstraint(*constraint, updateType);
			PxConstraintConnector* conn = constraint->getPxConnector();
			// visualization is updated here
			{
				PxU32 typeId = 0;
				void* joint = NULL;
				if(conn)
					joint = conn->getExternalReference(typeId);
				// visualize:
				Sc::ConstraintSim* sim = constraint->getSim();
				if(visualizeJoints && sim && sim->getConstantsLL() && joint && constraint->getVisualize())
				{
					Sc::BodySim* b0 = sim->getBody(0);
					Sc::BodySim* b1 = sim->getBody(1);
					PxTransform t0 = b0 ? b0->getBody2World() : PxTransform(PxIdentity);
					PxTransform t1 = b1 ? b1->getBody2World() : PxTransform(PxIdentity);
					PvdConstraintVisualizer viz(joint, *mUserRender);
					(*constraint->getVisualize())(viz, sim->getConstantsLL(), t0, t1, 0xffffFFFF);
				}
			}
			++constraintCount;
		}

		mUserRender->flushRenderEvents();
	}
}

void PvdSceneClient::updateContacts()
{
	if(!checkPvdDebugFlag())
		return;

	PX_PROFILE_ZONE("PVD.updateContacts", getContextId(mScene));

	// if contacts are disabled, send empty array and return
	const PxScene* theScene = &mScene;
	if(!(getScenePvdFlagsFast() & PxPvdSceneFlag::eTRANSMIT_CONTACTS))
	{
		mMetaDataBinding.sendContacts(*mPvdDataStream, *theScene);
		return;
	}

	PxsContactManagerOutputIterator outputIter;

	Sc::ContactIterator contactIter;
	mScene.getScScene().initContactsIterator(contactIter, outputIter);
	Sc::ContactIterator::Pair* pair;
	Sc::Contact* contact;
	PxArray<Sc::Contact> contacts;
	while ((pair = contactIter.getNextPair()) != NULL)
	{
		while ((contact = pair->getNextContact()) != NULL)
			contacts.pushBack(*contact);
	}
	
	mMetaDataBinding.sendContacts(*mPvdDataStream, *theScene, contacts);
}

void PvdSceneClient::updateSceneQueries()
{
	if(checkPvdDebugFlag() && (getScenePvdFlagsFast() & PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES))
		mMetaDataBinding.sendSceneQueries(*mPvdDataStream, mScene, mPvd);
}

void PvdSceneClient::visualize(PxArticulationLink& link)
{
#if PX_ENABLE_DEBUG_VISUALIZATION
	NpArticulationLink& npLink = static_cast<NpArticulationLink&>(link);
	const void* itemId = npLink.getInboundJoint();
	if(itemId && mUserRender)
	{
		PvdConstraintVisualizer viz(itemId, *mUserRender);
		npLink.visualizeJoint(viz);
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
	PX_UNUSED(link);
#endif
}

void PvdSceneClient::visualize(const PxRenderBuffer& debugRenderable)
{
	if(mUserRender)
	{
		// PT: I think the mUserRender object can contain extra data (including things coming from the user), because the various
		// draw functions are exposed e.g. in PxPvdSceneClient.h. So I suppose we have to keep the render buffer around regardless
		// of the connection flags. Thus I only skip the "drawRenderbuffer" call, for minimal intrusion into this file.
		if(checkPvdDebugFlag())
		{
			mUserRender->drawRenderbuffer(
				reinterpret_cast<const PxDebugPoint*>(debugRenderable.getPoints()), debugRenderable.getNbPoints(),
				reinterpret_cast<const PxDebugLine*>(debugRenderable.getLines()), debugRenderable.getNbLines(),
				reinterpret_cast<const PxDebugTriangle*>(debugRenderable.getTriangles()), debugRenderable.getNbTriangles());
		}
		mUserRender->flushRenderEvents();
	}
}

#endif

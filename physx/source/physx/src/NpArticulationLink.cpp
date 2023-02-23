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

#include "NpArticulationLink.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpCheck.h"
#include "CmVisualization.h"
#include "CmConeLimitHelper.h"
#include "CmUtils.h"
#include "NpRigidActorTemplateInternal.h"

using namespace physx;
using namespace Cm;

// PX_SERIALIZATION
void NpArticulationLink::requiresObjects(PxProcessPxBaseCallback& c)
{
	NpArticulationLinkT::requiresObjects(c);
	
	if(mInboundJoint)
		c.process(*mInboundJoint);
}

void NpArticulationLink::exportExtraData(PxSerializationContext& stream)
{
	NpArticulationLinkT::exportExtraData(stream);
	exportInlineArray(mChildLinks, stream);
}

void NpArticulationLink::importExtraData(PxDeserializationContext& context)
{
	NpArticulationLinkT::importExtraData(context);
	importInlineArray(mChildLinks, context);
}

void NpArticulationLink::resolveReferences(PxDeserializationContext& context)
{	
    context.translatePxBase(mRoot);
    context.translatePxBase(mInboundJoint);
    context.translatePxBase(mParent);
       
    NpArticulationLinkT::resolveReferences(context);

    const PxU32 nbLinks = mChildLinks.size();
    for(PxU32 i=0;i<nbLinks;i++)
        context.translatePxBase(mChildLinks[i]);
}

NpArticulationLink* NpArticulationLink::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationLink* obj = PX_PLACEMENT_NEW(address, NpArticulationLink(PxBaseFlags(0)));
	address += sizeof(NpArticulationLink);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulationLink::NpArticulationLink(const PxTransform& bodyPose, PxArticulationReducedCoordinate& root, NpArticulationLink* parent) :
	NpArticulationLinkT	(PxConcreteType::eARTICULATION_LINK, PxBaseFlag::eOWNS_MEMORY, PxActorType::eARTICULATION_LINK, NpType::eBODY_FROM_ARTICULATION_LINK, bodyPose),
	mRoot				(&root),
	mInboundJoint		(NULL),
	mParent				(parent),
	mLLIndex			(0xffffffff),
	mInboundJointDof	(0xffffffff)
{
	if (parent)
		parent->addToChildList(*this);
}

NpArticulationLink::~NpArticulationLink()
{
}

void NpArticulationLink::releaseInternal()
{
	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, userData);

	NpArticulationReducedCoordinate* npArticulation = static_cast<NpArticulationReducedCoordinate*>(mRoot);
	npArticulation->removeLinkFromList(*this);

	if (mParent)
		mParent->removeFromChildList(*this);

	if (mInboundJoint)
		mInboundJoint->release();

	//Remove constraints, aggregates, scene, shapes. 
	removeRigidActorT<PxArticulationLink>(*this);

	PX_ASSERT(!isAPIWriteForbidden());
	NpDestroyArticulationLink(this);
}

void NpArticulationLink::release()
{
	if(getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationLink::release() not allowed while the articulation link is in a scene. Call will be ignored.");
		return;
	}

	//! this function doesn't get called when the articulation root is released
	// therefore, put deregistration code etc. into dtor, not here

	if (mChildLinks.empty())
	{
		releaseInternal();
	}
	else
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationLink::release(): Only leaf articulation links can be released. Call will be ignored.");
	}
}

PxTransform NpArticulationLink::getGlobalPose() const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxArticulationLink::getGlobalPose() not allowed while simulation is running (except during PxScene::collide()).", PxTransform(PxIdentity));

	return mCore.getBody2World() * mCore.getBody2Actor().getInverse();
}

bool NpArticulationLink::attachShape(PxShape& shape)
{
	static_cast<NpArticulationReducedCoordinate*>(mRoot)->incrementShapeCount();
	return NpRigidActorTemplate::attachShape(shape);
}

void NpArticulationLink::detachShape(PxShape& shape, bool wakeOnLostTouch)
{
	static_cast<NpArticulationReducedCoordinate*>(mRoot)->decrementShapeCount();
	NpRigidActorTemplate::detachShape(shape, wakeOnLostTouch);
}

PxArticulationReducedCoordinate& NpArticulationLink::getArticulation() const
{
	NP_READ_CHECK(getNpScene());
	return *mRoot;
}

PxArticulationJointReducedCoordinate* NpArticulationLink::getInboundJoint() const
{
	NP_READ_CHECK(getNpScene());
	return mInboundJoint;
}

PxU32 NpArticulationLink::getInboundJointDof() const
{
	NP_READ_CHECK(getNpScene());

	return getNpScene() ? mInboundJointDof : 0xffffffffu;
}

PxU32 NpArticulationLink::getNbChildren() const
{
	NP_READ_CHECK(getNpScene());
	return mChildLinks.size();
}

PxU32 NpArticulationLink::getChildren(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mChildLinks.begin(), mChildLinks.size());
}

PxU32 NpArticulationLink::getLinkIndex() const
{
	NP_READ_CHECK(getNpScene());
	return getNpScene() ? mLLIndex : 0xffffffffu;
}

void NpArticulationLink::setCMassLocalPose(const PxTransform& pose)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(pose.isSane(), "PxArticulationLink::setCMassLocalPose: invalid parameter");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationLink::setCMassLocalPose() not allowed while simulation is running. Call will be ignored.")

	if (getNpScene() && getNpScene()->getFlags() & PxSceneFlag::eSUPPRESS_READBACK)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationLink::setCMassLocalPose() : it is illegal to call this method if PxSceneFlag::eSUPPRESS_ARTICULATION_READBACK is enabled!");
	}

	const PxTransform p = pose.getNormalized();
	const PxTransform oldpose = mCore.getBody2Actor();
	const PxTransform comShift = p.transformInv(oldpose);

	NpArticulationLinkT::setCMassLocalPoseInternal(p);

	if(mInboundJoint)
	{
		NpArticulationJointReducedCoordinate* j =static_cast<NpArticulationJointReducedCoordinate*>(mInboundJoint);
		j->scSetChildPose(comShift.transform(j->getCore().getChildPose()));
	}

	for(PxU32 i=0; i<mChildLinks.size(); i++)
	{
		NpArticulationJointReducedCoordinate* j = static_cast<NpArticulationJointReducedCoordinate*>(mChildLinks[i]->getInboundJoint());
		j->scSetParentPose(comShift.transform(j->getCore().getParentPose()));
	}
}

void NpArticulationLink::addForce(const PxVec3& force, PxForceMode::Enum mode, bool autowake)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(force.isFinite(), "PxArticulationLink::addForce: force is not valid.");
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationLink::addForce: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationLink::addForce() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.")

	addSpatialForce(&force, NULL, mode);

	static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal((!force.isZero()), autowake);
}

void NpArticulationLink::addTorque(const PxVec3& torque, PxForceMode::Enum mode, bool autowake)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(torque.isFinite(), "PxArticulationLink::addTorque: force is not valid.");
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationLink::addTorque: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationLink::addTorque() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.")

	addSpatialForce(NULL, &torque, mode);

	static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal((!torque.isZero()), autowake);
}

void NpArticulationLink::setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(torque.isFinite(), "PxArticulationLink::setForceAndTorque: torque is not valid.");
	PX_CHECK_AND_RETURN(force.isFinite(), "PxArticulationLink::setForceAndTorque: force is not valid.");
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationLink::addTorque: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationLink::setForceAndTorque() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.");

	setSpatialForce(&force, &torque, mode);

	static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal((!torque.isZero()), true);
}

void NpArticulationLink::clearForce(PxForceMode::Enum mode)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationLink::clearForce: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationLink::clearForce() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.");

	clearSpatialForce(mode, true, false);
}

void NpArticulationLink::clearTorque(PxForceMode::Enum mode)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene(), "PxArticulationLink::clearTorque: Articulation link must be in a scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxArticulationLink::clearTorque() not allowed while simulation is running, except in a split simulation in-between PxScene::fetchCollision() and PxScene::advance().Call will be ignored.");

	clearSpatialForce(mode, false, true);
}

void NpArticulationLink::setCfmScale(const PxReal cfmScale) 
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(cfmScale >= 0.f  && cfmScale <= 1.f, "PxArticulationLink::setCfmScale: cfm is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxArticulationLink::setCfmScale() not allowed while simulation is running. Call will be ignored.")

	mCore.getCore().cfmScale = cfmScale;
	OMNI_PVD_SET(actor, CFMScale, static_cast<PxActor&>(*this), cfmScale); // @@@
}

PxReal NpArticulationLink::getCfmScale() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getCore().cfmScale;
}

void NpArticulationLink::setGlobalPoseInternal(const PxTransform& pose, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(pose.isSane(), "PxArticulationLink::setGlobalPose: pose is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxArticulationLink::setGlobalPose() not allowed while simulation is running. Call will be ignored.")

#if PX_CHECKED
	if (npScene)
		npScene->checkPositionSanity(*this, pose, "PxArticulationLink::setGlobalPose");
#endif

	const PxTransform newPose = pose.getNormalized();	//AM: added to fix 1461 where users read and write orientations for no reason.

	const PxTransform body2World = newPose * mCore.getBody2Actor();
	scSetBody2World(body2World);

	if (npScene && autowake)
		static_cast<NpArticulationReducedCoordinate*>(mRoot)->wakeUpInternal(false, true);

	if (npScene)
		static_cast<NpArticulationReducedCoordinate*>(mRoot)->setGlobalPose();
}

void NpArticulationLink::setKinematicLink(const bool value)
{
	NP_WRITE_CHECK(getNpScene());

	mCore.setKinematicLink(value);
}

PxU32 physx::NpArticulationGetShapes(NpArticulationLink& actor, NpShape* const*& shapes, bool* isCompound)
{
	NpShapeManager& sm = actor.getShapeManager();
	shapes = sm.getShapes();
	if (isCompound)
		*isCompound = sm.isSqCompound();
	return sm.getNbShapes();
}

#if PX_ENABLE_DEBUG_VISUALIZATION
void NpArticulationLink::visualize(PxRenderOutput& out, NpScene& scene, float scale) const
{
	PX_ASSERT(scale!=0.0f);	// Else we shouldn't have been called
	if(!(mCore.getActorFlags() & PxActorFlag::eVISUALIZATION))
		return;

	NpArticulationLinkT::visualize(out, scene, scale);

	const Sc::Scene& scScene = scene.getScScene();

	// PT: TODO: can we share this with the PxRigidDynamic version?
	const PxReal massAxes = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eBODY_MASS_AXES);
	if(massAxes != 0)
	{
		PxU32 color = 0xff;
		color = (color<<16 | color<<8 | color);
		PxVec3 dims = invertDiagInertia(mCore.getInverseInertia());
		dims = getDimsFromBodyInertia(dims, 1.0f / mCore.getInverseMass());
		out << color << mCore.getBody2World();
		const PxVec3 extents = dims * 0.5f;
		renderOutputDebugBox(out, PxBounds3(-extents, extents));
	}

	const PxReal frameScale = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES);
	const PxReal limitScale = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS);
	if(frameScale != 0.0f || limitScale != 0.0f)
	{
		ConstraintImmediateVisualizer viz(frameScale, limitScale, out);
		visualizeJoint(viz);
	}
}

static PX_FORCE_INLINE PxReal computePhi(const PxQuat& q)
{
	PxQuat twist = q;
	twist.normalize();

	PxReal angle = twist.getAngle();
	if (twist.x<0.0f)
		angle = -angle;
	return angle;
}

// PT: TODO: don't duplicate this, it should be available in MathUtils or something
static PX_FORCE_INLINE float computeSwingAngle(float swingYZ, float swingW)
{
	return 4.0f * PxAtan2(swingYZ, 1.0f + swingW);	// tan (t/2) = sin(t)/(1+cos t), so this is the quarter angle
}

static PX_FORCE_INLINE void separateSwingTwist(const PxQuat& q, PxQuat& twist, PxQuat& swing1, PxQuat& swing2)
{
	twist = q.x != 0.0f ? PxQuat(q.x, 0, 0, q.w).getNormalized() : PxQuat(PxIdentity);
	PxQuat swing = q * twist.getConjugate();
	swing1 = swing.y != 0.f ? PxQuat(0.f, swing.y, 0.f, swing.w).getNormalized() : PxQuat(PxIdentity);
	swing = swing * swing1.getConjugate();
	swing2 = swing.z != 0.f ? PxQuat(0.f, 0.f, swing.z, swing.w).getNormalized() : PxQuat(PxIdentity);
}

void NpArticulationLink::visualizeJoint(PxConstraintVisualizer& jointViz) const
{
	const NpArticulationLink* parent = getParent();
	if(parent)
	{
		PxTransform cA2w = getGlobalPose().transform(mInboundJoint->getChildPose());
		PxTransform cB2w = parent->getGlobalPose().transform(mInboundJoint->getParentPose());
	
		jointViz.visualizeJointFrames(cA2w, cB2w);

		NpArticulationJointReducedCoordinate* impl = static_cast<NpArticulationJointReducedCoordinate*>(mInboundJoint);

		PX_ASSERT(getArticulation().getConcreteType() == PxConcreteType::eARTICULATION_REDUCED_COORDINATE);
		//(1) visualize any angular dofs/limits...

		const PxMat33 cA2w_m(cA2w.q), cB2w_m(cB2w.q);

		PxTransform parentFrame = cB2w;

		if (cA2w.q.dot(cB2w.q) < 0)
			cB2w.q = -cB2w.q;

		//const PxTransform cB2cA = cA2w.transformInv(cB2w);

		const PxTransform cA2cB = cB2w.transformInv(cA2w);

		Sc::ArticulationJointCore& joint = impl->getCore();

		PxQuat swing1, swing2, twist;
		separateSwingTwist(cA2cB.q, twist, swing1, swing2);

		const PxReal pad = 0.01f;

		if(joint.getMotion(PxArticulationAxis::eTWIST))
		{
			PxArticulationLimit pair;

			const PxReal angle = computePhi(twist);
			pair = joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eTWIST));

			bool active = (angle-pad) < pair.low || (angle+pad) > pair.high;

			PxTransform tmp = parentFrame;

			jointViz.visualizeAngularLimit(tmp, pair.low, pair.high, active);
		}

		if (joint.getMotion(PxArticulationAxis::eSWING1))
		{
			PxArticulationLimit pair;

			pair = joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eSWING1));

			const PxReal angle = computeSwingAngle(swing1.y, swing1.w);

			bool active = (angle - pad) < pair.low || (angle + pad) > pair.high;

			PxTransform tmp = parentFrame;
			tmp.q = tmp.q * PxQuat(-PxPiDivTwo, PxVec3(0.f, 0.f, 1.f));

				
			jointViz.visualizeAngularLimit(tmp, -pair.high, -pair.low, active);
		}

		if (joint.getMotion(PxArticulationAxis::eSWING2))
		{
			PxArticulationLimit pair;

			pair= joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eSWING2));

			const PxReal angle = computeSwingAngle(swing2.z, swing2.w);

			bool active = (angle - pad) < pair.low || (angle + pad) > pair.high;

			PxTransform tmp = parentFrame;
			tmp.q = tmp.q * PxQuat(PxPiDivTwo, PxVec3(0.f, 1.f, 0.f));

			jointViz.visualizeAngularLimit(tmp, -pair.high, -pair.low, active);
		}

		for (PxU32 i = PxArticulationAxis::eX; i <= PxArticulationAxis::eZ; ++i)
		{
			if (joint.getMotion(PxArticulationAxis::Enum(i)) == PxArticulationMotion::eLIMITED)
			{

				PxArticulationLimit pair;

				PxU32 index = i - PxArticulationAxis::eX;
				
				pair = joint.getLimit(PxArticulationAxis::Enum(i));
				PxReal ordinate = cA2cB.p[index];
				PxVec3 origin = cB2w.p;
				PxVec3 axis = cA2w_m[index];
				const bool active = ordinate < pair.low || ordinate > pair.high;
				const PxVec3 p0 = origin + axis * pair.low;
				const PxVec3 p1 = origin + axis * pair.high;
				jointViz.visualizeLine(p0, p1, active ? 0xff0000u : 0xffffffu);
			}
		}
		
	}	
}

#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

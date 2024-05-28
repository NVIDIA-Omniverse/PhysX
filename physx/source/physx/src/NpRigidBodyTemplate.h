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

#ifndef NP_RIGIDBODY_TEMPLATE_H
#define NP_RIGIDBODY_TEMPLATE_H

#include "NpRigidActorTemplate.h"
#include "ScBodyCore.h"
#include "NpPhysics.h"
#include "NpShape.h"
#include "NpScene.h"
#include "CmVisualization.h"
#include "NpDebugViz.h"
#include "omnipvd/NpOmniPvdSetData.h"

#if PX_SUPPORT_PVD
	// PT: updatePvdProperties() is overloaded and the compiler needs to know 'this' type to do the right thing.
	// Thus we can't just move this as an inlined Base function.
	#define UPDATE_PVD_PROPERTY_BODY																					\
		{																												\
			NpScene* sceneForPVD = RigidActorTemplateClass::getNpScene();	/* shared shapes also return zero here */	\
			if(sceneForPVD)																								\
				sceneForPVD->getScenePvdClientInternal().updateBodyPvdProperties(static_cast<NpActor*>(this));				\
		}
#else
	#define UPDATE_PVD_PROPERTY_BODY
#endif

namespace physx
{
PX_INLINE PxVec3 invertDiagInertia(const PxVec3& m)
{
	return PxVec3(	m.x == 0.0f ? 0.0f : 1.0f/m.x,
					m.y == 0.0f ? 0.0f : 1.0f/m.y,
					m.z == 0.0f ? 0.0f : 1.0f/m.z);
}

#if PX_ENABLE_DEBUG_VISUALIZATION
/*
given the diagonal of the body space inertia tensor, and the total mass
this returns the body space AABB width, height and depth of an equivalent box
*/
PX_INLINE PxVec3 getDimsFromBodyInertia(const PxVec3& inertiaMoments, PxReal mass)
{
	const PxVec3 inertia = inertiaMoments * (6.0f/mass);
	return PxVec3(	PxSqrt(PxAbs(- inertia.x + inertia.y + inertia.z)),
					PxSqrt(PxAbs(+ inertia.x - inertia.y + inertia.z)),
					PxSqrt(PxAbs(+ inertia.x + inertia.y - inertia.z)));
}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

template<class APIClass>
class NpRigidBodyTemplate : public NpRigidActorTemplate<APIClass>
{
protected:
	typedef		NpRigidActorTemplate<APIClass> RigidActorTemplateClass;
public:
// PX_SERIALIZATION
										NpRigidBodyTemplate(PxBaseFlags baseFlags) : RigidActorTemplateClass(baseFlags), mCore(PxEmpty)	{}
//~PX_SERIALIZATION
	virtual								~NpRigidBodyTemplate();

	// The rule is: If an API method is used somewhere in here, it has to be redeclared, else GCC whines

	// PxRigidActor
	virtual			PxTransform			getGlobalPose() const = 0;
	virtual			bool				attachShape(PxShape& shape)	PX_OVERRIDE;
	//~PxRigidActor

	// PxRigidBody
	virtual			PxTransform 		getCMassLocalPose() const	PX_OVERRIDE PX_FINAL;
	virtual			void				setMass(PxReal mass)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getMass() const	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getInvMass() const	PX_OVERRIDE PX_FINAL;
	virtual			void				setMassSpaceInertiaTensor(const PxVec3& m)	PX_OVERRIDE PX_FINAL;
	virtual			PxVec3				getMassSpaceInertiaTensor() const	PX_OVERRIDE PX_FINAL;
	virtual			PxVec3				getMassSpaceInvInertiaTensor() const	PX_OVERRIDE PX_FINAL;
	virtual			void				setLinearDamping(PxReal linDamp)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getLinearDamping()	const	PX_OVERRIDE PX_FINAL;
	virtual			void				setAngularDamping(PxReal angDamp)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getAngularDamping()		const	PX_OVERRIDE PX_FINAL;
	virtual			PxVec3				getLinearVelocity()		const	PX_OVERRIDE PX_FINAL;
	virtual			PxVec3				getAngularVelocity()	const	PX_OVERRIDE PX_FINAL;
	virtual			void				setMaxLinearVelocity(PxReal maxLinVel)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getMaxLinearVelocity()	const	PX_OVERRIDE PX_FINAL;
	virtual			void				setMaxAngularVelocity(PxReal maxAngVel)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getMaxAngularVelocity()	const	PX_OVERRIDE PX_FINAL;
	//~PxRigidBody

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
										NpRigidBodyTemplate(PxType concreteType, PxBaseFlags baseFlags, const PxActorType::Enum type, NpType::Enum npType, const PxTransform& bodyPose);

	PX_FORCE_INLINE	const Sc::BodyCore&	getCore()		const	{ return mCore;			}
	PX_FORCE_INLINE	Sc::BodyCore&		getCore()				{ return mCore;			}

	// Flags
	virtual			void				setRigidBodyFlag(PxRigidBodyFlag::Enum, bool value)	PX_OVERRIDE PX_FINAL;
	virtual			void				setRigidBodyFlags(PxRigidBodyFlags inFlags)	PX_OVERRIDE PX_FINAL;
	PX_FORCE_INLINE	PxRigidBodyFlags	getRigidBodyFlagsFast() const	{ return mCore.getFlags();	}
	virtual			PxRigidBodyFlags	getRigidBodyFlags() const	PX_OVERRIDE PX_FINAL
										{
											NP_READ_CHECK(RigidActorTemplateClass::getNpScene());
											return getRigidBodyFlagsFast() & ~PxRigidBodyFlag::eRESERVED;
										}

	virtual			void				setMinCCDAdvanceCoefficient(PxReal advanceCoefficient)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getMinCCDAdvanceCoefficient() const	PX_OVERRIDE PX_FINAL;
	virtual			void				setMaxDepenetrationVelocity(PxReal maxDepenVel)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getMaxDepenetrationVelocity() const	PX_OVERRIDE PX_FINAL;
	virtual			void				setMaxContactImpulse(PxReal maxDepenVel)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getMaxContactImpulse() const	PX_OVERRIDE PX_FINAL;
	virtual			void				setContactSlopCoefficient(PxReal slopCoefficient)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal				getContactSlopCoefficient() const	PX_OVERRIDE PX_FINAL;

	virtual			PxNodeIndex			getInternalIslandNodeIndex() const	PX_OVERRIDE PX_FINAL;

protected:
					void				setCMassLocalPoseInternal(const PxTransform&);

					void				addSpatialForce(const PxVec3* force, const PxVec3* torque, PxForceMode::Enum mode);
					void				clearSpatialForce(PxForceMode::Enum mode, bool force, bool torque);
					void				setSpatialForce(const PxVec3* force, const PxVec3* torque, PxForceMode::Enum mode);

	PX_FORCE_INLINE void				setRigidBodyFlagsInternal(const PxRigidBodyFlags& currentFlags, const PxRigidBodyFlags& newFlags);

public:
#if PX_ENABLE_DEBUG_VISUALIZATION
					void				visualize(PxRenderOutput& out, NpScene& scene, float scale)	const;
#else
					PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	PX_FORCE_INLINE bool				isKinematic() const
										{
											return (APIClass::getConcreteType() == PxConcreteType::eRIGID_DYNAMIC) && (mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC);
										}

	PX_INLINE		void				scSetSolverIterationCounts(PxU16 c)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbidden());
											mCore.setSolverIterationCounts(c);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scSetLockFlags(PxRigidDynamicLockFlags f)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbidden());
											mCore.setRigidDynamicLockFlags(f);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scSetBody2World(const PxTransform& p)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbidden());
											mCore.setBody2World(p);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scSetCMassLocalPose(const PxTransform& newBody2Actor)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbidden());
											mCore.setCMassLocalPose(newBody2Actor);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scSetLinearVelocity(const PxVec3& v)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());
											mCore.setLinearVelocity(v);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scSetAngularVelocity(const PxVec3& v)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());
											mCore.setAngularVelocity(v);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scWakeUpInternal(PxReal wakeCounter)
										{
											PX_ASSERT(RigidActorTemplateClass::getNpScene());

											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());
											mCore.wakeUp(wakeCounter);
										}

	PX_FORCE_INLINE void				scWakeUp()
										{
											PX_ASSERT(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC));

											NpScene* scene = RigidActorTemplateClass::getNpScene();
											PX_ASSERT(scene);  // only allowed for an object in a scene

											scWakeUpInternal(scene->getWakeCounterResetValueInternal());
										}

	PX_INLINE		void				scPutToSleepInternal()
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbidden());
											mCore.putToSleep();
										}

	PX_FORCE_INLINE void				scPutToSleep()
										{
											PX_ASSERT(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC));

											scPutToSleepInternal();
										}

	PX_INLINE		void				scSetWakeCounter(PxReal w)
										{
											PX_ASSERT(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC));

											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());
											mCore.setWakeCounter(w);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scSetFlags(PxRigidBodyFlags f)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbidden());
											mCore.setFlags(f);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE		void				scAddSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());

											mCore.addSpatialAcceleration(linAcc, angAcc);
											//Spatial acceleration isn't sent to PVD.
										}

	PX_INLINE		void				scSetSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());

											mCore.setSpatialAcceleration(linAcc, angAcc);
											//Spatial acceleration isn't sent to PVD.
										}

	PX_INLINE	void					scClearSpatialAcceleration(bool force, bool torque)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());

											mCore.clearSpatialAcceleration(force, torque);
											//Spatial acceleration isn't sent to PVD.
										}

	PX_INLINE	void					scAddSpatialVelocity(const PxVec3* linVelDelta, const PxVec3* angVelDelta)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());

											mCore.addSpatialVelocity(linVelDelta, angVelDelta);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE	void					scClearSpatialVelocity(bool force, bool torque)
										{
											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());

											mCore.clearSpatialVelocity(force, torque);
											UPDATE_PVD_PROPERTY_BODY
										}

	PX_INLINE	void					scSetKinematicTarget(const PxTransform& p)
										{
											NpScene* scene = RigidActorTemplateClass::getNpScene();
											PX_ASSERT(scene);  // only allowed for an object in a scene
											const PxReal wakeCounterResetValue = scene->getWakeCounterResetValueInternal();

											PX_ASSERT(!RigidActorTemplateClass::isAPIWriteForbiddenExceptSplitSim());

											mCore.setKinematicTarget(p, wakeCounterResetValue);

											UPDATE_PVD_PROPERTY_BODY

										#if PX_SUPPORT_PVD
											scene->getScenePvdClientInternal().updateKinematicTarget(this, p);
										#endif
										}

	PX_INLINE	PxMat33					scGetGlobalInertiaTensorInverse() const
										{
											PxMat33 inverseInertiaWorldSpace;
											Cm::transformInertiaTensor(mCore.getInverseInertia(), PxMat33Padded(mCore.getBody2World().q), inverseInertiaWorldSpace);
											return inverseInertiaWorldSpace;
										}

	PX_FORCE_INLINE bool				scCheckSleepReadinessBesidesWakeCounter()
										{
											return (getLinearVelocity().isZero() && getAngularVelocity().isZero());
											// no need to test for pending force updates yet since currently this is not supported on scene insertion
										}

protected:
					Sc::BodyCore		mCore;
};

template<class APIClass>
NpRigidBodyTemplate<APIClass>::NpRigidBodyTemplate(PxType concreteType, PxBaseFlags baseFlags, PxActorType::Enum type, NpType::Enum npType, const PxTransform& bodyPose) :
	RigidActorTemplateClass	(concreteType, baseFlags, npType),
	mCore					(type, bodyPose)
{
}

template<class APIClass>
NpRigidBodyTemplate<APIClass>::~NpRigidBodyTemplate()
{
}

namespace
{
	PX_FORCE_INLINE static bool hasNegativeMass(const PxShape& shape)
	{
		const PxGeometry& geom = shape.getGeometry();
		const PxGeometryType::Enum t = geom.getType();
		if (t == PxGeometryType::eTRIANGLEMESH)
		{
			const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
			const Gu::TriangleMesh* mesh = static_cast<const Gu::TriangleMesh*>(triGeom.triangleMesh);
			return mesh->getSdfDataFast().mSdf != NULL && mesh->getMass() < 0.f;
		}
		return false;
	}

	PX_FORCE_INLINE static bool isDynamicMesh(const PxGeometry& geom)
	{
		PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
		const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
		const Gu::TriangleMesh* mesh = static_cast<const Gu::TriangleMesh*>(triGeom.triangleMesh);
		return mesh->getSdfDataFast().mSdf != NULL;
		// Note: We're not testing for (mesh->getMass() > 0.f) here because
		// a) we cannot infer the mass of the rigid body from the mesh volume and
		// b) in principle, it is ok to have zero-width meshes, even though collision quality may suffer
	}

	PX_FORCE_INLINE static bool isSimGeom(const PxShape& shape)
	{
		const PxGeometryType::Enum t = shape.getGeometry().getType();
		return t != PxGeometryType::ePLANE && t != PxGeometryType::eHEIGHTFIELD && t != PxGeometryType::eTETRAHEDRONMESH &&
			(t != PxGeometryType::eTRIANGLEMESH || isDynamicMesh(shape.getGeometry()));
	}
}

template<class APIClass>
bool NpRigidBodyTemplate<APIClass>::attachShape(PxShape& shape)
{
	NP_WRITE_CHECK(RigidActorTemplateClass::getNpScene());
	PX_CHECK_AND_RETURN_VAL(!(shape.getFlags() & PxShapeFlag::eSIMULATION_SHAPE)
		|| !hasNegativeMass(shape)
		|| isKinematic(),
		"attachShape: The faces of the mesh are oriented the wrong way round leading to a negative mass. Please invert the orientation of all faces and try again.", false);

	PX_CHECK_AND_RETURN_VAL(!(shape.getFlags() & PxShapeFlag::eSIMULATION_SHAPE) 
						|| isSimGeom(shape) 
						|| isKinematic(),
						"attachShape: non-SDF triangle mesh, tetrahedron mesh, heightfield or plane geometry shapes configured as eSIMULATION_SHAPE are not supported for non-kinematic PxRigidDynamic instances.", false);

	return RigidActorTemplateClass::attachShape(shape);
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setCMassLocalPoseInternal(const PxTransform& body2Actor)
{
	//the point here is to change the mass distribution w/o changing the actors' pose in the world

	// AD note: I added an interface directly into the bodycore and pushed calculations there to avoid
	// NP and BP transform/bounds update notifications because this does not change the global pose.
	
	scSetCMassLocalPose(body2Actor);

	RigidActorTemplateClass::updateShaderComs();

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, cMassLocalPose, static_cast<PxRigidBody&>(*this), body2Actor)
}

template<class APIClass>
PxTransform NpRigidBodyTemplate<APIClass>::getCMassLocalPose() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return mCore.getBody2Actor();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setMass(PxReal mass)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(mass), "PxRigidBody::setMass(): invalid float");
	PX_CHECK_AND_RETURN(mass>=0, "PxRigidBody::setMass(): mass must be non-negative!");
	PX_CHECK_AND_RETURN(this->getType() != PxActorType::eARTICULATION_LINK || mass > 0.0f, "PxRigidBody::setMass(): components must be > 0 for articulations");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setMass() not allowed while simulation is running. Call will be ignored.")

	mCore.setInverseMass(mass > 0.0f ? 1.0f/mass : 0.0f);

	UPDATE_PVD_PROPERTY_BODY

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, mass, static_cast<PxRigidBody&>(*this), mass)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getMass() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());
	const PxReal invMass = mCore.getInverseMass();

	return invMass > 0.0f ? 1.0f/invMass : 0.0f;
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getInvMass() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return mCore.getInverseMass();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setMassSpaceInertiaTensor(const PxVec3& m)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
 	PX_CHECK_AND_RETURN(m.isFinite(), "PxRigidBody::setMassSpaceInertiaTensor(): invalid inertia");
	PX_CHECK_AND_RETURN(m.x>=0.0f && m.y>=0.0f && m.z>=0.0f, "PxRigidBody::setMassSpaceInertiaTensor(): components must be non-negative");
	PX_CHECK_AND_RETURN(this->getType() != PxActorType::eARTICULATION_LINK || (m.x > 0.0f && m.y > 0.0f && m.z > 0.0f), "PxRigidBody::setMassSpaceInertiaTensor(): components must be > 0 for articulations");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setMassSpaceInertiaTensor() not allowed while simulation is running. Call will be ignored.")

	mCore.setInverseInertia(invertDiagInertia(m));
	UPDATE_PVD_PROPERTY_BODY

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, massSpaceInertiaTensor, static_cast<PxRigidBody&>(*this), m)
}

template<class APIClass>
PxVec3 NpRigidBodyTemplate<APIClass>::getMassSpaceInertiaTensor() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return invertDiagInertia(mCore.getInverseInertia());
}

template<class APIClass>
PxVec3 NpRigidBodyTemplate<APIClass>::getMassSpaceInvInertiaTensor() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return mCore.getInverseInertia();
}

template<class APIClass>
PxVec3 NpRigidBodyTemplate<APIClass>::getLinearVelocity() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(RigidActorTemplateClass::getNpScene(), "PxRigidBody::getLinearVelocity() not allowed while simulation is running (except during PxScene::collide()).", PxVec3(PxZero));

	return mCore.getLinearVelocity();
}

template<class APIClass>
PxVec3 NpRigidBodyTemplate<APIClass>::getAngularVelocity() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(RigidActorTemplateClass::getNpScene(), "PxRigidBody::getAngularVelocity() not allowed while simulation is running (except during PxScene::collide()).", PxVec3(PxZero));

	return mCore.getAngularVelocity();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::addSpatialForce(const PxVec3* force, const PxVec3* torque, PxForceMode::Enum mode)
{
	PX_ASSERT(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC));

	switch (mode)
	{
		case PxForceMode::eFORCE:
		{
			PxVec3 linAcc, angAcc;
			if (force)
			{
				linAcc = (*force) * mCore.getInverseMass();
				force = &linAcc;
			}
			if (torque)
			{
				angAcc = scGetGlobalInertiaTensorInverse() * (*torque);
				torque = &angAcc;
			}
			scAddSpatialAcceleration(force, torque);
		}
		break;

		case PxForceMode::eACCELERATION:
			scAddSpatialAcceleration(force, torque);
		break;

		case PxForceMode::eIMPULSE:
		{
			PxVec3 linVelDelta, angVelDelta;
			if (force)
			{
				linVelDelta = (*force) * mCore.getInverseMass();
				force = &linVelDelta;
			}
			if (torque)
			{
				angVelDelta = scGetGlobalInertiaTensorInverse() * (*torque);
				torque = &angVelDelta;
			}
			scAddSpatialVelocity(force, torque);
		}
		break;

		case PxForceMode::eVELOCITY_CHANGE:
			scAddSpatialVelocity(force, torque);
		break;
	}
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setSpatialForce(const PxVec3* force, const PxVec3* torque, PxForceMode::Enum mode)
{
	PX_ASSERT(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC));

	switch (mode)
	{
	case PxForceMode::eFORCE:
	{
		PxVec3 linAcc, angAcc;
		if (force)
		{
			linAcc = (*force) * mCore.getInverseMass();
			force = &linAcc;
		}
		if (torque)
		{
			angAcc = scGetGlobalInertiaTensorInverse() * (*torque);
			torque = &angAcc;
		}
		scSetSpatialAcceleration(force, torque);
	}
	break;

	case PxForceMode::eACCELERATION:
		scSetSpatialAcceleration(force, torque);
		break;

	case PxForceMode::eIMPULSE:
	{
		PxVec3 linVelDelta, angVelDelta;
		if (force)
		{
			linVelDelta = (*force) * mCore.getInverseMass();
			force = &linVelDelta;
		}
		if (torque)
		{
			angVelDelta = scGetGlobalInertiaTensorInverse() * (*torque);
			torque = &angVelDelta;
		}
		scAddSpatialVelocity(force, torque);
	}
	break;

	case PxForceMode::eVELOCITY_CHANGE:
		scAddSpatialVelocity(force, torque);
		break;
	}
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::clearSpatialForce(PxForceMode::Enum mode, bool force, bool torque)
{
	PX_ASSERT(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC));

	switch (mode)
	{
	case PxForceMode::eFORCE:
	case PxForceMode::eACCELERATION:
		scClearSpatialAcceleration(force, torque);
		break;
	case PxForceMode::eIMPULSE:
	case PxForceMode::eVELOCITY_CHANGE:
		scClearSpatialVelocity(force, torque);
		break;
	}
}

#if PX_ENABLE_DEBUG_VISUALIZATION
template<class APIClass>
void NpRigidBodyTemplate<APIClass>::visualize(PxRenderOutput& out, NpScene& scene, float scale) const
{
	RigidActorTemplateClass::visualize(out, scene, scale);

	visualizeRigidBody(out, scene, *this, mCore, scale);
}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

template<class APIClass>
PX_FORCE_INLINE void NpRigidBodyTemplate<APIClass>::setRigidBodyFlagsInternal(const PxRigidBodyFlags& currentFlags, const PxRigidBodyFlags& newFlags)
{
	PxRigidBodyFlags filteredNewFlags = newFlags;
	//Test to ensure we are not enabling both CCD and kinematic state on a body. This is unsupported
	if((filteredNewFlags & PxRigidBodyFlag::eENABLE_CCD) && (filteredNewFlags & PxRigidBodyFlag::eKINEMATIC))
	{
		PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, 
			"PxRigidBody::setRigidBodyFlag(): kinematic bodies with CCD enabled are not supported! CCD will be ignored.");
		filteredNewFlags &= PxRigidBodyFlags(~PxRigidBodyFlag::eENABLE_CCD);
	}

	NpScene* scene = RigidActorTemplateClass::getNpScene();
	Sc::Scene* scScene = scene ? &scene->getScScene() : NULL;

	const bool isKinematic = currentFlags & PxRigidBodyFlag::eKINEMATIC;
	const bool willBeKinematic = filteredNewFlags & PxRigidBodyFlag::eKINEMATIC;
	const bool kinematicSwitchingToDynamic = isKinematic && (!willBeKinematic);
	const bool dynamicSwitchingToKinematic = (!isKinematic) && willBeKinematic;

	bool mustUpdateSQ = false;

	if(kinematicSwitchingToDynamic)
	{
		const NpShapeManager& shapeManager = this->getShapeManager();
		const PxU32 nbShapes = shapeManager.getNbShapes();
		NpShape*const* shapes = shapeManager.getShapes();
		bool hasIllegalShape = false;
		for(PxU32 i = 0; i < nbShapes; i++)
		{
			const PxShape& shape = *shapes[i];
			const bool isSimShape_ = shape.getFlags() & PxShapeFlag::eSIMULATION_SHAPE;
			const bool isSimGeom_ = isSimGeom(shape);
			if(isSimShape_ && !isSimGeom_)
			{
				// if shape is configured for simulation but underlying geometry does not support it, we have problem
				hasIllegalShape = true;
				break;
			}
		}
		if(hasIllegalShape)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxRigidBody::setRigidBodyFlag(): dynamic meshes (without SDF)/planes/heightfields are not supported!");
			return;
		}

		PxTransform bodyTarget;
		if ((currentFlags & PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES) && mCore.getKinematicTarget(bodyTarget) && scene)
			mustUpdateSQ = true;

		if(scScene)
		{
			scScene->decreaseNumKinematicsCounter();
			scScene->increaseNumDynamicsCounter();
		}
	}
	else if (dynamicSwitchingToKinematic)
	{
		if (this->getType() == PxActorType::eARTICULATION_LINK)
		{
			//We're an articulation, raise an issue
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxRigidBody::setRigidBodyFlag(): kinematic articulation links are not supported!");
			return;
		}

		if(scScene)
		{
			scScene->decreaseNumDynamicsCounter();
			scScene->increaseNumKinematicsCounter();
		}
	}

	const bool kinematicSwitchingUseTargetForSceneQuery = isKinematic && willBeKinematic && 
														((currentFlags & PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES) != (filteredNewFlags & PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES));
	if (kinematicSwitchingUseTargetForSceneQuery)
	{
		PxTransform bodyTarget;
		if (mCore.getKinematicTarget(bodyTarget) && scene)
			mustUpdateSQ = true;
	}

	scSetFlags(filteredNewFlags);

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, static_cast<PxRigidBody&>(*this), filteredNewFlags)

	// PT: the SQ update should be done after the scSetFlags() call
	if(mustUpdateSQ)
		this->getShapeManager().markActorForSQUpdate(scene->getSQAPI(), *this);
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setRigidBodyFlag(PxRigidBodyFlag::Enum flag, bool value)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setRigidBodyFlag() not allowed while simulation is running. Call will be ignored.")

	const PxRigidBodyFlags currentFlags = mCore.getFlags();
	const PxRigidBodyFlags newFlags = value ? currentFlags | flag : currentFlags & (~PxRigidBodyFlags(flag));

	setRigidBodyFlagsInternal(currentFlags, newFlags);
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setRigidBodyFlags(PxRigidBodyFlags inFlags)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setRigidBodyFlags() not allowed while simulation is running. Call will be ignored.")

	const PxRigidBodyFlags currentFlags = mCore.getFlags();

	setRigidBodyFlagsInternal(currentFlags, inFlags);
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setMinCCDAdvanceCoefficient(PxReal minCCDAdvanceCoefficient)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setMinCCDAdvanceCoefficient() not allowed while simulation is running. Call will be ignored.")

	mCore.setCCDAdvanceCoefficient(minCCDAdvanceCoefficient);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, minAdvancedCCDCoefficient, static_cast<PxRigidBody&>(*this), minCCDAdvanceCoefficient)

}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getMinCCDAdvanceCoefficient() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());
	return mCore.getCCDAdvanceCoefficient();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setMaxDepenetrationVelocity(PxReal maxDepenVel)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(maxDepenVel > 0.0f, "PxRigidBody::setMaxDepenetrationVelocity(): maxDepenVel must be greater than zero.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setMaxDepenetrationVelocity() not allowed while simulation is running. Call will be ignored.")

	mCore.setMaxPenetrationBias(-maxDepenVel);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxDepenetrationVelocity, static_cast<PxRigidBody&>(*this), maxDepenVel)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getMaxDepenetrationVelocity() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());
	return -mCore.getMaxPenetrationBias();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setMaxContactImpulse(const PxReal maxImpulse)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(maxImpulse >= 0.f, "PxRigidBody::setMaxContactImpulse(): impulse limit must be greater than or equal to zero.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setMaxContactImpulse() not allowed while simulation is running. Call will be ignored.")

	mCore.setMaxContactImpulse(maxImpulse);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxContactImpulse, static_cast<PxRigidBody&>(*this), maxImpulse)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getMaxContactImpulse() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());
	return mCore.getMaxContactImpulse();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setContactSlopCoefficient(const PxReal contactSlopCoefficient)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(contactSlopCoefficient >= 0.f, "PxRigidBody::setContactSlopCoefficient(): contact slop coefficientmust be greater than or equal to zero.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setContactSlopCoefficient() not allowed while simulation is running. Call will be ignored.")

	mCore.setOffsetSlop(contactSlopCoefficient);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, contactSlopCoefficient, static_cast<PxRigidBody&>(*this), contactSlopCoefficient)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getContactSlopCoefficient() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());
	return mCore.getOffsetSlop();
}

template<class APIClass>
PxNodeIndex NpRigidBodyTemplate<APIClass>::getInternalIslandNodeIndex() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());
	return mCore.getInternalIslandNodeIndex();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setLinearDamping(PxReal linearDamping)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(linearDamping), "PxRigidBody::setLinearDamping(): invalid float");
	PX_CHECK_AND_RETURN(linearDamping >= 0, "PxRigidBody::setLinearDamping(): The linear damping must be nonnegative!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setLinearDamping() not allowed while simulation is running. Call will be ignored.")

	mCore.setLinearDamping(linearDamping);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearDamping, static_cast<PxRigidBody&>(*this), linearDamping)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getLinearDamping() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return mCore.getLinearDamping();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setAngularDamping(PxReal angularDamping)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(angularDamping), "PxRigidBody::setAngularDamping(): invalid float");
	PX_CHECK_AND_RETURN(angularDamping>=0, "PxRigidBody::setAngularDamping(): The angular damping must be nonnegative!")

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setAngularDamping() not allowed while simulation is running. Call will be ignored.")

	mCore.setAngularDamping(angularDamping);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularDamping, static_cast<PxRigidBody&>(*this), angularDamping)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getAngularDamping() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return mCore.getAngularDamping();
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setMaxAngularVelocity(PxReal maxAngularVelocity)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(maxAngularVelocity), "PxRigidBody::setMaxAngularVelocity(): invalid float");
	PX_CHECK_AND_RETURN(maxAngularVelocity>=0.0f, "PxRigidBody::setMaxAngularVelocity(): threshold must be non-negative!");
	PX_CHECK_AND_RETURN(maxAngularVelocity <= PxReal(1.00000003e+16f), "PxRigidBody::setMaxAngularVelocity(): maxAngularVelocity*maxAngularVelocity must be less than 1e16");
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setMaxAngularVelocity() not allowed while simulation is running. Call will be ignored.")

	mCore.setMaxAngVelSq(maxAngularVelocity * maxAngularVelocity);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxAngularVelocity, static_cast<PxRigidBody&>(*this), maxAngularVelocity)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getMaxAngularVelocity() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return PxSqrt(mCore.getMaxAngVelSq());
}

template<class APIClass>
void NpRigidBodyTemplate<APIClass>::setMaxLinearVelocity(PxReal maxLinearVelocity)
{
	NpScene* npScene = RigidActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(maxLinearVelocity), "PxRigidBody::setMaxLinearVelocity(): invalid float");
	PX_CHECK_AND_RETURN(maxLinearVelocity >= 0.0f, "PxRigidBody::setMaxLinearVelocity(): threshold must be non-negative!");
	PX_CHECK_AND_RETURN(maxLinearVelocity <= PxReal(1.00000003e+16), "PxRigidBody::setMaxLinearVelocity(): maxLinearVelocity*maxLinearVelocity must be less than 1e16");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidBody::setMaxLinearVelocity() not allowed while simulation is running. Call will be ignored.")

	mCore.setMaxLinVelSq(maxLinearVelocity * maxLinearVelocity);
	UPDATE_PVD_PROPERTY_BODY
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxLinearVelocity, static_cast<PxRigidBody&>(*this), maxLinearVelocity)
}

template<class APIClass>
PxReal NpRigidBodyTemplate<APIClass>::getMaxLinearVelocity() const
{
	NP_READ_CHECK(RigidActorTemplateClass::getNpScene());

	return PxSqrt(mCore.getMaxLinVelSq());
}

}

#endif

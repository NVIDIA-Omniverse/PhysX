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

#ifndef EXT_JOINT_H
#define EXT_JOINT_H

#include "PxPhysics.h"
#include "extensions/PxConstraintExt.h"
#include "PxRigidStatic.h"
#include "PxRigidDynamic.h"
#include "PxArticulationLink.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxScene.h"

#include "foundation/PxAllocator.h"
#include "foundation/PxMathUtils.h"
#include "CmUtils.h"
#include "ExtJointData.h"

#if PX_SUPPORT_PVD
	#include "pvd/PxPvdSceneClient.h"
	#include "ExtPvd.h"
	#include "PxPvdClient.h"
#endif

#include "omnipvd/OmniPvdPxExtensionsSampler.h"

namespace physx
{
// PX_SERIALIZATION
	class PxDeserializationContext;

PxConstraint* resolveConstraintPtr(PxDeserializationContext& v, PxConstraint* old, PxConstraintConnector* connector, PxConstraintShaderTable& shaders);

// ~PX_SERIALIZATION

namespace Ext
{
	PX_FORCE_INLINE float computeSwingAngle(float swingYZ, float swingW)
	{
		return 4.0f * PxAtan2(swingYZ, 1.0f + swingW);	// tan (t/2) = sin(t)/(1+cos t), so this is the quarter angle
	}

	template<class JointType, class DataType>
	PX_FORCE_INLINE	JointType* createJointT(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1, const PxConstraintShaderTable& shaders)
	{
		JointType* j;
		PX_NEW_SERIALIZED(j, JointType)(physics.getTolerancesScale(), actor0, localFrame0, actor1, localFrame1);

		if(!physics.createConstraint(actor0, actor1, *j, shaders, sizeof(DataType)))
			PX_DELETE(j);

#if PX_SUPPORT_OMNI_PVD
		if (j)
		{
			omniPvdInitJoint(j);
		}
#endif

		return j;
	}

// PX_SERIALIZATION
	template<class JointType>
	PX_FORCE_INLINE	JointType*	createJointObject(PxU8*& address, PxDeserializationContext& context)
	{
		JointType* obj = PX_PLACEMENT_NEW(address, JointType(PxBaseFlag::eIS_RELEASABLE));
		address += sizeof(JointType);	
		obj->importExtraData(context);
		obj->resolveReferences(context);
		return obj;
	}
//~PX_SERIALIZATION

	template <class Base, class DataClass, class ValueStruct>
	class JointT : public Base, public PxConstraintConnector, public PxUserAllocated
	{
    public:
// PX_SERIALIZATION
						JointT(PxBaseFlags baseFlags) : Base(baseFlags)	{}

				void	exportExtraData(PxSerializationContext& stream)
				{
					if(mData)
					{
						stream.alignData(PX_SERIAL_ALIGN);
						stream.writeData(mData, sizeof(DataClass));
					}
					stream.writeName(mName);
				}

				void	importExtraData(PxDeserializationContext& context)
				{
					if(mData)
						mData = context.readExtraData<DataClass, PX_SERIAL_ALIGN>();
					context.readName(mName);
				}

		virtual void	preExportDataReset(){}
		virtual	void	requiresObjects(PxProcessPxBaseCallback& c)
		{			
			c.process(*mPxConstraint);
			
			{
				PxRigidActor* a0 = NULL;
				PxRigidActor* a1 = NULL;
				mPxConstraint->getActors(a0,a1);
				
				if (a0)
				{
					c.process(*a0);
				}
				if (a1)
				{
					c.process(*a1);
				}
			}
		}	
//~PX_SERIALIZATION
		
#if PX_SUPPORT_PVD
		// PxConstraintConnector
		virtual bool updatePvdProperties(physx::pvdsdk::PvdDataStream& pvdConnection, const PxConstraint* c, PxPvdUpdateType::Enum updateType) const	PX_OVERRIDE
		{
			if(updateType == PxPvdUpdateType::UPDATE_SIM_PROPERTIES)
			{
				Ext::Pvd::simUpdate<Base>(pvdConnection, *this);
				return true;
			}
			else if(updateType == PxPvdUpdateType::UPDATE_ALL_PROPERTIES)
			{
				Ext::Pvd::updatePvdProperties<Base, ValueStruct>(pvdConnection, *this);
				return true;
			}
			else if(updateType == PxPvdUpdateType::CREATE_INSTANCE)
			{
				Ext::Pvd::createPvdInstance<Base>(pvdConnection, *c, *this);
				return true;
			}
			else if(updateType == PxPvdUpdateType::RELEASE_INSTANCE)
			{
				Ext::Pvd::releasePvdInstance(pvdConnection, *c, *this);
				return true;
			}
			return false;
		}
#else
		virtual bool updatePvdProperties(physx::pvdsdk::PvdDataStream&, const PxConstraint*, PxPvdUpdateType::Enum) const	PX_OVERRIDE
		{
			return false;
		}
#endif

		// PxConstraintConnector
		virtual void updateOmniPvdProperties() const PX_OVERRIDE
		{
		}

		// PxJoint
		virtual void setActors(PxRigidActor* actor0, PxRigidActor* actor1)	PX_OVERRIDE
		{	
			//TODO SDK-DEV
			//You can get the debugger stream from the NpScene
			//Ext::Pvd::setActors( stream, this, mPxConstraint, actor0, actor1 );
			PX_CHECK_AND_RETURN(actor0 != actor1, "PxJoint::setActors: actors must be different");
			PX_CHECK_AND_RETURN((actor0 && !actor0->is<PxRigidStatic>()) || (actor1 && !actor1->is<PxRigidStatic>()), "PxJoint::setActors: at least one actor must be non-static");

#if PX_SUPPORT_PVD
			PxScene* scene = getScene();
			if(scene)
			{
				//if pvd not connect data stream is NULL
				physx::pvdsdk::PvdDataStream* conn = scene->getScenePvdClient()->getClientInternal()->getDataStream();
				if( conn != NULL )
					Ext::Pvd::setActors(
					*conn,
					*this,
					*mPxConstraint,
					actor0, 
					actor1
					);
			}
#endif
			mPxConstraint->setActors(actor0, actor1);
			mData->c2b[0] = getCom(actor0).transformInv(mLocalPose[0]);
			mData->c2b[1] = getCom(actor1).transformInv(mLocalPose[1]);
			mPxConstraint->markDirty();

			OMNI_PVD_SET(PxJoint, actor0, static_cast<PxJoint&>(*this), actor0)
			OMNI_PVD_SET(PxJoint, actor1, static_cast<PxJoint&>(*this), actor1)
		}

		// PxJoint
		virtual	void getActors(PxRigidActor*& actor0, PxRigidActor*& actor1)	const	PX_OVERRIDE
		{	
			if(mPxConstraint)
				mPxConstraint->getActors(actor0,actor1);
			else
			{
				actor0 = NULL;
				actor1 = NULL;
			}
		}

		// this is the local pose relative to the actor, and we store internally the local
		// pose relative to the body 

		// PxJoint
		virtual void setLocalPose(PxJointActorIndex::Enum actor, const PxTransform& pose)	PX_OVERRIDE
		{
			PX_CHECK_AND_RETURN(pose.isSane(), "PxJoint::setLocalPose: transform is invalid");
			const PxTransform p = pose.getNormalized();
			mLocalPose[actor] = p;
			mData->c2b[actor] = getCom(actor).transformInv(p); 
			mPxConstraint->markDirty();

			OMNI_PVD_SET(PxJoint, actor0LocalPose, static_cast<PxJoint&>(*this), mLocalPose[0])
			OMNI_PVD_SET(PxJoint, actor1LocalPose, static_cast<PxJoint&>(*this), mLocalPose[1])
		}

		// PxJoint
		virtual	PxTransform	getLocalPose(PxJointActorIndex::Enum actor) const	PX_OVERRIDE
		{	
			return mLocalPose[actor];
		}

		static	PxTransform	getGlobalPose(const PxRigidActor* actor)
		{
			if(!actor)
				return PxTransform(PxIdentity);
			return actor->getGlobalPose();
		}

		static	void	getActorVelocity(const PxRigidActor* actor, PxVec3& linear, PxVec3& angular)
		{
			if(!actor || actor->is<PxRigidStatic>())
			{
				linear = angular = PxVec3(0.0f);
				return;
			}
			
			linear = static_cast<const PxRigidBody*>(actor)->getLinearVelocity();
			angular = static_cast<const PxRigidBody*>(actor)->getAngularVelocity();
		}

		// PxJoint
		virtual	PxTransform	getRelativeTransform()	const	PX_OVERRIDE
		{
			PxRigidActor* actor0, * actor1;
			mPxConstraint->getActors(actor0, actor1);
			const PxTransform t0 = getGlobalPose(actor0) * mLocalPose[0];
			const PxTransform t1 = getGlobalPose(actor1) * mLocalPose[1];
			return t0.transformInv(t1);
		}

		// PxJoint
		virtual	PxVec3	getRelativeLinearVelocity()	const	PX_OVERRIDE
		{
			PxRigidActor* actor0, * actor1;
			PxVec3 l0, a0, l1, a1;
			mPxConstraint->getActors(actor0, actor1);

			PxTransform t0 = getCom(actor0), t1 = getCom(actor1);
			getActorVelocity(actor0, l0, a0);
			getActorVelocity(actor1, l1, a1);

			PxVec3 p0 = t0.q.rotate(mLocalPose[0].p), 
				   p1 = t1.q.rotate(mLocalPose[1].p);
			return t0.transformInv(l1 - a1.cross(p1) - l0 + a0.cross(p0));
		}

		// PxJoint
		virtual	PxVec3	getRelativeAngularVelocity()	const	PX_OVERRIDE
		{
			PxRigidActor* actor0, * actor1;
			PxVec3 l0, a0, l1, a1;
			mPxConstraint->getActors(actor0, actor1);

			PxTransform t0 = getCom(actor0);
			getActorVelocity(actor0, l0, a0);
			getActorVelocity(actor1, l1, a1);

			return t0.transformInv(a1 - a0);
		}

		// PxJoint
		virtual	void setBreakForce(PxReal force, PxReal torque)	PX_OVERRIDE
		{
			PX_CHECK_AND_RETURN(PxIsFinite(force) && PxIsFinite(torque), "PxJoint::setBreakForce: invalid float");
			mPxConstraint->setBreakForce(force,torque);

			OMNI_PVD_SET(PxJoint, breakForce, static_cast<PxJoint&>(*this), force)
			OMNI_PVD_SET(PxJoint, breakTorque, static_cast<PxJoint&>(*this), torque)
		}

		// PxJoint
		virtual	void getBreakForce(PxReal& force, PxReal& torque)	const	PX_OVERRIDE
		{
			mPxConstraint->getBreakForce(force,torque);
		}

		// PxJoint
		virtual	void setConstraintFlags(PxConstraintFlags flags)	PX_OVERRIDE
		{
			mPxConstraint->setFlags(flags);

			OMNI_PVD_SET(PxJoint, constraintFlags, static_cast<PxJoint&>(*this), flags)
		}

		// PxJoint
		virtual	void setConstraintFlag(PxConstraintFlag::Enum flag, bool value)	PX_OVERRIDE
		{
			mPxConstraint->setFlag(flag, value);

			OMNI_PVD_SET(PxJoint, constraintFlags, static_cast<PxJoint&>(*this), getConstraintFlags())
		}

		// PxJoint
		virtual	PxConstraintFlags getConstraintFlags()	const	PX_OVERRIDE
		{
			return mPxConstraint->getFlags();
		}

		// PxJoint
		virtual	void setInvMassScale0(PxReal invMassScale)	PX_OVERRIDE
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invMassScale) && invMassScale>=0, "PxJoint::setInvMassScale0: scale must be non-negative");
			mData->invMassScale.linear0 = invMassScale;
			mPxConstraint->markDirty();

			OMNI_PVD_SET(PxJoint, invMassScale0, static_cast<PxJoint&>(*this), invMassScale)
		}

		// PxJoint
		virtual	PxReal getInvMassScale0() const	PX_OVERRIDE
		{
			return mData->invMassScale.linear0;
		}

		// PxJoint
		virtual	void setInvInertiaScale0(PxReal invInertiaScale)	PX_OVERRIDE
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invInertiaScale) && invInertiaScale>=0, "PxJoint::setInvInertiaScale0: scale must be non-negative");
			mData->invMassScale.angular0 = invInertiaScale;
			mPxConstraint->markDirty();

			OMNI_PVD_SET(PxJoint, invInertiaScale0, static_cast<PxJoint&>(*this), invInertiaScale)
		}

		// PxJoint
		virtual	PxReal getInvInertiaScale0() const	PX_OVERRIDE
		{
			return mData->invMassScale.angular0;
		}

		// PxJoint
		virtual	void setInvMassScale1(PxReal invMassScale)	PX_OVERRIDE
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invMassScale) && invMassScale>=0, "PxJoint::setInvMassScale1: scale must be non-negative");
			mData->invMassScale.linear1 = invMassScale;
			mPxConstraint->markDirty();

			OMNI_PVD_SET(PxJoint, invMassScale1, static_cast<PxJoint&>(*this), invMassScale)
		}

		// PxJoint
		virtual	PxReal getInvMassScale1() const	PX_OVERRIDE
		{
			return mData->invMassScale.linear1;
		}

		// PxJoint
		virtual	void setInvInertiaScale1(PxReal invInertiaScale)	PX_OVERRIDE
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invInertiaScale) && invInertiaScale>=0, "PxJoint::setInvInertiaScale: scale must be non-negative");
			mData->invMassScale.angular1 = invInertiaScale;
			mPxConstraint->markDirty();

			OMNI_PVD_SET(PxJoint, invInertiaScale1, static_cast<PxJoint&>(*this), invInertiaScale)
		}

		// PxJoint
		virtual	PxReal getInvInertiaScale1() const	PX_OVERRIDE
		{
			return mData->invMassScale.angular1;
		}

		// PxJoint
		virtual	PxConstraint* getConstraint()	const	PX_OVERRIDE
		{
			return mPxConstraint;
		}

		// PxJoint
		virtual	void setName(const char* name)	PX_OVERRIDE
		{
			mName = name;
#if PX_SUPPORT_OMNI_PVD
			const char* n = name ? name : "";
			PxU32 nLen = PxU32(strlen(n)) + 1;
			OMNI_PVD_SETB(PxJoint, name, static_cast<PxJoint&>(*this), n, nLen)
#endif
		}

		// PxJoint
		virtual	const char* getName()	const	PX_OVERRIDE
		{
			return mName;
		}

		// PxJoint
		virtual	void release()	PX_OVERRIDE
		{
			mPxConstraint->release();
		}

		// PxJoint
		virtual	PxScene* getScene() const	PX_OVERRIDE
		{
			return mPxConstraint ? mPxConstraint->getScene() : NULL;
		}

		// PxConstraintConnector
		virtual	void onComShift(PxU32 actor)	PX_OVERRIDE
		{
			mData->c2b[actor] = getCom(actor).transformInv(mLocalPose[actor]); 
			markDirty();
		}

		// PxConstraintConnector
		virtual	void onOriginShift(const PxVec3& shift)	PX_OVERRIDE
		{
			PxRigidActor* a[2];
			mPxConstraint->getActors(a[0], a[1]);

			if (!a[0])
			{
				mLocalPose[0].p -= shift;
				mData->c2b[0].p -= shift;
				markDirty();
			}
			else if (!a[1])
			{
				mLocalPose[1].p -= shift;
				mData->c2b[1].p -= shift;
				markDirty();
			}
		}

		// PxConstraintConnector
		virtual	void* prepareData()	PX_OVERRIDE
		{
			return mData;
		}

		// PxConstraintConnector
		virtual	void* getExternalReference(PxU32& typeID)	PX_OVERRIDE
		{
			typeID = PxConstraintExtIDs::eJOINT;
			return static_cast<PxJoint*>( this );
		}

		// PxConstraintConnector
		virtual	PxBase* getSerializable()	PX_OVERRIDE
		{
			return this;
		}

		// PxConstraintConnector
		virtual	void onConstraintRelease()	PX_OVERRIDE
		{
			PX_FREE(mData);
			PX_DELETE_THIS;
		}

		// PxConstraintConnector
		virtual const void* getConstantBlock()	const	PX_OVERRIDE
		{
			return mData; 
		}

		virtual	void		connectToConstraint(PxConstraint* c)	PX_OVERRIDE
		{
			mPxConstraint = c;
		}

	private:
		PxTransform getCom(PxU32 index) const
		{
			PxRigidActor* a[2];
			mPxConstraint->getActors(a[0],a[1]);
			return getCom(a[index]);
		}

		PxTransform getCom(PxRigidActor* actor) const
		{
			if (!actor)
				return PxTransform(PxIdentity);
			else if (actor->getType() == PxActorType::eRIGID_DYNAMIC || actor->getType() == PxActorType::eARTICULATION_LINK)
				return static_cast<PxRigidBody*>(actor)->getCMassLocalPose();
			else
			{
				PX_ASSERT(actor->getType() == PxActorType::eRIGID_STATIC);
				return static_cast<PxRigidStatic*>(actor)->getGlobalPose().getInverse();
			}
		}

	protected:

		JointT(PxType concreteType, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1, const char* name) :
			Base			(concreteType, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
			mName			(NULL),
			mPxConstraint	(NULL)
		{
			PX_UNUSED(name);
			Base::userData = NULL;

			const PxU32 size = sizeof(DataClass);
			JointData* data = reinterpret_cast<JointData*>(PX_ALLOC(size, name));
			PxMarkSerializedMemory(data, size);

			mLocalPose[0]				= localFrame0.getNormalized();
			mLocalPose[1]				= localFrame1.getNormalized();
			data->c2b[0]				= getCom(actor0).transformInv(mLocalPose[0]);
			data->c2b[1]				= getCom(actor1).transformInv(mLocalPose[1]);
			data->invMassScale.linear0	= 1.0f;
			data->invMassScale.angular0	= 1.0f;
			data->invMassScale.linear1	= 1.0f;
			data->invMassScale.angular1	= 1.0f;

			mData = data;
		}

		virtual ~JointT()
		{
			if(Base::getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
				PX_FREE(mData);

			OMNI_PVD_DESTROY(PxJoint, static_cast<PxJoint&>(*this))
		}

		PX_FORCE_INLINE DataClass& data() const
		{
			return *static_cast<DataClass*>(mData);
		}

		PX_FORCE_INLINE	void markDirty()
		{ 
			mPxConstraint->markDirty();
		}

		void wakeUpActors()
		{
			PxRigidActor* a[2];
			mPxConstraint->getActors(a[0], a[1]);
			for(PxU32 i = 0; i < 2; i++)
			{
				if(a[i] && a[i]->getScene())
				{
					if(a[i]->getType() == PxActorType::eRIGID_DYNAMIC)
					{
						PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(a[i]);
						if(!(rd->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
						{
							const PxScene* scene = rd->getScene();
							const PxReal wakeCounterResetValue = scene->getWakeCounterResetValue();
							const PxReal wakeCounter = rd->getWakeCounter();
							if(wakeCounter < wakeCounterResetValue)
							{
								rd->wakeUp();
							}
						}
					}
					else if(a[i]->getType() == PxActorType::eARTICULATION_LINK)
					{
						PxArticulationLink* link = reinterpret_cast<PxArticulationLink*>(a[i]);
						PxArticulationReducedCoordinate& articulation = link->getArticulation();
						const PxReal wakeCounter = articulation.getWakeCounter();
						const PxScene* scene = link->getScene();
						const PxReal wakeCounterResetValue = scene->getWakeCounterResetValue();
						if(wakeCounter < wakeCounterResetValue)
						{
							articulation.wakeUp();
						}
					}
				}
			}
		}

		PX_FORCE_INLINE	PxQuat	getTwistOrSwing(bool needTwist) const
		{
			const PxQuat q = getRelativeTransform().q;
			// PT: TODO: we don't need to compute both quats here
			PxQuat swing, twist;
			PxSeparateSwingTwist(q, swing, twist);
			return needTwist ? twist : swing;
		}

		PxReal	getTwistAngle_Internal() const
		{
			const PxQuat twist = getTwistOrSwing(true);

			// PT: the angle-axis formulation creates the quat like this:
			//
			//	const float a = angleRadians * 0.5f;
			//	const float s = PxSin(a);
			//	w = PxCos(a);
			//	x = unitAxis.x * s;
			//	y = unitAxis.y * s;
			//	z = unitAxis.z * s;
			//
			// With the twist axis = (1;0;0) this gives:
			//
			//	w = PxCos(angleRadians * 0.5f);
			//	x = PxSin(angleRadians * 0.5f);
			//	y = 0.0f;
			//	z = 0.0f;
			//
			// Thus the quat's "getAngle" function returns:
			//
			//	angle = PxAcos(w) * 2.0f;
			//
			// PxAcos will return an angle between 0 and PI in radians, so "getAngle" will return an angle between 0 and PI*2.

			PxReal angle = twist.getAngle();

			if(twist.x<0.0f)
				angle = -angle;

			return angle;
		}

		PxReal	getSwingYAngle_Internal()	const
		{
			PxQuat swing = getTwistOrSwing(false);

			if(swing.w < 0.0f)		// choose the shortest rotation
				swing = -swing;

			const PxReal angle = computeSwingAngle(swing.y, swing.w);
			PX_ASSERT(angle>-PxPi && angle<=PxPi);				// since |y| < w+1, the atan magnitude is < PI/4
			return angle;
		}

		PxReal	getSwingZAngle_Internal()	const
		{
			PxQuat swing = getTwistOrSwing(false);

			if(swing.w < 0.0f)		// choose the shortest rotation
				swing = -swing;

			const PxReal angle = computeSwingAngle(swing.z, swing.w);
			PX_ASSERT(angle>-PxPi && angle <= PxPi);			// since |y| < w+1, the atan magnitude is < PI/4
			return angle;
		}

		const char*		mName;
		PxTransform		mLocalPose[2];
		PxConstraint*	mPxConstraint;
		JointData*		mData;
	};

#if PX_SUPPORT_OMNI_PVD
	void omniPvdSetBaseJointParams(PxJoint& joint, PxJointConcreteType::Enum cType);
	template<typename JointType> void omniPvdInitJoint(JointType* joint);
#endif

} // namespace Ext

}

#endif

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

#ifndef NP_RIGID_ACTOR_TEMPLATE_H
#define NP_RIGID_ACTOR_TEMPLATE_H

#include "NpActorTemplate.h"
#include "NpShapeManager.h"
#include "NpConstraint.h"
#include "NpFactory.h"
#include "NpActor.h"

// PX_SERIALIZATION
#include "foundation/PxErrors.h"
//~PX_SERIALIZATION
#include "omnipvd/OmniPvdPxSampler.h"

namespace physx
{
template<class APIClass>
class NpRigidActorTemplate : public NpActorTemplate<APIClass>
{
private:
	typedef NpActorTemplate<APIClass> ActorTemplateClass;

public:
// PX_SERIALIZATION
											NpRigidActorTemplate(PxBaseFlags baseFlags) : ActorTemplateClass(baseFlags), mShapeManager(PxEmpty)//, mIndex(0xFFFFFFFF)
											{
												NpBase::mFreeSlot = 0xFFFFFFFF;
											}
	virtual			void					requiresObjects(PxProcessPxBaseCallback& c);
					void					preExportDataReset();
	virtual			void					exportExtraData(PxSerializationContext& context);
					void					importExtraData(PxDeserializationContext& context);
					void					resolveReferences(PxDeserializationContext& context);
//~PX_SERIALIZATION
	virtual									~NpRigidActorTemplate();

	// The rule is: If an API method is used somewhere in here, it has to be redeclared, else GCC whines

	// PxActor
					void					removeShapes(PxSceneQuerySystem* sqManager);
	virtual			PxActorType::Enum		getType() const = 0;
	virtual			PxBounds3				getWorldBounds(float inflation=1.01f) const	PX_OVERRIDE;
	virtual			void					setActorFlag(PxActorFlag::Enum flag, bool value)	PX_OVERRIDE;
	virtual			void					setActorFlags(PxActorFlags inFlags)	PX_OVERRIDE;
	//~PxActor

	// PxRigidActor
	virtual			PxU32					getInternalActorIndex() const	PX_OVERRIDE;
	virtual			bool					attachShape(PxShape& s)	PX_OVERRIDE;
	virtual			void					detachShape(PxShape& s, bool wakeOnLostTouch)	PX_OVERRIDE;
	virtual			PxU32					getNbShapes() const	PX_OVERRIDE;
	virtual			PxU32					getShapes(PxShape** buffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE;
	virtual			PxU32					getNbConstraints() const	PX_OVERRIDE;
	virtual			PxU32					getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE;
	//~PxRigidActor
											NpRigidActorTemplate(PxType concreteType, PxBaseFlags baseFlags, NpType::Enum type);

	// not optimal but the template alternative is hardly more readable and perf is not that critical here
	virtual			void					switchToNoSim() { PX_ASSERT(false); }
	virtual			void					switchFromNoSim() { PX_ASSERT(false); }

	PX_FORCE_INLINE	NpShapeManager&			getShapeManager()		{ return mShapeManager; }
	PX_FORCE_INLINE	const NpShapeManager&	getShapeManager() const { return mShapeManager; }

					void					updateShaderComs();

	// index for the NpScene rigid dynamic or static array
	// PT: note that this index changes during the lifetime of the object, e.g. when another object
	// is removed and swaps happen in the scene's mRigidStatics/mRigidDynamics arrays.
	PX_FORCE_INLINE PxU32					getRigidActorArrayIndex()			const	{ return NpBase::mFreeSlot;		}
	PX_FORCE_INLINE void					setRigidActorArrayIndex(PxU32 index)		{ NpBase::mFreeSlot = index;	}
//	PX_FORCE_INLINE PxU32					getRigidActorArrayIndex()			const	{ return mIndex;				}
//	PX_FORCE_INLINE void					setRigidActorArrayIndex(PxU32 index)		{ mIndex = index;				}

	PX_FORCE_INLINE PxU32					getRigidActorSceneIndex()			const	{ return NpBase::getBaseIndex();	}
	PX_FORCE_INLINE void					setRigidActorSceneIndex(PxU32 index)		{ NpBase::setBaseIndex(index);		}

					bool					resetFiltering_(NpActor& ro, Sc::RigidCore& core, PxShape*const* shapes, PxU32 shapeCount);

#if PX_ENABLE_DEBUG_VISUALIZATION
public:
					void					visualize(PxRenderOutput& out, NpScene& scene, float scale)	const;
#else
					PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
protected:
	PX_FORCE_INLINE void					setActorSimFlag(bool value);

					NpShapeManager			mShapeManager;
					// PT: note that this index changes during the lifetime of the object, e.g. when another object
					// is removed and swaps happen in the scene's mRigidStatics/mRigidDynamics arrays.
//					PxU32					mIndex;    // index for the NpScene rigid dynamic or static array
					// PT: TODO: reduce padding
};

// PX_SERIALIZATION

template<class APIClass>
void NpRigidActorTemplate<APIClass>::requiresObjects(PxProcessPxBaseCallback& c)
{
	// export shapes
	PxU32 nbShapes = mShapeManager.getNbShapes();
	for(PxU32 i=0;i<nbShapes;i++)
	{
		NpShape* np = mShapeManager.getShapes()[i];
		c.process(*np);
	}
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::preExportDataReset() 
{
	//Clearing the aggregate ID for serialization so we avoid having a stale 
	//reference after deserialization. The aggregate ID get's reset on readding to the 
	//scene anyway.
	Sc::ActorCore& actorCore = NpActor::getActorCore();
	actorCore.setAggregateID(PX_INVALID_U32);
	mShapeManager.preExportDataReset();
	//mIndex = 0xFFFFFFFF;
	NpBase::mFreeSlot = 0xFFFFFFFF;
	NpBase::setBaseIndex(NP_UNUSED_BASE_INDEX);
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::exportExtraData(PxSerializationContext& context)
{
	mShapeManager.exportExtraData(context);
	ActorTemplateClass::exportExtraData(context);
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::importExtraData(PxDeserializationContext& context)
{
	mShapeManager.importExtraData(context);
	ActorTemplateClass::importExtraData(context);
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::resolveReferences(PxDeserializationContext& context)
{
	const PxU32 nbShapes = mShapeManager.getNbShapes();
	NpShape** shapes = const_cast<NpShape**>(mShapeManager.getShapes());
	for(PxU32 j=0;j<nbShapes;j++)
	{						
		context.translatePxBase(shapes[j]);
		shapes[j]->onActorAttach(*this);
	}

	ActorTemplateClass::resolveReferences(context);
}

//~PX_SERIALIZATION

template<class APIClass>
NpRigidActorTemplate<APIClass>::NpRigidActorTemplate(PxType concreteType, PxBaseFlags baseFlags, NpType::Enum type) :
	ActorTemplateClass	(concreteType, baseFlags, type)
	//mIndex				(0xffffffff)
{
	NpBase::mFreeSlot = 0xFFFFFFFF;
}

template<class APIClass>
NpRigidActorTemplate<APIClass>::~NpRigidActorTemplate()
{
	// TODO: no mechanism for notifying shaders of actor destruction yet
}

template<class APIClass>
PxU32 NpRigidActorTemplate<APIClass>::getInternalActorIndex() const
{
	NP_READ_CHECK(ActorTemplateClass::getNpScene());

	const PxU32 index = NpBase::getBaseIndex();
	return index!=NP_UNUSED_BASE_INDEX ? index : 0xffffffff;
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::removeShapes(PxSceneQuerySystem* sqManager)
{
	if(mShapeManager.getPruningStructure())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxRigidActor::release: Actor is part of a pruning structure, pruning structure is now invalid!");
		mShapeManager.getPruningStructure()->invalidate(this);
	}

	mShapeManager.detachAll(sqManager, *this);
}

template<class APIClass>
bool NpRigidActorTemplate<APIClass>::attachShape(PxShape& shape)
{
	NpScene* npScene = ActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);
	NpShape& npShape = static_cast<NpShape&>(shape);
	PX_CHECK_AND_RETURN_VAL(!static_cast<NpShape&>(shape).isExclusive() || shape.getActor()==NULL, "PxRigidActor::attachShape: shape must be shared or unowned", false);
	PX_CHECK_AND_RETURN_VAL(!(npShape.getCore().getCore().mShapeCoreFlags & PxShapeCoreFlag::eSOFT_BODY_SHAPE), "PxRigidActor::attachShape() not allowed to attach a soft body shape to a rigid actor", false);
	PX_CHECK_AND_RETURN_VAL(!(npShape.getCore().getCore().mShapeCoreFlags & PxShapeCoreFlag::eCLOTH_SHAPE), "PxRigidActor::attachShape() not allowed to attach a cloth shape to a rigid actor", false);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(npScene, "PxRigidActor::attachShape() not allowed while simulation is running. Call will be ignored.", false);

	PX_SIMD_GUARD
	// invalidate the pruning structure if the actor bounds changed
	if (mShapeManager.getPruningStructure())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxRigidActor::attachShape: Actor is part of a pruning structure, pruning structure is now invalid!");
		mShapeManager.getPruningStructure()->invalidate(this);
	}

	mShapeManager.attachShape(npShape, *this);

	OMNI_PVD_ADD(PxRigidActor, shapes, static_cast<PxRigidActor&>(*this), shape)

	return true;
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::detachShape(PxShape& shape, bool wakeOnLostTouch)
{
	NpScene* npScene = ActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidActor::detachShape() not allowed while simulation is running. Call will be ignored.")

	OMNI_PVD_REMOVE(PxRigidActor, shapes, static_cast<PxRigidActor&>(*this), shape)	//this needs to happen before the actual detach happens below, because that detach might actually delete the shape, which invalidates the shape handle.

	if (mShapeManager.getPruningStructure())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxRigidActor::detachShape: Actor is part of a pruning structure, pruning structure is now invalid!");
		mShapeManager.getPruningStructure()->invalidate(this);
	}

	if(!mShapeManager.detachShape(static_cast<NpShape&>(shape), *this, wakeOnLostTouch))
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxRigidActor::detachShape: shape is not attached to this actor!");
	}
}

template<class APIClass>
PxU32 NpRigidActorTemplate<APIClass>::getNbShapes() const
{
	NP_READ_CHECK(ActorTemplateClass::getNpScene());
	return mShapeManager.getNbShapes();
}

template<class APIClass>
PxU32 NpRigidActorTemplate<APIClass>::getShapes(PxShape** buffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(ActorTemplateClass::getNpScene());
	return mShapeManager.getShapes(buffer, bufferSize, startIndex);
}

template<class APIClass>
PxU32 NpRigidActorTemplate<APIClass>::getNbConstraints() const
{
	NP_READ_CHECK(ActorTemplateClass::getNpScene());
	return ActorTemplateClass::getNbConnectors(NpConnectorType::eConstraint);
}

template<class APIClass>
PxU32 NpRigidActorTemplate<APIClass>::getConstraints(PxConstraint** buffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(ActorTemplateClass::getNpScene());
	return ActorTemplateClass::template getConnectors<PxConstraint>(NpConnectorType::eConstraint, buffer, bufferSize, startIndex);  // Some people will love me for this one... The syntax is to be standard compliant and
																														// picky gcc won't compile without it. It is needed if you call a templated member function
																														// of a templated class
}

template<class APIClass>
PxBounds3 NpRigidActorTemplate<APIClass>::getWorldBounds(float inflation) const
{
	NP_READ_CHECK(ActorTemplateClass::getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(ActorTemplateClass::getNpScene(), "PxRigidActor::getWorldBounds() not allowed while simulation is running (except during PxScene::collide()).", PxBounds3::empty());
	PX_SIMD_GUARD;

	const PxBounds3 bounds = mShapeManager.getWorldBounds_(*this);
	PX_ASSERT(bounds.isValid());

	// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
	const PxVec3 center = bounds.getCenter();
	const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
	return PxBounds3::centerExtents(center, inflatedExtents);
}

template<class APIClass>
PX_FORCE_INLINE void NpRigidActorTemplate<APIClass>::setActorSimFlag(bool value)
{
	NpScene* scene = ActorTemplateClass::getNpScene();

	PxActorFlags oldFlags = ActorTemplateClass::getActorFlags();
	bool hadNoSimFlag = oldFlags.isSet(PxActorFlag::eDISABLE_SIMULATION);

	PX_CHECK_AND_RETURN((getType() != PxActorType::eARTICULATION_LINK) || (!value && !hadNoSimFlag), "PxActor::setActorFlag: PxActorFlag::eDISABLE_SIMULATION is only supported by PxRigidDynamic and PxRigidStatic objects.");

	if (hadNoSimFlag && (!value))
	{
		switchFromNoSim();
		ActorTemplateClass::setActorFlagsInternal(oldFlags & (~PxActorFlag::eDISABLE_SIMULATION));  // needs to be done before the code below to make sure the latest flags get picked up
		if (scene)
			NpActor::addConstraintsToScene();
	}
	else if ((!hadNoSimFlag) && value)
	{
		if (scene)
			NpActor::removeConstraintsFromScene();
		ActorTemplateClass::setActorFlagsInternal(oldFlags | PxActorFlag::eDISABLE_SIMULATION);
		switchToNoSim();
	}
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::setActorFlag(PxActorFlag::Enum flag, bool value)
{
	NpScene* npScene = ActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidActor::setActorFlag() not allowed while simulation is running. Call will be ignored.")

	if (flag == PxActorFlag::eDISABLE_SIMULATION)
		setActorSimFlag(value);
	
	ActorTemplateClass::setActorFlagInternal(flag, value);
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::setActorFlags(PxActorFlags inFlags)
{
	NpScene* npScene = ActorTemplateClass::getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidActor::setActorFlags() not allowed while simulation is running. Call will be ignored.")

	bool noSim = inFlags.isSet(PxActorFlag::eDISABLE_SIMULATION);
	setActorSimFlag(noSim);
	
	ActorTemplateClass::setActorFlagsInternal(inFlags);
}

template<class APIClass>
void NpRigidActorTemplate<APIClass>::updateShaderComs()
{
	NpConnectorIterator iter = ActorTemplateClass::getConnectorIterator(NpConnectorType::eConstraint);
	while (PxBase* ser = iter.getNext())
	{
		NpConstraint* c = static_cast<NpConstraint*>(ser);
		c->comShift(this);
	}
}

template<class APIClass>
bool NpRigidActorTemplate<APIClass>::resetFiltering_(NpActor& ro, Sc::RigidCore& core, PxShape*const* shapes, PxU32 shapeCount)
{
#if PX_CHECKED
	PX_CHECK_AND_RETURN_VAL(!(ro.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxScene::resetFiltering(): Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!", false);
	for(PxU32 i=0; i < shapeCount; i++)
	{
		PxRigidActor* ra = shapes[i]->getActor();
		if (ra != this)
		{
			bool found = false;
			if (ra == NULL)
			{
				NpShape*const* sh = mShapeManager.getShapes();
				for(PxU32 j=0; j < mShapeManager.getNbShapes(); j++)
				{
					if (sh[j] == shapes[i])
					{
						found = true;
						break;
					}
				}
			}

			PX_CHECK_AND_RETURN_VAL(found, "PxScene::resetFiltering(): specified shape not in actor!", false);
		}
		PX_CHECK_AND_RETURN_VAL(static_cast<NpShape*>(shapes[i])->getCore().getFlags() & (PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE), "PxScene::resetFiltering(): specified shapes not of type eSIMULATION_SHAPE or eTRIGGER_SHAPE!", false);
	}
#endif

	PxU32 sCount;
	if (shapes)
		sCount = shapeCount;
	else
		sCount = mShapeManager.getNbShapes();
	
	PX_ALLOCA(scShapes, NpShape*, sCount);
	if (scShapes)
	{
		if (shapes)  // the user specified the shapes
		{
			PxU32 sAccepted = 0;
			for(PxU32 i=0; i < sCount; i++)
				scShapes[sAccepted++] = static_cast<NpShape*>(shapes[i]);

			sCount = sAccepted;
		}
		else  // the user just specified the actor and the shapes are taken from the actor
		{
			NpShape* const* sh = mShapeManager.getShapes();
			PxU32 sAccepted = 0;
			for(PxU32 i=0; i < sCount; i++)
			{
				if(sh[i]->getCore().getFlags() & (PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
					scShapes[sAccepted++] = sh[i];
			}
			sCount = sAccepted;
		}

		if (sCount)
		{
			PX_ASSERT(!(core.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)));
			PX_ASSERT(!ro.isAPIWriteForbidden());
			PX_UNUSED(ro);

			// PT: TODO: rewrite this thing, we end up in getSimForShape() each time
			for(PxU32 i=0; i < sCount; i++)
				core.onShapeChange(scShapes[i]->getCore(), Sc::ShapeChangeNotifyFlag::eRESET_FILTERING);
		}
	}

	return true;
}

#if PX_ENABLE_DEBUG_VISUALIZATION
template<class APIClass>
void NpRigidActorTemplate<APIClass>::visualize(PxRenderOutput& out, NpScene& scene, float scale) const
{
	mShapeManager.visualize(out, scene, *this, scale);
}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

}

#endif

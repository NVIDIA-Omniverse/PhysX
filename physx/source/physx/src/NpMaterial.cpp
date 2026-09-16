// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"
#include "omnipvd/NpOmniPvdSetData.h"

using namespace physx;
using namespace Cm;

NpMaterial::NpMaterial(const PxsMaterialCore& desc) :
	PxMaterial(PxConcreteType::eMATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMaterial(desc)
{
	mMaterial.mMaterial = this;  // back-reference	
}

NpMaterial::~NpMaterial()
{
	OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, static_cast<PxMaterial &>(*this))

	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpMaterial::resolveReferences(PxDeserializationContext&)
{
	// ### this one could be automated if NpMaterial would inherit from MaterialCore
	// ### well actually in that case the pointer would not even be needed....
	mMaterial.mMaterial = this;	// Resolve MaterialCore::mMaterial

	// Maybe not the best place to do it but it has to be done before the shapes resolve material indices
	// since the material index translation table is needed there. This requires that the materials have
	// been added to the table already.
	NpPhysics::getInstance().addMaterial(this);
}

void NpMaterial::onRefCountZero()
{
	void* ud = userData;	

	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseMaterialToPool(*this);
	else
		this->~NpMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpMaterial* NpMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpMaterial* obj = PX_PLACEMENT_NEW(address, NpMaterial(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpMaterial);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpMaterial::release()
{
	RefCountable_decRefCount(*this);
}

void NpMaterial::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 NpMaterial::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

PX_INLINE void NpMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setDynamicFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxMaterial::setDynamicFriction: invalid float");
	mMaterial.dynamicFriction = x;
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, dynamicFriction, static_cast<PxMaterial &>(*this), x)
}

PxReal NpMaterial::getDynamicFriction() const
{
	return mMaterial.dynamicFriction;
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setStaticFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxMaterial::setStaticFriction: invalid float");
	mMaterial.staticFriction = x;
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, staticFriction, static_cast<PxMaterial &>(*this), x)
}

PxReal NpMaterial::getStaticFriction() const
{
	return mMaterial.staticFriction;
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setRestitution(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxMaterial::setRestitution: invalid float");
	PX_CHECK_AND_RETURN(x <= 1.0f, "PxMaterial::setRestitution: Restitution value has to be smaller or equal 1.0!");
	if (x > 1.0f)
	{
		x = PxMin(1.0f, x);
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxMaterial::setRestitution: Invalid value %f was clamped to 1.0!", PxF64(x));
	}
	mMaterial.restitution = x;
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitution, static_cast<PxMaterial &>(*this), x)
}

PxReal NpMaterial::getRestitution() const
{
	return mMaterial.restitution;
}

/////////////////////////////////////////////////////////////////////////////////

void NpMaterial::setDamping(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x) && x >= 0.f, "PxMaterial::setDamping: invalid float. Must be >= 0");

	mMaterial.damping = x;
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, damping, static_cast<PxMaterial &>(*this), x)
}

PxReal NpMaterial::getDamping() const
{
	return mMaterial.damping;
}

/////////////////////////////////////////////////////////////////////////////////

void NpMaterial::setFlag(PxMaterialFlag::Enum flag, bool value)
{
	if (value)
		mMaterial.flags |= flag;
	else
		mMaterial.flags &= ~PxMaterialFlags(flag);
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, flags, static_cast<PxMaterial &>(*this), mMaterial.flags)
}

void NpMaterial::setFlags(PxMaterialFlags inFlags)
{
	mMaterial.flags = inFlags;
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, flags, static_cast<PxMaterial &>(*this), mMaterial.flags)
}

PxMaterialFlags NpMaterial::getFlags() const
{
	return mMaterial.flags;
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setFrictionCombineMode(PxCombineMode::Enum x)
{
	mMaterial.setFrictionCombineMode(x);
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, frictionCombineMode, static_cast<PxMaterial &>(*this), x)
}

PxCombineMode::Enum NpMaterial::getFrictionCombineMode() const
{
	return mMaterial.getFrictionCombineMode();
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setRestitutionCombineMode(PxCombineMode::Enum x)
{
	mMaterial.setRestitutionCombineMode(x);
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitutionCombineMode, static_cast<PxMaterial &>(*this), x)
}

PxCombineMode::Enum NpMaterial::getRestitutionCombineMode() const
{
	return mMaterial.getRestitutionCombineMode();
}

///////////////////////////////////////////////////////////////////////////////
void NpMaterial::setDampingCombineMode(PxCombineMode::Enum combMode)
{
	mMaterial.setDampingCombineMode(combMode);
	updateMaterial();
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, dampingCombineMode, static_cast<PxMaterial &>(*this), combMode)
}

PxCombineMode::Enum NpMaterial::getDampingCombineMode() const
{
	return mMaterial.getDampingCombineMode();
}


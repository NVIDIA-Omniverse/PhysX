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

#include "NpMPMMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"

#if PX_SUPPORT_GPU_PHYSX
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
using namespace physx;
using namespace Cm;

NpMPMMaterial::NpMPMMaterial(const PxsMPMMaterialCore& desc) :
	PxMPMMaterial(PxConcreteType::eMPM_MATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMaterial(desc)
{
	mMaterial.mMaterial = this;  // back-reference	
}

NpMPMMaterial::~NpMPMMaterial()
{
	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpMPMMaterial::resolveReferences(PxDeserializationContext&)
{
	// ### this one could be automated if NpMaterial would inherit from MaterialCore
	// ### well actually in that case the pointer would not even be needed....
	mMaterial.mMaterial = this;	// Resolve MaterialCore::mMaterial

	// Maybe not the best place to do it but it has to be done before the shapes resolve material indices
	// since the material index translation table is needed there. This requires that the materials have
	// been added to the table already.
	// PT: TODO: missing line here?
}

void NpMPMMaterial::onRefCountZero()
{
	void* ud = userData;

	if (getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		NpFactory::getInstance().releaseMPMMaterialToPool(*this);
	}
	else
		this->~NpMPMMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpMPMMaterial* NpMPMMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpMPMMaterial* obj = PX_PLACEMENT_NEW(address, NpMPMMaterial(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpMPMMaterial);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpMPMMaterial::release()
{
	RefCountable_decRefCount(*this);
}

void NpMPMMaterial::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 NpMPMMaterial::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

PX_INLINE void NpMPMMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setFriction: invalid float");

	mMaterial.friction = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getFriction() const
{
	return mMaterial.friction;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setDamping(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setDamping: invalid float");
	mMaterial.damping = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getDamping() const
{
	return mMaterial.damping;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setStretchAndShearDamping(PxReal stretchAndShearDamping)
{
	mMaterial.stretchAndShearDamping = stretchAndShearDamping;
	updateMaterial();
}

PxReal NpMPMMaterial::getStretchAndShearDamping() const
{
	return mMaterial.stretchAndShearDamping;
}


void NpMPMMaterial::setRotationalDamping(PxReal rotationalDamping)
{
	mMaterial.rotationalDamping = rotationalDamping;
	updateMaterial();
}

PxReal NpMPMMaterial::getRotationalDamping() const
{
	return mMaterial.rotationalDamping;
}


void NpMPMMaterial::setDensity(PxReal density)
{
	mMaterial.density = density;
	updateMaterial();
}

PxReal NpMPMMaterial::getDensity() const
{
	return mMaterial.density;
}


void NpMPMMaterial::setMaterialModel(PxMPMMaterialModel::Enum materialModel)
{
	mMaterial.materialModel = materialModel;
	updateMaterial();
}

PxMPMMaterialModel::Enum NpMPMMaterial::getMaterialModel() const
{
	return mMaterial.materialModel;
}


void NpMPMMaterial::setCuttingFlags(PxMPMCuttingFlags cuttingFlags)
{
	mMaterial.cuttingFlags = cuttingFlags;
	updateMaterial();
}

PxMPMCuttingFlags NpMPMMaterial::getCuttingFlags() const
{
	return mMaterial.cuttingFlags;
}


void NpMPMMaterial::setSandFrictionAngle(PxReal sandFrictionAngle)
{
	mMaterial.sandFrictionAngle = sandFrictionAngle;
	updateMaterial();
}

PxReal NpMPMMaterial::getSandFrictionAngle() const
{
	return mMaterial.sandFrictionAngle;
}


void NpMPMMaterial::setYieldStress(PxReal yieldStress)
{
	mMaterial.yieldStress = yieldStress;
	updateMaterial();
}

PxReal NpMPMMaterial::getYieldStress() const
{
	return mMaterial.yieldStress;
}


void NpMPMMaterial::setIsPlastic(bool x)
{
	mMaterial.isPlastic = x;

	updateMaterial();
}

bool NpMPMMaterial::getIsPlastic() const
{
	return mMaterial.isPlastic != 0;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setYoungsModulus(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setYoungsModulus: invalid float");

	mMaterial.youngsModulus = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getYoungsModulus() const
{
	return mMaterial.youngsModulus;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setPoissons(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f && x <= 0.5f, "PxMPMMaterial::setPoissons: invalid float");

	mMaterial.poissonsRatio = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getPoissons() const
{
	return mMaterial.poissonsRatio;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setHardening(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setPoissons: invalid float");

	mMaterial.hardening = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getHardening() const
{
	return mMaterial.hardening;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setCriticalCompression(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setCriticalCompression: invalid float");

	mMaterial.criticalCompression = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getCriticalCompression() const
{
	return mMaterial.criticalCompression;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setCriticalStretch(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setCriticalStretch: invalid float");

	mMaterial.criticalStretch = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getCriticalStretch() const
{
	return mMaterial.criticalStretch;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setTensileDamageSensitivity(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setTensileDamageSensitivity: invalid float");

	mMaterial.tensileDamageSensitivity = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getTensileDamageSensitivity() const
{
	return mMaterial.tensileDamageSensitivity;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setCompressiveDamageSensitivity(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setCompressiveDamageSensitivity: invalid float");

	mMaterial.compressiveDamageSensitivity = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getCompressiveDamageSensitivity() const
{
	return mMaterial.compressiveDamageSensitivity;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setAttractiveForceResidual(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setAttractiveForceResidual: invalid float");

	mMaterial.attractiveForceResidual = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getAttractiveForceResidual() const
{
	return mMaterial.attractiveForceResidual;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setAdhesion(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setAdhesion: invalid float");
	mMaterial.adhesion = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getAdhesion() const
{
	return mMaterial.adhesion;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setGravityScale(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxMPMMaterial::setAdhesion: invalid float");
	mMaterial.gravityScale = x;

	updateMaterial();
}

PxReal NpMPMMaterial::getGravityScale() const
{
	return mMaterial.gravityScale;
}

///////////////////////////////////////////////////////////////////////////////

void NpMPMMaterial::setAdhesionRadiusScale(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMPMMaterial::setAdhesionRadiusScale: scale must be positive");
	mMaterial.adhesionRadiusScale = x;

	updateMaterial();
}
PxReal NpMPMMaterial::getAdhesionRadiusScale() const
{
	return mMaterial.adhesionRadiusScale;
}

//////////////////////////////////////////////////////////////////////////////

#endif
#endif

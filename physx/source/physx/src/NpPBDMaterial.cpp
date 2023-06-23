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

#include "NpPBDMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"

#if PX_SUPPORT_GPU_PHYSX
using namespace physx;
using namespace Cm;

NpPBDMaterial::NpPBDMaterial(const PxsPBDMaterialCore& desc) :
	PxPBDMaterial(PxConcreteType::ePBD_MATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMaterial(desc)
{
	mMaterial.mMaterial = this;  // back-reference	
}

NpPBDMaterial::~NpPBDMaterial()
{
	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpPBDMaterial::resolveReferences(PxDeserializationContext&)
{
	// ### this one could be automated if NpMaterial would inherit from MaterialCore
	// ### well actually in that case the pointer would not even be needed....
	mMaterial.mMaterial = this;	// Resolve MaterialCore::mMaterial

	// Maybe not the best place to do it but it has to be done before the shapes resolve material indices
	// since the material index translation table is needed there. This requires that the materials have
	// been added to the table already.
	// PT: TODO: missing line here?
}

void NpPBDMaterial::onRefCountZero()
{
	void* ud = userData;

	if (getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		NpFactory::getInstance().releasePBDMaterialToPool(*this);
	}
	else
		this->~NpPBDMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpPBDMaterial* NpPBDMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpPBDMaterial* obj = PX_PLACEMENT_NEW(address, NpPBDMaterial(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpPBDMaterial);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpPBDMaterial::release()
{
	RefCountable_decRefCount(*this);
}

void NpPBDMaterial::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 NpPBDMaterial::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

PX_INLINE void NpPBDMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setFriction: invalid float");

	mMaterial.friction = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getFriction() const
{
	return mMaterial.friction;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setViscosity(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setViscosity: invalid float");
	mMaterial.viscosity = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getViscosity() const
{
	return mMaterial.viscosity;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setDamping(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setDamping: invalid float");
	mMaterial.damping = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getDamping() const
{
	return mMaterial.damping;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setLift(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setLift: invalid float");
	mMaterial.lift = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getLift() const
{
	return mMaterial.lift;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setDrag(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setDrag: invalid float");

	mMaterial.drag = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getDrag() const
{
	return mMaterial.drag;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setCFLCoefficient(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 1.f, "PxPBDMaterial::setCFLCoefficient: invalid float");

	mMaterial.cflCoefficient = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getCFLCoefficient() const
{
	return mMaterial.cflCoefficient;
}


///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setVorticityConfinement(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setVorticityConfinement: invalid float");

	mMaterial.vorticityConfinement = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getVorticityConfinement() const
{
	return mMaterial.vorticityConfinement;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setSurfaceTension(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setSurfaceTension: invalid float");

	mMaterial.surfaceTension = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getSurfaceTension() const
{
	return mMaterial.surfaceTension;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setCohesion(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setCohesion: invalid float");

	mMaterial.cohesion = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getCohesion() const
{
	return mMaterial.cohesion;
}

//////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setAdhesion(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFLIPMaterial::setAdhesion: invalid float");
	mMaterial.adhesion = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getAdhesion() const
{
	return mMaterial.adhesion;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setGravityScale(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxFLIPMaterial::setAdhesion: invalid float");
	mMaterial.gravityScale = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getGravityScale() const
{
	return mMaterial.gravityScale;
}

//////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setAdhesionRadiusScale(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setAdhesionRadiusScale: invalid float");

	mMaterial.adhesionRadiusScale = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getAdhesionRadiusScale() const
{
	return mMaterial.adhesionRadiusScale;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setParticleFrictionScale(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxPBDMaterial::setParticleFrictionScale: invalid float");

	mMaterial.particleFrictionScale = x;

	updateMaterial();
}

PxReal NpPBDMaterial::getParticleFrictionScale() const
{
	return mMaterial.particleFrictionScale;
}

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::setParticleAdhesionScale(PxReal adhesionScale)
{
	PX_CHECK_AND_RETURN(adhesionScale >= 0.f, "PxPBDMaterial::setParticleAdhesionScale: adhesion value must be >= 0");

	mMaterial.particleAdhesionScale = adhesionScale;

	updateMaterial();
}


PxReal NpPBDMaterial::getParticleAdhesionScale() const
{
	return mMaterial.particleAdhesionScale;
}

///////////////////////////////////////////////////////////////////////////////
#endif

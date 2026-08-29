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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#if PX_SUPPORT_OMNI_PVD

#include "OmniPvdPxExtensionsSampler.h"
#include "omnipvd/PxOmniPvd.h"

#include "PxPhysics.h"
#include "PxScene.h"
#include "PxShape.h"
#include "PxConstraint.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxCustomGeometry.h"
#include "extensions/PxCustomGeometryExt.h"
#include "extensions/PxJoint.h"
#include "extensions/PxConstraintExt.h"

// The joint dispatch needs the internal concrete joint classes (physx::Ext::D6Joint, ...)
// and brings in the omniPvdInitJoint<T> friend declarations specialized per joint .cpp.
#include "ExtD6Joint.h"
#include "ExtRevoluteJoint.h"
#include "ExtPrismaticJoint.h"
#include "ExtFixedJoint.h"
#include "ExtDistanceJoint.h"
#include "ExtSphericalJoint.h"
#include "ExtGearJoint.h"
#include "ExtRackAndPinionJoint.h"


using namespace physx;

// Shared per-type custom-geometry-ext emitters, defined in ExtCustomGeometryExt.cpp next to the
// constructors that also use them (the same sharing the joints use via omniPvdInitJoint). The
// snapshot pass below switches on the callbacks' custom type and calls the matching one.
namespace physx { namespace Ext {
	void omniPvdInitCustomGeometryExt(OmniPvdWriter* pvdWriter, const OmniPvdPxExtensionsRegistrationData* pvdRegData, const PxCustomGeometryExt::CylinderCallbacks& c);
	void omniPvdInitCustomGeometryExt(OmniPvdWriter* pvdWriter, const OmniPvdPxExtensionsRegistrationData* pvdRegData, const PxCustomGeometryExt::ConeCallbacks& c);
} }


void OmniPvdPxExtensionsSampler::registerClasses()
{
	PxOmniPvd::ScopedExclusiveWriter scope(mOmniPvdInstance);
	OmniPvdWriter* writer = scope.getWriter();
	if (writer)
	{
		mRegistrationData.registerData(*mOmniPvdInstance->getWriter());
	}
}

OmniPvdPxExtensionsSampler::OmniPvdPxExtensionsSampler(physx::PxPhysics& physics)
	: mPhysics(physics)
{
	mOmniPvdInstance = NULL;
}

OmniPvdPxExtensionsSampler::~OmniPvdPxExtensionsSampler()
{
}

void OmniPvdPxExtensionsSampler::setOmniPvdInstance(physx::PxOmniPvd* omniPvdInstance)
{
	mOmniPvdInstance = omniPvdInstance;
}

void OmniPvdPxExtensionsSampler::onStartSampling(physx::PxOmniPvd& omniPvd)
{
	// PxOmniPvdEventCallback entry: startSampling() calls this after the core world is emitted.
	// Re-emit the live extension joints of the captured PxPhysics onto the bound stream.
	// PxInitExtensions bound the instance via setOmniPvdInstance(); startSampling() passes the
	// same PxOmniPvd, so assert it matches rather than silently adopting it.
	PX_ASSERT(mOmniPvdInstance == &omniPvd);
	PX_UNUSED(omniPvd); // only read by the assert (compiled out in release)
	if (mOmniPvdInstance == NULL)
		return;

	// Re-register the full PhysXExtensions schema (every extension class and attribute definition)
	// onto the freshly-bound stream once, before either snapshot pass, because setWriteStream zeroed
	// the writer's class/attribute handle counters. This schema is self-contained (every class derives
	// from PxJoint and every flag attribute references PxConstraintFlag, both registered here; actor /
	// constraint references are plain object handles resolved at read time), so it neither reads core
	// PhysX class data nor depends on the core schema being registered first. The emission passes do
	// need to come after the core snapshot, since they reference core actor / constraint objects.
	registerClasses();
	snapshotConstraints();
	snapshotCustomGeometryCallbacks();
}

void OmniPvdPxExtensionsSampler::snapshotConstraints()
{
	// Enumerate every constraint the SDK tracks, not only the ones already added to a scene, so
	// scene-less joints are snapshotted too (matching what from-start sampling records as each
	// joint is created). PxPhysics::getConstraints exposes the full tracked list as public API.
	const PxU32 nbConstraints = mPhysics.getNbConstraints();
	if (nbConstraints == 0)
		return;

	// Lock the writer once for the whole pass (matching OmniPvdPxSampler::snapshotAll()), then page the
	// tracked constraints through a stack-sized batch buffer so there is no per-object lock or heap alloc.
	PxOmniPvd::ScopedExclusiveWriter scope(mOmniPvdInstance);
	OmniPvdWriter* pvdWriter = scope.getWriter();
	if (pvdWriter == NULL)
		return;
	const physx::Ext::OmniPvdPxExtensionsRegistrationData* pvdRegData = &mRegistrationData;

	const PxU32 batchSize = 256;
	PxConstraint* batch[batchSize];
	for (PxU32 off = 0; off < nbConstraints; off += batchSize)
	{
		const PxU32 got = mPhysics.getConstraints(batch, batchSize, off);
		for (PxU32 i = 0; i < got; ++i)
		{
			PxConstraint* constraint = batch[i];
			if (constraint == NULL)
				continue;

			PxU32 typeID = PxConstraintExtIDs::eINVALID_ID;
			void* externalRef = constraint->getExternalReference(typeID);
			if (typeID != PxConstraintExtIDs::eJOINT || externalRef == NULL)
				continue; // skip vehicle joints and any non-joint constraints

			PxJoint* joint = static_cast<PxJoint*>(externalRef);
			switch (joint->getConcreteType())
			{
			case PxJointConcreteType::eD6:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::D6Joint&>(*joint)); break;
			case PxJointConcreteType::eREVOLUTE:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::RevoluteJoint&>(*joint)); break;
			case PxJointConcreteType::ePRISMATIC:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::PrismaticJoint&>(*joint)); break;
			case PxJointConcreteType::eFIXED:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::FixedJoint&>(*joint)); break;
			case PxJointConcreteType::eDISTANCE:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::DistanceJoint&>(*joint)); break;
			case PxJointConcreteType::eSPHERICAL:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::SphericalJoint&>(*joint)); break;
			case PxJointConcreteType::eGEAR:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::GearJoint&>(*joint)); break;
			case PxJointConcreteType::eRACK_AND_PINION:
				physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::RackAndPinionJoint&>(*joint)); break;
			default:
				break;
			}
		}
	}
}

void OmniPvdPxExtensionsSampler::snapshotCustomGeometryCallbacks()
{
	// The extension schema was already re-registered by onStartSampling() before this pass, so the
	// PxCustomGeometryExt* class/attribute handles are valid here.

	// Walk every shape the SDK tracks (scene-less included, like getConstraints), reach each shape's
	// custom geometry callbacks, and re-emit the PxCustomGeometryExt ones. The same callbacks object
	// can back many shapes (the SDK only stores the pointer), and the live ctor emits it once, so we
	// dedup by callbacks pointer and emit each once.
	const PxU32 nbShapes = mPhysics.getNbShapes();
	if (nbShapes == 0)
		return;

	// Lock the writer once for the whole pass (matching snapshotConstraints and
	// OmniPvdPxSampler::snapshotAll()), then page the shapes through a stack-sized batch buffer. The
	// dedup set spans all batches: the same callbacks object can back many shapes, and the live ctor
	// emits it once, so emit each once.
	PxOmniPvd::ScopedExclusiveWriter scope(mOmniPvdInstance);
	OmniPvdWriter* pvdWriter = scope.getWriter();
	if (pvdWriter == NULL)
		return;
	const physx::Ext::OmniPvdPxExtensionsRegistrationData* pvdRegData = &mRegistrationData;

	const PxU32 batchSize = 256;
	PxShape* shapeBatch[batchSize];
	PxHashSet<const void*> emitted;
	for (PxU32 off = 0; off < nbShapes; off += batchSize)
	{
		const PxU32 got = mPhysics.getShapes(shapeBatch, batchSize, off);
		for (PxU32 i = 0; i < got; ++i)
		{
			PxShape* shape = shapeBatch[i];
			if (shape == NULL)
				continue;
			const PxGeometry& geom = shape->getGeometry();
			if (geom.getType() != PxGeometryType::eCUSTOM)
				continue;

			PxCustomGeometry::Callbacks* cb = static_cast<const PxCustomGeometry&>(geom).callbacks;
			if (cb == NULL)
				continue;
			if (!emitted.insert(cb)) // already emitted (the same callbacks object can back many shapes)
				continue;

			// Switch on the custom type and call the shared per-type emitter (the same one the
			// callbacks constructor uses), as the joint pass does with omniPvdInitJoint. A user's own
			// (non-Ext) callbacks matches neither type and carries no extension schema, so it is skipped.
			const PxCustomGeometry::Type type = cb->getCustomType();
			if (type == PxCustomGeometryExt::CylinderCallbacks::TYPE())
				physx::Ext::omniPvdInitCustomGeometryExt(pvdWriter, pvdRegData, static_cast<const PxCustomGeometryExt::CylinderCallbacks&>(*cb));
			else if (type == PxCustomGeometryExt::ConeCallbacks::TYPE())
				physx::Ext::omniPvdInitCustomGeometryExt(pvdWriter, pvdRegData, static_cast<const PxCustomGeometryExt::ConeCallbacks&>(*cb));
		}
	}
}

physx::PxOmniPvd* OmniPvdPxExtensionsSampler::getOmniPvdInstance() {
	return mOmniPvdInstance;
}


///////////////////////////////////////////////////////////////////////////////

static OmniPvdPxExtensionsSampler* gOmniPvdPxExtensionsSampler = NULL;

bool OmniPvdPxExtensionsSampler::createInstance(physx::PxPhysics& physics)
{
	gOmniPvdPxExtensionsSampler = PX_NEW(OmniPvdPxExtensionsSampler)(physics);
	return gOmniPvdPxExtensionsSampler != NULL;
}

OmniPvdPxExtensionsSampler* OmniPvdPxExtensionsSampler::getInstance()
{
	return gOmniPvdPxExtensionsSampler;
}

void OmniPvdPxExtensionsSampler::destroyInstance()
{
	PX_DELETE(gOmniPvdPxExtensionsSampler);
}


namespace physx
{
namespace Ext
{

const OmniPvdPxExtensionsRegistrationData* OmniPvdGetPxExtensionsRegistrationData()
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	if (sampler)
	{
		return &sampler->getRegistrationData();
	}
	else
	{
		return NULL;
	}
}

PxOmniPvd* OmniPvdGetInstance()
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	if (sampler)
	{
		return sampler->getOmniPvdInstance();
	}
	else
	{
		return NULL;
	}
}

}
}

#endif

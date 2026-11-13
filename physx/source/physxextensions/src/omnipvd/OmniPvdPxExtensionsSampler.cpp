// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#if PX_SUPPORT_OMNI_PVD

#include "OmniPvdPxExtensionsSampler.h"
#include "omnipvd/PxOmniPvd.h"

#include "PxPhysics.h"
#include "PxScene.h"
#include "PxShape.h"
#include "PxConstraint.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxString.h"
#include <string.h>
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
	: mPhysics(physics), mRecordSecondaryData(true), mRecordAllObjects(true)
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

void OmniPvdPxExtensionsSampler::onStartSampling(physx::PxOmniPvd& omniPvd, const physx::PxOmniPvdFilterDesc* filter)
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
	mRecordSecondaryData = !filter || filter->recordSecondaryData;
	mRecordAllObjects = !filter || (!filter->nbScenes && !filter->nbObjects);
	registerClasses();
	mEmittedObjects.clear();
	mRefreshingJoints.clear();
	mJointReferences.clear();
	mSelectedCustomCallbacks.clear();
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
			const void* externalRef = constraint->getExternalReference(typeID);
			if (typeID == PxConstraintExtIDs::eJOINT && externalRef &&
				mEmittedObjects.contains(reinterpret_cast<PxU64>(externalRef)))
				continue;
			snapshotConstraint(*constraint, pvdWriter);
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
	const PxU32 batchSize = 256;
	PxShape* shapeBatch[batchSize];
	for (PxU32 off = 0; off < nbShapes; off += batchSize)
	{
		const PxU32 got = mPhysics.getShapes(shapeBatch, batchSize, off);
		for (PxU32 i = 0; i < got; ++i)
		{
			PxShape* shape = shapeBatch[i];
			if (shape == NULL || !mOmniPvdInstance->isObjectRecorded(*shape))
				continue;
			snapshotCustomGeometry(*shape, pvdWriter);
		}
	}
}

void OmniPvdPxExtensionsSampler::snapshotConstraint(PxConstraint& constraint, OmniPvdWriter* pvdWriter)
{
	PxU32 typeID = PxConstraintExtIDs::eINVALID_ID;
	void* externalRef = constraint.getExternalReference(typeID);
	if (typeID != PxConstraintExtIDs::eJOINT || externalRef == NULL)
		return;
	PxJoint& joint = *static_cast<PxJoint*>(externalRef);
	// Scene admission can precede the constructor's connector attachment.
	if (joint.getConstraint() != &constraint)
		return;
	if (!mOmniPvdInstance->isObjectRecorded(constraint))
	{
		if (shouldRecordDestruction(reinterpret_cast<PxU64>(&joint)))
		{
			// D6 drive objects use synthetic handles owned by the joint.
			if (joint.getConcreteType() == PxJointConcreteType::eD6)
				static_cast<physx::Ext::D6Joint&>(joint).omniPvdDestroyDrives(pvdWriter, &mRegistrationData);
			pvdWriter->destroyObject(1, reinterpret_cast<PxU64>(&joint));
		}
		return;
	}
	const physx::Ext::OmniPvdPxExtensionsRegistrationData* pvdRegData = &mRegistrationData;
	// Keep the recorded identity, but refresh configuration that may have changed while excluded.
	// Gear/rack references can recurse into this pass. Only the outer refresh emits each joint.
	const PxU64 handle = reinterpret_cast<PxU64>(&joint);
	if (!mRefreshingJoints.insert(handle)) return;
	switch (joint.getConcreteType())
	{
	case PxJointConcreteType::eD6:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::D6Joint&>(joint)); break;
	case PxJointConcreteType::eREVOLUTE:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::RevoluteJoint&>(joint)); break;
	case PxJointConcreteType::ePRISMATIC:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::PrismaticJoint&>(joint)); break;
	case PxJointConcreteType::eFIXED:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::FixedJoint&>(joint)); break;
	case PxJointConcreteType::eDISTANCE:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::DistanceJoint&>(joint)); break;
	case PxJointConcreteType::eSPHERICAL:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::SphericalJoint&>(joint)); break;
	case PxJointConcreteType::eGEAR:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::GearJoint&>(joint)); break;
	case PxJointConcreteType::eRACK_AND_PINION:
		physx::Ext::omniPvdInitJoint(pvdWriter, pvdRegData, static_cast<physx::Ext::RackAndPinionJoint&>(joint)); break;
	default: break;
	}
	mRefreshingJoints.erase(handle);
}

const PxBase* OmniPvdPxExtensionsSampler::captureJointReference(const PxBase* reference, OmniPvdWriter* writer)
{
	if (!reference || !writer || !mOmniPvdInstance || !mOmniPvdInstance->isSampling())
		return NULL;
	if (reference->getConcreteType() == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
		return mOmniPvdInstance->isObjectRecorded(*reference) ? reference : NULL;
	const PxJoint* joint = reference->is<PxJoint>();
	if (!joint || !joint->getConstraint() || !mOmniPvdInstance->isObjectRecorded(*joint->getConstraint()))
		return NULL;
	// Register identities before attributes to deduplicate recursion and create targets before references.
	const PxU64 handle = reinterpret_cast<PxU64>(joint);
	if (!mEmittedObjects.contains(handle))
		snapshotConstraint(*joint->getConstraint(), writer);
	return mEmittedObjects.contains(handle) ? reference : NULL;
}

void OmniPvdPxExtensionsSampler::cacheJointReferences(PxU64 dependent, const PxBase* joint0, const PxBase* joint1)
{
	const JointReferences references = { reinterpret_cast<PxU64>(joint0), reinterpret_cast<PxU64>(joint1) };
	mJointReferences[dependent] = references;
}

void OmniPvdPxExtensionsSampler::snapshotCustomGeometry(const PxShape& shape, OmniPvdWriter* pvdWriter)
{
	const PxGeometry& geom = shape.getGeometry();
	if (geom.getType() != PxGeometryType::eCUSTOM)
		return;
	PxCustomGeometry::Callbacks* cb = static_cast<const PxCustomGeometry&>(geom).callbacks;
	if (!cb || mEmittedObjects.contains(reinterpret_cast<PxU64>(cb)))
		return;
	mSelectedCustomCallbacks.insert(cb);
	const physx::Ext::OmniPvdPxExtensionsRegistrationData* pvdRegData = &mRegistrationData;
	const PxCustomGeometry::Type type = cb->getCustomType();
	if (type == PxCustomGeometryExt::CylinderCallbacks::TYPE())
		physx::Ext::omniPvdInitCustomGeometryExt(pvdWriter, pvdRegData, static_cast<const PxCustomGeometryExt::CylinderCallbacks&>(*cb));
	else if (type == PxCustomGeometryExt::ConeCallbacks::TYPE())
		physx::Ext::omniPvdInitCustomGeometryExt(pvdWriter, pvdRegData, static_cast<const PxCustomGeometryExt::ConeCallbacks&>(*cb));
	mSelectedCustomCallbacks.erase(cb);
}

void OmniPvdPxExtensionsSampler::onCaptureReset(PxOmniPvd& omniPvd, const PxOmniPvdFilterDesc& filter)
{
	PX_UNUSED(filter);
	// Retire module objects before old core objects are retired.
	PxOmniPvd::ScopedExclusiveWriter scope(&omniPvd);
	OmniPvdWriter* writer = scope.getWriter();
	if (writer)
		for (PxHashSet<PxU64>::Iterator it = mEmittedObjects.getIterator(); !it.done(); ++it)
			writer->destroyObject(1, *it);
	mEmittedObjects.clear();
	mSelectedCustomCallbacks.clear();
}

void OmniPvdPxExtensionsSampler::onObjectCapture(PxOmniPvd& omniPvd, const PxBase& object)
{
	PX_ASSERT(mOmniPvdInstance == &omniPvd);
	// The core invokes this callback while it owns the writer lock.
	OmniPvdWriter* writer = omniPvd.getWriter();
	if (!writer)
		return;
	if (object.getConcreteType() == PxConcreteType::eSHAPE)
		snapshotCustomGeometry(static_cast<const PxShape&>(object), writer);
	else if (object.getConcreteType() == PxConcreteType::eCONSTRAINT)
	{
		PxConstraint& constraint = const_cast<PxConstraint&>(static_cast<const PxConstraint&>(object));
		const bool selected = omniPvd.isObjectRecorded(constraint);
		if (selected) snapshotConstraint(constraint, writer);
		if (mRecordAllObjects) return;
		PxU32 typeID = PxConstraintExtIDs::eINVALID_ID;
		const void* changed = constraint.getExternalReference(typeID);
		if (!changed || typeID != PxConstraintExtIDs::eJOINT) return;
		// Never walk the factory constraint set here. Other threads can change other scenes.
		const PxU64 changedHandle = reinterpret_cast<PxU64>(changed);
		PxArray<PxU64> dependents; // snapshotConstraint can update the map.
		for (PxHashMap<PxU64, JointReferences>::Iterator it = mJointReferences.getIterator(); !it.done(); ++it)
			if (it->second.joint0 == changedHandle || it->second.joint1 == changedHandle)
				dependents.pushBack(it->first);
		for (PxU32 i = 0; i < dependents.size(); ++i)
		{
			PxConstraint* dependent = reinterpret_cast<const PxJoint*>(dependents[i])->getConstraint();
			if (dependent && omniPvd.isObjectRecorded(*dependent)) snapshotConstraint(*dependent, writer);
		}
		if (!selected) snapshotConstraint(constraint, writer);
	}
}

bool OmniPvdPxExtensionsSampler::shouldRecordCreation(const char* className, PxU64 objectHandle)
{
	if (!mOmniPvdInstance || !mOmniPvdInstance->isSampling())
		return false;
	if (Pxstrncmp(className, "PxCustomGeometryExt", sizeof("PxCustomGeometryExt") - 1) == 0)
	{
		// Shape emission deduplicates callbacks; a constructor reusing an address retires its prior lifetime.
		if (mEmittedObjects.erase(objectHandle))
			mOmniPvdInstance->getWriter()->destroyObject(1, objectHandle);
		if (!mRecordAllObjects && !mSelectedCustomCallbacks.contains(reinterpret_cast<const void*>(objectHandle)))
			return false;
	}
	else if (Pxstrcmp(className, "PxD6JointDrive") == 0)
	{
		if (!mRecordSecondaryData)
			return false;
	}
	else if (!physx::Ext::OmniPvdCaptureJoint(*reinterpret_cast<const PxJoint*>(objectHandle)))
		return false;
	return mEmittedObjects.insert(objectHandle);
}

bool OmniPvdPxExtensionsSampler::shouldRecordDestruction(PxU64 objectHandle)
{
	mJointReferences.erase(objectHandle);
	return mEmittedObjects.erase(objectHandle);
}

bool OmniPvdPxExtensionsSampler::shouldRecordValue(const char* className, const char* attributeName, PxU64 objectHandle) const
{
	if (!mEmittedObjects.contains(objectHandle))
		return false;
	if (mRecordAllObjects && mRecordSecondaryData)
		return true;
	if (Pxstrncmp(className, "PxCustomGeometryExt", sizeof("PxCustomGeometryExt") - 1) == 0)
		return true;
	if (Pxstrcmp(className, "PxD6JointDrive") &&
		!physx::Ext::OmniPvdCaptureJoint(*reinterpret_cast<const PxJoint*>(objectHandle)))
		return false;
	static const char* const primaryAttributes[] = { "constraint", "type", "actor0", "actor1", "name", "concreteTypeName",
		"actor0LocalPose", "actor1LocalPose", "angle", "position", "velocity", "twistAngle", "distance", "swingYAngle",
		"swingZAngle", "hinges", "joints" };
	for (PxU32 i = 0; i < PxU32(sizeof(primaryAttributes) / sizeof(primaryAttributes[0])); ++i)
		if (Pxstrcmp(attributeName, primaryAttributes[i]) == 0)
			return true;
	return mRecordSecondaryData;
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

bool OmniPvdCaptureJoint(const PxJoint& joint)
{
	PxOmniPvd* ovd = OmniPvdGetInstance();
	return ovd && ovd->isSampling() && joint.getConstraint() && ovd->isObjectRecorded(*joint.getConstraint());
}

const PxBase* OmniPvdCaptureJointReference(const PxBase* joint, OmniPvdWriter* writer)
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	return sampler ? sampler->captureJointReference(joint, writer) : NULL;
}

void OmniPvdCacheJointReferences(const PxJoint& dependent, const PxBase* joint0, const PxBase* joint1)
{
	if (OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance())
		sampler->cacheJointReferences(reinterpret_cast<PxU64>(&dependent), joint0, joint1);
}

bool OmniPvdShouldRecordCreation(const char* className, PxU64 objectHandle)
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	return sampler && sampler->shouldRecordCreation(className, objectHandle);
}

bool OmniPvdShouldRecordDestruction(PxU64 objectHandle)
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	return sampler && sampler->shouldRecordDestruction(objectHandle);
}

bool OmniPvdShouldRecordValue(const char* className, const char* attributeName, PxU64 objectHandle)
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	return sampler && sampler->shouldRecordValue(className, attributeName, objectHandle);
}

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

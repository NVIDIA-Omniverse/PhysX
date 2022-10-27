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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdPxSampler.h"
#include "common/PxProfileZone.h"
#include "NpPhysics.h"
#include "NpScene.h"
#include "NpShape.h"
#include "NpArticulationLink.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpActor.h"
#include "NpSoftBody.h"
#include "NpAggregate.h"
#include "NpOmniPvd.h"
#include "OmniPvdWriter.h"
#include "OmniPvdWriteStream.h"
#include "foundation/PxAllocator.h"
#include "ScInteraction.h"
#include "NpArticulationJointReducedCoordinate.h"

#include <stdio.h>

using namespace physx;

#define UNNECESSARY_SCENE_HANDLE 1

void createGeometry(const physx::PxGeometry & g);
void destroyGeometry(const physx::PxGeometry & g);

class OmniPvdStreamContainer
{
public:
	OmniPvdStreamContainer();
	~OmniPvdStreamContainer();
	bool initOmniPvd();
	void registerClasses();
	void setOmniPvdWriter(OmniPvdWriter* omniPvdWriter);

	OmniPvdWriter* mWriter;
	physx::PxMutex mMutex;
	bool mClassesRegistered;
};

class OmniPvdSamplerInternals : public physx::PxUserAllocated
{
public:
OmniPvdStreamContainer mPvdStream;
bool addSharedMeshIfNotSeen(const void* geom, OmniPvdSharedMeshEnum geomEnum); // Returns true if the Geom was not yet seen and added
physx::PxMutex mSampleMutex;
bool mIsSampling;

physx::PxMutex mSampledScenesMutex;
physx::PxHashMap<physx::NpScene*, OmniPvdPxScene*> mSampledScenes;

physx::PxMutex mSharedGeomsMutex;
physx::PxHashMap<const void*, OmniPvdSharedMeshEnum> mSharedMeshesMap;
};
OmniPvdSamplerInternals * samplerInternals = NULL;

class OmniPvdPxScene : public physx::PxUserAllocated
{
public:
	OmniPvdPxScene() : mFrameId(0) {}
	~OmniPvdPxScene() {}

	void sampleScene()
	{
		mFrameId++;
		samplerInternals->mPvdStream.mWriter->startFrame(UNNECESSARY_SCENE_HANDLE, mFrameId);		
	}

	physx::PxU64 mFrameId;
};

OmniPvdStreamContainer::OmniPvdStreamContainer()
{
	physx::PxMutex::ScopedLock myLock(mMutex);
	mWriter = NULL;
	mClassesRegistered = false;
}

OmniPvdStreamContainer::~OmniPvdStreamContainer()
{
}

void OmniPvdStreamContainer::setOmniPvdWriter(OmniPvdWriter* omniPvdWriter)
{
	mWriter = omniPvdWriter;
}

bool OmniPvdStreamContainer::initOmniPvd()
{
	physx::PxMutex::ScopedLock myLock(mMutex);

	registerClasses();

	PxPhysics& physicsRef = static_cast<PxPhysics&>(NpPhysics::getInstance());

	OMNI_PVD_CREATE(physics, physicsRef);
	const physx::PxTolerancesScale tolScale = physicsRef.getTolerancesScale();
	PxF32 scale[2];
	scale[0] = tolScale.length;
	scale[1] = tolScale.speed;
	OMNI_PVD_SETB(physics, tolerancesScale, physicsRef, scale, sizeof(PxF32) * 2);

	return true;
}

//array to hold PVD type to size conversion table.  Should be large enough for all basic PVD types.
static unsigned sizeOfOmniPvdTypes[32];

//create a class for each SDK type and attribute -- the primary thing is to allocate handle storage, the rest is just fluff.
#define OMNI_PVD_FAKE_CLASS(c, classT, classStr) OmniPvdClassHandle OmniPvdPxSampler::classHandle_##c;
#define OMNI_PVD_CLASS(c, classT) OmniPvdClassHandle OmniPvdPxSampler::classHandle_##c;
#define OMNI_PVD_ENUM(c, classT) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_CLASS_DERIVED(c, classT, baseClass) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_ATTRIBUTE_SET(c, a, classT, attrT) OmniPvdAttributeHandle OmniPvdPxSampler::attributeHandle_##c##_##a;

//enum values don't need to save their handle, since they are const/immutable:
#define OMNI_PVD_ENUM_VALUE(c, a, v)	
#define OMNI_PVD_ATTRIBUTE(c, a, classT, attrT, t, n) OmniPvdAttributeHandle OmniPvdPxSampler::attributeHandle_##c##_##a;
#define OMNI_PVD_ATTRIBUTE_FLAG(c, a, classT, attrT, enumClass) OmniPvdAttributeHandle OmniPvdPxSampler::attributeHandle_##c##_##a;


#include "OmniPvdTypes.h"
#undef OMNI_PVD_ENUM
#undef OMNI_PVD_ENUM_VALUE
#undef OMNI_PVD_FAKE_CLASS
#undef OMNI_PVD_CLASS
#undef OMNI_PVD_CLASS_DERIVED
#undef OMNI_PVD_ATTRIBUTE
#undef OMNI_PVD_ATTRIBUTE_SET
#undef OMNI_PVD_ATTRIBUTE_FLAG

void OmniPvdStreamContainer::registerClasses()
{
	if (mClassesRegistered) return;
	if (mWriter)
	{
//register all SDK classes and attributes:
#define OMNI_PVD_FAKE_CLASS(c, classT, classStr) OmniPvdPxSampler::classHandle_##c = mWriter->registerClass(#classStr);
#define OMNI_PVD_CLASS(c, classT) OmniPvdPxSampler::classHandle_##c = mWriter->registerClass(#classT);
#define OMNI_PVD_ENUM(c, classT) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_CLASS_DERIVED(c, classT, baseClass) OmniPvdPxSampler::classHandle_##c = mWriter->registerClass(#classT, OmniPvdPxSampler::classHandle_##baseClass);
#define OMNI_PVD_ENUM_VALUE(c, a, v)				mWriter->registerEnumValue(OmniPvdPxSampler::classHandle_##c, #a, v);
#define OMNI_PVD_ATTRIBUTE_SET(c, a, classT, attrT)	OmniPvdPxSampler::attributeHandle_##c##_##a = mWriter->registerSetAttribute(OmniPvdPxSampler::classHandle_##c, #a, OmniPvdDataTypeEnum::eOBJECT_HANDLE);
#define OMNI_PVD_ATTRIBUTE(c, a, classT, attrT, t, n) PX_ASSERT((n == 0) || (sizeof(attrT) == sizeOfOmniPvdTypes[t] * n)); OmniPvdPxSampler::attributeHandle_##c##_##a = mWriter->registerAttribute(OmniPvdPxSampler::classHandle_##c, #a, t, n);
#define OMNI_PVD_ATTRIBUTE_FLAG(c, a, classT, attrT, enumClass) OmniPvdPxSampler::attributeHandle_##c##_##a = mWriter->registerFlagsAttribute(OmniPvdPxSampler::classHandle_##c, OmniPvdPxSampler::classHandle_##enumClass, #a);
#include "OmniPvdTypes.h"
#undef OMNI_PVD_ENUM
#undef OMNI_PVD_ENUM_VALUE
#undef OMNI_PVD_FAKE_CLASS
#undef OMNI_PVD_CLASS
#undef OMNI_PVD_CLASS_DERIVED
#undef OMNI_PVD_ATTRIBUTE
#undef OMNI_PVD_ATTRIBUTE_SET
#undef OMNI_PVD_ATTRIBUTE_FLAG

		mClassesRegistered = true;
	}
}



//instance any templates that are not used in this compilation unit so that code gets generated anyways

#define OMNI_PVD_FAKE_CLASS(c, classT, classStr) \
template void OmniPvdPxSampler::createObject <classT>(OmniPvdClassHandle, classT const &);\
template void OmniPvdPxSampler::destroyObject<classT>(classT const &);

#define OMNI_PVD_CLASS(c, classT) \
template void OmniPvdPxSampler::createObject <classT>(OmniPvdClassHandle, classT const &);\
template void OmniPvdPxSampler::destroyObject<classT>(classT const &);

#define OMNI_PVD_CLASS_DERIVED(c, classT, baseClass) OMNI_PVD_CLASS(c, classT)

#define OMNI_PVD_ATTRIBUTE_SET(c, a, classT, attrT) \
template void OmniPvdPxSampler::addToSet<classT, attrT>(OmniPvdAttributeHandle, classT const & , attrT const & );\
template void OmniPvdPxSampler::removeFromSet<classT, attrT>(OmniPvdAttributeHandle, classT const & , attrT const & );

#define OMNI_PVD_ATTRIBUTE(c, a, classT, attrT, t, n) \
template void OmniPvdPxSampler::setAttribute<classT, attrT>(OmniPvdAttributeHandle, const classT&, attrT const &); \
template void OmniPvdPxSampler::setAttributeBytes<classT, attrT>(OmniPvdAttributeHandle, const classT&, attrT const *, unsigned);

#define OMNI_PVD_ATTRIBUTE_FLAG(c, a, classT, attrT, enumClass) \
template void OmniPvdPxSampler::setAttribute<classT, attrT>(OmniPvdAttributeHandle, classT const &, attrT const &);

#define OMNI_PVD_ENUM(c, enumT)
#define OMNI_PVD_ENUM_VALUE(c, a, v)

#include "OmniPvdTypes.h"
#undef OMNI_PVD_ENUM
#undef OMNI_PVD_ENUM_VALUE
#undef OMNI_PVD_FAKE_CLASS
#undef OMNI_PVD_CLASS
#undef OMNI_PVD_CLASS_DERIVED
#undef OMNI_PVD_ATTRIBUTE
#undef OMNI_PVD_ATTRIBUTE_SET
//end instance templates

void streamActorName(physx::PxActor & a, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	if (NpPhysics::getInstance().mOmniPvdSampler == NULL)
	{
		return;
	}
	if (name == NULL)
	{
		return;
	}
	int len = static_cast<int>(strlen(name));
	if (len > 0)
	{
		len += 1;
		OMNI_PVD_SETB(actor, name, a, name, len); // copies over the trailing zero too
	}
#endif
}

void streamSphere(const physx::PxGeometry& g)
{
	PxGeometryHolder gh(g);
	OMNI_PVD_CREATE(geomsphere, g);
	OMNI_PVD_SET(geomsphere, radius, g, gh.sphere().radius);
}

void streamCapsule(const physx::PxGeometry& g)
{
	PxGeometryHolder gh(g);
	OMNI_PVD_CREATE(geomcapsule, g);
	OMNI_PVD_SET(geomcapsule, halfHeight, g, gh.capsule().halfHeight);
	OMNI_PVD_SET(geomcapsule, radius, g, gh.capsule().radius);
}

void streamBox(const physx::PxGeometry& g)
{
	PxGeometryHolder gh(g);
	OMNI_PVD_CREATE(geombox, g);
	OMNI_PVD_SET(geombox, halfExtents, g, gh.box().halfExtents);
}

void streamPlane(const physx::PxGeometry& g)
{
	OMNI_PVD_CREATE(geomplane, g);
}

void streamConvexMesh(const physx::PxConvexMesh* mesh)
{		
	if (samplerInternals->addSharedMeshIfNotSeen(mesh, OmniPvdSharedMeshEnum::eOmniPvdConvexMesh))
	{
		OMNI_PVD_CREATE(convexmesh, *mesh);

		const physx::PxU32 nbPolys = mesh->getNbPolygons();
		const physx::PxU8* polygons = mesh->getIndexBuffer();
		const physx::PxVec3* verts = mesh->getVertices();
		const int nbrVerts = mesh->getNbVertices();

		int totalTris = 0;
		for (unsigned int i = 0; i < nbPolys; i++)
		{
			physx::PxHullPolygon data;
			mesh->getPolygonData(i, data);
			totalTris += physx::PxU32(data.mNbVerts - 2);
		}

		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
		int* tmpIndices = (int*)PX_ALLOC(sizeof(int)*(totalTris * 3), "tmpIndices");
		//TODO: this copy is useless

		int vertIndex = 0;
		for (int v = 0; v < nbrVerts; v++)
		{
			tmpVerts[vertIndex + 0] = verts[v].x;
			tmpVerts[vertIndex + 1] = verts[v].y;
			tmpVerts[vertIndex + 2] = verts[v].z;
			vertIndex += 3;
		}

		int triIndex = 0;
		for (unsigned int p = 0; p < nbPolys; p++)
		{
			physx::PxHullPolygon data;
			mesh->getPolygonData(p, data);
			int nbTris = physx::PxU32(data.mNbVerts - 2);
			const physx::PxU32 vref0 = polygons[data.mIndexBase + 0 + 0];
			for (int t = 0; t < nbTris; t++)
			{
				const physx::PxU32 vref1 = polygons[data.mIndexBase + t + 1];
				const physx::PxU32 vref2 = polygons[data.mIndexBase + t + 2];
				tmpIndices[triIndex + 0] = vref0;
				tmpIndices[triIndex + 1] = vref1;
				tmpIndices[triIndex + 2] = vref2;
				triIndex += 3;
			}
		}

		OMNI_PVD_SETB(convexmesh, verts, *mesh, tmpVerts, sizeof(float) * 3 * nbrVerts);
		OMNI_PVD_SETB(convexmesh, tris, *mesh, tmpIndices, sizeof(int) * 3 * totalTris);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamConvexMeshGeometry(const physx::PxGeometry& g)
{		
	PxGeometryHolder gh(g);
	OMNI_PVD_CREATE(geomconvexmesh, g);
	OMNI_PVD_SET(geomconvexmesh, scale, g, gh.convexMesh().scale.scale);
	const physx::PxConvexMeshGeometry& geometry = gh.convexMesh();
	physx::PxConvexMesh const* mesh = geometry.convexMesh;
	streamConvexMesh(mesh);
	OMNI_PVD_SET(geomconvexmesh, convexMesh, g, mesh);
}

void streamHeightField(const physx::PxHeightField* hf)
{
	if (samplerInternals->addSharedMeshIfNotSeen(hf, OmniPvdSharedMeshEnum::eOmniPvdHeightField))
	{
		OMNI_PVD_CREATE(heightfield, *hf);
		const physx::PxU32 nbCols = hf->getNbColumns();
		const physx::PxU32 nbRows = hf->getNbRows();
		const physx::PxU32 nbVerts = nbRows * nbCols;
		const physx::PxU32 nbFaces = (nbCols - 1) * (nbRows - 1) * 2;
		physx::PxHeightFieldSample* sampleBuffer = (physx::PxHeightFieldSample*)PX_ALLOC(sizeof(physx::PxHeightFieldSample)*(nbVerts), "sampleBuffer");
		hf->saveCells(sampleBuffer, nbVerts * sizeof(physx::PxHeightFieldSample));
		//TODO: are the copies necessary?
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbVerts * 3), "tmpVerts");
		int* tmpIndices = (int*)PX_ALLOC(sizeof(int)*(nbFaces * 3), "tmpIndices");
		for (physx::PxU32 i = 0; i < nbRows; i++)
		{
			for (physx::PxU32 j = 0; j < nbCols; j++)
			{
				const float x = physx::PxReal(i);// *rs;
				const float y = physx::PxReal(sampleBuffer[j + (i*nbCols)].height);// *hs;
				const float z = physx::PxReal(j);// *cs;
				const int vertexIndex = 3 * (i * nbCols + j);
				float* vert = &tmpVerts[vertexIndex];
				vert[0] = x;
				vert[1] = y;
				vert[2] = z;
			}
		}
		for (physx::PxU32 i = 0; i < (nbCols - 1); ++i)
		{
			for (physx::PxU32 j = 0; j < (nbRows - 1); ++j)
			{
				physx::PxU8 tessFlag = sampleBuffer[i + j * nbCols].tessFlag();
				physx::PxU32 i0 = j * nbCols + i;
				physx::PxU32 i1 = j * nbCols + i + 1;
				physx::PxU32 i2 = (j + 1) * nbCols + i;
				physx::PxU32 i3 = (j + 1) * nbCols + i + 1;
				// i2---i3
				// |    |
				// |    |
				// i0---i1
				// this is really a corner vertex index, not triangle index
				physx::PxU32 mat0 = hf->getTriangleMaterialIndex((j*nbCols + i) * 2);
				physx::PxU32 mat1 = hf->getTriangleMaterialIndex((j*nbCols + i) * 2 + 1);
				bool hole0 = (mat0 == physx::PxHeightFieldMaterial::eHOLE);
				bool hole1 = (mat1 == physx::PxHeightFieldMaterial::eHOLE);
				// first triangle
				tmpIndices[6 * (i * (nbRows - 1) + j) + 0] = hole0 ? i0 : i2; // duplicate i0 to make a hole
				tmpIndices[6 * (i * (nbRows - 1) + j) + 1] = i0;
				tmpIndices[6 * (i * (nbRows - 1) + j) + 2] = tessFlag ? i3 : i1;
				// second triangle
				tmpIndices[6 * (i * (nbRows - 1) + j) + 3] = hole1 ? i1 : i3; // duplicate i1 to make a hole
				tmpIndices[6 * (i * (nbRows - 1) + j) + 4] = tessFlag ? i0 : i2;
				tmpIndices[6 * (i * (nbRows - 1) + j) + 5] = i1;
			}
		}
		PX_FREE(sampleBuffer);
		OMNI_PVD_SETB(heightfield, verts, *hf, tmpVerts, sizeof(float) * 3 * nbVerts);
		OMNI_PVD_SETB(heightfield, tris, *hf, tmpIndices, sizeof(int) * 3 * nbFaces);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamHeightFieldGeometry(const physx::PxGeometry& g)
{
	PxGeometryHolder gh(g);
	OMNI_PVD_CREATE(geomheightfield, g);
	const physx::PxHeightFieldGeometry& geometry = gh.heightField();

	float vertScale[3];
	vertScale[0] = geometry.rowScale;
	vertScale[1] = geometry.heightScale;
	vertScale[2] = geometry.columnScale;
	OMNI_PVD_SET(geomheightfield, scale, g, vertScale);

	physx::PxHeightField const * hf = geometry.heightField;
	streamHeightField(hf);
	OMNI_PVD_SET(geomheightfield, heightField, g, hf);
}

void streamRigidStatic(const physx::PxRigidStatic* rs)
{
	const PxActor& a = *rs;
	OMNI_PVD_CREATE(actor, a);
	OMNI_PVD_SET(actor, type, a, PxActorType::eRIGID_STATIC);
	PxTransform t = rs->getGlobalPose();
	OMNI_PVD_SET(actor, translation, a, t.p);
	OMNI_PVD_SET(actor, rotation, a, t.q);
}

void streamRigidDynamic(const physx::PxRigidDynamic* rd)
{
	const PxActor& a = *rd;
	OMNI_PVD_CREATE(actor, a);
	OMNI_PVD_SET(actor, type, a, PxActorType::eRIGID_DYNAMIC);
	PxTransform t = rd->getGlobalPose();
	OMNI_PVD_SET(actor, translation, a, t.p);
	OMNI_PVD_SET(actor, rotation, a, t.q);
	//OMNI_PVD_SET(actor, isSleeping, a, rd->isSleeping()); // Seems to throw errors since you cannot call isSleeping before a body is part of a scene
	OMNI_PVD_SET(actor, sleepThreshold, a, rd->getSleepThreshold());
	OMNI_PVD_SET(actor, wakeCounter, a, rd->getWakeCounter());
	OMNI_PVD_SET(actor, stabilizationThreshold, a, rd->getStabilizationThreshold());
	PxU32 positionIters, velocityIters; rd->getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET(actor, positionIterations, a, positionIters);
	OMNI_PVD_SET(actor, velocityIterations, a, velocityIters);
	OMNI_PVD_SET(actor, rigidDynamicLockFlags, a, rd->getRigidDynamicLockFlags());
	OMNI_PVD_SET(actor, contactReportThreshold, a, rd->getContactReportThreshold());
}

void streamArticulationJoint(const physx::PxArticulationJointReducedCoordinate* joint)
{
	const PxArticulationJointReducedCoordinate& jointRef = *joint;
	PxArticulationJointType::Enum jointType = jointRef.getJointType();
	const PxArticulationLink* parentPxLinkPtr = &jointRef.getParentArticulationLink();
	const PxArticulationLink* childPxLinkPtr = &jointRef.getChildArticulationLink();
	PxArticulationMotion::Enum motions[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		motions[ax] = jointRef.getMotion(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal armatures[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		armatures[ax] = jointRef.getArmature(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal coefficient = jointRef.getFrictionCoefficient();
	PxReal maxJointV = jointRef.getMaxJointVelocity();
	PxReal positions[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		positions[ax] = jointRef.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal velocitys[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		velocitys[ax] = jointRef.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
	const char* concreteTypeName = jointRef.getConcreteTypeName();
	PxU32 concreteTypeNameLen = PxU32(strlen(concreteTypeName)) + 1;
	PxReal lowlimits[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		lowlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).low;
	PxReal highlimits[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		highlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).high;
	PxReal stiffnesss[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		stiffnesss[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).stiffness;
	PxReal dampings[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		dampings[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).damping;
	PxReal maxforces[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		maxforces[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).maxForce;
	PxArticulationDriveType::Enum drivetypes[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		drivetypes[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).driveType;
	PxReal drivetargets[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		drivetargets[ax] = jointRef.getDriveTarget(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal drivevelocitys[6];
	for (PxU32 ax = 0; ax < 6; ++ax)
		drivevelocitys[ax] = jointRef.getDriveVelocity(static_cast<PxArticulationAxis::Enum>(ax));

	OMNI_PVD_CREATE(articulationjoint, jointRef);
	OMNI_PVD_SET(articulationjoint, type, jointRef, jointType);
	OMNI_PVD_SET(articulationjoint, parentLink, jointRef, parentPxLinkPtr);
	OMNI_PVD_SET(articulationjoint, childLink, jointRef, childPxLinkPtr);
	OMNI_PVD_SETB(articulationjoint, motion, jointRef, motions, sizeof(motions));
	OMNI_PVD_SETB(articulationjoint, armature, jointRef, armatures, sizeof(armatures));
	OMNI_PVD_SET(articulationjoint, frictionCoefficient, jointRef, coefficient);
	OMNI_PVD_SET(articulationjoint, maxJointVelocity, jointRef, maxJointV);
	OMNI_PVD_SETB(articulationjoint, jointPosition, jointRef, positions, sizeof(positions));
	OMNI_PVD_SETB(articulationjoint, jointVelocity, jointRef, velocitys, sizeof(velocitys));
	OMNI_PVD_SETB(articulationjoint, concreteTypeName, jointRef, concreteTypeName, concreteTypeNameLen);
	OMNI_PVD_SETB(articulationjoint, limitLow, jointRef, lowlimits, sizeof(lowlimits));
	OMNI_PVD_SETB(articulationjoint, limitHigh, jointRef, highlimits, sizeof(highlimits));
	OMNI_PVD_SETB(articulationjoint, driveStiffness, jointRef, stiffnesss, sizeof(stiffnesss));
	OMNI_PVD_SETB(articulationjoint, driveDamping, jointRef, dampings, sizeof(dampings));
	OMNI_PVD_SETB(articulationjoint, driveMaxForce, jointRef, maxforces, sizeof(maxforces));
	OMNI_PVD_SETB(articulationjoint, driveType, jointRef, drivetypes, sizeof(drivetypes));
	OMNI_PVD_SETB(articulationjoint, driveTarget, jointRef, drivetargets, sizeof(drivetargets));
	OMNI_PVD_SETB(articulationjoint, driveVelocity, jointRef, drivevelocitys, sizeof(drivevelocitys));
}

void streamArticulationLink(const physx::PxArticulationLink* al)
{
	const PxActor& a = *al;
	OMNI_PVD_CREATE(actor, a);
	OMNI_PVD_SET(actor, type, a, PxActorType::eARTICULATION_LINK);
	OMNI_PVD_SET(actor, articulation, a, &al->getArticulation());
	OMNI_PVD_SET(actor, CFMScale, a, al->getCfmScale());
	OMNI_PVD_SET(actor, inboundJointDOF, a, al->getInboundJointDof());
}

void streamArticulation(const physx::PxArticulationReducedCoordinate* art)
{
	OMNI_PVD_CREATE(articulation, *art);
	PxU32 solverIterations[2]; art->getSolverIterationCounts(solverIterations[0], solverIterations[1]);
	OMNI_PVD_SET(articulation, positionIterations, *art, solverIterations[0]);
	OMNI_PVD_SET(articulation, velocityIterations, *art, solverIterations[1]);
	OMNI_PVD_SET(articulation, isSleeping, *art, false);
	OMNI_PVD_SET(articulation, sleepThreshold, *art, art->getSleepThreshold());
	OMNI_PVD_SET(articulation, stabilizationThreshold, *art, art->getStabilizationThreshold());
	OMNI_PVD_SET(articulation, wakeCounter, *art, art->getWakeCounter());
	OMNI_PVD_SET(articulation, maxLinVelocity, *art, art->getMaxCOMLinearVelocity());
	OMNI_PVD_SET(articulation, maxAngVelocity, *art, art->getMaxCOMAngularVelocity());
	OMNI_PVD_SET(articulation, worldBounds, *art, art->getWorldBounds());
	OMNI_PVD_SET(articulation, articulationFlags, *art, art->getArticulationFlags());
	OMNI_PVD_SET(articulation, dofs, *art, art->getDofs());
}

void streamAggregate(const physx::PxAggregate* agg)
{
	OMNI_PVD_CREATE(aggregate, *agg);
	PxU32 actorCount = agg->getNbActors();
	for (PxU32 i = 0; i < actorCount; ++i)
	{
		PxActor* a; agg->getActors(&a, 1, i);
		OMNI_PVD_ADD(aggregate, actors, *agg, *a);
	}
	OMNI_PVD_SET(aggregate, selfCollision, *agg, agg->getSelfCollision());
	OMNI_PVD_SET(aggregate, maxNbShapes, *agg, agg->getMaxNbShapes());
	OMNI_PVD_SET(aggregate, scene, *agg, const_cast<physx::PxAggregate*>(agg)->getScene());
}

void streamSoBo(const physx::PxSoftBody* b)
{
	OMNI_PVD_CREATE(actor, *b); // @@@
}

void streamMPMMaterial(const physx::PxMPMMaterial* m)
{
	OMNI_PVD_CREATE(mpmmaterial, *m);
}

void streamFLIPMaterial(const physx::PxFLIPMaterial* m)
{
	OMNI_PVD_CREATE(flipmaterial, *m);
}

void streamPBDMaterial(const physx::PxPBDMaterial* m)
{
	OMNI_PVD_CREATE(pbdmaterial, *m);
}

void streamFEMClothMaterial(const physx::PxFEMClothMaterial* m)
{
	OMNI_PVD_CREATE(femclothmaterial, *m);
}

void streamFEMSoBoMaterial(const physx::PxFEMSoftBodyMaterial* m)
{
	OMNI_PVD_CREATE(femsoftbodymaterial, *m);
}

void streamMaterial(const physx::PxMaterial* m)
{
	OMNI_PVD_CREATE(material, *m);
	OMNI_PVD_SET(material, flags, *m, m->getFlags());
	OMNI_PVD_SET(material, frictionCombineMode, *m, m->getFrictionCombineMode());
	OMNI_PVD_SET(material, restitutionCombineMode, *m, m->getRestitutionCombineMode());
	OMNI_PVD_SET(material, staticFriction, *m, m->getStaticFriction());
	OMNI_PVD_SET(material, dynamicFriction, *m, m->getDynamicFriction());
	OMNI_PVD_SET(material, restitution, *m, m->getRestitution());
}

void streamShapeMaterials(physx::PxShape* shapePtr, physx::PxMaterial** mats, physx::PxU32 nbrMaterials)
{
	OMNI_PVD_SETB(shape, materials, *shapePtr, mats, sizeof(PxMaterial*) * nbrMaterials);
}

void streamShape(const physx::PxShape* shape)
{
	OMNI_PVD_CREATE(shape, *shape);
	OMNI_PVD_SET(shape, isExclusive, *shape, shape->isExclusive());
	OMNI_PVD_SET(shape, geom, *shape, &shape->getGeometry());
	OMNI_PVD_SET(shape, contactOffset, *shape, shape->getContactOffset());
	OMNI_PVD_SET(shape, restOffset, *shape, shape->getRestOffset());
	OMNI_PVD_SET(shape, densityForFluid, *shape, shape->getDensityForFluid());
	OMNI_PVD_SET(shape, torsionalPatchRadius, *shape, shape->getTorsionalPatchRadius());
	OMNI_PVD_SET(shape, minTorsionalPatchRadius, *shape, shape->getMinTorsionalPatchRadius());
	OMNI_PVD_SET(shape, shapeFlags, *shape, shape->getFlags());
	OMNI_PVD_SET(shape, simulationFilterData, *shape, shape->getSimulationFilterData());
	OMNI_PVD_SET(shape, queryFilterData, *shape, shape->getQueryFilterData());

	const int nbrMaterials = shape->getNbMaterials();
	PxMaterial** tmpMaterials = (PxMaterial**)PX_ALLOC(sizeof(PxMaterial*) * nbrMaterials, "tmpMaterials");
	physx::PxU32 nbrMats = shape->getMaterials(tmpMaterials, nbrMaterials);
	streamShapeMaterials((PxShape*)shape, tmpMaterials, nbrMats);

	PX_FREE(tmpMaterials);
}

void streamBVH(const physx::PxBVH* bvh)
{
	OMNI_PVD_CREATE(bvh, *bvh);
}

void streamSoBoMesh(const physx::PxSoftBodyMesh* mesh)
{
	OMNI_PVD_CREATE(softbodymesh, *mesh);
	OMNI_PVD_SET(softbodymesh, collisionMesh, *mesh, mesh->getCollisionMesh());
	OMNI_PVD_SET(softbodymesh, simulationMesh, *mesh, mesh->getSimulationMesh());
}

void streamTetMesh(const physx::PxTetrahedronMesh* mesh)
{
	OMNI_PVD_CREATE(tetrahedronmesh, *mesh);
	//this gets done at the bottom now
	const physx::PxU32 tetrahedronCount = mesh->getNbTetrahedrons();
	const physx::PxU32 has16BitIndices = mesh->getTetrahedronMeshFlags() & physx::PxTetrahedronMeshFlag::e16_BIT_INDICES;
	const void* indexBuffer = mesh->getTetrahedrons();
	const physx::PxVec3* vertexBuffer = mesh->getVertices();
	const physx::PxU32* intIndices = reinterpret_cast<const physx::PxU32*>(indexBuffer);
	const physx::PxU16* shortIndices = reinterpret_cast<const physx::PxU16*>(indexBuffer);
	//TODO: not needed to copy this
	const int nbrVerts = mesh->getNbVertices();
	const int nbrTets = mesh->getNbTetrahedrons();
	float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
	int vertIndex = 0;
	for (int v = 0; v < nbrVerts; v++)
	{
		tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
		tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
		tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
		vertIndex += 3;
	}
	int* tmpIndices = (int*)PX_ALLOC(sizeof(int)*(nbrTets * 4), "tmpIndices");
	const int totalIndexCount = tetrahedronCount * 4;
	if (has16BitIndices)
	{
		for (int i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = shortIndices[i];
		}
	}
	else
	{
		for (int i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = intIndices[i];
		}
	}
	OMNI_PVD_SETB(tetrahedronmesh, verts, *mesh, tmpVerts, sizeof(float) * 3 * nbrVerts);
	OMNI_PVD_SETB(tetrahedronmesh, tets, *mesh, tmpIndices, sizeof(int) * 4 * nbrTets);
	PX_FREE(tmpVerts);
	PX_FREE(tmpIndices);
}

void streamTriMesh(const physx::PxTriangleMesh* mesh)
{
	if (samplerInternals->addSharedMeshIfNotSeen(mesh, OmniPvdSharedMeshEnum::eOmniPvdTriMesh))
	{
		OMNI_PVD_CREATE(trianglemesh, *mesh);
		//this gets done at the bottom now
		const physx::PxU32 triangleCount = mesh->getNbTriangles();
		const physx::PxU32 has16BitIndices = mesh->getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES;
		const void* indexBuffer = mesh->getTriangles();
		const physx::PxVec3* vertexBuffer = mesh->getVertices();
		const physx::PxU32* intIndices = reinterpret_cast<const physx::PxU32*>(indexBuffer);
		const physx::PxU16* shortIndices = reinterpret_cast<const physx::PxU16*>(indexBuffer);
		//TODO: not needed to copy this
		const int nbrVerts = mesh->getNbVertices();
		const int nbrTris = mesh->getNbTriangles();
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
		int vertIndex = 0;
		for (int v = 0; v < nbrVerts; v++)
		{
			tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
			tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
			tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
			vertIndex += 3;
		}
		int* tmpIndices = (int*)PX_ALLOC(sizeof(float)*(nbrTris * 3), "tmpIndices");
		const int totalIndexCount = triangleCount * 3;
		if (has16BitIndices)
		{
			for (int i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = shortIndices[i];
			}
		}
		else
		{
			for (int i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = intIndices[i];
			}
		}
		OMNI_PVD_SETB(trianglemesh, verts, *mesh, tmpVerts, sizeof(float) * 3 * nbrVerts);
		OMNI_PVD_SETB(trianglemesh, tris, *mesh, tmpIndices, sizeof(int) * 3 * nbrTris);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamTriMeshGeometry(const physx::PxGeometry& g)
{
	PxGeometryHolder gh(g);
	OMNI_PVD_CREATE(geomtrianglemesh, g);
	const physx::PxTriangleMeshGeometry& geometry = gh.triangleMesh();
	physx::PxTriangleMesh const * mesh = geometry.triangleMesh;
	OMNI_PVD_SET(geomtrianglemesh, scale, g, geometry.scale.scale);
	streamTriMesh(mesh);
	OMNI_PVD_SET(geomtrianglemesh, triangleMesh, g, mesh);
}

void OmniPvdPxSampler::streamSceneContacts(physx::NpScene* scene)
{
	if (!isSampling()) return;
	PxsContactManagerOutputIterator outputIter;
	Sc::ContactIterator contactIter;
	scene->getScScene().initContactsIterator(contactIter, outputIter);
	Sc::ContactIterator::Pair* pair;
	Sc::Contact* contact;
	PxU32 pairCount = 0;
	PxArray<PxActor*> pairsActors;
	PxArray<PxU32> pairsContactCounts;
	PxArray<PxVec3> pairsContactPoints;
	PxArray<PxVec3> pairsContactNormals;
	PxArray<PxReal> pairsContactSeparations;
	PxArray<PxShape*> pairsContactShapes;
	PxArray<PxU32> pairsContactFacesIndices;
	
	Sc::Interaction* interaction = contactIter.getCurrentInteraction();
	while ((pair = contactIter.getNextPair()) != NULL)
	{
		++pairCount;		
		pairsActors.pushBack(interaction->getActorSim0().getPxActor());
		pairsActors.pushBack(interaction->getActorSim1().getPxActor());
		PxU32 pairContactCount = 0;
		while ((contact = pair->getNextContact()) != NULL)
		{
			++pairContactCount;
			pairsContactPoints.pushBack(contact->point);
			pairsContactNormals.pushBack(contact->normal);
			pairsContactSeparations.pushBack(contact->separation);
			pairsContactShapes.pushBack(contact->shape0);
			pairsContactShapes.pushBack(contact->shape1);
			pairsContactFacesIndices.pushBack(contact->faceIndex0);
			pairsContactFacesIndices.pushBack(contact->faceIndex1);
		}
		pairsContactCounts.pushBack(pairContactCount);
		interaction = contactIter.getCurrentInteraction();
	}
	OMNI_PVD_SET(scene, pairCount, *scene, pairCount);
	PxU32 actorsSize = pairsActors.size() * sizeof(PxActor*);
	PxActor** actors = actorsSize ? &pairsActors[0] : NULL;
	OMNI_PVD_SETB(scene, pairsActors, *scene, actors, actorsSize);
	PxU32 contactCountsSize = pairsContactCounts.size() * sizeof(PxU32);
	PxU32* contactCounts = contactCountsSize ? &pairsContactCounts[0] : NULL;
	OMNI_PVD_SETB(scene, pairsContactCounts, *scene, contactCounts, contactCountsSize);
	PxU32 contactPointsSize = pairsContactPoints.size() * sizeof(PxVec3);
	PxReal* contactPoints = contactPointsSize ? &pairsContactPoints[0].x : NULL;
	OMNI_PVD_SETB(scene, pairsContactPoints, *scene, contactPoints, contactPointsSize);
	PxU32 contactNormalsSize = pairsContactNormals.size() * sizeof(PxVec3);
	PxReal* contactNormals = contactNormalsSize ? &pairsContactNormals[0].x : NULL;
	OMNI_PVD_SETB(scene, pairsContactNormals, *scene, contactNormals, contactNormalsSize);
	PxU32 contactSeparationsSize = pairsContactSeparations.size() * sizeof(PxReal);
	PxReal* contactSeparations = contactSeparationsSize ? &pairsContactSeparations[0] : NULL;
	OMNI_PVD_SETB(scene, pairsContactSeparations, *scene, contactSeparations, contactSeparationsSize);
	PxU32 contactShapesSize = pairsContactShapes.size() * sizeof(PxShape*);
	PxShape** contactShapes = contactShapesSize ? &pairsContactShapes[0] : NULL;
	OMNI_PVD_SETB(scene, pairsContactShapes, *scene, contactShapes, contactShapesSize);
	PxU32 contactFacesIndicesSize = pairsContactFacesIndices.size() * sizeof(PxU32);
	PxU32* contactFacesIndices = contactFacesIndicesSize ? &pairsContactFacesIndices[0] : NULL;
	OMNI_PVD_SETB(scene, pairsContactFacesIndices, *scene, contactFacesIndices, contactFacesIndicesSize);
}

OmniPvdPxSampler::OmniPvdPxSampler()
{
	samplerInternals = PX_NEW(OmniPvdSamplerInternals)();

	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	samplerInternals->mIsSampling = false;

	//TODO: this could be done better
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT8] = 1;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT16] = 2;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT32] = 4;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT64] = 8;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT8] = 1;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT16] = 2;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT32] = 4;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT64] = 8;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eFLOAT32] = 4;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eFLOAT64] = 8;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eSTRING] = 1;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eOBJECT_HANDLE] = sizeof(uint64_t);
}

OmniPvdPxSampler::~OmniPvdPxSampler()
{
	physx::PxHashMap<physx::NpScene*, OmniPvdPxScene*>::Iterator iterScenes = samplerInternals->mSampledScenes.getIterator();
	while (!iterScenes.done())
	{
		OmniPvdPxScene* scene = iterScenes->second;
		PX_DELETE(scene);
		iterScenes++;
	}
	PX_DELETE(samplerInternals);
}

void OmniPvdPxSampler::startSampling()
{
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	if (samplerInternals->mIsSampling)
	{
		return;
	}
	if (samplerInternals->mPvdStream.initOmniPvd())
	{
		samplerInternals->mIsSampling = true;
	}
}

bool OmniPvdPxSampler::isSampling()
{
	if (!samplerInternals) return false;
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	return samplerInternals->mIsSampling;
}

void OmniPvdPxSampler::setOmniPvdWriter(OmniPvdWriter* omniPvdWriter)
{
	samplerInternals->mPvdStream.setOmniPvdWriter(omniPvdWriter);
}

void createGeometry(const physx::PxGeometry & pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		streamSphere(pxGeom);
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		streamCapsule(pxGeom);
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		streamBox(pxGeom);
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		streamTriMeshGeometry(pxGeom);
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		streamConvexMeshGeometry(pxGeom);
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		streamHeightFieldGeometry(pxGeom);
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		streamPlane(pxGeom);
	}	
	break;
	default:
	break;
	}
}

void destroyGeometry(const physx::PxGeometry & pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		OMNI_PVD_DESTROY(geomsphere, pxGeom);
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		OMNI_PVD_DESTROY(geomcapsule, pxGeom);
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		OMNI_PVD_DESTROY(geombox, pxGeom);
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		OMNI_PVD_DESTROY(geomtrianglemesh, pxGeom);
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		OMNI_PVD_DESTROY(geomconvexmesh, pxGeom);
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		OMNI_PVD_DESTROY(geomheightfield, pxGeom);
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		OMNI_PVD_DESTROY(geomplane, pxGeom);
	}	
	break;
	default:
	break;
	}
}

void OmniPvdPxSampler::sampleScene(physx::NpScene* scene)
{
	{
		physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
		if (!samplerInternals->mIsSampling) return;
	}
	OmniPvdPxScene* ovdScene = getSampledScene(scene);
	ovdScene->sampleScene();
}

void OmniPvdPxSampler::onObjectAdd(const physx::PxBase* object)
{
	if (!isSampling()) return;
	switch (object->getConcreteType())
	{
	case physx::PxConcreteType::eHEIGHTFIELD:
		streamHeightField(static_cast<const physx::PxHeightField*>(object));
		OMNI_PVD_ADD(physics, heightFields, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxHeightField&>(*object));
		break;
	case physx::PxConcreteType::eCONVEX_MESH:
		streamConvexMesh(static_cast<const physx::PxConvexMesh*>(object));
		OMNI_PVD_ADD(physics, convexMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxConvexMesh&>(*object));
		break;
	case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
	case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		streamTriMesh(static_cast<const physx::PxTriangleMesh*>(object));
		OMNI_PVD_ADD(physics, triangleMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxTriangleMesh&>(*object));
		break;
	case physx::PxConcreteType::eTETRAHEDRON_MESH:
		streamTetMesh(static_cast<const physx::PxTetrahedronMesh*>(object));
		OMNI_PVD_ADD(physics, tetrahedronMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxTetrahedronMesh&>(*object));
		break;
	case physx::PxConcreteType::eSOFTBODY_MESH:
		streamSoBoMesh(static_cast<const physx::PxSoftBodyMesh*>(object));
		OMNI_PVD_ADD(physics, softBodyMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxSoftBodyMesh&>(*object));
		break;
	case physx::PxConcreteType::eBVH:
		streamBVH(static_cast<const physx::PxBVH*>(object));
		OMNI_PVD_ADD(physics, bvhs, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxBVH&>(*object));
		break;
	case physx::PxConcreteType::eSHAPE:
		createGeometry(static_cast<const physx::PxShape*>(object)->getGeometry());
		streamShape(static_cast<const physx::PxShape*>(object));
		OMNI_PVD_ADD(physics, shapes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxShape&>(*object));
		break;
	case physx::PxConcreteType::eMATERIAL:
		streamMaterial(static_cast<const physx::PxMaterial*>(object));
		OMNI_PVD_ADD(physics, materials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxMaterial&>(*object));
		break;
	case physx::PxConcreteType::eSOFTBODY_MATERIAL:
		streamFEMSoBoMaterial(static_cast<const physx::PxFEMSoftBodyMaterial*>(object));
		OMNI_PVD_ADD(physics, FEMSoftBodyMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxFEMSoftBodyMaterial&>(*object));
		break;
	case physx::PxConcreteType::eCLOTH_MATERIAL:
		streamFEMClothMaterial(static_cast<const physx::PxFEMClothMaterial*>(object));
		OMNI_PVD_ADD(physics, FEMClothMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxFEMClothMaterial&>(*object));
		break;
	case physx::PxConcreteType::ePBD_MATERIAL:
		streamPBDMaterial(static_cast<const physx::PxPBDMaterial*>(object));
		OMNI_PVD_ADD(physics, PBDMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxPBDMaterial&>(*object));
		break;
	case physx::PxConcreteType::eFLIP_MATERIAL:
		streamFLIPMaterial(static_cast<const physx::PxFLIPMaterial*>(object));
		OMNI_PVD_ADD(physics, FLIPMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxFLIPMaterial&>(*object));
		break;
	case physx::PxConcreteType::eMPM_MATERIAL:
		streamMPMMaterial(static_cast<const physx::PxMPMMaterial*>(object));
		OMNI_PVD_ADD(physics, MPMMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxMPMMaterial&>(*object));
		break;
	case physx::PxConcreteType::eSOFT_BODY:
		streamSoBo(static_cast<const physx::PxSoftBody*>(object));
		OMNI_PVD_ADD(physics, softBodies, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxActor&>(*object));
		break;
	case physx::PxConcreteType::eAGGREGATE:
		streamAggregate(static_cast<const physx::PxAggregate*>(object));
		OMNI_PVD_ADD(physics, aggregates, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxAggregate&>(*object));
		break;
	case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		streamArticulation(static_cast<const physx::PxArticulationReducedCoordinate*>(object));
		OMNI_PVD_ADD(physics, articulations, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxArticulationReducedCoordinate&>(*object));
		break;
	case physx::PxConcreteType::eARTICULATION_LINK:
		streamArticulationLink(static_cast<const physx::PxArticulationLink*>(object));
		OMNI_PVD_ADD(articulation, links, static_cast<const physx::PxArticulationLink*>(object)->getArticulation(), static_cast<const physx::PxArticulationLink&>(*object));
		break;
	case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		streamArticulationJoint(static_cast<const physx::PxArticulationJointReducedCoordinate*>(object));
		break;
	case physx::PxConcreteType::eRIGID_DYNAMIC:
		streamRigidDynamic(static_cast<const physx::PxRigidDynamic*>(object));
		OMNI_PVD_ADD(physics, rigidDynamics, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxRigidDynamic&>(*object));
		break;
	case physx::PxConcreteType::eRIGID_STATIC:
		streamRigidStatic(static_cast<const physx::PxRigidStatic*>(object));
		OMNI_PVD_ADD(physics, rigidStatics, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxRigidStatic&>(*object));
		break;
	}
}

void OmniPvdPxSampler::onObjectRemove(const physx::PxBase* object)
{
	if (!isSampling()) return;
	switch (object->getConcreteType())
	{
	case physx::PxConcreteType::eHEIGHTFIELD:
		OMNI_PVD_REMOVE(physics, heightFields, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxHeightField&>(*object));
		OMNI_PVD_DESTROY(heightfield, static_cast<const physx::PxHeightField&>(*object));
		break;
	case physx::PxConcreteType::eCONVEX_MESH:
		OMNI_PVD_REMOVE(physics, convexMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxConvexMesh&>(*object));
		OMNI_PVD_DESTROY(convexmesh, static_cast<const physx::PxConvexMesh&>(*object));
		break;
	case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
	case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		OMNI_PVD_REMOVE(physics, triangleMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxTriangleMesh&>(*object));
		OMNI_PVD_DESTROY(trianglemesh, static_cast<const physx::PxTriangleMesh&>(*object));
		break;
	case physx::PxConcreteType::eTETRAHEDRON_MESH:
		OMNI_PVD_REMOVE(physics, tetrahedronMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxTetrahedronMesh&>(*object));
		OMNI_PVD_DESTROY(tetrahedronmesh, static_cast<const physx::PxTetrahedronMesh&>(*object));
		break;
	case physx::PxConcreteType::eSOFTBODY_MESH:
		OMNI_PVD_REMOVE(physics, softBodyMeshes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxSoftBodyMesh&>(*object));
		OMNI_PVD_DESTROY(softbodymesh, static_cast<const physx::PxSoftBodyMesh&>(*object));
		break;
	case physx::PxConcreteType::eBVH:
		OMNI_PVD_REMOVE(physics, bvhs, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxBVH&>(*object));
		OMNI_PVD_DESTROY(bvh, static_cast<const physx::PxBVH&>(*object));
		break;
	case physx::PxConcreteType::eSHAPE:
		OMNI_PVD_REMOVE(physics, shapes, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxShape&>(*object));
		OMNI_PVD_DESTROY(shape, static_cast<const physx::PxShape&>(*object));
		destroyGeometry(static_cast<const physx::PxShape&>(*object).getGeometry());
		break;
	case physx::PxConcreteType::eMATERIAL:
		OMNI_PVD_REMOVE(physics, materials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxMaterial&>(*object));
		OMNI_PVD_DESTROY(material, static_cast<const physx::PxMaterial&>(*object));
		break;
	case physx::PxConcreteType::eSOFTBODY_MATERIAL:
		OMNI_PVD_REMOVE(physics, FEMSoftBodyMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxFEMSoftBodyMaterial&>(*object));
		OMNI_PVD_DESTROY(femsoftbodymaterial, static_cast<const physx::PxFEMSoftBodyMaterial&>(*object));
		break;
	case physx::PxConcreteType::eCLOTH_MATERIAL:
		OMNI_PVD_REMOVE(physics, FEMClothMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxFEMClothMaterial&>(*object));
		OMNI_PVD_DESTROY(femclothmaterial, static_cast<const physx::PxFEMClothMaterial&>(*object));
		break;
	case physx::PxConcreteType::ePBD_MATERIAL:
		OMNI_PVD_REMOVE(physics, PBDMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxPBDMaterial&>(*object));
		OMNI_PVD_DESTROY(pbdmaterial, static_cast<const physx::PxPBDMaterial&>(*object));
		break;
	case physx::PxConcreteType::eFLIP_MATERIAL:
		OMNI_PVD_REMOVE(physics, FLIPMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxFLIPMaterial&>(*object));
		OMNI_PVD_DESTROY(flipmaterial, static_cast<const physx::PxFLIPMaterial&>(*object));
		break;
	case physx::PxConcreteType::eMPM_MATERIAL:
		OMNI_PVD_REMOVE(physics, MPMMaterials, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxMPMMaterial&>(*object));
		OMNI_PVD_DESTROY(mpmmaterial, static_cast<const physx::PxMPMMaterial&>(*object));
		break;
	case physx::PxConcreteType::eSOFT_BODY:
		OMNI_PVD_REMOVE(physics, softBodies, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxSoftBody&>(*object));
		OMNI_PVD_DESTROY(actor, static_cast<const physx::PxSoftBody&>(*object)); // @@@
		break;
	case physx::PxConcreteType::eAGGREGATE:
		OMNI_PVD_REMOVE(physics, aggregates, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxAggregate&>(*object));
		OMNI_PVD_DESTROY(aggregate, static_cast<const physx::PxAggregate&>(*object));
		break;
	case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		OMNI_PVD_REMOVE(physics, articulations, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxArticulationReducedCoordinate&>(*object));
		OMNI_PVD_DESTROY(articulation, static_cast<const physx::PxArticulationReducedCoordinate&>(*object));
		break;
	case physx::PxConcreteType::eARTICULATION_LINK:
		OMNI_PVD_DESTROY(actor, static_cast<const physx::PxArticulationLink&>(*object));
		break;
	case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		OMNI_PVD_DESTROY(articulationjoint, static_cast<const physx::PxArticulationJointReducedCoordinate&>(*object));
		break;
	case physx::PxConcreteType::eRIGID_DYNAMIC:
		OMNI_PVD_REMOVE(physics, rigidDynamics, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxRigidDynamic&>(*object));
		OMNI_PVD_DESTROY(actor, static_cast<const physx::PxRigidDynamic&>(*object));
		break;
	case physx::PxConcreteType::eRIGID_STATIC:
		OMNI_PVD_REMOVE(physics, rigidStatics, static_cast<PxPhysics&>(NpPhysics::getInstance()), static_cast<const physx::PxRigidStatic&>(*object));
		OMNI_PVD_DESTROY(actor, static_cast<const physx::PxRigidStatic&>(*object));
		break;
	}
}

OmniPvdPxScene* OmniPvdPxSampler::getSampledScene(physx::NpScene* scene)
{
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampledScenesMutex);
	const physx::PxHashMap<physx::NpScene*, OmniPvdPxScene*>::Entry* entry = samplerInternals->mSampledScenes.find(scene);
	if (entry)
	{
		return entry->second;
	}
	else
	{
		OmniPvdPxScene* ovdScene = PX_NEW(OmniPvdPxScene)();
		samplerInternals->mSampledScenes[scene] = ovdScene;
		return ovdScene;
	}
}

// Returns true if the Geom was not yet seen and added
bool OmniPvdSamplerInternals::addSharedMeshIfNotSeen(const void* geom, OmniPvdSharedMeshEnum geomEnum)
{
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSharedGeomsMutex);
	const physx::PxHashMap<const void*, OmniPvdSharedMeshEnum>::Entry* entry = samplerInternals->mSharedMeshesMap.find(geom);
	if (entry)
	{
		return false;
	}
	else
	{
		samplerInternals->mSharedMeshesMap[geom] = geomEnum;
		return true;
	}
}

//generic PVD API: 
//TODO: Put back context handles.

template <typename ClassType> void OmniPvdPxSampler::createObject(OmniPvdClassHandle ch, ClassType const & objectId)
{
	samplerInternals->mPvdStream.mWriter->createObject(UNNECESSARY_SCENE_HANDLE, ch, OmniPvdObjectHandle(&objectId), NULL);
}

template <typename ClassType> void OmniPvdPxSampler::destroyObject(ClassType const & objectId)
{
	samplerInternals->mPvdStream.mWriter->destroyObject(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId));
}

template <typename ClassType, typename AttributeType> void OmniPvdPxSampler::setAttribute(OmniPvdAttributeHandle ah, const ClassType & objectId, const AttributeType & value)
{
	samplerInternals->mPvdStream.mWriter->setAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)&value, sizeof(AttributeType));
}

template <typename ClassType, typename AttributeType> void OmniPvdPxSampler::setAttributeBytes(OmniPvdAttributeHandle ah, ClassType const & objectId, const AttributeType * value, unsigned nBytes)
{
	samplerInternals->mPvdStream.mWriter->setAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)value, nBytes);
}

template <typename ClassType, typename AttributeType> void OmniPvdPxSampler::addToSet(OmniPvdAttributeHandle ah, ClassType const & objectId, AttributeType const & value)
{
	const AttributeType * atp = &value;
	samplerInternals->mPvdStream.mWriter->addToSetAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)&atp, sizeof(atp));
}

template <typename ClassType, typename AttributeType> void OmniPvdPxSampler::removeFromSet(OmniPvdAttributeHandle ah, ClassType const & objectId, AttributeType const & value)
{
	const AttributeType * atp = &value;
	samplerInternals->mPvdStream.mWriter->removeFromSetAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)&atp, sizeof(atp) );
}

///////////////////////////////////////////////////////////////////////////////

OmniPvdPxSampler* OmniPvdPxSampler::getInstance()
{
	PX_ASSERT(&physx::NpPhysics::getInstance() != NULL);
	return &physx::NpPhysics::getInstance() ? physx::NpPhysics::getInstance().mOmniPvdSampler : NULL;
}

#endif

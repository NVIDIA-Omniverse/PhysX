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

#if PX_SUPPORT_OMNI_PVD
#include <stdio.h>

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAllocator.h"

#include "ScIterators.h"

#include "omnipvd/NpOmniPvd.h"
#include "OmniPvdPxSampler.h"
#include "OmniPvdWriteStream.h"
#include "NpOmniPvdSetData.h"

#include "NpPhysics.h"
#include "NpScene.h"
#include "NpAggregate.h"
#include "NpRigidStatic.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationLink.h"


using namespace physx;

class OmniPvdStreamContainer
{
public:
	OmniPvdStreamContainer();
	~OmniPvdStreamContainer();
	bool initOmniPvd();
	void registerClasses();
	void setOmniPvdInstance(NpOmniPvd* omniPvdInstance);

	NpOmniPvd* mOmniPvdInstance;
	physx::PxMutex mMutex;
	OmniPvdPxCoreRegistrationData mRegistrationData;
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
		samplerInternals->mPvdStream.mOmniPvdInstance->getWriter()->startFrame(OMNI_PVD_CONTEXT_HANDLE, mFrameId);
	}

	physx::PxU64 mFrameId;
};

OmniPvdStreamContainer::OmniPvdStreamContainer()
{
	physx::PxMutex::ScopedLock myLock(mMutex);
	mClassesRegistered = false;
	mOmniPvdInstance = NULL;
}

OmniPvdStreamContainer::~OmniPvdStreamContainer()
{
}

void OmniPvdStreamContainer::setOmniPvdInstance(NpOmniPvd* omniPvdInstance)
{
	mOmniPvdInstance = omniPvdInstance;
}

bool OmniPvdStreamContainer::initOmniPvd()
{
	physx::PxMutex::ScopedLock myLock(mMutex);

	registerClasses();

	PxPhysics& physicsRef = static_cast<PxPhysics&>(NpPhysics::getInstance());

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, physicsRef);
	const physx::PxTolerancesScale& tolScale = physicsRef.getTolerancesScale();
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tolerancesScale, physicsRef, tolScale);
	OMNI_PVD_WRITE_SCOPE_END

	return true;
}

void OmniPvdStreamContainer::registerClasses()
{
	if (mClassesRegistered) return;
	OmniPvdWriter* writer = mOmniPvdInstance->acquireExclusiveWriterAccess();
	if (writer)
	{
		mRegistrationData.registerData(*writer);
		mClassesRegistered = true;
	}
	mOmniPvdInstance->releaseExclusiveWriterAccess();
}


int streamStringLength(const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	if (NpPhysics::getInstance().mOmniPvdSampler == NULL)
	{
		return 0;
	}
	if (name == NULL)
	{
		return 0;
	}
	int len = static_cast<int>(strlen(name));
	if (len > 0)
	{
		return len;
	}
	else
	{
		return 0;
	}
#else
	return 0;
#endif
}

void streamActorName(const physx::PxActor & a, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxActor, name, a, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamSceneName(const physx::PxScene & s, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxScene, name, s, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamSphereGeometry(const physx::PxSphereGeometry& g)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, radius, g, g.radius);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamCapsuleGeometry(const physx::PxCapsuleGeometry& g)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, halfHeight, g, g.halfHeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, radius, g, g.radius);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamBoxGeometry(const physx::PxBoxGeometry& g)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, halfExtents, g, g.halfExtents);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamPlaneGeometry(const physx::PxPlaneGeometry& g)
{
	OMNI_PVD_CREATE(OMNI_PVD_CONTEXT_HANDLE, PxPlaneGeometry, g);
}

void streamCustomGeometry(const physx::PxCustomGeometry& g)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, callbacks, g, g.callbacks);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamConvexMesh(const physx::PxConvexMesh& mesh)
{		
	if (samplerInternals->addSharedMeshIfNotSeen(&mesh, OmniPvdSharedMeshEnum::eOmniPvdConvexMesh))
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, mesh);

		const PxU32 nbPolys = mesh.getNbPolygons();
		const PxU8* polygons = mesh.getIndexBuffer();
		const PxVec3* verts = mesh.getVertices();
		const PxU32 nbrVerts = mesh.getNbVertices();

		PxU32 totalTris = 0;
		for (PxU32 i = 0; i < nbPolys; i++)
		{
			physx::PxHullPolygon data;
			mesh.getPolygonData(i, data);
			totalTris += data.mNbVerts - 2;
		}

		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(totalTris * 3), "tmpIndices");
		//TODO: this copy is useless

		PxU32 vertIndex = 0;
		for (PxU32 v = 0; v < nbrVerts; v++)
		{
			tmpVerts[vertIndex + 0] = verts[v].x;
			tmpVerts[vertIndex + 1] = verts[v].y;
			tmpVerts[vertIndex + 2] = verts[v].z;
			vertIndex += 3;
		}

		PxU32 triIndex = 0;
		for (PxU32 p = 0; p < nbPolys; p++)
		{
			physx::PxHullPolygon data;
			mesh.getPolygonData(p, data);
			PxU32 nbTris = data.mNbVerts - 2;
			const PxU32 vref0 = polygons[data.mIndexBase + 0 + 0];
			for (PxU32 t = 0; t < nbTris; t++)
			{
				const PxU32 vref1 = polygons[data.mIndexBase + t + 1];
				const PxU32 vref2 = polygons[data.mIndexBase + t + 2];
				tmpIndices[triIndex + 0] = vref0;
				tmpIndices[triIndex + 1] = vref1;
				tmpIndices[triIndex + 2] = vref2;
				triIndex += 3;
			}
		}

		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, tris, mesh, tmpIndices, 3 * totalTris);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
		OMNI_PVD_WRITE_SCOPE_END
	}
}

void streamConvexMeshGeometry(const physx::PxConvexMeshGeometry& g)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, scale, g, g.scale.scale);
	streamConvexMesh(*g.convexMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, convexMesh, g, g.convexMesh);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamHeightField(const physx::PxHeightField& hf)
{
	if (samplerInternals->addSharedMeshIfNotSeen(&hf, OmniPvdSharedMeshEnum::eOmniPvdHeightField))
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, hf);
		const PxU32 nbCols = hf.getNbColumns();
		const PxU32 nbRows = hf.getNbRows();
		const PxU32 nbVerts = nbRows * nbCols;
		const PxU32 nbFaces = (nbCols - 1) * (nbRows - 1) * 2;
		physx::PxHeightFieldSample* sampleBuffer = (physx::PxHeightFieldSample*)PX_ALLOC(sizeof(physx::PxHeightFieldSample)*(nbVerts), "sampleBuffer");
		hf.saveCells(sampleBuffer, nbVerts * sizeof(physx::PxHeightFieldSample));
		//TODO: are the copies necessary?
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbVerts * 3), "tmpVerts");
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbFaces * 3), "tmpIndices");
		for (PxU32 i = 0; i < nbRows; i++)
		{
			for (PxU32 j = 0; j < nbCols; j++)
			{
				const float x = PxReal(i);// *rs;
				const float y = PxReal(sampleBuffer[j + (i*nbCols)].height);// *hs;
				const float z = PxReal(j);// *cs;
				const PxU32 vertexIndex = 3 * (i * nbCols + j);
				float* vert = &tmpVerts[vertexIndex];
				vert[0] = x;
				vert[1] = y;
				vert[2] = z;
			}
		}
		for (PxU32 i = 0; i < (nbCols - 1); ++i)
		{
			for (PxU32 j = 0; j < (nbRows - 1); ++j)
			{
				PxU8 tessFlag = sampleBuffer[i + j * nbCols].tessFlag();
				PxU32 i0 = j * nbCols + i;
				PxU32 i1 = j * nbCols + i + 1;
				PxU32 i2 = (j + 1) * nbCols + i;
				PxU32 i3 = (j + 1) * nbCols + i + 1;
				// i2---i3
				// |    |
				// |    |
				// i0---i1
				// this is really a corner vertex index, not triangle index
				PxU32 mat0 = hf.getTriangleMaterialIndex((j*nbCols + i) * 2);
				PxU32 mat1 = hf.getTriangleMaterialIndex((j*nbCols + i) * 2 + 1);
				bool hole0 = (mat0 == PxHeightFieldMaterial::eHOLE);
				bool hole1 = (mat1 == PxHeightFieldMaterial::eHOLE);
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
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, verts, hf, tmpVerts, 3 * nbVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, tris, hf, tmpIndices, 3 * nbFaces);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
		OMNI_PVD_WRITE_SCOPE_END
	}
}

void streamHeightFieldGeometry(const physx::PxHeightFieldGeometry& g)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, g);

	PxVec3 vertScale(g.rowScale, g.heightScale, g.columnScale);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, scale, g, vertScale);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, heightField, g, g.heightField);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamActorAttributes(const physx::PxActor& actor)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, flags, actor, actor.getActorFlags());
	streamActorName(actor, actor.getName());
	// Should we stream the worldBounds if the actor is not part of a Scene yet?
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, actor, actor.getWorldBounds())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, dominance, actor, actor.getDominanceGroup())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, ownerClient, actor, actor.getOwnerClient())

	OMNI_PVD_WRITE_SCOPE_END
}

void streamRigidActorAttributes(const PxRigidActor &ra)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	
	if (ra.is<PxArticulationLink>())
	{
		const PxArticulationLink& link = static_cast<const PxArticulationLink&>(ra);
		PxTransform TArtLinkLocal;
		PxArticulationJointReducedCoordinate* joint = link.getInboundJoint();
		if (joint)
		{
			PxArticulationLink& parentLink = joint->getParentArticulationLink();
			// TGlobal = TFatherGlobal * TLocal
			// Inv(TFatherGlobal)* TGlobal = Inv(TFatherGlobal)*TFatherGlobal * TLocal
			// Inv(TFatherGlobal)* TGlobal = TLocal
			// TLocal = Inv(TFatherGlobal) * TGlobal
			//physx::PxTransform TParentGlobal = pxArticulationParentLink->getGlobalPose();
			PxTransform TParentGlobalInv = parentLink.getGlobalPose().getInverse();
			PxTransform TArtLinkGlobal = link.getGlobalPose();
			// PT:: tag: scalar transform*transform
			TArtLinkLocal = TParentGlobalInv * TArtLinkGlobal;
		}
		else
		{
			TArtLinkLocal = link.getGlobalPose();
		}
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, translation, ra, TArtLinkLocal.p)
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, rotation, ra, TArtLinkLocal.q)
	}
	else
	{
		PxTransform t = ra.getGlobalPose();	
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, translation, ra, t.p);
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, rotation, ra, t.q);
	}

	// Stream shapes too
	const int nbrShapes = ra.getNbShapes();
	for (int s = 0; s < nbrShapes; s++)
	{
		PxShape* shape[1];
		ra.getShapes(shape, 1, s);
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, shapes, ra, *shape[0])
	}

	OMNI_PVD_WRITE_SCOPE_END
}

void streamRigidBodyAttributes(const physx::PxRigidBody& rigidBody)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, cMassLocalPose, rigidBody, rigidBody.getCMassLocalPose());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, mass, rigidBody, rigidBody.getMass());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, massSpaceInertiaTensor, rigidBody, rigidBody.getMassSpaceInertiaTensor());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearDamping, rigidBody, rigidBody.getLinearDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularDamping, rigidBody, rigidBody.getAngularDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, rigidBody, rigidBody.getLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, rigidBody, rigidBody.getAngularVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxLinearVelocity, rigidBody, rigidBody.getMaxLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxAngularVelocity, rigidBody, rigidBody.getMaxAngularVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, rigidBody, rigidBody.getRigidBodyFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, minAdvancedCCDCoefficient, rigidBody, rigidBody.getMinCCDAdvanceCoefficient());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxDepenetrationVelocity, rigidBody, rigidBody.getMaxDepenetrationVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxContactImpulse, rigidBody, rigidBody.getMaxContactImpulse());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, contactSlopCoefficient, rigidBody, rigidBody.getContactSlopCoefficient());

	OMNI_PVD_WRITE_SCOPE_END
}

void streamRigidDynamicAttributes(const physx::PxRigidDynamic& rd)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	if (rd.getScene())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, isSleeping, rd, rd.isSleeping());
	}
	
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, sleepThreshold, rd, rd.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, stabilizationThreshold, rd, rd.getStabilizationThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rigidDynamicLockFlags, rd, rd.getRigidDynamicLockFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, wakeCounter, rd, rd.getWakeCounter());
	
	PxU32 positionIters, velocityIters; rd.getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, positionIterations, rd, positionIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, velocityIterations, rd, velocityIters);
	
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, contactReportThreshold, rd, rd.getContactReportThreshold());

	OMNI_PVD_WRITE_SCOPE_END
}

void streamRigidDynamic(const physx::PxRigidDynamic& rd)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	const PxActor& a = rd;
	PX_ASSERT(&a == &rd);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rd);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eRIGID_DYNAMIC);

	OMNI_PVD_WRITE_SCOPE_END
	
	streamActorAttributes(rd);
	streamRigidActorAttributes(rd);
	streamRigidBodyAttributes(rd);	
	streamRigidDynamicAttributes(rd);
}

void streamRigidStatic(const physx::PxRigidStatic& rs)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	const PxActor& a = rs;
	PX_ASSERT(&a == &rs);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidStatic, rs);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eRIGID_STATIC);

	OMNI_PVD_WRITE_SCOPE_END

	streamActorAttributes(rs);
	streamRigidActorAttributes(rs);
}

void streamArticulationJoint(const physx::PxArticulationJointReducedCoordinate& jointRef)
{
	const PxU32 degreesOfFreedom = PxArticulationAxis::eCOUNT;

	// make sure size matches the size used in the PVD description
	PX_ASSERT(sizeof(PxArticulationMotion::Enum) == getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>());
	PX_ASSERT(sizeof(PxArticulationDriveType::Enum) == getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>());

	PxArticulationJointType::Enum jointType = jointRef.getJointType();
	const PxArticulationLink* parentPxLinkPtr = &jointRef.getParentArticulationLink();
	const PxArticulationLink* childPxLinkPtr = &jointRef.getChildArticulationLink();
	PxArticulationMotion::Enum motions[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		motions[ax] = jointRef.getMotion(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal armatures[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		armatures[ax] = jointRef.getArmature(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal coefficient = jointRef.getFrictionCoefficient();
	PxReal maxJointV = jointRef.getMaxJointVelocity();
	PxReal positions[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		positions[ax] = jointRef.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal velocitys[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		velocitys[ax] = jointRef.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
	const char* concreteTypeName = jointRef.getConcreteTypeName();
	PxU32 concreteTypeNameLen = PxU32(strlen(concreteTypeName)) + 1;
	PxReal lowlimits[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		lowlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).low;
	PxReal highlimits[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		highlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).high;
	PxReal stiffnesss[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		stiffnesss[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).stiffness;
	PxReal dampings[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		dampings[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).damping;
	PxReal maxforces[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxforces[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).maxForce;
	PxArticulationDriveType::Enum drivetypes[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivetypes[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).driveType;
	PxReal drivetargets[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivetargets[ax] = jointRef.getDriveTarget(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal drivevelocitys[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivevelocitys[ax] = jointRef.getDriveVelocity(static_cast<PxArticulationAxis::Enum>(ax));

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointRef);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, type, jointRef, jointType);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentLink, jointRef, parentPxLinkPtr);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childLink, jointRef, childPxLinkPtr);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, motion, jointRef, motions, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, armature, jointRef, armatures, degreesOfFreedom);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, frictionCoefficient, jointRef, coefficient);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, maxJointVelocity, jointRef, maxJointV);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, jointRef, positions, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, jointRef, velocitys, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, concreteTypeName, jointRef, concreteTypeName, concreteTypeNameLen);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitLow, jointRef, lowlimits, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitHigh, jointRef, highlimits, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveStiffness, jointRef, stiffnesss, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveDamping, jointRef, dampings, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveMaxForce, jointRef, maxforces, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveType, jointRef, drivetypes, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveTarget, jointRef, drivetargets, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocity, jointRef, drivevelocitys, degreesOfFreedom);

	OMNI_PVD_WRITE_SCOPE_END
}

void streamArticulationLink(const physx::PxArticulationLink& al)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	const PxActor& a = al;
	PX_ASSERT(&a == &al);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, al);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eARTICULATION_LINK);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, articulation, al, &al.getArticulation());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, CFMScale, al, al.getCfmScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, inboundJointDOF, al, al.getInboundJointDof());

	OMNI_PVD_WRITE_SCOPE_END

	streamActorAttributes(al);
	streamRigidActorAttributes(al);
	streamRigidBodyAttributes(al);
}

void streamArticulation(const physx::PxArticulationReducedCoordinate& art)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, art);
	PxU32 solverIterations[2]; art.getSolverIterationCounts(solverIterations[0], solverIterations[1]);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, positionIterations, art, solverIterations[0]);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, velocityIterations, art, solverIterations[1]);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, isSleeping, art, false);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, sleepThreshold, art, art.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, stabilizationThreshold, art, art.getStabilizationThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, wakeCounter, art, art.getWakeCounter());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, maxLinearVelocity, art, art.getMaxCOMLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, maxAngularVelocity, art, art.getMaxCOMAngularVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, worldBounds, art, art.getWorldBounds());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, articulationFlags, art, art.getArticulationFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, dofs, art, art.getDofs());

	OMNI_PVD_WRITE_SCOPE_END
}

void streamAggregate(const physx::PxAggregate& agg)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, agg);
	PxU32 actorCount = agg.getNbActors();
	for (PxU32 i = 0; i < actorCount; ++i)
	{
		PxActor* a; agg.getActors(&a, 1, i);
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, actors, agg, *a);
	}
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, selfCollision, agg, agg.getSelfCollision());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, maxNbShapes, agg, agg.getMaxNbShapes());
	PxScene* scene = static_cast<const NpAggregate&>(agg).getNpScene();  // because PxAggregate::getScene() is not marked const
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, scene, agg, scene);

	OMNI_PVD_WRITE_SCOPE_END
}

void streamMPMMaterial(const physx::PxMPMMaterial& m)
{
	OMNI_PVD_CREATE(OMNI_PVD_CONTEXT_HANDLE, PxMPMMaterial, m);
}

void streamFLIPMaterial(const physx::PxFLIPMaterial& m)
{
	OMNI_PVD_CREATE(OMNI_PVD_CONTEXT_HANDLE, PxFLIPMaterial, m);
}

void streamPBDMaterial(const physx::PxPBDMaterial& m)
{
	OMNI_PVD_CREATE(OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, m);
}

void streamFEMClothMaterial(const physx::PxFEMClothMaterial& m)
{
	OMNI_PVD_CREATE(OMNI_PVD_CONTEXT_HANDLE, PxFEMClothMaterial, m);
}

void streamFEMSoBoMaterial(const physx::PxFEMSoftBodyMaterial& m)
{
	OMNI_PVD_CREATE(OMNI_PVD_CONTEXT_HANDLE, PxFEMSoftBodyMaterial, m);
}

void streamMaterial(const physx::PxMaterial& m)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, flags, m, m.getFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, frictionCombineMode, m, m.getFrictionCombineMode());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitutionCombineMode, m, m.getRestitutionCombineMode());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, staticFriction, m, m.getStaticFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, dynamicFriction, m, m.getDynamicFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitution, m, m.getRestitution());

	OMNI_PVD_WRITE_SCOPE_END
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	OMNI_PVD_SET_ARRAY(OMNI_PVD_CONTEXT_HANDLE, PxShape, materials, shape, mats, nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxFEMClothMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxFEMMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxFEMSoftBodyMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxFLIPMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxMPMMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxParticleMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxPBDMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}


void streamShape(const physx::PxShape& shape)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, shape);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, isExclusive, shape, shape.isExclusive());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, geom, shape, &shape.getGeometry());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, contactOffset, shape, shape.getContactOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, restOffset, shape, shape.getRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, densityForFluid, shape, shape.getDensityForFluid());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, torsionalPatchRadius, shape, shape.getTorsionalPatchRadius());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, minTorsionalPatchRadius, shape, shape.getMinTorsionalPatchRadius());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, shapeFlags, shape, shape.getFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, simulationFilterData, shape, shape.getSimulationFilterData());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, queryFilterData, shape, shape.getQueryFilterData());

	const int nbrMaterials = shape.getNbMaterials();
	PxMaterial** tmpMaterials = (PxMaterial**)PX_ALLOC(sizeof(PxMaterial*) * nbrMaterials, "tmpMaterials");
	physx::PxU32 nbrMats = shape.getMaterials(tmpMaterials, nbrMaterials);
	streamShapeMaterials(shape, tmpMaterials, nbrMats);

	PX_FREE(tmpMaterials);

	OMNI_PVD_WRITE_SCOPE_END
}

void streamBVH(const physx::PxBVH& bvh)
{
	OMNI_PVD_CREATE(OMNI_PVD_CONTEXT_HANDLE, PxBVH, bvh);
}

void streamSoBoMesh(const physx::PxSoftBodyMesh& mesh)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSoftBodyMesh, mesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSoftBodyMesh, collisionMesh, mesh, mesh.getCollisionMesh());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSoftBodyMesh, simulationMesh, mesh, mesh.getSimulationMesh());

	OMNI_PVD_WRITE_SCOPE_END
}

void streamTetMesh(const physx::PxTetrahedronMesh& mesh)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, mesh);
	//this gets done at the bottom now
	const PxU32 tetrahedronCount = mesh.getNbTetrahedrons();
	const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & physx::PxTetrahedronMeshFlag::e16_BIT_INDICES;
	const void* indexBuffer = mesh.getTetrahedrons();
	const PxVec3* vertexBuffer = mesh.getVertices();
	const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
	const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
	//TODO: not needed to copy this
	const PxU32 nbrVerts = mesh.getNbVertices();
	const PxU32 nbrTets = mesh.getNbTetrahedrons();
	float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
	PxU32 vertIndex = 0;
	for (PxU32 v = 0; v < nbrVerts; v++)
	{
		tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
		tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
		tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
		vertIndex += 3;
	}
	PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbrTets * 4), "tmpIndices");
	const PxU32 totalIndexCount = tetrahedronCount * 4;
	if (has16BitIndices)
	{
		for (PxU32 i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = shortIndices[i];
		}
	}
	else
	{
		for (PxU32 i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = intIndices[i];
		}
	}
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, tets, mesh, tmpIndices, 4 * nbrTets);
	PX_FREE(tmpVerts);
	PX_FREE(tmpIndices);

	OMNI_PVD_WRITE_SCOPE_END
}

void streamTriMesh(const physx::PxTriangleMesh& mesh)
{
	if (samplerInternals->addSharedMeshIfNotSeen(&mesh, OmniPvdSharedMeshEnum::eOmniPvdTriMesh))
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, mesh);
		//this gets done at the bottom now
		const PxU32 triangleCount = mesh.getNbTriangles();
		const PxU32 has16BitIndices = mesh.getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES;
		const void* indexBuffer = mesh.getTriangles();
		const PxVec3* vertexBuffer = mesh.getVertices();
		const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
		const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
		//TODO: not needed to copy this
		const PxU32 nbrVerts = mesh.getNbVertices();
		const PxU32 nbrTris = mesh.getNbTriangles();
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
		PxU32 vertIndex = 0;
		for (PxU32 v = 0; v < nbrVerts; v++)
		{
			tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
			tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
			tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
			vertIndex += 3;
		}
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbrTris * 3), "tmpIndices");
		const PxU32 totalIndexCount = triangleCount * 3;
		if (has16BitIndices)
		{
			for (PxU32 i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = shortIndices[i];
			}
		}
		else
		{
			for (PxU32 i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = intIndices[i];
			}
		}
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, tris, mesh, tmpIndices, 3 * nbrTris);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);

		OMNI_PVD_WRITE_SCOPE_END
	}
}

void streamTriMeshGeometry(const physx::PxTriangleMeshGeometry& g)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, scale, g, g.scale.scale);
	streamTriMesh(*g.triangleMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, triangleMesh, g, g.triangleMesh);

	OMNI_PVD_WRITE_SCOPE_END
}

void OmniPvdPxSampler::streamSceneContacts(physx::NpScene& scene)
{
	if (!isSampling()) return;
	PxsContactManagerOutputIterator outputIter;
	Sc::ContactIterator contactIter;
	scene.getScScene().initContactsIterator(contactIter, outputIter);
	Sc::ContactIterator::Pair* pair;
	PxU32 pairCount = 0;
	PxArray<PxActor*> pairsActors;
	PxArray<PxU32> pairsContactCounts;
	PxArray<PxVec3> pairsContactPoints;
	PxArray<PxVec3> pairsContactNormals;
	PxArray<PxReal> pairsContactSeparations;
	PxArray<PxShape*> pairsContactShapes;
	PxArray<PxU32> pairsContactFacesIndices;
	
	while ((pair = contactIter.getNextPair()) != NULL)
	{
		PxU32 pairContactCount = 0;
		Sc::Contact* contact = NULL;
		bool firstContact = true;
		while ((contact = pair->getNextContact()) != NULL)
		{
			if (firstContact) {
				pairsActors.pushBack(pair->getActor0());
				pairsActors.pushBack(pair->getActor1());
				++pairCount;
				firstContact = false;
			}
			++pairContactCount;
			pairsContactPoints.pushBack(contact->point);
			pairsContactNormals.pushBack(contact->normal);
			pairsContactSeparations.pushBack(contact->separation);
			pairsContactShapes.pushBack(contact->shape0);
			pairsContactShapes.pushBack(contact->shape1);
			pairsContactFacesIndices.pushBack(contact->faceIndex0);
			pairsContactFacesIndices.pushBack(contact->faceIndex1);
		}
		if (pairContactCount) 
		{
			pairsContactCounts.pushBack(pairContactCount);
		}
	}

	if (pairCount == 0) return;

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairCount, scene, pairCount);
	const PxU32 actorCount = pairsActors.size();
	const PxActor** actors = actorCount ? const_cast<const PxActor**>(pairsActors.begin()) : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsActors, scene, actors, actorCount);
	PxU32 nbContactCount = pairsContactCounts.size();
	PxU32* contactCounts = nbContactCount ? pairsContactCounts.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactCounts, scene, contactCounts, nbContactCount);
	PxU32 contactPointFloatCount = pairsContactPoints.size() * 3;
	PxReal* contactPoints = contactPointFloatCount ? &pairsContactPoints[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactPoints, scene, contactPoints, contactPointFloatCount);
	PxU32 contactNormalFloatCount = pairsContactNormals.size() * 3;
	PxReal* contactNormals = contactNormalFloatCount ? &pairsContactNormals[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactNormals, scene, contactNormals, contactNormalFloatCount);
	PxU32 contactSeparationCount = pairsContactSeparations.size();
	PxReal* contactSeparations = contactSeparationCount ? pairsContactSeparations.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactSeparations, scene, contactSeparations, contactSeparationCount);
	PxU32 contactShapeCount = pairsContactShapes.size();
	const PxShape** contactShapes = contactShapeCount ? const_cast<const PxShape**>(pairsContactShapes.begin()) : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactShapes, scene, contactShapes, contactShapeCount);
	PxU32 contactFacesIndexCount = pairsContactFacesIndices.size();
	PxU32* contactFacesIndices = contactFacesIndexCount ? pairsContactFacesIndices.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactFacesIndices, scene, contactFacesIndices, contactFacesIndexCount);

	OMNI_PVD_WRITE_SCOPE_END
}

OmniPvdPxSampler::OmniPvdPxSampler()
{
	samplerInternals = PX_NEW(OmniPvdSamplerInternals)();

	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	samplerInternals->mIsSampling = false;
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

void OmniPvdPxSampler::setOmniPvdInstance(physx::NpOmniPvd* omniPvdInstance)
{
	samplerInternals->mPvdStream.setOmniPvdInstance(omniPvdInstance);
}

void createGeometry(const physx::PxGeometry & pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		streamSphereGeometry((const physx::PxSphereGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		streamCapsuleGeometry((const physx::PxCapsuleGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		streamBoxGeometry((const physx::PxBoxGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		streamTriMeshGeometry((const physx::PxTriangleMeshGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		streamConvexMeshGeometry((const physx::PxConvexMeshGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		streamHeightFieldGeometry((const physx::PxHeightFieldGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		streamPlaneGeometry((const physx::PxPlaneGeometry &)pxGeom);
	}	
	break;
	case physx::PxGeometryType::eCUSTOM:
	{
		streamCustomGeometry((const physx::PxCustomGeometry &)pxGeom);
	}
	break;
	default:
	break;
	}
}

void destroyGeometry(const physx::PxGeometry& pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, static_cast<const PxSphereGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, static_cast<const PxCapsuleGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, static_cast<const PxBoxGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, static_cast<const PxTriangleMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, static_cast<const PxConvexMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, static_cast<const PxHeightFieldGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxPlaneGeometry, static_cast<const PxPlaneGeometry&>(pxGeom));
	}	
	break;
	case physx::PxGeometryType::eCUSTOM:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, static_cast<const PxCustomGeometry&>(pxGeom));
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

void OmniPvdPxSampler::onObjectAdd(const physx::PxBase& object)
{
	if (!isSampling()) return;

	const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());

	switch (object.getConcreteType())
	{
		case physx::PxConcreteType::eHEIGHTFIELD:
		{
			const PxHeightField& hf = static_cast<const PxHeightField&>(object);
			streamHeightField(hf);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, heightFields, physics, hf);
		}
		break;
		case physx::PxConcreteType::eCONVEX_MESH:
		{
			const PxConvexMesh& cm = static_cast<const PxConvexMesh&>(object);
			streamConvexMesh(cm);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, convexMeshes, physics, cm);
		}
		break;
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		{
			const PxTriangleMesh& m = static_cast<const PxTriangleMesh&>(object);
			streamTriMesh(m);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, triangleMeshes, physics, m);
		}
		break;
		case physx::PxConcreteType::eTETRAHEDRON_MESH:
		{
			const PxTetrahedronMesh& tm = static_cast<const PxTetrahedronMesh&>(object);
			streamTetMesh(tm);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, tm);
		}
		break;
		case physx::PxConcreteType::eSOFTBODY_MESH:
		{
			const PxSoftBodyMesh& sm = static_cast<const PxSoftBodyMesh&>(object);
			streamSoBoMesh(sm);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, softBodyMeshes, physics, sm);
		}
		break;
		case physx::PxConcreteType::eBVH:
		{
			const PxBVH& bvh = static_cast<const PxBVH&>(object);
			streamBVH(bvh);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, bvhs, physics, bvh);
		}
		break;
		case physx::PxConcreteType::eSHAPE:
		{
			const PxShape& shape = static_cast<const physx::PxShape&>(object);
			createGeometry(shape.getGeometry());
			streamShape(shape);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, shapes, physics, shape);
		}
		break;
		case physx::PxConcreteType::eMATERIAL:
		{
			const PxMaterial& mat = static_cast<const PxMaterial&>(object);
			streamMaterial(mat);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, materials, physics, mat);
		}
		break;
		case physx::PxConcreteType::eSOFTBODY_MATERIAL:
		{
			const PxFEMSoftBodyMaterial& sbMat = static_cast<const PxFEMSoftBodyMaterial&>(object);
			streamFEMSoBoMaterial(sbMat);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, FEMSoftBodyMaterials, physics, sbMat);
		}
		break;
		case physx::PxConcreteType::eCLOTH_MATERIAL:
		{
			const PxFEMClothMaterial& clothMat = static_cast<const PxFEMClothMaterial&>(object);
			streamFEMClothMaterial(clothMat);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, FEMClothMaterials, physics, clothMat);
		}
		break;
		case physx::PxConcreteType::ePBD_MATERIAL:
		{
			const PxPBDMaterial& pbdhMat = static_cast<const PxPBDMaterial&>(object);
			streamPBDMaterial(pbdhMat);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, PBDMaterials, physics, pbdhMat);
		}
		break;
		case physx::PxConcreteType::eFLIP_MATERIAL:
		{
			const PxFLIPMaterial& flipMat = static_cast<const PxFLIPMaterial&>(object);
			streamFLIPMaterial(flipMat);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, FLIPMaterials, physics, flipMat);
		}
		break;
		case physx::PxConcreteType::eMPM_MATERIAL:
		{
			const PxMPMMaterial& mpmMat = static_cast<const PxMPMMaterial&>(object);
			streamMPMMaterial(mpmMat);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, MPMMaterials, physics, mpmMat);
		}
		break;
		case physx::PxConcreteType::eAGGREGATE:
		{
			const PxAggregate& agg = static_cast<const PxAggregate&>(object);
			streamAggregate(agg);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, aggregates, physics, agg);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		{
			const PxArticulationReducedCoordinate& art = static_cast<const PxArticulationReducedCoordinate&>(object);
			streamArticulation(art);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, articulations, physics, art);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_LINK:
		{
			const PxArticulationLink& artLink = static_cast<const PxArticulationLink&>(object);
			streamArticulationLink(artLink);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, links, artLink.getArticulation(), artLink);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		{
			const PxArticulationJointReducedCoordinate& artJoint = static_cast<const PxArticulationJointReducedCoordinate&>(object);
			streamArticulationJoint(artJoint);
			break;
		}
		case physx::PxConcreteType::eRIGID_DYNAMIC:
		{
			const PxRigidDynamic& rd = static_cast<const PxRigidDynamic&>(object);
			streamRigidDynamic(rd);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidDynamics, physics, rd);
		}
		break;
		case physx::PxConcreteType::eRIGID_STATIC:
		{
			const PxRigidStatic& rs = static_cast<const PxRigidStatic&>(object);
			streamRigidStatic(rs);
			OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidStatics, physics, rs);
		}
		break;
	}
}

void OmniPvdPxSampler::onObjectRemove(const physx::PxBase& object)
{
	if (!isSampling()) return;

	const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());

	switch (object.getConcreteType())
	{
		case physx::PxConcreteType::eHEIGHTFIELD:
		{
			const PxHeightField& hf = static_cast<const PxHeightField&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, heightFields, physics, hf);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxHeightField, hf);
		}
		break;
		case physx::PxConcreteType::eCONVEX_MESH:
		{
			const PxConvexMesh& cm = static_cast<const PxConvexMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, convexMeshes, physics, cm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, cm);
		}
		break;
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		{
			const PxTriangleMesh& m = static_cast<const PxTriangleMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, triangleMeshes, physics, m);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, m);
		}
		break;
		case physx::PxConcreteType::eTETRAHEDRON_MESH:
		{
			const PxTetrahedronMesh& tm = static_cast<const PxTetrahedronMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, tm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, tm);
		}
		break;
		case physx::PxConcreteType::eSOFTBODY_MESH:
		{
			const PxSoftBodyMesh& sm = static_cast<const PxSoftBodyMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, softBodyMeshes, physics, sm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxSoftBodyMesh, sm);
		}
		break;
		case physx::PxConcreteType::eBVH:
		{
			const PxBVH& bvh = static_cast<const PxBVH&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, bvhs, physics, bvh);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxBVH, bvh);
		}
		break;
		case physx::PxConcreteType::eSHAPE:
		{
			const PxShape& shape = static_cast<const physx::PxShape&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, shapes, physics, shape);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxShape, shape);
			destroyGeometry(shape.getGeometry());
		}
		break;
		case physx::PxConcreteType::eMATERIAL:
		{
			const PxMaterial& mat = static_cast<const PxMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, materials, physics, mat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, mat);
		}
		break;
		case physx::PxConcreteType::eSOFTBODY_MATERIAL:
		{
			const PxFEMSoftBodyMaterial& sbMat = static_cast<const PxFEMSoftBodyMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, FEMSoftBodyMaterials, physics, sbMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxFEMSoftBodyMaterial, sbMat);
		}
		break;
		case physx::PxConcreteType::eCLOTH_MATERIAL:
		{
			const PxFEMClothMaterial& clothMat = static_cast<const PxFEMClothMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, FEMClothMaterials, physics, clothMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxFEMClothMaterial, clothMat);
		}
		break;
		case physx::PxConcreteType::ePBD_MATERIAL:
		{
			const PxPBDMaterial& pbdhMat = static_cast<const PxPBDMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, PBDMaterials, physics, pbdhMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, pbdhMat);
		}
		break;
		case physx::PxConcreteType::eFLIP_MATERIAL:
		{
			const PxFLIPMaterial& flipMat = static_cast<const PxFLIPMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, FLIPMaterials, physics, flipMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxFLIPMaterial, flipMat);
		}
		break;
		case physx::PxConcreteType::eMPM_MATERIAL:
		{
			const PxMPMMaterial& mpmMat = static_cast<const PxMPMMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, MPMMaterials, physics, mpmMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxMPMMaterial, mpmMat);
		}
		break;
		case physx::PxConcreteType::eAGGREGATE:
		{
			const PxAggregate& agg = static_cast<const PxAggregate&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, aggregates, physics, agg);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxAggregate, agg);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		{
			const PxArticulationReducedCoordinate& art = static_cast<const PxArticulationReducedCoordinate&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, articulations, physics, art);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, art);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_LINK:
		{
			const PxArticulationLink& artLink = static_cast<const PxArticulationLink&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, artLink);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		{
			const PxArticulationJointReducedCoordinate& artJoint = static_cast<const PxArticulationJointReducedCoordinate&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, artJoint);
		}
		break;
		case physx::PxConcreteType::eRIGID_DYNAMIC:
		{
			const PxRigidDynamic& rd = static_cast<const PxRigidDynamic&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidDynamics, physics, rd);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, rd);
		}
		break;
		case physx::PxConcreteType::eRIGID_STATIC:
		{
			const PxRigidStatic& rs = static_cast<const PxRigidStatic&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidStatics, physics, rs);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, rs);
		}
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

///////////////////////////////////////////////////////////////////////////////

OmniPvdPxSampler* OmniPvdPxSampler::getInstance()
{
	PX_ASSERT(&physx::NpPhysics::getInstance() != NULL);
	return &physx::NpPhysics::getInstance() ? physx::NpPhysics::getInstance().mOmniPvdSampler : NULL;
}


namespace physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData()
{
	if (samplerInternals)
	{
		return &samplerInternals->mPvdStream.mRegistrationData;
	}
	else
	{
		return NULL;
	}
}

physx::NpOmniPvd* NpOmniPvdGetInstance()
{
	if (samplerInternals)
	{
		return samplerInternals->mPvdStream.mOmniPvdInstance;
	}
	else
	{
		return NULL;
	}
}

}

#endif

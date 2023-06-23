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

// ****************************************************************************
// This snippet shows how to implement custom geometries generateContacts
// callback, using PhysX Immediate Mode contacts generation.
// ****************************************************************************

#include <ctype.h>
#include <vector>
#include "PxPhysicsAPI.h"
#include "PxImmediateMode.h"
#include "geomutils/PxContactBuffer.h"

// temporary disable this snippet, cannot work without rendering we cannot include GL directly
#ifdef RENDER_SNIPPET

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetrender/SnippetRender.h"

using namespace physx;

/*
	10x10 grid of boxes with even boxes removed.
*/
struct CheckerBoard : PxCustomGeometry::Callbacks
{
	int boardSize;
	float boxExtent;

	DECLARE_CUSTOM_GEOMETRY_TYPE

	CheckerBoard()
		:
		boardSize(10),
		boxExtent(10.0f)
	{}

	struct ContactRecorder : immediate::PxContactRecorder
	{
		PxContactBuffer* contactBuffer;
		ContactRecorder(PxContactBuffer& _contactBuffer) : contactBuffer(&_contactBuffer) {}
		virtual bool recordContacts(const PxContactPoint* contactPoints, PxU32 nbContacts, PxU32 /*index*/)
		{
			for (PxU32 i = 0; i < nbContacts; ++i)
				if (!contactBuffer->contact(contactPoints[i]))
					return false;
			return true;
		}
	};
	struct ContactCacheAllocator : PxCacheAllocator
	{
		PxU8 buffer[1024];
		ContactCacheAllocator() { memset(buffer, 0, sizeof(buffer)); }
		virtual PxU8* allocateCacheData(const PxU32 /*byteSize*/) { return (PxU8*)(size_t(buffer + 0xf) & ~0xf); }
	};

	PxBounds3 getBoardLocalBounds() const
	{
		return PxBounds3(-PxVec3(boardSize * boxExtent * 0.5f, boxExtent * 0.5f, boardSize * boxExtent * 0.5f),
						  PxVec3(boardSize * boxExtent * 0.5f, boxExtent * 0.5f, boardSize * boxExtent * 0.5f));
	}

	virtual PxBounds3 getLocalBounds(const PxGeometry&) const
	{
		return getBoardLocalBounds();
	}
	virtual bool generateContacts(const PxGeometry&, const PxGeometry& geom1, const PxTransform& pose0, const PxTransform& pose1,
		const PxReal contactDistance, const PxReal meshContactMargin, const PxReal toleranceLength,
		PxContactBuffer& contactBuffer) const
	{
		PxBoxGeometry boxGeom(PxVec3(boxExtent * 0.5f));
		PxGeometry* pGeom0 = &boxGeom;

		const PxGeometry* pGeom1 = &geom1;
		PxTransform pose1in0 = pose0.transformInv(pose1);
		PxBounds3 bounds1; PxGeometryQuery::computeGeomBounds(bounds1, geom1, pose1in0, contactDistance);

		ContactRecorder contactRecorder(contactBuffer);
		PxCache contactCache;
		ContactCacheAllocator contactCacheAllocator;

		PxBounds3 bounds0 = getBoardLocalBounds();
		PxVec3 s = bounds1.minimum + bounds0.getExtents();
		PxVec3 e = bounds1.maximum + bounds0.getExtents();
		int sx = int(PxFloor(s.x / boxExtent));
		int sy = int(PxFloor(s.y / boxExtent));
		int sz = int(PxFloor(s.z / boxExtent));
		int ex = int(PxFloor(e.x / boxExtent));
		int ey = int(PxFloor(e.y / boxExtent));
		int ez = int(PxFloor(e.z / boxExtent));
		for (int x = sx; x <= ex; ++x)
			for (int y = sy; y <= ey; ++y)
				for (int z = sz; z <= ez; ++z)
					if (x >= 0 && x < boardSize &&
						y >= 0 && y < boardSize &&
						z >= 0 && z < boardSize &&
						(x + z) & 1 &&
						y == 0)
					{
						PxVec3 boxPos = PxVec3((x + 0.5f) * boxExtent, (y + 0.5f) * boxExtent, (z + 0.5f) * boxExtent) - bounds0.getExtents();
						PxTransform p0 = pose0.transform(PxTransform(boxPos));
						immediate::PxGenerateContacts(&pGeom0, &pGeom1, &p0, &pose1, &contactCache, 1, contactRecorder,
							contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);
					}

		return true;
	}
	virtual PxU32 raycast(const PxVec3&, const PxVec3&, const PxGeometry&, const PxTransform&,
		PxReal, PxHitFlags, PxU32, PxGeomRaycastHit*, PxU32, PxRaycastThreadContext*) const
	{
		return 0;
	}
	virtual bool overlap(const PxGeometry&, const PxTransform&, const PxGeometry&, const PxTransform&, PxOverlapThreadContext*) const
	{
		return false;
	}
	virtual bool sweep(const PxVec3&, const PxReal,
		const PxGeometry&, const PxTransform&, const PxGeometry&, const PxTransform&,
		PxGeomSweepHit&, PxHitFlags, const PxReal, PxSweepThreadContext*) const
	{
		return false;
	}
	virtual void visualize(const PxGeometry&, PxRenderOutput&, const PxTransform&, const PxBounds3&) const {}
	virtual void computeMassProperties(const physx::PxGeometry&, physx::PxMassProperties&) const {}
	virtual bool usePersistentContactManifold(const PxGeometry&, PxReal&) const { return false; }
};

IMPLEMENT_CUSTOM_GEOMETRY_TYPE(CheckerBoard)

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;
static PxFoundation* gFoundation = NULL;
static PxPhysics* gPhysics = NULL;
static PxDefaultCpuDispatcher* gDispatcher = NULL;
static PxScene* gScene = NULL;
static PxMaterial* gMaterial = NULL;
static PxPvd* gPvd = NULL;
static PxRigidStatic* gActor = NULL;

static CheckerBoard gCheckerBoard;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0), PxReal density = 1.0f)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f * 3, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Create checker board actor
	PxRigidStatic* checkerBoardActor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, gCheckerBoard.boxExtent * 0.5f, 0)));
	PxRigidActorExt::createExclusiveShape(*checkerBoardActor, PxCustomGeometry(gCheckerBoard), *gMaterial);
	gScene->addActor(*checkerBoardActor);
	gActor = checkerBoardActor;

	// Ground plane
	PxRigidStatic* planeActor = gPhysics->createRigidStatic(PxTransform(PxQuat(PX_PIDIV2, PxVec3(0, 0, 1))));
	PxRigidActorExt::createExclusiveShape(*planeActor, PxPlaneGeometry(), *gMaterial);
	gScene->addActor(*planeActor);

	createStack(PxTransform(PxVec3(0, 22, 0)), 10, 2.0f);
}

void debugRender()
{
	float boxExtent = gCheckerBoard.boxExtent;
	PxBounds3 boardBounds = gCheckerBoard.getBoardLocalBounds();
	PxGeometryHolder geom;
	geom.storeAny(PxBoxGeometry(PxVec3(boxExtent * 0.5f)));

	for (int x = 0; x < gCheckerBoard.boardSize; ++x)
		for (int y = 0; y < 1; ++y)
			for (int z = 0; z < gCheckerBoard.boardSize; ++z)
				if ((x + z) & 1)
				{
					PxVec3 boxPos = PxVec3((x + 0.5f) * boxExtent, (y + 0.5f) * boxExtent, (z + 0.5f) * boxExtent) - boardBounds.getExtents();
					PxTransform pose = gActor->getGlobalPose().transform(PxTransform(boxPos));
					Snippets::renderGeoms(1, &geom, &pose, false, PxVec3(0.5f));
				}
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetGeometryCollision done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200, 3.0f);	break;
	}
}

int snippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

#else
int snippetMain(int, const char* const*)
{
	return 0;
}

#endif


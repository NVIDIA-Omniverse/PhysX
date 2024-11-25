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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet shows how to implement custom geometries queries
// callbacks, using PhysX geometry queries.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"

// temporary disable this snippet, cannot work without rendering we cannot include GL directly
#ifdef RENDER_SNIPPET

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetrender/SnippetRender.h"

using namespace physx;

void renderRaycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxRaycastHit* hit);
void renderSweepBox(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxVec3& halfExtents, const PxSweepHit* hit);
void renderOverlapBox(const PxVec3& origin, const PxVec3& halfExtents, bool hit);

/*
	Two crossed bars.
*/
struct BarCrosss : PxCustomGeometry::Callbacks
{
	PxVec3 barExtents;

	DECLARE_CUSTOM_GEOMETRY_TYPE

	BarCrosss() : barExtents(27, 9, 3) {}

	virtual PxBounds3 getLocalBounds(const PxGeometry&) const
	{
		return PxBounds3(-PxVec3(barExtents.x * 0.5f, barExtents.y * 0.5f, barExtents.x * 0.5f),
						  PxVec3(barExtents.x * 0.5f, barExtents.y * 0.5f, barExtents.x * 0.5f));
	}
	virtual bool generateContacts(const PxGeometry&, const PxGeometry&, const PxTransform&, const PxTransform&,
		const PxReal, const PxReal, const PxReal,
		PxContactBuffer&) const
	{
		return false;
	}
	virtual PxU32 raycast(const PxVec3& origin, const PxVec3& unitDir, const PxGeometry&, const PxTransform& pose,
		PxReal maxDist, PxHitFlags hitFlags, PxU32, PxGeomRaycastHit* rayHits, PxU32, PxRaycastThreadContext*) const
	{
		PxBoxGeometry barGeom(barExtents * 0.5f);
		PxTransform p0 = pose;
		PxGeomRaycastHit hits[2];
		PxGeometryQuery::raycast(origin, unitDir, barGeom, p0, maxDist, hitFlags, 1, hits + 0);
		p0 = pose.transform(PxTransform(PxQuat(PX_PIDIV2, PxVec3(0, 1, 0))));
		PxGeometryQuery::raycast(origin, unitDir, barGeom, p0, maxDist, hitFlags, 1, hits + 1);
		rayHits[0] = hits[0].distance < hits[1].distance ? hits[0] : hits[1];
		return hits[0].distance < PX_MAX_REAL || hits[1].distance < PX_MAX_REAL ? 1 : 0;
	}
	virtual bool overlap(const PxGeometry&, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1, PxOverlapThreadContext*) const
	{
		PxBoxGeometry barGeom(barExtents * 0.5f);
		PxTransform p0 = pose0;
		if (PxGeometryQuery::overlap(barGeom, p0, geom1, pose1, PxGeometryQueryFlags(0)))
			return true;
		p0 = pose0.transform(PxTransform(PxQuat(PX_PIDIV2, PxVec3(0, 1, 0))));
		if (PxGeometryQuery::overlap(barGeom, p0, geom1, pose1, PxGeometryQueryFlags(0)))
			return true;
		return false;
	}
	virtual bool sweep(const PxVec3& unitDir, const PxReal maxDist,
		const PxGeometry&, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1,
		PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation, PxSweepThreadContext*) const
	{
		PxBoxGeometry barGeom(barExtents * 0.5f);
		PxTransform p0 = pose0;
		PxGeomSweepHit hits[2];
		PxGeometryQuery::sweep(unitDir, maxDist, geom1, pose1, barGeom, p0, hits[0], hitFlags, inflation);
		p0 = pose0.transform(PxTransform(PxQuat(PX_PIDIV2, PxVec3(0, 1, 0))));
		PxGeometryQuery::sweep(unitDir, maxDist, geom1, pose1, barGeom, p0, hits[1], hitFlags, inflation);
		sweepHit = hits[0].distance < hits[1].distance ? hits[0] : hits[1];
		return hits[0].distance < PX_MAX_REAL || hits[1].distance < PX_MAX_REAL;
	}
	virtual void visualize(const PxGeometry&, PxRenderOutput&, const PxTransform&, const PxBounds3&) const {}
	virtual void computeMassProperties(const PxGeometry&, PxMassProperties&) const {}
	virtual bool usePersistentContactManifold(const PxGeometry&, PxReal&) const { return false; }
};

IMPLEMENT_CUSTOM_GEOMETRY_TYPE(BarCrosss)

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;
static PxFoundation* gFoundation = NULL;
static PxPhysics* gPhysics = NULL;
static PxDefaultCpuDispatcher* gDispatcher = NULL;
static PxScene* gScene = NULL;
static PxMaterial* gMaterial = NULL;
static PxPvd* gPvd = NULL;
static PxRigidDynamic* gActor = NULL;

static BarCrosss gBarCrosss;
static PxReal gTime = 0;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0), PxReal density = 1.0f)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
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

	// Create bar cross actor
	PxRigidDynamic* barCrossActor = gPhysics->createRigidDynamic(PxTransform(PxVec3(0, gBarCrosss.barExtents.y * 0.5f, 0)));
	barCrossActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	PxRigidActorExt::createExclusiveShape(*barCrossActor, PxCustomGeometry(gBarCrosss), *gMaterial);
	gScene->addActor(*barCrossActor);
	gActor = barCrossActor;
}

void debugRender()
{
	PxGeometryHolder geom;
	geom.storeAny(PxBoxGeometry(gBarCrosss.barExtents * 0.5f));
	PxTransform pose = gActor->getGlobalPose();
	Snippets::renderGeoms(1, &geom, &pose, false, PxVec3(0.7f));
	pose = pose.transform(PxTransform(PxQuat(PX_PIDIV2, PxVec3(0, 1, 0))));
	Snippets::renderGeoms(1, &geom, &pose, false, PxVec3(0.7f));

	// Raycast
	{
		PxVec3 origin((gBarCrosss.barExtents.x + 10) * 0.5f, 0, 0);
		PxVec3 unitDir(-1, 0, 0);
		float maxDist = gBarCrosss.barExtents.x + 20;
		PxRaycastBuffer buffer;
		gScene->raycast(origin, unitDir, maxDist, buffer);
		renderRaycast(origin, unitDir, maxDist, buffer.hasBlock ? &buffer.block : nullptr);
	}

	// Sweep
	{
		PxVec3 origin(0, 0, (gBarCrosss.barExtents.x + 10) * 0.5f);
		PxVec3 unitDir(0, 0, -1);
		float maxDist = gBarCrosss.barExtents.x + 20;
		PxVec3 halfExtents(1.5f, 0.5f, 1.0f);
		PxSweepBuffer buffer;
		gScene->sweep(PxBoxGeometry(halfExtents), PxTransform(origin), unitDir, maxDist, buffer);
		renderSweepBox(origin, unitDir, maxDist, halfExtents, buffer.hasBlock ? &buffer.block : nullptr);
	}

	// Overlap
	{
		PxVec3 origin((gBarCrosss.barExtents.x) * -0.4f, 0, (gBarCrosss.barExtents.x) * -0.4f);
		PxVec3 halfExtents(gBarCrosss.barExtents.z * 1.5f, gBarCrosss.barExtents.y * 1.1f, gBarCrosss.barExtents.z * 1.5f);
		PxOverlapBuffer buffer;
		gScene->overlap(PxBoxGeometry(halfExtents), PxTransform(origin), buffer, PxQueryFilterData(PxQueryFlag::eANY_HIT | PxQueryFlag::eDYNAMIC));
		renderOverlapBox(origin, halfExtents, buffer.hasAnyHits());
	}
}

void stepPhysics(bool /*interactive*/)
{
	gTime += 1.0f / 60.0f;
	gActor->setKinematicTarget(PxTransform(PxQuat(gTime * 0.3f, PxVec3(0, 1, 0))));

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
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetCustomGeometryQueries done.\n");
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


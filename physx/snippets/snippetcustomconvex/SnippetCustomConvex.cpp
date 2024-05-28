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
// This snippet shows how to use GJK queries to create custom convex geometry.
// ****************************************************************************

#include <ctype.h>
#include <vector>
#include "PxPhysicsAPI.h"
#include "geometry/PxGjkQuery.h"
#include "CustomConvex.h"
#include "extensions/PxCustomGeometryExt.h"

// temporary disable this snippet, cannot work without rendering we cannot include GL directly
#ifdef RENDER_SNIPPET

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetrender/SnippetRender.h"

using namespace physx;

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;
static PxFoundation* gFoundation = NULL;
static PxPhysics* gPhysics = NULL;
static PxDefaultCpuDispatcher* gDispatcher = NULL;
static PxScene* gScene = NULL;
static PxMaterial* gMaterial = NULL;
static PxPvd* gPvd = NULL;
//static std::vector<CustomConvex*> gConvexes;
static std::vector<PxCustomGeometryExt::BaseConvexCallbacks*> gConvexes;
static std::vector<PxRigidActor*> gActors;
struct RenderMesh;
static std::vector<RenderMesh*> gMeshes;

RenderMesh* createRenderCylinder(float radius, float height, float margin);
RenderMesh* createRenderCone(float height, float radius, float margin);
void destroyRenderMesh(RenderMesh* mesh);
void renderMesh(const RenderMesh& mesh, const PxTransform& pose, bool sleeping);
void renderRaycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxRaycastHit* hit);
void renderSweepBox(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxVec3& halfExtents, const PxSweepHit* hit);
void renderOverlapBox(const PxVec3& origin, const PxVec3& halfExtents, bool hit);

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0), PxReal density = 1.0f)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void createCylinderActor(float height, float radius, float margin, const PxTransform& pose)
{
	//CustomCylinder* cylinder = new CustomCylinder(height, radius, margin);
	PxCustomGeometryExt::CylinderCallbacks* cylinder = new PxCustomGeometryExt::CylinderCallbacks(height, radius, 0, margin);
	gConvexes.push_back(cylinder);

	PxRigidDynamic* actor = gPhysics->createRigidDynamic(pose);
	actor->setActorFlag(PxActorFlag::eVISUALIZATION, true);

	PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, PxCustomGeometry(*cylinder), *gMaterial);
	shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
	PxRigidBodyExt::updateMassAndInertia(*actor, 100);
	gScene->addActor(*actor);
	gActors.push_back(actor);

	RenderMesh* mesh = createRenderCylinder(height, radius, margin);
	gMeshes.push_back(mesh);
}

static void createConeActor(float height, float radius, float margin, const PxTransform& pose)
{
	//CustomCone* cone = new CustomCone(height, radius, margin);
	PxCustomGeometryExt::ConeCallbacks* cone = new PxCustomGeometryExt::ConeCallbacks(height, radius, 0, margin);
	gConvexes.push_back(cone);

	PxRigidDynamic* actor = gPhysics->createRigidDynamic(pose);
	actor->setActorFlag(PxActorFlag::eVISUALIZATION, true);

	PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, PxCustomGeometry(*cone), *gMaterial);
	shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
	PxRigidBodyExt::updateMassAndInertia(*actor, 100);
	gScene->addActor(*actor);
	gActors.push_back(actor);

	RenderMesh* mesh = createRenderCone(height, radius, margin);
	gMeshes.push_back(mesh);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Some custom convexes
	float heights[] = { 1.0f, 1.25f, 1.5f, 1.75f };
	float radiuss[] = { 0.3f, 0.35f, 0.4f, 0.45f };
	float margins[] = { 0.0f, 0.05f, 0.1f, 0.15f };
	for (int i = 0; i < 50; ++i)
	{
		float height = heights[rand() % (sizeof(heights) / sizeof(heights[0]))];
		float raduis = radiuss[rand() % (sizeof(radiuss) / sizeof(radiuss[0]))];
		float margin = margins[rand() % (sizeof(margins) / sizeof(margins[0]))];
		float angle = PX_PIDIV2;
		createCylinderActor(height, raduis, margin, (PxTransform(PxVec3(-2.0f, 2.0f + i * 2, 2.0f), PxQuat(angle, PxVec3(0.0f, 0.0f, 1.0f)))));
	}
	for (int i = 0; i < 50; ++i)
	{
		float height = heights[rand() % (sizeof(heights) / sizeof(heights[0]))];
		float raduis = radiuss[rand() % (sizeof(radiuss) / sizeof(radiuss[0]))];
		float margin = margins[rand() % (sizeof(margins) / sizeof(margins[0]))];
		float angle = PX_PIDIV2;
		createConeActor(height, raduis, margin, (PxTransform(PxVec3(2.0f, 2.0f + i * 2, -2.0f), PxQuat(angle, PxVec3(0, 0, 1)))));
	}

	// Ground plane
	PxRigidStatic* planeActor = gPhysics->createRigidStatic(PxTransform(PxQuat(PX_PIDIV2, PxVec3(0.0f, 0.0f, 1.0f))));
	PxRigidActorExt::createExclusiveShape(*planeActor, PxPlaneGeometry(), *gMaterial);
	gScene->addActor(*planeActor);
}

void debugRender()
{
	for (int i = 0; i < int(gConvexes.size()); ++i)
	{
		PxRigidActor* actor = gActors[i];
		RenderMesh* mesh = gMeshes[i];
		renderMesh(*mesh, actor->getGlobalPose(), !actor->is<PxRigidDynamic>() || actor->is<PxRigidDynamic>()->isSleeping());
	}

	int count = 20;
	for (int i = 0; i < count; ++i)
	{
		float x = -count / 2.0f;
		PxVec3 origin(x + i, 0.5f, x);
		PxVec3 unitDir(0, 0, 1);
		float maxDist = (float)count;
		PxRaycastBuffer buffer;
		gScene->raycast(origin, unitDir, maxDist, buffer);
		renderRaycast(origin, unitDir, maxDist, buffer.hasBlock ? &buffer.block : nullptr);
	}
	for (int i = 0; i < count; ++i)
	{
		float x = -count / 2.0f;
		PxVec3 origin(x, 0.5f, x + i);
		PxVec3 unitDir(1, 0, 0);
		float maxDist = (float)count;
		PxVec3 halfExtents(0.2f, 0.1f, 0.4f);
		PxSweepBuffer buffer;
		gScene->sweep(PxBoxGeometry(halfExtents), PxTransform(origin), unitDir, maxDist, buffer);
		renderSweepBox(origin, unitDir, maxDist, halfExtents, buffer.hasBlock ? &buffer.block : nullptr);
	}
	for (int i = 0; i < count; ++i)
	{
		float x = -count / 2.0f;
		for (int j = 0; j < count; ++j)
		{
			PxVec3 origin(x + i, 0.0f, x + j);
			PxVec3 halfExtents(0.4f, 0.1f, 0.4f);
			PxOverlapBuffer buffer;
			gScene->overlap(PxBoxGeometry(halfExtents), PxTransform(origin), buffer, PxQueryFilterData(PxQueryFlag::eANY_HIT | PxQueryFlag::eDYNAMIC));
			renderOverlapBox(origin, halfExtents, buffer.hasAnyHits());
		}
	}
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
	while (!gConvexes.empty())
	{
		delete gConvexes.back();
		gConvexes.pop_back();
	}

	while (!gMeshes.empty())
	{
		destroyRenderMesh(gMeshes.back());
		gMeshes.pop_back();
	}

	while (!gActors.empty())
	{
		PX_RELEASE(gActors.back());
		gActors.pop_back();
	}

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

	printf("SnippetCustomConvex done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case ' ':	createDynamic(camera, PxSphereGeometry(1.0f), camera.rotate(PxVec3(0, 0, -1)) * 100, 3.0f);	break;
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


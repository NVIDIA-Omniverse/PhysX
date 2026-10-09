// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ****************************************************************************
// This snippet compares the two CPU contact generation methods for triangle
// meshes with SDFs: the per-triangle optimization (default) and the cached,
// importance-sampled SDF-SDF search enabled by
// PxSceneFlag::eENABLE_SDF_SDF_CONTACTS.
//
// Every scene is simulated twice with the same settings, once per method, and
// a line per run reports how the bodies came to rest, how deep the contacts
// got, how many contact points the solver saw and how long a step took. The
// scenes are a cube stack, the box pyramid of the paper's Table 3 and a tower
// of gears, all built from procedural meshes, on a ground with an SDF and,
// for the last scene, on a ground without one (pairs with a single SDF).
// ****************************************************************************

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "PxPhysicsAPI.h"
#include "../snippetsdf/MeshGenerator.h"
#include "../snippetutils/SnippetUtils.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation	= NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher	= NULL;
static PxMaterial*				gMaterial	= NULL;

static const PxReal	gTimeStep = 1.0f / 60.0f;
static const PxU32	gNbSteps = 600;

// Pair flags so that contact counts can be read back
static PxFilterFlags filterShader(PxFilterObjectAttributes, PxFilterData, PxFilterObjectAttributes, PxFilterData, PxPairFlags& pairFlags, const void*, PxU32)
{
	pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}

struct ContactCounter : PxSimulationEventCallback
{
	PxU32	points;
	PxReal	minSeparation;
	ContactCounter() : points(0), minSeparation(0.0f) {}

	void onContact(const PxContactPairHeader&, const PxContactPair* pairs, PxU32 nbPairs)
	{
		PxContactPairPoint pts[64];
		for (PxU32 i = 0; i < nbPairs; ++i)
		{
			points += pairs[i].contactCount;
			const PxU32 n = pairs[i].extractContacts(pts, 64);
			for (PxU32 j = 0; j < n; ++j)
				minSeparation = PxMin(minSeparation, pts[j].separation);
		}
	}
	void onConstraintBreak(PxConstraintInfo*, PxU32) {}
	void onWake(PxActor**, PxU32) {}
	void onSleep(PxActor**, PxU32) {}
	void onTrigger(PxTriggerPair*, PxU32) {}
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
};

// ---------------------------------------------------------------------------
// Procedural meshes

// A spur gear: a thick ring with trapezoidal teeth, axis along Y, outer radius 1
static void createGear(PxArray<PxVec3>& verts, PxArray<PxU32>& indices, PxU32 nbTeeth, PxReal rootRadius, PxReal tipRadius, PxReal halfThickness)
{
	verts.clear();
	indices.clear();
	// 2D profile: four points per tooth (root, tip, tip, root)
	PxArray<PxVec3> profile;
	for (PxU32 t = 0; t < nbTeeth; ++t)
	{
		const PxReal a0 = (PxReal(t) + 0.0f) / PxReal(nbTeeth) * PxTwoPi;
		const PxReal a1 = (PxReal(t) + 0.2f) / PxReal(nbTeeth) * PxTwoPi;
		const PxReal a2 = (PxReal(t) + 0.5f) / PxReal(nbTeeth) * PxTwoPi;
		const PxReal a3 = (PxReal(t) + 0.7f) / PxReal(nbTeeth) * PxTwoPi;
		profile.pushBack(PxVec3(rootRadius * PxCos(a0), 0.0f, rootRadius * PxSin(a0)));
		profile.pushBack(PxVec3(tipRadius * PxCos(a1), 0.0f, tipRadius * PxSin(a1)));
		profile.pushBack(PxVec3(tipRadius * PxCos(a2), 0.0f, tipRadius * PxSin(a2)));
		profile.pushBack(PxVec3(rootRadius * PxCos(a3), 0.0f, rootRadius * PxSin(a3)));
	}
	const PxU32 n = profile.size();
	// bottom ring, top ring, bottom center, top center
	for (PxU32 i = 0; i < n; ++i) verts.pushBack(profile[i] + PxVec3(0.0f, -halfThickness, 0.0f));
	for (PxU32 i = 0; i < n; ++i) verts.pushBack(profile[i] + PxVec3(0.0f, halfThickness, 0.0f));
	const PxU32 bc = verts.size(); verts.pushBack(PxVec3(0.0f, -halfThickness, 0.0f));
	const PxU32 tc = verts.size(); verts.pushBack(PxVec3(0.0f, halfThickness, 0.0f));
	for (PxU32 i = 0; i < n; ++i)
	{
		const PxU32 j = (i + 1) % n;
		// side quad, outward
		indices.pushBack(i); indices.pushBack(j); indices.pushBack(n + j);
		indices.pushBack(i); indices.pushBack(n + j); indices.pushBack(n + i);
		// caps
		indices.pushBack(bc); indices.pushBack(j); indices.pushBack(i);
		indices.pushBack(tc); indices.pushBack(n + i); indices.pushBack(n + j);
	}
}

static PxTriangleMesh* cookMesh(const PxArray<PxVec3>& verts, const PxArray<PxU32>& indices, PxReal sdfSpacing)
{
	PxTolerancesScale scale;
	PxCookingParams params(scale);
	params.meshWeldTolerance = 0.0001f;
	params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);

	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = verts.size();
	meshDesc.points.data = verts.begin();
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = indices.size() / 3;
	meshDesc.triangles.data = indices.begin();
	meshDesc.triangles.stride = 3 * sizeof(PxU32);

	PxSDFDesc sdfDesc;
	if (sdfSpacing > 0.0f)
	{
		sdfDesc.spacing = sdfSpacing;
		sdfDesc.subgridSize = 0;	// dense grid
		sdfDesc.numThreadsForSdfConstruction = 4;
		meshDesc.sdfDesc = &sdfDesc;
	}
	return PxCreateTriangleMesh(params, meshDesc, gPhysics->getPhysicsInsertionCallback());
}

static PxTriangleMesh* cubeMesh(PxReal size, PxReal sdfSpacing)
{
	PxArray<PxVec3> verts;
	PxArray<PxU32> indices;
	meshgenerator::createCube(verts, indices, PxVec3(0.0f), size);
	return cookMesh(verts, indices, sdfSpacing);
}

// ---------------------------------------------------------------------------
// Scenes

struct Scene
{
	PxScene*						scene;
	PxArray<PxRigidDynamic*>		bodies;
	PxArray<PxTriangleMesh*>		meshes;
	PxRigidDynamic*					top;
	PxVec3							topStart;
	PxReal							expectedTopY;
	ContactCounter					counter;
};

static void addActor(Scene& s, PxTriangleMesh* mesh, const PxTransform& pose, PxReal scale, bool dynamic)
{
	PxTriangleMeshGeometry geom(mesh, PxMeshScale(scale));
	if (dynamic)
	{
		PxRigidDynamic* dyn = gPhysics->createRigidDynamic(pose);
		PxShape* shape = PxRigidActorExt::createExclusiveShape(*dyn, geom, *gMaterial);
		shape->setContactOffset(0.02f);
		shape->setRestOffset(0.0f);
		PxRigidBodyExt::updateMassAndInertia(*dyn, 1000.0f);
		dyn->setLinearDamping(0.2f);
		dyn->setAngularDamping(0.1f);
		s.scene->addActor(*dyn);
		s.bodies.pushBack(dyn);
		s.top = dyn;
		s.topStart = pose.p;
	}
	else
	{
		PxRigidStatic* st = gPhysics->createRigidStatic(pose);
		PxShape* shape = PxRigidActorExt::createExclusiveShape(*st, geom, *gMaterial);
		shape->setContactOffset(0.02f);
		s.scene->addActor(*st);
	}
}

// A static ground cube with its top face at y = 0. A box SDF is exact away from its edges, so a
// coarse grid is enough; sdfSpacing <= 0 makes a plain mesh without SDF.
static void addGround(Scene& s, PxReal sdfSpacing)
{
	PxTriangleMesh* ground = cubeMesh(20.0f, sdfSpacing);
	s.meshes.pushBack(ground);
	addActor(s, ground, PxTransform(PxVec3(0.0f, -10.0f, 0.0f)), 1.0f, false);
}

static void sceneStack(Scene& s)
{
	addGround(s, 20.0f / 96.0f);
	PxTriangleMesh* box = cubeMesh(1.0f, 1.0f / 32.0f);
	s.meshes.pushBack(box);
	for (PxU32 i = 0; i < 6; ++i)
		addActor(s, box, PxTransform(PxVec3(0.05f * PxReal(i), 0.51f + PxReal(i) * 1.01f, 0.0f), PxQuat(0.02f * PxReal(i + 1), PxVec3(0.0f, 1.0f, 0.0f))), 1.0f, true);
	s.expectedTopY = 5.5f;
}

static void scenePyramid(Scene& s)
{
	// the paper's Box-stacking scene: 7 levels, 28 boxes, started a small gap apart
	addGround(s, 20.0f / 96.0f);
	PxTriangleMesh* box = cubeMesh(1.0f, 1.0f / 32.0f);
	s.meshes.pushBack(box);
	const PxU32 levels = 7;
	const PxReal gap = 0.01f;
	for (PxU32 level = 0; level < levels; ++level)
		for (PxU32 i = 0; i < levels - level; ++i)
			addActor(s, box, PxTransform(PxVec3((PxReal(i) - 0.5f * PxReal(levels - level - 1)) * 1.05f, 0.5f + gap + PxReal(level) * (1.0f + gap), 0.0f)), 1.0f, true);
	s.expectedTopY = 0.5f + gap + PxReal(levels - 1) * (1.0f + gap);
}

static void sceneGears(Scene& s)
{
	// a tower of 10 gears with random yaw, as in the paper's Gear-stacking scene
	addGround(s, 20.0f / 96.0f);
	PxArray<PxVec3> verts;
	PxArray<PxU32> indices;
	createGear(verts, indices, 20, 0.85f, 1.0f, 0.18f);
	PxTriangleMesh* gear = cookMesh(verts, indices, 2.0f / 64.0f);
	s.meshes.pushBack(gear);
	SnippetUtils::BasicRandom rng(11);
	for (PxU32 i = 0; i < 10; ++i)
	{
		const PxReal yaw = rng.rand(0.0f, PxTwoPi);
		addActor(s, gear, PxTransform(PxVec3(rng.rand(-0.05f, 0.05f), 0.2f + PxReal(i) * 0.45f, rng.rand(-0.05f, 0.05f)), PxQuat(yaw, PxVec3(0.0f, 1.0f, 0.0f))), 0.75f, true);
	}
	s.expectedTopY = 0.27f * 9.5f;
}

static void sceneGearsPlainGround(Scene& s)
{
	// the gear tower on a ground mesh without an SDF: gear-ground pairs have one SDF, gear-gear
	// pairs two, so both modes of the method run in one scene
	addGround(s, 0.0f);
	PxArray<PxVec3> verts;
	PxArray<PxU32> indices;
	createGear(verts, indices, 20, 0.85f, 1.0f, 0.18f);
	PxTriangleMesh* gear = cookMesh(verts, indices, 2.0f / 64.0f);
	s.meshes.pushBack(gear);
	SnippetUtils::BasicRandom rng(11);
	for (PxU32 i = 0; i < 10; ++i)
	{
		const PxReal yaw = rng.rand(0.0f, PxTwoPi);
		addActor(s, gear, PxTransform(PxVec3(rng.rand(-0.05f, 0.05f), 0.2f + PxReal(i) * 0.45f, rng.rand(-0.05f, 0.05f)), PxQuat(yaw, PxVec3(0.0f, 1.0f, 0.0f))), 0.75f, true);
	}
	s.expectedTopY = 0.27f * 9.5f;
}

typedef void (*SceneBuilder)(Scene&);

static void runScene(const char* title, SceneBuilder build, bool sdfSdfContacts)
{
	Scene s;
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = filterShader;
	sceneDesc.simulationEventCallback = &s.counter;
	sceneDesc.solverType = PxSolverType::eTGS;
	if (sdfSdfContacts)
		sceneDesc.flags |= PxSceneFlag::eENABLE_SDF_SDF_CONTACTS;
	s.scene = gPhysics->createScene(sceneDesc);
	s.top = NULL;
	s.expectedTopY = -1.0f;
	build(s);

	PxReal maxDrift = 0.0f;
	const PxU64 t0 = SnippetUtils::getCurrentTimeCounterValue();
	for (PxU32 step = 0; step < gNbSteps; ++step)
	{
		s.scene->simulate(gTimeStep);
		s.scene->fetchResults(true);
		const PxVec3 p = s.top->getGlobalPose().p;
		maxDrift = PxMax(maxDrift, PxVec3(p.x - s.topStart.x, 0.0f, p.z - s.topStart.z).magnitude());
	}
	const PxU64 t1 = SnippetUtils::getCurrentTimeCounterValue();

	PxU32 asleep = 0;
	PxReal maxSpeed = 0.0f, minY = PX_MAX_F32;
	for (PxU32 i = 0; i < s.bodies.size(); ++i)
	{
		asleep += s.bodies[i]->isSleeping() ? 1 : 0;
		maxSpeed = PxMax(maxSpeed, s.bodies[i]->getLinearVelocity().magnitude());
		minY = PxMin(minY, s.bodies[i]->getGlobalPose().p.y);
	}
	printf("  %-11s %-9s top y %7.3f", title, sdfSdfContacts ? "sdf-sdf" : "triangles", double(s.top->getGlobalPose().p.y));
	if (s.expectedTopY > 0.0f)
		printf(" (~%5.2f)", double(s.expectedTopY));
	else
		printf("        ");
	printf("  drift %6.3f  min y %6.3f  max speed %7.4f  asleep %2u/%-2u  min sep %7.4f  contacts/step %6.1f  %6.3f ms/step\n",
		double(maxDrift), double(minY), double(maxSpeed), asleep, s.bodies.size(), double(s.counter.minSeparation),
		double(s.counter.points) / gNbSteps, double(SnippetUtils::getElapsedTimeInMilliseconds(t1 - t0)) / gNbSteps);

	s.scene->release();
	for (PxU32 i = 0; i < s.meshes.size(); ++i)
		s.meshes[i]->release();
}

int snippetMain(int, const char*const*)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL);
	PxInitExtensions(*gPhysics, NULL);
	gDispatcher = PxDefaultCpuDispatcherCreate(0);
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.1f);

	printf("SDF contact generation: per-triangle optimization vs cached SDF-SDF search, %u steps at %g Hz\n", gNbSteps, 1.0 / double(gTimeStep));
	struct Entry { const char* title; SceneBuilder build; };
	const Entry entries[] = {
		{ "stack", sceneStack },
		{ "pyramid", scenePyramid },
		{ "gears", sceneGears },
		{ "gears/plain", sceneGearsPlainGround },
	};
	for (PxU32 e = 0; e < sizeof(entries) / sizeof(entries[0]); ++e)
	{
		runScene(entries[e].title, entries[e].build, true);
		runScene(entries[e].title, entries[e].build, false);
	}

	PxCloseExtensions();
	gPhysics->release();
	gFoundation->release();
	printf("SnippetSDFContacts done.\n");
	return 0;
}

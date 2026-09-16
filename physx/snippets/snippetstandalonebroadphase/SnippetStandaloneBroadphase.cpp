// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ****************************************************************************
// This snippet illustrates how to use a standalone broadphase.
// It creates a small custom scene (no PxScene) and creates a broadphase for the
// scene objects. These objects are then updated each frame and rendered in red
// when they touch another object, or green if they don't. Use the P and O keys
// to pause and step the simulation one frame, to visually check the results.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "PxImmediateMode.h"
#include "foundation/PxArray.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetCamera.h"
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;
using namespace immediate;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static const PxU32				gNbObjects = 100;
static float					gTime = 0.0f;
static bool						gPause = false;
static bool						gOneFrame = false;

static PxVec3 computeObjectPosition(PxU32 i)
{
	const float coeff = float(i)*0.2f;
	return PxVec3(	sinf(gTime*coeff*0.567f)*cosf(gTime+coeff)*3.0f,
					cosf(gTime*coeff*0.0917f)*cosf(gTime*1.17f+coeff)*2.0f,
					sinf(gTime*coeff*0.533f)*cosf(gTime*0.33f+coeff)*3.0f);
}

namespace
{
	class CustomScene
	{
		public:
			CustomScene();
			~CustomScene();

			void	release();
			void	addGeom(const PxGeometry& geom, const PxTransform& pose);
			void	render();
			void	updateObjects();
			void	createBroadphase();
			void	runBroadphase();

			struct Object
			{
				PxGeometryHolder	mGeom;
				PxTransform			mPose;
				PxU32				mNbCollisions;
			};

			PxArray<Object>	mObjects;
			PxBroadPhase*	mBroadphase;
			PxAABBManager*	mAABBManager;
	};

CustomScene::CustomScene() : mBroadphase(NULL), mAABBManager(NULL)
{
}

CustomScene::~CustomScene()
{
}

void CustomScene::release()
{
	PX_RELEASE(mAABBManager);
	PX_RELEASE(mBroadphase);
	mObjects.reset();
	PX_DELETE_THIS;
}

void CustomScene::addGeom(const PxGeometry& geom, const PxTransform& pose)
{
	Object obj;
	obj.mGeom.storeAny(geom);
	obj.mPose = pose;
	mObjects.pushBack(obj);
}

void CustomScene::createBroadphase()
{
	PxBroadPhaseDesc bpDesc(PxBroadPhaseType::eABP);
	mBroadphase = PxCreateBroadPhase(bpDesc);
	mAABBManager = PxCreateAABBManager(*mBroadphase);

	const PxU32 nbObjects = mObjects.size();
	for(PxU32 i=0;i<nbObjects;i++)
	{
		Object& obj = mObjects[i];
		obj.mPose.p = computeObjectPosition(i);
		obj.mNbCollisions = 0;

		PxBounds3 bounds;
		PxGeometryQuery::computeGeomBounds(bounds, obj.mGeom.any(), obj.mPose);
		mAABBManager->addObject(i, bounds, PxGetBroadPhaseDynamicFilterGroup(i));
	}

	runBroadphase();
}

void CustomScene::runBroadphase()
{
	PxBroadPhaseResults results;
	mAABBManager->updateAndFetchResults(results);

	for(PxU32 i=0;i<results.mNbCreatedPairs;i++)
	{
		const PxU32 id0 = results.mCreatedPairs[i].mID0;
		const PxU32 id1 = results.mCreatedPairs[i].mID1;
		mObjects[id0].mNbCollisions++;
		mObjects[id1].mNbCollisions++;
	}

	for(PxU32 i=0;i<results.mNbDeletedPairs;i++)
	{
		const PxU32 id0 = results.mDeletedPairs[i].mID0;
		const PxU32 id1 = results.mDeletedPairs[i].mID1;
		PX_ASSERT(mObjects[id0].mNbCollisions);
		PX_ASSERT(mObjects[id1].mNbCollisions);
		mObjects[id0].mNbCollisions--;
		mObjects[id1].mNbCollisions--;
	}
}

void CustomScene::updateObjects()
{
	if(gPause && !gOneFrame)
		return;
	gOneFrame = false;

	gTime += 0.001f;

	const PxU32 nbObjects = mObjects.size();
	for(PxU32 i=0;i<nbObjects;i++)
	{
		Object& obj = mObjects[i];
		obj.mPose.p = computeObjectPosition(i);

		PxBounds3 newBounds;
		PxGeometryQuery::computeGeomBounds(newBounds, obj.mGeom.any(), obj.mPose);

		mAABBManager->updateObject(i, &newBounds);
	}

	runBroadphase();
}

void CustomScene::render()
{
	updateObjects();

#ifdef RENDER_SNIPPET
	const PxU32 nbObjects = mObjects.size();
	for(PxU32 i=0;i<nbObjects;i++)
	{
		const Object& obj = mObjects[i];

		const PxVec3 color = obj.mNbCollisions ? PxVec3(1.0f, 0.0f, 0.0f) : PxVec3(0.0f, 1.0f, 0.0f);

		Snippets::renderGeoms(1, &obj.mGeom, &obj.mPose, false, color);
	}
#endif
}


}

static void initScene()
{
}

static void releaseScene()
{
}

static CustomScene* gScene = NULL;

void renderScene()
{
	if(gScene)
		gScene->render();
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gScene = new CustomScene;
	for(PxU32 i=0;i<gNbObjects;i++)
		gScene->addGeom(PxBoxGeometry(PxVec3(0.1f)), PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));

	gScene->createBroadphase();

	initScene();
}

void stepPhysics(bool /*interactive*/)
{
}

void cleanupPhysics(bool /*interactive*/)
{
	releaseScene();

	PX_RELEASE(gScene);
	PX_RELEASE(gFoundation);
	
	printf("SnippetStandaloneBroadphase done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	if(key=='p' || key=='P')
		gPause = !gPause;

	if(key=='o' || key=='O')
	{
		gPause = true;
		gOneFrame = true;
	}
}

int snippetMain(int, const char*const*)
{
	printf("Standalone broadphase snippet.\n");

#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

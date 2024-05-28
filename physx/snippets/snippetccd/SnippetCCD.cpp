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
// This snippet illustrates how to use different types of CCD methods, 
// including linear, raycast and speculative CCD.
//
// The scene has two parts:
// - a simple box stack and a fast moving sphere. Without (linear) CCD the
// sphere goes through the box stack. With CCD the sphere hits the stack and
// the behavior is more convincing.
// - a simple rotating plank (fixed in space except for one rotation axis)
// that collides with a falling box. Wihout (angular) CCD the collision is missed.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "extensions/PxRaycastCCD.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd        = NULL;
static RaycastCCDManager*		gRaycastCCD	= NULL;
static PxReal					stackZ = 10.0f;

enum CCDAlgorithm
{
	// Uses linear CCD algorithm 
	LINEAR_CCD,

	// Uses speculative/angular CCD algorithm
	SPECULATIVE_CCD,

	// Uses linear & angular CCD at the same time
	FULL_CCD,

	// Uses raycast CCD algorithm 
	RAYCAST_CCD,

	// Switch to NO_CCD to see the sphere go through the box stack without CCD.
	NO_CCD,

	// Number of CCD algorithms used in this snippet
	CCD_COUNT
};

static bool			gPause			= false;
static bool			gOneFrame		= false;
static const PxU32	gScenarioCount	= CCD_COUNT;
static PxU32		gScenario		= 0;

static PX_FORCE_INLINE CCDAlgorithm getCCDAlgorithm()
{
	return CCDAlgorithm(gScenario);
}

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0), bool enableLinearCCD = false, bool enableSpeculativeCCD = false)
{
	PX_ASSERT(gScene);
	PxRigidDynamic* dynamic = NULL;
	if(gScene)
	{
		dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
		dynamic->setAngularDamping(0.5f);
		dynamic->setLinearVelocity(velocity);
		gScene->addActor(*dynamic);
		if(enableLinearCCD)
			dynamic->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
		if(enableSpeculativeCCD)
			dynamic->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
	}

	return dynamic;
}

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent, bool enableLinearCCD = false, bool enableSpeculativeCCD = false)
{
	PX_ASSERT(gScene);
	if(!gScene)
		return;

	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);	
	PX_ASSERT(shape);
	if(!shape)
		return;

	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			const PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			if(enableLinearCCD)
				body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			if(enableSpeculativeCCD)
				body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);

			gScene->addActor(*body);
		}
	}
	shape->release();
}

static PxFilterFlags ccdFilterShader(
	PxFilterObjectAttributes attributes0,
	PxFilterData filterData0,
	PxFilterObjectAttributes attributes1,
	PxFilterData filterData1,
	PxPairFlags& pairFlags,
	const void* constantBlock,
	PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(filterData0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);

	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT | PxPairFlag::eDETECT_CCD_CONTACT;

	return PxFilterFlags();
}

static void registerForRaycastCCD(PxRigidDynamic* actor)
{
	if(actor)
	{
		PxShape* shape = NULL;

		actor->getShapes(&shape, 1);

		// Register each object for which CCD should be enabled. In this snippet we only enable it for the sphere.
		gRaycastCCD->registerRaycastCCDObject(actor, shape);
	}
}

static void initScene()
{
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	bool enableLinearCCD = false;
	bool enableSpeculativeCCD = false;

	const CCDAlgorithm ccd = getCCDAlgorithm();
	if(ccd == LINEAR_CCD)
	{
		enableLinearCCD = true;
		sceneDesc.filterShader = ccdFilterShader;
		sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
		gScene = gPhysics->createScene(sceneDesc);

		printf("- Using linear CCD.\n");
	}
	else if(ccd == SPECULATIVE_CCD)
	{
		enableSpeculativeCCD = true;
		gScene = gPhysics->createScene(sceneDesc);

		printf("- Using speculative/angular CCD.\n");
	}
	else if(ccd == FULL_CCD)
	{
		enableLinearCCD = true;
		enableSpeculativeCCD = true;
		sceneDesc.filterShader = ccdFilterShader;
		sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;

		gScene = gPhysics->createScene(sceneDesc);

		printf("- Using full CCD.\n");
	}
	else if(ccd == RAYCAST_CCD)
	{
		gScene = gPhysics->createScene(sceneDesc);
		gRaycastCCD = new RaycastCCDManager(gScene);

		printf("- Using raycast CCD.\n");
	}
	else if(ccd == NO_CCD)
	{
		gScene = gPhysics->createScene(sceneDesc);

		printf("- Using no CCD.\n");
	}

	// Create a scenario that requires angular CCD: a rotating plank colliding with a falling box.
	{
		PxRigidDynamic* actor = createDynamic(PxTransform(PxVec3(40.0f, 20.0f, 0.0f)), PxBoxGeometry(10.0f, 1.0f, 0.1f), PxVec3(0.0f), enableLinearCCD, enableSpeculativeCCD);
		actor->setAngularVelocity(PxVec3(0.0f, 10.0f, 0.0f));
		actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
		actor->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_X, true);
		actor->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y, true);
		actor->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z, true);
		actor->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, true);
		actor->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, true);

		if(gRaycastCCD)
			registerForRaycastCCD(actor);

		PxRigidDynamic* actor2 = createDynamic(PxTransform(PxVec3(40.0f, 20.0f, 10.0f)), PxBoxGeometry(0.1f, 1.0f, 1.0f), PxVec3(0.0f), enableLinearCCD, enableSpeculativeCCD);

		if(gRaycastCCD)
			registerForRaycastCCD(actor2);
	}

	// Create a scenario that requires linear CCD: a fast moving sphere moving towards a box stack.
	{
		PxRigidDynamic* actor = createDynamic(PxTransform(PxVec3(0.0f, 18.0f, 100.0f)), PxSphereGeometry(2.0f), PxVec3(0.0f, 0.0f, -1000.0f), enableLinearCCD, enableSpeculativeCCD);
	
		if(gRaycastCCD)
			registerForRaycastCCD(actor);

		createStack(PxTransform(PxVec3(0, 0, stackZ)), 10, 2.0f, enableLinearCCD, enableSpeculativeCCD);

		PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
		gScene->addActor(*groundPlane);
	}

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
}

void renderText()
{
#ifdef RENDER_SNIPPET
	Snippets::print("Press F1 to F4 to select a scenario.");
	switch(PxU32(gScenario))
	{
		case 0:	{ Snippets::print("Current scenario: linear CCD");				}break;
		case 1:	{ Snippets::print("Current scenario: angular CCD");				}break;
		case 2:	{ Snippets::print("Current scenario: linear + angular CCD");	}break;
		case 3:	{ Snippets::print("Current scenario: raycast CCD");				}break;
		case 4:	{ Snippets::print("Current scenario: no CCD");					}break;
	}
#endif
}

void initPhysics(bool /*interactive*/)
{
	printf("CCD snippet. Use these keys:\n");
	printf(" P        - enable/disable pause\n");
	printf(" O        - step simulation for one frame\n");
	printf(" R        - reset scene\n");
	printf(" F1 to F4 - select scenes with different CCD algorithms\n");
	printf("    F1	  - Using linear CCD\n");
	printf("    F2    - Using speculative/angular CCD\n");
	printf("    F3    - Using full CCD (linear+angular)\n");
	printf("    F4	  - Using raycast CCD\n");
	printf("    F5	  - Using no CCD\n");
	printf("\n");

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	const PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.25f);

	initScene();
}

void stepPhysics(bool /*interactive*/)
{
	if (gPause && !gOneFrame)
		return;
	gOneFrame = false;

	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);

	// Simply call this after fetchResults to perform CCD raycasts.
	if(gRaycastCCD)
		gRaycastCCD->doRaycastCCD(true);
}
	
static void releaseScene()
{
	PX_RELEASE(gScene);
	PX_DELETE(gRaycastCCD);
}

void cleanupPhysics(bool /*interactive*/)
{
	releaseScene();
		
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetCCD done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	if(key == 'p' || key == 'P')
		gPause = !gPause;

	if(key == 'o' || key == 'O')
	{
		gPause = true;
		gOneFrame = true;
	}

	if(gScene)
	{
		if(key >= 1 && key <= gScenarioCount)
		{
			gScenario = key - 1;
			releaseScene();
			initScene();
		}

		if(key == 'r' || key == 'R')
		{
			releaseScene();
			initScene();
		}
	}
}

int snippetMain(int, const char*const*)
{
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

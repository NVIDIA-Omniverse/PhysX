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
// This snippet illustrates how to enable gyroscopic forces. The behavior of
// the object is known as the Dzhanibekov effect.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
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

static bool			gPause			= false;
static bool			gOneFrame		= false;
static const PxU32	gScenarioCount	= 2;
static PxU32		gScenario		= 0;

static void initScene()
{
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	const PxTransform pose(PxVec3(0.0f, 1.0f, 0.0f));

	PxRigidDynamic* actor = gPhysics->createRigidDynamic(pose);

	PxShape* shape0 = gPhysics->createShape(PxBoxGeometry(PxVec3(0.05f, 0.5f, 0.05f)), *gMaterial, true);
	actor->attachShape(*shape0);

	PxShape* shape1 = gPhysics->createShape(PxBoxGeometry(PxVec3(0.1f, 0.05f, 0.05f)), *gMaterial, true);
	shape1->setLocalPose(PxTransform(PxVec3(0.1f, 0.0f, 0.0f)));
	actor->attachShape(*shape1);

	PxRigidBodyExt::updateMassAndInertia(*actor, 1.0f);

	actor->setAngularVelocity(PxVec3(30.f*0.25f, 20.1f*0.25f, 0.0f));
	actor->setAngularDamping(0.0f);

	if(gScenario==0)
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, true);

	gScene->addActor(*actor);

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
	Snippets::print("Press F1 or F2 to run with or without gyroscopic forces enabled.");
#endif
}

void initPhysics(bool /*interactive*/)
{
	printf("Gyroscopic snippet. Use these keys:\n");
	printf(" P        - enable/disable pause\n");
	printf(" O        - step simulation for one frame\n");
	printf(" R        - reset scene\n");
	printf(" F1 to F2 - run with or without gyroscopic forces enabled\n");
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
}
	
static void releaseScene()
{
	PX_RELEASE(gScene);
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
	
	printf("SnippetGyroscopic done.\n");
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

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
// This snippet illustrates different ways of setting mass for rigid bodies.
//
// It creates 5 snowmen with different mass properties:
// - massless with a weight at the bottom
// - only the mass of the lowest snowball
// - the mass of all the snowballs
// - the whole mass but with a low center of gravity
// - manual setup of masses
//
// The different mass properties can be visually inspected by firing a rigid 
// ball towards each snowman using the space key.
// 
// For more details, please consult the "Rigid Body Dynamics" section of the 
// user guide.
// 
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd = NULL;


// create a dynamic ball to throw at the snowmen.
static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static PxRigidDynamic* createSnowMan(const PxTransform& pos, PxU32 mode)
{
	PxRigidDynamic* snowmanActor = gPhysics->createRigidDynamic(PxTransform(pos));
	if(!snowmanActor)
	{
		printf("create snowman actor failed");
		return NULL;
	}

	PxShape* armL = NULL; PxShape* armR = NULL;

	switch(mode%5)
	{
	case 0: // with a weight at the bottom
		{
			PxShape* shape = NULL;
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.2), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,-.29,0)));
			
			PxRigidBodyExt::updateMassAndInertia(*snowmanActor,10);

			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.5), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.4), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,.6,0)));
			
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.3), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,1.1,0)));
			
			armL = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armL)
				printf("creating snowman shape failed");
			armL->setLocalPose(PxTransform(PxVec3(-.4,.7,0)));

			armR = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armR)
				printf("creating snowman shape failed");			
			armR->setLocalPose(PxTransform(PxVec3( .4,.7,0)));
		}
		break;
	case 1: // only considering lowest shape mass
		{
			PxShape* shape = NULL;
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.5), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			
			PxRigidBodyExt::updateMassAndInertia(*snowmanActor,1);

			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.4), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,.6,0)));

			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.3), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,1.1,0)));

			armL = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armL)
				printf("creating snowman shape failed");
			armL->setLocalPose(PxTransform(PxVec3(-.4,.7,0)));

			armR = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armR)
				printf("creating snowman shape failed");			
			armR->setLocalPose(PxTransform(PxVec3( .4,.7,0)));

			snowmanActor->setCMassLocalPose(PxTransform(PxVec3(0,-.5,0)));
		}
		break;
	case 2: // considering whole mass
		{
			PxShape* shape = NULL;
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.5), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.4), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,.6,0)));

			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.3), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,1.1,0)));
			
			armL = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armL)
				printf("creating snowman shape failed");
			armL->setLocalPose(PxTransform(PxVec3(-.4,.7,0)));

			armR = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armR)
				printf("creating snowman shape failed");			
			armR->setLocalPose(PxTransform(PxVec3( .4,.7,0)));

			PxRigidBodyExt::updateMassAndInertia(*snowmanActor,1);
			snowmanActor->setCMassLocalPose(PxTransform(PxVec3(0,-.5,0)));
		}
		break;
	case 3: // considering whole mass with low COM
		{
			PxShape* shape = NULL;
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.5), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");

			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.4), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,.6,0)));

			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.3), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,1.1,0)));

			armL = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armL)
				printf("creating snowman shape failed");
			armL->setLocalPose(PxTransform(PxVec3(-.4,.7,0)));

			armR = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armR)
				printf("creating snowman shape failed");			
			armR->setLocalPose(PxTransform(PxVec3( .4,.7,0)));

			const PxVec3 localPos = PxVec3(0,-.5,0);
			PxRigidBodyExt::updateMassAndInertia(*snowmanActor,1,&localPos);
		}
		break;
	case 4: // setting up mass properties manually
		{
			PxShape* shape = NULL;
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.5), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			
			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.4), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,.6,0)));

			shape = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxSphereGeometry(.3), *gMaterial);
			if(!shape)
				printf("creating snowman shape failed");
			shape->setLocalPose(PxTransform(PxVec3(0,1.1,0)));

			armL = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armL)
				printf("creating snowman shape failed");
			armL->setLocalPose(PxTransform(PxVec3(-.4,.7,0)));

			armR = PxRigidActorExt::createExclusiveShape(*snowmanActor, PxCapsuleGeometry(.1,.1), *gMaterial);
			if(!armR)
				printf("creating snowman shape failed");			
			armR->setLocalPose(PxTransform(PxVec3( .4,.7,0)));

			snowmanActor->setMass(1);
			snowmanActor->setCMassLocalPose(PxTransform(PxVec3(0,-.5,0)));
			snowmanActor->setMassSpaceInertiaTensor(PxVec3(.05,100,100));
		}
		break;
	default:
		break;
	}
	
	gScene->addActor(*snowmanActor);

	return snowmanActor;
}

static void createSnowMen()
{
	PxU32 numSnowmen = 5;
	for(PxU32 i=0; i<numSnowmen; i++)
	{	
		PxVec3 pos(i * 2.5f,1,-8);
		createSnowMan(PxTransform(pos), i);
	}
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

    createSnowMen();
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);

	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}

	PX_RELEASE(gFoundation);
	
	printf("SnippetMassProperties done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case ' ':	createDynamic(camera, PxSphereGeometry(0.1f), camera.rotate(PxVec3(0,0,-1))*20);	break;
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

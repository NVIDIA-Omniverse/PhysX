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


// *******************************************************************************************************
// In addition to the simulate() function, which performs both collision detection and dynamics update,
// the PhysX SDK provides an api for separate execution of the collision detection and dynamics update steps.
// We shall refer to this feature as "split sim". This snippet demonstrates two ways to use the split sim feature 
// so that application work can be performed concurrently with the collision detection step.

// The snippet creates a list of kinematic box actors along with a number of dynamic actors that
// interact with the kinematic actors. 

//The defines OVERLAP_COLLISION_AND_RENDER_WITH_NO_LAG and OVERLAP_COLLISION_AND_RENDER_WITH_ONE_FRAME_LAG 
//demonstrate two distinct modes of split sim operation:

// (1)Enabling OVERLAP_COLLISION_AND_RENDER_WITH_NO_LAG allows the collision detection step to run in parallel 
//    with the renderer and with the update of the kinematic target poses without introducing any lag between 
//    application time and physics time.  This is equivalent to calling simulate() and fetchResults() with the key 
//    difference being that the application can schedule work to run concurrently with the collision detection.  
//    A consequence of this approach is that the first frame is more expensive than subsequent frames because it has to 
//    perform blocking collision detection and dynamics update calls.

// (2)OVERLAP_COLLISION_AND_RENDER_WITH_ONE_FRAME_LAG also allows the collision to run in parallel with 
//    the renderer and the update of the kinematic target poses but this time with a lag between physics time and 
//    application time; that is, the physics is always a single timestep behind the application because the first
//    frame merely starts the collision detection for the subsequent frame.  A consequence of this approach is that 
//    the first frame is cheaper than subsequent frames.
// ********************************************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

//This will allow the split sim to overlap collision and render and game logic.
#define OVERLAP_COLLISION_AND_RENDER_WITH_NO_LAG  1
#define OVERLAP_COLLISION_AND_RENDER_WITH_ONE_FRAME_LAG 0

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd        = NULL;

#define NB_KINE_X	16
#define NB_KINE_Y	16
#define KINE_SCALE	3.1f

static bool isFirstFrame = true;

PxRigidDynamic* gKinematics[NB_KINE_Y][NB_KINE_X];
PxTransform gKinematicTargets[NB_KINE_Y][NB_KINE_X];

void createDynamics()
{
	const PxU32 NbX = 8;
	const PxU32 NbY = 8;

	const PxVec3 dims(0.2f, 0.1f, 0.2f);
	const PxReal sphereRadius = 0.2f;
	const PxReal capsuleRadius = 0.2f;
	const PxReal halfHeight = 0.5f;

	const PxU32 NbLayers = 3;
	const float YScale = 0.4f;
	const float YStart = 6.0f;
	PxShape* boxShape = gPhysics->createShape(PxBoxGeometry(dims), *gMaterial);
	PxShape* sphereShape = gPhysics->createShape(PxSphereGeometry(sphereRadius), *gMaterial);
	PxShape* capsuleShape = gPhysics->createShape(PxCapsuleGeometry(capsuleRadius, halfHeight), *gMaterial);
	PX_UNUSED(boxShape);
	PX_UNUSED(sphereShape);
	PX_UNUSED(capsuleShape);
	for(PxU32 j=0;j<NbLayers;j++)
	{
		const float angle = float(j)*0.08f;
		const PxQuat rot = PxGetRotYQuat(angle);

		const float ScaleX = 4.0f;
		const float ScaleY = 4.0f;

		for(PxU32 y=0;y<NbY;y++)
		{
			for(PxU32 x=0;x<NbX;x++)
			{
				const float xf = (float(x)-float(NbX)*0.5f)*ScaleX;
				const float yf = (float(y)-float(NbY)*0.5f)*ScaleY;

				PxRigidDynamic* dynamic = NULL;

				PxU32 v = j&3;
				PxVec3 pos = PxVec3(xf, YStart + float(j)*YScale, yf);

				switch(v)
				{
					case 0:
						{
							PxTransform pose(pos, rot);
							dynamic = gPhysics->createRigidDynamic(pose);
							dynamic->attachShape(*boxShape);
							break;
						}
					case 1:
						{
							PxTransform pose(pos, PxQuat(PxIdentity));
							dynamic = gPhysics->createRigidDynamic(pose);
							dynamic->attachShape(*sphereShape);
							break;
						}
					default:
						{
							PxTransform pose(pos, rot);
							dynamic = gPhysics->createRigidDynamic(pose);
							dynamic->attachShape(*capsuleShape);
							break;
						}
				};

				PxRigidBodyExt::updateMassAndInertia(*dynamic, 10.f);

				gScene->addActor(*dynamic);

			}
		}
	}
}

void createGroudPlane()
{
	PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	PxRigidStatic* actor = gPhysics->createRigidStatic(pose);
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, PxPlaneGeometry(), *gMaterial);
	PX_UNUSED(shape);
	gScene->addActor(*actor);
}

void createKinematics()
{
	const PxU32 NbX = NB_KINE_X;
	const PxU32 NbY = NB_KINE_Y;

	const PxVec3 dims(1.5f, 0.2f, 1.5f);
	const PxQuat rot = PxQuat(PxIdentity);

	const float YScale = 0.4f;
	
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(dims), *gMaterial);

	
	const float ScaleX = KINE_SCALE;
	const float ScaleY = KINE_SCALE;
	for(PxU32 y=0;y<NbY;y++)
	{
		for(PxU32 x=0;x<NbX;x++)
		{
			const float xf = (float(x)-float(NbX)*0.5f)*ScaleX;
			const float yf = (float(y)-float(NbY)*0.5f)*ScaleY;
			PxTransform pose(PxVec3(xf, 0.2f + YScale, yf), rot);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(pose);
			body->attachShape(*shape);
			gScene->addActor(*body);
			body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

			gKinematics[y][x] = body;
		}
	}
	
}

void updateKinematicTargets(PxReal timeStep)
{
	const float YScale = 0.4f;
	
	PxTransform motion;
	motion.q = PxQuat(PxIdentity);

	static float gTime = 0.0f;
	gTime += timeStep;

	const PxU32 NbX = NB_KINE_X;
	const PxU32 NbY = NB_KINE_Y;

	const float Coeff = 0.2f;

	const float ScaleX = KINE_SCALE;
	const float ScaleY = KINE_SCALE;
	for(PxU32 y=0;y<NbY;y++)
	{
		for(PxU32 x=0;x<NbX;x++)
		{
			const float xf = (float(x)-float(NbX)*0.5f)*ScaleX;
			const float yf = (float(y)-float(NbY)*0.5f)*ScaleY;

			const float h = sinf(gTime*2.0f + float(x)*Coeff + + float(y)*Coeff)*2.0f;
			motion.p = PxVec3(xf, h + 2.0f + YScale, yf);

			gKinematicTargets[y][x] = motion;
		}
	}
}

void applyKinematicTargets()
{
	const PxU32 NbX = NB_KINE_X;
	const PxU32 NbY = NB_KINE_Y;

	for(PxU32 y=0;y<NbY;y++)
	{
		for(PxU32 x=0;x<NbX;x++)
		{
			PxRigidDynamic* kine = gKinematics[y][x];
			const PxTransform& target = gKinematicTargets[y][x];
			kine->setKinematicTarget(target);
		}
	}
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);
	
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

	createKinematics();
	createDynamics();

}

#if OVERLAP_COLLISION_AND_RENDER_WITH_NO_LAG
void stepPhysics(bool /*interactive*/)
{
	const PxReal timeStep = 1.0f/60.0f;

	if(isFirstFrame)
	{
		//Run the first frame's collision detection
		gScene->collide(timeStep);
		isFirstFrame = false;
	}
	//update the kinematice target pose in parallel with collision running
	updateKinematicTargets(timeStep);
	gScene->fetchCollision(true);
	//apply the computed and buffered kinematic target poses
	applyKinematicTargets();
	gScene->advance();
	gScene->fetchResults(true); 
	
	//Run the deferred collision detection for the next frame. This will run in parallel with render.
	gScene->collide(timeStep);
}
#elif OVERLAP_COLLISION_AND_RENDER_WITH_ONE_FRAME_LAG

void stepPhysics(bool /*interactive*/)
{
	PxReal timeStep = 1.0/60.0f;

	//update the kinematice target pose in parallel with collision running
	updateKinematicTargets(timeStep);
	if(!isFirstFrame)
	{
		gScene->fetchCollision(true);
		//apply the computed and buffered kinematic target poses
		applyKinematicTargets();
		gScene->advance();
		gScene->fetchResults(true); 
	}
	else
		applyKinematicTargets();

	isFirstFrame = false;
	//Run the deferred collision detection for the next frame. This will run in parallel with render.
	gScene->collide(timeStep);
}

#else

void stepPhysics(bool /*interactive*/)
{
	PxReal timeStep = 1.0/60.0f;
	//update the kinematice target pose in parallel with collision running
	gScene->collide(timeStep);
	updateKinematicTargets(timeStep);
	gScene->fetchCollision(true);
	//apply the computed and buffered kinematic target poses
	applyKinematicTargets();
	gScene->advance();
	gScene->fetchResults(true); 
}
#endif

void cleanupPhysics(bool /*interactive*/)
{
#if OVERLAP_COLLISION_AND_RENDER_WITH_NO_LAG || OVERLAP_COLLISION_AND_RENDER_WITH_ONE_FRAME_LAG
	//Close out remainder of previously running scene. If we don't do this, it will be implicitly done
	//in gScene->release() but a warning will be issued.
	gScene->fetchCollision(true);
	gScene->advance();
	gScene->fetchResults(true);
#endif

	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetSplitSim done.\n");
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

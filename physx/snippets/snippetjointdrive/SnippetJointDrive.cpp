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
// This snippet illustrates simple use of joint drives in physx
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation					= NULL;
static PxPhysics*				gPhysics					= NULL;
static PxDefaultCpuDispatcher*	gDispatcher					= NULL;
static PxScene*					gScene						= NULL;
static PxMaterial*				gMaterial					= NULL;
static PxPvd*					gPvd						= NULL;
#if PX_SUPPORT_GPU_PHYSX
static PxCudaContextManager*	gCudaContextManager			= NULL;
#endif
static bool						gPause						= false;
static bool						gOneFrame					= false;
static bool						gChangeObjectAType			= false;
static bool						gChangeObjectBRotation		= false;
static bool						gChangeJointFrameARotation	= false;
static bool						gChangeJointFrameBRotation	= false;
#if PX_SUPPORT_GPU_PHYSX
static bool						gUseGPU						= false;
#endif
static PxU32					gSceneIndex					= 0;
static const PxU32				gMaxSceneIndex				= 4;

static void setupActor(PxRigidActor* actor)
{
	actor->setActorFlag(PxActorFlag::eVISUALIZATION, true);
	gScene->addActor(*actor);
}

static void createScene()
{
	PX_RELEASE(gScene);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	// Disable gravity so that the motion is only produced by the drive
	sceneDesc.gravity = PxVec3(0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
#if PX_SUPPORT_GPU_PHYSX
	if(gUseGPU)
	{
		sceneDesc.cudaContextManager = gCudaContextManager;
		sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
		sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
		sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
		sceneDesc.gpuMaxNumPartitions = 8;
	}
#endif
	gScene = gPhysics->createScene(sceneDesc);
	// Visualize joint local frames
	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	if(gSceneIndex<gMaxSceneIndex)
	{
		const PxQuat rotZ = PxGetRotZQuat(-PxPi/4.0f);

		const PxBoxGeometry boxGeom(0.5f, 0.5f, 0.5f);
		PxTransform tr(PxVec3(0.0f, 2.0f, -20.0f));

		PxRigidActor* actor0;
		if(gChangeObjectAType)
		{
			PxRigidDynamic* actor = PxCreateDynamic(*gPhysics, tr, boxGeom, *gMaterial, 1.0f);
			actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
			actor0 = actor;
		}
		else
		{
			actor0 = PxCreateStatic(*gPhysics, tr, boxGeom, *gMaterial);
		}
		setupActor(actor0);

		tr.p.x += boxGeom.halfExtents.x * 2.0f;
		if(gChangeObjectBRotation)
			tr.q = rotZ;

		PxRigidDynamic* actor1 = PxCreateDynamic(*gPhysics, tr, boxGeom, *gMaterial, 1.0f);
		setupActor(actor1);

		PxTransform jointFrame0(PxIdentity);
		PxTransform jointFrame1(PxIdentity);

		// We're going to setup a linear drive along "X" = actor0's joint frame's X axis.
		// That axis will be either aligned with the actors' X axis or tilted 45 degrees.
		if(gChangeJointFrameARotation)
			jointFrame0.q = rotZ;
		else
			jointFrame0.q = PxQuat(PxIdentity);

		if(gChangeJointFrameBRotation)
			jointFrame1.q = rotZ;
		else
			jointFrame1.q = PxQuat(PxIdentity);

		PxD6Joint* j = PxD6JointCreate(*gPhysics, actor0, jointFrame0, actor1, jointFrame1);
		j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);

		// Locked axes would move the joint frames & snap them together. In this test we explicitly want them disjoint,
		// to check in which direction the drives operates. So we set all DOFs free to make sure none of that interferes
		// with the drive.
		j->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);

		if(gSceneIndex==0)
		{
			// Linear drive along "X" = actor0's joint frame's X axis
			j->setDrive(PxD6Drive::eX, PxD6JointDrive(0, 1000, FLT_MAX, true));
			j->setDriveVelocity(PxVec3(1.0f, 0.0f, 0.0f), PxVec3(0.0f), true);
		}
		else if(gSceneIndex==1)
		{
			j->setDrive(PxD6Drive::eTWIST, PxD6JointDrive(0, 1000, FLT_MAX, true));
			j->setDriveVelocity(PxVec3(0.0f), PxVec3(1.0f, 0.0f, 0.0f), true);
		}
		else if(gSceneIndex==2)
		{
			j->setDrive(PxD6Drive::eSWING, PxD6JointDrive(0, 1000, FLT_MAX, true));
			j->setDriveVelocity(PxVec3(0.0f), PxVec3(0.0f, 1.0f, 0.0f), true);
		}
		else if(gSceneIndex==3)
		{
			j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));
			j->setDriveVelocity(PxVec3(0.0f), PxVec3(0.0f, 1.0f, 0.0f), true);
		}
	}
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

#if PX_SUPPORT_GPU_PHYSX
	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	if(gCudaContextManager)
	{
		if(!gCudaContextManager->contextIsValid())
			PX_RELEASE(gCudaContextManager);
	}	
#endif

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	createScene();
}

void stepPhysics(bool /*interactive*/)
{
	if(gPause && !gOneFrame)
		return;
	gOneFrame = false;

	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
#if PX_SUPPORT_GPU_PHYSX
	PX_RELEASE(gCudaContextManager);
#endif
	PX_RELEASE(gFoundation);
	
	printf("SnippetJointDrive done.\n");
}

void renderText()
{
#ifdef RENDER_SNIPPET
	Snippets::print("Press F1 to change body0's joint frame orientation");
	Snippets::print("Press F2 to change body0's type (static/kinematic)");
	Snippets::print("Press F3 to change body1's joint frame orientation");
	Snippets::print("Press F4 to change body1's orientation");
#if PX_SUPPORT_GPU_PHYSX
	Snippets::print("Press F5 to use CPU or GPU");
#endif
	Snippets::print("Press F6 to select the next drive");
	switch(gSceneIndex)
	{
	case 0:
		Snippets::print("Current drive: linear X");
		break;
	case 1:
		Snippets::print("Current drive: angular twist (around X)");
		break;
	case 2:
		Snippets::print("Current drive: angular swing (around Y)");
		break;
	case 3:
		Snippets::print("Current drive: angular slerp (around Y)");
		break;
	}
#if PX_SUPPORT_GPU_PHYSX
	if(gUseGPU)
		Snippets::print("Current mode: GPU");
	else
		Snippets::print("Current mode: CPU");
#endif
	Snippets::print("body1's translation or rotation (drive) axis should only depend on body0's joint axes.");
#endif
}

void keyPress(unsigned char key, const PxTransform&)
{
	if(key=='p' || key=='P')
		gPause = !gPause;

	if(key=='o' || key=='O')
	{
		gPause = true;
		gOneFrame = true;
	}

	if(key==1)
	{
		gChangeJointFrameARotation = !gChangeJointFrameARotation;
		createScene();
	}
	else if(key==2)
	{
		gChangeObjectAType = !gChangeObjectAType;
		createScene();
	}
	else if(key==3)
	{
		gChangeJointFrameBRotation = !gChangeJointFrameBRotation;
		createScene();
	}
	else if(key==4)
	{
		gChangeObjectBRotation = !gChangeObjectBRotation;
		createScene();
	}
#if PX_SUPPORT_GPU_PHYSX
	else if(key==5)
	{
		gUseGPU = !gUseGPU;
		createScene();
	}
#endif
	else if(key==6)
	{
		gSceneIndex = gSceneIndex + 1;
		if(gSceneIndex==gMaxSceneIndex)
			gSceneIndex = 0;
		createScene();
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

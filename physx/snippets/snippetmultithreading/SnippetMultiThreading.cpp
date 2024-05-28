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
// This snippet shows how to coordinate threads performing asynchronous
// work during the scene simulation. After simulate() is called, user threads 
// are started that perform ray-casts against the scene. The call to 
// fetchResults() is delayed until all ray-casts have completed.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd = NULL;

struct RaycastThread
{
	SnippetUtils::Sync*		mWorkReadySyncHandle;
	SnippetUtils::Thread*	mThreadHandle;
};
const PxU32				 gNumThreads = 1;
RaycastThread			 gThreads[gNumThreads];

SnippetUtils::Sync*		 gWorkDoneSyncHandle;

const PxI32				 gRayCount = 1024;
volatile PxI32			 gRaysAvailable;
volatile PxI32			 gRaysCompleted;

static PxVec3 randVec3() 
{
	return (PxVec3(float(rand())/float(RAND_MAX),
		float(rand())/float(RAND_MAX), 
		float(rand())/float(RAND_MAX))*2.0f - PxVec3(1.0f)).getNormalized();
}

static void threadExecute(void* data)
{
	RaycastThread* raycastThread = static_cast<RaycastThread*>(data);

	// Perform random raycasts against the scene until stop.
	for(;;)
	{
		// Wait here for the sync to be set then reset the sync
		// to ensure that we only perform raycast work after the 
		// sync has been set again.
		SnippetUtils::syncWait(raycastThread->mWorkReadySyncHandle);
		SnippetUtils::syncReset(raycastThread->mWorkReadySyncHandle);

		// If the thread has been signaled to quit then exit this function.
		if (SnippetUtils::threadQuitIsSignalled(raycastThread->mThreadHandle))
			break;

		// Perform a fixed number of random raycasts against the scene
		// and share the work between multiple threads.
		while (SnippetUtils::atomicDecrement(&gRaysAvailable) >= 0)
		{
			PxVec3 dir = randVec3();

			PxRaycastBuffer buf;
			gScene->raycast(PxVec3(0.0f), dir.getNormalized(), 1000.0f, buf, PxHitFlag::eDEFAULT);

			// If this is the last raycast then signal this to the main thread.
			if (SnippetUtils::atomicIncrement(&gRaysCompleted) == gRayCount)
			{
				SnippetUtils::syncSet(gWorkDoneSyncHandle);
			}
		}
	}

	// Quit the current thread.
	SnippetUtils::threadQuit(raycastThread->mThreadHandle);
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void createPhysicsAndScene()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	
	gScene = gPhysics->createScene(sceneDesc);
	
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	for(PxU32 i=0;i<5;i++)
		createStack(PxTransform(PxVec3(0,0,i*10.0f)), 10, 2.0f);
}

void createRaycastThreads()
{
	// Create and start threads that will perform raycasts.
	// Create a sync for each thread so that a signal may be sent
	// from the main thread to the raycast thread that it can start 
	// performing raycasts.
	for (PxU32 i=0; i < gNumThreads; ++i)
	{
		//Create a sync.
		gThreads[i].mWorkReadySyncHandle = SnippetUtils::syncCreate();

		//Create and start a thread.
		gThreads[i].mThreadHandle =  SnippetUtils::threadCreate(threadExecute, &gThreads[i]);
	}

	// Create another sync so that the raycast threads can signal to the main 
	// thread that they have finished performing their raycasts.
	gWorkDoneSyncHandle = SnippetUtils::syncCreate();
}

void initPhysics()
{
	createPhysicsAndScene();
	createRaycastThreads();
}

void stepPhysics()
{
	// Start simulation
	gScene->simulate(1.0f/60.0f);

	// Start ray-cast threads
	gRaysAvailable = gRayCount;
	gRaysCompleted = 0;

	// Signal to each raycast thread that they can start performing raycasts.
	for (PxU32 i=0; i < gNumThreads; ++i)
	{
		SnippetUtils::syncSet(gThreads[i].mWorkReadySyncHandle);
	}

	// Wait for raycast threads to finish.
	SnippetUtils::syncWait(gWorkDoneSyncHandle);
	SnippetUtils::syncReset(gWorkDoneSyncHandle);

	// Fetch simulation results
	gScene->fetchResults(true);
}
	
void cleanupPhysics()
{
	// Signal threads to quit.
	for (PxU32 i=0; i < gNumThreads; ++i)
	{
		SnippetUtils::threadSignalQuit(gThreads[i].mThreadHandle);
		SnippetUtils::syncSet(gThreads[i].mWorkReadySyncHandle);
	}

	// Clean up raycast threads and syncs.
	for (PxU32 i=0; i < gNumThreads; ++i)
	{
		SnippetUtils::threadWaitForQuit(gThreads[i].mThreadHandle);
		SnippetUtils::threadRelease(gThreads[i].mThreadHandle);
		SnippetUtils::syncRelease(gThreads[i].mWorkReadySyncHandle);
	}

	// Clean up the sync for the main thread.
	SnippetUtils::syncRelease(gWorkDoneSyncHandle);

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
	
	printf("SnippetMultiThreading done.\n");
}

int snippetMain(int, const char*const*)
{
	initPhysics();

	for(PxU32 i=0; i<100; ++i)
		stepPhysics();

	cleanupPhysics();

	return 0;
}


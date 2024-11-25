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
// This snippet illustrates how to setup a custom profiler, and potentially
// re-route it to PVD's profiling functions.
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
static PxPvd*					gPvd        = NULL;

static PxReal stackZ = 10.0f;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
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

static const bool gCallPVDProfilingFunctions = false;

class CustomProfilerCallback : public PxProfilerCallback
{
public:
	virtual ~CustomProfilerCallback()	{}

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId)
	{
		// Option 1: add your own profiling code here (before calling the PVD function).
		// If you call the PVD profiling function below, adding your own profiling code here
		// means it will capture the cost of the PVD zoneStart function.

		// NB: we don't have an actual profiler implementation in the snippet so we just call printf instead
		printf("start: %s\n", eventName);

		// Optional: call the PVD function if you want to see the profiling results in PVD.
		void* profilerData = gCallPVDProfilingFunctions ? gPvd->zoneStart(eventName, detached, contextId) : NULL;

		// Option 2: add your own profiling code here (after calling the PVD function).
		// If you call the PVD profiling function above, adding your own profiling code here
		// means its cost will be captured by the PVD profiler.

		return profilerData;
	}

	virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId)
	{
		// Option 2: add your own profiling code here (before calling the PVD function).
		// If you call the PVD profiling function below, adding your own profiling code here
		// means its cost will be captured by the PVD profiler.

		// Optional: call the PVD function if you want to see the profiling results in PVD.
		if(gCallPVDProfilingFunctions)
			gPvd->zoneEnd(profilerData, eventName, detached, contextId);

		// Option 1: add your own profiling code here (after calling the PVD function).
		// If you call the PVD profiling function above, adding your own profiling code here
		// means it will capture the cost of the PVD zoneEnd function.

		// NB: we don't have an actual profiler implementation in the snippet so we just call printf instead
		printf("end: %s\n", eventName);
	}

	virtual void recordData(int32_t value, const char* valueName, uint64_t contextId)
	{
		printf("data: %s (context ID %llu) = %d\n", valueName, (unsigned long long)contextId, value);
	}

	virtual void recordData(float value, const char* valueName, uint64_t contextId)
	{
		printf("data: %s (context ID %llu) = %f\n", valueName, (unsigned long long)contextId, (double)value);
	}

}gCustomProfilerCallback;

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);

	// During the "connect" call, PVD sets itself up as the profiler if PxPvdInstrumentationFlag::ePROFILE is used.
	// That is, it internally calls PxSetProfilerCallback() to setup its own profiling callback. Any calls to
	// PxSetProfilerCallback() prior to calling "connect" is thus lost.
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	// This call should be performed after PVD is initialized, otherwise it will have no effect.
	PxSetProfilerCallback(&gCustomProfilerCallback);

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

	for(PxU32 i=0;i<5;i++)
		createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);

	if(!interactive)
		createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)

{	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

#if (PX_DEBUG || PX_CHECKED || PX_PROFILE)
	printf("SnippetCustomProfiler done.\n");
#else
	printf("Warning: SnippetCustomProfiler does not capture the profiler timings in release build.\n");
#endif
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);	break;
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

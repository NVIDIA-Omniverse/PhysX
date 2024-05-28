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
// This snippet illustrates the use of split fetchResults() calls to improve
// the performace contact report processing.
//
// It defines a filter shader function that requests touch reports for 
// all pairs, and a contact callback function that saves the contact points.  
// It configures the scene to use this filter and callback, and prints the 
// number of contact reports each frame. If rendering, it renders each 
// contact as a line whose length and direction are defined by the contact 
// impulse. The callback can be processed earlier than usual by using the
// split fetchResults() sequence of fetchResultsStart(), processCallbacks(), 
// fetchResultsFinish().
// 
// ****************************************************************************

#include <vector>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "foundation/PxAtomic.h"
#include "task/PxTask.h"

#define PARALLEL_CALLBACKS 1

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics = NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene = NULL;
static PxMaterial*				gMaterial = NULL;
static PxPvd*					gPvd = NULL;

const PxI32 maxCount = 10000;

PxI32 gSharedIndex = 0;

PxVec3* gContactPositions;
PxVec3* gContactImpulses;
PxVec3* gContactVertices;

class CallbackFinishTask : public PxLightCpuTask
{
	SnippetUtils::Sync* mSync;
public:
	CallbackFinishTask(){ mSync = SnippetUtils::syncCreate(); }

	virtual void release()
	{
		PxLightCpuTask::release();
		SnippetUtils::syncSet(mSync);
	}

	void reset() { SnippetUtils::syncReset(mSync); }

	void wait() { SnippetUtils::syncWait(mSync); }

	virtual void run() { /*Do nothing - release the sync in the release method for thread-safety*/}

	virtual const char* getName() const { return "CallbackFinishTask"; }
} 
callbackFinishTask;


PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT
		| PxPairFlag::eNOTIFY_TOUCH_FOUND
		| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
		| PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}

class ContactReportCallback : public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)	{ PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count)					{ PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
	{
		PX_UNUSED((pairHeader));
		//Maximum of 64 vertices can be produced by contact gen
		PxContactPairPoint contactPoints[64];

		for (PxU32 i = 0; i<nbPairs; i++)
		{
			PxU32 contactCount = pairs[i].contactCount;
			if (contactCount)
			{
				pairs[i].extractContacts(&contactPoints[0], contactCount);

				PxI32 startIdx = physx::PxAtomicAdd(&gSharedIndex, int32_t(contactCount));
				for (PxU32 j = 0; j<contactCount; j++)
				{
					gContactPositions[startIdx+j] = contactPoints[j].position;
					gContactImpulses[startIdx+j] = contactPoints[j].impulse;
					gContactVertices[2*(startIdx + j)] = contactPoints[j].position;
					gContactVertices[2*(startIdx + j) + 1] = contactPoints[j].position + contactPoints[j].impulse * 0.1f;
				}
			}
		}
	}
};

ContactReportCallback gContactReportCallback;

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (PxU32 i = 0; i<size; i++)
	{
		for (PxU32 j = 0; j<size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initPhysics(bool /*interactive*/)
{
	gContactPositions = new PxVec3[maxCount];
	gContactImpulses = new PxVec3[maxCount];
	gContactVertices = new PxVec3[2*maxCount];

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.gravity = PxVec3(0, -9.81f, 0);
	sceneDesc.filterShader = contactReportFilterShader;
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	const PxU32 nbStacks = 50;

	for (PxU32 i = 0; i < nbStacks; ++i)
	{
		createStack(PxTransform(PxVec3(0, 3.0f, 10.f - 5.f*i)), 5, 2.0f);
	}
}

void stepPhysics(bool /*interactive*/)
{
	gSharedIndex = 0;

	gScene->simulate(1.0f / 60.0f);

#if !PARALLEL_CALLBACKS
	gScene->fetchResults(true);
#else
	//Call fetchResultsStart. Get the set of pair headers
	const PxContactPairHeader* pairHeader;
	PxU32 nbContactPairs;
	gScene->fetchResultsStart(pairHeader, nbContactPairs, true);

	//Set up continuation task to be run after callbacks have been processed in parallel
	callbackFinishTask.setContinuation(*gScene->getTaskManager(), NULL);
	callbackFinishTask.reset();

	//process the callbacks
	gScene->processCallbacks(&callbackFinishTask);

	callbackFinishTask.removeReference();

	callbackFinishTask.wait();
	
	gScene->fetchResultsFinish();
#endif

	printf("%d contact reports\n", PxU32(gSharedIndex));
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	delete gContactPositions;
	delete gContactImpulses;
	delete gContactVertices;

	printf("SnippetSplitFetchResults done.\n");
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	initPhysics(false);
	for (PxU32 i = 0; i<250; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}


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
// This snippet illustrates kinematic actor updates in a substepped simulation.
//
// It uses chained continuation tasks that call fetchResults and run simulation steps. 
// The scene consists of a kinematic platform interacting with a dynamic 
// sphere. The kinematic actor's target pose is updated before every substep.
// ****************************************************************************

#include <new>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

using namespace physx;
using namespace SnippetUtils;

// The usual PhysX resources.
static PxDefaultAllocator			gAllocator;
static PxDefaultErrorCallback		gErrorCallback;
static PxFoundation*				gFoundation		= NULL;
static PxPhysics*					gPhysics		= NULL;
static PxMaterial*					gMaterial		= NULL;
static PxDefaultCpuDispatcher*		gDispatcher		= NULL;
static PxScene*						gScene			= NULL;
static PxRigidDynamic*				gKinematic		= NULL;

// A very simple substepping policy: just take 2 60Hz substeps per step.
static const PxReal			SUBSTEP_LENGTH	= 1.0f/60.0f;
static const PxU32			NUM_STEPS		= 1000;
static const PxI32			NUM_SUBSTEPS	= 2;
PxPvd*                  	gPvd			= NULL;
// Context for keeping track of the stepper state.
struct StepContext
{
	class SubstepCompletionTask*	taskPool;
	Sync*							completionSync;
	PxI32							nbSubstepsFinished;
	volatile PxI32					nbTasksDestroyed;
} gStepContext;

// Completion task for running a substep.

// The following sequencing is guaranteed:
// * the section of the run() method up to the removeReference() in startNextSubstep() will execute prior  
//   to the run() method of the task submitted by startNextSubstep()
// * the run() method of a task will run before its release() method
//
// Any work done by a task after releasing the next task (via removeReference()) could end up running in 
// parallel with that task, if simulate() completes sufficiently quickly or there is a context switch. In order
// to prevent races, it is therefore recommended that a completion task do no work after releasing the next.

class SubstepCompletionTask : public PxLightCpuTask
{
public:
	SubstepCompletionTask()
	{
		mTm = gScene->getTaskManager();
	}

	void run() 
	{		
		void startNextSubstep();

		gScene->fetchResults(true);
		if(++gStepContext.nbSubstepsFinished < NUM_SUBSTEPS)
			startNextSubstep();
	}

	void release()
	{
		this->~SubstepCompletionTask();

		// If we're done with all the substeps , synchronize with the main thread. In a real application 
		// we would most likely run dependent tasks instead.

		// We can only signal completion once all substepping resources are cleaned up.  
		// Release() calls may run concurrently or out of order, so we use an atomic counter.

		if(atomicIncrement(&gStepContext.nbTasksDestroyed) == NUM_SUBSTEPS)
			syncSet(gStepContext.completionSync);
	}

	const char* getName() const { return "Substep Completion Task"; }
};


// Update the sim inputs and start the next PhysX substep.

void startNextSubstep()
{
	// Compute new target pose for the kinematic at the end of the substep.

	static PxReal sTotalSeconds		= 0.0f;
	sTotalSeconds += SUBSTEP_LENGTH;

	const PxReal period		= 4.0f;
	const PxReal amplitude	= 10.0f;
	const PxReal angVel		= PxTwoPi/period;

	PxReal yPos = PxSin(angVel * sTotalSeconds) * amplitude;
	gKinematic->setKinematicTarget(PxTransform(0.0f, yPos, 0.0f));

	// Create a completion task and set its reference count to 1. This way we can safely submit it to simulate()
	// and, even if we get context-switched and simulate() completes before we get back, the task's run()
	// method will not execute until we're ready.

	SubstepCompletionTask* nextCompletion = new (gStepContext.taskPool+gStepContext.nbSubstepsFinished) SubstepCompletionTask();
	nextCompletion->addReference();

	// Kick off the sim with the new completion task. Once this call returns, worker threads will update the PhysX 
	// state in parallel with the rest of this function.

	gScene->simulate(SUBSTEP_LENGTH, nextCompletion);

	// We can do things here that can run in parallel with the simulation, but must happen before the next task's
	// run method executes. In this snippet, there's nothing to do...
	
	// Finally, remove the reference that prevents the next completion task running.
	
	nextCompletion->removeReference();
}



void runPhysics()
{
	// Initialize the substepping context.
	syncReset(gStepContext.completionSync);
	gStepContext.nbSubstepsFinished = 0;
	gStepContext.nbTasksDestroyed = 0;

	// Start the first substep, then wait for the last one to finish.
	startNextSubstep();
	syncWait(gStepContext.completionSync);
}

void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.2f);

	PxSceneDesc desc(gPhysics->getTolerancesScale());
	desc.filterShader = PxDefaultSimulationFilterShader;
	desc.cpuDispatcher = gDispatcher = PxDefaultCpuDispatcherCreate(2);
	desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gScene = gPhysics->createScene(desc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gKinematic = PxCreateKinematic(*gPhysics, PxTransform(PxIdentity), PxBoxGeometry(5.0f, 1.0f, 5.0f), *gMaterial, 1.0f);
	gScene->addActor(*gKinematic);

	PxRigidDynamic* sphere = PxCreateDynamic(*gPhysics, PxTransform(0.0f, 5.0f, 0.0f), PxSphereGeometry(1.0f), *gMaterial, 1.0f);
	gScene->addActor(*sphere);
}

void cleanupPhysics()
{
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
}

int snippetMain(int, const char*const*)
{
	initPhysics();

	// Storage and synchronization for substepping.
	gStepContext.taskPool = reinterpret_cast<SubstepCompletionTask*>(malloc(NUM_SUBSTEPS * sizeof(SubstepCompletionTask)));
	gStepContext.completionSync = syncCreate();

	for(PxU32 i=0; i<NUM_STEPS; i++)
		runPhysics();

	syncRelease(gStepContext.completionSync);
	free(gStepContext.taskPool);

	cleanupPhysics();

	printf("SnippetStepper done.\n");

	return 0;
}

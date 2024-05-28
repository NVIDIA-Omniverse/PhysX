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

// ********************************************************************************
// This snippet illustrates the concept of PxToleranceScale.
//
// It creates 2 scenes using different units for length and mass.
// Use PVD to replay the scene and see how scaling affects the simulation.
// ********************************************************************************

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

static PxReal gStackZ = 10.0f;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxReal& mass, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	PxRigidBodyExt::setMassAndUpdateInertia(*dynamic, mass);
	gScene->addActor(*dynamic);
	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent, const PxReal& mass)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::setMassAndUpdateInertia(*body, mass);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initPhysics(bool interactive, const PxTolerancesScale& scale, PxReal scaleMass)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale, true, gPvd);

	PxReal scaleLength = scale.length;

	PxSceneDesc sceneDesc(scale);
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f) * scaleLength;
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
		createStack(PxTransform(PxVec3(0,0,gStackZ-=10.0f) * scaleLength), 10, 2.0f * scaleLength, 1.0f * scaleMass);

	if(!interactive)
		createDynamic(PxTransform(PxVec3(0,40,100) * scaleLength), PxSphereGeometry(10 * scaleLength), 100.0f * scaleMass, PxVec3(0,-50,-100) * scaleLength);
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
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
}

void runSim(const PxTolerancesScale& scale, PxReal scaleMass)
{
	static const PxU32 frameCount = 150;
	initPhysics(false, scale, scaleMass);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
}

int snippetMain(int, const char*const*)
{
	PxTolerancesScale scale;

	// Default
	printf("PxToleranceScale (Default).\n");
	runSim(scale, 1000.0f);

	// Reset position of pyramid stack z coordinate to default
	gStackZ = 10.0f;

	// Scaled assets
	printf("PxToleranceScale (Scaled).\n");
	scale.length = 100;				// length in cm
 	scale.speed *= scale.length;	// speed in cm/s
	runSim(scale, 1.0f);

	printf("SnippetToleranceScale done.\n");

	return 0;
}

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
// This snippet illustrates the usage of PxBVH for PxScene's addActor function.
//
// It creates a large number of small sphere shapes forming a large sphere. Large sphere
// represents an actor and the actor is inserted into the scene with a BVH
// that is precomputed from all the small spheres. When an actor is inserted this
// way the scene queries against this object behave actor centric rather than shape
// centric.
// Each actor that is added with a BVH does not update any of its shape bounds
// within a pruning structure. It does update just the actor bounds and the query then
// goes into actors bounds pruner, then a local query is done against the shapes in the
// actor.
// For a dynamic actor consisting of a large amound of shapes there can be a significant
// performance benefits. During fetch results, there is no need to synchronize all
// shape bounds into scene query system. Also when a new AABB tree is build inside
// scene query system these actors shapes are not contained there.
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

static void createLargeSphere(const PxTransform& t, PxU32 density, PxReal largeRadius, PxReal radius, bool useAggregate)
{
	PxRigidDynamic* body = gPhysics->createRigidDynamic(t);

	// generate the sphere shapes
	const float gStep = PxPi/float(density);
	const float tStep = 2.0f*PxPi/float(density);
	for(PxU32 i=0; i<density;i++)
	{
		for(PxU32 j=0;j<density;j++)
		{
			const float sinG = PxSin(gStep * i);
			const float cosG = PxCos(gStep * i);
			const float sinT = PxSin(tStep * j);
			const float cosT = PxCos(tStep * j);

			PxTransform localTm(PxVec3(largeRadius*sinG*cosT, largeRadius*sinG*sinT, largeRadius*cosG));
			PxShape* shape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);
			shape->setLocalPose(localTm);
			body->attachShape(*shape);
			shape->release();
		}
	}
	PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);

	// get the bounds from the actor, this can be done through a helper function in PhysX extensions
	PxU32 numBounds = 0;
	PxBounds3* bounds = PxRigidActorExt::getRigidActorShapeLocalBoundsList(*body, numBounds);

	printf("Creating BVH structure for large compound actor...\n");

	// setup the PxBVHDesc, it does contain only the PxBounds3 data
	PxBVHDesc bvhDesc;
	bvhDesc.bounds.count = numBounds;
	bvhDesc.bounds.data = bounds;
	bvhDesc.bounds.stride = sizeof(PxBounds3);

	// cook the bvh
	PxBVH* bvh = PxCreateBVH(bvhDesc, gPhysics->getPhysicsInsertionCallback());

	// release the memory allocated within extensions, the bounds are not required anymore
	gAllocator.deallocate(bounds);

	if(useAggregate)
		printf("Adding actor + BVH structure to aggregate...\n");
	else
		printf("Adding actor + BVH structure to scene...\n");

	// add the actor to the scene and provide the bvh structure (regular path without aggregate usage)
	if(!useAggregate)
		gScene->addActor(*body, bvh);

	// Note that when objects with large amound of shapes are created it is also
	// recommended to create an aggregate from them, see the code below that would replace
	// the gScene->addActor(*body, bvh)
	if(useAggregate)
	{
		PxAggregate* aggregate = gPhysics->createAggregate(1, body->getNbShapes(), false);
		aggregate->addActor(*body, bvh);
		gScene->addAggregate(*aggregate);
	}

	// bvh can be released at this point, the precomputed BVH structure was copied to the SDK pruners.
	bvh->release();
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

	for(PxU32 i = 0; i < 10; i++)
		createLargeSphere(PxTransform(PxVec3(200.0f*i, .0f, 100.0f)), 50, 30.0f, 1.0f, false);
}

void stepPhysics(bool /*interactive*/)
{
	printf("Simulating...\n");
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

	printf("SnippetBVH done.\n");
}

void keyPress(unsigned char , const PxTransform& )
{
}

int snippetMain(int, const char*const*)
{
	static const PxU32 frameCount = 50;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
	return 0;
}

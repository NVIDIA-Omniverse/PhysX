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
// This snippet illustrates the use of simple contact reports in combination
// with continuous collision detection (CCD). Furthermore, extra contact report
// data will be requested.
//
// The snippet defines a filter shader function that enables CCD and requests
// touch reports for all pairs, and a contact callback function that saves the 
// contact points and the actor positions at time of impact.
// It configures the scene to use this filter and callback, enables CCD and 
// prints the number of contact points found. If rendering, it renders each 
// contact as a line whose length and direction are defined by the contact 
// impulse (the line points in the opposite direction of the impulse). In
// addition, the path of the fast moving dynamic object is drawn with lines.
// 
// ****************************************************************************

#include <vector>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation			= NULL;
static PxPhysics*				gPhysics			= NULL;
static PxDefaultCpuDispatcher*	gDispatcher			= NULL;
static PxScene*					gScene				= NULL;
static PxMaterial*				gMaterial			= NULL;
static PxTriangleMesh*			gTriangleMesh		= NULL;
static PxRigidStatic*			gTriangleMeshActor	= NULL;
static PxRigidDynamic*			gSphereActor		= NULL;
static PxPvd*					gPvd                = NULL;
static PxU32					gSimStepCount		= 0;

std::vector<PxVec3> gContactPositions;
std::vector<PxVec3> gContactImpulses;
std::vector<PxVec3> gContactSphereActorPositions;

static PxFilterFlags contactReportFilterShader(	PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
												PxFilterObjectAttributes attributes1, PxFilterData filterData1,
												PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	//
	// Enable CCD for the pair, request contact reports for initial and CCD contacts.
	// Additionally, provide information per contact point and provide the actor
	// pose at the time of contact.
	//

	pairFlags = PxPairFlag::eCONTACT_DEFAULT
			  | PxPairFlag::eDETECT_CCD_CONTACT
			  | PxPairFlag::eNOTIFY_TOUCH_CCD
			  |	PxPairFlag::eNOTIFY_TOUCH_FOUND
			  | PxPairFlag::eNOTIFY_CONTACT_POINTS
			  | PxPairFlag::eCONTACT_EVENT_POSE;
	return PxFilterFlag::eDEFAULT;
}

class ContactReportCallback: public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)	{ PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count)					{ PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) 
	{
		std::vector<PxContactPairPoint> contactPoints;

		PxTransform spherePose(PxIdentity);
		PxU32 nextPairIndex = 0xffffffff;

		PxContactPairExtraDataIterator iter(pairHeader.extraDataStream, pairHeader.extraDataStreamSize);
		bool hasItemSet = iter.nextItemSet();
		if (hasItemSet)
			nextPairIndex = iter.contactPairIndex;

		for(PxU32 i=0; i < nbPairs; i++)
		{
			//
			// Get the pose of the dynamic object at time of impact.
			//
			if (nextPairIndex == i)
			{
				if (pairHeader.actors[0]->is<PxRigidDynamic>())
					spherePose = iter.eventPose->globalPose[0];
				else
					spherePose = iter.eventPose->globalPose[1];

				gContactSphereActorPositions.push_back(spherePose.p);

				hasItemSet = iter.nextItemSet();
				if (hasItemSet)
					nextPairIndex = iter.contactPairIndex;
			}

			//
			// Get the contact points for the pair.
			//
			const PxContactPair& cPair = pairs[i];
			if (cPair.events & (PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_CCD))
			{
				PxU32 contactCount = cPair.contactCount;
				contactPoints.resize(contactCount);
				cPair.extractContacts(&contactPoints[0], contactCount);

				for(PxU32 j=0; j < contactCount; j++)
				{
					gContactPositions.push_back(contactPoints[j].position);
					gContactImpulses.push_back(contactPoints[j].impulse);
				}
			}
		}
	}
};

ContactReportCallback gContactReportCallback;

static void initScene()
{
	//
	// Create a static triangle mesh
	//

	PxVec3 vertices[] = {	PxVec3(-8.0f, 0.0f, -3.0f),
							PxVec3(-8.0f, 0.0f, 3.0f),
							PxVec3(0.0f, 0.0f, 3.0f),
							PxVec3(0.0f, 0.0f, -3.0f),
							PxVec3(-8.0f, 10.0f, -3.0f),
							PxVec3(-8.0f, 10.0f, 3.0f),
							PxVec3(0.0f, 10.0f, 3.0f),
							PxVec3(0.0f, 10.0f, -3.0f),
						};

	PxU32 vertexCount = sizeof(vertices) / sizeof(vertices[0]);

	PxU32 triangleIndices[] = {	0, 1, 2,
								0, 2, 3,
								0, 5, 1,
								0, 4, 5,
								4, 6, 5,
								4, 7, 6
							};
	PxU32 triangleCount = (sizeof(triangleIndices) / sizeof(triangleIndices[0])) / 3;

	PxTriangleMeshDesc triangleMeshDesc;
	triangleMeshDesc.points.count = vertexCount;
	triangleMeshDesc.points.data = vertices;
	triangleMeshDesc.points.stride = sizeof(PxVec3);
	triangleMeshDesc.triangles.count = triangleCount;
	triangleMeshDesc.triangles.data = triangleIndices;
	triangleMeshDesc.triangles.stride = 3 * sizeof(PxU32);

	PxTolerancesScale tolerances;
	const PxCookingParams params(tolerances);
	gTriangleMesh = PxCreateTriangleMesh(params, triangleMeshDesc, gPhysics->getPhysicsInsertionCallback());

	if (!gTriangleMesh)
		return;

	gTriangleMeshActor = gPhysics->createRigidStatic(PxTransform(PxVec3(0.0f, 1.0f, 0.0f), PxQuat(PxHalfPi / 60.0f, PxVec3(0.0f, 1.0f, 0.0f))));

	if (!gTriangleMeshActor)
		return;

	PxTriangleMeshGeometry triGeom(gTriangleMesh);
	PxShape* triangleMeshShape = PxRigidActorExt::createExclusiveShape(*gTriangleMeshActor, triGeom, *gMaterial);

	if (!triangleMeshShape)
		return;

	gScene->addActor(*gTriangleMeshActor);

	
	//
	// Create a fast moving sphere that will hit and bounce off the static triangle mesh 3 times
	// in one simulation step.
	//

	PxTransform spherePose(PxVec3(0.0f, 5.0f, 1.0f));
	gContactSphereActorPositions.push_back(spherePose.p);
	gSphereActor = gPhysics->createRigidDynamic(spherePose);
	gSphereActor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);

	if (!gSphereActor)
		return;

	PxSphereGeometry sphereGeom(1.0f);
	PxShape* sphereShape = PxRigidActorExt::createExclusiveShape(*gSphereActor, sphereGeom, *gMaterial);

	if (!sphereShape)
		return;

	PxRigidBodyExt::updateMassAndInertia(*gSphereActor, 1.0f);

	PxReal velMagn = 900.0f;
	PxVec3 vel = PxVec3(-1.0f, -1.0f, 0.0f);
	vel.normalize();
	vel *= velMagn;
	gSphereActor->setLinearVelocity(vel);

	gScene->addActor(*gSphereActor);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.gravity = PxVec3(0, 0, 0);
	sceneDesc.filterShader	= contactReportFilterShader;			
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDesc.ccdMaxPasses = 4;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 1.0f);

	initScene();
}

void stepPhysics(bool /*interactive*/)
{
	if (!gSimStepCount)
	{
		gScene->simulate(1.0f/60.0f);
		gScene->fetchResults(true);
		printf("%d contact points\n", PxU32(gContactPositions.size()));

		if (gSphereActor)
			gContactSphereActorPositions.push_back(gSphereActor->getGlobalPose().p);

		gSimStepCount = 1;
	}
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gSphereActor);
	PX_RELEASE(gTriangleMeshActor);
	PX_RELEASE(gTriangleMesh);

	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetContactReportCCD done.\n");
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	initPhysics(false);

	stepPhysics(false);

	cleanupPhysics(false);
#endif

	return 0;
}


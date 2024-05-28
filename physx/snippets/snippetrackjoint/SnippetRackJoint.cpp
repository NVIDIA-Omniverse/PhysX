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
// This snippet illustrates simple use of rack & pinion joints
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

static PxRigidDynamic* createGearWithBoxes(PxPhysics& sdk, const PxBoxGeometry& boxGeom, const PxTransform& transform, PxMaterial& material, int nbShapes)
{
	PxRigidDynamic* actor = sdk.createRigidDynamic(transform);

	PxMat33 m(PxIdentity);

	for(int i=0;i<nbShapes;i++)
	{
		const float coeff = float(i)/float(nbShapes);
		const float angle = PxPi * 0.5f * coeff;

		PxShape* shape = sdk.createShape(boxGeom, material, true);

		const PxReal cos = cosf(angle);
		const PxReal sin = sinf(angle);

		m[0][0] = m[1][1] = cos;
		m[0][1] = sin;
		m[1][0] = -sin;

		PxTransform localPose;
		localPose.p = PxVec3(0.0f);
		localPose.q = PxQuat(m);

		shape->setLocalPose(localPose);

		actor->attachShape(*shape);
	}
	PxRigidBodyExt::updateMassAndInertia(*actor, 1.0f);

	return actor;
}

static PxRigidDynamic* createRackWithBoxes(PxPhysics& sdk, const PxTransform& transform, PxMaterial& material, int nbTeeth, float rackLength)
{
	PxRigidDynamic* actor = sdk.createRigidDynamic(transform);

	{
		const PxBoxGeometry boxGeom(rackLength*0.5f, 0.25f, 0.25f);
		PxShape* shape = sdk.createShape(boxGeom, material, true);
		actor->attachShape(*shape);
	}

	PxMat33 m(PxIdentity);
	const float angle = PxPi * 0.25f;
	const PxReal cos = cosf(angle);
	const PxReal sin = sinf(angle);
	m[0][0] = m[1][1] = cos;
	m[0][1] = sin;
	m[1][0] = -sin;

	PxTransform localPose;
	localPose.p = PxVec3(0.0f);
	localPose.q = PxQuat(m);

	const float offset = rackLength / float(nbTeeth);
	localPose.p.x = (offset - rackLength)*0.5f;

	for(int i=0;i<nbTeeth;i++)
	{
		const PxBoxGeometry boxGeom(0.75f, 0.75f, 0.25f);
		PxShape* shape = sdk.createShape(boxGeom, material, true);
		shape->setLocalPose(localPose);

		actor->attachShape(*shape);

		localPose.p.x += offset;
	}

	PxRigidBodyExt::updateMassAndInertia(*actor, 1.0f);

	return actor;
}

static PxRevoluteJoint* gHinge0 = NULL;

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

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

	const float velocityTarget = 0.5f;
	const float radius = 3.0f;
	const float rackLength = 3.0f*10.0f;

	const int nbPinionTeeth = int(radius)*4;	// 'radius' teeth for PI/2
	const int nbRackTeeth = 5*3;

	const PxBoxGeometry boxGeom0(radius, radius, 0.5f);
	const PxVec3 boxPos0(0.0f, 10.0f, 0.0f);
	const PxVec3 boxPos1(0.0f, 10.0f+radius+1.5f, 0.0f);

	PxRigidDynamic* actor0 = createGearWithBoxes(*gPhysics, boxGeom0, PxTransform(boxPos0), *gMaterial, int(radius));
	gScene->addActor(*actor0);
	PxRigidDynamic*	actor1 = createRackWithBoxes(*gPhysics, PxTransform(boxPos1), *gMaterial, nbRackTeeth, rackLength);
	gScene->addActor(*actor1);

	const PxQuat x2z = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), PxVec3(0.0f, 0.0f, 1.0f));

	PxRevoluteJoint* hinge = PxRevoluteJointCreate(*gPhysics, NULL, PxTransform(boxPos0, x2z), actor0, PxTransform(PxVec3(0.0f), x2z));
	PxPrismaticJoint* prismatic = PxPrismaticJointCreate(*gPhysics, NULL, PxTransform(boxPos1), actor1, PxTransform(PxVec3(0.0f)));

	if(1)
	{
		hinge->setDriveVelocity(velocityTarget);
		hinge->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
		gHinge0 = hinge;
	}

	PxRackAndPinionJoint* rackJoint = PxRackAndPinionJointCreate(*gPhysics, actor0, PxTransform(PxVec3(0.0f), x2z), actor1, PxTransform(PxVec3(0.0f)));

	rackJoint->setJoints(hinge, prismatic);

	rackJoint->setData(nbRackTeeth, nbPinionTeeth, rackLength);
}

void stepPhysics(bool /*interactive*/)
{
	if(gHinge0)
	{
		static float globalTime = 0.0f;
		globalTime += 1.0f/60.0f;
		const float velocityTarget = cosf(globalTime)*3.0f;
		gHinge0->setDriveVelocity(velocityTarget);
	}

	gScene->simulate(1.0f/60.0f);
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
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetRackJoint done.\n");
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

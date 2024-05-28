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

// ***************************************************************************************
// This snippet demonstrates the use of a fixed tendon to mirror articulation joint angles
// ***************************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

static PxDefaultAllocator						gAllocator;
static PxDefaultErrorCallback					gErrorCallback;
static PxFoundation*							gFoundation		= NULL;
static PxPhysics*								gPhysics		= NULL;
static PxDefaultCpuDispatcher*					gDispatcher		= NULL;
static PxScene*									gScene			= NULL;
static PxMaterial*								gMaterial		= NULL;
static PxPvd*									gPvd			= NULL;
static PxArticulationReducedCoordinate*			gArticulation	= NULL;
static PxArticulationJointReducedCoordinate*	gDriveJoint		= NULL;
static const PxReal								gGravity		= 9.81f;
static PxReal									gDriveTargetPos = 0.0f;

static void createArticulation()
{
	gArticulation->setArticulationFlags(PxArticulationFlag::eFIX_BASE);
	gArticulation->setSolverIterationCounts(10, 1);

	// link geometry and density:
	const PxVec3 halfLengths(0.50f, 0.05f, 0.05f);
	const PxBoxGeometry linkGeom = PxBoxGeometry(halfLengths);
	const PxReal density = 1000.0f;

	//Create links
	PxTransform pose = PxTransform(PxIdentity);
	pose.p.y = 3.0f;
	pose.p.x -= 2.0f * halfLengths.x;
	PxArticulationLink* parent = NULL;

	const PxU32 numLinks = 3;
	for(PxU32 j = 0; j < numLinks; ++j)
	{
		pose.p.x += 2.0f * halfLengths.x;
		parent = gArticulation->createLink(parent, pose);
		PxRigidActorExt::createExclusiveShape(*parent, linkGeom, *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*parent, density);
		PxArticulationJointReducedCoordinate* joint = parent->getInboundJoint();
		if(joint)
		{
			PxVec3 parentOffset(halfLengths.x, 0.0f, 0.0f);
			PxVec3 childOffset(-halfLengths.x, 0.0f, 0.0f);
			joint->setParentPose(PxTransform(parentOffset, PxQuat(PxIdentity)));
			joint->setChildPose(PxTransform(childOffset, PxQuat(PxIdentity)));
			joint->setJointType(PxArticulationJointType::eREVOLUTE);
			joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		}
	}

	// tendon and drive stiffness sizing
	// assuming all links extend horizontally, size to allow for two degrees
	// deviation due to gravity
	const PxReal linkMass = parent->getMass();
	const PxReal deflectionAngle = 2.0f * PxPi / 180.0f;  // two degrees
	// moment arm of first link is one half-length, for second it is three half-lengths
	const PxReal gravityTorque = gGravity * linkMass * (halfLengths.x + 3.0f * halfLengths.x);
	const PxReal driveStiffness = gravityTorque / deflectionAngle;
	const PxReal driveDamping = 0.2f * driveStiffness;
	// same idea for the tendon, but it has to support only a single link
	const PxReal tendonStiffness = gGravity * linkMass * halfLengths.x / deflectionAngle;
	const PxReal tendonDamping = 0.2f * tendonStiffness;

	// compute drive target angle that compensates, statically, for the first fixed tendon joint
	// torque acting on the drive joint:
	const PxReal targetAngle = PxPiDivFour;
	const PxReal tendonTorque = targetAngle * tendonStiffness;
	gDriveTargetPos = targetAngle + tendonTorque / driveStiffness;

	// setup fixed tendon
	PxArticulationLink* links[numLinks];
	gArticulation->getLinks(links, numLinks, 0u);
	PxArticulationFixedTendon* tendon = gArticulation->createFixedTendon();
	tendon->setLimitStiffness(0.0f);
	tendon->setDamping(tendonDamping);
	tendon->setStiffness(tendonStiffness);
	tendon->setRestLength(0.f);
	tendon->setOffset(0.f);
	PxArticulationTendonJoint* tendonParentJoint = NULL;
	// root fixed-tendon joint - does not contribute to length so its coefficient and axis are irrelevant
	// but its parent link experiences all tendon-joint reaction forces
	tendonParentJoint = tendon->createTendonJoint(tendonParentJoint, PxArticulationAxis::eSWING2, 42.0f, 1.f/42.f, links[0]);
	// drive joint
	tendonParentJoint = tendon->createTendonJoint(tendonParentJoint, PxArticulationAxis::eSWING2, 1.0f, 1.f, links[1]);
	// second joint that is driven only by the tendon - negative coefficient to mirror angle of drive joint
	tendonParentJoint = tendon->createTendonJoint(tendonParentJoint, PxArticulationAxis::eSWING2, -1.0f, -1.0f, links[2]);

	// configure joint drive
	gDriveJoint = links[1]->getInboundJoint();
	PxArticulationDrive driveConfiguration;
	driveConfiguration.damping = driveDamping;
	driveConfiguration.stiffness = driveStiffness;
	driveConfiguration.maxForce = PX_MAX_F32;
	driveConfiguration.driveType = PxArticulationDriveType::eFORCE;
	gDriveJoint->setDriveParams(PxArticulationAxis::eSWING2, driveConfiguration);
	gDriveJoint->setDriveVelocity(PxArticulationAxis::eSWING2, 0.0f);
	gDriveJoint->setDriveTarget(PxArticulationAxis::eSWING2, 0.0f);

	// add articulation to scene:
	gScene->addArticulation(*gArticulation);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -gGravity, 0.0f);
	
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	sceneDesc.solverType = PxSolverType::eTGS;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);
	gArticulation = gPhysics->createArticulationReducedCoordinate();

	createArticulation();
}

void stepPhysics(bool /*interactive*/)
{
	static bool dir = false;
	static PxReal time = 0.0f;
	const PxReal switchTime = 3.0f;
	const PxReal dt = 1.0f / 60.f;

	time += dt;
	if(time > switchTime)
	{
		if(dir)
		{
			gDriveJoint->setDriveTarget(PxArticulationAxis::eSWING2, 0.0f);
		}
		else
		{
			gDriveJoint->setDriveTarget(PxArticulationAxis::eSWING2, gDriveTargetPos);
		}
		dir = !dir;
		time = 0.0f;
	}

	gScene->simulate(dt);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gArticulation);
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PxPvdTransport* transport = gPvd->getTransport();
	PX_RELEASE(gPvd);
	PX_RELEASE(transport);
	PxCloseExtensions();  
	PX_RELEASE(gFoundation);

	printf("SnippetFixedTendon done.\n");
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

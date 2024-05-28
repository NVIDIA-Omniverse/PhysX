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
// This snippet demonstrates the use of an articulation mimic joint.
// 
// Mimic joints couple two degrees of freedom of the same articulation instance using the 
// the rule qA + qB*gearRatio + offset = 0 with qA and qB demoting
// the joint position of two degrees of freedom A and B. The degrees of freedom A and B
// may be in different sub-trees of the articulation or in the same sub-tree.
// The parameters gearRatio and offset are constants of the mimic joint.
//
// The snippet creates an articulation with a fixed root link and two dynamic links that 
// are children of the fixed root such that each dynamic link has the root as its parent.
// The two joints in the articulation permit motion only along a single linear or angular axis.
// The rest poses of the two joints place the dynamic links symmetrically around the root
// along the PxArticulation::eX axis.
// To illustrate the effect of the mimic joint feature, one of the joints is driven with a stiff 
// drive that oscillates between an upper and lower joint position. This translates (or rotates)
// the corresponding dynamic link. The 2nd joint is coupled to the 1st with a mimic joint to ensure 
// that the motion of the 1st is mimicked by the 2nd. 
// 
// The parameter gDriveAxis determines the drive axis of the driven joint.
// The parameter gMimicAxis determines the axis on the complementary joint of the mimic
// joint.
// It is worthwhile experimenting with gDriveAxis and gMimicAxis to get an idea of how to 
// use mimic joints to simulate complex joints such as rack and pinion.
// It is also worthwhile experimenting with gMimicJointGearRatio and gMimicJointOffset 
// to illustrate the effect of each parameter.
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
static const PxReal								gGravity		= 9.81f;

//The drive target position is updated as follows: driveTargetPos += driveDirection*gDriveSpeed*dt.
//driveDirection swaps sign each time driveTargetPos falls outside the range (gDrivePositionMin, gDrivePositionMax).
static const PxReal								gDrivePositionMin = -1.0f;
static const PxReal								gDrivePositionMax = 1.0f;
static const PxReal								gDriveSpeed = 5.0f;
static PxArticulationJointReducedCoordinate*	gDriveJoint		= NULL;

//The parameters of the mimic joint.
static PxArticulationAxis::Enum					gDriveAxis		= PxArticulationAxis::eTWIST;
static PxArticulationAxis::Enum					gMimicAxis		= PxArticulationAxis::eX;
static const PxReal								gMimicJointGearRatio = 1.0f;
static const PxReal								gMimicJointOffset = 0.0f;


static PxArticulationReducedCoordinate* createArticulation(PxArticulationJointReducedCoordinate*& driveJoint)
{
	PxArticulationReducedCoordinate* articulation = gPhysics->createArticulationReducedCoordinate();
	articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
	articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
	articulation->setSolverIterationCounts(32, 1);
	articulation->setSleepThreshold(0.0f);

	PxArticulationLink* rootLink = articulation->createLink(NULL, PxTransform(PxIdentity));
	rootLink->setCfmScale(0.0f);
	rootLink->setLinearDamping(0.0f);
	rootLink->setAngularDamping(0.0f);
	PxShape* rootShape = gPhysics->createShape(PxSphereGeometry(0.5f), *gMaterial, true);
	rootLink->attachShape(*rootShape);
	rootShape->release();

	const PxVec3 linkBoxHalfExtent(0.5f, 0.5f, 0.5f);
	const PxTransform linkATransform(PxVec3(-5.0f, 0.0f, 0.0f));
	const PxTransform linkBTransform(PxVec3(5.0f, 0.0f, 0.0f));

	//The inbound joint of linkA will be driven with a stiff drive.
	PxArticulationLink* linkA =articulation->createLink(rootLink, PxTransform(PxIdentity));
	linkA->setCfmScale(0.0f);
	linkA->setLinearDamping(0.0f);
	linkA->setAngularDamping(0.0f);
	PxShape* linkAShape = gPhysics->createShape(PxBoxGeometry(linkBoxHalfExtent), *gMaterial, true);
	linkA->attachShape(*linkAShape);
	linkAShape->release();
	PxArticulationJointReducedCoordinate* linkAJoint = linkA->getInboundJoint();
	switch(gDriveAxis)
	{
		case PxArticulationAxis::eX:
		case PxArticulationAxis::eY:
		case PxArticulationAxis::eZ:
			linkAJoint->setJointType(PxArticulationJointType::ePRISMATIC);
			break;
		case PxArticulationAxis::eSWING1:
		case PxArticulationAxis::eSWING2:
		case PxArticulationAxis::eTWIST:
			linkAJoint->setJointType(PxArticulationJointType::eREVOLUTE);
			break;
		case PxArticulationAxis::eCOUNT:
			printf("gDriveAxis must be a legal articulation axis \n");
			break;
	}
	linkAJoint->setMotion(gDriveAxis, PxArticulationMotion::eFREE);
	linkAJoint->setParentPose(linkATransform);
	linkAJoint->setChildPose(PxTransform(PxVec3(0.0f)));
	linkAJoint->setFrictionCoefficient(0.0f);

	//The inbound joint of linkB will be influenced only by the mimic joint.
	PxArticulationLink* linkB =articulation->createLink(rootLink, PxTransform(PxIdentity));
	linkB->setCfmScale(0.0f);
	linkB->setLinearDamping(0.0f);
	linkB->setAngularDamping(0.0f);
	PxShape* linkBShape = gPhysics->createShape(PxBoxGeometry(linkBoxHalfExtent), *gMaterial, true);
	linkB->attachShape(*linkBShape);
	linkBShape->release();
	PxArticulationJointReducedCoordinate* linkBJoint = linkB->getInboundJoint();
	switch(gMimicAxis)
	{
		case PxArticulationAxis::eX:
		case PxArticulationAxis::eY:
		case PxArticulationAxis::eZ:
			linkBJoint->setJointType(PxArticulationJointType::ePRISMATIC);
			break;
		case PxArticulationAxis::eSWING1:
		case PxArticulationAxis::eSWING2:
		case PxArticulationAxis::eTWIST:
			linkBJoint->setJointType(PxArticulationJointType::eREVOLUTE);
			break;
		case PxArticulationAxis::eCOUNT:
			printf("gMimicAxis must be a legal articulation axis \n");
			break;
	}
	linkBJoint->setMotion(gMimicAxis, PxArticulationMotion::eFREE);
	linkBJoint->setParentPose(linkBTransform);
	linkBJoint->setChildPose(PxTransform(PxVec3(0.0f)));
	linkBJoint->setFrictionCoefficient(0.0f);

	//Drive linkAJoint with a very stiff position drive so that the target position is achieved in 1 simulation step.
	driveJoint = linkAJoint;
	driveJoint->setDriveParams(gDriveAxis, PxArticulationDrive(1e10f, 0.0f, PX_MAX_F32));

	//Mimic the drive with a mimic joint such that linkBJoint will follow linkAJoint
	articulation->createMimicJoint(*linkAJoint, gDriveAxis, *linkBJoint, gMimicAxis, gMimicJointGearRatio, gMimicJointOffset);

	return articulation;
}

void initPhysics(bool /*interactive*/)
{
	// Create a PxFoundation instance
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	// Create a PxPhysics instance
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	//Create a PxMaterial instance
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);

	//Create a PxScene instance
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -gGravity, 0.0f);
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	sceneDesc.solverType = PxSolverType::eTGS;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	//Create an articulation with a joint drive and a mimic joint to replicate the drive 
	gArticulation = createArticulation(gDriveJoint);
	gScene->addArticulation(*gArticulation);
}

void stepPhysics(bool /*interactive*/)
{
	static const PxReal dt = 0.016777f;

	static PxReal driveDirection = 1.0f;
	static PxReal drivePosition = 0.0f;

	//If we exceed the upper or lower limit of the drive then clamp the drive
	//to the limit and reverse direction.
	drivePosition += gDriveSpeed * driveDirection * dt;
	if(drivePosition > gDrivePositionMax)
	{
		drivePosition = gDrivePositionMax;
		driveDirection = -driveDirection;
	}
	else if(drivePosition < gDrivePositionMin)
	{
		drivePosition = gDrivePositionMin;
		driveDirection = -driveDirection;
	}

	//Set the updated drive target position.
	gDriveJoint->setDriveTarget(gDriveAxis, drivePosition);

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

	printf("SnippetMimicJoint done.\n");
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

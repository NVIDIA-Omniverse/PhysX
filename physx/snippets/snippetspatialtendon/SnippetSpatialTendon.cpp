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

// *****************************************************************************************
// This snippet demonstrates the use of a spatial tendon to actuate a symmetric articulation
// *****************************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

static PxDefaultAllocator						gAllocator;
static PxDefaultErrorCallback					gErrorCallback;
static PxFoundation*							gFoundation = NULL;
static PxPhysics*								gPhysics = NULL;
static PxDefaultCpuDispatcher*					gDispatcher = NULL;
static PxScene*									gScene = NULL;
static PxMaterial*								gMaterial = NULL;
static PxPvd*									gPvd = NULL;
static PxArticulationReducedCoordinate*			gArticulations[2] = { NULL };
static const PxReal								gGravity = 9.81f;
static const PxReal								gLinkHalfLength = 0.5f;

PxRigidStatic** getAttachments()
{
	static PxRigidStatic* attachments[6] = { NULL };
	return attachments;
}

static void createSpatialTendonArticulation(PxArticulationReducedCoordinate* articulation,
											PxRigidStatic** attachmentRigidStatics,
											const PxVec3 offset)
{
	// link geometry and density:
	const PxVec3 linkExtents(gLinkHalfLength, 0.05f, 0.05f);
	const PxBoxGeometry linkGeom = PxBoxGeometry(linkExtents);
	const PxReal density = 1000.0f;

	articulation->setArticulationFlags(PxArticulationFlag::eFIX_BASE);
	articulation->setSolverIterationCounts(10, 1);

	PxTransform pose = PxTransform(offset, PxQuat(PxIdentity));
	pose.p.y += 3.0f;
	PxArticulationLink* baseLink = articulation->createLink(NULL, pose);
	PxRigidActorExt::createExclusiveShape(*baseLink, linkGeom, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*baseLink, density);

	pose.p.x -= linkExtents.x * 2.0f;
	PxArticulationLink* leftLink = articulation->createLink(baseLink, pose);
	PxRigidActorExt::createExclusiveShape(*leftLink, linkGeom, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*leftLink, density);

	pose.p.x += linkExtents.x * 4.0f;
	PxArticulationLink* rightLink = articulation->createLink(baseLink, pose);
	PxRigidActorExt::createExclusiveShape(*rightLink, linkGeom, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*rightLink, density);

	// setup revolute joints:
	{
		PxArticulationJointReducedCoordinate* joint = leftLink->getInboundJoint();
		if(joint)
		{
			PxVec3 parentOffset(-linkExtents.x, 0.0f, 0.0f);
			PxVec3 childOffset(linkExtents.x, 0.0f, 0.0f);
			joint->setParentPose(PxTransform(parentOffset, PxQuat(PxIdentity)));
			joint->setChildPose(PxTransform(childOffset, PxQuat(PxIdentity)));
			joint->setJointType(PxArticulationJointType::eREVOLUTE);
			joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		}
	}

	{
		PxArticulationJointReducedCoordinate* joint = rightLink->getInboundJoint();
		if(joint)
		{
			PxVec3 parentOffset(linkExtents.x, 0.0f, 0.0f);
			PxVec3 childOffset(-linkExtents.x, 0.0f, 0.0f);
			joint->setParentPose(PxTransform(parentOffset, PxQuat(PxIdentity)));
			joint->setChildPose(PxTransform(childOffset, PxQuat(PxIdentity)));
			joint->setJointType(PxArticulationJointType::eREVOLUTE);
			joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		}

	}

	// tendon stiffness sizing:
	// scale articulation geometry:
	// r: root attachment on left moving link
	// a: attachment on fixed-base link
	// l: leaf attachment on right moving link
	// o: revolute joint
	//               a
	//
	// ----r----o---------o----l----
	// The root and leaf attachment are at the center of mass of the moving links.
	// The attachment on the fixed-base link is a link halfLength above the link.

	// Therefore, the (rest)length of the tendon r-a-l when both joints are at zero is:
	// 2 * sqrt((2 * gLinkHalfLength)^2 + gLinkHalfLength^2) = 2 * gLinkHalfLength * sqrt(5)
	const PxReal restLength = 2.0f * gLinkHalfLength * PxSqrt(5.0f);

	// The goal is to have the revolute joints deviate just a few degrees from the horizontal
	// and we compute the length of the tendon at that angle:
	const PxReal deviationAngle = 3.0f * PxPi / 180.0f;
	// Distances from the base-link attachment to the root and leaf attachments
	const PxReal verticalDistance = gLinkHalfLength * (1.0f + PxSin(deviationAngle));
	const PxReal horizontalDistance = gLinkHalfLength * (1.0f + PxCos(deviationAngle));
	const PxReal deviatedLength = 2.0f * PxSqrt(verticalDistance * verticalDistance + horizontalDistance * horizontalDistance);

	// At rest, the force on the leaf attachment is (deviatedLength - restLength) * tendonStiffness.
	// This force needs to be equal to the gravity force acting on the link. An equal and opposing
	// (in the direction of the tendon) force acts on the root link and will hold that link up.
	// In order to calculate the tendon stiffness that produces that force, we consider the forces
	// and attachment geometry at zero joint angles.
	const PxReal linkMass = baseLink->getMass();
	const PxReal gravityForce = gGravity * linkMass;
	// Project onto tendon at rest length / with joints at zero angle
	const PxReal tendonForce = gravityForce * PxSqrt(5.0f); // gravityForce * 0.5f * restLength / halfLength
	// and compute stiffness to get tendon force at deviated length to hold the link:
	const PxReal tendonStiffness = tendonForce / (deviatedLength - restLength);
	const PxReal tendonDamping = 0.3f * tendonStiffness;

	PxArticulationSpatialTendon* tendon = articulation->createSpatialTendon();
	tendon->setStiffness(tendonStiffness);
	tendon->setDamping(tendonDamping);
	PxArticulationAttachment* rootAttachment = tendon->createAttachment(NULL, 1.0f, PxVec3(0.0f), leftLink);
	PxArticulationAttachment* baseAttachment = tendon->createAttachment(rootAttachment, 1.0f, PxVec3(0.0f, gLinkHalfLength, 0.0f), baseLink);
	PxArticulationAttachment* leafAttachment = tendon->createAttachment(baseAttachment, 1.0f, PxVec3(0.0f), rightLink);
	leafAttachment->setRestLength(restLength);

	// create attachment render shapes
	attachmentRigidStatics[0] = gPhysics->createRigidStatic(baseLink->getGlobalPose() * PxTransform(PxVec3(0.0f, gLinkHalfLength, 0.0f), PxQuat(PxIdentity)));
	attachmentRigidStatics[1] = gPhysics->createRigidStatic(leftLink->getGlobalPose());
	attachmentRigidStatics[2] = gPhysics->createRigidStatic(rightLink->getGlobalPose());

	PxSphereGeometry attachmentGeom(linkExtents.y * 1.5f); // slightly bigger than links to see the attachment on the moving links
	PxShape* attachmentShape = gPhysics->createShape(attachmentGeom, *gMaterial, false, PxShapeFlags(0));
	for(PxU32 i = 0; i < 3; ++i)
	{
		PxRigidStatic* attachment = attachmentRigidStatics[i];
		attachment->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, true); // attachments are viz only
		attachment->attachShape(*attachmentShape);
	}

	// add articulation to scene:
	gScene->addArticulation(*articulation);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -gGravity, 0.0f);

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	sceneDesc.solverType = PxSolverType::eTGS;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);
	gArticulations[0] = gPhysics->createArticulationReducedCoordinate();
	createSpatialTendonArticulation(gArticulations[0], getAttachments(), PxVec3(0.0f));
	gArticulations[1] = gPhysics->createArticulationReducedCoordinate();
	createSpatialTendonArticulation(gArticulations[1], &getAttachments()[3], PxVec3(0.0f, 0.0f, 2.0f));
}

void stepPhysics(bool /*interactive*/)
{
	const PxReal dt = 1.0f / 60.f;
	static PxReal time = 0.0f;
	// update articulation that actuates via tendon-length offset:
	{
		const PxReal amplitude = 0.3f;
		// move at 0.25 Hz, and offset sinusoid by an amplitude
		const PxReal offset = amplitude * (1.0f + PxSin(time * PxTwoPi * 0.25f - PxPiDivTwo));
		PxArticulationSpatialTendon* tendon = NULL;
		gArticulations[0]->getSpatialTendons(&tendon, 1, 0);
		tendon->setOffset(offset);
		PxArticulationLink* links[3];
		gArticulations[0]->getLinks(links, 3, 0);
		getAttachments()[1]->setGlobalPose(links[1]->getGlobalPose());
		getAttachments()[2]->setGlobalPose(links[2]->getGlobalPose());
	}
	// update articulation that actuates via base link attachment relative pose
	{
		const PxReal amplitude = 0.2f;
		// move at 0.25 Hz, and offset sinusoid by an amplitude
		const PxReal offset = gLinkHalfLength + amplitude * (1.0f + PxSin(time * PxTwoPi * 0.25f - PxPiDivTwo));
		PxArticulationSpatialTendon* tendon = NULL;
		gArticulations[1]->getSpatialTendons(&tendon, 1, 0);
		PxArticulationAttachment* baseAttachment = NULL;
		tendon->getAttachments(&baseAttachment, 1, 1);
		baseAttachment->setRelativeOffset(PxVec3(0.f, offset, 0.f));
		gArticulations[1]->wakeUp();  // wake up articulation (relative offset setting does not wake)
		PxArticulationLink* links[3];
		gArticulations[1]->getLinks(links, 3, 0);
		PxTransform attachmentPose = links[0]->getGlobalPose();
		attachmentPose.p.y += offset;
		getAttachments()[3]->setGlobalPose(attachmentPose);
		getAttachments()[4]->setGlobalPose(links[1]->getGlobalPose());
		getAttachments()[5]->setGlobalPose(links[2]->getGlobalPose());
	}

	gScene->simulate(dt);
	gScene->fetchResults(true);
	time += dt;
}

void cleanupPhysics(bool /*interactive*/)
{
	gArticulations[0]->release();
	gArticulations[1]->release();
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PxCloseExtensions();
	PX_RELEASE(gFoundation);

	printf("SnippetSpatialTendon done.\n");
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

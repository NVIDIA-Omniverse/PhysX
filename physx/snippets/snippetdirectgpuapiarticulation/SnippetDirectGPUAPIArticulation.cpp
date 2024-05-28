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
// This snippet illustrates the use of the Direct GPU API for articulations.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContext.h"
#include "../snippetutils/SnippetUtils.h"

using namespace physx;

static PxDefaultAllocator						gAllocator;
static PxDefaultErrorCallback					gErrorCallback;
static PxFoundation*							gFoundation			= NULL;
static PxPhysics*								gPhysics			= NULL;
static PxDefaultCpuDispatcher*					gDispatcher			= NULL;
static PxScene*									gScene				= NULL;
static PxMaterial*								gMaterial			= NULL;
static PxArticulationReducedCoordinate*			gArticulation		= NULL;
static PxArticulationJointReducedCoordinate*	gDriveJoint			= NULL;
static PxCudaContextManager*					gCudaContextManager	= NULL;

static const PxVec3	gGravity(0.0f, -9.81f, 0.0f);
static const PxU32	gNbIterPos					= 32;
static const PxReal	gContactOffset 				= 0.2f;
static const PxReal	gLinearDamping				= 0.2f;
static const PxReal	gAngularDamping				= 0.2f;
static const PxReal	gMaxLinearVelocity			= 100.0f;
static const PxReal	gMaxAngularVelocity			= 20.0f;
static const PxReal	gDriveStiffness				= 100000.f;
static const PxReal	gDriveDamping				= 0.0f;

static PxArray<PxU32> gRBIndices;
static PxArray<PxGeometryHolder> gRBGeometries;
static PxArray<PxTransform> gRBPoses;

static PxArray<PxU32> gArtiIndices;
static PxArray<PxGeometryHolder> gLinkGeometries;
static PxArray<PxTransform> gLinkPoses;

static PxArray<PxReal> gDriveValue;

static CUdeviceptr gRBIndicesD;
static CUdeviceptr gRBPosesD;
static CUdeviceptr gLinkPosesD;

static CUdeviceptr gArtiIndicesD;
static CUdeviceptr gDriveD;

PxU32 getRBCount()
{
	return gRBIndices.size();
}

PxU32 getLinkCount()
{
	return gLinkPoses.size();
}

const PxGeometryHolder* getRBGeometries()
{
	return gRBGeometries.empty() ? NULL : &gRBGeometries[0];
}

const PxGeometryHolder* getLinkGeometries()
{
	return gLinkGeometries.empty() ? NULL : &gLinkGeometries[0];
}

const PxTransform* getRBPoses()
{
	return gRBPoses.empty() ? NULL : &gRBPoses[0];
}

const PxTransform* getLinkPoses()
{
	return gLinkPoses.empty() ? NULL : &gLinkPoses[0];
}

static PxFilterFlags scissorFilter(	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
									PxFilterObjectAttributes attributes1, PxFilterData filterData1,
									PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);
	if (filterData0.word2 != 0 && filterData0.word2 == filterData1.word2)
		return PxFilterFlag::eKILL;
	pairFlags |= PxPairFlag::eCONTACT_DEFAULT;
	return PxFilterFlag::eDEFAULT;
}

static void createScissorLift()
{
	const PxReal runnerLength = 2.f;
	const PxReal placementDistance = 1.8f;
	const PxReal cosAng = (placementDistance) / (runnerLength);
	const PxReal angle = PxAcos(cosAng);
	const PxReal sinAng = PxSin(angle);
	const PxQuat leftRot(-angle, PxVec3(1.f, 0.f, 0.f));
	const PxQuat rightRot(angle, PxVec3(1.f, 0.f, 0.f));

	// create base...
	PxArticulationLink* base = gArticulation->createLink(NULL, PxTransform(PxVec3(0.f, 0.25f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*base, PxBoxGeometry(0.5f, 0.25f, 1.5f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*base, 3.f);

	// now create the slider and fixed joints...
	gArticulation->setSolverIterationCounts(gNbIterPos);

	PxArticulationLink* leftRoot = gArticulation->createLink(base, PxTransform(PxVec3(0.f, 0.55f, -0.9f)));
	PxRigidActorExt::createExclusiveShape(*leftRoot, PxBoxGeometry(0.5f, 0.05f, 0.05f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*leftRoot, 1.f);

	PxArticulationLink* rightRoot = gArticulation->createLink(base, PxTransform(PxVec3(0.f, 0.55f, 0.9f)));
	PxRigidActorExt::createExclusiveShape(*rightRoot, PxBoxGeometry(0.5f, 0.05f, 0.05f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*rightRoot, 1.f);

	PxArticulationJointReducedCoordinate* joint = leftRoot->getInboundJoint();
	joint->setJointType(PxArticulationJointType::eFIX);
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.25f, -0.9f)));
	joint->setChildPose(PxTransform(PxVec3(0.f, -0.05f, 0.f)));

	// set up the drive joint...
	gDriveJoint = rightRoot->getInboundJoint();
	gDriveJoint->setJointType(PxArticulationJointType::ePRISMATIC);
	gDriveJoint->setMotion(PxArticulationAxis::eZ, PxArticulationMotion::eLIMITED);
	gDriveJoint->setLimitParams(PxArticulationAxis::eZ, PxArticulationLimit(-1.4f, 0.2f));
	gDriveJoint->setDriveParams(PxArticulationAxis::eZ, PxArticulationDrive(gDriveStiffness, gDriveDamping, PX_MAX_F32));
	gDriveJoint->setParentPose(PxTransform(PxVec3(0.f, 0.25f, 0.9f)));
	gDriveJoint->setChildPose(PxTransform(PxVec3(0.f, -0.05f, 0.f)));

	const PxU32 linkHeight = 3;
	PxArticulationLink* currFrontLeft = leftRoot, *currFrontRight = rightRoot;
	PxArticulationLink* currBackLeft = leftRoot, *currBackRight = rightRoot;

	// set up the scissor
	PxQuat frontRightParentRot(PxIdentity);
	PxQuat frontLeftParentRot(PxIdentity);
	PxQuat backRightParentRot(PxIdentity);
	PxQuat backLeftParentRot(PxIdentity);
	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		PxVec3 pos(0.5f, 0.55f + 0.1f*(1 + i), 0.f);
		PxArticulationLink* frontLeftLink = gArticulation->createLink(currFrontLeft, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), leftRot));
		PxRigidActorExt::createExclusiveShape(*frontLeftLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*frontLeftLink, 1.f);

		const PxVec3 frontLeftAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), -0.9f);

		joint = frontLeftLink->getInboundJoint();
		joint->setParentPose(PxTransform(currFrontLeft->getGlobalPose().transformInv(frontLeftAnchorLocation), frontLeftParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot));
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
        joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));
		frontLeftParentRot = leftRot;

		PxArticulationLink* frontRightLink = gArticulation->createLink(currFrontRight, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), rightRot));
		PxRigidActorExt::createExclusiveShape(*frontRightLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*frontRightLink, 1.f);

		const PxVec3 frontRightAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), 0.9f);

		joint = frontRightLink->getInboundJoint();
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setParentPose(PxTransform(currFrontRight->getGlobalPose().transformInv(frontRightAnchorLocation), frontRightParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot));
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));

		frontRightParentRot = rightRot;

        pos = PxVec3(-0.5f, 0.55f + 0.1f*(1 + i), 0.f);
        PxArticulationLink* backLeftLink = gArticulation->createLink(currBackLeft, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), leftRot));
        PxRigidActorExt::createExclusiveShape(*backLeftLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
        PxRigidBodyExt::updateMassAndInertia(*backLeftLink, 1.f);

        const PxVec3 backLeftAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), -0.9f);

        joint = backLeftLink->getInboundJoint();
        joint->setJointType(PxArticulationJointType::eREVOLUTE);
        joint->setParentPose(PxTransform(currBackLeft->getGlobalPose().transformInv(backLeftAnchorLocation), backLeftParentRot));
        joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot));
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
        joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));

        backLeftParentRot = leftRot;

		PxArticulationLink* backRightLink = gArticulation->createLink(currBackRight, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), rightRot));
		PxRigidActorExt::createExclusiveShape(*backRightLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*backRightLink, 1.f);

		const PxVec3 backRightAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), 0.9f);

		joint = backRightLink->getInboundJoint();
		joint->setParentPose(PxTransform(currBackRight->getGlobalPose().transformInv(backRightAnchorLocation), backRightParentRot));
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot));
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));

		backRightParentRot = rightRot;

		PxD6Joint* d6FrontJoint = PxD6JointCreate(*gPhysics, frontLeftLink, PxTransform(PxIdentity), frontRightLink, PxTransform(PxIdentity));

		d6FrontJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6FrontJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6FrontJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		PxD6Joint* d6BackJoint = PxD6JointCreate(*gPhysics, backLeftLink, PxTransform(PxIdentity), backRightLink, PxTransform(PxIdentity));

		d6BackJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6BackJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6BackJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currFrontLeft = frontRightLink;
		currFrontRight = frontLeftLink;
		currBackLeft = backRightLink;
		currBackRight = backLeftLink;
	}

	// set up the top
	PxArticulationLink* rightTop = gArticulation->createLink(currFrontRight, currFrontRight->getGlobalPose().transform(PxTransform(PxVec3(-0.5f, 0.f, 1.0f), frontRightParentRot)));
	PxRigidActorExt::createExclusiveShape(*rightTop, PxCapsuleGeometry(0.05f, 0.8f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*rightTop, 1.f);

	PxArticulationLink* leftTop = gArticulation->createLink(currFrontLeft, currFrontLeft->getGlobalPose().transform(PxTransform(PxVec3(-0.5f, 0.f, -1.0f), frontLeftParentRot)));
	PxRigidActorExt::createExclusiveShape(*leftTop, PxBoxGeometry(0.5f, 0.05f, 0.05f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*leftTop, 1.f);

	joint = leftTop->getInboundJoint();
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.f, -1.f), currFrontLeft->getGlobalPose().q.getConjugate()));
	joint->setChildPose(PxTransform(PxVec3(0.5f, 0.f, 0.f), leftTop->getGlobalPose().q.getConjugate()));
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);

	joint = rightTop->getInboundJoint();
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.f, 1.f), currFrontRight->getGlobalPose().q.getConjugate()));
	joint->setChildPose(PxTransform(PxVec3(0.5f, 0.f, 0.f), rightTop->getGlobalPose().q.getConjugate()));
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);

	PxD6Joint* d6joint = PxD6JointCreate(*gPhysics, currBackLeft, PxTransform(PxVec3(0.f, 0.f, -1.f)), leftTop, PxTransform(PxVec3(-0.5f, 0.f, 0.f)));

	d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	d6joint = PxD6JointCreate(*gPhysics, currBackRight, PxTransform(PxVec3(0.f, 0.f, 1.f)), rightTop, PxTransform(PxVec3(-0.5f, 0.f, 0.f)));

	d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	const PxTransform topPose(PxVec3(0.f, leftTop->getGlobalPose().p.y + 0.15f, 0.f));

	PxArticulationLink* top = gArticulation->createLink(leftTop, topPose);
	PxRigidActorExt::createExclusiveShape(*top, PxBoxGeometry(0.5f, 0.1f, 1.5f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*top, 1.f);

	joint = top->getInboundJoint();
	joint->setJointType(PxArticulationJointType::eFIX);
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.0f, 0.f)));
	joint->setChildPose(PxTransform(PxVec3(0.f, -0.15f, -0.9f)));

	gScene->addArticulation(*gArticulation);

	for (PxU32 i = 0; i < gArticulation->getNbLinks(); ++i)
	{
		PxArticulationLink* link;
		gArticulation->getLinks(&link, 1, i);

		link->setLinearDamping(gLinearDamping);
		link->setAngularDamping(gAngularDamping);

		link->setMaxAngularVelocity(gMaxAngularVelocity);
		link->setMaxLinearVelocity(gMaxLinearVelocity);

		if (link != top)
		{
			for (PxU32 b = 0; b < link->getNbShapes(); ++b)
			{
				PxShape* shape;
				link->getShapes(&shape, 1, b);

				shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));
			}
		}
	}

	// set up the box stack
	const PxVec3 halfExt(0.25f);
	const PxReal density(0.5f);

	PxBoxGeometry boxGeometryBox0(halfExt);
	PxTransform poseBox0(PxVec3(-0.25f, 5.f, 0.5f));
	PxRigidDynamic* box0 = gPhysics->createRigidDynamic(poseBox0);
	PxShape* shape0 = PxRigidActorExt::createExclusiveShape(*box0, boxGeometryBox0, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box0, density);
	gScene->addActor(*box0);

	gRBIndices.pushBack(box0->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox0);
	gRBPoses.pushBack(poseBox0);

	PxBoxGeometry boxGeometryBox1(halfExt);
	PxTransform poseBox1(PxVec3(0.25f, 5.f, 0.5f));
	PxRigidDynamic* box1 = gPhysics->createRigidDynamic(poseBox1);
	PxShape* shape1 = PxRigidActorExt::createExclusiveShape(*box1, boxGeometryBox1, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box1, density);
	gScene->addActor(*box1);

	gRBIndices.pushBack(box1->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox1);
	gRBPoses.pushBack(poseBox1);

	PxBoxGeometry boxGeometryBox2(halfExt);
	PxTransform poseBox2(PxVec3(-0.25f, 4.5f, 0.5f));
	PxRigidDynamic* box2 = gPhysics->createRigidDynamic(poseBox2);
	PxShape* shape2 = PxRigidActorExt::createExclusiveShape(*box2, boxGeometryBox2, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box2, density);
	gScene->addActor(*box2);

	gRBIndices.pushBack(box2->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox2);
	gRBPoses.pushBack(poseBox2);

	PxBoxGeometry boxGeometryBox3(halfExt);
	PxTransform poseBox3(PxVec3(0.25f, 4.5f, 0.5f));
	PxRigidDynamic* box3 = gPhysics->createRigidDynamic(poseBox3);
	PxShape* shape3 = PxRigidActorExt::createExclusiveShape(*box3, boxGeometryBox3, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box3, density);
	gScene->addActor(*box3);

	gRBIndices.pushBack(box3->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox3);
	gRBPoses.pushBack(poseBox3);

	PxBoxGeometry boxGeometryBox4(halfExt);
	PxTransform poseBox4(PxVec3(-0.25f, 5.f, 0.f));
	PxRigidDynamic* box4 = gPhysics->createRigidDynamic(poseBox4);
	PxShape* shape4 = PxRigidActorExt::createExclusiveShape(*box4, boxGeometryBox4, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box4, density);
	gScene->addActor(*box4);

	gRBIndices.pushBack(box4->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox4);
	gRBPoses.pushBack(poseBox4);

	PxBoxGeometry boxGeometryBox5(halfExt);
	PxTransform poseBox5(PxVec3(0.25f, 5.f, 0.f));
	PxRigidDynamic* box5 = gPhysics->createRigidDynamic(poseBox5);
	PxShape* shape5 = PxRigidActorExt::createExclusiveShape(*box5, boxGeometryBox5, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box5, density);
	gScene->addActor(*box5);

	gRBIndices.pushBack(box5->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox5);
	gRBPoses.pushBack(poseBox5);

	PxBoxGeometry boxGeometryBox6(halfExt);
	PxTransform poseBox6(PxVec3(-0.25f, 4.5f, 0.f));
	PxRigidDynamic* box6 = gPhysics->createRigidDynamic(poseBox6);
	PxShape* shape6 = PxRigidActorExt::createExclusiveShape(*box6, boxGeometryBox6, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box6, density);
	gScene->addActor(*box6);

	gRBIndices.pushBack(box6->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox6);
	gRBPoses.pushBack(poseBox6);

	PxBoxGeometry boxGeometryBox7(halfExt);
	PxTransform poseBox7(PxVec3(0.25f, 4.5f, 0.f));
	PxRigidDynamic* box7 = gPhysics->createRigidDynamic(poseBox7);
	PxShape* shape7 = PxRigidActorExt::createExclusiveShape(*box7, boxGeometryBox7, *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box7, density);
	gScene->addActor(*box7);

	gRBIndices.pushBack(box7->getGPUIndex());
	gRBGeometries.pushBack(boxGeometryBox7);
	gRBPoses.pushBack(poseBox7);

	shape0->setContactOffset(gContactOffset);
	shape1->setContactOffset(gContactOffset);
	shape2->setContactOffset(gContactOffset);
	shape3->setContactOffset(gContactOffset);
	shape4->setContactOffset(gContactOffset);
	shape5->setContactOffset(gContactOffset);
	shape6->setContactOffset(gContactOffset);
	shape7->setContactOffset(gContactOffset);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());

	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc);
	if (!gCudaContextManager || !gCudaContextManager->contextIsValid())
	{
		PX_RELEASE(gCudaContextManager);
		printf("Failed to initialize cuda context.\n");
		printf("The direct GPU API feature is only supported on the GPU.\n");
		return;
	}

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = gGravity;
	
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	sceneDesc.cudaContextManager = gCudaContextManager;

	// enable GPU simulation and direct GPU access
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_DIRECT_GPU_API;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;

	sceneDesc.solverType = PxSolverType::eTGS;
	sceneDesc.filterShader = scissorFilter;

	gScene = gPhysics->createScene(sceneDesc);
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	gArticulation = gPhysics->createArticulationReducedCoordinate();

	createScissorLift();

	// run one setp to initialize the direct GPU API
	gScene->simulate(1.0f / 60.f);
	gScene->fetchResults(true);

	// note that the link indexing in the direct GPU API does not follow the order
	// in which the link are created, link geometries must followed the low level indexing,
	// this is critical to render properly the scene.
	// first put placeholders in the arrays
	for (PxU32 i = 0; i < gArticulation->getNbLinks(); ++i)
	{
		gLinkGeometries.pushBack(PxGeometryHolder());
		gLinkPoses.pushBack(PxTransform(PxIdentity));
	}
	// then populate the array in the correct order
	for (PxU32 i = 0; i < gArticulation->getNbLinks(); ++i)
	{
		// getting the link
		PxArticulationLink* link;
		gArticulation->getLinks(&link, 1, i);
		// getting the link geometry
		PxShape* shape;
		link->getShapes(&shape, 1, 0);
		// adding the link geometry at the correct index
		gLinkGeometries[link->getLinkIndex()] = shape->getGeometry();
	}

	PxCudaContext* cudaContext = gCudaContextManager->getCudaContext();

	// prepare RBs indices
	cudaContext->memAlloc(&gRBIndicesD, getRBCount() * sizeof(PxU32));
	cudaContext->memcpyHtoD(gRBIndicesD, &gRBIndices[0], getRBCount() * sizeof(PxU32));
    // a buffer to read box pose for rendering
	cudaContext->memAlloc(&gRBPosesD, getRBCount() * sizeof(PxTransform));

	// prepare link indices
	const PxArticulationGPUIndex gpuIndex = gArticulation->getGPUIndex();
	PX_ASSERT(gpuIndex != 0xFFffFFff);
	gArtiIndices.pushBack(gpuIndex);
	cudaContext->memAlloc(&gArtiIndicesD, sizeof(PxU32));
	cudaContext->memcpyHtoD(gArtiIndicesD, &gArtiIndices[0], sizeof(PxU32));
	// a buffer to read link pose for rendering
	cudaContext->memAlloc(&gLinkPosesD, getLinkCount() * sizeof(PxTransform));

    // prepare joint target position
	gDriveValue.pushBack(0.0);
	const PxU32 nbDofs = gArticulation->getDofs();
	cudaContext->memAlloc(&gDriveD, sizeof(PxReal) * nbDofs);
	cudaContext->memcpyHtoD(gDriveD, &gDriveValue[0], sizeof(PxReal));
}

static bool gClosing = true;

void stepPhysics(bool /*interactive*/)
{
	if (gCudaContextManager)
	{
		PxCudaContext* cudaContext = gCudaContextManager->getCudaContext();

		const PxReal dt = 1.0f / 60.f;

		// get drive value
		gScene->getDirectGPUAPI().getArticulationData(
			(void*)gDriveD, (const PxArticulationGPUIndex*)(gArtiIndicesD),
			PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION, 1);
		cudaContext->memcpyDtoH(&gDriveValue[0], gDriveD, sizeof(PxReal));

		if (gClosing && gDriveValue[0] < -1.2f)
			gClosing = false;
		else if (!gClosing && gDriveValue[0] > 0.f)
			gClosing = true;

		if (gClosing)
			gDriveValue[0] -= dt*0.25f;
		else
			gDriveValue[0] += dt*0.25f;
		// set drive value
		cudaContext->memcpyHtoD(gDriveD, &gDriveValue[0], sizeof(PxReal));

		gScene->getDirectGPUAPI().setArticulationData(
			(void*)gDriveD, (const PxArticulationGPUIndex*)(gArtiIndicesD),
			PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION, 1);

		gScene->simulate(dt);
		gScene->fetchResults(true);

		// read current poses of boxes for rendering
		gScene->getDirectGPUAPI().getRigidDynamicData(reinterpret_cast<void*>(gRBPosesD),
			reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
			PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
			getRBCount());
		cudaContext->memcpyDtoH(&gRBPoses[0], gRBPosesD, getRBCount() * sizeof(PxTransform));

		// read current poses of the scissor lift for rendering
		gScene->getDirectGPUAPI().getArticulationData(reinterpret_cast<void*>(gLinkPosesD),
			reinterpret_cast<const PxArticulationGPUIndex*>(gArtiIndicesD),
			PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE,
			1);
		cudaContext->memcpyDtoH(&gLinkPoses[0], gLinkPosesD, getLinkCount() * sizeof(PxTransform));
	}
}

void cleanupPhysics(bool /*interactive*/)
{
	if (gCudaContextManager)
	{
		PxCudaContext* cudaContext = gCudaContextManager->getCudaContext();
		cudaContext->memFree(gArtiIndicesD);
		cudaContext->memFree(gDriveD);
		cudaContext->memFree(gRBPosesD);
		cudaContext->memFree(gRBIndicesD);
		cudaContext->memFree(gLinkPosesD);
	}
	gRBIndices.reset();
	gRBGeometries.reset();
	gRBPoses.reset();
	gArtiIndices.reset();
	gLinkGeometries.reset();
	gLinkPoses.reset();
	gDriveValue.reset();

	PX_RELEASE(gArticulation);
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gCudaContextManager);
	PX_RELEASE(gFoundation);

	printf("SnippetDirectGPUAPIArticulation done.\n");
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

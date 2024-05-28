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
// This snippet illustrates simple use of the physx vehicle sdk and demonstrates
// how to simulate multiple vehicles jointed together.  The snippet introduces 
// the simple example of a tractor pulling a trailer.  The wheels of the tractor
// respond to brake, throttle and steer.  The trailer, on the other hand, has no 
// engine or steering column and is only able to apply brake torques to the wheels.
// The snippet uses only parameters, states and components maintained by the PhysX Vehicle SDK.

// Vehicles are made of parameters, states and components.

// Parameters describe the configuration of a vehicle.  Examples are vehicle mass, wheel radius 
// and suspension stiffness.

// States describe the instantaneous dynamic state of a vehicle.  Examples are engine revs, wheel 
// yaw angle and tire slip angles.

// Components forward integrate the dynamic state of the vehicle, given the previous vehicle state 
// and the vehicle's parameterisation.
// Components update dynamic state by invoking reusable functions in a particular sequence. 
// An example component is a rigid body component that updates the linear and angular velocity of 
// the vehicle's rigid body given the instantaneous forces and torques of the suspension and tire 
// states.

// The pipeline of vehicle computation is a sequence of components that run in order.  For example, 
// one component might compute the plane under the wheel by performing a scene query against the 
// world geometry. The next component in the sequence might compute the suspension compression required 
// to place the wheel on the surface of the hit plane. Following this, another component might compute 
// the suspension force that arises from that compression.  The rigid body component, as discussed earlier, 
// can then forward integrate the rigid body's linear velocity using the suspension force.

// Custom combinations of parameter, state and component allow different behaviours to be simulated with 
// different simulation fidelities.  For example, a suspension component that implements a linear force 
// response with respect to its compression state could be replaced with one that imlements a non-linear
// response.  The replacement component would consume the same suspension compression state data and 
// would output the same suspension force data structure.  In this example, the change has been localised 
// to the  component that converts suspension compression to force and to the parameterisation that governs 
// that conversion.
// Another combination example could be the replacement of the tire component from a low fidelity model to 
// a high fidelty model such as Pacejka. The low and high fidelity components consume the same state data 
// (tire slip, load, friction) and  output the same state data  for the tire forces. Again, the 
// change has been localised to the component that converts slip angle to tire force and the 
// parameterisation that governs the conversion.

//The PhysX Vehicle SDK presents a maintained set of parameters, states and components.  The maintained 
//set of parameters, states and components may be combined on their own or combined with custom parameters, 
//states and components.

//This snippet breaks the vehicle into into three distinct models:
//1) a base vehicle model that describes the mechanical configuration of suspensions, tires, wheels and an 
//   associated rigid body.
//2) a drivetrain model that forwards input controls to wheel torques via a drivetrain model
//   that includes engine, clutch, differential and gears.
//3) a physx integration model that provides a representation of the vehicle in an associated physx scene.

// It is a good idea to record and playback with pvd (PhysX Visual Debugger).
// ****************************************************************************

#include <ctype.h>

#include "PxPhysicsAPI.h"
#include "../snippetvehicle2common/enginedrivetrain/EngineDrivetrain.h"
#include "../snippetvehicle2common/serialization/BaseSerialization.h"
#include "../snippetvehicle2common/serialization/EngineDrivetrainSerialization.h"
#include "../snippetvehicle2common/serialization/DirectDrivetrainSerialization.h"
#include "../snippetvehicle2common/SnippetVehicleHelpers.h"

#include "../snippetcommon/SnippetPVD.h"

using namespace physx;
using namespace physx::vehicle2;
using namespace snippetvehicle2;


//PhysX management class instances.

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;
PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;
PxMaterial*				gMaterial	= NULL;
PxPvd*                  gPvd        = NULL;

//The path to the vehicle json files to be loaded.
const char* gVehicleDataPath = NULL;

//The tractor with engine drivetrain.
//The trailer with direct drivetrain
//A joint connecting the two vehicles together.
EngineDriveVehicle gTractor;
DirectDriveVehicle gTrailer;
PxD6Joint* gJoint = NULL;

//Vehicle simulation needs a simulation context
//to store global parameters of the simulation such as 
//gravitational acceleration.
PxVehiclePhysXSimulationContext gVehicleSimulationContext;

//Gravitational acceleration
const PxVec3 gGravity(0.0f, -9.81f, 0.0f);

//The mapping between PxMaterial and friction.
PxVehiclePhysXMaterialFriction gPhysXMaterialFrictions[16];
PxU32 gNbPhysXMaterialFrictions = 0;
PxReal gPhysXDefaultMaterialFriction = 1.0f;

//Give the vehicles names so they can be identified in PVD.
const char gTractorName[] = "tractor";
const char gTrailerName[] = "trailer";

//Commands are issued to the vehicles in a pre-choreographed sequence.
//Note: 
//   the tractor responds to brake, throttle and steer commands.
//   the trailer responds only to brake commands.
struct Command
{
	PxF32 brake;
	PxF32 throttle;
	PxF32 steer;
	PxU32 gear;
	PxF32 duration;
};
const PxU32 gTargetGearCommand = PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;
Command gCommands[] =
{
	{0.5f, 0.0f, 0.0f, gTargetGearCommand, 2.0f},	//brake on and come to rest for 2 seconds
	{0.0f, 0.65f, 0.0f, gTargetGearCommand, 5.0f},	//throttle for 5 seconds
	{0.5f, 0.0f, 0.0f, gTargetGearCommand, 5.0f},	//brake for 5 seconds
	{0.0f, 0.75f, 0.0f, gTargetGearCommand, 5.0f}	//throttle for 5 seconds
};
const PxU32 gNbCommands = sizeof(gCommands) / sizeof(Command);
PxReal gCommandTime = 0.0f;			//Time spent on current command
PxU32 gCommandProgress = 0;			//The id of the current command.

//A ground plane to drive on.
PxRigidStatic*	gGroundPlane = NULL;

void initPhysX()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
		
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = gGravity;
	
	PxU32 numWorkers = 1;
	gDispatcher = PxDefaultCpuDispatcherCreate(numWorkers);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= VehicleFilterShader;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, false);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);	

	PxInitExtensions(*gPhysics, gPvd);
	PxInitVehicleExtension(*gFoundation);
}

void cleanupPhysX()
{
	PxCloseVehicleExtension();
	PxCloseExtensions();

	PX_RELEASE(gMaterial);
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

void initGroundPlane()
{
	gGroundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	for (PxU32 i = 0; i < gGroundPlane->getNbShapes(); i++)
	{
		PxShape* shape = NULL;
		gGroundPlane->getShapes(&shape, 1, i);
		shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
		shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false);
	}
	gScene->addActor(*gGroundPlane);
}

void cleanupGroundPlane()
{
	gGroundPlane->release();
}

void initMaterialFrictionTable()
{
	//Each physx material can be mapped to a tire friction value on a per tire basis.
	//If a material is encountered that is not mapped to a friction value, the friction value used is the specified default value.
	//In this snippet there is only a single material so there can only be a single mapping between material and friction.
	//In this snippet the same mapping is used by all tires.
	gPhysXMaterialFrictions[0].friction = 1.0f;
	gPhysXMaterialFrictions[0].material = gMaterial;
	gPhysXDefaultMaterialFriction = 1.0f;
	gNbPhysXMaterialFrictions = 1;
}

bool initVehicles()
{
	//Load the tractor params from json or set directly.
	readBaseParamsFromJsonFile(gVehicleDataPath, "Base.json", gTractor.mBaseParams);
	setPhysXIntegrationParams(gTractor.mBaseParams.axleDescription,
		gPhysXMaterialFrictions, gNbPhysXMaterialFrictions, gPhysXDefaultMaterialFriction,
		gTractor.mPhysXParams);
	readEngineDrivetrainParamsFromJsonFile(gVehicleDataPath, "EngineDrive.json", 
		gTractor.mEngineDriveParams);

	//Load the trailer params from json or set directly.
	readBaseParamsFromJsonFile(gVehicleDataPath, "Base.json", gTrailer.mBaseParams);
	setPhysXIntegrationParams(gTrailer.mBaseParams.axleDescription,
		gPhysXMaterialFrictions, gNbPhysXMaterialFrictions, gPhysXDefaultMaterialFriction,
		gTrailer.mPhysXParams);
	readDirectDrivetrainParamsFromJsonFile(gVehicleDataPath, "DirectDrive.json", gTrailer.mBaseParams.axleDescription,
		gTrailer.mDirectDriveParams);

	//Set the states to default.
	if (!gTractor.initialize(*gPhysics, PxCookingParams(PxTolerancesScale()), *gMaterial, 
		EngineDriveVehicle::eDIFFTYPE_FOURWHEELDRIVE))
	{
		return false;
	}
	if (!gTrailer.initialize(*gPhysics, PxCookingParams(PxTolerancesScale()), *gMaterial))
	{
		return false;
	}

	//Create a PhysX joint to connect tractor and trailer.
	//Create a joint anchor that is 1.5m behind the rear wheels of the tractor and 1.5m in front of the front wheels of the trailer.
	PxTransform anchorTractorFrame(PxIdentity);
	{
		//Rear wheels are 2 and 3.
		PxRigidBody* rigidActor = gTractor.mPhysXState.physxActor.rigidBody;
		const PxTransform cMassLocalPoseActorFrame = rigidActor->getCMassLocalPose();
		const PxVec3 frontAxlePosCMassFrame = (gTractor.mBaseParams.suspensionParams[2].suspensionAttachment.p + gTractor.mBaseParams.suspensionParams[3].suspensionAttachment.p)*0.5f;
		const PxQuat frontAxleQuatCMassFrame = gTractor.mBaseParams.suspensionParams[2].suspensionAttachment.q;
		const PxTransform anchorCMassFrame(frontAxlePosCMassFrame - PxVec3(0, 0, 1.5f), frontAxleQuatCMassFrame);
		anchorTractorFrame = cMassLocalPoseActorFrame * anchorCMassFrame;
	}
	PxTransform anchorTrailerFrame(PxIdentity);
	{
		//Front wheels are 0 and 1.
		PxRigidBody* rigidActor = gTrailer.mPhysXState.physxActor.rigidBody;
		const PxTransform cMassLocalPoseActorFrame = rigidActor->getCMassLocalPose();
		const PxVec3 rearAxlePosCMassFrame = (gTrailer.mBaseParams.suspensionParams[0].suspensionAttachment.p + gTractor.mBaseParams.suspensionParams[1].suspensionAttachment.p)*0.5f;
		const PxQuat rearAxleQuatCMassFrame = gTrailer.mBaseParams.suspensionParams[0].suspensionAttachment.q;
		const PxTransform anchorCMassFrame(rearAxlePosCMassFrame + PxVec3(0, 0, 1.5f), rearAxleQuatCMassFrame);
		anchorTrailerFrame = cMassLocalPoseActorFrame * anchorCMassFrame;
	}

	//Apply a start pose to the physx actor of tractor and trailer and add them to the physx scene.
	const PxTransform tractorPose(PxVec3(0.000000000f, -0.0500000119f, -1.59399998f), PxQuat(PxIdentity));
	gTractor.setUpActor(*gScene, tractorPose, gTractorName);
	const PxTransform  trailerPose = tractorPose*anchorTractorFrame*anchorTrailerFrame.getInverse();
	gTrailer.setUpActor(*gScene, trailerPose, gTrailerName);

	//Create a joint between tractor and trailer.
	{
		gJoint = PxD6JointCreate(*gPhysics, gTractor.mPhysXState.physxActor.rigidBody, anchorTractorFrame, gTrailer.mPhysXState.physxActor.rigidBody, anchorTrailerFrame);
		gJoint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
		gJoint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
		gJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
		gJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
		gJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		gJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
		gJoint->getConstraint()->setFlags(gJoint->getConstraint()->getFlags() | PxConstraintFlag::eCOLLISION_ENABLED);
	}

	//Set the tractor in 1st gear and to use the autobox
	gTractor.mEngineDriveState.gearboxState.currentGear = gTractor.mEngineDriveParams.gearBoxParams.neutralGear + 1;
	gTractor.mEngineDriveState.gearboxState.targetGear = gTractor.mEngineDriveParams.gearBoxParams.neutralGear + 1;
	gTractor.mTransmissionCommandState.targetGear = PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;

	//Set the trailer in neutral gear to prevent any drive torques being applied to the trailer.
	gTrailer.mTransmissionCommandState.gear = PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL;

	//Set up the simulation context.
	//The snippet is set up with
	//a) z as the longitudinal axis
	//b) x as the lateral axis
	//c) y as the vertical axis.
	//d) metres  as the lengthscale.
	gVehicleSimulationContext.setToDefault();
	gVehicleSimulationContext.frame.lngAxis = PxVehicleAxes::ePosZ;
	gVehicleSimulationContext.frame.latAxis = PxVehicleAxes::ePosX;
	gVehicleSimulationContext.frame.vrtAxis = PxVehicleAxes::ePosY;
	gVehicleSimulationContext.scale.scale = 1.0f;
	gVehicleSimulationContext.gravity = gGravity;
	gVehicleSimulationContext.physxScene = gScene;
	gVehicleSimulationContext.physxActorUpdateMode = PxVehiclePhysXActorUpdateMode::eAPPLY_ACCELERATION;
	return true;
}

void cleanupVehicles()
{
	gJoint->release();
	gTractor.destroy();
	gTrailer.destroy();
}

bool initPhysics()
{
	initPhysX();
	initGroundPlane();
	initMaterialFrictionTable();
	if (!initVehicles())
		return false;
	return true;
}

void cleanupPhysics()
{
	cleanupVehicles();
	cleanupGroundPlane();
	cleanupPhysX();
	printf("SnippetVehicle2Truck done.\n");
}

void stepPhysics()
{
	if (gNbCommands == gCommandProgress)
		return;

	const PxReal timestep = 1.0f/60.0f;

	//Apply the brake, throttle and steer to the command state of the tractor.
	const Command& command = gCommands[gCommandProgress];
	gTractor.mCommandState.brakes[0] = command.brake;
	gTractor.mCommandState.nbBrakes = 1;
	gTractor.mCommandState.throttle = command.throttle;
	gTractor.mCommandState.steer = command.steer;
	gTractor.mTransmissionCommandState.targetGear = command.gear;

	//Apply the brake to the command state of the trailer.
	gTrailer.mCommandState.brakes[0] = command.brake;
	gTrailer.mCommandState.nbBrakes = 1;

	//Apply substepping at low forward speed to improve simulation fidelity.
	//Tractor and trailer will have approximately the same forward speed so we can apply
	//the same substepping rules to the tractor and trailer.
	const PxVec3 linVel = gTractor.mPhysXState.physxActor.rigidBody->getLinearVelocity();
	const PxVec3 forwardDir = gTractor.mPhysXState.physxActor.rigidBody->getGlobalPose().q.getBasisVector2();
	const PxReal forwardSpeed = linVel.dot(forwardDir);
	const PxU8 nbSubsteps = (forwardSpeed < 5.0f ? 3 : 1);
	gTractor.mComponentSequence.setSubsteps(gTractor.mComponentSequenceSubstepGroupHandle, nbSubsteps);
	gTrailer.mComponentSequence.setSubsteps(gTrailer.mComponentSequenceSubstepGroupHandle, nbSubsteps);

	//Reset the sticky states on the trailer using the actuation state of the truck.
	//Vehicles are brought to rest with sticky constraints that apply velocity constraints to the tires of the vehicle.
	//A drive torque applied to any wheel of the tractor signals an intent to accelerate.
	//An intent to accelerate will release any sticky constraints applied to the tires of the tractor.
	//It is important to apply the intent to accelerate to the wheels of the trailer as well to prevent the trailer being
	//held at rest with its own sticky constraints. 
	//It is not possible to determine an intent to accelerate from the trailer alone because the wheels of the trailer
	//do not respond to the throttle commands and therefore receive zero drive torque.
	//The function PxVehicleTireStickyStateReset() will reset the sticky states of the trailer using an intention to 
	//accelerate derived from the state of the tractor.
	const PxVehicleArrayData<const PxVehicleWheelActuationState> tractorActuationStates(gTractor.mBaseState.actuationStates);
	PxVehicleArrayData<PxVehicleTireStickyState> trailerTireStickyStates(gTrailer.mBaseState.tireStickyStates);
	const bool intentionToAccelerate = PxVehicleAccelerationIntentCompute(gTractor.mBaseParams.axleDescription, tractorActuationStates);
	PxVehicleTireStickyStateReset(intentionToAccelerate, gTrailer.mBaseParams.axleDescription, trailerTireStickyStates);

	//Forward integrate the vehicles by a single timestep.
	gTractor.step(timestep, gVehicleSimulationContext);
	gTrailer.step(timestep, gVehicleSimulationContext);
	
	//Forward integrate the phsyx scene by a single timestep.
	gScene->simulate(timestep);
	gScene->fetchResults(true);

	//Increment the time spent on the current command.
	//Move to the next command in the list if enough time has lapsed.
	gCommandTime += timestep;
	if (gCommandTime > gCommands[gCommandProgress].duration)
	{
		gCommandProgress++;
		gCommandTime = 0.0f;
	}
}
	
int snippetMain(int argc, const char*const* argv)
{
	if (!parseVehicleDataPath(argc, argv, "SnippetVehicle2Truck", gVehicleDataPath))
		return 1;

	//Check that we can read from the json file before continuing.
	BaseVehicleParams baseParams;
	if (!readBaseParamsFromJsonFile(gVehicleDataPath, "Base.json", baseParams))
		return 1;

	//Check that we can read from the json file before continuing.
	EngineDrivetrainParams engineDrivetrainParams;
	if (!readEngineDrivetrainParamsFromJsonFile(gVehicleDataPath, "EngineDrive.json",
		engineDrivetrainParams))
		return 1;

#ifdef RENDER_SNIPPET
	extern void renderLoop(const char*);
	renderLoop("PhysX Snippet Vehicle2 Truck");
#else
	if (initPhysics())
	{
		while (gCommandProgress != gNbCommands)
		{
			stepPhysics();
		}
		cleanupPhysics();
	}
#endif

	return 0;
}

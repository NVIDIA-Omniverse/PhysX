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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of the physx vehicle sdk and demonstrates
// how to simulate a vehicle with direct drive using parameters, states and 
// components maintained by the PhysX Vehicle SDK. Particlar attention is paid
// to the simulation of a PhysX vehicle in a multi-threaded environment.

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
//2) a direct drive drivetrain model that forwards input controls to wheel torques and angles.
//3) a physx integration model that provides a representation of the vehicle in an associated physx scene.

// It is a good idea to record and playback with pvd (PhysX Visual Debugger).

//This snippet 
// ****************************************************************************

#include <ctype.h>

#include "PxPhysicsAPI.h"
#include "vehicle2/PxVehicleAPI.h"

#include "../snippetvehicle2common/directdrivetrain/DirectDrivetrain.h"
#include "../snippetvehicle2common/serialization/BaseSerialization.h"
#include "../snippetvehicle2common/serialization/DirectDrivetrainSerialization.h"
#include "../snippetvehicle2common/SnippetVehicleHelpers.h"

#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPVD.h"

#include "common/PxProfileZone.h"

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
PxTaskManager*			gTaskManager = NULL;

//The path to the vehicle json files to be loaded.
const char* gVehicleDataPath = NULL;

//The vehicles with direct drivetrain
#define NUM_VEHICLES 1024
DirectDriveVehicle gVehicles[NUM_VEHICLES];
PxVehiclePhysXActorBeginComponent* gPhysXBeginComponents[NUM_VEHICLES];
PxVehiclePhysXActorEndComponent* gPhysXEndComponents[NUM_VEHICLES];

#define NUM_WORKER_THREADS 4
#define UPDATE_BATCH_SIZE 1
#define NB_SUBSTEPS 1

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

//Give the vehicles a name so they can be identified in PVD.
const char gVehicleName[] = "directDrive";

//A ground plane to drive on.
PxRigidStatic*	gGroundPlane = NULL;

//Track the number of simulation steps.
PxU32 gNbSimulateSteps = 0;

//Commands are issued to the vehicle in a pre-choreographed sequence.
struct Command
{
	PxF32 brake;
	PxF32 throttle;
	PxF32 steer;
	PxF32 duration;
};
Command gCommands[] =
{
	{0.0f, 0.5f, 0.0f, 4.26f},		//throttle for 256 update steps at 60Hz
};
const PxU32 gNbCommands = sizeof(gCommands) / sizeof(Command);
PxReal gCommandTime = 0.0f;			//Time spent on current command
PxU32 gCommandProgress = 0;			//The id of the current command.


//Profile the different phases of a simulate step.

struct UpdatePhases
{
	enum Enum
	{
		eVEHICLE_PHYSX_BEGIN_COMPONENTS,
		eVEHICLE_UPDATE_COMPONENTS,
		eVEHICLE_PHYSX_END_COMPONENTS,
		ePHYSX_SCENE_SIMULATE,
		eMAX_NUM_UPDATE_STAGES
	};
};
static const char* gUpdatePhaseNames[UpdatePhases::eMAX_NUM_UPDATE_STAGES] =
{
	"vehiclePhysXBeginComponents",
	"vehicleUpdateComponents",
	"vehiclePhysXEndComponents",
	"physXSceneSimulate"
};

struct ProfileZones
{
	PxU64 times[UpdatePhases::eMAX_NUM_UPDATE_STAGES];

	ProfileZones()
	{
		for (int i = 0; i < UpdatePhases::eMAX_NUM_UPDATE_STAGES; ++i)
			times[i] = 0;
	}

	void print()
	{
		for (int i = 0; i < UpdatePhases::eMAX_NUM_UPDATE_STAGES; ++i)
		{
			float ms = SnippetUtils::getElapsedTimeInMilliseconds(times[i]);
			printf("%s: %f ms\n", gUpdatePhaseNames[i], PxF64(ms));
		}
	}

	void zoneStart(UpdatePhases::Enum zoneId)
	{
		PxU64 time = SnippetUtils::getCurrentTimeCounterValue();
		times[zoneId] -= time;
	}

	void zoneEnd(UpdatePhases::Enum zoneId)
	{
		PxU64 time = SnippetUtils::getCurrentTimeCounterValue();
		times[zoneId] += time;
	}
};
ProfileZones gProfileZones;

class ScopedProfileZone
{
private:
	ScopedProfileZone(const ScopedProfileZone&);
	ScopedProfileZone& operator=(const ScopedProfileZone&);

public:
	ScopedProfileZone(ProfileZones& zones, UpdatePhases::Enum zoneId)
		: mZones(zones)
		, mZoneId(zoneId)
	{
		zones.zoneStart(zoneId);
	}

	~ScopedProfileZone()
	{
		mZones.zoneEnd(mZoneId);
	}


private:
	ProfileZones& mZones;
	UpdatePhases::Enum mZoneId;
};

#define SNIPPET_PROFILE_ZONE(zoneId) ScopedProfileZone PX_CONCAT(_scoped, __LINE__)(gProfileZones, zoneId)

//TaskVehicleUpdates allows vehicle updates to be performed concurrently across
//multiple threads.
class TaskVehicleUpdates : public PxLightCpuTask
{
public:

	TaskVehicleUpdates()
		: PxLightCpuTask(),
		mTimestep(0),
		mGravity(PxVec3(0, 0, 0)),
		mThreadId(0xffffffff),
		mCommandProgress(0)
	{
	}

	void setThreadId(const PxU32 threadId)
	{
		mThreadId = threadId;
	}

	void setTimestep(const PxF32 timestep)
	{
		mTimestep = timestep;
	}

	void setGravity(const PxVec3& gravity)
	{
		mGravity = gravity;
	}

	void setCommandProgress(const PxU32 commandProgress)
	{
		mCommandProgress = commandProgress;
	}

	virtual void run()
	{
		PxU32 vehicleId = mThreadId * UPDATE_BATCH_SIZE;
		while (vehicleId < NUM_VEHICLES)
		{
			const PxU32 numToUpdate = PxMin(NUM_VEHICLES - vehicleId, static_cast<PxU32>(UPDATE_BATCH_SIZE));
			for (PxU32 i = 0; i < numToUpdate; i++)
			{
				gVehicles[vehicleId + i].mCommandState.brakes[0] = gCommands[mCommandProgress].brake;
				gVehicles[vehicleId + i].mCommandState.nbBrakes = 1;
				gVehicles[vehicleId + i].mCommandState.throttle = gCommands[mCommandProgress].throttle;
				gVehicles[vehicleId + i].mCommandState.steer = gCommands[mCommandProgress].steer;
				gVehicles[vehicleId + i].mTransmissionCommandState.gear = PxVehicleDirectDriveTransmissionCommandState::eFORWARD;
				gVehicles[vehicleId + i].step(mTimestep, gVehicleSimulationContext);
			}
			vehicleId += NUM_WORKER_THREADS * UPDATE_BATCH_SIZE;
		}
	}

	virtual const char* getName() const { return "TaskVehicleUpdates"; }

private:

	PxF32 mTimestep;
	PxVec3 mGravity;

	PxU32 mThreadId;

	PxU32 mCommandProgress;
};

//TaskWait runs after all concurrent updates have completed.
class TaskWait : public PxLightCpuTask
{
public:

	TaskWait(SnippetUtils::Sync* syncHandle)
		: PxLightCpuTask(),
		mSyncHandle(syncHandle)
	{
	}

	virtual void run()
	{
	}

	PX_INLINE void release()
	{
		PxLightCpuTask::release();
		SnippetUtils::syncSet(mSyncHandle);
	}

	virtual const char* getName() const { return "TaskWait"; }

private:

	SnippetUtils::Sync* mSyncHandle;
};

void initPhysX()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::ePROFILE);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
		
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = gGravity;
	
	gDispatcher = PxDefaultCpuDispatcherCreate(NUM_WORKER_THREADS);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= VehicleFilterShader;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, false);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, false);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, false);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);	

	/////////////////////////////////////////////
	//Create a task manager that will be used to 
	//update the vehicles concurrently across 
	//multiple threads.
	/////////////////////////////////////////////

	gTaskManager = PxTaskManager::createTaskManager(gFoundation->getErrorCallback(), gDispatcher);

	PxInitVehicleExtension(*gFoundation);
}

void cleanupPhysX()
{
	PxCloseVehicleExtension();

	PX_RELEASE(gTaskManager);
	PX_RELEASE(gMaterial);
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();
		transport->release();
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
	//Load the params from json 

	BaseVehicleParams baseParams;
	readBaseParamsFromJsonFile(gVehicleDataPath, "Base.json", baseParams);

	PhysXIntegrationParams physxParams;
	setPhysXIntegrationParams(baseParams.axleDescription,
		gPhysXMaterialFrictions, gNbPhysXMaterialFrictions, gPhysXDefaultMaterialFriction,
		physxParams);

	DirectDrivetrainParams directDrivetrainParams;
	readDirectDrivetrainParamsFromJsonFile(gVehicleDataPath, "DirectDrive.json", baseParams.axleDescription,
		directDrivetrainParams);

	//Create the params, states and component sequences for direct drive vehicles.
	//Take care not to add PxVehiclePhysXActorBeginComponent or PxVehiclePhysXActorEndComponent
	//to the sequences because are executed in a separate step.
	for (PxU32 i = 0; i < NUM_VEHICLES; i++)
	{
		//Set the vehicle params.
		//Every vehicle is identical.
		gVehicles[i].mBaseParams = baseParams;
		gVehicles[i].mPhysXParams = physxParams;
		gVehicles[i].mDirectDriveParams = directDrivetrainParams;

		//Set the states to default and create the component sequence.
		//Take care not to add PxVehiclePhysXActorBeginComponent and PxVehiclePhysXActorEndComponent 
		//to the sequence because these are handled separately to take advantage of multi-threading.
		const bool addPhysXBeginAndEndComponentsToSequence = false;
		if (!gVehicles[i].initialize(*gPhysics, PxCookingParams(PxTolerancesScale()), *gMaterial, 
			addPhysXBeginAndEndComponentsToSequence))
		{
			return false;
		}

		//Force a known substep count per simulation step so that we have a perfect understanding of 
		//the amount of computational effort involved in running the snippet.
		gVehicles[i].mComponentSequence.setSubsteps(gVehicles[i].mComponentSequenceSubstepGroupHandle, NB_SUBSTEPS);

		//Apply a start pose to the physx actor and add it to the physx scene.
		PxTransform pose(PxVec3(5.0f*(PxI32(i) - NUM_VEHICLES/2), 0.0f, 0.0f), PxQuat(PxIdentity));
		gVehicles[i].setUpActor(*gScene, pose, gVehicleName);
	}

	//PhysX reads/writes require read/write locks that serialize executions.
	//Perform all physx reads/writes serially in a separate step to avoid serializing code that can take 
	//advantage of multithreading.
	for (PxU32 i = 0; i < NUM_VEHICLES; i++)
	{
		gPhysXBeginComponents[i] = (static_cast<PxVehiclePhysXActorBeginComponent*>(gVehicles + i));
		gPhysXEndComponents[i] = (static_cast<PxVehiclePhysXActorEndComponent*>(gVehicles + i));
	}
	
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
	for (PxU32 i = 0; i < NUM_VEHICLES; i++)
	{
		gVehicles[i].destroy();
	}
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
}

void concurrentVehicleUpdates(const PxReal timestep)
{
	SnippetUtils::Sync* vehicleUpdatesComplete = SnippetUtils::syncCreate();
	SnippetUtils::syncReset(vehicleUpdatesComplete);

	//Create tasks that will update the vehicles concurrently then wait until all vehicles 
	//have completed their update.
	TaskWait taskWait(vehicleUpdatesComplete);
	TaskVehicleUpdates taskVehicleUpdates[NUM_WORKER_THREADS];
	for (PxU32 i = 0; i < NUM_WORKER_THREADS; i++)
	{
		taskVehicleUpdates[i].setThreadId(i);
		taskVehicleUpdates[i].setTimestep(timestep);
		taskVehicleUpdates[i].setGravity(gScene->getGravity());
		taskVehicleUpdates[i].setCommandProgress(gCommandProgress);
	}

	//Start the task manager.
	gTaskManager->resetDependencies();
	gTaskManager->startSimulation();

	//Perform a vehicle simulation step and profile each phase of the simulation.
	{
		//PhysX reads/writes require read/write locks that serialize executions.
		//Perform all physx reads/writes serially in a separate step to avoid serializing code that can take 
		//advantage of multithreading.
		{
			SNIPPET_PROFILE_ZONE(UpdatePhases::eVEHICLE_PHYSX_BEGIN_COMPONENTS);
			for (PxU32 i = 0; i < NUM_VEHICLES; i++)
			{
				gPhysXBeginComponents[i]->update(timestep, gVehicleSimulationContext);
			}
		}

		//Multi-threaded update of direct drive vehicles.
		{
			SNIPPET_PROFILE_ZONE(UpdatePhases::eVEHICLE_UPDATE_COMPONENTS);

			//Update the vehicles concurrently then wait until all vehicles 
			//have completed their update.
			taskWait.setContinuation(*gTaskManager, NULL);
			for (PxU32 i = 0; i < NUM_WORKER_THREADS; i++)
			{
				taskVehicleUpdates[i].setContinuation(&taskWait);
			}
			taskWait.removeReference();
			for (PxU32 i = 0; i < NUM_WORKER_THREADS; i++)
			{
				taskVehicleUpdates[i].removeReference();
			}

			//Wait for the signal that the work has been completed.
			SnippetUtils::syncWait(vehicleUpdatesComplete);

			//Release the sync handle
			SnippetUtils::syncRelease(vehicleUpdatesComplete);
		}

		//PhysX reads/writes require read/write locks that serialize executions.
		//Perform all physx reads/writes serially in a separate step to avoid serializing code that can take 
		//advantage of multithreading.
		{
			SNIPPET_PROFILE_ZONE(UpdatePhases::eVEHICLE_PHYSX_END_COMPONENTS);
			for (PxU32 i = 0; i < NUM_VEHICLES; i++)
			{
				gPhysXEndComponents[i]->update(timestep, gVehicleSimulationContext);
			}
		}
	}
}

void stepPhysics()
{
	if(gNbCommands == gCommandProgress)
		return;

	const PxF32 timestep = 0.0166667f;

	//Multithreaded update of all vehicles.
	concurrentVehicleUpdates(timestep);

	//Forward integrate the phsyx scene by a single timestep.
	SNIPPET_PROFILE_ZONE(UpdatePhases::ePHYSX_SCENE_SIMULATE);
	gScene->simulate(timestep);
	gScene->fetchResults(true);

	//Increment the time spent on the current command.
	//Move to the next command in the list if enough time has lapsed.
	gCommandTime += timestep;
	if(gCommandTime > gCommands[gCommandProgress].duration)
	{
		gCommandProgress++;
		gCommandTime = 0.0f;
	}

	gNbSimulateSteps++;
}
	
int snippetMain(int argc, const char*const* argv)
{
	if(!parseVehicleDataPath(argc, argv, "SnippetVehicle2Multithreading", gVehicleDataPath))
		return 1;

	//Check that we can read from the json file before continuing.
	BaseVehicleParams baseParams;
	if (!readBaseParamsFromJsonFile(gVehicleDataPath, "Base.json", baseParams))
		return 1;

	//Check that we can read from the json file before continuing.
	DirectDrivetrainParams directDrivetrainParams;
	if (!readDirectDrivetrainParamsFromJsonFile(gVehicleDataPath, "DirectDrive.json",
		baseParams.axleDescription, directDrivetrainParams))
		return 1;

	printf("Initialising ... \n");
	if(initPhysics())
	{
		printf("Simulating %d vehicles with %d threads \n", NUM_VEHICLES, NUM_WORKER_THREADS);
		while(gCommandProgress != gNbCommands)
		{
			stepPhysics();
		}
		printf("Completed %d simulate steps with %d substeps per simulate step \n", gNbSimulateSteps, NB_SUBSTEPS);
		gProfileZones.print();
		cleanupPhysics();
	}

	return 0;
}

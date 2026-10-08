// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAllocator.h"

#include "vehicle/PxVehicleParams.h"

#include "vehicle/physxConstraints/PxVehiclePhysXConstraintStates.h"
#include "vehicle/physxConstraints/PxVehiclePhysXConstraintHelpers.h"

#include "vehicle/physxActor/PxVehiclePhysXActorStates.h"

#include "PxConstraintDesc.h"
#include "PxConstraint.h"
#include "PxPhysics.h"
#include "PxRigidDynamic.h"
#include "PxArticulationLink.h"

namespace physx
{
PxConstraintShaderTable gVehicleConstraintTable =
{
	vehicleConstraintSolverPrep,
	visualiseVehicleConstraint,
	PxConstraintFlag::Enum(0)
};

void PxVehicleConstraintsCreate(
 const PxVehicleAxleDescription& axleDescription, 
 PxPhysics& physics, PxRigidBody& physxActor,
 PxVehiclePhysXConstraints& vehicleConstraints)
{
	vehicleConstraints.setToDefault();

	//Each PxConstraint has a limit of 12 1d constraints.
	//Each wheel has longitudinal, lateral and suspension limit degrees of freedom.
	//This sums up to 3 dofs per wheel and 12 dofs per 4 wheels.
	//4 wheels therefore equals 1 PxConstraint 
	//Iterate over each block of 4 wheels and create a PxConstraints for each block of 4.
	PxU32 constraintIndex = 0;
	for(PxU32 i = 0; i < axleDescription.getNbWheels(); i+= PxVehiclePhysXConstraintLimits::eNB_WHEELS_PER_PXCONSTRAINT)
	{
		void* memory = PX_ALLOC(sizeof(PxVehicleConstraintConnector), PxVehicleConstraintConnector);
		PxVehicleConstraintConnector* pxConnector = PX_PLACEMENT_NEW(memory, PxVehicleConstraintConnector)(vehicleConstraints.constraintStates + i);
		PxConstraint* pxConstraint = physics.createConstraint(&physxActor, NULL, *pxConnector, gVehicleConstraintTable, sizeof(PxVehiclePhysXConstraintState)*PxVehiclePhysXConstraintLimits::eNB_WHEELS_PER_PXCONSTRAINT);
		vehicleConstraints.constraints[constraintIndex] = pxConstraint;
		vehicleConstraints.constraintConnectors[constraintIndex] = pxConnector;
		constraintIndex++;
	}
}

void PxVehicleConstraintsDirtyStateUpdate
(PxVehiclePhysXConstraints& vehicleConstraints)
{
	for (PxU32 i = 0; i < PxVehiclePhysXConstraintLimits::eNB_CONSTRAINTS_PER_VEHICLE; i++)
	{
		if (vehicleConstraints.constraints[i])
		{
			vehicleConstraints.constraints[i]->markDirty();
		}
	}
}

void PxVehicleConstraintsDestroy(
 PxVehiclePhysXConstraints& vehicleConstraints)
{
	for (PxU32 i = 0; i < PxVehiclePhysXConstraintLimits::eNB_CONSTRAINTS_PER_VEHICLE; i++)
	{
		if (vehicleConstraints.constraints[i])
		{
			vehicleConstraints.constraints[i]->release(); 
			vehicleConstraints.constraints[i] = NULL;
		}

		if (vehicleConstraints.constraintConnectors[i])
		{
			vehicleConstraints.constraintConnectors[i]->~PxVehicleConstraintConnector();
			PX_FREE(vehicleConstraints.constraintConnectors[i]);
			vehicleConstraints.constraintConnectors[i] = NULL;
		}
	}
}

} //namespace physx

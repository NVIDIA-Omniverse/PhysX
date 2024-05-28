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

#include "foundation/PxAllocator.h"

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintStates.h"
#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintHelpers.h"

#include "vehicle2/physxActor/PxVehiclePhysXActorStates.h"

#include "PxConstraintDesc.h"
#include "PxConstraint.h"
#include "PxPhysics.h"
#include "PxRigidDynamic.h"
#include "PxArticulationLink.h"

namespace physx
{
namespace vehicle2
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

} //namespace vehicle2
} //namespace physx

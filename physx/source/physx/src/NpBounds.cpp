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

#include "foundation/PxTransform.h"
#include "NpBounds.h"
#include "CmTransformUtils.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "GuBounds.h"

using namespace physx;
using namespace Sq;

static void computeStaticWorldAABB(PxBounds3& bounds, const NpShape& npShape, const NpActor& npActor)
{
	const PxTransform& shape2Actor = npShape.getCore().getShape2Actor();

	PX_ALIGN(16, PxTransform) globalPose;

	Cm::getStaticGlobalPoseAligned(static_cast<const NpRigidStatic&>(npActor).getCore().getActor2World(), shape2Actor, globalPose);

	Gu::computeBounds(bounds, npShape.getCore().getGeometry(), globalPose, 0.0f, SQ_PRUNER_INFLATION);
}

static void computeDynamicWorldAABB(PxBounds3& bounds, const NpShape& npShape, const NpActor& npActor)
{
	const PxTransform& shape2Actor = npShape.getCore().getShape2Actor();

	PX_ALIGN(16, PxTransform) globalPose;
	{
		PX_ALIGN(16, PxTransform) kinematicTarget;
		const PxU16 sqktFlags = PxRigidBodyFlag::eKINEMATIC | PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES;
		// PT: TODO: revisit this once the dust has settled
		PX_ASSERT(npActor.getNpType()==NpType::eBODY_FROM_ARTICULATION_LINK || npActor.getNpType()==NpType::eBODY);
		const Sc::BodyCore& core = npActor.getNpType()==NpType::eBODY_FROM_ARTICULATION_LINK ? static_cast<const NpArticulationLink&>(npActor).getCore() : static_cast<const NpRigidDynamic&>(npActor).getCore();
		const bool useTarget = (PxU16(core.getFlags()) & sqktFlags) == sqktFlags;
		const PxTransform& body2World = (useTarget && core.getKinematicTarget(kinematicTarget)) ? kinematicTarget : core.getBody2World();
		Cm::getDynamicGlobalPoseAligned(body2World, shape2Actor, core.getBody2Actor(), globalPose);
	}

	Gu::computeBounds(bounds, npShape.getCore().getGeometry(), globalPose, 0.0f, SQ_PRUNER_INFLATION);
}

const ComputeBoundsFunc Sq::gComputeBoundsTable[2] = 
{ 
	computeStaticWorldAABB, 
	computeDynamicWorldAABB
};

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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/physxRoadGeometry/PxVehiclePhysXRoadGeometryHelpers.h"

#include "cooking/PxConvexMeshDesc.h"
#include "cooking/PxCooking.h"
#include "extensions/PxDefaultStreams.h"
#include "PxPhysics.h"

namespace physx
{
namespace vehicle2
{

PxConvexMesh* PxVehicleUnitCylinderSweepMeshCreate
(const PxVehicleFrame& runtimeFrame, PxPhysics& physics, const PxCookingParams& params)
{
	const PxMat33 mat33 = runtimeFrame.getFrame();
	const PxQuat frame(mat33);
	const PxReal radius = 1.0f;
	const PxReal halfWidth = 1.0f;

	#define NB_CIRCUMFERENCE_POINTS 64
	PxVec3 points[2 * NB_CIRCUMFERENCE_POINTS];
	for (PxU32 i = 0; i < NB_CIRCUMFERENCE_POINTS; i++)
	{
		const PxF32 cosTheta = PxCos(i * PxPi * 2.0f / float(NB_CIRCUMFERENCE_POINTS));
		const PxF32 sinTheta = PxSin(i * PxPi * 2.0f / float(NB_CIRCUMFERENCE_POINTS));
		const PxF32 x = radius * cosTheta;
		const PxF32 z = radius * sinTheta;
		points[2 * i + 0] = frame.rotate(PxVec3(x, -halfWidth, z));
		points[2 * i + 1] = frame.rotate(PxVec3(x, +halfWidth, z));
	}

	// Create descriptor for convex mesh
	PxConvexMeshDesc convexDesc;
	convexDesc.points.count = sizeof(points)/sizeof(PxVec3);
	convexDesc.points.stride = sizeof(PxVec3);
	convexDesc.points.data = points;
	convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxConvexMesh* convexMesh = NULL;
	PxDefaultMemoryOutputStream buf;
	if (PxCookConvexMesh(params, convexDesc, buf))
	{
		PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
		convexMesh = physics.createConvexMesh(id);
	}

	return convexMesh;
}

void PxVehicleUnitCylinderSweepMeshDestroy(PxConvexMesh* mesh)
{
	mesh->release();
}

} //namespace vehicle2
} //namespace physx

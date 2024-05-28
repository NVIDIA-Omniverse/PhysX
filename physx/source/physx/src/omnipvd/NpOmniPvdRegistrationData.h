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

#ifndef NP_OMNI_PVD_REGISTRATION_DATA_H
#define NP_OMNI_PVD_REGISTRATION_DATA_H


#if PX_SUPPORT_OMNI_PVD

#include "OmniPvdWriter.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxBounds3.h"
#include "PxSceneDesc.h"  // for PxGpuDynamicsMemoryConfig
#include "PxActor.h"// for PxDominanceGroup
#include "PxClient.h"  // for PxClientID
#include "PxMaterial.h"  // for PxMaterialFlags, PxCombineMode
#include "PxRigidBody.h"  // for PxRigidBodyFlags
#include "PxRigidDynamic.h"  // for PxRigidDynamicLockFlags
#include "PxShape.h"  // for PxShapeFlags
#include "solver/PxSolverDefs.h"  // for PxArticulationMotion
#include "geometry/PxCustomGeometry.h"  // for PxCustomGeometry::Callbacks

#if PX_SUPPORT_GPU_PHYSX
#include "PxPBDParticleSystem.h"
#include "PxParticleBuffer.h"
#endif

namespace physx
{

class PxAggregate;
class PxArticulationJointReducedCoordinate;
class PxArticulationReducedCoordinate;
class PxArticulationMimicJoint;
class PxBaseMaterial;
class PxBoxGeometry;
class PxBVH;
class PxCapsuleGeometry;
class PxConvexMesh;
class PxConvexMeshGeometry;
class PxFEMClothMaterial;
class PxFEMSoftBodyMaterial;
class PxGeometry;
class PxHeightField;
class PxHeightFieldGeometry;
class PxPBDMaterial;
class PxPhysics;
class PxPlaneGeometry;
class PxRigidActor;
class PxRigidStatic;
class PxScene;
class PxSoftBodyMesh;
class PxSphereGeometry;
class PxTetrahedronMesh;
class PxTetrahedronMeshGeometry;
class PxTolerancesScale;
class PxTriangleMesh;
class PxTriangleMeshGeometry;

struct OmniPvdPxCoreRegistrationData
{
	void registerData(OmniPvdWriter&);

	// auto-generate members and setter methods from object definition file
#include "omnipvd/CmOmniPvdAutoGenCreateRegistrationStruct.h"
#include "OmniPvdTypes.h"
#include "omnipvd/CmOmniPvdAutoGenClearDefines.h"

};

}

#endif  // PX_SUPPORT_OMNI_PVD

#endif  // NP_OMNI_PVD_REGISTRATION_DATA_H

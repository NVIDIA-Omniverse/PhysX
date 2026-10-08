// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

#include "PxDeformableBodyFlag.h"

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
class PxDeformableBody;
class PxDeformableSurface;
class PxDeformableSurfaceMaterial;
class PxDeformableVolume;
class PxDeformableVolumeMaterial;
class PxGeometry;
class PxHeightField;
class PxHeightFieldGeometry;
class PxPBDMaterial;
class PxPhysics;
class PxPlaneGeometry;
class PxRigidActor;
class PxRigidStatic;
class PxScene;
class PxDeformableVolumeMesh;
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

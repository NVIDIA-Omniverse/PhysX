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
#ifndef PX_PHYSICS_NXPHYSICSWITHEXTENSIONS_API
#define PX_PHYSICS_NXPHYSICSWITHEXTENSIONS_API

#include "PxExtensionsCommon.h"

//Property overrides will output this exact property name instead of the general
//property name that would be used.  The properties need to have no template arguments
//and exactly the same initialization as the classes they are overriding.
static PropertyOverride gPropertyOverrides[] = {
	PropertyOverride( "PxShape", "Materials", "PxShapeMaterialsProperty" ),
	PropertyOverride( "PxRigidActor", "Shapes", "PxRigidActorShapeCollection" ),
	PropertyOverride( "PxArticulationReducedCoordinate", "Links", "PxArticulationLinkCollectionProp" ),
};

//The meta data generator ignores properties that are marked as disabled.  Note that in the declaration
//for properties that are defined via getter and setter methods, the 'get' and 'set' prefix can be dropped.
//For protected fields the "m" prefix can be dropped, however for public fields the whole field qualifier is expected.
static DisabledPropertyEntry gDisabledProperties[] = {
	DisabledPropertyEntry( "PxSceneLimits", "IsValid" ),
	DisabledPropertyEntry( "PxSceneDesc", "TolerancesScale" ),
	DisabledPropertyEntry( "PxSceneDesc", "IsValid" ),
	DisabledPropertyEntry( "PxSceneDesc", "SceneQuerySystem" ),
    DisabledPropertyEntry( "PxSceneDesc", "DeformableSurfacePostSolveCallback" ),
    DisabledPropertyEntry( "PxSceneDesc", "DeformableVolumePostSolveCallback" ),
	DisabledPropertyEntry( "PxShape", "Actor" ),
	DisabledPropertyEntry( "PxShape", "Geometry" ),
	DisabledPropertyEntry("PxShape", "GPUIndex"),
	DisabledPropertyEntry( "PxArticulationLink", "Articulation" ),
	DisabledPropertyEntry("PxArticulationJointReducedCoordinate", "ParentArticulationLink"),
	DisabledPropertyEntry("PxArticulationJointReducedCoordinate", "ChildArticulationLink"),
	DisabledPropertyEntry("PxArticulationJointReducedCoordinate", "Limit"),
	DisabledPropertyEntry( "PxArticulationReducedCoordinate", "LoopJoints"),
	DisabledPropertyEntry("PxArticulationReducedCoordinate", "Dofs"),
	DisabledPropertyEntry("PxArticulationReducedCoordinate", "CacheDataSize"),
	DisabledPropertyEntry("PxArticulationReducedCoordinate", "CoefficientMatrixSize"),
	DisabledPropertyEntry("PxArticulationReducedCoordinate", "GPUIndex"),
	DisabledPropertyEntry( "PxRigidActor", "IsRigidActor" ),
	DisabledPropertyEntry( "PxRigidActor", "ClassName" ),
	DisabledPropertyEntry( "PxRigidActor", "InternalActorIndex" ),
	DisabledPropertyEntry( "PxRigidStatic", "ClassName" ),
	DisabledPropertyEntry( "PxRigidDynamic", "ClassName" ),
	DisabledPropertyEntry("PxRigidDynamic", "GPUIndex" ),
	DisabledPropertyEntry( "PxRigidBody", "IsRigidBody" ),
	DisabledPropertyEntry( "PxRigidBody", "InternalIslandNodeIndex"),
	DisabledPropertyEntry( "PxRigidBody", "LinearVelocity"),
	DisabledPropertyEntry("PxRigidBody", "AngularVelocity"),
	DisabledPropertyEntry( "PxActor", "IsRigidStatic" ),
	DisabledPropertyEntry( "PxActor", "Type" ),
	DisabledPropertyEntry( "PxActor", "ClassName" ),
	DisabledPropertyEntry( "PxActor", "IsRigidDynamic" ),
	DisabledPropertyEntry( "PxActor", "IsArticulationLink" ),
	DisabledPropertyEntry( "PxActor", "IsRigidActor" ),
	DisabledPropertyEntry( "PxActor", "IsRigidBody" ),
	DisabledPropertyEntry( "PxMeshScale", "Inverse" ),
	DisabledPropertyEntry( "PxMeshScale", "IsIdentity" ),
	DisabledPropertyEntry( "PxMeshScale", "IsValidForTriangleMesh" ),
	DisabledPropertyEntry( "PxMeshScale", "IsValidForConvexMesh" ),
	DisabledPropertyEntry( "PxGeometry", "Type" ),
	DisabledPropertyEntry( "PxGeometry", "MTypePadding" ),
	DisabledPropertyEntry( "PxBoxGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxSphereGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxPlaneGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxCapsuleGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxConvexMeshGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxTetrahedronMeshGeometry", "IsValid"),
	DisabledPropertyEntry( "PxTriangleMeshGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxHeightFieldGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxCustomGeometry", "IsValid" ),
	DisabledPropertyEntry( "PxCustomGeometry", "Callbacks" ),
	DisabledPropertyEntry( "PxJoint", "ClassName" ),
	DisabledPropertyEntry( "PxDistanceJoint", "ClassName" ),
	DisabledPropertyEntry( "PxContactJoint", "ClassName"),
	DisabledPropertyEntry( "PxGearJoint", "ClassName"),
	DisabledPropertyEntry( "PxRackAndPinionJoint", "ClassName"),
	DisabledPropertyEntry( "PxFixedJoint", "ClassName" ),
	DisabledPropertyEntry( "PxRevoluteJoint", "ClassName" ),
	DisabledPropertyEntry( "PxPrismaticJoint", "ClassName" ),
	DisabledPropertyEntry( "PxSphericalJoint", "ClassName" ),
	DisabledPropertyEntry( "PxD6Joint", "ClassName" ),
	DisabledPropertyEntry( "PxJointLimitParameters", "IsValid" ),
	DisabledPropertyEntry( "PxJointLimitParameters", "IsSoft" ),
	DisabledPropertyEntry( "PxJointLinearLimit", "IsValid" ),
	DisabledPropertyEntry( "PxJointLinearLimitPair", "IsValid" ),
	DisabledPropertyEntry( "PxJointAngularLimitPair", "IsValid" ),
	DisabledPropertyEntry( "PxJointLimitCone", "IsValid" ),
	DisabledPropertyEntry( "PxJointLimitPyramid", "IsValid" ),
	DisabledPropertyEntry( "PxD6JointDrive", "IsValid" ),
	DisabledPropertyEntry( "PxScene", "ParticleSystems"),
	DisabledPropertyEntry( "PxScene", "DeformableSurfaces"),
	// PT: added this for PVD-315. It's a mystery to me why we don't need to do that here for PxConvexMeshDesc. Maybe because the convex desc is in the cooking lib.
	DisabledPropertyEntry( "PxHeightFieldDesc", "IsValid" ),
//	DisabledPropertyEntry( "PxConstraint", "IsValid" ),
//	DisabledPropertyEntry( "PxTolerancesScale", "IsValid" ),
	DisabledPropertyEntry( "PxConstraint", "SolverResidual" ),
	DisabledPropertyEntry( "PxArticulationReducedCoordinate", "SolverResidual" ),
};

//Append these properties to this type.
static CustomProperty gCustomProperties[] = {
#define DEFINE_SIM_STATS_DUAL_INDEXED_PROPERTY( propName, propType, fieldName ) CustomProperty("PxSimulationStatistics", #propName,	#propType, "PxU32 " #propName "[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];", "PxMemCopy( "#propName ", inSource->"#fieldName", sizeof( "#propName" ) );" )
	DEFINE_SIM_STATS_DUAL_INDEXED_PROPERTY( NbDiscreteContactPairs, NbDiscreteContactPairsProperty, nbDiscreteContactPairs ),
	DEFINE_SIM_STATS_DUAL_INDEXED_PROPERTY( NbModifiedContactPairs, NbModifiedContactPairsProperty, nbModifiedContactPairs),
	DEFINE_SIM_STATS_DUAL_INDEXED_PROPERTY( NbCCDPairs, NbCCDPairsProperty, nbCCDPairs),
	DEFINE_SIM_STATS_DUAL_INDEXED_PROPERTY( NbTriggerPairs, NbTriggerPairsProperty, nbTriggerPairs),
#undef DEFINE_SIM_STATS_DUAL_INDEXED_PROPERTY
	CustomProperty( "PxSimulationStatistics",	"NbShapes",				"NbShapesProperty", "PxU32 NbShapes[PxGeometryType::eGEOMETRY_COUNT];", "PxMemCopy( NbShapes, inSource->nbShapes, sizeof( NbShapes ) );" ),
	CustomProperty( "PxScene",					"SimulationStatistics",	"SimulationStatisticsProperty", "PxSimulationStatistics SimulationStatistics;", "inSource->getSimulationStatistics(SimulationStatistics);"  ),
	CustomProperty( "PxShape",					"Geom",					"PxShapeGeomProperty", "PxGeometryHolder Geom;", "Geom = PxGeometryHolder(inSource->getGeometry());"  ),
	CustomProperty( "PxCustomGeometry", "CustomType", "PxCustomGeometryCustomTypeProperty", "PxU32 CustomType;", "PxCustomGeometry::Type t = inSource->callbacks->getCustomType(); CustomType = *reinterpret_cast<const PxU32*>(&t);"  ),
};

static const char* gImportantPhysXTypes[] =
{
	"PxRigidStatic",
	"PxRigidDynamic",
	"PxShape",
	"PxArticulationReducedCoordinate",
	"PxArticulationLink",
	"PxMaterial",
	"PxDeformableSurfaceMaterial",
	"PxDeformableVolumeMaterial",
	"PxPBDMaterial",
	"PxArticulationJointReducedCoordinate",
	"PxArticulationLimit",
	"PxArticulationDrive",
	"PxScene",
	"PxPhysics",
	"PxHeightFieldDesc",
	"PxMeshScale",
	"PxConstraint",
	"PxTolerancesScale",
	"PxSimulationStatistics",
	"PxSceneDesc",
	"PxSceneLimits",
	"PxGpuDynamicsMemoryConfig",
	"PxBroadPhaseDesc",
	"PxGeometry",
	"PxBoxGeometry",
	"PxCapsuleGeometry",
	"PxConvexMeshGeometry",
	"PxSphereGeometry",
	"PxPlaneGeometry",
	"PxTetrahedronMeshGeometry",
	"PxTriangleMeshGeometry",
	"PxHeightFieldGeometry",
	"PxCustomGeometry",
    "PxAggregate",
    "PxPruningStructure",
	"PxGpuDynamicsMemoryConfigStatistics"
	//The mesh and heightfield buffers will need to be
	//handled by hand; they are very unorthodox compared
	//to the rest of the objects.
};

static const char* gExtensionPhysXTypes[] =
{
	"PxD6JointDrive",
	"PxJointLinearLimit",
	"PxJointLinearLimitPair",
	"PxJointAngularLimitPair",
	"PxJointLimitCone",
	"PxJointLimitPyramid",
	"PxD6Joint",
	"PxDistanceJoint",
	"PxGearJoint",
	"PxRackAndPinionJoint",
	"PxContactJoint",
	"PxFixedJoint",
	"PxPrismaticJoint",
	"PxRevoluteJoint",
	"PxSphericalJoint",
};

//We absolutely never generate information about these types, even if types
//we do care about are derived from these types.
static const char* gAvoidedPhysXTypes[] =
{
	"PxSerializable",
    "PxObservable",
	"PxBase",
    "PxBaseFlag::Enum",
    "PxFLIPMaterial",
    "PxMPMMaterial",
    "PxFEMSoftBodyMaterial",
    "PxSoftBody"
};

#include "PxPhysicsAPI.h"
#include "extensions/PxExtensionsAPI.h"

#endif

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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

// suppress LNK4221
#include "foundation/PxPreprocessor.h"
PX_DUMMY_SYMBOL

#if PX_SUPPORT_PVD

#include "foundation/PxSimpleTypes.h"
#include "foundation/Px.h"

#include "PxMetaDataObjects.h"
#include "PxPvdDataStream.h"
#include "PxScene.h"
#include "ScBodyCore.h"
#include "PvdMetaDataExtensions.h"
#include "PvdMetaDataPropertyVisitor.h"
#include "PvdMetaDataDefineProperties.h"
#include "PvdMetaDataBindingData.h"
#include "PxRigidDynamic.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationLink.h"
#include "NpScene.h"
#include "NpPhysics.h"

#include "PvdTypeNames.h"
#include "PvdMetaDataPvdBinding.h"

using namespace physx;
using namespace Sc;
using namespace Vd;
using namespace Sq;

namespace physx
{
namespace Vd
{

struct NameValuePair
{
	const char* mName;
	PxU32 mValue;
};

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreserved-identifier"
#endif

static const NameValuePair g_physx_Sq_SceneQueryID__EnumConversion[] = {
	{ "QUERY_RAYCAST_ANY_OBJECT", PxU32(QueryID::QUERY_RAYCAST_ANY_OBJECT) },
	{ "QUERY_RAYCAST_CLOSEST_OBJECT", PxU32(QueryID::QUERY_RAYCAST_CLOSEST_OBJECT) },
	{ "QUERY_RAYCAST_ALL_OBJECTS", PxU32(QueryID::QUERY_RAYCAST_ALL_OBJECTS) },
	{ "QUERY_OVERLAP_SPHERE_ALL_OBJECTS", PxU32(QueryID::QUERY_OVERLAP_SPHERE_ALL_OBJECTS) },
	{ "QUERY_OVERLAP_AABB_ALL_OBJECTS", PxU32(QueryID::QUERY_OVERLAP_AABB_ALL_OBJECTS) },
	{ "QUERY_OVERLAP_OBB_ALL_OBJECTS", PxU32(QueryID::QUERY_OVERLAP_OBB_ALL_OBJECTS) },
	{ "QUERY_OVERLAP_CAPSULE_ALL_OBJECTS", PxU32(QueryID::QUERY_OVERLAP_CAPSULE_ALL_OBJECTS) },
	{ "QUERY_OVERLAP_CONVEX_ALL_OBJECTS", PxU32(QueryID::QUERY_OVERLAP_CONVEX_ALL_OBJECTS) },
	{ "QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT", PxU32(QueryID::QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT) },
	{ "QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT", PxU32(QueryID::QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT) },
	{ "QUERY_LINEAR_CONVEX_SWEEP_CLOSEST_OBJECT", PxU32(QueryID::QUERY_LINEAR_CONVEX_SWEEP_CLOSEST_OBJECT) },
	{ NULL, 0 }
};

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif

struct SceneQueryIDConvertor
{
	const NameValuePair* NameConversion;
	SceneQueryIDConvertor() : NameConversion(g_physx_Sq_SceneQueryID__EnumConversion)
	{
	}
};

PvdMetaDataBinding::PvdMetaDataBinding() : mBindingData(PX_NEW(PvdMetaDataBindingData)())
{
}

PvdMetaDataBinding::~PvdMetaDataBinding()
{
	for(OwnerActorsMap::Iterator iter = mBindingData->mOwnerActorsMap.getIterator(); !iter.done(); iter++)
	{
		iter->second->~OwnerActorsValueType();
		PX_FREE(iter->second);
	}

	PX_DELETE(mBindingData);
}

template <typename TDataType, typename TValueType, typename TClassType>
static inline void definePropertyStruct(PvdDataStream& inStream, const char* pushName = NULL)
{
	PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
	PvdClassInfoValueStructDefine definitionObj(helper);
	bool doPush = pushName && *pushName;
	if(doPush)
		definitionObj.pushName(pushName);
	visitAllPvdProperties<TDataType>(definitionObj);
	if(doPush)
		definitionObj.popName();
	helper.addPropertyMessage(getPvdNamespacedNameForType<TClassType>(), getPvdNamespacedNameForType<TValueType>(), sizeof(TValueType));
}

template <typename TDataType>
static inline void createClassAndDefineProperties(PvdDataStream& inStream)
{
	inStream.createClass<TDataType>();
	PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
	PvdClassInfoDefine definitionObj(helper, getPvdNamespacedNameForType<TDataType>());
	visitAllPvdProperties<TDataType>(definitionObj);
}

template <typename TDataType, typename TParentType>
static inline void createClassDeriveAndDefineProperties(PvdDataStream& inStream)
{
	inStream.createClass<TDataType>();
	inStream.deriveClass<TParentType, TDataType>();
	PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
	PvdClassInfoDefine definitionObj(helper, getPvdNamespacedNameForType<TDataType>());
	visitInstancePvdProperties<TDataType>(definitionObj);
}

template <typename TDataType, typename TConvertSrc, typename TConvertData>
static inline void defineProperty(PvdDataStream& inStream, const char* inPropertyName, const char* semantic)
{
	PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
	// PxEnumTraits< TValueType > filterFlagsEnum;
	TConvertSrc filterFlagsEnum;
	const TConvertData* convertor = filterFlagsEnum.NameConversion;

	for(; convertor->mName != NULL; ++convertor)
		helper.addNamedValue(convertor->mName, convertor->mValue);

	inStream.createProperty<TDataType, PxU32>(inPropertyName, semantic, PropertyType::Scalar, helper.getNamedValues());
	helper.clearNamedValues();
}

template <typename TDataType, typename TConvertSrc, typename TConvertData>
static inline void definePropertyFlags(PvdDataStream& inStream, const char* inPropertyName)
{
	defineProperty<TDataType, TConvertSrc, TConvertData>(inStream, inPropertyName, "Bitflag");
}
template <typename TDataType, typename TConvertSrc, typename TConvertData>
static inline void definePropertyEnums(PvdDataStream& inStream, const char* inPropertyName)
{
	defineProperty<TDataType, TConvertSrc, TConvertData>(inStream, inPropertyName, "Enumeration Value");
}

static PX_FORCE_INLINE void registerPvdRaycast(PvdDataStream& inStream)
{
	inStream.createClass<PvdRaycast>();
	definePropertyEnums<PvdRaycast, SceneQueryIDConvertor, NameValuePair>(inStream, "type");
	inStream.createProperty<PvdRaycast, PxFilterData>("filterData");
	definePropertyFlags<PvdRaycast, PxEnumTraits<physx::PxQueryFlag::Enum>, PxU32ToName>(inStream, "filterFlags");
	inStream.createProperty<PvdRaycast, PxVec3>("origin");
	inStream.createProperty<PvdRaycast, PxVec3>("unitDir");
	inStream.createProperty<PvdRaycast, PxF32>("distance");
	inStream.createProperty<PvdRaycast, String>("hits_arrayName");
	inStream.createProperty<PvdRaycast, PxU32>("hits_baseIndex");
	inStream.createProperty<PvdRaycast, PxU32>("hits_count");
}

static PX_FORCE_INLINE void registerPvdSweep(PvdDataStream& inStream)
{
	inStream.createClass<PvdSweep>();
	definePropertyEnums<PvdSweep, SceneQueryIDConvertor, NameValuePair>(inStream, "type");
	definePropertyFlags<PvdSweep, PxEnumTraits<physx::PxQueryFlag::Enum>, PxU32ToName>(inStream, "filterFlags");
	inStream.createProperty<PvdSweep, PxVec3>("unitDir");
	inStream.createProperty<PvdSweep, PxF32>("distance");
	inStream.createProperty<PvdSweep, String>("geom_arrayName");
	inStream.createProperty<PvdSweep, PxU32>("geom_baseIndex");
	inStream.createProperty<PvdSweep, PxU32>("geom_count");
	inStream.createProperty<PvdSweep, String>("pose_arrayName");
	inStream.createProperty<PvdSweep, PxU32>("pose_baseIndex");
	inStream.createProperty<PvdSweep, PxU32>("pose_count");
	inStream.createProperty<PvdSweep, String>("filterData_arrayName");
	inStream.createProperty<PvdSweep, PxU32>("filterData_baseIndex");
	inStream.createProperty<PvdSweep, PxU32>("filterData_count");
	inStream.createProperty<PvdSweep, String>("hits_arrayName");
	inStream.createProperty<PvdSweep, PxU32>("hits_baseIndex");
	inStream.createProperty<PvdSweep, PxU32>("hits_count");
}

static PX_FORCE_INLINE void registerPvdOverlap(PvdDataStream& inStream)
{
	inStream.createClass<PvdOverlap>();
	definePropertyEnums<PvdOverlap, SceneQueryIDConvertor, NameValuePair>(inStream, "type");
	inStream.createProperty<PvdOverlap, PxFilterData>("filterData");
	definePropertyFlags<PvdOverlap, PxEnumTraits<physx::PxQueryFlag::Enum>, PxU32ToName>(inStream, "filterFlags");
	inStream.createProperty<PvdOverlap, PxTransform>("pose");
	inStream.createProperty<PvdOverlap, String>("geom_arrayName");
	inStream.createProperty<PvdOverlap, PxU32>("geom_baseIndex");
	inStream.createProperty<PvdOverlap, PxU32>("geom_count");
	inStream.createProperty<PvdOverlap, String>("hits_arrayName");
	inStream.createProperty<PvdOverlap, PxU32>("hits_baseIndex");
	inStream.createProperty<PvdOverlap, PxU32>("hits_count");
}

static PX_FORCE_INLINE void registerPvdSqHit(PvdDataStream& inStream)
{
	inStream.createClass<PvdSqHit>();
	inStream.createProperty<PvdSqHit, ObjectRef>("Shape");
	inStream.createProperty<PvdSqHit, ObjectRef>("Actor");
	inStream.createProperty<PvdSqHit, PxU32>("FaceIndex");
	definePropertyFlags<PvdSqHit, PxEnumTraits<physx::PxHitFlag::Enum>, PxU32ToName>(inStream, "Flags");
	inStream.createProperty<PvdSqHit, PxVec3>("Impact");
	inStream.createProperty<PvdSqHit, PxVec3>("Normal");
	inStream.createProperty<PvdSqHit, PxF32>("Distance");
	inStream.createProperty<PvdSqHit, PxF32>("U");
	inStream.createProperty<PvdSqHit, PxF32>("V");
}

void PvdMetaDataBinding::registerSDKProperties(PvdDataStream& inStream)
{
	if (inStream.isClassExist<PxPhysics>())
		return;
	// PxPhysics
	{
		inStream.createClass<PxPhysics>();
		PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
		PvdClassInfoDefine definitionObj(helper, getPvdNamespacedNameForType<PxPhysics>());
		helper.pushName("TolerancesScale");
		visitAllPvdProperties<PxTolerancesScale>(definitionObj);
		helper.popName();
		inStream.createProperty<PxPhysics, ObjectRef>("Scenes", "children", PropertyType::Array);
		inStream.createProperty<PxPhysics, ObjectRef>("SharedShapes", "children", PropertyType::Array);
		inStream.createProperty<PxPhysics, ObjectRef>("Materials", "children", PropertyType::Array);
		inStream.createProperty<PxPhysics, ObjectRef>("HeightFields", "children", PropertyType::Array);
		inStream.createProperty<PxPhysics, ObjectRef>("ConvexMeshes", "children", PropertyType::Array);
		inStream.createProperty<PxPhysics, ObjectRef>("TriangleMeshes", "children", PropertyType::Array);
		inStream.createProperty<PxPhysics, PxU32>("Version.Major");
		inStream.createProperty<PxPhysics, PxU32>("Version.Minor");
		inStream.createProperty<PxPhysics, PxU32>("Version.Bugfix");
		inStream.createProperty<PxPhysics, String>("Version.Build");
		definePropertyStruct<PxTolerancesScale, PxTolerancesScaleGeneratedValues, PxPhysics>(inStream, "TolerancesScale");
	}
	{ // PxGeometry
		inStream.createClass<PxGeometry>();
		inStream.createProperty<PxGeometry, ObjectRef>("Shape", "parents", PropertyType::Scalar);
	}
	{ // PxBoxGeometry
		createClassDeriveAndDefineProperties<PxBoxGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxBoxGeometry, PxBoxGeometryGeneratedValues, PxBoxGeometry>(inStream);
	}
	{ // PxSphereGeometry
		createClassDeriveAndDefineProperties<PxSphereGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxSphereGeometry, PxSphereGeometryGeneratedValues, PxSphereGeometry>(inStream);
	}
	{ // PxCapsuleGeometry
		createClassDeriveAndDefineProperties<PxCapsuleGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxCapsuleGeometry, PxCapsuleGeometryGeneratedValues, PxCapsuleGeometry>(inStream);
	}
	{ // PxPlaneGeometry
		createClassDeriveAndDefineProperties<PxPlaneGeometry, PxGeometry>(inStream);
	}
	{ // PxConvexMeshGeometry
		createClassDeriveAndDefineProperties<PxConvexMeshGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxConvexMeshGeometry, PxConvexMeshGeometryGeneratedValues, PxConvexMeshGeometry>(inStream);
	}

	{ // PxTetrahedronMeshGeometry
		createClassDeriveAndDefineProperties<PxTetrahedronMeshGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxTetrahedronMeshGeometry, PxTetrahedronMeshGeometryGeneratedValues, PxTetrahedronMeshGeometry>(inStream);
	}

	{ // PxTriangleMeshGeometry
		createClassDeriveAndDefineProperties<PxTriangleMeshGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxTriangleMeshGeometry, PxTriangleMeshGeometryGeneratedValues, PxTriangleMeshGeometry>(inStream);
	}
	{ // PxHeightFieldGeometry
		createClassDeriveAndDefineProperties<PxHeightFieldGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxHeightFieldGeometry, PxHeightFieldGeometryGeneratedValues, PxHeightFieldGeometry>(inStream);
	}
	{ // PxCustomGeometry
		createClassDeriveAndDefineProperties<PxCustomGeometry, PxGeometry>(inStream);
		definePropertyStruct<PxCustomGeometry, PxCustomGeometryGeneratedValues, PxCustomGeometry>(inStream);
	}

	// PxScene
	{
		// PT: TODO: why inline this for PvdContact but do PvdRaycast/etc in separate functions?
		{ // contact information
			inStream.createClass<PvdContact>();
			inStream.createProperty<PvdContact, PxVec3>("Point");
			inStream.createProperty<PvdContact, PxVec3>("Axis");
			inStream.createProperty<PvdContact, ObjectRef>("Shapes[0]");
			inStream.createProperty<PvdContact, ObjectRef>("Shapes[1]");
			inStream.createProperty<PvdContact, PxF32>("Separation");
			inStream.createProperty<PvdContact, PxF32>("NormalForce");
			inStream.createProperty<PvdContact, PxU32>("InternalFaceIndex[0]");
			inStream.createProperty<PvdContact, PxU32>("InternalFaceIndex[1]");
			inStream.createProperty<PvdContact, bool>("NormalForceValid");
		}

		registerPvdSqHit(inStream);
		registerPvdRaycast(inStream);
		registerPvdSweep(inStream);
		registerPvdOverlap(inStream);

		inStream.createClass<PxScene>();
		PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
		PvdClassInfoDefine definitionObj(helper, getPvdNamespacedNameForType<PxScene>());
		visitAllPvdProperties<PxSceneDesc>(definitionObj);
		helper.pushName("SimulationStatistics");
		visitAllPvdProperties<PxSimulationStatistics>(definitionObj);
		helper.popName();
		inStream.createProperty<PxScene, ObjectRef>("Physics", "parents", PropertyType::Scalar);
		inStream.createProperty<PxScene, PxU32>("Timestamp");
		inStream.createProperty<PxScene, PxReal>("SimulateElapsedTime");
		definePropertyStruct<PxSceneDesc, PxSceneDescGeneratedValues, PxScene>(inStream);
		definePropertyStruct<PxSimulationStatistics, PxSimulationStatisticsGeneratedValues, PxScene>(inStream, "SimulationStatistics");
		inStream.createProperty<PxScene, PvdContact>("Contacts", "", PropertyType::Array);

		inStream.createProperty<PxScene, PvdOverlap>("SceneQueries.Overlaps", "", PropertyType::Array);
		inStream.createProperty<PxScene, PvdSweep>("SceneQueries.Sweeps", "", PropertyType::Array);
		inStream.createProperty<PxScene, PvdSqHit>("SceneQueries.Hits", "", PropertyType::Array);
		inStream.createProperty<PxScene, PvdRaycast>("SceneQueries.Raycasts", "", PropertyType::Array);
		inStream.createProperty<PxScene, PxTransform>("SceneQueries.PoseList", "", PropertyType::Array);
		inStream.createProperty<PxScene, PxFilterData>("SceneQueries.FilterDataList", "", PropertyType::Array);
		inStream.createProperty<PxScene, ObjectRef>("SceneQueries.GeometryList", "", PropertyType::Array);

		inStream.createProperty<PxScene, PvdOverlap>("BatchedQueries.Overlaps", "", PropertyType::Array);
		inStream.createProperty<PxScene, PvdSweep>("BatchedQueries.Sweeps", "", PropertyType::Array);
		inStream.createProperty<PxScene, PvdSqHit>("BatchedQueries.Hits", "", PropertyType::Array);
		inStream.createProperty<PxScene, PvdRaycast>("BatchedQueries.Raycasts", "", PropertyType::Array);
		inStream.createProperty<PxScene, PxTransform>("BatchedQueries.PoseList", "", PropertyType::Array);
		inStream.createProperty<PxScene, PxFilterData>("BatchedQueries.FilterDataList", "", PropertyType::Array);
		inStream.createProperty<PxScene, ObjectRef>("BatchedQueries.GeometryList", "", PropertyType::Array);

		inStream.createProperty<PxScene, ObjectRef>("RigidStatics", "children", PropertyType::Array);
		inStream.createProperty<PxScene, ObjectRef>("RigidDynamics", "children", PropertyType::Array);
		inStream.createProperty<PxScene, ObjectRef>("Articulations", "children", PropertyType::Array);
		inStream.createProperty<PxScene, ObjectRef>("Joints", "children", PropertyType::Array);
		inStream.createProperty<PxScene, ObjectRef>("Aggregates", "children", PropertyType::Array);
	}
	// PxMaterial
	{
		createClassAndDefineProperties<PxMaterial>(inStream);
		definePropertyStruct<PxMaterial, PxMaterialGeneratedValues, PxMaterial>(inStream);
		inStream.createProperty<PxMaterial, ObjectRef>("Physics", "parents", PropertyType::Scalar);
	}
	// PxHeightField
	{
		{
			inStream.createClass<PxHeightFieldSample>();
			inStream.createProperty<PxHeightFieldSample, PxU16>("Height");
			inStream.createProperty<PxHeightFieldSample, PxU8>("MaterialIndex[0]");
			inStream.createProperty<PxHeightFieldSample, PxU8>("MaterialIndex[1]");
		}

		inStream.createClass<PxHeightField>();
		// It is important the PVD fields match the RepX fields, so this has
		// to be hand coded.
		PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
		PvdClassInfoDefine definitionObj(helper, getPvdNamespacedNameForType<PxHeightField>());
		visitAllPvdProperties<PxHeightFieldDesc>(definitionObj);
		inStream.createProperty<PxHeightField, PxHeightFieldSample>("Samples", "", PropertyType::Array);
		inStream.createProperty<PxHeightField, ObjectRef>("Physics", "parents", PropertyType::Scalar);
		definePropertyStruct<PxHeightFieldDesc, PxHeightFieldDescGeneratedValues, PxHeightField>(inStream);
	}
	// PxConvexMesh
	{
		{ // hull polygon data.
			inStream.createClass<PvdHullPolygonData>();
			inStream.createProperty<PvdHullPolygonData, PxU16>("NumVertices");
			inStream.createProperty<PvdHullPolygonData, PxU16>("IndexBase");
		}
		inStream.createClass<PxConvexMesh>();
		inStream.createProperty<PxConvexMesh, PxF32>("Mass");
		inStream.createProperty<PxConvexMesh, PxMat33>("LocalInertia");
		inStream.createProperty<PxConvexMesh, PxVec3>("LocalCenterOfMass");
		inStream.createProperty<PxConvexMesh, PxVec3>("Points", "", PropertyType::Array);
		inStream.createProperty<PxConvexMesh, PvdHullPolygonData>("HullPolygons", "", PropertyType::Array);
		inStream.createProperty<PxConvexMesh, PxU8>("PolygonIndexes", "", PropertyType::Array);
		inStream.createProperty<PxConvexMesh, ObjectRef>("Physics", "parents", PropertyType::Scalar);
	}
	// PxTriangleMesh
	{
		inStream.createClass<PxTriangleMesh>();
		inStream.createProperty<PxTriangleMesh, PxVec3>("Points", "", PropertyType::Array);
		inStream.createProperty<PxTriangleMesh, PxU32>("NbTriangles", "", PropertyType::Scalar);
		inStream.createProperty<PxTriangleMesh, PxU32>("Triangles", "", PropertyType::Array);
		inStream.createProperty<PxTriangleMesh, PxU16>("MaterialIndices", "", PropertyType::Array);
		inStream.createProperty<PxTriangleMesh, ObjectRef>("Physics", "parents", PropertyType::Scalar);
	}

	// PxTetrahedronMesh
	{
		inStream.createClass<PxTetrahedronMesh>();
		inStream.createProperty<PxTetrahedronMesh, PxVec3>("Points", "", PropertyType::Array);
		inStream.createProperty<PxTetrahedronMesh, PxU32>("NbTriangles", "", PropertyType::Scalar);
		inStream.createProperty<PxTetrahedronMesh, PxU32>("Triangles", "", PropertyType::Array);
		inStream.createProperty<PxTetrahedronMesh, ObjectRef>("Physics", "parents", PropertyType::Scalar);
	}

	// PxFEMSoftBodyMaterial
	{
		createClassAndDefineProperties<PxFEMSoftBodyMaterial>(inStream);
		definePropertyStruct<PxFEMSoftBodyMaterial, PxFEMSoftBodyMaterialGeneratedValues, PxFEMSoftBodyMaterial>(inStream);
		inStream.createProperty<PxFEMSoftBodyMaterial, ObjectRef>("Physics", "parents", PropertyType::Scalar);
	}

	// PxFEMClothMaterial
	// jcarius: Commented-out until FEMCloth is not under construction anymore
	// {
	// 	createClassAndDefineProperties<PxFEMClothMaterial>(inStream);
	// 	definePropertyStruct<PxFEMClothMaterial, PxFEMClothMaterialGeneratedValues, PxFEMClothMaterial>(inStream);
	// 	inStream.createProperty<PxFEMClothMaterial, ObjectRef>("Physics", "parents", PropertyType::Scalar);
	// }

	{ // PxShape
		createClassAndDefineProperties<PxShape>(inStream);
		definePropertyStruct<PxShape, PxShapeGeneratedValues, PxShape>(inStream);
		inStream.createProperty<PxShape, ObjectRef>("Geometry", "children");
		inStream.createProperty<PxShape, ObjectRef>("Materials", "children", PropertyType::Array);
		inStream.createProperty<PxShape, ObjectRef>("Actor", "parents");
	}
	// PxActor
	{
		createClassAndDefineProperties<PxActor>(inStream);
		inStream.createProperty<PxActor, ObjectRef>("Scene", "parents");
	}
	// PxRigidActor
	{
		createClassDeriveAndDefineProperties<PxRigidActor, PxActor>(inStream);
		inStream.createProperty<PxRigidActor, ObjectRef>("Shapes", "children", PropertyType::Array);
		inStream.createProperty<PxRigidActor, ObjectRef>("Joints", "children", PropertyType::Array);
	}
	// PxRigidStatic
	{
		createClassDeriveAndDefineProperties<PxRigidStatic, PxRigidActor>(inStream);
		definePropertyStruct<PxRigidStatic, PxRigidStaticGeneratedValues, PxRigidStatic>(inStream);
	}
	{ // PxRigidBody
		createClassDeriveAndDefineProperties<PxRigidBody, PxRigidActor>(inStream);
	}
	// PxRigidDynamic
	{
		createClassDeriveAndDefineProperties<PxRigidDynamic, PxRigidBody>(inStream);
		// If anyone adds a 'getKinematicTarget' to PxRigidDynamic you can remove the line
		// below (after the code generator has run).
		inStream.createProperty<PxRigidDynamic, PxTransform>("KinematicTarget");
		definePropertyStruct<PxRigidDynamic, PxRigidDynamicGeneratedValues, PxRigidDynamic>(inStream);
		// Manually define the update struct.
		PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
		/*struct PxRigidDynamicUpdateBlock
		{
		    Transform	GlobalPose;
		    Float3		LinearVelocity;
		    Float3		AngularVelocity;
		    PxU8		IsSleeping;
		    PxU8		padding[3];
		};
		*/
		helper.pushName("GlobalPose");
		helper.addPropertyMessageArg<PxTransform>(PX_OFFSET_OF_RT(PxRigidDynamicUpdateBlock, GlobalPose));
		helper.popName();
		helper.pushName("LinearVelocity");
		helper.addPropertyMessageArg<PxVec3>(PX_OFFSET_OF_RT(PxRigidDynamicUpdateBlock, LinearVelocity));
		helper.popName();
		helper.pushName("AngularVelocity");
		helper.addPropertyMessageArg<PxVec3>(PX_OFFSET_OF_RT(PxRigidDynamicUpdateBlock, AngularVelocity));
		helper.popName();
		helper.pushName("IsSleeping");
		helper.addPropertyMessageArg<bool>(PX_OFFSET_OF_RT(PxRigidDynamicUpdateBlock, IsSleeping));
		helper.popName();
		helper.addPropertyMessage<PxRigidDynamic, PxRigidDynamicUpdateBlock>();
	}

	// PxArticulationReducedCoordinate
	{
		createClassAndDefineProperties<PxArticulationReducedCoordinate>(inStream);
		inStream.createProperty<PxArticulationReducedCoordinate, ObjectRef>("Scene", "parents");
		inStream.createProperty<PxArticulationReducedCoordinate, ObjectRef>("Links", "children", PropertyType::Array);
		definePropertyStruct<PxArticulationReducedCoordinate, PxArticulationReducedCoordinateGeneratedValues, PxArticulationReducedCoordinate>(inStream);
	}

	{ // PxArticulationLink
		createClassDeriveAndDefineProperties<PxArticulationLink, PxRigidBody>(inStream);
		inStream.createProperty<PxArticulationLink, ObjectRef>("Parent", "parents");
		inStream.createProperty<PxArticulationLink, ObjectRef>("Links", "children", PropertyType::Array);
		inStream.createProperty<PxArticulationLink, ObjectRef>("InboundJoint", "children");
		definePropertyStruct<PxArticulationLink, PxArticulationLinkGeneratedValues, PxArticulationLink>(inStream);

		PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());
		/*struct PxArticulationLinkUpdateBlock
		{
		    Transform	GlobalPose;
		    Float3		LinearVelocity;
		    Float3		AngularVelocity;
		};*/
		
		helper.pushName("GlobalPose");
		helper.addPropertyMessageArg<PxTransform>(PX_OFFSET_OF(PxArticulationLinkUpdateBlock, GlobalPose));
		helper.popName();
		helper.pushName("LinearVelocity");
		helper.addPropertyMessageArg<PxVec3>(PX_OFFSET_OF(PxArticulationLinkUpdateBlock, LinearVelocity));
		helper.popName();
		helper.pushName("AngularVelocity");
		helper.addPropertyMessageArg<PxVec3>(PX_OFFSET_OF(PxArticulationLinkUpdateBlock, AngularVelocity));
		helper.popName();
		helper.addPropertyMessage<PxArticulationLink, PxArticulationLinkUpdateBlock>();
		
	}
	{ // PxArticulationJoint
		createClassAndDefineProperties<PxArticulationJointReducedCoordinate>(inStream);
		inStream.createProperty<PxArticulationJointReducedCoordinate, ObjectRef>("Link", "parents");
		definePropertyStruct<PxArticulationJointReducedCoordinate, PxArticulationJointReducedCoordinateGeneratedValues, PxArticulationJointReducedCoordinate>(inStream);

		PvdPropertyDefinitionHelper& helper(inStream.getPropertyDefinitionHelper());

		helper.pushName("JointPosition[eX]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointPosition_eX));
		helper.popName();
		helper.pushName("JointPosition[eY]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointPosition_eY));
		helper.popName();
		helper.pushName("JointPosition[eZ]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointPosition_eZ));
		helper.popName();
		helper.pushName("JointPosition[eTWIST]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointPosition_eTwist));
		helper.popName();
		helper.pushName("JointPosition[eSWING1]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointPosition_eSwing1));
		helper.popName();
		helper.pushName("JointPosition[eSWING2]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointPosition_eSwing2));
		helper.popName();

		helper.pushName("JointVelocity[eX]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointVelocity_eX));
		helper.popName();
		helper.pushName("JointVelocity[eY]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointVelocity_eY));
		helper.popName();
		helper.pushName("JointVelocity[eZ]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointVelocity_eZ));
		helper.popName();
		helper.pushName("JointVelocity[eTWIST]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointVelocity_eTwist));
		helper.popName();
		helper.pushName("JointVelocity[eSWING1]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointVelocity_eSwing1));
		helper.popName();
		helper.pushName("JointVelocity[eSWING2]");
		helper.addPropertyMessageArg<PxReal>(PX_OFFSET_OF(PxArticulationJointUpdateBlock, JointVelocity_eSwing2));
		helper.popName();

		helper.addPropertyMessage<PxArticulationJointReducedCoordinate, PxArticulationJointUpdateBlock>();
	}
	{ // PxConstraint
		createClassAndDefineProperties<PxConstraint>(inStream);
		definePropertyStruct<PxConstraint, PxConstraintGeneratedValues, PxConstraint>(inStream);
	}
	{
		// PxAggregate
		createClassAndDefineProperties<PxAggregate>(inStream);
		inStream.createProperty<PxAggregate, ObjectRef>("Scene", "parents");
		definePropertyStruct<PxAggregate, PxAggregateGeneratedValues, PxAggregate>(inStream);
		inStream.createProperty<PxAggregate, ObjectRef>("Actors", "children", PropertyType::Array);
		inStream.createProperty<PxAggregate, ObjectRef>("Articulations", "children", PropertyType::Array);
	}
}

template <typename TClassType, typename TValueType, typename TDataType>
static void doSendAllProperties(PvdDataStream& inStream, const TDataType* inDatatype, const void* instanceId)
{
	TValueType theValues(inDatatype);
	inStream.setPropertyMessage(instanceId, theValues);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxPhysics& inPhysics)
{
	PxTolerancesScale theScale(inPhysics.getTolerancesScale());
	doSendAllProperties<PxPhysics, PxTolerancesScaleGeneratedValues>(inStream, &theScale, &inPhysics);
	inStream.setPropertyValue(&inPhysics, "Version.Major", PxU32(PX_PHYSICS_VERSION_MAJOR));
	inStream.setPropertyValue(&inPhysics, "Version.Minor", PxU32(PX_PHYSICS_VERSION_MINOR));
	inStream.setPropertyValue(&inPhysics, "Version.Bugfix", PxU32(PX_PHYSICS_VERSION_BUGFIX));

#if PX_CHECKED
#if defined(NDEBUG)
	// This is a checked build
	String buildType = "Checked";
#elif defined(_DEBUG)
	// This is a debug build
	String buildType = "Debug";
#endif
#elif PX_PROFILE
	String buildType = "Profile";
#elif defined(NDEBUG)
	// This is a release build
	String buildType = "Release";
#endif
	inStream.setPropertyValue(&inPhysics, "Version.Build", buildType);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxScene& inScene)
{
	const PxPhysics& physics(const_cast<PxScene&>(inScene).getPhysics());
	PxTolerancesScale theScale;
	PxSceneDesc theDesc(theScale);

	{
		// setDominanceGroupPair ?

		theDesc.gravity							= inScene.getGravity();
		theDesc.simulationEventCallback			= inScene.getSimulationEventCallback();
		theDesc.contactModifyCallback			= inScene.getContactModifyCallback();
		theDesc.ccdContactModifyCallback		= inScene.getCCDContactModifyCallback();
		theDesc.filterShaderData				= inScene.getFilterShaderData();
		theDesc.filterShaderDataSize			= inScene.getFilterShaderDataSize();
		theDesc.filterShader					= inScene.getFilterShader();
		theDesc.filterCallback					= inScene.getFilterCallback();
//		PxPairFilteringMode::Enum KineKineFilteringMode;
//		PxPairFilteringMode::Enum StaticKineFilteringMode;
		theDesc.broadPhaseType					= inScene.getBroadPhaseType();
		theDesc.broadPhaseCallback				= inScene.getBroadPhaseCallback();
		theDesc.limits							= inScene.getLimits();
		theDesc.frictionType					= inScene.getFrictionType();
//		PxSolverType::Enum SolverType;
		theDesc.bounceThresholdVelocity			= inScene.getBounceThresholdVelocity();
		theDesc.frictionOffsetThreshold			= inScene.getFrictionOffsetThreshold();
		theDesc.frictionCorrelationDistance		= inScene.getFrictionCorrelationDistance();
		theDesc.flags							= inScene.getFlags();
		theDesc.cpuDispatcher					= inScene.getCpuDispatcher();
		theDesc.cudaContextManager				= inScene.getCudaContextManager();
		theDesc.staticStructure					= inScene.getStaticStructure();
		theDesc.dynamicStructure				= inScene.getDynamicStructure();
		theDesc.dynamicTreeRebuildRateHint		= inScene.getDynamicTreeRebuildRateHint();
		theDesc.sceneQueryUpdateMode			= inScene.getSceneQueryUpdateMode();
		theDesc.userData						= inScene.userData;
		theDesc.solverBatchSize					= inScene.getSolverBatchSize();
		theDesc.solverArticulationBatchSize		= inScene.getSolverArticulationBatchSize();
//		PxU32 NbContactDataBlocks;
//		PxU32 MaxNbContactDataBlocks;
		// theDesc.nbContactDataBlocks			= inScene.getNbContactDataBlocksUsed();
		// theDesc.maxNbContactDataBlocks		= inScene.getMaxNbContactDataBlocksUsed();
		theDesc.maxBiasCoefficient				= inScene.getMaxBiasCoefficient();
		theDesc.contactReportStreamBufferSize	= inScene.getContactReportStreamBufferSize();
		theDesc.ccdMaxPasses					= inScene.getCCDMaxPasses();
		theDesc.ccdThreshold					= inScene.getCCDThreshold();
		theDesc.ccdMaxSeparation				= inScene.getCCDMaxSeparation();
		// theDesc.simulationOrder				= inScene.getSimulationOrder();
		theDesc.wakeCounterResetValue			= inScene.getWakeCounterResetValue();

		theDesc.gpuDynamicsConfig				= inScene.getGpuDynamicsConfig();
//		PxBounds3 SanityBounds;
//		PxgDynamicsMemoryConfig GpuDynamicsConfig;
//		PxU32 GpuMaxNumPartitions;
//		PxU32 GpuComputeVersion;
//		PxReal BroadPhaseInflation;
//		PxU32 ContactPairSlabSize;
	}

	PxSceneDescGeneratedValues theValues(&theDesc);
	inStream.setPropertyMessage(&inScene, theValues);
	// Create parent/child relationship.
	inStream.setPropertyValue(&inScene, "Physics", reinterpret_cast<const void*>(&physics));
	inStream.pushBackObjectRef(&physics, "Scenes", &inScene);
}

void PvdMetaDataBinding::sendBeginFrame(PvdDataStream& inStream, const PxScene* inScene, PxReal simulateElapsedTime)
{
	inStream.beginSection(inScene, "frame");
	inStream.setPropertyValue(inScene, "Timestamp", inScene->getTimestamp());
	inStream.setPropertyValue(inScene, "SimulateElapsedTime", simulateElapsedTime);
}

template <typename TDataType>
struct NullConverter
{
	void operator()(TDataType& data, const TDataType& src)
	{
		data = src;
	}
};

template <typename TTargetType, PxU32 T_NUM_ITEMS, typename TSourceType = TTargetType,
          typename Converter = NullConverter<TTargetType> >
class ScopedPropertyValueSender
{
	TTargetType			mStack[T_NUM_ITEMS];
	TTargetType*		mCur;
	const TTargetType*	mEnd;
	PvdDataStream&		mStream;

  public:
	ScopedPropertyValueSender(PvdDataStream& inStream, const void* inObj, String name)
	: mCur(mStack), mEnd(&mStack[T_NUM_ITEMS]), mStream(inStream)
	{
		mStream.beginSetPropertyValue(inObj, name, getPvdNamespacedNameForType<TTargetType>());
	}

	~ScopedPropertyValueSender()
	{
		if(mStack != mCur)
		{
			PxU32 size = sizeof(TTargetType) * PxU32(mCur - mStack);
			mStream.appendPropertyValueData(DataRef<const PxU8>(reinterpret_cast<PxU8*>(mStack), size));
		}
		mStream.endSetPropertyValue();
	}

	void append(const TSourceType& data)
	{
		Converter()(*mCur, data);
		if(mCur < mEnd - 1)
			++mCur;
		else
		{
			mStream.appendPropertyValueData(DataRef<const PxU8>(reinterpret_cast<PxU8*>(mStack), sizeof mStack));
			mCur = mStack;
		}
	}

  private:
	ScopedPropertyValueSender& operator=(const ScopedPropertyValueSender&);
};

void PvdMetaDataBinding::sendContacts(PvdDataStream& inStream, const PxScene& inScene)
{
	inStream.setPropertyValue(&inScene, "Contacts", DataRef<const PxU8>(), getPvdNamespacedNameForType<PvdContact>());
}

void PvdMetaDataBinding::sendStats(PvdDataStream& inStream, const PxScene* inScene)
{
	PxSimulationStatistics theStats;
	inScene->getSimulationStatistics(theStats);

	PxSimulationStatisticsGeneratedValues values(&theStats);
	inStream.setPropertyMessage(inScene, values);
}

struct PvdContactConverter
{
	void operator()(PvdContact& data, const Sc::Contact& src)
	{
		data.point = src.point;
		data.axis = src.normal;
		data.shape0 = src.shape0;
		data.shape1 = src.shape1;
		data.separation = src.separation;
		data.normalForce = src.normalForce;
		data.internalFaceIndex0 = src.faceIndex0;
		data.internalFaceIndex1 = src.faceIndex1;
		data.normalForceAvailable = src.normalForceAvailable;
	}
};

void PvdMetaDataBinding::sendContacts(PvdDataStream& inStream, const PxScene& inScene, PxArray<Contact>& inContacts)
{
	ScopedPropertyValueSender<PvdContact, 32, Sc::Contact, PvdContactConverter> sender(inStream, &inScene, "Contacts");

	for(PxU32 i = 0; i < inContacts.size(); i++)
	{
		sender.append(inContacts[i]);
	}
}

void PvdMetaDataBinding::sendEndFrame(PvdDataStream& inStream, const PxScene* inScene)
{
	//flush other client
	inStream.endSection(inScene, "frame");
}

template <typename TDataType>
static void addPhysicsGroupProperty(PvdDataStream& inStream, const char* groupName, const TDataType& inData, const PxPhysics& ownerPhysics)
{
	inStream.setPropertyValue(&inData, "Physics", reinterpret_cast<const void*>(&ownerPhysics));
	inStream.pushBackObjectRef(&ownerPhysics, groupName, &inData);
	// Buffer type objects *have* to be flushed directly out once created else scene creation doesn't work.
}

template <typename TDataType>
static void removePhysicsGroupProperty(PvdDataStream& inStream, const char* groupName, const TDataType& inData, const PxPhysics& ownerPhysics)
{
	inStream.removeObjectRef(&ownerPhysics, groupName, &inData);
	inStream.destroyInstance(&inData);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxMaterial& inMaterial, const PxPhysics& ownerPhysics)
{
	inStream.createInstance(&inMaterial);
	sendAllProperties(inStream, inMaterial);
	addPhysicsGroupProperty(inStream, "Materials", inMaterial, ownerPhysics);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxMaterial& inMaterial)
{
	PxMaterialGeneratedValues values(&inMaterial);
	inStream.setPropertyMessage(&inMaterial, values);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxMaterial& inMaterial, const PxPhysics& ownerPhysics)
{
	removePhysicsGroupProperty(inStream, "Materials", inMaterial, ownerPhysics);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxFEMSoftBodyMaterial& inMaterial, const PxPhysics& ownerPhysics)
{
	inStream.createInstance(&inMaterial);
	sendAllProperties(inStream, inMaterial);
	addPhysicsGroupProperty(inStream, "FEMMaterials", inMaterial, ownerPhysics);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& /*inStream*/, const PxFEMSoftBodyMaterial& /*inMaterial*/)
{
	/*PxMaterialGeneratedValues values(&inMaterial);
	inStream.setPropertyMessage(&inMaterial, values);*/
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& /*inStream*/, const PxFEMSoftBodyMaterial& /*inMaterial*/, const PxPhysics& /*ownerPhysics*/)
{
	//removePhysicsGroupProperty(inStream, "Materials", inMaterial, ownerPhysics);
}
// jcarius: Commented-out until FEMCloth is not under construction anymore
// void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxFEMClothMaterial& inMaterial, const PxPhysics& ownerPhysics)
// {
// 	inStream.createInstance(&inMaterial);
// 	sendAllProperties(inStream, inMaterial);
// 	addPhysicsGroupProperty(inStream, "FEMMaterials", inMaterial, ownerPhysics);
// }

// void PvdMetaDataBinding::sendAllProperties(PvdDataStream& /*inStream*/, const PxFEMClothMaterial& /*inMaterial*/)
// {
// 	/*PxMaterialGeneratedValues values(&inMaterial);
// 	inStream.setPropertyMessage(&inMaterial, values);*/
// }

// void PvdMetaDataBinding::destroyInstance(PvdDataStream& /*inStream*/, const PxFEMClothMaterial& /*inMaterial*/, const PxPhysics& /*ownerPhysics*/)
// {
// 	//removePhysicsGroupProperty(inStream, "Materials", inMaterial, ownerPhysics);
// }

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxPBDMaterial& inMaterial, const PxPhysics& ownerPhysics)
{
	inStream.createInstance(&inMaterial);
	sendAllProperties(inStream, inMaterial);
	addPhysicsGroupProperty(inStream, "PBDMaterials", inMaterial, ownerPhysics);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& /*inStream*/, const PxPBDMaterial& /*inMaterial*/)
{
	/*PxMaterialGeneratedValues values(&inMaterial);
	inStream.setPropertyMessage(&inMaterial, values);*/
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& /*inStream*/, const PxPBDMaterial& /*inMaterial*/, const PxPhysics& /*ownerPhysics*/)
{
	//removePhysicsGroupProperty(inStream, "Materials", inMaterial, ownerPhysics);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxHeightField& inData)
{
	PxHeightFieldDesc theDesc;
	// Save the height field to desc.
	theDesc.nbRows = inData.getNbRows();
	theDesc.nbColumns = inData.getNbColumns();
	theDesc.format = inData.getFormat();
	theDesc.samples.stride = inData.getSampleStride();
	theDesc.samples.data = NULL;
	theDesc.convexEdgeThreshold = inData.getConvexEdgeThreshold();
	theDesc.flags = inData.getFlags();

	PxU32 theCellCount = inData.getNbRows() * inData.getNbColumns();
	PxU32 theSampleStride = sizeof(PxHeightFieldSample);
	PxU32 theSampleBufSize = theCellCount * theSampleStride;
	mBindingData->mTempU8Array.resize(theSampleBufSize);
	PxHeightFieldSample* theSamples = reinterpret_cast<PxHeightFieldSample*>(mBindingData->mTempU8Array.begin());
	inData.saveCells(theSamples, theSampleBufSize);
	theDesc.samples.data = theSamples;
	PxHeightFieldDescGeneratedValues values(&theDesc);
	inStream.setPropertyMessage(&inData, values);
	PxHeightFieldSample* theSampleData = reinterpret_cast<PxHeightFieldSample*>(mBindingData->mTempU8Array.begin());
	inStream.setPropertyValue(&inData, "Samples", theSampleData, theCellCount);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxHeightField& inData, const PxPhysics& ownerPhysics)
{
	inStream.createInstance(&inData);
	sendAllProperties(inStream, inData);
	addPhysicsGroupProperty(inStream, "HeightFields", inData, ownerPhysics);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxHeightField& inData, const PxPhysics& ownerPhysics)
{
	removePhysicsGroupProperty(inStream, "HeightFields", inData, ownerPhysics);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxConvexMesh& inData, const PxPhysics& ownerPhysics)
{
	inStream.createInstance(&inData);
	PxReal mass;
	PxMat33 localInertia;
	PxVec3 localCom;
	inData.getMassInformation(mass, localInertia, localCom);
	inStream.setPropertyValue(&inData, "Mass", mass);
	inStream.setPropertyValue(&inData, "LocalInertia", localInertia);
	inStream.setPropertyValue(&inData, "LocalCenterOfMass", localCom);

	// update arrays:
	// vertex Array:
	{
		const PxVec3* vertexPtr = inData.getVertices();
		const PxU32 numVertices = inData.getNbVertices();
		inStream.setPropertyValue(&inData, "Points", vertexPtr, numVertices);
	}

	// HullPolyArray:
	PxU16 maxIndices = 0;
	{

		PxU32 numPolygons = inData.getNbPolygons();
		PvdHullPolygonData* tempData = mBindingData->allocateTemp<PvdHullPolygonData>(numPolygons);
		// Get the polygon data stripping the plane equations
		for(PxU32 index = 0; index < numPolygons; index++)
		{
			PxHullPolygon curOut;
			inData.getPolygonData(index, curOut);
			maxIndices = PxMax(maxIndices, PxU16(curOut.mIndexBase + curOut.mNbVerts));
			tempData[index].mIndexBase = curOut.mIndexBase;
			tempData[index].mNumVertices = curOut.mNbVerts;
		}
		inStream.setPropertyValue(&inData, "HullPolygons", tempData, numPolygons);
	}

	// poly index Array:
	{
		const PxU8* indices = inData.getIndexBuffer();
		inStream.setPropertyValue(&inData, "PolygonIndexes", indices, maxIndices);
	}
	addPhysicsGroupProperty(inStream, "ConvexMeshes", inData, ownerPhysics);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxConvexMesh& inData, const PxPhysics& ownerPhysics)
{
	removePhysicsGroupProperty(inStream, "ConvexMeshes", inData, ownerPhysics);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxTetrahedronMesh& inData, const PxPhysics& ownerPhysics)
{
	inStream.createInstance(&inData);
	
	// vertex Array:
	{
		const PxVec3* vertexPtr = inData.getVertices();
		const PxU32 numVertices = inData.getNbVertices();
		inStream.setPropertyValue(&inData, "Vertices", vertexPtr, numVertices);
	}

	////invert mass array
	//{
	//	const float* invMassPtr = inData.getVertInvMasses();
	//	const PxU32 numVertices = inData.getCollisionMesh().getNbVertices();
	//	inStream.setPropertyValue(&inData, "invert mass", invMassPtr, numVertices);
	//}

	// index Array:
	{
		const bool has16BitIndices = inData.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES ? true : false;
		const PxU32 numTetrahedrons= inData.getNbTetrahedrons();

		inStream.setPropertyValue(&inData, "NbTetrahedron", numTetrahedrons);

		const PxU32 numIndexes = numTetrahedrons * 4;
		const PxU8* tetrahedronsPtr = reinterpret_cast<const PxU8*>(inData.getTetrahedrons());
		// We declared this type as a 32 bit integer above.
		// PVD will automatically unsigned-extend data that is smaller than the target type.
		if (has16BitIndices)
			inStream.setPropertyValue(&inData, "Tetrahedrons", reinterpret_cast<const PxU16*>(tetrahedronsPtr), numIndexes);
		else
			inStream.setPropertyValue(&inData, "Tetrahedrons", reinterpret_cast<const PxU32*>(tetrahedronsPtr), numIndexes);
	}

	addPhysicsGroupProperty(inStream, "TetrahedronMeshes", inData, ownerPhysics);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxTetrahedronMesh& inData, const PxPhysics& ownerPhysics)
{
	removePhysicsGroupProperty(inStream, "TetrahedronMeshes", inData, ownerPhysics);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxTriangleMesh& inData, const PxPhysics& ownerPhysics)
{
	inStream.createInstance(&inData);
	bool hasMatIndex = inData.getTriangleMaterialIndex(0) != 0xffff;
	// update arrays:
	// vertex Array:
	{
		const PxVec3* vertexPtr = inData.getVertices();
		const PxU32 numVertices = inData.getNbVertices();
		inStream.setPropertyValue(&inData, "Points", vertexPtr, numVertices);
	}

	// index Array:
	{
		const bool has16BitIndices = inData.getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES ? true : false;
		const PxU32 numTriangles = inData.getNbTriangles();

		inStream.setPropertyValue(&inData, "NbTriangles", numTriangles);

		const PxU32 numIndexes = numTriangles * 3;
		const PxU8* trianglePtr = reinterpret_cast<const PxU8*>(inData.getTriangles());
		// We declared this type as a 32 bit integer above.
		// PVD will automatically unsigned-extend data that is smaller than the target type.
		if(has16BitIndices)
			inStream.setPropertyValue(&inData, "Triangles", reinterpret_cast<const PxU16*>(trianglePtr), numIndexes);
		else
			inStream.setPropertyValue(&inData, "Triangles", reinterpret_cast<const PxU32*>(trianglePtr), numIndexes);
	}

	// material Array:
	if(hasMatIndex)
	{
		PxU32 numMaterials = inData.getNbTriangles();
		PxU16* matIndexData = mBindingData->allocateTemp<PxU16>(numMaterials);
		for(PxU32 m = 0; m < numMaterials; m++)
			matIndexData[m] = inData.getTriangleMaterialIndex(m);
		inStream.setPropertyValue(&inData, "MaterialIndices", matIndexData, numMaterials);
	}
	addPhysicsGroupProperty(inStream, "TriangleMeshes", inData, ownerPhysics);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxTriangleMesh& inData, const PxPhysics& ownerPhysics)
{
	removePhysicsGroupProperty(inStream, "TriangleMeshes", inData, ownerPhysics);
}

template <typename TDataType>
void PvdMetaDataBinding::registrarPhysicsObject(PvdDataStream&, const TDataType&, PsPvd*)
{
}

template <>
void PvdMetaDataBinding::registrarPhysicsObject<PxConvexMeshGeometry>(PvdDataStream& inStream, const PxConvexMeshGeometry& geom, PsPvd* pvd)
{
	if(pvd->registerObject(geom.convexMesh))
		createInstance(inStream, *geom.convexMesh, PxGetPhysics());
}

template <>
void PvdMetaDataBinding::registrarPhysicsObject<PxTetrahedronMeshGeometry>(PvdDataStream& inStream, const PxTetrahedronMeshGeometry& geom, PsPvd* pvd)
{
	if (pvd->registerObject(geom.tetrahedronMesh))
		createInstance(inStream, *geom.tetrahedronMesh, PxGetPhysics());
}

template <>
void PvdMetaDataBinding::registrarPhysicsObject<PxTriangleMeshGeometry>(PvdDataStream& inStream, const PxTriangleMeshGeometry& geom, PsPvd* pvd)
{
	if(pvd->registerObject(geom.triangleMesh))
		createInstance(inStream, *geom.triangleMesh, PxGetPhysics());
}

template <>
void PvdMetaDataBinding::registrarPhysicsObject<PxHeightFieldGeometry>(PvdDataStream& inStream, const PxHeightFieldGeometry& geom, PsPvd* pvd)
{
	if(pvd->registerObject(geom.heightField))
		createInstance(inStream, *geom.heightField, PxGetPhysics());
}

template <typename TGeneratedValuesType, typename TGeomType>
static void sendGeometry(PvdMetaDataBinding& metaBind, PvdDataStream& inStream, const PxShape& inShape, const TGeomType& geom, PsPvd* pvd)
{
	const void* geomInst = (reinterpret_cast<const PxU8*>(&inShape)) + 4;
	inStream.createInstance(getPvdNamespacedNameForType<TGeomType>(), geomInst);
	metaBind.registrarPhysicsObject<TGeomType>(inStream, geom, pvd);
	TGeneratedValuesType values(&geom);
	inStream.setPropertyMessage(geomInst, values);
	inStream.setPropertyValue(&inShape, "Geometry", geomInst);
	inStream.setPropertyValue(geomInst, "Shape", reinterpret_cast<const void*>(&inShape));
}

static void setGeometry(PvdMetaDataBinding& metaBind, PvdDataStream& inStream, const PxShape& inObj, PsPvd* pvd)
{
	const PxGeometry& geom = inObj.getGeometry();
	switch(geom.getType())
	{
#define SEND_PVD_GEOM_TYPE(enumType, geomType, valueType)                           \
	case PxGeometryType::enumType:                                                  \
	{                                                                               \
		Px##geomType geomT = static_cast<const Px##geomType&>(geom);                \
		sendGeometry<valueType>(metaBind, inStream, inObj, geomT, pvd);             \
	}                                                                               \
	break;
		SEND_PVD_GEOM_TYPE(eSPHERE, SphereGeometry, PxSphereGeometryGeneratedValues);
	// Plane geometries don't have any properties, so this avoids using a property
	// struct for them.
	case PxGeometryType::ePLANE:
	{
		const void* geomInst = (reinterpret_cast<const PxU8*>(&inObj)) + 4;
		inStream.createInstance(getPvdNamespacedNameForType<PxPlaneGeometry>(), geomInst);
		inStream.setPropertyValue(&inObj, "Geometry", geomInst);
		inStream.setPropertyValue(geomInst, "Shape", reinterpret_cast<const void*>(&inObj));
	}
	break;
		SEND_PVD_GEOM_TYPE(eCAPSULE, CapsuleGeometry, PxCapsuleGeometryGeneratedValues);
		SEND_PVD_GEOM_TYPE(eBOX, BoxGeometry, PxBoxGeometryGeneratedValues);
		SEND_PVD_GEOM_TYPE(eCONVEXMESH, ConvexMeshGeometry, PxConvexMeshGeometryGeneratedValues);
		SEND_PVD_GEOM_TYPE(eTETRAHEDRONMESH, TetrahedronMeshGeometry, PxTetrahedronMeshGeometryGeneratedValues);
		SEND_PVD_GEOM_TYPE(eTRIANGLEMESH, TriangleMeshGeometry, PxTriangleMeshGeometryGeneratedValues);
		SEND_PVD_GEOM_TYPE(eHEIGHTFIELD, HeightFieldGeometry, PxHeightFieldGeometryGeneratedValues);		
		SEND_PVD_GEOM_TYPE(eCUSTOM, CustomGeometry, PxCustomGeometryGeneratedValues);
#undef SEND_PVD_GEOM_TYPE
	case PxGeometryType::ePARTICLESYSTEM:
		// A.B. implement later
		break;
	case PxGeometryType::eHAIRSYSTEM:
		break;
	case PxGeometryType::eGEOMETRY_COUNT:
	case PxGeometryType::eINVALID:
		PX_ASSERT(false);
		break;
	}
}

static void setMaterials(PvdMetaDataBinding& metaBing, PvdDataStream& inStream, const PxShape& inObj, PsPvd* pvd, PvdMetaDataBindingData* mBindingData)
{
	PxU32 numMaterials = inObj.getNbMaterials();
	PxMaterial** materialPtr = mBindingData->allocateTemp<PxMaterial*>(numMaterials);
	inObj.getMaterials(materialPtr, numMaterials);
	for(PxU32 idx = 0; idx < numMaterials; ++idx)
	{
		if(pvd->registerObject(materialPtr[idx]))
			metaBing.createInstance(inStream, *materialPtr[idx], PxGetPhysics());
		inStream.pushBackObjectRef(&inObj, "Materials", materialPtr[idx]);
	}
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxShape& inObj, const PxRigidActor& owner, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	if(!inStream.isInstanceValid(&owner))
		return;

	const OwnerActorsMap::Entry* entry = mBindingData->mOwnerActorsMap.find(&inObj);
	if(entry != NULL)
	{
		if(!mBindingData->mOwnerActorsMap[&inObj]->contains(&owner))
			mBindingData->mOwnerActorsMap[&inObj]->insert(&owner);
	}
	else
	{
		OwnerActorsValueType* data = reinterpret_cast<OwnerActorsValueType*>(
		    PX_ALLOC(sizeof(OwnerActorsValueType), "mOwnerActorsMapValue")); //( 1 );
		OwnerActorsValueType* actors = PX_PLACEMENT_NEW(data, OwnerActorsValueType);
		actors->insert(&owner);

		mBindingData->mOwnerActorsMap.insert(&inObj, actors);
	}

	if(inStream.isInstanceValid(&inObj))
	{
		inStream.pushBackObjectRef(&owner, "Shapes", &inObj);
		return;
	}

	inStream.createInstance(&inObj);
	inStream.pushBackObjectRef(&owner, "Shapes", &inObj);
	inStream.setPropertyValue(&inObj, "Actor", reinterpret_cast<const void*>(&owner));
	sendAllProperties(inStream, inObj);
	setGeometry(*this, inStream, inObj, pvd);
	setMaterials(*this, inStream, inObj, pvd, mBindingData);
	if(!inObj.isExclusive())
		inStream.pushBackObjectRef(&ownerPhysics, "SharedShapes", &inObj);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxShape& inObj)
{
	PxShapeGeneratedValues values(&inObj);
	inStream.setPropertyMessage(&inObj, values);
}

void PvdMetaDataBinding::releaseAndRecreateGeometry(PvdDataStream& inStream, const PxShape& inObj, PxPhysics& /*ownerPhysics*/, PsPvd* pvd)
{
	const void* geomInst = (reinterpret_cast<const PxU8*>(&inObj)) + 4;
	inStream.destroyInstance(geomInst);
	const PxGeometry& geom = inObj.getGeometry();
	// Quick fix for HF modify, PxConvexMesh and PxTriangleMesh need recook, they should always be new if modified
	if(geom.getType() == PxGeometryType::eHEIGHTFIELD)
	{
		const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);
		if(inStream.isInstanceValid(hfGeom.heightField))
			sendAllProperties(inStream, *hfGeom.heightField);
	}

	setGeometry(*this, inStream, inObj, pvd);

	// Need update actor cause PVD takes actor-shape as a pair.
	{
		PxRigidActor* actor = inObj.getActor();
		if(actor != NULL)
		{
			if(const PxRigidStatic* rgS = actor->is<PxRigidStatic>())
				sendAllProperties(inStream, *rgS);
			else if(const PxRigidDynamic* rgD = actor->is<PxRigidDynamic>())
				sendAllProperties(inStream, *rgD);
		}
	}
}

void PvdMetaDataBinding::updateMaterials(PvdDataStream& inStream, const PxShape& inObj, PsPvd* pvd)
{
	// Clear the shape's materials array.
	inStream.setPropertyValue(&inObj, "Materials", DataRef<const PxU8>(), getPvdNamespacedNameForType<ObjectRef>());
	setMaterials(*this, inStream, inObj, pvd, mBindingData);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxShape& inObj, const PxRigidActor& owner)
{
	if(inStream.isInstanceValid(&inObj))
	{
		inStream.removeObjectRef(&owner, "Shapes", &inObj);

		bool bDestroy = true;
		const OwnerActorsMap::Entry* entry0 = mBindingData->mOwnerActorsMap.find(&inObj);
		if(entry0 != NULL)
		{
			entry0->second->erase(&owner);
			if(entry0->second->size() > 0)
				bDestroy = false;
			else
			{
				mBindingData->mOwnerActorsMap[&inObj]->~OwnerActorsValueType();
				PX_FREE(mBindingData->mOwnerActorsMap[&inObj]);
				mBindingData->mOwnerActorsMap.erase(&inObj);
			}
		}

		if(bDestroy)
		{			
		    if(!inObj.isExclusive())
	            inStream.removeObjectRef(&PxGetPhysics(), "SharedShapes", &inObj);

			const void* geomInst = (reinterpret_cast<const PxU8*>(&inObj)) + 4;
			inStream.destroyInstance(geomInst);
			inStream.destroyInstance(&inObj);

			const OwnerActorsMap::Entry* entry = mBindingData->mOwnerActorsMap.find(&inObj);
			if(entry != NULL)
			{
				entry->second->~OwnerActorsValueType();
				OwnerActorsValueType* ptr = entry->second;
				PX_FREE(ptr);
				mBindingData->mOwnerActorsMap.erase(&inObj);
			}			
		}
	}
}

template <typename TDataType>
static void addSceneGroupProperty(PvdDataStream& inStream, const char* groupName, const TDataType& inObj, const PxScene& inScene)
{
	inStream.createInstance(&inObj);
	inStream.pushBackObjectRef(&inScene, groupName, &inObj);
	inStream.setPropertyValue(&inObj, "Scene", reinterpret_cast<const void*>(&inScene));
}

template <typename TDataType>
static void removeSceneGroupProperty(PvdDataStream& inStream, const char* groupName, const TDataType& inObj, const PxScene& inScene)
{
	inStream.removeObjectRef(&inScene, groupName, &inObj);
	inStream.destroyInstance(&inObj);
}

static void sendShapes(PvdMetaDataBinding& binding, PvdDataStream& inStream, const PxRigidActor& inObj, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	PxInlineArray<PxShape*, 5> shapeData;
	PxU32 nbShapes = inObj.getNbShapes();
	shapeData.resize(nbShapes);
	inObj.getShapes(shapeData.begin(), nbShapes);
	for(PxU32 idx = 0; idx < nbShapes; ++idx)
		binding.createInstance(inStream, *shapeData[idx], inObj, ownerPhysics, pvd);
}

static void releaseShapes(PvdMetaDataBinding& binding, PvdDataStream& inStream, const PxRigidActor& inObj)
{
	PxInlineArray<PxShape*, 5> shapeData;
	PxU32 nbShapes = inObj.getNbShapes();
	shapeData.resize(nbShapes);
	inObj.getShapes(shapeData.begin(), nbShapes);
	for(PxU32 idx = 0; idx < nbShapes; ++idx)
		binding.destroyInstance(inStream, *shapeData[idx], inObj);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxRigidStatic& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	addSceneGroupProperty(inStream, "RigidStatics", inObj, ownerScene);
	sendAllProperties(inStream, inObj);
	sendShapes(*this, inStream, inObj, ownerPhysics, pvd);
}
void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxRigidStatic& inObj)
{
	PxRigidStaticGeneratedValues values(&inObj);
	inStream.setPropertyMessage(&inObj, values);
}
void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxRigidStatic& inObj, const PxScene& ownerScene)
{
	releaseShapes(*this, inStream, inObj);
	removeSceneGroupProperty(inStream, "RigidStatics", inObj, ownerScene);
}
void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxRigidDynamic& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	addSceneGroupProperty(inStream, "RigidDynamics", inObj, ownerScene);
	sendAllProperties(inStream, inObj);
	sendShapes(*this, inStream, inObj, ownerPhysics, pvd);
}
void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxRigidDynamic& inObj)
{
	PxRigidDynamicGeneratedValues values(&inObj);
	inStream.setPropertyMessage(&inObj, values);
}
void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxRigidDynamic& inObj, const PxScene& ownerScene)
{
	releaseShapes(*this, inStream, inObj);
	removeSceneGroupProperty(inStream, "RigidDynamics", inObj, ownerScene);
}

static void addChild(PvdDataStream& inStream, const void* inParent, const PxArticulationLink& inChild)
{
	inStream.pushBackObjectRef(inParent, "Links", &inChild);
	inStream.setPropertyValue(&inChild, "Parent", inParent);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxArticulationReducedCoordinate& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	addSceneGroupProperty(inStream, "Articulations", inObj, ownerScene);
	sendAllProperties(inStream, inObj);
	PxU32 numLinks = inObj.getNbLinks();
	mBindingData->mArticulationLinks.resize(numLinks);
	inObj.getLinks(mBindingData->mArticulationLinks.begin(), numLinks);
	// From Dilip Sequiera:
	/*
	    No, there can only be one root, and in all the code I wrote (which is not 100% of the HL code for
	   articulations),  the index of a child is always > the index of the parent.
	*/

	// Create all the links
	for(PxU32 idx = 0; idx < numLinks; ++idx)
	{
		if(!inStream.isInstanceValid(mBindingData->mArticulationLinks[idx]))
			createInstance(inStream, *mBindingData->mArticulationLinks[idx], ownerPhysics, pvd);
	}

	// Setup the link graph
	for(PxU32 idx = 0; idx < numLinks; ++idx)
	{
		PxArticulationLink* link = mBindingData->mArticulationLinks[idx];
		if(idx == 0)
			addChild(inStream, &inObj, *link);

		PxU32 numChildren = link->getNbChildren();
		PxArticulationLink** children = mBindingData->allocateTemp<PxArticulationLink*>(numChildren);
		link->getChildren(children, numChildren);
		for(PxU32 i = 0; i < numChildren; ++i)
			addChild(inStream, link, *children[i]);
	}
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxArticulationReducedCoordinate& inObj)
{
	PxArticulationReducedCoordinateGeneratedValues values(&inObj);
	inStream.setPropertyMessage(&inObj, values);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxArticulationReducedCoordinate& inObj, const PxScene& ownerScene)
{
	removeSceneGroupProperty(inStream, "Articulations", inObj, ownerScene);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxArticulationLink& inObj, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	inStream.createInstance(&inObj);
	PxArticulationJointReducedCoordinate* joint(inObj.getInboundJoint());
	if(joint)
	{
		inStream.createInstance(joint);
		inStream.setPropertyValue(&inObj, "InboundJoint", reinterpret_cast<const void*>(joint));
		inStream.setPropertyValue(joint, "Link", reinterpret_cast<const void*>(&inObj));
		sendAllProperties(inStream, *joint);
	}
	sendAllProperties(inStream, inObj);
	sendShapes(*this, inStream, inObj, ownerPhysics, pvd);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxArticulationLink& inObj)
{
	PxArticulationLinkGeneratedValues values(&inObj);
	inStream.setPropertyMessage(&inObj, values);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxArticulationLink& inObj)
{
	PxArticulationJointReducedCoordinate* joint(inObj.getInboundJoint());
	if(joint)
		inStream.destroyInstance(joint);
	releaseShapes(*this, inStream, inObj);
	inStream.destroyInstance(&inObj);
}
// These are created as part of the articulation link's creation process, so outside entities don't need to
// create them.
void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxArticulationJointReducedCoordinate& inObj)
{
	PxArticulationJointReducedCoordinateGeneratedValues values(&inObj);
	inStream.setPropertyMessage(&inObj, values);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxSoftBody& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
	PX_UNUSED(ownerPhysics);
	PX_UNUSED(pvd);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxSoftBody& inObj)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxSoftBody& inObj, const PxScene& ownerScene)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxFEMCloth& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
	PX_UNUSED(ownerPhysics);
	PX_UNUSED(pvd);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxFEMCloth& inObj)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxFEMCloth& inObj, const PxScene& ownerScene)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxPBDParticleSystem& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
	PX_UNUSED(ownerPhysics);
	PX_UNUSED(pvd);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxPBDParticleSystem& inObj)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxPBDParticleSystem& inObj, const PxScene& ownerScene)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
}


void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxFLIPParticleSystem& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
	PX_UNUSED(ownerPhysics);
	PX_UNUSED(pvd);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxFLIPParticleSystem& inObj)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxFLIPParticleSystem& inObj, const PxScene& ownerScene)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxMPMParticleSystem& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
	PX_UNUSED(ownerPhysics);
	PX_UNUSED(pvd);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxMPMParticleSystem& inObj)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxMPMParticleSystem& inObj, const PxScene& ownerScene)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
}

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxHairSystem& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
	PX_UNUSED(ownerPhysics);
	PX_UNUSED(pvd);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxHairSystem& inObj)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxHairSystem& inObj, const PxScene& ownerScene)
{
	PX_UNUSED(inStream);
	PX_UNUSED(inObj);
	PX_UNUSED(ownerScene);
}

void PvdMetaDataBinding::originShift(PvdDataStream& inStream, const PxScene* inScene, PxVec3 shift)
{
	inStream.originShift(inScene, shift);
}

template <typename TBlockType, typename TActorType, typename TOperator>
static void updateActor(PvdDataStream& inStream, TActorType** actorGroup, PxU32 numActors, TOperator sleepingOp, PvdMetaDataBindingData& bindingData)
{
	TBlockType theBlock;
	if(numActors == 0)
		return;
	for(PxU32 idx = 0; idx < numActors; ++idx)
	{
		TActorType* theActor(actorGroup[idx]);
		bool sleeping = sleepingOp(theActor, theBlock);
		bool wasSleeping = bindingData.mSleepingActors.contains(theActor);

		if(sleeping == false || sleeping != wasSleeping)
		{
			theBlock.GlobalPose = theActor->getGlobalPose();
			theBlock.AngularVelocity = theActor->getAngularVelocity();
			theBlock.LinearVelocity = theActor->getLinearVelocity();
			inStream.sendPropertyMessageFromGroup(theActor, theBlock);
			if(sleeping != wasSleeping)
			{
				if(sleeping)
					bindingData.mSleepingActors.insert(theActor);
				else
					bindingData.mSleepingActors.erase(theActor);
			}
		}
	}
}

struct RigidDynamicUpdateOp
{
	bool operator()(PxRigidDynamic* actor, PxRigidDynamicUpdateBlock& block)
	{
		bool sleeping = actor->isSleeping();
		block.IsSleeping = sleeping;
		return sleeping;
	}
};

struct ArticulationLinkUpdateOp
{
	bool sleeping;
	ArticulationLinkUpdateOp(bool s) : sleeping(s)
	{
	}
	bool operator()(PxArticulationLink*, PxArticulationLinkUpdateBlock&)
	{
		return sleeping;
	}
};

void PvdMetaDataBinding::updateDynamicActorsAndArticulations(PvdDataStream& inStream, const PxScene* inScene, PvdVisualizer* linkJointViz)
{
	PX_COMPILE_TIME_ASSERT(sizeof(PxRigidDynamicUpdateBlock) == 14 * 4);
	{
		PxU32 actorCount = inScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
		if(actorCount)
		{
			inStream.beginPropertyMessageGroup<PxRigidDynamicUpdateBlock>();
			mBindingData->mActors.resize(actorCount);
			PxActor** theActors = mBindingData->mActors.begin();
			inScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, theActors, actorCount);
			updateActor<PxRigidDynamicUpdateBlock>(inStream, reinterpret_cast<PxRigidDynamic**>(theActors), actorCount, RigidDynamicUpdateOp(), *mBindingData);
			inStream.endPropertyMessageGroup();
		}
	}
	{
		PxU32 articulationCount = inScene->getNbArticulations();
		if(articulationCount)
		{
			mBindingData->mArticulations.resize(articulationCount);
			PxArticulationReducedCoordinate** firstArticulation = mBindingData->mArticulations.begin();
			PxArticulationReducedCoordinate** lastArticulation = firstArticulation + articulationCount;
			inScene->getArticulations(firstArticulation, articulationCount);
			inStream.beginPropertyMessageGroup<PxArticulationLinkUpdateBlock>();
			for(; firstArticulation < lastArticulation; ++firstArticulation)
			{
				PxU32 linkCount = (*firstArticulation)->getNbLinks();
				bool sleeping = (*firstArticulation)->isSleeping();
				if(linkCount)
				{
					mBindingData->mArticulationLinks.resize(linkCount);
					PxArticulationLink** theLink = mBindingData->mArticulationLinks.begin();
					(*firstArticulation)->getLinks(theLink, linkCount);
					updateActor<PxArticulationLinkUpdateBlock>(inStream, theLink, linkCount, ArticulationLinkUpdateOp(sleeping), *mBindingData);
					if(linkJointViz)
					{
						for(PxU32 idx = 0; idx < linkCount; ++idx)
							linkJointViz->visualize(*theLink[idx]);
					}
				}
			}
			inStream.endPropertyMessageGroup();
			firstArticulation = mBindingData->mArticulations.begin();
			for (; firstArticulation < lastArticulation; ++firstArticulation)
			{
				inStream.setPropertyValue(*firstArticulation, "IsSleeping", (*firstArticulation)->isSleeping());
				inStream.setPropertyValue(*firstArticulation, "RootGlobalPose", (*firstArticulation)->getRootGlobalPose());
				inStream.setPropertyValue(*firstArticulation, "RootLinearVelocity", (*firstArticulation)->getRootLinearVelocity());
				inStream.setPropertyValue(*firstArticulation, "RootAngularVelocity", (*firstArticulation)->getRootAngularVelocity());
			}

			inStream.beginPropertyMessageGroup<PxArticulationJointUpdateBlock>();
			firstArticulation = mBindingData->mArticulations.begin();

			for (; firstArticulation < lastArticulation; ++firstArticulation)
			{
				PxU32 linkCount = (*firstArticulation)->getNbLinks();
				bool sleeping = (*firstArticulation)->isSleeping();
				if (!sleeping)
				{
					for (PxU32 idx = 1; idx < linkCount; ++idx)
					{
						mBindingData->mArticulationLinks.resize(linkCount);
						PxArticulationLink** theLink = mBindingData->mArticulationLinks.begin();
						(*firstArticulation)->getLinks(theLink, linkCount);

						PxArticulationJointUpdateBlock jointBlock;
						PxArticulationJointReducedCoordinate* joint = theLink[idx]->getInboundJoint();
						jointBlock.JointPosition_eX = joint->getJointPosition(PxArticulationAxis::eX);
						jointBlock.JointPosition_eY = joint->getJointPosition(PxArticulationAxis::eY);
						jointBlock.JointPosition_eZ = joint->getJointPosition(PxArticulationAxis::eZ);
						jointBlock.JointPosition_eTwist = joint->getJointPosition(PxArticulationAxis::eTWIST);
						jointBlock.JointPosition_eSwing1 = joint->getJointPosition(PxArticulationAxis::eSWING1);
						jointBlock.JointPosition_eSwing2 = joint->getJointPosition(PxArticulationAxis::eSWING2);

						jointBlock.JointVelocity_eX = joint->getJointVelocity(PxArticulationAxis::eX);
						jointBlock.JointVelocity_eY = joint->getJointVelocity(PxArticulationAxis::eY);
						jointBlock.JointVelocity_eZ = joint->getJointVelocity(PxArticulationAxis::eZ);
						jointBlock.JointVelocity_eTwist = joint->getJointVelocity(PxArticulationAxis::eTWIST);
						jointBlock.JointVelocity_eSwing1 = joint->getJointVelocity(PxArticulationAxis::eSWING1);
						jointBlock.JointVelocity_eSwing2 = joint->getJointVelocity(PxArticulationAxis::eSWING2);

						inStream.sendPropertyMessageFromGroup(joint, jointBlock);
					}
				}				
			}
			inStream.endPropertyMessageGroup();
		}
	}
}

template <typename TObjType>
struct CollectionOperator
{
	PxArray<PxU8>& mTempArray;
	const TObjType& mObject;
	PvdDataStream& mStream;

	CollectionOperator(PxArray<PxU8>& ary, const TObjType& obj, PvdDataStream& stream)
	: mTempArray(ary), mObject(obj), mStream(stream)
	{
	}
	void pushName(const char*)
	{
	}
	void popName()
	{
	}
	template <typename TAccessor>
	void simpleProperty(PxU32 /*key*/, const TAccessor&)
	{
	}
	template <typename TAccessor>
	void flagsProperty(PxU32 /*key*/, const TAccessor&, const PxU32ToName*)
	{
	}

	template <typename TColType, typename TDataType, typename TCollectionProp>
	void handleCollection(const TCollectionProp& prop, NamespacedName dtype, PxU32 countMultiplier = 1)
	{
		PxU32 count = prop.size(&mObject);
		mTempArray.resize(count * sizeof(TDataType));
		TColType* start = reinterpret_cast<TColType*>(mTempArray.begin());
		prop.get(&mObject, start, count * countMultiplier);
		mStream.setPropertyValue(&mObject, prop.mName, DataRef<const PxU8>(mTempArray.begin(), mTempArray.size()), dtype);
	}
	template <PxU32 TKey, typename TObject, typename TColType>
	void handleCollection(const PxReadOnlyCollectionPropertyInfo<TKey, TObject, TColType>& prop)
	{
		handleCollection<TColType, TColType>(prop, getPvdNamespacedNameForType<TColType>());
	}
	// Enumerations or bitflags.
	template <PxU32 TKey, typename TObject, typename TColType>
	void handleCollection(const PxReadOnlyCollectionPropertyInfo<TKey, TObject, TColType>& prop, const PxU32ToName*)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(TColType) == sizeof(PxU32));
		handleCollection<TColType, PxU32>(prop, getPvdNamespacedNameForType<PxU32>());
	}

  private:
	CollectionOperator& operator=(const CollectionOperator&);
};

// per frame update

#define ENABLE_AGGREGATE_PVD_SUPPORT 1
#ifdef ENABLE_AGGREGATE_PVD_SUPPORT

void PvdMetaDataBinding::createInstance(PvdDataStream& inStream, const PxAggregate& inObj, const PxScene& ownerScene)
{
	addSceneGroupProperty(inStream, "Aggregates", inObj, ownerScene);
	sendAllProperties(inStream, inObj);
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream& inStream, const PxAggregate& inObj)
{
	PxAggregateGeneratedValues values(&inObj);
	inStream.setPropertyMessage(&inObj, values);
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream& inStream, const PxAggregate& inObj, const PxScene& ownerScene)
{
	removeSceneGroupProperty(inStream, "Aggregates", inObj, ownerScene);
}

class ChangeOjectRefCmd : public PvdDataStream::PvdCommand
{
	ChangeOjectRefCmd& operator=(const ChangeOjectRefCmd&)
	{
		PX_ASSERT(0);
		return *this;
	} // PX_NOCOPY doesn't work for local classes
  public:
	const void*	mInstance;
	String		mPropName;
	const void*	mPropObj;
	const bool	mPushBack;

	ChangeOjectRefCmd(const void* inInst, String inName, const void* inObj, bool pushBack)
	: mInstance(inInst), mPropName(inName), mPropObj(inObj), mPushBack(pushBack)
	{
	}

	// Assigned is needed for copying
	ChangeOjectRefCmd(const ChangeOjectRefCmd& other)
	: PvdDataStream::PvdCommand(other), mInstance(other.mInstance), mPropName(other.mPropName), mPropObj(other.mPropObj), mPushBack(other.mPushBack)
	{
	}

	virtual bool canRun(PvdInstanceDataStream& inStream)
	{
		PX_ASSERT(inStream.isInstanceValid(mInstance));
		return inStream.isInstanceValid(mPropObj);
	}
	virtual void run(PvdInstanceDataStream& inStream)
	{
		if(!inStream.isInstanceValid(mInstance))
			return;

		if(mPushBack)
		{
			if(inStream.isInstanceValid(mPropObj))
				inStream.pushBackObjectRef(mInstance, mPropName, mPropObj);
		}
		else
		{
			// the called function will assert if propobj is already removed
			inStream.removeObjectRef(mInstance, mPropName, mPropObj);
		}
	}
};

static void changeAggregateSubActors(PvdDataStream& inStream, const PxAggregate& inObj, const PxActor& inActor, bool pushBack)
{
	const PxArticulationLink* link = inActor.is<PxArticulationLink>();
	String propName = NULL;
	const void* object = NULL;
	if(link == NULL)
	{
		propName = "Actors";
		object = &inActor;
	}
	else if(link->getInboundJoint() == NULL)
	{
		propName = "Articulations";
		object = &link->getArticulation();
	}
	else
		return;

	ChangeOjectRefCmd* cmd = PX_PLACEMENT_NEW(inStream.allocateMemForCmd(sizeof(ChangeOjectRefCmd)), ChangeOjectRefCmd)(&inObj, propName, object, pushBack);

	if(cmd->canRun(inStream))
		cmd->run(inStream);
	else
		inStream.pushPvdCommand(*cmd);
}
void PvdMetaDataBinding::detachAggregateActor(PvdDataStream& inStream, const PxAggregate& inObj, const PxActor& inActor)
{
	changeAggregateSubActors(inStream, inObj, inActor, false);
}

void PvdMetaDataBinding::attachAggregateActor(PvdDataStream& inStream, const PxAggregate& inObj, const PxActor& inActor)
{
	changeAggregateSubActors(inStream, inObj, inActor, true);
}
#else
void PvdMetaDataBinding::createInstance(PvdDataStream&, const PxAggregate&, const PxScene&, ObjectRegistrar&)
{
}

void PvdMetaDataBinding::sendAllProperties(PvdDataStream&, const PxAggregate&)
{
}

void PvdMetaDataBinding::destroyInstance(PvdDataStream&, const PxAggregate&, const PxScene&)
{
}

void PvdMetaDataBinding::detachAggregateActor(PvdDataStream&, const PxAggregate&, const PxActor&)
{
}

void PvdMetaDataBinding::attachAggregateActor(PvdDataStream&, const PxAggregate&, const PxActor&)
{
}
#endif

template <typename TDataType>
static void sendSceneArray(PvdDataStream& inStream, const PxScene& inScene, const PxArray<TDataType>& inArray, const char* propName)
{
	if(0 == inArray.size())
		inStream.setPropertyValue(&inScene, propName, DataRef<const PxU8>(), getPvdNamespacedNameForType<TDataType>());
	else
	{
		ScopedPropertyValueSender<TDataType, 32> sender(inStream, &inScene, propName);
		for(PxU32 i = 0; i < inArray.size(); ++i)
			sender.append(inArray[i]);
	}
}

static void sendSceneArray(PvdDataStream& inStream, const PxScene& inScene, const PxArray<PvdSqHit>& inArray, const char* propName)
{
	if(0 == inArray.size())
		inStream.setPropertyValue(&inScene, propName, DataRef<const PxU8>(), getPvdNamespacedNameForType<PvdSqHit>());
	else
	{
		ScopedPropertyValueSender<PvdSqHit, 32> sender(inStream, &inScene, propName);
		for(PxU32 i = 0; i < inArray.size(); ++i)
		{
			if(!inStream.isInstanceValid(inArray[i].mShape) || !inStream.isInstanceValid(inArray[i].mActor))
			{
				PvdSqHit hit = inArray[i];
				hit.mShape = NULL;
				hit.mActor = NULL;
				sender.append(hit);
			}
			else
				sender.append(inArray[i]);
		}
	}
}

void PvdMetaDataBinding::sendSceneQueries(PvdDataStream& inStream, const PxScene& inScene, PsPvd* pvd)
{
	if(!inStream.isConnected())
		return;

	const physx::NpScene& scene = static_cast<const NpScene&>(inScene);
	
	{
		PvdSceneQueryCollector& collector = scene.getNpSQ().getSingleSqCollector();
		PxMutex::ScopedLock lock(collector.getLock());

		String propName = collector.getArrayName(collector.mPvdSqHits);
		sendSceneArray(inStream, inScene, collector.mPvdSqHits, propName);

		propName = collector.getArrayName(collector.mPoses);
		sendSceneArray(inStream, inScene, collector.mPoses, propName);

		propName = collector.getArrayName(collector.mFilterData);
		sendSceneArray(inStream, inScene, collector.mFilterData, propName);

		const NamedArray<PxGeometryHolder>& geometriesToDestroy = collector.getPrevFrameGeometries();
		propName = collector.getArrayName(geometriesToDestroy);
		for(PxU32 k = 0; k < geometriesToDestroy.size(); ++k)
		{
			const PxGeometryHolder& inObj = geometriesToDestroy[k];
			inStream.removeObjectRef(&inScene, propName, &inObj);
			inStream.destroyInstance(&inObj);
		}
		const PxArray<PxGeometryHolder>& geometriesToCreate = collector.getCurrentFrameGeometries();
		for(PxU32 k = 0; k < geometriesToCreate.size(); ++k)
		{
			const PxGeometry& geometry = geometriesToCreate[k].any();
			switch(geometry.getType())
			{
#define SEND_PVD_GEOM_TYPE(enumType, TGeomType, TValueType)                         \
	case enumType:                                                                  \
	{                                                                               \
		const TGeomType& inObj = static_cast<const TGeomType&>(geometry);           \
		inStream.createInstance(getPvdNamespacedNameForType<TGeomType>(), &inObj);  \
		registrarPhysicsObject<TGeomType>(inStream, inObj, pvd);					\
		TValueType values(&inObj);                                                  \
		inStream.setPropertyMessage(&inObj, values);                                \
		inStream.pushBackObjectRef(&inScene, propName, &inObj);                     \
	}                                                                               \
	break;
				SEND_PVD_GEOM_TYPE(PxGeometryType::eBOX, PxBoxGeometry, PxBoxGeometryGeneratedValues)
				SEND_PVD_GEOM_TYPE(PxGeometryType::eSPHERE, PxSphereGeometry, PxSphereGeometryGeneratedValues)
				SEND_PVD_GEOM_TYPE(PxGeometryType::eCAPSULE, PxCapsuleGeometry, PxCapsuleGeometryGeneratedValues)
				SEND_PVD_GEOM_TYPE(PxGeometryType::eCONVEXMESH, PxConvexMeshGeometry, PxConvexMeshGeometryGeneratedValues)
#undef SEND_PVD_GEOM_TYPE
			case PxGeometryType::ePLANE:
			case PxGeometryType::eTRIANGLEMESH:
			case PxGeometryType::eHEIGHTFIELD:
			case PxGeometryType::eTETRAHEDRONMESH:
			case PxGeometryType::ePARTICLESYSTEM:
			case PxGeometryType::eHAIRSYSTEM:
			case PxGeometryType::eCUSTOM:
			case PxGeometryType::eGEOMETRY_COUNT:
			case PxGeometryType::eINVALID:
				PX_ALWAYS_ASSERT_MESSAGE("unsupported scene query geometry type");
				break;
			}
		}
		collector.prepareNextFrameGeometries();

		propName = collector.getArrayName(collector.mAccumulatedRaycastQueries);
		sendSceneArray(inStream, inScene, collector.mAccumulatedRaycastQueries, propName);

		propName = collector.getArrayName(collector.mAccumulatedOverlapQueries);
		sendSceneArray(inStream, inScene, collector.mAccumulatedOverlapQueries, propName);

		propName = collector.getArrayName(collector.mAccumulatedSweepQueries);
		sendSceneArray(inStream, inScene, collector.mAccumulatedSweepQueries, propName);
	}
}
}
}

#endif

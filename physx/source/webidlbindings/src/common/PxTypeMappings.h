#ifndef PX_TYPE_MAPPINGS_H
#define PX_TYPE_MAPPINGS_H

#include "PxPhysicsAPI.h"
#include <cstring> // for memcpy

// typedefs for vehicle lookup tables
typedef physx::vehicle2::PxVehicleFixedSizeLookupTable<physx::PxReal,3> PxVehicleFixedSizeLookupTableFloat_3;
typedef physx::vehicle2::PxVehicleFixedSizeLookupTable<physx::PxVec3,3> PxVehicleFixedSizeLookupTableVec3_3;
typedef physx::vehicle2::PxVehicleFixedSizeLookupTable<physx::PxReal,8> PxVehicleTorqueCurveLookupTable;

// typedefs for pointer types
typedef const physx::PxU8* PxU8ConstPtr;
typedef const physx::PxU16* PxU16ConstPtr;
typedef const physx::PxU32* PxU32ConstPtr;
typedef const physx::PxI32* PxI32ConstPtr;
typedef const physx::PxReal* PxRealConstPtr;
typedef const physx::PxMaterial* PxMaterialConstPtr;
typedef physx::PxU8* PxU8Ptr;
typedef physx::PxU16* PxU16Ptr;
typedef physx::PxU32* PxU32Ptr;
typedef physx::PxI32* PxI32Ptr;
typedef physx::PxReal* PxRealPtr;
typedef physx::PxMaterial* PxMaterialPtr;
typedef physx::PxActor* PxActorPtr;
typedef physx::PxShape* PxShapePtr;

typedef physx::PxOverlapBufferN<10> PxOverlapBuffer10;
typedef physx::PxRaycastBufferN<10> PxRaycastBuffer10;
typedef physx::PxSweepBufferN<10> PxSweepBuffer10;

typedef physx::PxTypedBoundedData<physx::PxU16> PxTypedBoundedData_PxU16;
typedef physx::PxTypedBoundedData<const physx::PxU16> PxTypedBoundedData_PxU16Const;

/**
 * PxArrayExt extends PxArray to get a slightly more Java(-script) friendly interface with get() and set() methods.
 * Also adds at(), push_back() and data() methods, so it can be used as drop-in replacement for std::vector.
 */
template <class T>
class PxArrayExt : public physx::PxArray<T> {
public:
    PX_INLINE PxArrayExt() : physx::PxArray<T>() { }
    PX_INLINE PxArrayExt(uint32_t size, const T& a = T()) : physx::PxArray<T>(size, a) { }

    PX_INLINE T& get(uint32_t index) { return this->operator[](index); }
    PX_INLINE void set(uint32_t index, const T& value) { get(index) = value; }

    // compatibility functions, so that PxArrayList can be used as replacement for std::vector
    PX_INLINE T& at(uint32_t index) { return this->operator[](index); }
    PX_INLINE void push_back(const T& value) { this->pushBack(value); }
    PX_INLINE T* data() { return this->begin(); }
    PX_INLINE void setFromBuffer(const void* buffer, uint32_t size) {
        this->reset();
        this->resize(size);
        std::memcpy(this->begin(), buffer, size * sizeof(T));
    }
};

class PxArray_PxVec3 : public PxArrayExt<physx::PxVec3> {
public:
    PxArray_PxVec3() : PxArrayExt<physx::PxVec3>() { }
    PxArray_PxVec3(uint32_t size) : PxArrayExt<physx::PxVec3>(size, physx::PxVec3(physx::PxZERO::PxZero)) { }
};

class PxArray_PxVec4 : public PxArrayExt<physx::PxVec4> {
public:
    PxArray_PxVec4() : PxArrayExt<physx::PxVec4>() { }
    PxArray_PxVec4(uint32_t size) : PxArrayExt<physx::PxVec4>(size, physx::PxVec4(physx::PxZERO::PxZero)) { }
};

typedef PxArrayExt<PxMaterialConstPtr> PxArray_PxMaterialConst;
typedef PxArrayExt<PxActorPtr> PxArray_PxActorPtr;
typedef PxArrayExt<PxShapePtr> PxArray_PxShapePtr;
typedef PxArrayExt<physx::PxContactPairPoint> PxArray_PxContactPairPoint;
typedef PxArrayExt<physx::PxHeightFieldSample> PxArray_PxHeightFieldSample;
typedef PxArrayExt<physx::PxRaycastHit> PxArray_PxRaycastHit;
typedef PxArrayExt<physx::PxSweepHit> PxArray_PxSweepHit;

typedef PxArrayExt<physx::PxReal> PxArray_PxReal;
typedef PxArrayExt<physx::PxU8> PxArray_PxU8;
typedef PxArrayExt<physx::PxU16> PxArray_PxU16;
typedef PxArrayExt<physx::PxU32> PxArray_PxU32;
typedef PxArray_PxVec3 PxArray_PxVec3;
typedef PxArray_PxVec4 PxArray_PxVec4;

// deprecated std::vector style types
typedef PxArrayExt<PxMaterialConstPtr> Vector_PxMaterialConst;
typedef PxArrayExt<PxActorPtr> Vector_PxActorPtr;
typedef PxArrayExt<physx::PxContactPairPoint> Vector_PxContactPairPoint;
typedef PxArrayExt<physx::PxHeightFieldSample> Vector_PxHeightFieldSample;
typedef PxArrayExt<physx::PxRaycastHit> Vector_PxRaycastHit;
typedef PxArrayExt<physx::PxSweepHit> Vector_PxSweepHit;

typedef PxArrayExt<physx::PxReal> Vector_PxReal;
typedef PxArrayExt<physx::PxU8> Vector_PxU8;
typedef PxArrayExt<physx::PxU16> Vector_PxU16;
typedef PxArrayExt<physx::PxU32> Vector_PxU32;
typedef PxArray_PxVec3 Vector_PxVec3;
typedef PxArray_PxVec4 Vector_PxVec4;

// enums within namespaces are not supported by webidl binder, use typedefs to work around that
typedef physx::PxActorFlag::Enum PxActorFlagEnum;
typedef physx::PxPvdInstrumentationFlag::Enum PxPvdInstrumentationFlagEnum;
typedef physx::PxActorType::Enum PxActorTypeEnum;
typedef physx::PxActorTypeFlag::Enum PxActorTypeFlagEnum;
typedef physx::PxArticulationAxis::Enum PxArticulationAxisEnum;
typedef physx::PxArticulationCacheFlag::Enum PxArticulationCacheFlagEnum;
typedef physx::PxArticulationDriveType::Enum PxArticulationDriveTypeEnum;
typedef physx::PxArticulationFlag::Enum PxArticulationFlagEnum;
typedef physx::PxArticulationJointType::Enum PxArticulationJointTypeEnum;
typedef physx::PxArticulationKinematicFlag::Enum PxArticulationKinematicFlagEnum;
typedef physx::PxArticulationMotion::Enum PxArticulationMotionEnum;
typedef physx::PxBaseFlag::Enum PxBaseFlagEnum;
typedef physx::PxBroadPhaseType::Enum PxBroadPhaseTypeEnum;
typedef physx::PxBVHBuildStrategy::Enum PxBVHBuildStrategyEnum;
typedef physx::PxCapsuleClimbingMode::Enum PxCapsuleClimbingModeEnum;
typedef physx::PxCombineMode::Enum PxCombineModeEnum;
typedef physx::PxConstraintFlag::Enum PxConstraintFlagEnum;
typedef physx::PxContactPairFlag::Enum PxContactPairFlagEnum;
typedef physx::PxContactPairHeaderFlag::Enum PxContactPairHeaderFlagEnum;
typedef physx::PxControllerBehaviorFlag::Enum PxControllerBehaviorFlagEnum;
typedef physx::PxControllerCollisionFlag::Enum PxControllerCollisionFlagEnum;
typedef physx::PxControllerNonWalkableMode::Enum PxControllerNonWalkableModeEnum;
typedef physx::PxControllerShapeType::Enum PxControllerShapeTypeEnum;
typedef physx::PxConvexFlag::Enum PxConvexFlagEnum;
typedef physx::PxConvexMeshCookingType::Enum PxConvexMeshCookingTypeEnum;
typedef physx::PxConvexMeshGeometryFlag::Enum PxConvexMeshGeometryFlagEnum;
typedef physx::PxD6Axis::Enum PxD6AxisEnum;
typedef physx::PxD6Drive::Enum PxD6DriveEnum;
typedef physx::PxD6Motion::Enum PxD6MotionEnum;
typedef physx::PxD6JointDriveFlag::Enum PxD6JointDriveFlagEnum;
typedef physx::PxDebugColor::Enum PxDebugColorEnum;
typedef physx::PxDistanceJointFlag::Enum PxDistanceJointFlagEnum;
typedef physx::PxDynamicTreeSecondaryPruner::Enum PxDynamicTreeSecondaryPrunerEnum;
typedef physx::PxErrorCode::Enum PxErrorCodeEnum;
typedef physx::PxFilterFlag::Enum PxFilterFlagEnum;
typedef physx::PxFilterObjectFlag::Enum PxFilterObjectFlagEnum;
typedef physx::PxForceMode::Enum PxForceModeEnum;
typedef physx::PxFrictionType::Enum PxFrictionTypeEnum;
typedef physx::PxGeometryType::Enum PxGeometryTypeEnum;
typedef physx::PxHeightFieldFlag::Enum PxHeightFieldFlagEnum;
typedef physx::PxHeightFieldFormat::Enum PxHeightFieldFormatEnum;
typedef physx::PxHitFlag::Enum PxHitFlagEnum;
typedef physx::PxIDENTITY PxIDENTITYEnum;
typedef physx::PxJointActorIndex::Enum PxJointActorIndexEnum;
typedef physx::PxMaterialFlag::Enum PxMaterialFlagEnum;
typedef physx::PxMeshCookingHint::Enum PxMeshCookingHintEnum;
typedef physx::PxMeshFlag::Enum PxMeshFlagEnum;
typedef physx::PxMeshGeometryFlag::Enum PxMeshGeometryFlagEnum;
typedef physx::PxMeshMidPhase::Enum PxMeshMidPhaseEnum;
typedef physx::PxMeshPreprocessingFlag::Enum PxMeshPreprocessingFlagEnum;
typedef physx::PxPairFilteringMode::Enum PxPairFilteringModeEnum;
typedef physx::PxPairFlag::Enum PxPairFlagEnum;
typedef physx::PxPrismaticJointFlag::Enum PxPrismaticJointFlagEnum;
typedef physx::PxPruningStructureType::Enum PxPruningStructureTypeEnum;
typedef physx::PxQueryFlag::Enum PxQueryFlagEnum;
typedef physx::PxQueryHitType::Enum PxQueryHitType;
typedef physx::PxRevoluteJointFlag::Enum PxRevoluteJointFlagEnum;
typedef physx::PxRigidBodyFlag::Enum PxRigidBodyFlagEnum;
typedef physx::PxRigidDynamicLockFlag::Enum PxRigidDynamicLockFlagEnum;
typedef physx::PxSceneFlag::Enum PxSceneFlagEnum;
typedef physx::PxSceneQueryUpdateMode::Enum PxSceneQueryUpdateModeEnum;
typedef physx::PxShapeFlag::Enum PxShapeFlagEnum;
typedef physx::PxSphericalJointFlag::Enum PxSphericalJointFlagEnum;
typedef physx::PxSolverType::Enum PxSolverTypeEnum;
typedef physx::PxTetrahedronMeshAnalysisResult::Enum PxTetrahedronMeshAnalysisResultEnum;
typedef physx::PxTetrahedronMeshDesc::PxMeshFormat PxTetrahedronMeshFormatEnum;
typedef physx::PxTetrahedronMeshFlag::Enum PxTetrahedronMeshFlagEnum;
typedef physx::PxTriangleMeshAnalysisResult::Enum PxTriangleMeshAnalysisResultEnum;
typedef physx::PxTriangleMeshFlag::Enum PxTriangleMeshFlagEnum;
typedef physx::PxTriggerPairFlag::Enum PxTriggerPairFlagEnum;
typedef physx::PxVisualizationParameter::Enum PxVisualizationParameterEnum;
typedef physx::vehicle2::PxVehicleAxes::Enum PxVehicleAxesEnum;
typedef physx::vehicle2::PxVehicleClutchAccuracyMode::Enum PxVehicleClutchAccuracyModeEnum;
typedef physx::vehicle2::PxVehicleCommandNonLinearResponseParams::Enum PxVehicleCommandNonLinearResponseParamsEnum;
typedef physx::vehicle2::PxVehicleCommandValueResponseTable::Enum PxVehicleCommandValueResponseTableEnum;
typedef physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState::Enum PxVehicleDirectDriveTransmissionCommandStateEnum;
typedef physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::Enum PxVehicleEngineDriveTransmissionCommandStateEnum;
typedef physx::vehicle2::PxVehicleGearboxParams::Enum PxVehicleGearboxParamsEnum;
typedef physx::vehicle2::PxVehicleLimits::Enum PxVehicleLimitsEnum;
typedef physx::vehicle2::PxVehiclePhysXActorUpdateMode::Enum PxVehiclePhysXActorUpdateModeEnum;
typedef physx::vehicle2::PxVehiclePhysXConstraintLimits::Enum PxVehiclePhysXConstraintLimitsEnum;
typedef physx::vehicle2::PxVehiclePhysXRoadGeometryQueryType::Enum PxVehiclePhysXRoadGeometryQueryTypeEnum;
typedef physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier PxVehiclePhysXSuspensionLimitConstraintParamsDirectionSpecifierEnum;
typedef physx::vehicle2::PxVehicleSimulationContextType::Enum PxVehicleSimulationContextTypeEnum;
typedef physx::vehicle2::PxVehicleSuspensionJounceCalculationType::Enum PxVehicleSuspensionJounceCalculationTypeEnum;
typedef physx::vehicle2::PxVehicleTireDirectionModes::Enum PxVehicleTireDirectionModesEnum;

#endif

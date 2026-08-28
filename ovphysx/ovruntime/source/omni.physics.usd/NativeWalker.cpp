// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Native USD prim walker.
 *
 * Walks the supplied `PrimIteratorBase` range, classifies each prim by
 * applied APIs + prim type, and emits parse-library descriptors via the
 * existing parse-lib parsers.
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-2 AC-3
 */

#include "NativeWalker.h"

#include "UsdSource.h"

#include <omni/physics/parse/ArticulationGraph.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <private/omni/physics/CollisionShapeTransform.h>
#include <private/omni/physics/JointFrameTransform.h>

#include <pxr/base/gf/quaternion.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/primTypeInfo.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cone.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/plane.h>
#include <pxr/usd/usdGeom/pointBased.h>
#include <pxr/usd/usdGeom/pointInstancer.h>
#include <pxr/usd/usdGeom/points.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/tetMesh.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdGeom/xformCache.h>

#include <physxSchema/physxCharacterControllerAPI.h>
#include <physxSchema/physxVehicleTireFrictionTable.h>
#include <physxSchema/physxParticleSystem.h>
#include <physxSchema/physxVehicleWheelAPI.h>
#include <physxSchema/physxVehicleTireAPI.h>
#include <physxSchema/physxVehicleSuspensionAPI.h>
#include <physxSchema/physxVehicleEngineAPI.h>
#include <physxSchema/physxVehicleGearsAPI.h>
#include <physxSchema/physxVehicleClutchAPI.h>
#include <physxSchema/physxVehicleDriveBasicAPI.h>
#include <physxSchema/physxVehicleDriveStandardAPI.h>
#include <physxSchema/physxVehicleMultiWheelDifferentialAPI.h>
#include <physxSchema/physxVehicleTankDifferentialAPI.h>
#include <physxSchema/physxVehicleAutoGearBoxAPI.h>
#include <physxSchema/physxVehicleBrakesAPI.h>
#include <physxSchema/physxVehicleSteeringAPI.h>
#include <physxSchema/physxVehicleAckermannSteeringAPI.h>
#include <physxSchema/physxVehicleNonlinearCommandResponseAPI.h>
#include <physxSchema/physxVehicleWheelAttachmentAPI.h>
#include <physxSchema/physxVehicleSuspensionComplianceAPI.h>
#include <physxSchema/physxVehicleAPI.h>
#include <physxSchema/tokens.h>
#include <pxr/usd/usdPhysics/metrics.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/collisionGroup.h>
#include <pxr/usd/usdPhysics/distanceJoint.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/limitAPI.h>
#include <pxr/usd/usdPhysics/materialAPI.h>
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
#include <pxr/usd/usdPhysics/tokens.h>

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include <carb/logging/Log.h>

#include <omni/physics/usd/PrimIterator.h>
#include <private/omni/physx/CustomGeometryHash.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace omni::physics::usd
{
using namespace omni::physics::parse;

// Custom-token registry queries — defined in `CustomTokens.cpp`.
// Forward-declared at namespace scope (not in the anonymous block
// below) so they resolve to the external linkage definitions in the
// sibling translation unit.
bool isCustomShapeToken(const PXR_NS::TfToken& token);
bool isCustomJointToken(const PXR_NS::TfToken& token);
bool isCustomPhysicsInstancerToken(const PXR_NS::TfToken& token);

namespace
{

// Bitmask of applied API schemas relevant to physics classification.
// The walker reads applied APIs off each prim, folds them into this
// bitmask, and downstream dispatch keys off the bits.
struct ApiFlag
{
    enum Enum : uint64_t
    {
        eArticulationRootAPI            = 1ull << 0,
        eCollisionAPI                   = 1ull << 1,
        eRigidBodyAPI                   = 1ull << 2,
        eDeformableBodyAPI              = 1ull << 3,
        eMaterialAPI                    = 1ull << 4,
        eDeformableMaterialAPI          = 1ull << 5,
        eSurfaceDeformableMaterialAPI   = 1ull << 6,
        eCurvesDeformableMaterialAPI    = 1ull << 7,
        eCharacterControllerAPI         = 1ull << 8,
        eVehicleContextAPI              = 1ull << 9,
        eVehicleWheelAPI                = 1ull << 10,
        eVehicleTireAPI                 = 1ull << 11,
        eVehicleSuspensionAPI           = 1ull << 12,
        eVehicleEngineAPI               = 1ull << 13,
        eVehicleGearsAPI                = 1ull << 14,
        eVehicleClutchAPI               = 1ull << 15,
        eVehicleDriveBasicAPI           = 1ull << 16,
        eVehicleMultiWheelDifferentialAPI = 1ull << 17,
        eVehicleTankDifferentialAPI     = 1ull << 18,
        eVehicleAutoGearBoxAPI          = 1ull << 19,
        eVehicleBrakesAPI               = 1ull << 20,
        eVehicleSteeringAPI             = 1ull << 21,
        eVehicleAckermannSteeringAPI    = 1ull << 22,
        eVehicleNonlinearCmdResponseAPI = 1ull << 23,
        eVehicleDriveStandardAPI        = 1ull << 24,
        eVehicleWheelAttachmentAPI      = 1ull << 25,
        eVehicleSuspensionComplianceAPI = 1ull << 26,
        eVehicleAPI                     = 1ull << 27,
        eParticleSetAPI                 = 1ull << 28,
        eParticleSamplingAPI            = 1ull << 29,
        ePBDMaterialAPI                 = 1ull << 30,
    };
};

// Prim-type classification by schema TfType, computed once via
// `IsA` chains. Stored as a bitmask for fast dispatch in the walker.
struct PrimTypeBits
{
    enum Enum : uint64_t
    {
        eUsdGeomImageable               = 1ull << 0,
        eUsdGeomXformable               = 1ull << 1,
        eUsdGeomGprim                   = 1ull << 2,
        eUsdGeomMesh                    = 1ull << 3,
        eUsdGeomTetMesh                 = 1ull << 4,
        eUsdGeomBasisCurves             = 1ull << 5,
        eUsdGeomPointInstancer          = 1ull << 6,
        eUsdPhysicsJoint                = 1ull << 7,
        eUsdPhysicsRevoluteJoint        = 1ull << 8,
        eUsdPhysicsFixedJoint           = 1ull << 9,
        eUsdPhysicsSphericalJoint       = 1ull << 10,
        eUsdPhysicsDistanceJoint        = 1ull << 11,
        eUsdPhysicsPrismaticJoint       = 1ull << 12,
        eUsdPhysicsScene                = 1ull << 13,
        eUsdPhysicsCollisionGroup       = 1ull << 14,
        eUsdPhysicsAttachment           = 1ull << 15,
        eUsdPhysicsElementCollisionFilter = 1ull << 16,
        ePhysxVehicleTireFrictionTable    = 1ull << 17,
        ePhysxParticleSystem              = 1ull << 18,
    };
};

// Cached TfTypes for the 8 OmniPhysics deformable-attachment schema
// types + ElementCollisionFilter. Looked up once at first use via
// UsdSchemaRegistry::GetTypeFromSchemaTypeName.
struct AttachmentTypes
{
    PXR_NS::TfType base;
    PXR_NS::TfType vtxVtx;
    PXR_NS::TfType vtxTri;
    PXR_NS::TfType vtxTet;
    PXR_NS::TfType vtxCrv;
    PXR_NS::TfType vtxXform;
    PXR_NS::TfType tetXform;
    PXR_NS::TfType triTri;
    PXR_NS::TfType elementFilter;
};

const AttachmentTypes& attachmentTypes()
{
    static const AttachmentTypes types = []() {
        AttachmentTypes t;
        auto get = [](const char* name) {
            return PXR_NS::UsdSchemaRegistry::GetTypeFromSchemaTypeName(PXR_NS::TfToken(name));
        };
        t.base          = get("OmniPhysicsAttachment");
        t.vtxVtx        = get("OmniPhysicsVtxVtxAttachment");
        t.vtxTri        = get("OmniPhysicsVtxTriAttachment");
        t.vtxTet        = get("OmniPhysicsVtxTetAttachment");
        t.vtxCrv        = get("OmniPhysicsVtxCrvAttachment");
        t.vtxXform      = get("OmniPhysicsVtxXformAttachment");
        t.tetXform      = get("OmniPhysicsTetXformAttachment");
        t.triTri        = get("OmniPhysicsTriTriAttachment");
        t.elementFilter = get("OmniPhysicsElementCollisionFilter");
        return t;
    }();
    return types;
}

uint64_t classifyPrimType(const PXR_NS::TfType& t)
{
    uint64_t types = 0;
    if (t.IsA<PXR_NS::UsdGeomImageable>())
    {
        types |= PrimTypeBits::eUsdGeomImageable;
        if (t.IsA<PXR_NS::UsdGeomXformable>())
        {
            types |= PrimTypeBits::eUsdGeomXformable;
            if (t.IsA<PXR_NS::UsdGeomGprim>())
            {
                types |= PrimTypeBits::eUsdGeomGprim;
                if (t.IsA<PXR_NS::UsdGeomMesh>())
                    types |= PrimTypeBits::eUsdGeomMesh;
                else if (t.IsA<PXR_NS::UsdGeomTetMesh>())
                    types |= PrimTypeBits::eUsdGeomTetMesh;
                else if (t.IsA<PXR_NS::UsdGeomBasisCurves>())
                    types |= PrimTypeBits::eUsdGeomBasisCurves;
                // PhysxParticleSystem is a UsdGeomGprim subclass, so it is
                // classified here (the dispatch checks ePhysxParticleSystem
                // before the shape/body branch).
                else if (t.IsA<PXR_NS::PhysxSchemaPhysxParticleSystem>())
                    types |= PrimTypeBits::ePhysxParticleSystem;
            }
            else if (t.IsA<PXR_NS::UsdGeomPointInstancer>())
            {
                types |= PrimTypeBits::eUsdGeomPointInstancer;
            }
        }
        else if (t.IsA<PXR_NS::UsdPhysicsJoint>())
        {
            types |= PrimTypeBits::eUsdPhysicsJoint;
            if (t.IsA<PXR_NS::UsdPhysicsRevoluteJoint>())
                types |= PrimTypeBits::eUsdPhysicsRevoluteJoint;
            else if (t.IsA<PXR_NS::UsdPhysicsFixedJoint>())
                types |= PrimTypeBits::eUsdPhysicsFixedJoint;
            else if (t.IsA<PXR_NS::UsdPhysicsSphericalJoint>())
                types |= PrimTypeBits::eUsdPhysicsSphericalJoint;
            else if (t.IsA<PXR_NS::UsdPhysicsDistanceJoint>())
                types |= PrimTypeBits::eUsdPhysicsDistanceJoint;
            else if (t.IsA<PXR_NS::UsdPhysicsPrismaticJoint>())
                types |= PrimTypeBits::eUsdPhysicsPrismaticJoint;
        }
        else
        {
            // OmniPhysics deformable-attachment + element-collision-
            // filter schemas — runtime-registered types looked up via
            // UsdSchemaRegistry.
            const AttachmentTypes& at = attachmentTypes();
            if (at.base && t.IsA(at.base))
                types |= PrimTypeBits::eUsdPhysicsAttachment;
            else if (at.elementFilter && t.IsA(at.elementFilter))
                types |= PrimTypeBits::eUsdPhysicsElementCollisionFilter;
        }
    }
    else if (t.IsA<PXR_NS::UsdPhysicsScene>())
    {
        types |= PrimTypeBits::eUsdPhysicsScene;
    }
    else if (t.IsA<PXR_NS::UsdPhysicsCollisionGroup>())
    {
        types |= PrimTypeBits::eUsdPhysicsCollisionGroup;
    }
    else if (t.IsA<PXR_NS::PhysxSchemaPhysxVehicleTireFrictionTable>())
    {
        types |= PrimTypeBits::ePhysxVehicleTireFrictionTable;
    }
    return types;
}

// Resolve an attachment subtype. Returns parse::eUndefined when the
// prim isn't a typed attachment subclass.
parse::ObjectType resolveAttachmentSubtype(const PXR_NS::UsdPrim& prim)
{
    const AttachmentTypes& at = attachmentTypes();
    if (at.vtxVtx   && prim.IsA(at.vtxVtx))   return parse::eAttachmentVtxVtx;
    if (at.vtxTri   && prim.IsA(at.vtxTri))   return parse::eAttachmentVtxTri;
    if (at.vtxTet   && prim.IsA(at.vtxTet))   return parse::eAttachmentVtxTet;
    if (at.vtxCrv   && prim.IsA(at.vtxCrv))   return parse::eAttachmentVtxCrv;
    if (at.vtxXform && prim.IsA(at.vtxXform)) return parse::eAttachmentVtxXform;
    if (at.tetXform && prim.IsA(at.tetXform)) return parse::eAttachmentTetXform;
    if (at.triTri   && prim.IsA(at.triTri))   return parse::eAttachmentTriTri;
    return parse::eUndefined;
}

uint64_t classifyApiSchemas(const PXR_NS::TfTokenVector& apis)
{
    static const PXR_NS::TfToken kArticulationRootAPI("PhysicsArticulationRootAPI");
    static const PXR_NS::TfToken kParticleSetAPI("PhysxParticleSetAPI");
    static const PXR_NS::TfToken kParticleSamplingAPI("PhysxParticleSamplingAPI");
    static const PXR_NS::TfToken kCollisionAPI("PhysicsCollisionAPI");
    static const PXR_NS::TfToken kRigidBodyAPI("PhysicsRigidBodyAPI");
    static const PXR_NS::TfToken kDeformableBodyAPI("OmniPhysicsDeformableBodyAPI");
    static const PXR_NS::TfToken kMaterialAPI("PhysicsMaterialAPI");
    static const PXR_NS::TfToken kPBDMaterialAPI("PhysxPBDMaterialAPI");
    static const PXR_NS::TfToken kDeformableMaterialAPI("OmniPhysicsDeformableMaterialAPI");
    static const PXR_NS::TfToken kSurfaceDeformableMaterialAPI("OmniPhysicsSurfaceDeformableMaterialAPI");
    static const PXR_NS::TfToken kCurvesDeformableMaterialAPI("OmniPhysicsCurveDeformableMaterialAPI");
    static const PXR_NS::TfToken kCharacterControllerAPI("PhysxCharacterControllerAPI");
    static const PXR_NS::TfToken kVehicleContextAPI("PhysxVehicleContextAPI");
    static const PXR_NS::TfToken kVehicleWheelAPI("PhysxVehicleWheelAPI");
    static const PXR_NS::TfToken kVehicleTireAPI("PhysxVehicleTireAPI");
    static const PXR_NS::TfToken kVehicleSuspensionAPI("PhysxVehicleSuspensionAPI");
    static const PXR_NS::TfToken kVehicleEngineAPI("PhysxVehicleEngineAPI");
    static const PXR_NS::TfToken kVehicleGearsAPI("PhysxVehicleGearsAPI");
    static const PXR_NS::TfToken kVehicleClutchAPI("PhysxVehicleClutchAPI");
    static const PXR_NS::TfToken kVehicleDriveBasicAPI("PhysxVehicleDriveBasicAPI");
    static const PXR_NS::TfToken kVehicleMultiWheelDifferentialAPI("PhysxVehicleMultiWheelDifferentialAPI");
    static const PXR_NS::TfToken kVehicleTankDifferentialAPI("PhysxVehicleTankDifferentialAPI");
    static const PXR_NS::TfToken kVehicleAutoGearBoxAPI("PhysxVehicleAutoGearBoxAPI");
    static const PXR_NS::TfToken kVehicleSteeringAPI("PhysxVehicleSteeringAPI");
    static const PXR_NS::TfToken kVehicleAckermannSteeringAPI("PhysxVehicleAckermannSteeringAPI");
    static const PXR_NS::TfToken kVehicleDriveStandardAPI("PhysxVehicleDriveStandardAPI");
    static const PXR_NS::TfToken kVehicleWheelAttachmentAPI("PhysxVehicleWheelAttachmentAPI");
    static const PXR_NS::TfToken kVehicleSuspensionComplianceAPI("PhysxVehicleSuspensionComplianceAPI");
    static const PXR_NS::TfToken kVehicleAPI("PhysxVehicleAPI");
    static const std::string kVehicleBrakesAPIPrefix = "PhysxVehicleBrakesAPI:";
    static const std::string kVehicleNonlinearCmdResponseAPIPrefix = "PhysxVehicleNonlinearCommandResponseAPI:";

    uint64_t flags = 0;
    for (const PXR_NS::TfToken& t : apis)
    {
        if      (t == kArticulationRootAPI)          flags |= ApiFlag::eArticulationRootAPI;
        else if (t == kParticleSetAPI)               flags |= ApiFlag::eParticleSetAPI;
        else if (t == kParticleSamplingAPI)          flags |= ApiFlag::eParticleSamplingAPI;
        else if (t == kCollisionAPI)                 flags |= ApiFlag::eCollisionAPI;
        else if (t == kRigidBodyAPI)                 flags |= ApiFlag::eRigidBodyAPI;
        else if (t == kDeformableBodyAPI)            flags |= ApiFlag::eDeformableBodyAPI;
        else if (t == kMaterialAPI)                  flags |= ApiFlag::eMaterialAPI;
        else if (t == kPBDMaterialAPI)               flags |= ApiFlag::ePBDMaterialAPI;
        else if (t == kDeformableMaterialAPI)        flags |= ApiFlag::eDeformableMaterialAPI;
        else if (t == kSurfaceDeformableMaterialAPI) flags |= ApiFlag::eSurfaceDeformableMaterialAPI;
        else if (t == kCurvesDeformableMaterialAPI)  flags |= ApiFlag::eCurvesDeformableMaterialAPI;
        else if (t == kCharacterControllerAPI)       flags |= ApiFlag::eCharacterControllerAPI;
        else if (t == kVehicleContextAPI)            flags |= ApiFlag::eVehicleContextAPI;
        else if (t == kVehicleWheelAPI)              flags |= ApiFlag::eVehicleWheelAPI;
        else if (t == kVehicleTireAPI)               flags |= ApiFlag::eVehicleTireAPI;
        else if (t == kVehicleSuspensionAPI)         flags |= ApiFlag::eVehicleSuspensionAPI;
        else if (t == kVehicleEngineAPI)             flags |= ApiFlag::eVehicleEngineAPI;
        else if (t == kVehicleGearsAPI)              flags |= ApiFlag::eVehicleGearsAPI;
        else if (t == kVehicleClutchAPI)             flags |= ApiFlag::eVehicleClutchAPI;
        else if (t == kVehicleDriveBasicAPI)         flags |= ApiFlag::eVehicleDriveBasicAPI;
        else if (t == kVehicleMultiWheelDifferentialAPI) flags |= ApiFlag::eVehicleMultiWheelDifferentialAPI;
        else if (t == kVehicleTankDifferentialAPI)   flags |= ApiFlag::eVehicleTankDifferentialAPI;
        else if (t == kVehicleAutoGearBoxAPI)        flags |= ApiFlag::eVehicleAutoGearBoxAPI;
        else if (t == kVehicleSteeringAPI)           flags |= ApiFlag::eVehicleSteeringAPI;
        else if (t == kVehicleAckermannSteeringAPI)  flags |= ApiFlag::eVehicleAckermannSteeringAPI;
        else if (t == kVehicleDriveStandardAPI)      flags |= ApiFlag::eVehicleDriveStandardAPI;
        else if (t == kVehicleWheelAttachmentAPI)    flags |= ApiFlag::eVehicleWheelAttachmentAPI;
        else if (t == kVehicleSuspensionComplianceAPI) flags |= ApiFlag::eVehicleSuspensionComplianceAPI;
        else if (t == kVehicleAPI)                   flags |= ApiFlag::eVehicleAPI;
        else
        {
            // BrakesAPI is multi-apply: applied tokens carry a per-instance
            // suffix like "PhysxVehicleBrakesAPI:brakes0". Match the
            // base via prefix.
            const std::string& s = t.GetString();
            if (s.size() >= kVehicleBrakesAPIPrefix.size() &&
                s.compare(0, kVehicleBrakesAPIPrefix.size(), kVehicleBrakesAPIPrefix) == 0)
            {
                flags |= ApiFlag::eVehicleBrakesAPI;
            }
            else if (s.size() >= kVehicleNonlinearCmdResponseAPIPrefix.size() &&
                     s.compare(0, kVehicleNonlinearCmdResponseAPIPrefix.size(),
                               kVehicleNonlinearCmdResponseAPIPrefix) == 0)
            {
                flags |= ApiFlag::eVehicleNonlinearCmdResponseAPI;
            }
        }
    }

    // Material-API drop: when a prim carries a material API AND any
    // non-material physics API, the material interpretation is silently
    // dropped (invalid configuration — body / shape / etc. wins).
    constexpr uint64_t kMaterialAPIs =
        ApiFlag::eMaterialAPI | ApiFlag::ePBDMaterialAPI | ApiFlag::eDeformableMaterialAPI |
        ApiFlag::eSurfaceDeformableMaterialAPI | ApiFlag::eCurvesDeformableMaterialAPI;
    const uint64_t matFlags = flags & kMaterialAPIs;
    const uint64_t otherFlags = flags & ~kMaterialAPIs;
    if (matFlags && otherFlags)
        flags &= otherFlags;

    return flags;
}

// ---------------------------------------------------------------------------
// Small helpers.
// ---------------------------------------------------------------------------

inline carb::Float3 toFloat3(const PXR_NS::GfVec3f& v) { return { v[0], v[1], v[2] }; }
inline carb::Float4 toFloat4(const PXR_NS::GfQuatf& q)
{
    const PXR_NS::GfVec3f& im = q.GetImaginary();
    return { im[0], im[1], im[2], q.GetReal() };
}

inline bool scaleIsUniform(double x, double y, double z)
{
    constexpr double tol = 1e-4;
    return std::abs(x - y) <= tol && std::abs(x - z) <= tol && std::abs(y - z) <= tol;
}

void checkNonUniformScale(const PXR_NS::GfVec3d& s, const PXR_NS::SdfPath& primKey)
{
    if (!scaleIsUniform(s[0], s[1], s[2]))
    {
        CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s",
                      primKey.GetText());
    }
}

// getMaterialBindingPath is defined at namespace scope (below the anonymous
// namespace) so UsdSource can reuse the single copy; declared in NativeWalker.h.

// ---------------------------------------------------------------------------
// Pass-1 walker state. Lives for the duration of one scanStageNative
// call; consumed by the pass-2 ancestor walk + finalizeCollision.
// ---------------------------------------------------------------------------

// Walk context: the concrete UsdSource backing the scan. The emit helpers
// resolve handles through `impl.source.*` (UsdSource-specific resolvers like
// keyFor(SdfPath) that are not on the abstract IPhysicsSource). A struct
// (rather than a bare `UsdSource&` parameter) keeps the `impl.source.` call
// syntax stable and leaves room for future walk-scoped state.
struct UsdWalkCtx
{
    UsdSource& source;
    PXR_NS::UsdStageWeakPtr stage;
    // The scan being built — emit helpers that lack a direct `out` parameter
    // append owned auxiliary storage (merged meshes) through `scan`.
    parse::ScannedStage& scan;
};

struct WalkState
{
    // Lazily-built xform cache for transform reads not routed through
    // IPhysicsSource (UsdGeomXformCache::GetResetXformStack, etc.).
    PXR_NS::UsdGeomXformCache xfCache;

    // Body lookup: ObjectKey → raw pointer into out.bodies (non-owning).
    // Used by pass-2 ancestor walk to resolve body-of-shape. The
    // `rigidBodyEnabled` bit lives on the desc; static bodies are
    // also recorded so the ancestor walk's "stop at the first body-API
    // ancestor" semantics see them.
    struct BodyEntry
    {
        parse::PhysxRigidBodyDesc* desc = nullptr;
        bool                       rigidBodyEnabled = false;
    };
    std::unordered_map<parse::ObjectKey, BodyEntry, parse::ObjectKey::Hash> bodyMap;

    // Deformable-body lookup — keyed on body prim's ObjectKey, value is
    // whether that body's enabled (i.e. participates in collider-is-
    // deformable detection).
    std::unordered_map<parse::ObjectKey, bool, parse::ObjectKey::Hash> deformableBodyMap;

    // Shape list in insertion order. Each entry maps shape's prim key
    // → raw pointer into out.shapes (non-owning). Pass-2 iterates this
    // to populate body-of-shape refs + local transforms.
    std::vector<std::pair<parse::ObjectKey, parse::PhysxShapeDesc*>> shapeList;

    // Per-walk instance-proxy descriptor cache — keyed on the prototype
    // prim's ObjectKey. When a UsdPhysicsCollisionAPI prim is an
    // instance proxy, we parse the prototype once and produce a fresh
    // descriptor per instance with the instance's world-space scale
    // baked in via parse::scaleShapeDescByInstance. The cache here
    // exists to skip the per-prototype mesh-attribute reads on hits.
    // The per-instance desc is always freshly allocated — at the
    // parse-lib layer each instance owns its own PhysxShapeDesc (no
    // master-mutation hazard at this level).
    std::unordered_set<parse::ObjectKey, parse::ObjectKey::Hash> seenInstanceProtoKeys;

    // Joint lookup — keyed on joint prim's ObjectKey, holds the
    // resolved body0/body1 + the per-joint enable flag. Consumed by
    // the pass-3 articulation aggregation (BodyJointMap construction).
    // The desc pointer is non-owning (out.joints holds the unique_ptr).
    struct JointEntry
    {
        parse::PhysxJointDesc* desc = nullptr;
        parse::ObjectKey       body0;
        parse::ObjectKey       body1;
        bool                   jointEnabled = true;
        bool                   excludeFromArticulation = false;
        std::vector<std::pair<parse::JointAxis, parse::JointLimitInfo>> jointLimits;
    };
    std::unordered_map<parse::ObjectKey, JointEntry, parse::ObjectKey::Hash> jointMap;

    // Joint emit order — drives deterministic body-joint iteration in
    // pass-3 so root election is order-stable when weights tie.
    std::vector<parse::ObjectKey> jointOrder;

    // Articulation lookup — keyed on the articulation-root prim's
    // ObjectKey, holds the PhysxArticulationAPI extension fields +
    // articulation-level filtered pairs collected during pass-1.
    // Articulation descs are NOT emitted during pass-1; pass-3 root
    // election walks each entry, computes rootPrims (one or more),
    // and inserts a `PhysxArticulationDesc` per resolved root into
    // out.articulations.
    struct ArticulationEntry
    {
        parse::ArticulationFields  fields;
        std::vector<parse::ObjectKey> sourceFilteredCollisions;
        PXR_NS::SdfPath            path; // for hierarchy walk via UsdPrimRange
    };
    std::unordered_map<parse::ObjectKey, ArticulationEntry, parse::ObjectKey::Hash> articulationMap;
    std::vector<parse::ObjectKey> articulationOrder;
};

// ---------------------------------------------------------------------------
// Per-concept emit helpers — called from the walker loop.
// ---------------------------------------------------------------------------

// Read gravity direction + magnitude from a UsdPhysicsScene prim and
// apply stage-derived defaults (negative up-axis when direction is
// unauthored; 9.81 / metersPerUnit when magnitude is at its sentinel).
void readSceneGravity(const PXR_NS::UsdPrim& scenePrim,
                      PXR_NS::UsdStageWeakPtr stage,
                      PXR_NS::GfVec3f& outDir,
                      float& outMagnitude)
{
    const PXR_NS::UsdPhysicsScene scene(scenePrim);

    scene.GetGravityDirectionAttr().Get(&outDir);
    if (outDir == PXR_NS::GfVec3f(0.0f))
    {
        const PXR_NS::TfToken upAxis = PXR_NS::UsdGeomGetStageUpAxis(stage);
        if (upAxis == PXR_NS::UsdGeomTokens.Get()->x)
            outDir = PXR_NS::GfVec3f(-1.0f, 0.0f, 0.0f);
        else if (upAxis == PXR_NS::UsdGeomTokens.Get()->y)
            outDir = PXR_NS::GfVec3f(0.0f, -1.0f, 0.0f);
        else
            outDir = PXR_NS::GfVec3f(0.0f, 0.0f, -1.0f);
    }
    else
    {
        outDir.Normalize();
    }

    scene.GetGravityMagnitudeAttr().Get(&outMagnitude);
    if (outMagnitude < -0.5e38f)
    {
        const float metersPerUnit = static_cast<float>(PXR_NS::UsdGeomGetStageMetersPerUnit(stage));
        outMagnitude = 9.81f / metersPerUnit;
    }
}

void emitScene(ScannedStage& out, UsdWalkCtx& impl,
               parse::ParseContext& ctx, parse::ObjectKey key,
               const PXR_NS::UsdPrim& scenePrim)
{
    parse::DescPtr<parse::PhysxSceneDesc> desc = parse::allocateDesc<parse::PhysxSceneDesc>(ctx.descriptorAllocator());
    parse::setToDefault(*desc, impl.source.getSourceUnits());

    PXR_NS::GfVec3f gravDir;
    float gravMag = 0.0f;
    readSceneGravity(scenePrim, impl.stage, gravDir, gravMag);

    parse::SceneInfo info;
    info.gravityDirection = { gravDir[0], gravDir[1], gravDir[2] };
    info.gravityMagnitude = gravMag;
    parse::parseScene(ctx, key, info, *desc);
    desc->primKey = key;

    // Nested scene-material binding overlay.
    const parse::ObjectKey materialKey = impl.source.getMaterialBinding(key);
    if (materialKey.valid())
    {
        if (parse::DescPtr<parse::PhysxMaterialDesc> matRaw = parse::parseMaterial(ctx, materialKey))
            desc->defaultMaterialDesc = *matRaw;
        parse::parseDeformableMaterial(ctx, materialKey, desc->defaultDeformableMaterialDesc);
        parse::parseSurfaceDeformableMaterial(ctx, materialKey, desc->defaultSurfaceDeformableMaterialDesc);
        if (parse::DescPtr<parse::PBDMaterialDesc> pbdRaw = parse::parsePBDMaterial(ctx, materialKey))
            desc->defaultPBDMaterialDesc = *pbdRaw;
    }

    out.scenes.push_back(std::move(desc));
}

void emitMaterial(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::PhysxMaterialDesc> raw = parse::parseMaterial(ctx, key))
        out.materials.push_back(std::move(raw));
}

void emitPBDMaterial(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::PBDMaterialDesc> raw = parse::parsePBDMaterial(ctx, key))
        out.pbdMaterials.push_back(std::move(raw));
}

void emitDeformableMaterial(ScannedStage& out, parse::ParseContext& ctx,
                            parse::ObjectKey key, uint64_t apiFlags)
{
    parse::DescPtr<parse::PhysxDeformableMaterialDesc> desc;
    if (apiFlags & ApiFlag::eSurfaceDeformableMaterialAPI)
    {
        parse::DescPtr<parse::PhysxSurfaceDeformableMaterialDesc> surf = parse::allocateDesc<parse::PhysxSurfaceDeformableMaterialDesc>(ctx.descriptorAllocator());
        parse::parseSurfaceDeformableMaterial(ctx, key, *surf);
        desc = parse::descPtrCast<parse::PhysxDeformableMaterialDesc>(std::move(surf));
    }
    else
    {
        desc = parse::allocateDesc<parse::PhysxDeformableMaterialDesc>(ctx.descriptorAllocator());
        parse::parseDeformableMaterial(ctx, key, *desc);
    }
    desc->poissonsRatio = std::min(desc->poissonsRatio, 0.4999f);
    desc->materialKey = key;
    out.deformableMaterials.push_back(std::move(desc));
}

void emitCollisionGroup(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    parse::CollisionGroupInfo info = parse::parseCollisionGroup(ctx, key);
    parse::DescPtr<parse::CollisionGroupDesc> desc = parse::allocateDesc<parse::CollisionGroupDesc>(ctx.descriptorAllocator());
    desc->primKey = key;
    desc->sourceFilteredGroups = std::move(info.filteredGroups);
    desc->sourceMembers        = std::move(info.members);
    out.collisionGroups.push_back(std::move(desc));
}

// Attachment emit. parse::parseAttachment does the
// attachmentEnabled / src0 / src1 / damping / stiffness reads; the
// walker is only responsible for subtype classification + primKey.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-ATTACH-001
// @covers AC-1 AC-2
void emitAttachment(ScannedStage& out, parse::ParseContext& ctx,
                    parse::ObjectKey key, const PXR_NS::UsdPrim& prim)
{
    const parse::ObjectType subtype = resolveAttachmentSubtype(prim);
    if (subtype == parse::eUndefined)
        return; // base-typed attachment with no known subclass — skip
    if (parse::DescPtr<parse::PhysxDeformableAttachmentDesc> raw = parse::parseAttachment(ctx, key, subtype))
    {
        raw->primKey = key;
        out.attachments.push_back(std::move(raw));
    }
}

// Element collision filter emit.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-FILTER-001
// @covers AC-1
void emitElementCollisionFilter(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::PhysxDeformableCollisionFilterDesc> raw = parse::parseElementCollisionFilter(ctx, key))
    {
        raw->primKey = key;
        out.deformableCollisionFilters.push_back(std::move(raw));
    }
}

// Tire friction table emit. PhysxVehicleTireFrictionTable typed prim.
// Walker pre-reads the friction-values float[] and the groundMaterials
// rel targets, passes them via TireFrictionTableInfo; the parser
// validates the materials have PhysicsMaterialAPI and emits.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-TIREFRICTION-001
// @covers AC-4
void emitTireFrictionTable(ScannedStage& out, UsdWalkCtx& impl,
                           parse::ParseContext& ctx, parse::ObjectKey key,
                           const PXR_NS::UsdPrim& prim)
{
    PXR_NS::PhysxSchemaPhysxVehicleTireFrictionTable tft(prim);

    parse::TireFrictionTableInfo info;
    info.defaultFrictionValue = 1.0f;
    if (PXR_NS::UsdAttribute defAttr = tft.GetDefaultFrictionValueAttr())
    {
        defAttr.Get(&info.defaultFrictionValue);
    }

    if (PXR_NS::UsdRelationship matRel = tft.GetGroundMaterialsRel())
    {
        PXR_NS::SdfPathVector paths;
        matRel.GetTargets(&paths);
        info.materialPaths.reserve(paths.size());
        for (const PXR_NS::SdfPath& p : paths)
            info.materialPaths.push_back(impl.source.keyFor(p));
    }

    if (PXR_NS::UsdAttribute fvAttr = tft.GetFrictionValuesAttr())
    {
        PXR_NS::VtFloatArray arr;
        fvAttr.Get(&arr);
        info.frictionValues.assign(arr.cdata(), arr.cdata() + arr.size());
    }

    if (parse::DescPtr<parse::TireFrictionTableDesc> desc =
            parse::parseTireFrictionTable(ctx, key, info))
        out.tireFrictionTables.push_back(std::move(desc));
}

// Particle system emit. PhysxParticleSystem typed prim. parseParticleSystem
// reads all fields through the source, so no Info struct / prim is needed here.
void emitParticleSystem(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    parse::DescPtr<parse::ParticleSystemDesc> sys = parse::parseParticleSystem(ctx, key);
    if (!sys)
        return;
    // Sub-API descriptors live on the same prim; isosurface autocompletion needs
    // the system's resolved fluidRestOffset (captured before the move).
    const float fluidRestOffset = sys->fluidRestOffset;
    out.particleSystems.push_back(std::move(sys));

    if (parse::DescPtr<parse::ParticleAnisotropyDesc> a = parse::parseParticleAnisotropy(ctx, key))
        out.particleAnisotropies.push_back(std::move(a));
    if (parse::DescPtr<parse::ParticleSmoothingDesc> s = parse::parseParticleSmoothing(ctx, key))
        out.particleSmoothings.push_back(std::move(s));
    if (parse::DescPtr<parse::ParticleIsosurfaceDesc> i = parse::parseParticleIsosurface(ctx, key, fluidRestOffset))
        out.particleIsosurfaces.push_back(std::move(i));
}

// Particle set emit. PhysxParticleSetAPI on a UsdGeomPointBased / PointInstancer
// prim (dispatched on the API flag, before the shape/body branch).
void emitParticleSet(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::ParticleSetDesc> desc = parse::parseParticleSet(ctx, key))
        out.particleSets.push_back(std::move(desc));
}

// Particle sampler emit. PhysxParticleSamplingAPI on a UsdGeomMesh. The desc has
// no own key, so the mesh key is recorded in the parallel particleSamplerKeys.
void emitParticleSampler(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::ParticleSamplingDesc> desc = parse::parseParticleSampling(ctx, key))
    {
        out.particleSamplers.push_back(std::move(desc));
        out.particleSamplerKeys.push_back(key);
    }
}

// Vehicle context emit. Per ADR-0008, parse-lib trusts the caller to
// have already validated that the prim is a UsdPhysicsScene-typed prim
// (parse-lib has no `IsA<UsdPhysicsScene>` query). The walker does
// this check before invoking.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-CONTEXT-001
// @covers AC-4
void emitVehicleContext(ScannedStage& out, parse::ParseContext& ctx,
                        parse::ObjectKey key, const PXR_NS::UsdPrim& prim)
{
    if (!prim.IsA<PXR_NS::UsdPhysicsScene>())
    {
        CARB_LOG_ERROR("Usd Physics: \"%s\": PhysxVehicleContextAPI requires to be applied to a PhysicsScene prim.",
                       prim.GetName().GetText());
        return;
    }
    if (parse::DescPtr<parse::VehicleContextDesc> desc = parse::parseVehicleContext(ctx, key))
        out.vehicleContexts.push_back(std::move(desc));
}

// Wheel emit. PhysxVehicleWheelAPI applied on any prim — parse-lib
// does pure attribute reads. The walker emits one descriptor per
// applied API; dedup across vehicles is the consumer adapter's job.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-4
void emitVehicleWheel(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::WheelDesc> desc = parse::parseWheel(ctx, key))
        out.vehicleWheels.push_back(std::move(desc));
}

// Tire emit. Walker pre-reads `frictionVsSlipGraph` (VtArray<GfVec2f>,
// exactly 3 entries when authored) and the `frictionTable`
// relationship target (single rel target with
// PhysxVehicleTireFrictionTable type; >1 targets => reject). Mass
// scale comes from the stage units (1 / kilogramsPerUnit).
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-4
void emitVehicleTire(ScannedStage& out, UsdWalkCtx& impl,
                     parse::ParseContext& ctx, parse::ObjectKey key,
                     const PXR_NS::UsdPrim& prim, float massScale)
{
    PXR_NS::PhysxSchemaPhysxVehicleTireAPI tireAPI(prim);

    parse::TireInfo info;
    info.massScale = massScale;

    if (PXR_NS::UsdAttribute fvsAttr = tireAPI.GetFrictionVsSlipGraphAttr())
    {
        if (fvsAttr.HasAuthoredValue())
        {
            PXR_NS::VtArray<PXR_NS::GfVec2f> arr;
            fvsAttr.Get(&arr);
            if (arr.size() == 3)
            {
                info.hasFrictionVsSlipGraph = true;
                for (uint32_t i = 0; i < 3; ++i)
                    info.frictionVsSlipGraph[i] = { arr[i][0], arr[i][1] };
            }
            else
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"frictionVsSlipGraph\" of tire \"%s\" needs to have exactly 3 entries.",
                               prim.GetName().GetText());
                return;
            }
        }
    }

    if (PXR_NS::UsdRelationship ftRel = tireAPI.GetFrictionTableRel())
    {
        if (ftRel.HasAuthoredTargets())
        {
            PXR_NS::SdfPathVector paths;
            ftRel.GetTargets(&paths);
            if (paths.size() == 1)
            {
                const PXR_NS::UsdPrim ftPrim = prim.GetStage()->GetPrimAtPath(paths[0]);
                if (!ftPrim)
                {
                    CARB_LOG_ERROR("Usd Physics: tire \"%s\": \"frictionTable\" relationship has nonexistent path \"%s\".",
                                   prim.GetName().GetText(), paths[0].GetText());
                    info.frictionTableTooMany = true;
                }
                else if (!ftPrim.IsA<PXR_NS::PhysxSchemaPhysxVehicleTireFrictionTable>())
                {
                    CARB_LOG_ERROR("Usd Physics: tire \"%s\": \"frictionTable\" relationship does not point to a "
                                   "PhysxVehicleTireFrictionTable prim (\"%s\").",
                                   prim.GetName().GetText(), paths[0].GetText());
                    info.frictionTableTooMany = true;
                }
                else
                {
                    info.frictionTableKey = impl.source.keyFor(paths[0]);
                }
            }
            else if (paths.size() > 1)
            {
                CARB_LOG_ERROR("Usd Physics: \"%s\" must not have more than 1 \"frictionTable\" relationship defined.",
                               prim.GetName().GetText());
                info.frictionTableTooMany = true;
            }
        }
    }

    if (parse::DescPtr<parse::TireDesc> desc = parse::parseTire(ctx, key, info))
        out.vehicleTires.push_back(std::move(desc));
}

// Suspension emit. Pure attribute reads.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-4
void emitVehicleSuspension(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::SuspensionDesc> desc = parse::parseSuspension(ctx, key))
        out.vehicleSuspensions.push_back(std::move(desc));
}

// Engine emit. Walker pre-reads torqueCurve (VtArray<GfVec2f>);
// parse-lib reads the scalar damping rates + idle/maxRotationSpeed.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-4
void emitVehicleEngine(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key,
                       const PXR_NS::UsdPrim& prim, float kgmsScale)
{
    PXR_NS::PhysxSchemaPhysxVehicleEngineAPI engineAPI(prim);

    parse::EngineInfo info;
    info.kgmsScale = kgmsScale;

    if (PXR_NS::UsdAttribute tcAttr = engineAPI.GetTorqueCurveAttr())
    {
        if (tcAttr.HasAuthoredValue())
        {
            PXR_NS::VtArray<PXR_NS::GfVec2f> arr;
            tcAttr.Get(&arr);
            info.hasTorqueCurve = true;
            info.torqueCurvePointCount = static_cast<uint32_t>(arr.size());
            const uint32_t copyCount = std::min(info.torqueCurvePointCount,
                static_cast<uint32_t>(parse::EngineDesc::maxNumberOfTorqueCurvePoints));
            for (uint32_t i = 0; i < copyCount; ++i)
                info.torqueCurve[i] = { arr[i][0], arr[i][1] };
        }
    }

    if (parse::DescPtr<parse::EngineDesc> desc = parse::parseEngine(ctx, key, info))
        out.vehicleEngines.push_back(std::move(desc));
}

// Gears emit. Walker pre-reads ratios (VtArray<float>).
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-4
void emitVehicleGears(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key,
                      const PXR_NS::UsdPrim& prim)
{
    PXR_NS::PhysxSchemaPhysxVehicleGearsAPI gearsAPI(prim);

    parse::GearsInfo info;
    if (PXR_NS::UsdAttribute rAttr = gearsAPI.GetRatiosAttr())
    {
        if (rAttr.HasAuthoredValue())
        {
            PXR_NS::VtArray<float> arr;
            rAttr.Get(&arr);
            info.hasRatios = true;
            info.ratios.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }

    if (parse::DescPtr<parse::GearsDesc> desc = parse::parseGears(ctx, key, info))
    {
        out.vehicleGears.push_back(std::move(desc));
        out.vehicleGearsPaths.push_back(key);
    }
}

// Clutch emit. Pure attribute read.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-DRIVETRAIN-001
// @covers AC-4
void emitVehicleClutch(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key,
                       float kgmsScale)
{
    if (parse::DescPtr<parse::ClutchDesc> desc = parse::parseClutch(ctx, key, kgmsScale))
    {
        out.vehicleClutches.push_back(std::move(desc));
        out.vehicleClutchPaths.push_back(key);
    }
}

// Forward decl: rel-or-API resolution helper used by WheelAttachment
// and DriveStandard; definition is further down.
parse::ObjectKey resolveRelOrApi(const PXR_NS::UsdPrim& prim, const PXR_NS::UsdRelationship& rel,
                                 const PXR_NS::TfToken& apiName, UsdWalkCtx& impl);

// Vehicle root emit. Walker pre-resolves xform scale +
// suspensionLineQueryType token + referenceFrameIsCenterOfMass
// custom-metadata bool, then defers scalar/bool attribute reads
// to parse-lib parseVehicle.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-4
void emitVehicle(ScannedStage& out, WalkState& walk, parse::ParseContext& ctx,
                 parse::ObjectKey key, const PXR_NS::UsdPrim& prim,
                 float lengthScale)
{
    parse::VehicleInfo info;
    info.lengthScale = lengthScale;
    info.queryType   = 0;  // eRAYCAST default
    info.referenceFrameIsCenterOfMass = true;  // default when the metadata key is absent

    // Total xform scale, via the prim's local-to-world transform.
    if (prim.IsA<PXR_NS::UsdGeomXformable>())
    {
        const PXR_NS::GfMatrix4d mat = walk.xfCache.GetLocalToWorldTransform(prim);
        const PXR_NS::GfTransform transform(mat);
        const PXR_NS::GfVec3d sc = transform.GetScale();
        info.scale = { static_cast<float>(sc[0]), static_cast<float>(sc[1]), static_cast<float>(sc[2]) };
    }
    if (info.scale.x == 0.0f || info.scale.y == 0.0f || info.scale.z == 0.0f)
    {
        CARB_LOG_ERROR("Usd Physics: vehicle \"%s\" has a scale component that is zero.\n",
                       prim.GetName().GetText());
        return;
    }

    // suspensionLineQueryType: token "raycast" (default) or "sweep".
    PXR_NS::PhysxSchemaPhysxVehicleAPI vehicleAPI(prim);
    if (PXR_NS::UsdAttribute qtAttr = vehicleAPI.GetSuspensionLineQueryTypeAttr())
    {
        PXR_NS::TfToken tok;
        if (qtAttr.Get(&tok) && tok == PXR_NS::PhysxSchemaTokens.Get()->sweep)
        {
            info.queryType = 1;  // eSWEEP
        }
    }

    // referenceFrameIsCenterOfMass: custom metadata under
    // SdfFieldKeys->CustomData with the PhysxSchemaTokens->referenceFrameIsCenterOfMass key.
    {
        PXR_NS::VtValue v;
        if (prim.GetMetadataByDictKey(PXR_NS::SdfFieldKeys->CustomData,
                                      PXR_NS::PhysxSchemaTokens->referenceFrameIsCenterOfMass, &v))
        {
            info.referenceFrameIsCenterOfMass = v.Get<bool>();
        }
        // If missing, the default `true` set above stands.
    }

    if (parse::DescPtr<parse::VehicleDesc> desc = parse::parseVehicle(ctx, key, info))
    {
        out.vehicles.push_back(std::move(desc));
        out.vehiclePaths.push_back(key);  // parallel side-table for consumer pre-pop
    }
}

// SuspensionCompliance emit. Walker pre-reads four optional
// VtArrays. Each list is stored verbatim in the Info; parser
// validates per-list cap + jounce/range/monotonicity.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-4
void emitVehicleSuspensionCompliance(ScannedStage& out, parse::ParseContext& ctx,
                                     parse::ObjectKey key, const PXR_NS::UsdPrim& prim)
{
    PXR_NS::PhysxSchemaPhysxVehicleSuspensionComplianceAPI scAPI(prim);

    parse::SuspensionComplianceInfo info;

    auto readVec2Arr = [&](const PXR_NS::UsdAttribute& a, std::vector<carb::Float2>& out) {
        if (!a || !a.HasAuthoredValue()) return;
        PXR_NS::VtArray<PXR_NS::GfVec2f> arr; a.Get(&arr);
        out.reserve(arr.size());
        for (const auto& v : arr) out.push_back({ v[0], v[1] });
    };
    auto readVec4Arr = [&](const PXR_NS::UsdAttribute& a, std::vector<carb::Float4>& out) {
        if (!a || !a.HasAuthoredValue()) return;
        PXR_NS::VtArray<PXR_NS::GfVec4f> arr; a.Get(&arr);
        out.reserve(arr.size());
        for (const auto& v : arr) out.push_back({ v[0], v[1], v[2], v[3] });
    };

    readVec2Arr(scAPI.GetWheelToeAngleAttr(),    info.wheelToeAngles);
    readVec2Arr(scAPI.GetWheelCamberAngleAttr(), info.wheelCamberAngles);
    readVec4Arr(scAPI.GetSuspensionForceAppPointAttr(), info.suspensionForceAppPoints);
    readVec4Arr(scAPI.GetTireForceAppPointAttr(),       info.tireForceAppPoints);

    if (parse::DescPtr<parse::SuspensionComplianceDesc> desc =
            parse::parseSuspensionCompliance(ctx, key, info))
    {
        out.vehicleSuspensionCompliances.push_back(std::move(desc));
        out.vehicleSuspensionCompliancePaths.push_back(key);
    }
}

// WheelAttachment emit. Walker pre-resolves three rel-or-API
// cross-references (wheel / tire / suspension) plus reads ~10 scalar /
// Vec3 / Quat attrs. Descendant walk discovers the single direct
// child with CollisionAPI applied — at most one such child is legal;
// nested colliders are rejected here.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-4
void emitVehicleWheelAttachment(ScannedStage& out, UsdWalkCtx& impl,
                                parse::ParseContext& ctx, parse::ObjectKey key,
                                const PXR_NS::UsdPrim& prim)
{
    static const PXR_NS::TfToken kWheelAPI("PhysxVehicleWheelAPI");
    static const PXR_NS::TfToken kTireAPI("PhysxVehicleTireAPI");
    static const PXR_NS::TfToken kSuspensionAPI("PhysxVehicleSuspensionAPI");

    PXR_NS::PhysxSchemaPhysxVehicleWheelAttachmentAPI waAPI(prim);

    parse::WheelAttachmentInfo info;

    // Cross-refs (rel-or-API): the resolver returns an empty key when
    // both rel is empty AND API is not applied. Consumer will check
    // for empty paths and reject the wheel attachment if any required
    // ref is missing.
    info.wheelKey      = resolveRelOrApi(prim, waAPI.GetWheelRel(),      kWheelAPI,      impl);
    info.tireKey       = resolveRelOrApi(prim, waAPI.GetTireRel(),       kTireAPI,       impl);
    info.suspensionKey = resolveRelOrApi(prim, waAPI.GetSuspensionRel(), kSuspensionAPI, impl);

    // Owning vehicle: nearest ancestor prim with PhysxVehicleAPI applied. Lets
    // the consumer group wheel attachments by vehicle without re-walking USD.
    for (PXR_NS::UsdPrim p = prim.GetParent(); p && !p.IsPseudoRoot(); p = p.GetParent())
    {
        if (p.HasAPI<PXR_NS::PhysxSchemaPhysxVehicleAPI>())
        {
            info.vehicleKey = impl.source.keyFor(p.GetPath());
            break;
        }
    }
    // Record the owner for EVERY attachment prim, before validation, so the
    // consumer still sees (and rejects the vehicle on) malformed attachments
    // that are dropped from vehicleWheelAttachments below.
    out.vehicleWheelAttachmentOwners.push_back({ info.vehicleKey, key });

    // suspensionTravelDirection (required)
    if (PXR_NS::UsdAttribute a = waAPI.GetSuspensionTravelDirectionAttr())
    {
        PXR_NS::GfVec3f v; if (a.Get(&v)) info.suspensionTravelDirection = { v[0], v[1], v[2] };
    }
    // suspensionForceAppPointOffset (deprecated, sets eHAS_SUSP_FORCE_APP_POINT)
    if (PXR_NS::UsdAttribute a = waAPI.GetSuspensionForceAppPointOffsetAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::GfVec3f v; a.Get(&v);
            info.suspensionForceAppPointOffset = { v[0], v[1], v[2] };
            info.state |= parse::WheelAttachmentDesc::eHAS_SUSP_FORCE_APP_POINT;
        }
    }
    // suspensionFramePosition (preferred) OR wheelCenterOfMassOffset (deprecated)
    if (PXR_NS::UsdAttribute a = waAPI.GetSuspensionFramePositionAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::GfVec3f v; a.Get(&v);
            info.suspensionFramePosition = { v[0], v[1], v[2] };
            info.state |= parse::WheelAttachmentDesc::eHAS_SUSPENSION_FRAME;
        }
    }
    if (!(info.state & parse::WheelAttachmentDesc::eHAS_SUSPENSION_FRAME))
    {
        if (PXR_NS::UsdAttribute a = waAPI.GetWheelCenterOfMassOffsetAttr())
        {
            if (a.HasAuthoredValue())
            {
                PXR_NS::GfVec3f v; a.Get(&v);
                info.wheelCenterOfMassOffset = { v[0], v[1], v[2] };
                info.state |= parse::WheelAttachmentDesc::eHAS_WHEEL_COM_OFFSET;
            }
        }
    }
    // suspensionFrameOrientation
    if (PXR_NS::UsdAttribute a = waAPI.GetSuspensionFrameOrientationAttr())
    {
        PXR_NS::GfQuatf q; if (a.Get(&q))
        {
            const PXR_NS::GfVec3f& im = q.GetImaginary();
            info.suspensionFrameOrientation = { im[0], im[1], im[2], q.GetReal() };
        }
    }
    // tireForceAppPointOffset (deprecated)
    if (PXR_NS::UsdAttribute a = waAPI.GetTireForceAppPointOffsetAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::GfVec3f v; a.Get(&v);
            info.tireForceAppPointOffset = { v[0], v[1], v[2] };
            info.state |= parse::WheelAttachmentDesc::eHAS_TIRE_FORCE_APP_POINT;
        }
    }
    // wheelFramePosition (required)
    if (PXR_NS::UsdAttribute a = waAPI.GetWheelFramePositionAttr())
    {
        PXR_NS::GfVec3f v; if (a.Get(&v)) info.wheelFramePosition = { v[0], v[1], v[2] };
    }
    // wheelFrameOrientation (required)
    if (PXR_NS::UsdAttribute a = waAPI.GetWheelFrameOrientationAttr())
    {
        PXR_NS::GfQuatf q; if (a.Get(&q))
        {
            const PXR_NS::GfVec3f& im = q.GetImaginary();
            info.wheelFrameOrientation = { im[0], im[1], im[2], q.GetReal() };
        }
    }
    // driven (deprecated)
    if (PXR_NS::UsdAttribute a = waAPI.GetDrivenAttr())
    {
        if (a.HasAuthoredValue()) a.Get(&info.driven);
    }
    // index (required)
    if (PXR_NS::UsdAttribute a = waAPI.GetIndexAttr())
    {
        int v = 0;
        if (a.Get(&v)) info.index = v;
    }

    // collisionGroup rel (single target; must point to UsdPhysicsCollisionGroup)
    if (PXR_NS::UsdRelationship rel = waAPI.GetCollisionGroupRel())
    {
        if (rel.HasAuthoredTargets())
        {
            PXR_NS::SdfPathVector paths;
            rel.GetTargets(&paths);
            if (paths.size() == 1)
            {
                const PXR_NS::UsdPrim cgPrim = prim.GetStage()->GetPrimAtPath(paths[0]);
                if (cgPrim && cgPrim.IsA<PXR_NS::UsdPhysicsCollisionGroup>())
                    info.collisionGroupKey = impl.source.keyFor(paths[0]);
                // Otherwise: leave path empty. Consumer adapter skips
                // pre-population on empty required refs.
            }
        }
    }

    // Xformable check + descendant walk for collision shape.
    if (prim.IsA<PXR_NS::UsdGeomXformable>())
    {
        info.state |= parse::WheelAttachmentDesc::eMANAGE_TRANSFORMS;

        bool foundShape = false;
        if (prim.HasAPI<PXR_NS::UsdPhysicsCollisionAPI>())
        {
            info.shapeKey = key;
            info.state |= parse::WheelAttachmentDesc::eHAS_SHAPE;
            foundShape = true;
        }
        // Walk descendants; only direct children with CollisionAPI are
        // legal. Nested colliders are an error and reject pre-population.
        bool sawNestedCollider = false;
        for (PXR_NS::UsdPrim sub : prim.GetDescendants())
        {
            if (!sub.HasAPI<PXR_NS::UsdPhysicsCollisionAPI>()) continue;
            if (sub.GetParent() != prim)
            {
                sawNestedCollider = true;
                break;
            }
            if (foundShape)
            {
                sawNestedCollider = true;  // second direct child collider
                break;
            }
            info.shapeKey = impl.source.keyFor(sub.GetPath());
            info.state |= parse::WheelAttachmentDesc::eHAS_SHAPE;
            foundShape = true;
        }
        if (sawNestedCollider)
        {
            // Nested / multiple colliders are illegal — log (the consumer marks
            // the vehicle invalid via the owners list) and skip emit.
            CARB_LOG_ERROR("Usd Physics: wheel attachment \"%s\": only the wheel attachment prim or a single "
                           "direct child may have CollisionAPI applied.",
                           prim.GetName().GetText());
            return;
        }
    }

    if (parse::DescPtr<parse::WheelAttachmentDesc> desc =
            parse::parseWheelAttachment(ctx, key, info))
    {
        out.vehicleWheelAttachments.push_back(std::move(desc));
        out.vehicleWheelAttachmentInfos.push_back(info);
    }
}

// DriveStandard emit. Walker pre-resolves the four rel-or-API
// cross-references (engine / gears / autoGearBox / clutch) to
// `ObjectKey`s: prefer the rel target; on empty rel fall back to the
// API applied on the same prim. Consumer adapter resolves the
// `ObjectKey`s to engine-side descriptor pointers via the tracker
// maps populated by the upstream engine/gears/clutch + drive emits.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-4
parse::ObjectKey resolveRelOrApi(const PXR_NS::UsdPrim& prim, const PXR_NS::UsdRelationship& rel,
                                 const PXR_NS::TfToken& apiName, UsdWalkCtx& impl)
{
    if (rel)
    {
        PXR_NS::SdfPathVector paths;
        rel.GetTargets(&paths);
        if (paths.size() == 1)
            return impl.source.keyFor(paths[0]);
        if (paths.size() > 1)
        {
            CARB_LOG_ERROR("Usd Physics: \"%s\" must not have more than 1 relationship target.",
                           prim.GetName().GetText());
            return {};
        }
        // empty paths — fall through to API-applied check
    }
    if (prim.HasAPI(apiName))
        return impl.source.keyFor(prim.GetPath());
    return {};
}

void emitVehicleDriveStandard(ScannedStage& out, UsdWalkCtx& impl,
                              parse::ParseContext& ctx, parse::ObjectKey key,
                              const PXR_NS::UsdPrim& prim)
{
    static const PXR_NS::TfToken kEngineAPI("PhysxVehicleEngineAPI");
    static const PXR_NS::TfToken kGearsAPI("PhysxVehicleGearsAPI");
    static const PXR_NS::TfToken kAutoGearBoxAPI("PhysxVehicleAutoGearBoxAPI");
    static const PXR_NS::TfToken kClutchAPI("PhysxVehicleClutchAPI");

    PXR_NS::PhysxSchemaPhysxVehicleDriveStandardAPI driveAPI(prim);

    parse::DriveStandardInfo info;
    info.engineKey      = resolveRelOrApi(prim, driveAPI.GetEngineRel(),      kEngineAPI,      impl);
    info.gearsKey       = resolveRelOrApi(prim, driveAPI.GetGearsRel(),       kGearsAPI,       impl);
    info.autoGearBoxKey = resolveRelOrApi(prim, driveAPI.GetAutoGearBoxRel(), kAutoGearBoxAPI, impl);
    info.clutchKey      = resolveRelOrApi(prim, driveAPI.GetClutchRel(),      kClutchAPI,      impl);

    if (parse::DescPtr<parse::DriveStandardDesc> desc =
            parse::parseDriveStandard(ctx, key, info))
    {
        out.vehicleDrivesStandard.push_back(std::move(desc));
        out.vehicleDrivesStandardPaths.push_back(key);
        out.vehicleDrivesStandardCrossRefs.push_back(info);
    }
}

// DriveBasic emit. Single-apply.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-4
void emitVehicleDriveBasic(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key, float kgmsScale)
{
    if (parse::DescPtr<parse::DriveBasicDesc> desc = parse::parseDriveBasic(ctx, key, kgmsScale))
        out.vehicleDrivesBasic.push_back(std::move(desc));
}

// Helper: walker pre-read for the four differential array fields the
// parser needs from the typed schema gateways. All four are int/float
// VtArrays that IPhysicsSource can't expose as scalars.
void readDifferentialInfo(const PXR_NS::UsdPrim& prim, parse::DifferentialInfo& info)
{
    PXR_NS::PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI mwAPI(prim);
    if (PXR_NS::UsdAttribute wAttr = mwAPI.GetWheelsAttr())
    {
        if (wAttr.HasAuthoredValue())
        {
            PXR_NS::VtArray<int> arr; wAttr.Get(&arr);
            info.wheels.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute trAttr = mwAPI.GetTorqueRatiosAttr())
    {
        if (trAttr.HasAuthoredValue())
        {
            PXR_NS::VtArray<float> arr; trAttr.Get(&arr);
            info.hasTorqueRatios = true;
            info.torqueRatios.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute avAttr = mwAPI.GetAverageWheelSpeedRatiosAttr())
    {
        if (avAttr.HasAuthoredValue())
        {
            PXR_NS::VtArray<float> arr; avAttr.Get(&arr);
            info.hasAverageWheelSpeedRatios = true;
            info.averageWheelSpeedRatios.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
}

// MultiWheelDifferential emit. Tank-shaped prims are
// dispatched separately via emitVehicleTankDifferential — Tank
// subclass-API check takes precedence in the walker dispatch.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-4
void emitVehicleMultiWheelDifferential(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key,
                                       const PXR_NS::UsdPrim& prim)
{
    parse::DifferentialInfo info;
    readDifferentialInfo(prim, info);
    if (parse::DescPtr<parse::MultiWheelDifferentialDesc> desc = parse::parseMultiWheelDifferential(ctx, key, info))
    {
        out.vehicleMultiWheelDifferentials.push_back(std::move(desc));
        out.vehicleMultiWheelDifferentialPaths.push_back(key);
    }
}

// TankDifferential emit. Tank API is applied alongside
// MultiWheel; the schema requires both. Tank-specific arrays are
// read via the Tank gateway here; MultiWheel arrays come from the
// shared readDifferentialInfo helper.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-4
void emitVehicleTankDifferential(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key,
                                 const PXR_NS::UsdPrim& prim)
{
    parse::DifferentialInfo info;
    readDifferentialInfo(prim, info);

    parse::TankDifferentialInfo tankInfo;
    PXR_NS::PhysxSchemaPhysxVehicleTankDifferentialAPI tkAPI(prim);
    if (PXR_NS::UsdAttribute a = tkAPI.GetNumberOfWheelsPerTrackAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<int> arr; a.Get(&arr);
            tankInfo.hasNumberOfWheelsPerTrack = true;
            tankInfo.numberOfWheelsPerTrack.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute a = tkAPI.GetThrustIndexPerTrackAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<int> arr; a.Get(&arr);
            tankInfo.hasThrustIndexPerTrack = true;
            tankInfo.thrustIndexPerTrack.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute a = tkAPI.GetWheelIndicesInTrackOrderAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<int> arr; a.Get(&arr);
            tankInfo.hasWheelIndicesInTrackOrder = true;
            tankInfo.wheelIndicesInTrackOrder.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute a = tkAPI.GetTrackToWheelIndicesAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<int> arr; a.Get(&arr);
            tankInfo.hasTrackToWheelIndices = true;
            tankInfo.trackToWheelIndices.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }

    if (parse::DescPtr<parse::TankDifferentialDesc> desc =
            parse::parseTankDifferential(ctx, key, info, tankInfo))
    {
        out.vehicleTankDifferentials.push_back(std::move(desc));
        out.vehicleTankDifferentialPaths.push_back(key);
    }
}

// AutoGearBox emit. Walker pre-reads upRatios + downRatios
// (VtArray<float>).
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-4
// Helper: walker pre-read for the two per-instance Brakes arrays.
// `instance` is the multi-apply instance token (e.g. "brakes0").
void readBrakesInfo(const PXR_NS::UsdPrim& prim, const std::string& instance,
                    parse::BrakesInfo& info)
{
    const PXR_NS::TfToken instTok(instance);
    PXR_NS::PhysxSchemaPhysxVehicleBrakesAPI brakesAPI =
        PXR_NS::PhysxSchemaPhysxVehicleBrakesAPI::Get(prim, instTok);
    if (!brakesAPI) return;

    if (PXR_NS::UsdAttribute a = brakesAPI.GetWheelsAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<int> arr; a.Get(&arr);
            info.wheels.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute a = brakesAPI.GetTorqueMultipliersAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<float> arr; a.Get(&arr);
            info.hasTorqueMultipliers = true;
            info.torqueMultipliers.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
}

// Brakes emit. Walker drives the multi-apply iteration via
// `forEachMultiApplyInstance` so the parse-lib emits one BrakesDesc
// per applied instance. Legacy only honors "brakes0" and "brakes1"
// (mapped to brakesIndex=0/1) — walker filters to match.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-4
void emitVehicleBrakes(ScannedStage& out, UsdWalkCtx& impl, parse::ParseContext& ctx,
                       parse::ObjectKey key, const PXR_NS::UsdPrim& prim)
{
    impl.source.forEachMultiApplyInstance(key, "PhysxVehicleBrakesAPI",
        [&](std::string_view instance)
        {
            uint8_t brakesIndex = 0;
            if (instance == "brakes0")      brakesIndex = 0;
            else if (instance == "brakes1") brakesIndex = 1;
            else                            return;  // only brakes0 / brakes1 are honored

            parse::BrakesInfo info;
            readBrakesInfo(prim, std::string(instance), info);

            if (parse::DescPtr<parse::BrakesDesc> desc =
                    parse::parseBrakes(ctx, key, instance, brakesIndex, info))
            {
                const parse::TokenId tok = impl.source.internToken(instance);
                out.vehicleBrakes.push_back(std::move(desc));
                out.vehicleBrakesPaths.push_back(key);
                out.vehicleBrakesInstanceTokens.push_back(tok);
            }
        });
}

// Steering emit. Walker pre-reads wheels (VtArray<int>) +
// angleMultipliers (VtArray<float>).
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-4
void emitVehicleSteeringBasic(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key,
                              const PXR_NS::UsdPrim& prim)
{
    PXR_NS::PhysxSchemaPhysxVehicleSteeringAPI steeringAPI(prim);

    parse::SteeringBasicInfo info;
    if (PXR_NS::UsdAttribute a = steeringAPI.GetWheelsAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<int> arr; a.Get(&arr);
            info.wheels.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute a = steeringAPI.GetAngleMultipliersAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<float> arr; a.Get(&arr);
            info.hasAngleMultipliers = true;
            info.angleMultipliers.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }

    if (parse::DescPtr<parse::SteeringBasicDesc> desc =
            parse::parseSteeringBasic(ctx, key, info))
    {
        out.vehicleSteeringBasic.push_back(std::move(desc));
        out.vehicleSteeringBasicPaths.push_back(key);
    }
}

// Ackermann steering emit. All-scalar reads.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-4
void emitVehicleSteeringAckermann(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key)
{
    if (parse::DescPtr<parse::SteeringAckermannDesc> desc =
            parse::parseSteeringAckermann(ctx, key))
    {
        out.vehicleSteeringAckermann.push_back(std::move(desc));
        out.vehicleSteeringAckermannPaths.push_back(key);
    }
}

// NonlinearCmdResponse emit. Multi-apply per command instance.
// Walker drives `forEachMultiApplyInstance` and pre-reads three
// VtArrays per instance. The instance token (e.g. "drive" / "steer" /
// "brakes0") goes into ScannedStage so the consumer adapter can wire
// the pointer onto the matching Drive / Steering / Brakes desc.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-4
void emitVehicleNonlinearCmdResponse(ScannedStage& out, UsdWalkCtx& impl,
                                     parse::ParseContext& ctx, parse::ObjectKey key,
                                     const PXR_NS::UsdPrim& prim)
{
    impl.source.forEachMultiApplyInstance(key, "PhysxVehicleNonlinearCommandResponseAPI",
        [&](std::string_view instance)
        {
            const PXR_NS::TfToken instTok{ std::string(instance) };
            PXR_NS::PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI ncrAPI =
                PXR_NS::PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI::Get(prim, instTok);
            if (!ncrAPI) return;

            parse::NonlinearCmdResponseInfo info;

            if (PXR_NS::UsdAttribute a = ncrAPI.GetCommandValuesAttr())
            {
                if (a.HasAuthoredValue())
                {
                    PXR_NS::VtArray<float> arr; a.Get(&arr);
                    info.hasCommandValues = true;
                    info.commandValues.assign(arr.cdata(), arr.cdata() + arr.size());
                }
            }
            if (PXR_NS::UsdAttribute a = ncrAPI.GetSpeedResponsesPerCommandValueAttr())
            {
                if (a.HasAuthoredValue())
                {
                    PXR_NS::VtArray<int> arr; a.Get(&arr);
                    info.hasSpeedResponsesPerCommandValue = true;
                    info.speedResponsesPerCommandValue.assign(arr.cdata(), arr.cdata() + arr.size());
                }
            }
            if (PXR_NS::UsdAttribute a = ncrAPI.GetSpeedResponsesAttr())
            {
                if (a.HasAuthoredValue())
                {
                    PXR_NS::VtArray<PXR_NS::GfVec2f> arr; a.Get(&arr);
                    info.hasSpeedResponses = true;
                    info.speedResponses.reserve(arr.size());
                    for (const PXR_NS::GfVec2f& v : arr)
                        info.speedResponses.push_back({ v[0], v[1] });
                }
            }

            if (parse::DescPtr<parse::NonlinearCmdResponseDesc> desc =
                    parse::parseNonlinearCmdResponse(ctx, key, instance, info))
            {
                out.vehicleNonlinearCmdResponses.push_back(std::move(desc));
                out.vehicleNonlinearCmdResponsePaths.push_back(key);
                out.vehicleNonlinearCmdResponseInstanceTokens.push_back(
                    impl.source.internToken(instance));
            }
        });
}

void emitVehicleAutoGearBox(ScannedStage& out, parse::ParseContext& ctx, parse::ObjectKey key,
                            const PXR_NS::UsdPrim& prim)
{
    PXR_NS::PhysxSchemaPhysxVehicleAutoGearBoxAPI agAPI(prim);

    parse::AutoGearBoxInfo info;
    if (PXR_NS::UsdAttribute a = agAPI.GetUpRatiosAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<float> arr; a.Get(&arr);
            info.hasUpRatios = true;
            info.upRatios.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }
    if (PXR_NS::UsdAttribute a = agAPI.GetDownRatiosAttr())
    {
        if (a.HasAuthoredValue())
        {
            PXR_NS::VtArray<float> arr; a.Get(&arr);
            info.hasDownRatios = true;
            info.downRatios.assign(arr.cdata(), arr.cdata() + arr.size());
        }
    }

    if (parse::DescPtr<parse::AutoGearBoxDesc> desc = parse::parseAutoGearBox(ctx, key, info))
    {
        out.vehicleAutoGearBoxes.push_back(std::move(desc));
        out.vehicleAutoGearBoxPaths.push_back(key);
    }
}

// Character-controller emit. Requires the prim to be a UsdGeomCapsule,
// reads capsule radius/height + world transform (translation + scale
// via GfTransform), and the simulationOwner relationship from
// PhysxCharacterControllerAPI. Per-axis scale baking: radius scaled by
// sc[1], halfHeight by sc[2]. parse-lib's parseCct resolves slopeLimit
// + the scene ObjectId.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-CCT-001
// @covers AC-1 AC-2 AC-3
void emitCct(ScannedStage& out, UsdWalkCtx& impl, parse::ParseContext& ctx,
             parse::ObjectKey key, const PXR_NS::UsdPrim& prim)
{
    if (!prim.IsA<PXR_NS::UsdGeomCapsule>())
    {
        CARB_LOG_ERROR("CCT prim must be a capsule geom. (%s)", prim.GetTypeName().GetText());
        return;
    }

    PXR_NS::UsdGeomXformable xform(prim);
    const PXR_NS::GfTransform tr(xform.ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode()));
    const PXR_NS::GfVec3d sc    = tr.GetScale();
    const PXR_NS::GfVec3d trans = tr.GetTranslation();

    double radiusAttr = 1.0, heightAttr = 1.0;
    PXR_NS::UsdGeomCapsule cap(prim);
    cap.GetRadiusAttr().Get(&radiusAttr);
    cap.GetHeightAttr().Get(&heightAttr);

    parse::CctInfo info;
    info.radius     = float(sc[1]) * float(radiusAttr);
    info.halfHeight = float(sc[2]) * float(heightAttr) * 0.5f;
    info.scale      = { float(sc[0]), float(sc[1]), float(sc[2]) };
    info.pos        = { float(trans[0]), float(trans[1]), float(trans[2]) };

    PXR_NS::PhysxSchemaPhysxCharacterControllerAPI usdCct(prim);
    if (PXR_NS::UsdRelationship rel = usdCct.GetSimulationOwnerRel())
    {
        PXR_NS::SdfPathVector owners;
        rel.GetTargets(&owners);
        info.simulationOwners.reserve(owners.size());
        for (const PXR_NS::SdfPath& p : owners)
            info.simulationOwners.push_back(impl.source.keyFor(p));
    }

    if (parse::DescPtr<parse::CapsuleCctDesc> desc = parse::parseCct(ctx, key, info))
        out.ccts.push_back(std::move(desc));
}

// ---------------------------------------------------------------------------
// Rigid body — emit. Reads transform via UsdGeomXformCache and
// pulls source-side cross-references (simulationOwners,
// filteredCollisions) from the body's own rels; `sourceCollisions`
// is populated in pass-2 by the ancestor walk over the shape list.
// ---------------------------------------------------------------------------

// Decompose the body's world transform into position / rotation /
// scale. Logs a warning when a non-uniform scale is combined with
// a non-identity scale orientation — PhysX has no notion of scale
// orientation so the result will look subtly wrong.
void readBodyTransform(PXR_NS::UsdGeomXformCache& xfCache,
                       const PXR_NS::UsdPrim& bodyPrim,
                       carb::Float3& outPos,
                       carb::Float4& outRot,
                       carb::Float3& outScale)
{
    const PXR_NS::GfMatrix4d mat = xfCache.GetLocalToWorldTransform(bodyPrim);
    const PXR_NS::GfTransform tr(mat);
    const PXR_NS::GfVec3d pos = tr.GetTranslation();
    const PXR_NS::GfQuatd rot = tr.GetRotation().GetQuat();
    const PXR_NS::GfVec3d sc  = tr.GetScale();

    if (!scaleIsUniform(sc[0], sc[1], sc[2]) &&
        tr.GetScaleOrientation().GetQuaternion() != PXR_NS::GfQuaternion::GetIdentity())
    {
        CARB_LOG_WARN("ScaleOrientation is not supported for rigid bodies, prim path: %s. "
                      "You may ignore this if the scale is close to uniform.",
                      bodyPrim.GetPrimPath().GetText());
    }

    outPos   = toFloat3(PXR_NS::GfVec3f(pos));
    outRot   = toFloat4(PXR_NS::GfQuatf(rot));
    outScale = toFloat3(PXR_NS::GfVec3f(sc));
}

void emitRigidBody(ScannedStage& out, UsdWalkCtx& impl,
                   parse::ParseContext& ctx, WalkState& walk,
                   parse::ObjectKey key, const PXR_NS::UsdPrim& bodyPrim,
                   uint64_t typeBits)
{
    if (!(typeBits & PrimTypeBits::eUsdGeomXformable))
    {
        CARB_LOG_ERROR("RigidBodyAPI applied to a non-xformable primitive. (%s)",
                       bodyPrim.GetPrimPath().GetText());
        return;
    }

    const PXR_NS::UsdPhysicsRigidBodyAPI rbAPI(bodyPrim);
    bool rigidBodyEnabled = true;
    rbAPI.GetRigidBodyEnabledAttr().Get(&rigidBodyEnabled);

    parse::DescPtr<parse::PhysxRigidBodyDesc> base;
    if (rigidBodyEnabled)
    {
        parse::DescPtr<parse::DynamicPhysxRigidBodyDesc> dyn =
            parse::allocateDesc<parse::DynamicPhysxRigidBodyDesc>(ctx.descriptorAllocator());
        parse::setToDefault(*dyn, impl.source.getSourceUnits());
        // parseDynamicBody handles every UsdPhysicsRigidBodyAPI field
        // (kinematic, startsAsleep, velocities incl. deg→rad and
        // localSpaceVelocities transform) plus PhysxRigidBodyAPI /
        // SurfaceVelocityAPI / SplinesSurfaceVelocityAPI extensions.
        if (parse::DescPtr<parse::DynamicPhysxRigidBodyDesc> parsed = parse::parseDynamicBody(ctx, key))
            *dyn = *parsed;
        base = parse::descPtrCast<parse::PhysxRigidBodyDesc>(std::move(dyn));
    }
    else
    {
        parse::DescPtr<parse::StaticPhysxRigidBodyDesc> stat =
            parse::allocateDesc<parse::StaticPhysxRigidBodyDesc>(ctx.descriptorAllocator());
        stat->sourceGPrimKey = key;
        base = parse::descPtrCast<parse::PhysxRigidBodyDesc>(std::move(stat));
    }

    readBodyTransform(walk.xfCache, bodyPrim, base->position, base->rotation, base->scale);
    base->primKey = key;

    // Source-side simulationOwners + filteredCollisions — parseDynamicBody
    // doesn't snapshot these, so they're read off the body's own rels here.
    const PXR_NS::UsdRelationship ownerRel = rbAPI.GetSimulationOwnerRel();
    if (ownerRel)
    {
        PXR_NS::SdfPathVector owners;
        ownerRel.GetTargets(&owners);
        base->sourceSimulationOwners.reserve(owners.size());
        for (const PXR_NS::SdfPath& p : owners)
            base->sourceSimulationOwners.push_back(impl.source.keyFor(p));
    }
    {
        std::vector<parse::ObjectKey> pairs = parse::parseFilteredPairs(ctx, key);
        base->sourceFilteredCollisions = std::move(pairs);
    }

    // Record into the walk-state map BEFORE move so pass-2 has the raw
    // pointer.
    WalkState::BodyEntry entry;
    entry.desc = base.get();
    entry.rigidBodyEnabled = rigidBodyEnabled;
    walk.bodyMap[key] = entry;

    out.bodies.push_back(std::move(base));
}

// ---------------------------------------------------------------------------
// Shape — per-prim geometry readers. Each helper reads the per-shape
// USD attributes (radius / size / height / axis), applies the world-
// space scale, and returns a typed parse-lib shape descriptor. The
// common shape fill (collisionEnabled, materials, simulationOwners,
// filteredPairs, PhysxCollisionAPI overlay) is done by emitShape's tail.
// ---------------------------------------------------------------------------

parse::Axis usdCapAxisToParse(const PXR_NS::TfToken& tok)
{
    if (tok == PXR_NS::UsdPhysicsTokens.Get()->y) return parse::eY;
    if (tok == PXR_NS::UsdPhysicsTokens.Get()->z) return parse::eZ;
    return parse::eX;
}

// Sphere: radius scales by the largest absolute axis; non-uniform
// scale logs a warning since PhysX has no anisotropic sphere.
parse::DescPtr<parse::PhysxShapeDesc> readSphereDesc(
    parse::ParseContext& ctx, const PXR_NS::UsdPrim& prim, const PXR_NS::GfVec3d& worldScale)
{
    PXR_NS::UsdGeomSphere sphere(prim);
    double radiusAttr = 1.0;
    sphere.GetRadiusAttr().Get(&radiusAttr);

    checkNonUniformScale(worldScale, prim.GetPrimPath());
    const float scaleMax = std::fmax(std::fmax(std::fabs(float(worldScale[0])), std::fabs(float(worldScale[1]))),
                                     std::fabs(float(worldScale[2])));
    parse::DescPtr<parse::SpherePhysxShapeDesc> desc = parse::allocateDesc<parse::SpherePhysxShapeDesc>(ctx.descriptorAllocator(), std::fabs(float(radiusAttr) * scaleMax));
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(desc));
}

// Cube: scale becomes part of the cube size (physics doesn't support
// scale separately) — halfExtents = abs(size) * 0.5 * scale.
parse::DescPtr<parse::PhysxShapeDesc> readCubeDesc(
    parse::ParseContext& ctx, const PXR_NS::UsdPrim& prim, const PXR_NS::GfVec3d& worldScale)
{
    PXR_NS::UsdGeomCube cube(prim);
    double sizeAttr = 1.0;
    cube.GetSizeAttr().Get(&sizeAttr);
    const float half = std::fabs(float(sizeAttr)) * 0.5f;

    PXR_NS::GfVec3f halfExtents(float(worldScale[0]) * half,
                                float(worldScale[1]) * half,
                                float(worldScale[2]) * half);
    parse::DescPtr<parse::BoxPhysxShapeDesc> desc = parse::allocateDesc<parse::BoxPhysxShapeDesc>(ctx.descriptorAllocator(), toFloat3(halfExtents));
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(desc));
}

// Capsule: radius + height + axis (token X/Y/Z). Per-axis scaling:
// halfHeight scales by the axis component, radius by max of the two
// other components.
parse::DescPtr<parse::PhysxShapeDesc> readCapsuleDesc(
    parse::ParseContext& ctx, const PXR_NS::UsdPrim& prim, const PXR_NS::GfVec3d& worldScale)
{
    PXR_NS::UsdGeomCapsule cap(prim);
    double radiusAttr = 0.0, heightAttr = 0.0;
    cap.GetRadiusAttr().Get(&radiusAttr);
    cap.GetHeightAttr().Get(&heightAttr);
    PXR_NS::TfToken axisTok;
    if (cap.GetAxisAttr())
        cap.GetAxisAttr().Get(&axisTok);
    const parse::Axis axis = usdCapAxisToParse(axisTok);

    checkNonUniformScale(worldScale, prim.GetPrimPath());
    float radius = float(radiusAttr);
    float halfHeight = float(heightAttr) * 0.5f;
    if (axis == parse::eX)
    {
        halfHeight *= float(worldScale[0]);
        radius *= std::fmax(std::fabs(float(worldScale[1])), std::fabs(float(worldScale[2])));
    }
    else if (axis == parse::eY)
    {
        halfHeight *= float(worldScale[1]);
        radius *= std::fmax(std::fabs(float(worldScale[0])), std::fabs(float(worldScale[2])));
    }
    else
    {
        halfHeight *= float(worldScale[2]);
        radius *= std::fmax(std::fabs(float(worldScale[1])), std::fabs(float(worldScale[0])));
    }
    parse::DescPtr<parse::CapsulePhysxShapeDesc> desc = parse::allocateDesc<parse::CapsulePhysxShapeDesc>(ctx.descriptorAllocator(), std::fabs(radius), std::fabs(halfHeight), axis);
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(desc));
}

// Cylinder: like Capsule, but warns only on per-cross-section
// non-uniform scale (the long-axis component is honoured).
parse::DescPtr<parse::PhysxShapeDesc> readCylinderDesc(
    parse::ParseContext& ctx, const PXR_NS::UsdPrim& prim, const PXR_NS::GfVec3d& worldScale)
{
    PXR_NS::UsdGeomCylinder cyl(prim);
    double radiusAttr = 0.0, heightAttr = 0.0;
    cyl.GetRadiusAttr().Get(&radiusAttr);
    cyl.GetHeightAttr().Get(&heightAttr);
    PXR_NS::TfToken axisTok;
    if (cyl.GetAxisAttr())
        cyl.GetAxisAttr().Get(&axisTok);
    const parse::Axis axis = usdCapAxisToParse(axisTok);

    constexpr double tol = 1e-4;
    float radius = float(radiusAttr);
    float halfHeight = float(heightAttr) * 0.5f;
    if (axis == parse::eX)
    {
        halfHeight *= float(worldScale[0]);
        if (std::abs(worldScale[2] - worldScale[1]) > tol)
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s",
                          prim.GetPrimPath().GetText());
        radius *= std::fmax(std::fabs(float(worldScale[1])), std::fabs(float(worldScale[2])));
    }
    else if (axis == parse::eY)
    {
        halfHeight *= float(worldScale[1]);
        if (std::abs(worldScale[2] - worldScale[0]) > tol)
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s",
                          prim.GetPrimPath().GetText());
        radius *= std::fmax(std::fabs(float(worldScale[0])), std::fabs(float(worldScale[2])));
    }
    else
    {
        halfHeight *= float(worldScale[2]);
        if (std::abs(worldScale[0] - worldScale[1]) > tol)
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s",
                          prim.GetPrimPath().GetText());
        radius *= std::fmax(std::fabs(float(worldScale[1])), std::fabs(float(worldScale[0])));
    }
    parse::DescPtr<parse::CylinderPhysxShapeDesc> desc = parse::allocateDesc<parse::CylinderPhysxShapeDesc>(
        ctx.descriptorAllocator(), std::fabs(radius), std::fabs(halfHeight), axis);
    // PhysxConvexGeometry:margin overlay — applied when authored,
    // clamped to >= 0.
    {
        static const PXR_NS::TfToken kMarginTok("physxConvexGeometry:margin");
        const PXR_NS::UsdAttribute attr = prim.GetAttribute(kMarginTok);
        if (attr && attr.HasAuthoredValue())
        {
            float v = 0.0f;
            if (attr.Get(&v))
                desc->margin = std::fmax(0.0f, v);
        }
    }
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(desc));
}

// Cone: same shape as Capsule scaling.
parse::DescPtr<parse::PhysxShapeDesc> readConeDesc(
    parse::ParseContext& ctx, const PXR_NS::UsdPrim& prim, const PXR_NS::GfVec3d& worldScale)
{
    PXR_NS::UsdGeomCone cone(prim);
    double radiusAttr = 0.0, heightAttr = 0.0;
    cone.GetRadiusAttr().Get(&radiusAttr);
    cone.GetHeightAttr().Get(&heightAttr);
    PXR_NS::TfToken axisTok;
    if (cone.GetAxisAttr())
        cone.GetAxisAttr().Get(&axisTok);
    const parse::Axis axis = usdCapAxisToParse(axisTok);

    checkNonUniformScale(worldScale, prim.GetPrimPath());
    float radius = float(radiusAttr);
    float halfHeight = float(heightAttr) * 0.5f;
    if (axis == parse::eX)
    {
        halfHeight *= float(worldScale[0]);
        radius *= std::fmax(std::fabs(float(worldScale[1])), std::fabs(float(worldScale[2])));
    }
    else if (axis == parse::eY)
    {
        halfHeight *= float(worldScale[1]);
        radius *= std::fmax(std::fabs(float(worldScale[0])), std::fabs(float(worldScale[2])));
    }
    else
    {
        halfHeight *= float(worldScale[2]);
        radius *= std::fmax(std::fabs(float(worldScale[1])), std::fabs(float(worldScale[0])));
    }
    parse::DescPtr<parse::ConePhysxShapeDesc> desc = parse::allocateDesc<parse::ConePhysxShapeDesc>(
        ctx.descriptorAllocator(), std::fabs(radius), std::fabs(halfHeight), axis);
    // PhysxConvexGeometry:margin overlay — applied when authored,
    // clamped to >= 0.
    {
        static const PXR_NS::TfToken kMarginTok("physxConvexGeometry:margin");
        const PXR_NS::UsdAttribute attr = prim.GetAttribute(kMarginTok);
        if (attr && attr.HasAuthoredValue())
        {
            float v = 0.0f;
            if (attr.Get(&v))
                desc->margin = std::fmax(0.0f, v);
        }
    }
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(desc));
}

// Plane: read the axis token (X/Y/Z), allocate a PlanePhysxShapeDesc.
parse::DescPtr<parse::PhysxShapeDesc> readPlaneDesc(parse::ParseContext& ctx, const PXR_NS::UsdPrim& prim)
{
    PXR_NS::UsdGeomPlane plane(prim);
    PXR_NS::TfToken axisTok;
    plane.GetAxisAttr().Get(&axisTok);
    parse::DescPtr<parse::PlanePhysxShapeDesc> desc = parse::allocateDesc<parse::PlanePhysxShapeDesc>(ctx.descriptorAllocator());
    desc->axis = usdCapAxisToParse(axisTok);
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(desc));
}

// Sphere-points: width/positions on UsdGeomPoints. Scale picked as
// max axis. Returns nullptr on malformed input (empty widths or a
// widths/positions size mismatch); the call site logs an error.
parse::DescPtr<parse::PhysxShapeDesc> readSpherePointsDesc(
    parse::ParseContext& ctx, const PXR_NS::UsdPrim& prim, const PXR_NS::GfVec3d& worldScale)
{
    PXR_NS::UsdGeomPoints points(prim);
    PXR_NS::VtArray<float> widths;
    PXR_NS::VtArray<PXR_NS::GfVec3f> positions;
    points.GetWidthsAttr().Get(&widths);
    if (widths.empty())
    {
        CARB_LOG_ERROR("Provided points geom with a PhysicsCollisionAPI does not have widths, collision will not be created. Prim: %s",
                       prim.GetPrimPath().GetText());
        return {};
    }
    points.GetPointsAttr().Get(&positions);
    if (positions.size() != widths.size())
    {
        CARB_LOG_ERROR("Provided points geom with a PhysicsCollisionAPI does not have the same number of positions and widths, collision will not be created. Prim: %s",
                       prim.GetPrimPath().GetText());
        return {};
    }

    checkNonUniformScale(worldScale, prim.GetPrimPath());
    const float sphereScale = std::fmax(std::fmax(std::fabs(float(worldScale[1])), std::fabs(float(worldScale[0]))),
                                        std::fabs(float(worldScale[2])));

    parse::DescPtr<parse::SpherePointsPhysxShapeDesc> sd = parse::allocateDesc<parse::SpherePointsPhysxShapeDesc>(ctx.descriptorAllocator());
    sd->spheres.resize(positions.size());
    for (size_t i = 0; i < positions.size(); ++i)
    {
        sd->spheres[i].radius   = sphereScale * widths[i] * 0.5f;
        sd->spheres[i].position = toFloat3(positions[i]);
    }
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(sd));
}

// ---------------------------------------------------------------------------
// Mesh-shape dispatch. Reads the mesh approximation + cooking knobs
// via parse-lib, allocates the typed descriptor, and applies the
// TriangleMesh→ConvexHull fallback for dynamic non-kinematic bodies.
// ---------------------------------------------------------------------------

parse::DescPtr<parse::PhysxShapeDesc> readMeshShapeDesc(
    parse::ParseContext& ctx, UsdWalkCtx& impl,
    parse::ObjectKey shapeKey, const PXR_NS::UsdPrim& shapePrim,
    const carb::Float3& meshScale, const carb::Float3& signScale,
    bool doubleSided, parse::ObjectKey rigidBodyKey,
    parse::ObjectKey gprimKey)
{
    parse::MeshApproximation approx = parse::parseMeshApproximation(ctx, shapeKey);

    // TriangleMesh / MeshSimplification on a dynamic non-kinematic
    // body → fall back to ConvexHull (PhysX rejects non-SDF triangle
    // meshes as eSIMULATION_SHAPE on non-kinematic PxRigidDynamic).
    if ((approx == parse::MeshApproximation::eNone ||
         approx == parse::MeshApproximation::eMeshSimplification) &&
        rigidBodyKey.valid())
    {
        const PXR_NS::SdfPath bodyPath = impl.source.pathFor(rigidBodyKey);
        if (!bodyPath.IsEmpty() && impl.stage)
        {
            PXR_NS::UsdPrim bodyPrim = impl.stage->GetPrimAtPath(bodyPath);
            if (bodyPrim)
            {
                PXR_NS::UsdPhysicsRigidBodyAPI rbAPI(bodyPrim);
                bool isKinematic = false, isEnabled = true;
                if (rbAPI)
                {
                    rbAPI.GetKinematicEnabledAttr().Get(&isKinematic);
                    rbAPI.GetRigidBodyEnabledAttr().Get(&isEnabled);
                }
                if (!isKinematic && isEnabled)
                    approx = parse::MeshApproximation::eConvexHull;
            }
        }
    }

    const parse::ObjectKey meshDataKey = gprimKey.valid() ? gprimKey : shapeKey;

    switch (approx)
    {
    case parse::MeshApproximation::eConvexHull:
    {
        parse::DescPtr<parse::ConvexMeshPhysxShapeDesc> cmd = parse::allocateDesc<parse::ConvexMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        cmd->meshScale = meshScale;
        cmd->meshPrimKey = meshDataKey;
        cmd->convexCookingParams.signScale = signScale;
        parse::parseConvexHullCookingExt(ctx, shapeKey, cmd->convexCookingParams);
        return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(cmd));
    }
    case parse::MeshApproximation::eMeshSimplification:
    {
        parse::DescPtr<parse::TriangleMeshPhysxShapeDesc> tmd = parse::allocateDesc<parse::TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = meshDataKey;
        tmd->doubleSided = doubleSided;
        tmd->triangleMeshCookingParams.mode =
            omni::physx::TriangleMeshMode::eQUADRIC_SIMPLIFICATION;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseTriangleMeshSimplificationCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(tmd));
    }
    case parse::MeshApproximation::eConvexDecomposition:
    {
        parse::DescPtr<parse::ConvexMeshDecompositionPhysxShapeDesc> cdd = parse::allocateDesc<parse::ConvexMeshDecompositionPhysxShapeDesc>(ctx.descriptorAllocator());
        cdd->meshScale = meshScale;
        cdd->meshPrimKey = meshDataKey;
        cdd->doubleSided = doubleSided;
        cdd->convexDecompositionCookingParams.signScale = signScale;
        cdd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseConvexDecompositionCookingExt(ctx, shapeKey, cdd->convexDecompositionCookingParams);
        return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(cdd));
    }
    case parse::MeshApproximation::eSphereFill:
    {
        parse::DescPtr<parse::SpherePointsPhysxShapeDesc> sfd = parse::allocateDesc<parse::SpherePointsPhysxShapeDesc>(ctx.descriptorAllocator());
        sfd->meshScale = meshScale;
        sfd->meshPrimKey = meshDataKey;
        sfd->doubleSided = doubleSided;
        sfd->sphereFillCookingParams.signScale = signScale;
        sfd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseSphereFillCookingExt(ctx, shapeKey, sfd->sphereFillCookingParams);
        return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(sfd));
    }
    case parse::MeshApproximation::eSdf:
    {
        parse::DescPtr<parse::TriangleMeshPhysxShapeDesc> tmd = parse::allocateDesc<parse::TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = meshDataKey;
        tmd->doubleSided = doubleSided;
        parse::parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        parse::parseSdfMeshCookingExt(ctx, shapeKey, tmd->sdfMeshCookingParams);
        return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(tmd));
    }
    case parse::MeshApproximation::eBoundingSphere:
    case parse::MeshApproximation::eBoundingCube:
    {
        // Read the gprim's points into a mergedMesh buffer; the consumer
        // computes the bounding-sphere radius / bounding-box halfExtents
        // from those points.
        parse::DescPtr<parse::MergeMeshDesc> mm = parse::allocateDesc<parse::MergeMeshDesc>(ctx.descriptorAllocator());
        const parse::MeshGeometry geom = parse::parseMeshGeometry(ctx, meshDataKey);
        const parse::BufferSpan<carb::Float3> pointsView = ctx.getBuffer<carb::Float3>(geom.points);
        mm->points.assign(pointsView.data, pointsView.data + pointsView.count);

        parse::DescPtr<parse::MergeMeshPhysxShapeDesc> typed;
        if (approx == parse::MeshApproximation::eBoundingSphere)
        {
            parse::DescPtr<parse::BoundingSpherePhysxShapeDesc> bs = parse::allocateDesc<parse::BoundingSpherePhysxShapeDesc>(ctx.descriptorAllocator());
            typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(bs));
        }
        else
        {
            parse::DescPtr<parse::BoundingBoxPhysxShapeDesc> bb = parse::allocateDesc<parse::BoundingBoxPhysxShapeDesc>(ctx.descriptorAllocator());
            typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(bb));
        }
        typed->mergedMesh = mm.get();
        impl.scan.retainOwnedMesh(std::move(mm));
        return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(typed));
    }
    case parse::MeshApproximation::eNone:
    default:
    {
        parse::DescPtr<parse::TriangleMeshPhysxShapeDesc> tmd = parse::allocateDesc<parse::TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = meshDataKey;
        tmd->doubleSided = doubleSided;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(tmd));
    }
    }
}

// ---------------------------------------------------------------------------
// Shape — top-level emit. Detects custom-shape (mesh-merge / plane
// gprim) vs simple shape, dispatches to the per-type reader, then
// applies the common shape fill (extension fields, materials,
// cross-references) and pushes into out.shapes. The walker tracks
// the (shapeKey, desc*) pair in WalkState::shapeList for pass-2
// ancestor-walk + finalizeCollision.
// ---------------------------------------------------------------------------

// Resolve material binding on a prim → ObjectKey if it has
// UsdPhysicsMaterialAPI applied. Returns invalid sentinel otherwise.
parse::ObjectKey resolvePhysicsMaterialBinding(UsdWalkCtx& impl,
                                               const PXR_NS::UsdPrim& prim)
{
    const PXR_NS::SdfPath matPath = getMaterialBindingPath(prim);
    if (matPath.IsEmpty() || !impl.stage)
        return {};
    const PXR_NS::UsdPrim matPrim = impl.stage->GetPrimAtPath(matPath);
    if (!matPrim || !matPrim.HasAPI<PXR_NS::UsdPhysicsMaterialAPI>())
        return {};
    return impl.source.keyFor(matPath);
}

// Custom-token registry queries are forward-declared at the parent
// namespace scope (above the anonymous block) so they resolve to the
// external-linkage definitions in `CustomTokens.cpp`.
bool isCustomShapeApplied(const PXR_NS::TfTokenVector& apis,
                          const PXR_NS::TfToken& primType,
                          PXR_NS::TfToken& outCustomToken)
{
    for (const auto& a : apis)
    {
        if (isCustomShapeToken(a))
        {
            outCustomToken = a;
            return true;
        }
    }
    if (isCustomShapeToken(primType))
    {
        outCustomToken = primType;
        return true;
    }
    return false;
}

// Walk children of a `PhysxMeshMergeCollisionAPI` parent and assemble
// a MergeMeshDesc from each child's points / indices / face counts /
// holes — concatenated into one merged buffer. Mixed-handedness
// children are normalised by flipping triangle winding on the minority.
void populateMergedMeshFromChildrenNative(
    parse::ParseContext& ctx, UsdWalkCtx& impl,
    parse::MergeMeshPhysxShapeDesc* desc, parse::ObjectKey parentKey)
{
    const parse::TokenId collectionTok = impl.source.internToken("collisionmeshes");
    const parse::TokenId orientationTok = impl.source.internToken("orientation");
    const parse::TokenId leftHandedTok = impl.source.internToken("leftHanded");

    std::vector<parse::ObjectKey> children;
    impl.source.resolveCollection(parentKey, collectionTok, children);

    struct ChildEntry
    {
        parse::ObjectKey                 key;
        parse::BufferSpan<carb::Float3>  points;
        parse::BufferSpan<int32_t>       indices;
        parse::BufferSpan<int32_t>       faceCounts;
        parse::BufferSpan<int32_t>       holes;
        size_t                           pointsOffset = 0;
        size_t                           indicesOffset = 0;
        size_t                           facesOffset = 0;
        size_t                           holesOffset = 0;
        bool                             changeIndicesOrder = false;
    };
    std::vector<ChildEntry> entries;
    entries.reserve(children.size());

    bool leftHanded = false, rightHanded = false;
    for (parse::ObjectKey child : children)
    {
        ChildEntry e;
        e.key = child;
        const parse::MeshGeometry geom = parse::parseMeshGeometry(ctx, child);
        e.points     = ctx.getBuffer<carb::Float3>(geom.points);
        e.indices    = ctx.getBuffer<int32_t>(geom.indices);
        e.faceCounts = ctx.getBuffer<int32_t>(geom.faceCounts);
        e.holes      = ctx.getBuffer<int32_t>(geom.holes);
        if (e.points.empty() || e.indices.empty() || e.faceCounts.empty())
            continue;

        parse::TokenId orient;
        const bool childLeftHanded = impl.source.getAttribute(child, orientationTok, orient) &&
                                     orient == leftHandedTok;

        if (!leftHanded && !rightHanded)
        {
            if (childLeftHanded) leftHanded = true; else rightHanded = true;
        }
        else
        {
            if (childLeftHanded && rightHanded)      e.changeIndicesOrder = true;
            else if (!childLeftHanded && leftHanded) e.changeIndicesOrder = true;
        }
        entries.push_back(std::move(e));
    }

    size_t totalPoints = 0, totalIndices = 0, totalFaces = 0, totalHoles = 0;
    for (auto& e : entries)
    {
        e.pointsOffset  = totalPoints;  totalPoints  += e.points.count;
        e.indicesOffset = totalIndices; totalIndices += e.indices.count;
        e.facesOffset   = totalFaces;   totalFaces   += e.faceCounts.count;
        e.holesOffset   = totalHoles;   totalHoles   += e.holes.count;
    }

    parse::DescPtr<parse::MergeMeshDesc> mm = parse::allocateDesc<parse::MergeMeshDesc>(ctx.descriptorAllocator());
    mm->points.resize(totalPoints);
    mm->indices.resize(totalIndices);
    mm->faces.resize(totalFaces);
    mm->holes.resize(totalHoles);

    parse::Matrix4d parentWorldM4d;
    impl.source.getLocalToWorldTransform(parentKey, parentWorldM4d);
    const PXR_NS::GfMatrix4d parentWorld(
        parentWorldM4d.data[0],  parentWorldM4d.data[1],  parentWorldM4d.data[2],  parentWorldM4d.data[3],
        parentWorldM4d.data[4],  parentWorldM4d.data[5],  parentWorldM4d.data[6],  parentWorldM4d.data[7],
        parentWorldM4d.data[8],  parentWorldM4d.data[9],  parentWorldM4d.data[10], parentWorldM4d.data[11],
        parentWorldM4d.data[12], parentWorldM4d.data[13], parentWorldM4d.data[14], parentWorldM4d.data[15]);
    const PXR_NS::GfMatrix4d parentWorldInv = parentWorld.GetInverse();

    for (const auto& e : entries)
    {
        parse::Matrix4d childWorldM4d;
        impl.source.getLocalToWorldTransform(e.key, childWorldM4d);
        const PXR_NS::GfMatrix4d childWorld(
            childWorldM4d.data[0],  childWorldM4d.data[1],  childWorldM4d.data[2],  childWorldM4d.data[3],
            childWorldM4d.data[4],  childWorldM4d.data[5],  childWorldM4d.data[6],  childWorldM4d.data[7],
            childWorldM4d.data[8],  childWorldM4d.data[9],  childWorldM4d.data[10], childWorldM4d.data[11],
            childWorldM4d.data[12], childWorldM4d.data[13], childWorldM4d.data[14], childWorldM4d.data[15]);
        const PXR_NS::GfMatrix4d relMatrix = childWorld * parentWorldInv;

        for (size_t i = 0; i < e.points.count; ++i)
        {
            const carb::Float3& p = e.points[i];
            const PXR_NS::GfVec3f xformed(relMatrix.Transform(PXR_NS::GfVec3f(p.x, p.y, p.z)));
            mm->points[e.pointsOffset + i] = { xformed[0], xformed[1], xformed[2] };
        }

        std::memcpy(mm->faces.data() + e.facesOffset, e.faceCounts.data,
                    sizeof(int32_t) * e.faceCounts.count);

        if (!e.changeIndicesOrder)
        {
            for (size_t i = 0; i < e.indices.count; ++i)
                mm->indices[e.indicesOffset + i] = e.indices[i] + int32_t(e.pointsOffset);
        }
        else
        {
            size_t faceOffset = 0;
            for (size_t f = 0; f < e.faceCounts.count; ++f)
            {
                const int32_t faceIndices = e.faceCounts[f];
                if (faceIndices > 0)
                {
                    size_t newIndex = 0;
                    for (size_t i = faceOffset + size_t(faceIndices) - 1; i >= faceOffset; --i)
                    {
                        mm->indices[e.indicesOffset + faceOffset + newIndex] =
                            e.indices[faceOffset + i] + int32_t(e.pointsOffset);
                        ++newIndex;
                        if (i == faceOffset) break;
                    }
                    faceOffset += size_t(faceIndices);
                }
            }
        }

        for (size_t i = 0; i < e.holes.count; ++i)
            mm->holes[e.holesOffset + i] = e.holes[i] + int32_t(e.facesOffset);
    }

    desc->mergedMesh = mm.get();
    impl.scan.retainOwnedMesh(std::move(mm));
}

// Build a custom-mesh-merge shape: discriminate the parent's mesh
// approximation + assemble mergedMesh from the `collisionMeshes`
// collection.
parse::DescPtr<parse::PhysxShapeDesc> buildMeshMergeCustomShape(
    parse::ParseContext& ctx, UsdWalkCtx& impl, parse::ObjectKey shapeKey)
{
    carb::Float3 meshScale = { 1.0f, 1.0f, 1.0f };
    {
        parse::Matrix3d rot;
        impl.source.getLocalToWorldRotationAndScale(shapeKey, rot, meshScale);
    }
    const carb::Float3 signScale = parse::scaleToSignScale(meshScale);

    parse::DescPtr<parse::MergeMeshPhysxShapeDesc> typed;
    const parse::MeshApproximation approx = parse::parseMeshApproximation(ctx, shapeKey);
    switch (approx)
    {
    case parse::MeshApproximation::eConvexHull:
    {
        parse::DescPtr<parse::ConvexMeshPhysxShapeDesc> cmd = parse::allocateDesc<parse::ConvexMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        cmd->meshScale = meshScale;
        cmd->meshPrimKey = shapeKey;
        cmd->convexCookingParams.signScale = signScale;
        parse::parseConvexHullCookingExt(ctx, shapeKey, cmd->convexCookingParams);
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(cmd));
        break;
    }
    case parse::MeshApproximation::eMeshSimplification:
    {
        parse::DescPtr<parse::TriangleMeshPhysxShapeDesc> tmd = parse::allocateDesc<parse::TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = shapeKey;
        tmd->triangleMeshCookingParams.mode = omni::physx::TriangleMeshMode::eQUADRIC_SIMPLIFICATION;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseTriangleMeshSimplificationCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(tmd));
        break;
    }
    case parse::MeshApproximation::eConvexDecomposition:
    {
        parse::DescPtr<parse::ConvexMeshDecompositionPhysxShapeDesc> cdd = parse::allocateDesc<parse::ConvexMeshDecompositionPhysxShapeDesc>(ctx.descriptorAllocator());
        cdd->meshScale = meshScale;
        cdd->meshPrimKey = shapeKey;
        cdd->convexDecompositionCookingParams.signScale = signScale;
        cdd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseConvexDecompositionCookingExt(ctx, shapeKey, cdd->convexDecompositionCookingParams);
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(cdd));
        break;
    }
    case parse::MeshApproximation::eSphereFill:
    {
        parse::DescPtr<parse::SpherePointsPhysxShapeDesc> sfd = parse::allocateDesc<parse::SpherePointsPhysxShapeDesc>(ctx.descriptorAllocator());
        sfd->meshScale = meshScale;
        sfd->meshPrimKey = shapeKey;
        sfd->sphereFillCookingParams.signScale = signScale;
        sfd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseSphereFillCookingExt(ctx, shapeKey, sfd->sphereFillCookingParams);
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(sfd));
        break;
    }
    case parse::MeshApproximation::eSdf:
    {
        parse::DescPtr<parse::TriangleMeshPhysxShapeDesc> tmd = parse::allocateDesc<parse::TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = shapeKey;
        parse::parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        parse::parseSdfMeshCookingExt(ctx, shapeKey, tmd->sdfMeshCookingParams);
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(tmd));
        break;
    }
    case parse::MeshApproximation::eBoundingSphere:
    {
        // Bounding-sphere: emit the BoundingSpherePhysxShapeDesc subclass;
        // the consumer fills positionOffset / radius from mergedMesh
        // (populated by populateMergedMeshFromChildrenNative below).
        parse::DescPtr<parse::BoundingSpherePhysxShapeDesc> bs = parse::allocateDesc<parse::BoundingSpherePhysxShapeDesc>(ctx.descriptorAllocator());
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(bs));
        break;
    }
    case parse::MeshApproximation::eBoundingCube:
    {
        parse::DescPtr<parse::BoundingBoxPhysxShapeDesc> bb = parse::allocateDesc<parse::BoundingBoxPhysxShapeDesc>(ctx.descriptorAllocator());
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(bb));
        break;
    }
    case parse::MeshApproximation::eNone:
    default:
    {
        parse::DescPtr<parse::TriangleMeshPhysxShapeDesc> tmd = parse::allocateDesc<parse::TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = shapeKey;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parse::parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        typed = parse::descPtrCast<parse::MergeMeshPhysxShapeDesc>(std::move(tmd));
        break;
    }
    }
    populateMergedMeshFromChildrenNative(ctx, impl, typed.get(), shapeKey);
    return parse::descPtrCast<parse::PhysxShapeDesc>(std::move(typed));
}

// Apply the common shape fill — extension fields, simulationOwners,
// materials, filteredCollisions, primKey, rigidBody (left empty
// until pass-2), sourceGprim.
void fillCommonShapeFields(parse::ParseContext& ctx, UsdWalkCtx& impl,
                           parse::PhysxShapeDesc& desc,
                           parse::ObjectKey shapeKey,
                           parse::ObjectKey gprimKey,
                           const PXR_NS::UsdPrim& shapePrim,
                           const PXR_NS::UsdPhysicsCollisionAPI& colAPI)
{
    parse::setToDefault(desc, impl.source.getSourceUnits());
    bool collisionEnabled = true;
    colAPI.GetCollisionEnabledAttr().Get(&collisionEnabled);
    desc.collisionEnabled = collisionEnabled;
    desc.primKey = shapeKey;
    desc.sourceGprim = gprimKey.valid() ? gprimKey : shapeKey;

    parse::CollisionExtFields extFields;
    extFields.contactOffset           = desc.contactOffset;
    extFields.restOffset              = desc.restOffset;
    extFields.torsionalPatchRadius    = desc.torsionalPatchRadius;
    extFields.minTorsionalPatchRadius = desc.minTorsionalPatchRadius;
    extFields.isTrigger               = desc.isTrigger;
    extFields.isTriggerUsdOutput      = desc.isTriggerUsdOutput;
    parse::parseCollisionExt(ctx, shapeKey, extFields);
    desc.contactOffset           = extFields.contactOffset;
    desc.restOffset              = extFields.restOffset;
    desc.torsionalPatchRadius    = extFields.torsionalPatchRadius;
    desc.minTorsionalPatchRadius = extFields.minTorsionalPatchRadius;
    desc.isTrigger               = extFields.isTrigger;
    desc.isTriggerUsdOutput      = extFields.isTriggerUsdOutput;

    // Materials: mesh-subset bindings first (mesh only), then the
    // collider's own binding as the trailing default-fallback slot.
    // Subset materials at indices [0..N), collider binding at [N].
    // Consumers drive PxShape::setMaterials directly from this vector.
    if (shapePrim.IsA<PXR_NS::UsdGeomMesh>())
    {
        const PXR_NS::UsdGeomMesh meshGeom(shapePrim);
        const std::vector<PXR_NS::UsdGeomSubset> subsets =
            PXR_NS::UsdGeomSubset::GetGeomSubsets(meshGeom, PXR_NS::UsdGeomTokens->face);
        for (const PXR_NS::UsdGeomSubset& subset : subsets)
        {
            const PXR_NS::SdfPath subMat = getMaterialBindingPath(subset.GetPrim());
            if (subMat.IsEmpty() || !impl.stage) continue;
            const PXR_NS::UsdPrim matPrim = impl.stage->GetPrimAtPath(subMat);
            if (matPrim && matPrim.HasAPI<PXR_NS::UsdPhysicsMaterialAPI>())
                desc.sourceMaterials.push_back(impl.source.keyFor(subMat));
        }
    }
    {
        const PXR_NS::SdfPath matPath = getMaterialBindingPath(shapePrim);
        if (!matPath.IsEmpty() && impl.stage)
        {
            const PXR_NS::UsdPrim matPrim = impl.stage->GetPrimAtPath(matPath);
            if (matPrim && matPrim.HasAPI<PXR_NS::UsdPhysicsMaterialAPI>())
                desc.sourceMaterials.push_back(impl.source.keyFor(matPath));
            else
                desc.sourceMaterials.push_back({});
        }
        else
        {
            desc.sourceMaterials.push_back({});
        }
    }

    // Simulation owners.
    const PXR_NS::UsdRelationship ownerRel = colAPI.GetSimulationOwnerRel();
    if (ownerRel)
    {
        PXR_NS::SdfPathVector owners;
        ownerRel.GetTargets(&owners);
        desc.sourceSimulationOwners.reserve(owners.size());
        for (const PXR_NS::SdfPath& p : owners)
            desc.sourceSimulationOwners.push_back(impl.source.keyFor(p));
    }

    // Filtered pairs (per-collider).
    desc.sourceFilteredCollisions = parse::parseFilteredPairs(ctx, shapeKey);
}

// Pre-check: should this prim be treated as a deformable collider
// instead of a rigid collider? Walks parents looking for an enabled
// deformable body via WalkState.
bool isDeformableCollider(WalkState& walk, UsdWalkCtx& impl,
                          const PXR_NS::UsdPrim& shapePrim)
{
    if (!impl.stage) return false;
    PXR_NS::UsdPrim p = shapePrim;
    while (p && p != impl.stage->GetPseudoRoot())
    {
        const parse::ObjectKey k = impl.source.keyFor(p.GetPrimPath());
        const auto it = walk.deformableBodyMap.find(k);
        if (it != walk.deformableBodyMap.end() && it->second)
            return true;
        p = p.GetParent();
    }
    return false;
}

void emitShape(ScannedStage& out, UsdWalkCtx& impl,
               parse::ParseContext& ctx, WalkState& walk,
               parse::ObjectKey key, const PXR_NS::UsdPrim& shapePrim,
               const PXR_NS::TfTokenVector& apis,
               uint64_t typeBits)
{
    if (isDeformableCollider(walk, impl, shapePrim))
        return;

    // Instance-proxy handling. Parse the prototype's geometry (the
    // prim-in-prototype has the authored attributes); apply
    // scaleShapeDescByInstance for the instance's world scale. Per-
    // instance descriptors are freshly allocated at the parse-lib
    // layer, so no master-mutation hazard.
    const bool isInstanceProxy = shapePrim.IsInstanceProxy();

    // World-space scale for the gprim drives per-shape geometry
    // scaling. Instance-proxy prims read the prototype with
    // worldM=identity; the instance's own world scale is baked in
    // separately below via scaleShapeDescByInstance.
    const PXR_NS::GfMatrix4d worldM = isInstanceProxy
                                        ? PXR_NS::GfMatrix4d(1.0)
                                        : walk.xfCache.GetLocalToWorldTransform(shapePrim);
    const PXR_NS::GfTransform worldT(worldM);
    const PXR_NS::GfVec3d worldScale = worldT.GetScale();

    // Custom-shape detection: the mesh-merge API takes the
    // buildMeshMergeCustomShape path; other registered custom tokens
    // route to a generic CustomPhysxShapeDesc.
    PXR_NS::TfToken customTok;
    const bool customShape = isCustomShapeApplied(apis, shapePrim.GetTypeName(), customTok);

    parse::DescPtr<parse::PhysxShapeDesc> desc;

    // Plane gprim special-cases to a PlanePhysxShapeDesc regardless of
    // custom-API presence.
    if (shapePrim.IsA<PXR_NS::UsdGeomPlane>())
    {
        desc = readPlaneDesc(ctx, shapePrim);
    }
    else if (customShape)
    {
        const size_t tokenHash = omni::physx::computeCustomGeometryHash(customTok);
        static const size_t sMeshMergeHash =
            omni::physx::computeCustomGeometryHash(std::string("PhysxMeshMergeCollisionAPI"));
        if (tokenHash == sMeshMergeHash)
        {
            desc = buildMeshMergeCustomShape(ctx, impl, key);
        }
        else
        {
            parse::DescPtr<parse::CustomPhysxShapeDesc> cu = parse::allocateDesc<parse::CustomPhysxShapeDesc>(ctx.descriptorAllocator());
            cu->customGeometryTokenHash = tokenHash;
            desc = parse::descPtrCast<parse::PhysxShapeDesc>(std::move(cu));
        }
    }
    else if (shapePrim.IsA<PXR_NS::UsdGeomCube>())
    {
        desc = readCubeDesc(ctx, shapePrim, worldScale);
    }
    else if (shapePrim.IsA<PXR_NS::UsdGeomSphere>())
    {
        desc = readSphereDesc(ctx, shapePrim, worldScale);
    }
    else if (shapePrim.IsA<PXR_NS::UsdGeomCapsule>())
    {
        desc = readCapsuleDesc(ctx, shapePrim, worldScale);
    }
    else if (shapePrim.IsA<PXR_NS::UsdGeomCylinder>())
    {
        desc = readCylinderDesc(ctx, shapePrim, worldScale);
    }
    else if (shapePrim.IsA<PXR_NS::UsdGeomCone>())
    {
        desc = readConeDesc(ctx, shapePrim, worldScale);
    }
    else if (shapePrim.IsA<PXR_NS::UsdGeomPoints>())
    {
        desc = readSpherePointsDesc(ctx, shapePrim, worldScale);
    }
    else if (shapePrim.IsA<PXR_NS::UsdGeomMesh>())
    {
        const carb::Float3 meshScale = isInstanceProxy
                                            ? carb::Float3{ 1.0f, 1.0f, 1.0f }
                                            : carb::Float3{ float(worldScale[0]),
                                                            float(worldScale[1]),
                                                            float(worldScale[2]) };
        const carb::Float3 signScale = parse::scaleToSignScale(meshScale);
        bool doubleSided = false;
        PXR_NS::UsdGeomMesh m(shapePrim);
        m.GetDoubleSidedAttr().Get(&doubleSided);
        // Resolve the body ancestor up-front so AC-15 (triangleMesh /
        // MeshSimplification → ConvexHull fallback for dynamic non-
        // kinematic bodies) can fire inside readMeshShapeDesc. Pass-2's
        // own ancestor walk only runs *after* the desc subclass is
        // chosen, so without this resolution the AC-15 gate would never
        // trigger and the consumer would later reject the resulting
        // TriangleMeshPhysxShapeDesc with `attachShape: non-SDF triangle
        // mesh ... eSIMULATION_SHAPE are not supported for
        // non-kinematic PxRigidDynamic instances`.
        parse::ObjectKey rigidBodyKey;
        if (impl.stage)
        {
            // Walk parents looking either for a body already in bodyMap
            // OR a prim with UsdPhysicsRigidBodyAPI applied (the latter
            // covers the case where the body and shape are on the same
            // prim; emitShape is called before emitRigidBody for that
            // prim, so the bodyMap entry isn't there yet).
            PXR_NS::UsdPrim p = shapePrim;
            while (p && p != impl.stage->GetPseudoRoot())
            {
                const parse::ObjectKey k = impl.source.keyFor(p.GetPrimPath());
                if (walk.bodyMap.find(k) != walk.bodyMap.end() ||
                    p.HasAPI<PXR_NS::UsdPhysicsRigidBodyAPI>())
                {
                    rigidBodyKey = k;
                    break;
                }
                p = p.GetParent();
            }
        }
        desc = readMeshShapeDesc(ctx, impl, key, shapePrim, meshScale, signScale,
                                 doubleSided, rigidBodyKey,
                                 /*gprimKey=*/ key);
    }
    else
    {
        // Non-gprim collider (an Xform with UsdPhysicsCollisionAPI):
        // walk the collider's subtree and emit one shape per descendant
        // gprim, all keyed at the collider's prim path so the consumer-
        // side static-body creation lands at the collider.
        const PXR_NS::UsdPhysicsCollisionAPI colliderColAPI(shapePrim);
        PXR_NS::UsdPrimRange range(shapePrim, PXR_NS::UsdTraverseInstanceProxies());
        for (auto rit = range.begin(); rit != range.end(); ++rit)
        {
            const PXR_NS::UsdPrim& childPrim = *rit;
            if (!childPrim) continue;
            if (!childPrim.IsA<PXR_NS::UsdGeomGprim>() &&
                !childPrim.IsA<PXR_NS::UsdGeomPlane>())
                continue;

            const parse::ObjectKey childKey = impl.source.keyFor(childPrim.GetPrimPath());

            const PXR_NS::GfMatrix4d childWorldM = walk.xfCache.GetLocalToWorldTransform(childPrim);
            const PXR_NS::GfTransform childWorldT(childWorldM);
            const PXR_NS::GfVec3d childWorldScale = childWorldT.GetScale();

            parse::DescPtr<parse::PhysxShapeDesc> childDesc;
            if (childPrim.IsA<PXR_NS::UsdGeomPlane>())
                childDesc = readPlaneDesc(ctx, childPrim);
            else if (childPrim.IsA<PXR_NS::UsdGeomCube>())
                childDesc = readCubeDesc(ctx, childPrim, childWorldScale);
            else if (childPrim.IsA<PXR_NS::UsdGeomSphere>())
                childDesc = readSphereDesc(ctx, childPrim, childWorldScale);
            else if (childPrim.IsA<PXR_NS::UsdGeomCapsule>())
                childDesc = readCapsuleDesc(ctx, childPrim, childWorldScale);
            else if (childPrim.IsA<PXR_NS::UsdGeomCylinder>())
                childDesc = readCylinderDesc(ctx, childPrim, childWorldScale);
            else if (childPrim.IsA<PXR_NS::UsdGeomCone>())
                childDesc = readConeDesc(ctx, childPrim, childWorldScale);
            else if (childPrim.IsA<PXR_NS::UsdGeomPoints>())
                childDesc = readSpherePointsDesc(ctx, childPrim, childWorldScale);
            else if (childPrim.IsA<PXR_NS::UsdGeomMesh>())
            {
                const carb::Float3 meshScale{ float(childWorldScale[0]),
                                              float(childWorldScale[1]),
                                              float(childWorldScale[2]) };
                const carb::Float3 signScale = parse::scaleToSignScale(meshScale);
                bool doubleSided = false;
                PXR_NS::UsdGeomMesh(childPrim).GetDoubleSidedAttr().Get(&doubleSided);
                // Resolve a body ancestor of the collider (not the child gprim)
                // for AC-15 (TriangleMesh/MeshSimplification → ConvexHull on
                // dynamic non-kinematic bodies).
                parse::ObjectKey rigidBodyKey;
                if (impl.stage)
                {
                    PXR_NS::UsdPrim p = shapePrim;
                    while (p && p != impl.stage->GetPseudoRoot())
                    {
                        const parse::ObjectKey k = impl.source.keyFor(p.GetPrimPath());
                        if (walk.bodyMap.find(k) != walk.bodyMap.end() ||
                            p.HasAPI<PXR_NS::UsdPhysicsRigidBodyAPI>())
                        {
                            rigidBodyKey = k;
                            break;
                        }
                        p = p.GetParent();
                    }
                }
                childDesc = readMeshShapeDesc(ctx, impl, key, childPrim, meshScale, signScale,
                                              doubleSided, rigidBodyKey, /*gprimKey=*/ childKey);
            }
            else
            {
                continue;
            }
            if (!childDesc) continue;

            // Common fields: shapeKey = collider's key (so consumer registers
            // the shape and any synthesized static body at the collider's
            // path); gprimKey = child gprim's key (so material/filter/ext
            // reads pull from the child gprim and pass-2's local-transform
            // computation can use the child gprim's world transform).
            fillCommonShapeFields(ctx, impl, *childDesc, key, /*gprimKey=*/ childKey,
                                  childPrim, colliderColAPI);

            walk.shapeList.emplace_back(key, childDesc.get());
            out.shapes.push_back(std::move(childDesc));
        }
        return;
    }

    if (!desc) return;

    const PXR_NS::UsdPhysicsCollisionAPI colAPI(shapePrim);
    fillCommonShapeFields(ctx, impl, *desc, key, /*gprimKey=*/ key, shapePrim, colAPI);

    // Instance-proxy: bake the instance's world transform.
    if (isInstanceProxy)
    {
        parse::Matrix4d wm;
        impl.source.getLocalToWorldTransform(key, wm);
        const carb::Float3 worldTranslate{
            static_cast<float>(wm.data[12]),
            static_cast<float>(wm.data[13]),
            static_cast<float>(wm.data[14]) };
        carb::Float3 worldScaleF = { 1.0f, 1.0f, 1.0f };
        parse::Matrix3d rot;
        impl.source.getLocalToWorldRotationAndScale(key, rot, worldScaleF);
        parse::scaleShapeDescByInstance(*desc, worldScaleF);
        desc->localPos   = worldTranslate;
        desc->localScale = worldScaleF;
    }

    walk.shapeList.emplace_back(key, desc.get());
    out.shapes.push_back(std::move(desc));
}

// ---------------------------------------------------------------------------
// Deformable body — hierarchy walk + per-prim emit. Classifies each
// descendant point-based prim as sim-mesh / collision-mesh / skin-geom
// and assembles a PhysxDeformableBodyDesc with per-prim attribute
// overlays (PhysxBaseDeformableBodyAPI, PhysxSurfaceDeformableBodyAPI,
// PhysxAutoDeformableBodyAPI extensions).
// ---------------------------------------------------------------------------

bool isPointBased(const PXR_NS::UsdPrim& p)
{
    return p.IsA<PXR_NS::UsdGeomPointBased>();
}

bool isSimMesh(parse::ObjectType& outType, const PXR_NS::UsdPrim& prim)
{
    static const PXR_NS::TfType volumeType =
        PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PXR_NS::TfToken("OmniPhysicsVolumeDeformableSimAPI"));
    static const PXR_NS::TfType surfaceType =
        PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PXR_NS::TfToken("OmniPhysicsSurfaceDeformableSimAPI"));
    static const PXR_NS::TfType curvesType =
        PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PXR_NS::TfToken("OmniPhysicsCurvesDeformableSimAPI"));

    outType = parse::eUndefined;
    if (prim.HasAPI(volumeType))
    {
        if (!prim.IsA<PXR_NS::UsdGeomTetMesh>())
        {
            CARB_LOG_ERROR("VolumeDeformableSimAPI applied to non-tetmesh. (%s)", prim.GetPrimPath().GetText());
            return false;
        }
        outType = parse::eVolumeDeformableBody;
    }
    else if (prim.HasAPI(surfaceType))
    {
        if (!prim.IsA<PXR_NS::UsdGeomMesh>())
        {
            CARB_LOG_ERROR("SurfaceDeformableSimAPI applied to non-trimesh. (%s)", prim.GetPrimPath().GetText());
            return false;
        }
        outType = parse::eSurfaceDeformableBody;
    }
    // CurvesDeformableSimAPI has no parse-lib enum (the parse-lib only
    // models Volume + Surface) — a curves body falls through as
    // eUndefined and is skipped by the caller.
    (void)curvesType;
    return outType != parse::eUndefined;
}

bool isEnabledCollisionGeom(const PXR_NS::UsdPrim& prim)
{
    PXR_NS::UsdPhysicsCollisionAPI colAPI(prim);
    if (!colAPI) return false;
    if (!prim.IsA<PXR_NS::UsdGeomPointBased>())
    {
        CARB_LOG_ERROR("CollisionAPI in deformable body subtree is not GeomPointBased. (%s)",
                       prim.GetPrimPath().GetText());
        return false;
    }
    bool enabled = true;
    colAPI.GetCollisionEnabledAttr().Get(&enabled);
    return enabled;
}

// Discover the pose-purposes-named instance name on a multi-applicable
// DeformablePoseAPI prim matching a given purpose token.
PXR_NS::TfToken getPoseNameFromPurpose(const PXR_NS::UsdPrim& prim,
                                       const PXR_NS::TfToken& posePurposeToken)
{
    // Lookup key MUST be the registered schemaIdentifier
    // "OmniPhysicsDeformablePoseAPI" — the C++ className suffix
    // "DeformablePoseAPI" alone returns TfType::GetUnknownType().
    static const PXR_NS::TfType poseType =
        PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(
            PXR_NS::TfToken("OmniPhysicsDeformablePoseAPI"));
    static const PXR_NS::TfToken poseTypeName =
        PXR_NS::UsdSchemaRegistry::GetAPISchemaTypeName(poseType);
    static const PXR_NS::TfToken purposesTemplate("omniphysics:__INSTANCE_NAME__:purposes");

    const PXR_NS::TfTokenVector allAPIs = prim.GetAppliedSchemas();
    for (const auto& api : allAPIs)
    {
        std::pair<PXR_NS::TfToken, PXR_NS::TfToken> nameAndInstance =
            PXR_NS::UsdSchemaRegistry::GetTypeNameAndInstance(api);
        if (nameAndInstance.first != poseTypeName) continue;

        const PXR_NS::TfToken attrName =
            PXR_NS::UsdSchemaRegistry::MakeMultipleApplyNameInstance(purposesTemplate, nameAndInstance.second);
        PXR_NS::VtArray<PXR_NS::TfToken> candTokens;
        prim.GetAttribute(attrName).Get(&candTokens);
        for (const PXR_NS::TfToken& cand : candTokens)
            if (cand == posePurposeToken)
                return nameAndInstance.second;
    }
    return {};
}

PXR_NS::SdfPath getDeformableMaterialBinding(PXR_NS::UsdStageWeakPtr stage,
                                             const PXR_NS::SdfPath& primKey,
                                             parse::ObjectType deformableType)
{
    static const PXR_NS::TfType matType =
        PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PXR_NS::TfToken("DeformableMaterialAPI"));
    static const PXR_NS::TfType surfaceMatType =
        PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PXR_NS::TfToken("SurfaceDeformableMaterialAPI"));
    static const PXR_NS::TfType curvesMatType =
        PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PXR_NS::TfToken("CurvesDeformableMaterialAPI"));

    if (!stage) return {};
    const PXR_NS::UsdPrim prim = stage->GetPrimAtPath(primKey);
    if (!prim) return {};
    const PXR_NS::SdfPath matPath = getMaterialBindingPath(prim);
    if (matPath.IsEmpty()) return {};
    const PXR_NS::UsdPrim matPrim = stage->GetPrimAtPath(matPath);
    if (!matPrim) return {};

    const bool needSurface = (deformableType == parse::eSurfaceDeformableBody);
    const bool needCurves  = false; // parse-lib has no Curves variant; gated upstream
    (void)curvesMatType;
    if (needSurface && matPrim.HasAPI(surfaceMatType)) return matPath;
    if (needCurves  && matPrim.HasAPI(curvesMatType))  return matPath;
    if (matPrim.HasAPI(matType))                       return matPath;
    return {};
}

void emitDeformableBody(ScannedStage& out, UsdWalkCtx& impl,
                        parse::ParseContext& ctx, WalkState& walk,
                        parse::ObjectKey key, const PXR_NS::UsdPrim& bodyPrim,
                        uint64_t typeBits)
{
    static const uint64_t geomTypes = PrimTypeBits::eUsdGeomTetMesh |
                                      PrimTypeBits::eUsdGeomMesh |
                                      PrimTypeBits::eUsdGeomBasisCurves;
    const bool validGeomTypes = (typeBits & geomTypes) > 0;
    const bool validRootTypes = (typeBits & PrimTypeBits::eUsdGeomGprim) == 0 &&
                                (typeBits & PrimTypeBits::eUsdGeomImageable) > 0;
    if (!validGeomTypes && !validRootTypes)
    {
        CARB_LOG_ERROR("DeformableBodyAPI can only be applied to Mesh, TetMesh or BasisCurves "
                       "for single node deformables or non-Gprim Imageable for hierarchical "
                       "setups. (%s)", bodyPrim.GetPrimPath().GetText());
        return;
    }

    // Per-prim overlay first — parseDeformableBody reads the per-prim
    // attribute and relationship fields (bodyEnabled, kinematic, mass,
    // simulationOwners, filteredPairs).
    const parse::DeformableBodyParse parsed = parse::parseDeformableBody(ctx, key);

    // Hierarchy walk — classify sim-mesh / collision-mesh / skin-geom.
    PXR_NS::UsdPrim simMeshPrim;
    parse::ObjectType simMeshType = parse::eUndefined;
    std::vector<PXR_NS::SdfPath> collisionGeomPaths;
    std::vector<PXR_NS::TfToken> collisionGeomBindPoseTokens;
    std::vector<bool>            collisionGeomLeftHandedOrientations;
    std::vector<PXR_NS::TfToken> collisionGeomSelfCollisionFilterPoseTokens;
    std::vector<PXR_NS::SdfPath> skinGeomPaths;
    std::vector<PXR_NS::TfToken> skinGeomBindPoseTokens;
    std::vector<PXR_NS::SdfPath> hierarchyFilteredPairs;

    static const PXR_NS::TfToken kBindPose("bindPose");
    static const PXR_NS::TfToken kSelfCollisionFilterPose("selfCollisionFilterPose");

    PXR_NS::UsdPrimRange prims(bodyPrim, PXR_NS::UsdPrimAllPrimsPredicate);
    for (auto it = prims.begin(); it != prims.end(); ++it)
    {
        const PXR_NS::UsdPrim p = *it;
        if (!isPointBased(p)) continue;
        if (p != bodyPrim && walk.xfCache.GetResetXformStack(p))
        {
            it.PruneChildren();
            continue;
        }
        parse::ObjectType simType = parse::eUndefined;
        const bool isSim  = isSimMesh(simType, p);
        const bool isColl = isEnabledCollisionGeom(p);
        if (isSim)
        {
            if (simMeshPrim.IsValid())
            {
                CARB_LOG_ERROR("Multiple deformable simulation meshes found at (%s).",
                               bodyPrim.GetPrimPath().GetText());
                return;
            }
            if (p != bodyPrim && p.GetParent() != bodyPrim)
            {
                CARB_LOG_ERROR("DeformableSimAPI is neither on the body prim nor an immediate child (%s).",
                               bodyPrim.GetPrimPath().GetText());
                return;
            }
            simMeshPrim = p;
            simMeshType = simType;
        }
        if (isColl)
        {
            collisionGeomPaths.push_back(p.GetPath());
            collisionGeomBindPoseTokens.push_back(getPoseNameFromPurpose(p, kBindPose));
            PXR_NS::TfToken orientation;
            PXR_NS::UsdGeomGprim(p).GetOrientationAttr().Get(&orientation);
            collisionGeomLeftHandedOrientations.push_back(orientation == PXR_NS::UsdGeomTokens->leftHanded);
            collisionGeomSelfCollisionFilterPoseTokens.push_back(
                getPoseNameFromPurpose(p, kSelfCollisionFilterPose));

            // Per-collision-mesh filtered-pairs: parseFilteredPairs on
            // the child key reads PhysxFilteredPairsAPI applied there.
            std::vector<parse::ObjectKey> childPairs =
                parse::parseFilteredPairs(ctx, impl.source.keyFor(p.GetPath()));
            for (auto k : childPairs)
                hierarchyFilteredPairs.push_back(impl.source.pathFor(k));
        }
        if (!isSim && !isColl)
        {
            skinGeomPaths.push_back(p.GetPath());
            skinGeomBindPoseTokens.push_back(getPoseNameFromPurpose(p, kBindPose));
        }
    }

    if (!simMeshPrim)
    {
        CARB_LOG_ERROR("Deformable simulation mesh is not valid (%s).",
                       bodyPrim.GetPrimPath().GetText());
        return;
    }

    // Build the parse-lib desc.
    parse::DescPtr<parse::PhysxDeformableBodyDesc> desc;
    const bool isSurface = (simMeshType == parse::eSurfaceDeformableBody);
    if (isSurface)
    {
        parse::DescPtr<parse::PhysxSurfaceDeformableBodyDesc> d = parse::allocateDesc<parse::PhysxSurfaceDeformableBodyDesc>(ctx.descriptorAllocator());
        parse::setToDefault(*d, impl.source.getSourceUnits());
        desc = parse::descPtrCast<parse::PhysxDeformableBodyDesc>(std::move(d));
    }
    else
    {
        parse::DescPtr<parse::PhysxVolumeDeformableBodyDesc> d = parse::allocateDesc<parse::PhysxVolumeDeformableBodyDesc>(ctx.descriptorAllocator());
        parse::setToDefault(*d, impl.source.getSourceUnits());
        desc = parse::descPtrCast<parse::PhysxDeformableBodyDesc>(std::move(d));
    }
    desc->primKey      = key;
    desc->bodyEnabled   = parsed.bodyEnabled;
    desc->kinematicBody = parsed.kinematicBody;
    desc->startsAsleep  = parsed.startsAsleep;
    desc->mass          = (parsed.mass <= 0.0f) ? -1.0f : parsed.mass;
    desc->sourceSimulationOwners = std::move(parsed.simulationOwners);

    // Combine hierarchy-resolved filtered pairs with the body-prim's
    // own parseFilteredPairs result (mirrors emitDeformableBody's
    // "prefer the schema parser's already-aggregated vector" path).
    {
        std::vector<parse::ObjectKey> bodyPairs = parsed.filteredCollisions;
        desc->sourceFilteredCollisions = std::move(bodyPairs);
        desc->sourceFilteredCollisions.reserve(
            desc->sourceFilteredCollisions.size() + hierarchyFilteredPairs.size());
        for (const auto& p : hierarchyFilteredPairs)
            desc->sourceFilteredCollisions.push_back(impl.source.keyFor(p));
    }

    // Transform — world-space matrix from the body prim.
    {
        parse::Matrix4d m;
        impl.source.getLocalToWorldTransform(key, m);
        std::memcpy(desc->transform.data, m.data, sizeof(double) * 16);
    }

    // Sim mesh + pose-purpose tokens + handedness.
    desc->simMeshKey = impl.source.keyFor(simMeshPrim.GetPath());
    {
        const PXR_NS::TfToken bindPose = getPoseNameFromPurpose(simMeshPrim, kBindPose);
        if (!bindPose.IsEmpty())
            desc->simMeshBindPoseToken = impl.source.internToken(bindPose.GetString());
        PXR_NS::TfToken orientation;
        PXR_NS::UsdGeomGprim(simMeshPrim).GetOrientationAttr().Get(&orientation);
        desc->simMeshLeftHandedOrientation = (orientation == PXR_NS::UsdGeomTokens->leftHanded);
    }

    // First collision geom only — multiple collision meshes per body
    // are silently dropped here (single-mesh semantic).
    if (!collisionGeomPaths.empty())
    {
        desc->collisionMeshKey = impl.source.keyFor(collisionGeomPaths[0]);
        if (!collisionGeomBindPoseTokens.empty() && !collisionGeomBindPoseTokens[0].IsEmpty())
            desc->collisionMeshBindPoseToken =
                impl.source.internToken(collisionGeomBindPoseTokens[0].GetString());
        if (!collisionGeomLeftHandedOrientations.empty())
            desc->collisionMeshLeftHandedOrientation = collisionGeomLeftHandedOrientations[0];
    }

    desc->skinGeomPaths.reserve(skinGeomPaths.size());
    for (const auto& p : skinGeomPaths)
        desc->skinGeomPaths.push_back(impl.source.keyFor(p));
    desc->skinGeomBindPoseTokens.reserve(skinGeomBindPoseTokens.size());
    for (const auto& t : skinGeomBindPoseTokens)
        desc->skinGeomBindPoseTokens.push_back(
            t.IsEmpty() ? parse::TokenId{} : impl.source.internToken(t.GetString()));

    // PhysxBaseDeformableBodyAPI / PhysxSurfaceDeformableBodyAPI /
    // PhysxAutoDeformableBodyAPI extension reads.
    const parse::TokenId baseDBTok = impl.source.internToken("PhysxBaseDeformableBodyAPI");
    if (impl.source.hasSchema(key, baseDBTok))
    {
        auto rdF = [&](const char* name, float def) {
            float v = def;
            impl.source.getAttribute(key, impl.source.internToken(name), v);
            return v;
        };
        auto rdB = [&](const char* name, bool def) {
            bool v = def;
            impl.source.getAttribute(key, impl.source.internToken(name), v);
            return v;
        };
        auto rdU = [&](const char* name, uint32_t def) {
            int64_t iv;
            if (impl.source.getAttribute(key, impl.source.internToken(name), iv))
                return uint32_t(iv);
            return def;
        };
        desc->linearDamping              = rdF("physxDeformableBody:linearDamping",              desc->linearDamping);
        desc->maxLinearVelocity          = rdF("physxDeformableBody:maxLinearVelocity",          desc->maxLinearVelocity);
        desc->sleepThreshold             = rdF("physxDeformableBody:sleepThreshold",             desc->sleepThreshold);
        desc->settlingThreshold          = rdF("physxDeformableBody:settlingThreshold",          desc->settlingThreshold);
        desc->settlingDamping            = rdF("physxDeformableBody:settlingDamping",            desc->settlingDamping);
        desc->maxDepenetrationVelocity   = rdF("physxDeformableBody:maxDepenetrationVelocity",   desc->maxDepenetrationVelocity);
        desc->selfCollisionFilterDistance = rdF("physxDeformableBody:selfCollisionFilterDistance", desc->selfCollisionFilterDistance);
        desc->solverPositionIterationCount = rdU("physxDeformableBody:solverPositionIterationCount", desc->solverPositionIterationCount);
        desc->enableSpeculativeCCD       = rdB("physxDeformableBody:enableSpeculativeCCD", desc->enableSpeculativeCCD);
        desc->selfCollision              = rdB("physxDeformableBody:selfCollision",       desc->selfCollision);
        desc->disableGravity             = rdB("physxDeformableBody:disableGravity",      desc->disableGravity);
    }
    if (isSurface)
    {
        const parse::TokenId surfDBTok = impl.source.internToken("PhysxSurfaceDeformableBodyAPI");
        if (impl.source.hasSchema(key, surfDBTok))
        {
            parse::PhysxSurfaceDeformableBodyDesc* surf = static_cast<parse::PhysxSurfaceDeformableBodyDesc*>(desc.get());
            int64_t freq;
            if (impl.source.getAttribute(key,
                    impl.source.internToken("physxDeformableBody:collisionPairUpdateFrequency"), freq))
                surf->collisionPairUpdateFrequency = uint32_t(freq);
            int64_t mult;
            if (impl.source.getAttribute(key,
                    impl.source.internToken("physxDeformableBody:collisionIterationMultiplier"), mult))
                surf->collisionIterationMultiplier = uint32_t(mult);
        }
    }
    const parse::TokenId autoDBTok = impl.source.internToken("PhysxAutoDeformableBodyAPI");
    if (impl.source.hasSchema(key, autoDBTok))
    {
        impl.source.getAttribute(key,
            impl.source.internToken("physxDeformableBody:autoDeformableBodyEnabled"), desc->hasAutoAPI);
        std::vector<parse::ObjectKey> cookingTargets;
        impl.source.getRelationshipTargets(key,
            impl.source.internToken("physxDeformableBody:cookingSourceMesh"), cookingTargets);
        if (!cookingTargets.empty())
        {
            desc->cookingSrcMeshKey = cookingTargets[0];
            // Cooking-source mesh bind-pose: same DeformablePoseAPI
            // purpose walk used for the sim/collision/skin meshes above.
            // Captured here so the consumer no longer re-reads USD.
            const PXR_NS::SdfPath cookingPath = impl.source.pathFor(cookingTargets[0]);
            if (!cookingPath.IsEmpty() && impl.stage)
            {
                const PXR_NS::UsdPrim cookingPrim = impl.stage->GetPrimAtPath(cookingPath);
                if (cookingPrim)
                {
                    const PXR_NS::TfToken bp = getPoseNameFromPurpose(cookingPrim, kBindPose);
                    if (!bp.IsEmpty())
                        desc->cookingSrcMeshBindPoseToken = impl.source.internToken(bp.GetString());
                }
            }
        }

        const parse::TokenId simpAPITok = impl.source.internToken("PhysxAutoDeformableMeshSimplificationAPI");
        if (impl.source.hasSchema(key, simpAPITok))
        {
            auto rdU = [&](const char* name, uint32_t def) {
                int64_t iv;
                if (impl.source.getAttribute(key, impl.source.internToken(name), iv))
                    return uint32_t(iv);
                return def;
            };
            auto rdB = [&](const char* name, bool def) {
                bool v = def;
                impl.source.getAttribute(key, impl.source.internToken(name), v);
                return v;
            };
            desc->isAutoMeshSimplificationEnabled = rdB("physxDeformableBody:autoDeformableMeshSimplificationEnabled",
                                                        desc->isAutoMeshSimplificationEnabled);
            desc->isAutoRemeshingEnabled = rdB("physxDeformableBody:remeshingEnabled", desc->isAutoRemeshingEnabled);
            desc->autoRemeshingResolution = rdU("physxDeformableBody:remeshingResolution", desc->autoRemeshingResolution);
            desc->autoTriangleTargetCount = rdU("physxDeformableBody:targetTriangleCount", desc->autoTriangleTargetCount);
            desc->hasAutoForceConforming = rdB("physxDeformableBody:forceConforming", desc->hasAutoForceConforming);
        }
        if (!isSurface)
        {
            const parse::TokenId hexAPITok = impl.source.internToken("PhysxAutoDeformableHexahedralMeshAPI");
            if (impl.source.hasSchema(key, hexAPITok))
            {
                parse::PhysxVolumeDeformableBodyDesc* vol = static_cast<parse::PhysxVolumeDeformableBodyDesc*>(desc.get());
                vol->isAutoHexahedralMeshEnabled = true;
                int64_t iv;
                if (impl.source.getAttribute(key,
                        impl.source.internToken("physxDeformableBody:resolution"), iv))
                    vol->autoHexahedralResolution = uint32_t(iv);
            }
        }
    }

    // Record into walk-state for collider-is-deformable detection.
    walk.deformableBodyMap[key] = parsed.bodyEnabled;

    out.deformables.push_back(std::move(desc));
}

// ---------------------------------------------------------------------------
// Joint — emit. parse::parseJoint(ctx, key, info) handles the PhysX
// extension overlay (per-axis APIs, joint state, drive performance);
// the walker reads per-joint USD attrs (enabled, breakForce, per-type
// axis / limit / drive scalars), resolves body0/body1 via ancestor
// walk, and computes local poses relative to bodies. Result is pushed
// onto out.joints.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-JOINT-001
// @covers AC-1 AC-2 AC-3
// ---------------------------------------------------------------------------

constexpr float kJointSentinelLimit = 0.5e38f;

parse::Axis usdJointAxisToParse(const PXR_NS::TfToken& tok)
{
    if (tok == PXR_NS::UsdPhysicsTokens.Get()->y) return parse::eY;
    if (tok == PXR_NS::UsdPhysicsTokens.Get()->z) return parse::eZ;
    return parse::eX;
}

// Ancestor walk for joint body resolution. Returns the closest
// ancestor that has UsdPhysicsRigidBodyAPI applied; if none found,
// falls back to the outermost ancestor with UsdPhysicsCollisionAPI
// (legacy static-body semantics). Returns invalid prim when neither is found.
PXR_NS::UsdPrim getJointBodyPrim(PXR_NS::UsdStageWeakPtr stage,
                                 const PXR_NS::SdfPath& relPath,
                                 PXR_NS::UsdPrim& outRelPrim)
{
    PXR_NS::UsdPrim relPrim = stage->GetPrimAtPath(relPath);
    outRelPrim = relPrim;
    PXR_NS::UsdPrim collisionPrim;
    PXR_NS::UsdPrim parent = relPrim;
    while (parent && parent != stage->GetPseudoRoot())
    {
        if (parent.HasAPI<PXR_NS::UsdPhysicsRigidBodyAPI>())
            return parent;
        if (parent.HasAPI<PXR_NS::UsdPhysicsCollisionAPI>())
            collisionPrim = parent;
        parent = parent.GetParent();
    }
    return collisionPrim;
}

// Compute a joint-side local pose relative to the resolved body.
// When rel != body, transform the rel-frame anchor into the body's
// frame; either way bake the body's world scale into the translation
// (PhysX has no separate body scale). Returns the resolved body's
// prim path (empty when no body ancestor).
PXR_NS::SdfPath computeJointLocalPose(PXR_NS::UsdStageWeakPtr stage,
                                      PXR_NS::UsdGeomXformCache& xfCache,
                                      const PXR_NS::SdfPath& relPath,
                                      PXR_NS::GfVec3f& t, PXR_NS::GfQuatf& q)
{
    PXR_NS::UsdPrim relPrim;
    const PXR_NS::UsdPrim body = getJointBodyPrim(stage, relPath, relPrim);

    const PXR_NS::GfMatrix4d relationshipToWorld = xfCache.GetLocalToWorldTransform(relPrim);
    PXR_NS::GfMatrix4d bodyToWorld;
    if (body)
        bodyToWorld = (relPrim == body) ? relationshipToWorld : xfCache.GetLocalToWorldTransform(body);
    else
        bodyToWorld.SetIdentity();

    omni::physics::transformJointFrameToBody(relationshipToWorld, bodyToWorld, relPrim == body, t, q);

    return body ? body.GetPrimPath() : PXR_NS::SdfPath();
}

parse::ObjectType jointPrimTypeToParse(uint64_t typeBits, const PXR_NS::TfToken& primType)
{
    if (typeBits & PrimTypeBits::eUsdPhysicsRevoluteJoint)  return parse::eJointRevolute;
    if (typeBits & PrimTypeBits::eUsdPhysicsPrismaticJoint) return parse::eJointPrismatic;
    if (typeBits & PrimTypeBits::eUsdPhysicsSphericalJoint) return parse::eJointSpherical;
    if (typeBits & PrimTypeBits::eUsdPhysicsDistanceJoint)  return parse::eJointDistance;
    if (typeBits & PrimTypeBits::eUsdPhysicsFixedJoint)     return parse::eJointFixed;
    // Gear / rack-and-pinion are built-in PhysX joints: classify them as
    // first-class typed joints so parseJoint captures their data (gearRatio /
    // hinge / prismatic) into the typed descriptor, rather than the generic
    // custom path. Third-party registered custom joints still fall through to
    // eJointCustom for consumer-side specialization.
    static const PXR_NS::TfToken gGearJointType("PhysxPhysicsGearJoint");
    static const PXR_NS::TfToken gRackJointType("PhysxPhysicsRackAndPinionJoint");
    if (primType == gGearJointType)
        return parse::eJointGear;
    if (primType == gRackJointType)
        return parse::eJointRackAndPinion;
    if (isCustomJointToken(primType))
        return parse::eJointCustom;
    return parse::eJointD6;
}

// Read a UsdPhysicsLimitAPI multi-applicable instance into a
// JointLimitInfo.
parse::JointLimitInfo readLimitAPI(const PXR_NS::UsdPrim& prim,
                                   const PXR_NS::TfToken& instance)
{
    parse::JointLimitInfo out;
    PXR_NS::UsdPhysicsLimitAPI api = PXR_NS::UsdPhysicsLimitAPI::Get(prim, instance);
    if (!api) return out;
    api.GetLowAttr().Get(&out.lower);
    api.GetHighAttr().Get(&out.upper);
    if ((std::isfinite(out.lower) && out.lower > -kJointSentinelLimit) ||
        (std::isfinite(out.upper) && out.upper <  kJointSentinelLimit))
        out.enabled = true;
    return out;
}

// Read a UsdPhysicsDriveAPI multi-applicable instance into a
// JointDriveInfo.
parse::JointDriveInfo readDriveAPI(const PXR_NS::UsdPrim& prim,
                                   const PXR_NS::TfToken& instance)
{
    parse::JointDriveInfo out;
    PXR_NS::UsdPhysicsDriveAPI api = PXR_NS::UsdPhysicsDriveAPI::Get(prim, instance);
    if (!api) return out;
    api.GetTargetPositionAttr().Get(&out.targetPosition);
    api.GetTargetVelocityAttr().Get(&out.targetVelocity);
    api.GetMaxForceAttr().Get(&out.forceLimit);
    api.GetDampingAttr().Get(&out.damping);
    api.GetStiffnessAttr().Get(&out.stiffness);
    PXR_NS::TfToken typeToken;
    api.GetTypeAttr().Get(&typeToken);
    if (typeToken == PXR_NS::UsdPhysicsTokens->acceleration)
        out.acceleration = true;
    out.enabled = true;
    return out;
}

// Helper: read body0/body1 relationship targets off the joint prim.
PXR_NS::SdfPath readJointRel(const PXR_NS::UsdRelationship& rel)
{
    PXR_NS::SdfPathVector targets;
    rel.GetTargets(&targets);
    if (targets.empty()) return {};
    if (targets.size() > 1)
        CARB_LOG_WARN("Joint prim has relationships to multiple bodies; using first.");
    return targets[0];
}

void emitJoint(ScannedStage& out, UsdWalkCtx& impl,
               parse::ParseContext& ctx, WalkState& walk,
               parse::ObjectKey key, const PXR_NS::UsdPrim& jointPrim,
               uint64_t typeBits)
{
    if (!jointPrim.IsA<PXR_NS::UsdPhysicsJoint>())
        return;
    PXR_NS::UsdPhysicsJoint jp(jointPrim);

    parse::JointInfo info;
    info.type = jointPrimTypeToParse(typeBits, jointPrim.GetTypeName());

    // Base joint attrs.
    jp.GetJointEnabledAttr().Get(&info.jointEnabled);
    jp.GetCollisionEnabledAttr().Get(&info.collisionEnabled);
    jp.GetBreakForceAttr().Get(&info.breakForce);
    jp.GetBreakTorqueAttr().Get(&info.breakTorque);
    jp.GetExcludeFromArticulationAttr().Get(&info.excludeFromArticulation);

    // body0 / body1 rels (path-level) — resolved to ObjectKeys.
    const PXR_NS::SdfPath rel0Path = readJointRel(jp.GetBody0Rel());
    const PXR_NS::SdfPath rel1Path = readJointRel(jp.GetBody1Rel());

    // Validate rels exist.
    auto validRel = [&](const PXR_NS::SdfPath& p) {
        if (p.IsEmpty()) return true;
        const PXR_NS::UsdPrim relPrim = impl.stage->GetPrimAtPath(p);
        if (!relPrim)
        {
            CARB_LOG_ERROR("Joint (%s) body relationship %s points to a non existent prim; joint will not be created.",
                           jointPrim.GetPrimPath().GetText(), p.GetText());
            return false;
        }
        return true;
    };
    if (!validRel(rel0Path) || !validRel(rel1Path))
        return;

    info.rel0 = rel0Path.IsEmpty() ? parse::ObjectKey{} : impl.source.keyFor(rel0Path);
    info.rel1 = rel1Path.IsEmpty() ? parse::ObjectKey{} : impl.source.keyFor(rel1Path);

    // Per-type axis / limit / drive scalars.
    switch (info.type)
    {
    case parse::eJointRevolute:
    {
        PXR_NS::UsdPhysicsRevoluteJoint rj(jointPrim);
        PXR_NS::TfToken axis = PXR_NS::UsdPhysicsTokens->x;
        rj.GetAxisAttr().Get(&axis);
        info.axis = usdJointAxisToParse(axis);
        info.limit.enabled = false;
        rj.GetLowerLimitAttr().Get(&info.limit.lower);
        rj.GetUpperLimitAttr().Get(&info.limit.upper);
        if (std::isfinite(info.limit.lower) && std::isfinite(info.limit.upper) &&
            info.limit.lower > -kJointSentinelLimit && info.limit.upper < kJointSentinelLimit)
            info.limit.enabled = true;
        if (jointPrim.HasAPI<PXR_NS::UsdPhysicsDriveAPI>(PXR_NS::UsdPhysicsTokens->angular))
            info.drive = readDriveAPI(jointPrim, PXR_NS::UsdPhysicsTokens->angular);
        break;
    }
    case parse::eJointPrismatic:
    {
        PXR_NS::UsdPhysicsPrismaticJoint pj(jointPrim);
        PXR_NS::TfToken axis = PXR_NS::UsdPhysicsTokens->x;
        pj.GetAxisAttr().Get(&axis);
        info.axis = usdJointAxisToParse(axis);
        info.limit.enabled = false;
        pj.GetLowerLimitAttr().Get(&info.limit.lower);
        pj.GetUpperLimitAttr().Get(&info.limit.upper);
        if ((std::isfinite(info.limit.lower) && info.limit.lower > -kJointSentinelLimit) ||
            (std::isfinite(info.limit.upper) && info.limit.upper <  kJointSentinelLimit))
            info.limit.enabled = true;
        if (jointPrim.HasAPI<PXR_NS::UsdPhysicsDriveAPI>(PXR_NS::UsdPhysicsTokens->linear))
            info.drive = readDriveAPI(jointPrim, PXR_NS::UsdPhysicsTokens->linear);
        break;
    }
    case parse::eJointSpherical:
    {
        // Spherical limit is a cone angle pair; parse::JointInfo's
        // `limit.lower/upper` field aliases `angle0/angle1` in a union.
        PXR_NS::UsdPhysicsSphericalJoint sj(jointPrim);
        PXR_NS::TfToken axis = PXR_NS::UsdPhysicsTokens->x;
        sj.GetAxisAttr().Get(&axis);
        info.axis = usdJointAxisToParse(axis);
        float a0 = 0.0f, a1 = 0.0f;
        sj.GetConeAngle0LimitAttr().Get(&a0);
        sj.GetConeAngle1LimitAttr().Get(&a1);
        info.limit.lower = a0;
        info.limit.upper = a1;
        if (std::isfinite(a0) && std::isfinite(a1) && a0 >= 0.0f && a1 >= 0.0f)
            info.limit.enabled = true;
        break;
    }
    case parse::eJointDistance:
    {
        PXR_NS::UsdPhysicsDistanceJoint dj(jointPrim);
        float minD = -1.0f, maxD = -1.0f;
        dj.GetMinDistanceAttr().Get(&minD);
        dj.GetMaxDistanceAttr().Get(&maxD);
        info.limit.lower = minD;
        info.limit.upper = maxD;
        info.minEnabled = (minD >= 0.0f);
        info.maxEnabled = (maxD >= 0.0f);
        break;
    }
    case parse::eJointFixed:
        // No extra fields beyond the base.
        break;
    case parse::eJointD6:
    {
        // Iterate the 7 UsdPhysicsLimitAPI / UsdPhysicsDriveAPI instances
        // (distance / transX/Y/Z / rotX/Y/Z), record per-axis limits +
        // drives.
        static const std::pair<parse::JointAxis, const PXR_NS::TfToken*> kAxes[] = {
            { parse::eDistance, &PXR_NS::UsdPhysicsTokens->distance },
            { parse::eTransX,   &PXR_NS::UsdPhysicsTokens->transX   },
            { parse::eTransY,   &PXR_NS::UsdPhysicsTokens->transY   },
            { parse::eTransZ,   &PXR_NS::UsdPhysicsTokens->transZ   },
            { parse::eRotX,     &PXR_NS::UsdPhysicsTokens->rotX     },
            { parse::eRotY,     &PXR_NS::UsdPhysicsTokens->rotY     },
            { parse::eRotZ,     &PXR_NS::UsdPhysicsTokens->rotZ     },
        };
        for (const auto& [axisTag, token] : kAxes)
        {
            if (PXR_NS::UsdPhysicsLimitAPI::Get(jointPrim, *token))
            {
                parse::JointLimitInfo l = readLimitAPI(jointPrim, *token);
                info.jointLimits.emplace_back(axisTag, l);
            }
            if (PXR_NS::UsdPhysicsDriveAPI::Get(jointPrim, *token))
            {
                parse::JointDriveInfo d = readDriveAPI(jointPrim, *token);
                info.jointDrives.emplace_back(axisTag, d);
            }
        }
        break;
    }
    default:
        // Custom joints: parse-lib has no per-type reader path; leave
        // the JointInfo with type set + base attrs, no per-axis data.
        break;
    }

    // Frame computation — resolve body0/body1 from rel0/rel1 ancestor
    // walk and bake transforms into local poses.
    PXR_NS::GfVec3f t0(0.0f), t1(0.0f);
    PXR_NS::GfQuatf q0(1.0f), q1(1.0f);
    jp.GetLocalPos0Attr().Get(&t0);
    jp.GetLocalRot0Attr().Get(&q0);
    jp.GetLocalPos1Attr().Get(&t1);
    jp.GetLocalRot1Attr().Get(&q1);
    q0.Normalize(); q1.Normalize();

    PXR_NS::SdfPath body0Path, body1Path;
    if (!rel0Path.IsEmpty())
        body0Path = computeJointLocalPose(impl.stage, walk.xfCache, rel0Path, t0, q0);
    if (!rel1Path.IsEmpty())
        body1Path = computeJointLocalPose(impl.stage, walk.xfCache, rel1Path, t1, q1);

    info.body0 = body0Path.IsEmpty() ? parse::ObjectKey{} : impl.source.keyFor(body0Path);
    info.body1 = body1Path.IsEmpty() ? parse::ObjectKey{} : impl.source.keyFor(body1Path);
    info.localPose0Position    = toFloat3(t0);
    info.localPose0Orientation = toFloat4(q0);
    info.localPose1Position    = toFloat3(t1);
    info.localPose1Orientation = toFloat4(q1);

    // Hand off to parse-lib for PhysX-extension overlay + per-axis APIs.
    parse::DescPtr<parse::PhysxJointDesc> raw = parse::parseJoint(ctx, key, info);
    if (!raw)
        return;
    raw->jointPrimKey = key;

    // Record for pass-3 (observer pointer into the heap-owned descriptor
    // about to move into out.joints).
    WalkState::JointEntry entry;
    entry.desc = raw.get();
    entry.body0 = info.body0;
    entry.body1 = info.body1;
    entry.jointEnabled = info.jointEnabled;
    entry.excludeFromArticulation = info.excludeFromArticulation;
    entry.jointLimits = info.jointLimits;
    walk.jointMap[key] = entry;
    walk.jointOrder.push_back(key);

    out.joints.push_back(std::move(raw));
}

// ---------------------------------------------------------------------------
// Articulation — pass-1 record only. ArticulationRootAPI is a
// multi-applicable; the walker records the per-articulation extension
// fields here, and pass-3 (`finalizeArticulations`) runs the root
// election + body/joint aggregation algorithm to emit one
// PhysxArticulationDesc per resolved root.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-6 AC-7
//
// @implements REQ-PARSE-ART-002
// @covers AC-1 AC-2 AC-3 AC-4 AC-5
// ---------------------------------------------------------------------------

bool checkNestedArticulationRoot(PXR_NS::UsdStageWeakPtr stage,
                                 const PXR_NS::UsdPrim& usdPrim,
                                 const WalkState& walk,
                                 const UsdSource& source)
{
    PXR_NS::UsdPrim parent = usdPrim.GetParent();
    while (parent && parent != stage->GetPseudoRoot())
    {
        const parse::ObjectKey k = source.keyFor(parent.GetPrimPath());
        if (walk.articulationMap.find(k) != walk.articulationMap.end())
            return true;
        parent = parent.GetParent();
    }
    return false;
}

void recordArticulation(UsdWalkCtx& impl, parse::ParseContext& ctx,
                        WalkState& walk, parse::ObjectKey key,
                        const PXR_NS::UsdPrim& artPrim)
{
    if (checkNestedArticulationRoot(impl.stage, artPrim, walk, impl.source))
    {
        CARB_LOG_ERROR("UsdPhysics: Nested articulation roots are not allowed.");
        return;
    }

    WalkState::ArticulationEntry entry;
    parse::setToDefault(entry.fields, impl.source.getSourceUnits());
    parse::parseArticulation(ctx, key, entry.fields);
    entry.path = artPrim.GetPrimPath();
    entry.sourceFilteredCollisions = parse::parseFilteredPairs(ctx, key);

    walk.articulationMap[key] = std::move(entry);
    walk.articulationOrder.push_back(key);
}

// ---------------------------------------------------------------------------
// Pass-3: hand the recorded articulation roots + the scanned bodies/joints to
// the shared root-election / graph-aggregation algorithm
// (parse::buildArticulations, ArticulationGraph.h) — the SAME code the ovstage
// walker runs, so the graph computation lives in one place.
// ---------------------------------------------------------------------------

void finalizeArticulations(WalkState& walk, ScannedStage& out, UsdWalkCtx& impl, parse::ParseContext& ctx)
{
    if (walk.articulationMap.empty())
        return;

    std::vector<parse::ArticulationRootInput> roots;
    roots.reserve(walk.articulationOrder.size());
    for (const parse::ObjectKey& artKey : walk.articulationOrder)
    {
        const auto it = walk.articulationMap.find(artKey);
        if (it == walk.articulationMap.end())
            continue;
        parse::ArticulationRootInput in;
        in.key = artKey;
        in.fields = it->second.fields;
        in.sourceFilteredCollisions = it->second.sourceFilteredCollisions;
        roots.push_back(std::move(in));
    }

    parse::buildArticulations(impl.source, ctx.descriptorAllocator(), roots, out.bodies, out.joints,
                              out.articulations);
}

// ---------------------------------------------------------------------------
// Pass-2 finalization. For each (shapeKey, shapeDesc*) in
// WalkState::shapeList, walk parents to find the resolved body, set
// shape.rigidBody + body.sourceCollisions, and compute the local
// transform of the shape relative to the body.
// ---------------------------------------------------------------------------

// Walk up the prim hierarchy from `start`; return the first ancestor's
// ObjectKey that resolves to a body in `bodyMap` (enabled OR disabled).
// `outEnabled` is the body's rigidBodyEnabled bit. Returns invalid
// sentinel when no body ancestor exists.
parse::ObjectKey resolveBodyAncestor(WalkState& walk, UsdWalkCtx& impl,
                                     const PXR_NS::UsdPrim& shapePrim,
                                     bool& outEnabled)
{
    outEnabled = false;
    if (!impl.stage) return {};
    PXR_NS::UsdPrim p = shapePrim;
    while (p && p != impl.stage->GetPseudoRoot())
    {
        const parse::ObjectKey k = impl.source.keyFor(p.GetPrimPath());
        const auto it = walk.bodyMap.find(k);
        if (it != walk.bodyMap.end())
        {
            outEnabled = it->second.rigidBodyEnabled;
            return k;
        }
        p = p.GetParent();
    }
    return {};
}

// Mimic joints — pass-3. For each emitted joint, runs the parse-lib
// per-joint mimic readers (PhysxMimicJointAPI + NewtonMimicAPI). Joint
// type for the reference joint is resolved via a closure over
// `walk.jointMap`, so the parser doesn't need a "what kind of joint is
// this prim?" query on the source.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-MIMIC-001
// @covers AC-5
void finalizeMimicJoints(WalkState& walk, ScannedStage& out, parse::ParseContext& ctx)
{
    if (walk.jointMap.empty())
        return;

    parse::JointTypeLookup jointTypeOf = [&walk](parse::ObjectKey key) -> parse::ObjectType {
        const auto it = walk.jointMap.find(key);
        if (it == walk.jointMap.end() || !it->second.desc) return parse::eUndefined;
        return it->second.desc->type;
    };

    // Per-axis lock lookup over the scanned joint limits, so the mimic D6
    // validation can see locked reference-joint axes (the raw UsdPhysicsLimitAPI
    // attributes are not surfaced through the source for arbitrary joints).
    parse::JointLimitLookup jointLimitOf =
        [&walk](parse::ObjectKey key, parse::JointAxis axis) -> const parse::JointLimitInfo* {
        const auto it = walk.jointMap.find(key);
        if (it == walk.jointMap.end()) return nullptr;
        for (const auto& [ax, limit] : it->second.jointLimits)
            if (ax == axis) return &limit;
        return nullptr;
    };

    for (const parse::ObjectKey& jKey : walk.jointOrder)
    {
        const auto it = walk.jointMap.find(jKey);
        if (it == walk.jointMap.end() || !it->second.desc) continue;
        const WalkState::JointEntry& j = it->second;

        parse::MimicJointParseInfo info;
        info.jointType = j.desc->type;
        info.jointEnabled = j.jointEnabled;
        info.excludeFromArticulation = j.excludeFromArticulation;

        // Legacy per-joint-type gating (LoadStage.cpp joint-loop switch):
        // PhysX mimic only fires on prismatic / revolute / D6; Newton mimic
        // fires for those plus fixed / spherical / distance. Custom joints
        // (Gear / rack-and-pinion / third-party) are excluded from both -- see
        // ADR-0003.
        const bool physxMimicAllowed =
            info.jointType == parse::eJointRevolute  ||
            info.jointType == parse::eJointPrismatic ||
            info.jointType == parse::eJointD6;
        const bool newtonMimicAllowed = physxMimicAllowed ||
            info.jointType == parse::eJointFixed     ||
            info.jointType == parse::eJointSpherical ||
            info.jointType == parse::eJointDistance;

        if (physxMimicAllowed)
            parse::parseMimicJoints(ctx, jKey, info, jointTypeOf, jointLimitOf, out.mimicJoints);
        if (newtonMimicAllowed)
            parse::parseNewtonMimicJoints(ctx, jKey, info, jointTypeOf, out.mimicJoints);
    }
}

// Spatial tendon attachments — pass-3c. Iterates emitted rigid bodies
// and, for each, calls `parseSpatialTendons` which emits 0..N attachment
// descriptors per multi-apply tendon API instance. Link world-scale
// (used to bake into authored localPos) is taken from the body
// descriptor's already-resolved `scale` so we don't re-walk USD.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-TENDON-001
// @covers AC-5
// Fixed tendons — pass-3d. Per-joint iteration of
// PhysxTendonAxisRootAPI:<inst> + PhysxTendonAxisAPI:<inst>. Joint kind
// and the joint's body0/body1 ObjectKeys come pre-resolved off
// `walk.jointMap`; the parser uses them to populate
// `PhysxTendonAxisDesc::link0/link1` and reject non-{revolute,prismatic}
// joints with a warning.
//
// @implements REQ-PARSE-SCAN-001
// @covers AC-4 AC-7
//
// @implements REQ-PARSE-TENDON-002
// @covers AC-5
void finalizeFixedTendons(WalkState& walk, ScannedStage& out, parse::ParseContext& ctx)
{
    for (const parse::ObjectKey& jKey : walk.jointOrder)
    {
        const auto it = walk.jointMap.find(jKey);
        if (it == walk.jointMap.end() || !it->second.desc) continue;
        const WalkState::JointEntry& j = it->second;

        parse::FixedTendonParseInfo info;
        info.jointType = j.desc->type;
        info.body0     = j.body0;
        info.body1     = j.body1;

        parse::parseFixedTendons(ctx, jKey, info, out.fixedTendonAxes, out.fixedTendons);
    }
}

void finalizeSpatialTendons(WalkState& walk, ScannedStage& out, parse::ParseContext& ctx)
{
    for (const auto& bodyEntry : walk.bodyMap)
    {
        const parse::ObjectKey bodyKey = bodyEntry.first;
        if (!bodyEntry.second.desc) continue;
        parse::SpatialTendonParseInfo info;
        info.linkWorldScale = bodyEntry.second.desc->scale;
        parse::parseSpatialTendons(ctx, bodyKey, info, out.spatialTendonAttachments);
    }
}

void finalizeBodiesAndShapes(WalkState& walk, UsdWalkCtx& impl)
{
    if (!impl.stage) return;
    for (auto& entry : walk.shapeList)
    {
        const parse::ObjectKey shapeKey = entry.first;
        parse::PhysxShapeDesc* desc     = entry.second;

        const PXR_NS::SdfPath shapeSdfPath = impl.source.pathFor(shapeKey);
        const PXR_NS::UsdPrim shapePrim = impl.stage->GetPrimAtPath(shapeSdfPath);
        if (!shapePrim) continue;

        // Body-of-shape ancestor walk. Returns invalid key when no
        // RigidBody / CollisionAPI ancestor was found at all. When
        // the first body-API ancestor is disabled (rigidBodyEnabled=
        // false), the path is still returned — that's the static-body
        // case.
        bool enabled = false;
        const parse::ObjectKey bodyKey = resolveBodyAncestor(walk, impl, shapePrim, enabled);
        if (bodyKey.valid())
        {
            desc->rigidBody = bodyKey;
            auto it = walk.bodyMap.find(bodyKey);
            if (it != walk.bodyMap.end() && it->second.desc)
                it->second.desc->sourceCollisions.insert(shapeKey);
        }

        // finalizeCollision: compute local transform of shape relative
        // to body (or pseudoRoot when no body). Skipped for instance
        // proxies — emitShape's instance-proxy branch already baked
        // their localPos/localScale from the instance world transform.
        if (shapePrim.IsInstanceProxy())
            continue;

        PXR_NS::UsdPrim bodyPrim;
        if (bodyKey.valid())
        {
            const PXR_NS::SdfPath bodyPath = impl.source.pathFor(bodyKey);
            bodyPrim = impl.stage->GetPrimAtPath(bodyPath);
        }
        if (!bodyPrim)
            bodyPrim = impl.stage->GetPseudoRoot();

        // For Xform-collider hierarchies the shape's geometry lives on a
        // descendant gprim (desc->sourceGprim) while shapeKey points at the
        // collider Xform. Use the gprim's prim for the local-transform
        // source so the shape's localPose reflects the gprim's pose, not
        // the collider's.
        PXR_NS::UsdPrim transformSrcPrim = shapePrim;
        if (desc->sourceGprim.valid() && desc->sourceGprim != shapeKey)
        {
            const PXR_NS::SdfPath gprimPath = impl.source.pathFor(desc->sourceGprim);
            const PXR_NS::UsdPrim gprimPrim = impl.stage->GetPrimAtPath(gprimPath);
            if (gprimPrim)
                transformSrcPrim = gprimPrim;
        }

        PXR_NS::GfVec3f localPos(0.0f);
        PXR_NS::GfQuatf localRot(1.0f);
        PXR_NS::GfVec3f localScale(1.0f);
        PXR_NS::GfMatrix4d shapeToBody(1.0);
        if (transformSrcPrim != bodyPrim)
        {
            bool resetXformStack = false;
            shapeToBody =
                walk.xfCache.ComputeRelativeTransform(transformSrcPrim, bodyPrim, &resetXformStack);
        }
        // Matrix sourcing remains specific to the native walker.
        omni::physics::decomposeCollisionShapeLocalTransform(
            shapeToBody, walk.xfCache.GetLocalToWorldTransform(bodyPrim), localPos, localRot, localScale);

        desc->localPos   = toFloat3(localPos);
        desc->localRot   = toFloat4(localRot);
        desc->localScale = toFloat3(localScale);
    }
}

} // namespace

// Resolve a prim's bound UsdShadeMaterial path for the "physics" purpose.
// Inlined here so the backend doesn't depend on common/; verbatim mirror of
// usdmaterialutils::getMaterialBinding (see NativeWalker.h).
PXR_NS::SdfPath getMaterialBindingPath(const PXR_NS::UsdPrim& usdPrim)
{
    static const PXR_NS::TfToken physicsPurpose("physics");
    PXR_NS::SdfPath materialKey;
    PXR_NS::UsdShadeMaterialBindingAPI bindingAPI(usdPrim);
    if (bindingAPI)
    {
        PXR_NS::UsdShadeMaterial material = bindingAPI.ComputeBoundMaterial(physicsPurpose);
        if (material)
            materialKey = material.GetPrim().GetPrimPath();
    }
    else
    {
        std::vector<PXR_NS::UsdPrim> prims{ usdPrim };
        std::vector<PXR_NS::UsdShadeMaterial> materials =
            PXR_NS::UsdShadeMaterialBindingAPI::ComputeBoundMaterials(prims, physicsPurpose);
        if (!materials.empty() && materials[0])
            materialKey = materials[0].GetPrim().GetPrimPath();
    }
    return materialKey;
}

ScannedStage scanStageNative(PXR_NS::UsdStageWeakPtr stage,
                             omni::physics::schema::PrimIteratorBase& primIterator,
                             parse::IDescriptorAllocator& allocator)
{
    if (!stage)
        return {};

    // The walk drives a concrete UsdSource (it needs UsdSource-specific
    // resolvers like keyFor(SdfPath)); the scan owns it type-erased as the
    // backend `IPhysicsSource`. Moving the unique_ptr does not move the
    // pointee, so `usdSource` stays valid for the walk.
    std::unique_ptr<UsdSource> usdPtr = std::make_unique<UsdSource>(stage);
    UsdSource& usdSource = *usdPtr;
    ScannedStage out{ parse::makeScannedStageFromSource(std::move(usdPtr)) };
    UsdWalkCtx impl{ usdSource, stage, out };
    parse::ParseContext ctx(usdSource, allocator);
    WalkState walk;

    // Mass scale (1 / kilogramsPerUnit) used by parse-lib vehicle
    // component parsers (Tire's deprecated longitudinalStiffnessPerUnit-
    // Gravity default). SourceUnits doesn't carry kg yet; resolve once
    // from the stage's USD metric.
    const double kilogramsPerUnit = PXR_NS::UsdPhysicsGetStageKilogramsPerUnit(stage);
    const double metersPerUnit    = PXR_NS::UsdGeomGetStageMetersPerUnit(stage);
    const float massScale   = 1.0f / static_cast<float>(kilogramsPerUnit);
    const float lengthScale = 1.0f / static_cast<float>(metersPerUnit);
    const float kgmsScale   = (lengthScale * lengthScale) * massScale;

    primIterator.reset();
    while (!primIterator.atEnd())
    {
        const PXR_NS::UsdPrim& prim = *primIterator.getCurrent();
        if (!prim)
        {
            primIterator.pruneChildren();
            primIterator.next();
            continue;
        }

        const PXR_NS::UsdPrimTypeInfo& typeInfo = prim.GetPrimTypeInfo();
        const uint64_t typeBits = classifyPrimType(typeInfo.GetSchemaType());
        const PXR_NS::TfTokenVector apis = typeInfo.GetAppliedAPISchemas();
        const uint64_t apiFlags = classifyApiSchemas(apis);

        // PointInstancer subtrees and custom PointInstancer-shaped
        // prims (e.g. PhysxPhysicsJointInstancer, registered via
        // registerCustomPhysicsInstancerToken) are parsed by the
        // consumer on its own scanStage call. Prune here.
        if ((typeBits & PrimTypeBits::eUsdGeomPointInstancer) ||
            isCustomPhysicsInstancerToken(prim.GetTypeName()))
        {
            out.hasPointInstancerPrims = true;
            primIterator.pruneChildren();
        }

        const parse::ObjectKey key = impl.source.keyFor(prim.GetPrimPath());

        // Dispatch precedence: Scene → CollisionGroup → MaterialAPI →
        // DeformableMaterialAPI → Attachment → ElementCollisionFilter
        // → else-branch (Body / Shape / DeformableBody / Joint /
        // Articulation). Order is significant — Material APIs combine
        // with Rigid/Deformable APIs follow the "material drop" rule.
        if (typeBits & PrimTypeBits::eUsdPhysicsScene)
        {
            emitScene(out, impl, ctx, key, prim);
            // PhysxVehicleContextAPI may be applied alongside the
            // scene prim.
            if (apiFlags & ApiFlag::eVehicleContextAPI)
                emitVehicleContext(out, ctx, key, prim);
        }
        else if (typeBits & PrimTypeBits::eUsdPhysicsCollisionGroup)
        {
            emitCollisionGroup(out, ctx, key);
        }
        else if (typeBits & PrimTypeBits::ePhysxVehicleTireFrictionTable)
        {
            emitTireFrictionTable(out, impl, ctx, key, prim);
        }
        else if (typeBits & PrimTypeBits::ePhysxParticleSystem)
        {
            emitParticleSystem(out, ctx, key);
        }
        else if (apiFlags & ApiFlag::eParticleSetAPI)
        {
            emitParticleSet(out, ctx, key);
        }
        else if (apiFlags & ApiFlag::eParticleSamplingAPI)
        {
            emitParticleSampler(out, ctx, key);
        }
        else if (apiFlags & (ApiFlag::eMaterialAPI | ApiFlag::ePBDMaterialAPI |
                              ApiFlag::eDeformableMaterialAPI | ApiFlag::eSurfaceDeformableMaterialAPI |
                              ApiFlag::eCurvesDeformableMaterialAPI))
        {
            if (apiFlags & ApiFlag::eMaterialAPI)
                emitMaterial(out, ctx, key);
            if (apiFlags & ApiFlag::ePBDMaterialAPI)
                emitPBDMaterial(out, ctx, key);
            if (apiFlags & (ApiFlag::eDeformableMaterialAPI | ApiFlag::eSurfaceDeformableMaterialAPI |
                            ApiFlag::eCurvesDeformableMaterialAPI))
                emitDeformableMaterial(out, ctx, key, apiFlags);
        }
        else if (typeBits & PrimTypeBits::eUsdPhysicsAttachment)
        {
            emitAttachment(out, ctx, key, prim);
        }
        else if (typeBits & PrimTypeBits::eUsdPhysicsElementCollisionFilter)
        {
            emitElementCollisionFilter(out, ctx, key);
        }
        else
        {
            // Character controller — independent of the body/shape branches
            // below. Lives on a UsdGeomCapsule with PhysxCharacterControllerAPI
            // applied; emitCct guards the geometry-type check.
            if (apiFlags & ApiFlag::eCharacterControllerAPI)
            {
                emitCct(out, impl, ctx, key, prim);
            }
            // CollisionAPI (rigid colliders only — deformable colliders
            // are skipped via isDeformableCollider check inside emitShape).
            if ((apiFlags & ApiFlag::eCollisionAPI) && !(apiFlags & ApiFlag::eDeformableBodyAPI))
            {
                // Shapes for instance-proxies emit with baked transforms
                // handled inside emitShape.
                emitShape(out, impl, ctx, walk, key, prim, apis, typeBits);
            }
            if (apiFlags & ApiFlag::eRigidBodyAPI)
            {
                // Instance-proxy with enabled non-kinematic RigidBodyAPI
                // is unsupported (PhysX can't share a body across
                // instances). Kinematic or disabled bodies are fine.
                bool reportInstanceError = false;
                if (prim.IsInstanceProxy())
                {
                    reportInstanceError = true;
                    PXR_NS::UsdPhysicsRigidBodyAPI rbAPI(prim);
                    bool kinematic = false, enabled = true;
                    rbAPI.GetKinematicEnabledAttr().Get(&kinematic);
                    rbAPI.GetRigidBodyEnabledAttr().Get(&enabled);
                    if (kinematic || !enabled)
                        reportInstanceError = false;
                    if (reportInstanceError)
                        CARB_LOG_WARN("RigidBodyAPI on an instance proxy not supported, "
                                      "unless set to kinematic or not enabled. %s",
                                      prim.GetPrimPath().GetText());
                }
                if (!reportInstanceError)
                    emitRigidBody(out, impl, ctx, walk, key, prim, typeBits);
            }
            if (apiFlags & ApiFlag::eDeformableBodyAPI)
            {
                bool reportInstanceError = false;
                if (prim.IsInstanceProxy())
                {
                    reportInstanceError = true;
                    const parse::TokenId enabledTok = impl.source.internToken("omniphysics:deformableBodyEnabled");
                    bool enabled = true;
                    if (impl.source.getAttribute(key, enabledTok, enabled) && !enabled)
                        reportInstanceError = false;
                    if (reportInstanceError)
                        CARB_LOG_WARN("DeformableBodyAPI on an instance proxy not supported, "
                                      "unless not enabled. %s",
                                      prim.GetPrimPath().GetText());
                }
                if (!reportInstanceError)
                    emitDeformableBody(out, impl, ctx, walk, key, prim, typeBits);
            }
            // Joints — emit per-joint, record into walk.jointMap for
            // pass-3 articulation aggregation.
            if (typeBits & PrimTypeBits::eUsdPhysicsJoint)
            {
                emitJoint(out, impl, ctx, walk, key, prim, typeBits);
            }
            // Articulations — ArticulationRootAPI applied. Record
            // only; pass-3 runs root election + body/joint aggregation
            // and emits one PhysxArticulationDesc per resolved root.
            if (apiFlags & ApiFlag::eArticulationRootAPI)
            {
                recordArticulation(impl, ctx, walk, key, prim);
            }
            // Vehicle shareable components. Wheel / Tire / Suspension
            // APIs can be applied on any prim; the consumer adapter
            // dedups across vehicles by SdfPath.
            if (apiFlags & ApiFlag::eVehicleWheelAPI)
            {
                emitVehicleWheel(out, ctx, key);
            }
            if (apiFlags & ApiFlag::eVehicleTireAPI)
            {
                emitVehicleTire(out, impl, ctx, key, prim, massScale);
            }
            if (apiFlags & ApiFlag::eVehicleSuspensionAPI)
            {
                emitVehicleSuspension(out, ctx, key);
            }
            // Vehicle drivetrain components. Engine / Gears / Clutch
            // APIs can be applied on any prim; the consumer adapter
            // dedups across vehicles by SdfPath. AutoGearBox is
            // emitted in the drive group below because of its
            // parent-Gears ratio-count dependency.
            if (apiFlags & ApiFlag::eVehicleEngineAPI)
            {
                emitVehicleEngine(out, ctx, key, prim, kgmsScale);
            }
            if (apiFlags & ApiFlag::eVehicleGearsAPI)
            {
                emitVehicleGears(out, ctx, key, prim);
            }
            if (apiFlags & ApiFlag::eVehicleClutchAPI)
            {
                emitVehicleClutch(out, ctx, key, kgmsScale);
            }
            // Vehicle drive + differential + auto-gear-box.
            // Tank differential subclasses MultiWheel; the schema allows
            // both APIs to be applied together on a tank-prim. When
            // both are present we emit only the Tank descriptor (it
            // carries the MultiWheel fields too).
            if (apiFlags & ApiFlag::eVehicleDriveBasicAPI)
            {
                emitVehicleDriveBasic(out, ctx, key, kgmsScale);
            }
            if (apiFlags & ApiFlag::eVehicleDriveStandardAPI)
            {
                emitVehicleDriveStandard(out, impl, ctx, key, prim);
            }
            if (apiFlags & ApiFlag::eVehicleTankDifferentialAPI)
            {
                emitVehicleTankDifferential(out, ctx, key, prim);
            }
            else if (apiFlags & ApiFlag::eVehicleMultiWheelDifferentialAPI)
            {
                emitVehicleMultiWheelDifferential(out, ctx, key, prim);
            }
            if (apiFlags & ApiFlag::eVehicleAutoGearBoxAPI)
            {
                emitVehicleAutoGearBox(out, ctx, key, prim);
            }
            // Vehicle brakes (multi-apply) + steering variants.
            // SteeringAPI and AckermannSteeringAPI are single-apply and
            // mutually exclusive. Brakes API is multi-apply; walker
            // enumerates instances internally.
            if (apiFlags & ApiFlag::eVehicleBrakesAPI)
            {
                emitVehicleBrakes(out, impl, ctx, key, prim);
            }
            if (apiFlags & ApiFlag::eVehicleSteeringAPI)
            {
                emitVehicleSteeringBasic(out, ctx, key, prim);
            }
            else if (apiFlags & ApiFlag::eVehicleAckermannSteeringAPI)
            {
                emitVehicleSteeringAckermann(out, ctx, key);
            }
            // NonlinearCmdResponse (multi-apply per command). Walker
            // emits per instance; consumer adapter wires onto the
            // matching Drive / Steering / Brakes descriptor at
            // pre-population time.
            if (apiFlags & ApiFlag::eVehicleNonlinearCmdResponseAPI)
            {
                emitVehicleNonlinearCmdResponse(out, impl, ctx, key, prim);
            }
            // WheelAttachment + SuspensionCompliance. Per-prim;
            // SuspensionCompliance is applied alongside on the same
            // wheel-attachment prim.
            if (apiFlags & ApiFlag::eVehicleWheelAttachmentAPI)
            {
                emitVehicleWheelAttachment(out, impl, ctx, key, prim);
            }
            if (apiFlags & ApiFlag::eVehicleSuspensionComplianceAPI)
            {
                emitVehicleSuspensionCompliance(out, ctx, key, prim);
            }
            // Vehicle chassis root. The vehicle prim is a rigid body
            // chassis with PhysxVehicleAPI applied. Walker emits the
            // chassis-scalar data; consumer adapter wires the rest.
            if (apiFlags & ApiFlag::eVehicleAPI)
            {
                emitVehicle(out, walk, ctx, key, prim, lengthScale);
            }
        }

        primIterator.next();
    }

    // Pass-2: shape ancestor walk + finalizeCollision.
    finalizeBodiesAndShapes(walk, impl);

    // Pass-3: articulation root election + body/joint aggregation,
    // emit one PhysxArticulationDesc per resolved rootPrim.
    finalizeArticulations(walk, out, impl, ctx);

    // Pass-3b: mimic joints. Runs after all joints are in out.joints so
    // the reference-joint type lookup has the full set available.
    finalizeMimicJoints(walk, out, ctx);

    // Pass-3c: spatial tendon attachments. Per-body iteration of
    // PhysxTendonAttachment*API multi-apply schemas; flat emit list
    // (root + intermediate + leaf), hierarchy resolved consumer-side.
    finalizeSpatialTendons(walk, out, ctx);

    // Pass-3d: fixed tendons. Per-joint iteration of
    // PhysxTendonAxis(Root)API multi-apply schemas; axes + tendons
    // emitted into separate lists, cross-referenced consumer-side
    // by matching jointKey + instanceToken.
    finalizeFixedTendons(walk, out, ctx);

    return out;
}

} // namespace omni::physics::usd

// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-9
 *
 * @implements REQ-PARSE-SHAPE-001
 * @covers AC-5
 *
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-6
 */

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <carb/tasking/ITasking.h>
#include <carb/profiler/Profile.h>
#include <carb/extras/Timer.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/Utilities.h>
#include <common/utilities/PrimUtilities.h>
#include <common/foundation/Algorithms.h>

#include "CookingDataAsync.h"
#include "MeshCache.h"

#include <private/omni/physx/IPhysxCookingServicePrivate.h>

#include "OmniPhysX.h"                  // Notice Handler
#include "PhysXFoundation.h"            // foundation::getInterface (CPU-only / CUDA availability)
#include "PhysXDebugVisualization.h"    // CollisionRepresentation
#include <PhysXTools.h>                 // internal::getWorldTransform
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Collision.h"
#include "usdLoad/DeformableBodyConverter.h"
#include "usdLoad/PhysicsBody.h"

#include <omni/physics/usd/StageScan.h>

// Source-agnostic output sink: the cooked-mesh write-back routes its array
// authoring through AttachedStage's IPhysicsDataWrite instead of direct USD Set().
#include <UsdPhysicsDataWrite.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/IChangeFeed.h>

#include "usdLoad/IceDescriptorAllocator.h"

#include "particles/PhysXParticleSampling.h"

#include <regex>

// Set this to 1 to enable the USD notice listener to start async tasks
#define USD_USD_NOTICE_LISTENER 1 // Listens for collision related attribute changes to automatically spawn background cooking tasks if needed
#define USE_ASYNC_COOKING 1 // By default we support asynchronous cooking, this is only here for debugging purposes
#define REPORT_PROGRESS 0 // A debug option only, used to track cooking task progress

static constexpr const char* PROGRESS_BAR_ENABLED = "/physics/progressBarEnabled";
static constexpr const char* PROGRESS_BAR_LABEL = "/physics/progressBarLabel";
static constexpr const char* PROGRESS_BAR_VALUE = "/physics/progressBarValue";

using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;
using namespace ::physx;
using namespace PXR_NS;

namespace
{
    // Source-keyed equivalent of the legacy prim-based findDeformableBodyAncestor:
    // walk parents until a DeformableBodyAPI carrier is found; stop (returning the
    // invalid key) at a RigidBodyAPI or a reset-xform-stack boundary. All reads go
    // through IPhysicsSource — schema gates via hasSchema/hasAppliedSchema, the
    // reset boundary via getLocalTransform's outResetsXformStack (the runtime
    // equivalent of UsdGeomXformable::GetResetXformStack), and the walk via
    // getParent (which yields getRootKey() for top-level objects).
    omni::physics::parse::ObjectKey findDeformableBodyAncestorKey(
        const omni::physics::parse::IPhysicsSource& src,
        omni::physics::parse::ObjectKey key,
        omni::physics::parse::TokenId deformableBodyToken)
    {
        const omni::physics::parse::ObjectKey root = src.getRootKey();
        while (key.valid() && key != root)
        {
            if (src.hasSchema(key, deformableBodyToken))
                return key;
            if (omni::physx::internal::hasAppliedSchema<UsdPhysicsRigidBodyAPI>(src, key))
                return {};
            omni::physics::parse::Matrix4d local;
            bool resetXformStack = false;
            src.getLocalTransform(key, omni::physics::parse::ReadTime::defaultTime(), local, resetXformStack);
            if (resetXformStack)
                return {};
            key = src.getParent(key);
        }
        return {};
    }

    PXR_NS::GfVec3d computeQuantizedDir(const PXR_NS::GfVec3d& dir)
    {
        double eps = 1e-7;
        double eps_inv = 1.0 / eps;

        PXR_NS::GfVec3d dirQuant(
            std::round(dir[0] * eps_inv) * eps, std::round(dir[1] * eps_inv) * eps, std::round(dir[2] * eps_inv) * eps);

        return dirQuant.GetNormalized();
    }

    PXR_NS::GfRotation computeQuantizedRotation(const PXR_NS::GfRotation& rotation)
    {
        double eps = 1e-7;
        double eps_inv = 1.0 / eps;

        double angleUnit = rotation.GetAngle() / 360.0;
        double angleUnitQuant = std::round(angleUnit * eps_inv) * eps;
        double angleQuant = angleUnitQuant * 360.0;

        return PXR_NS::GfRotation(computeQuantizedDir(rotation.GetAxis()), angleQuant);
    }

    PXR_NS::GfTransform computeQuantizedSkewTransform(double& scaleAbs, const PXR_NS::GfTransform& transformSkew)
    {
        PXR_NS::GfVec3d scaleNormalized = transformSkew.GetScale();
        scaleAbs = scaleNormalized.Normalize();

        PXR_NS::GfTransform transformSkewQuant(PXR_NS::GfVec3d(0.0), computeQuantizedRotation(transformSkew.GetRotation()),
                                            computeQuantizedDir(scaleNormalized), PXR_NS::GfVec3d(0.0),
                                            computeQuantizedRotation(transformSkew.GetPivotOrientation()));

        return transformSkewQuant;
    }

    void computeFitBounds(PXR_NS::GfVec3d& translation,
                          double& scale,
                          const PXR_NS::VtArray<PXR_NS::GfVec3f>& points,
                          const PXR_NS::GfTransform& transform)
    {
        PXR_NS::GfMatrix4d m = transform.GetMatrix();

        PXR_NS::GfRange3d bounds;
        for (const PXR_NS::GfVec3d& point : points)
        {
            bounds.UnionWith(m.Transform(point));
        }
        PXR_NS::GfVec3d dims = bounds.GetSize();
        double dimMax = std::max(std::max(dims[0], dims[1]), dims[2]);

        translation = bounds.GetMidpoint();
        scale = std::max(1e-7, dimMax);
    }

    bool computeDeformableCookingTransform(GfMatrix4d* simToCookingTransform,
                                           GfMatrix4d* cookingToWorldTransform,
                                           double* cookingToWorldScale,
                                           const GfMatrix4d& simToWorld,
                                           const VtArray<GfVec3f>& boundsFitPoints)
    {
        PXR_NS::GfMatrix4d simToWorldOrtho = simToWorld;
        bool orthonormalized = simToWorldOrtho.Orthonormalize(false);

        if (!orthonormalized)
        {
            return false;
        }

        PXR_NS::GfMatrix4d simToWorldSkew = simToWorld * simToWorldOrtho.GetInverse();

        double scaleAbs;
        PXR_NS::GfTransform skew(simToWorldSkew);
        PXR_NS::GfTransform skewQuant = computeQuantizedSkewTransform(scaleAbs, skew);

        PXR_NS::GfVec3d fbTrans;
        double fbScale;
        computeFitBounds(fbTrans, fbScale, boundsFitPoints, skewQuant);

        PXR_NS::GfRotation pivotOrient = skewQuant.GetPivotOrientation();
        PXR_NS::GfRotation rotation = skewQuant.GetRotation();
        PXR_NS::GfVec3d scale = skewQuant.GetScale();

        if (simToCookingTransform)
        {
            PXR_NS::GfMatrix4d matFbScaleInv;
            matFbScaleInv.SetScale(1.0 / fbScale);
            PXR_NS::GfMatrix4d matFbTransInv;
            matFbTransInv.SetTranslate(-fbTrans);
            PXR_NS::GfMatrix4d matScale;
            matScale.SetScale(scale);
            PXR_NS::GfMatrix4d matOrientInv;
            matOrientInv.SetTransform(pivotOrient.GetInverse(), PXR_NS::GfVec3d(0.0));
            *simToCookingTransform = matOrientInv * matScale * matFbTransInv * matFbScaleInv;
        }

        if (cookingToWorldTransform)
        {
            // this is the corresponding transform from cooking space to world space (ignoring the pre scale factor:
            // scaleAbs*fbScale)
            PXR_NS::GfMatrix4d rigid;
            {
                PXR_NS::GfMatrix4d matRot(pivotOrient * rotation, PXR_NS::GfVec3d(0.0));
                PXR_NS::GfMatrix4d matTrans(PXR_NS::GfRotation(PXR_NS::GfVec3d(1, 0, 0), 0.0), fbTrans * scaleAbs);
                rigid = matTrans * matRot * simToWorldOrtho;
            }
            *cookingToWorldTransform = rigid;
        }

        if (cookingToWorldScale)
        {
            *cookingToWorldScale = scaleAbs * fbScale;
        }
        return true;
    }

    /**
     * Compute mesh key based on custom mesh arrays
     */
    omni::physx::usdparser::MeshKey computeMeshKey(const PXR_NS::VtArray<PXR_NS::GfVec3f>& points,
                                                   const PXR_NS::VtArray<int32_t>& vertexIndices,
                                                   const PXR_NS::VtArray<int32_t>& vertexCounts)
    {
        omni::physx::usdparser::MeshKey meshKey;
        meshKey.computeVerticesHash(uint32_t(points.size()), reinterpret_cast<const float*>(points.data()));
        meshKey.computeIndicesHash(
            uint32_t(vertexIndices.size()), reinterpret_cast<const uint32_t*>(vertexIndices.data()));
        meshKey.computeIndicesHash(uint32_t(vertexCounts.size()), reinterpret_cast<const uint32_t*>(vertexCounts.data()));
        return meshKey;
    }

    // Signature binding (simPoints, simIndices, numTetsPerElement) for hex sim meshes,
    // versioned with the producer's (volume-deformable-body) cooking version.
    omni::physx::usdparser::MeshKey computeSimMeshHexCrc(const PXR_NS::VtArray<PXR_NS::GfVec3f>& simPoints,
                                                         const PXR_NS::VtArray<PXR_NS::GfVec4i>& simIndices,
                                                         uint32_t numTetsPerElement)
    {
        omni::physx::usdparser::MeshKey meshKey;
        meshKey.computeVerticesHash(uint32_t(simPoints.size()), reinterpret_cast<const float*>(simPoints.data()));
        meshKey.computeIndicesHash(uint32_t(simIndices.size()) * 4, reinterpret_cast<const uint32_t*>(simIndices.data()));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&numTetsPerElement), sizeof(numTetsPerElement));
        constexpr int version = omni::physx::PhysxCookingDataVersion_VolumeDeformableBody;
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&version), sizeof(version));
        return meshKey;
    }

    // All cooking-input reads/gates below are keyed by the object's USD path and
    // routed through AttachedStage's IPhysicsSource (the normal cooking-pump case,
    // run main-thread inside ScopedBlockUSDUpdates) — no UsdPrim is materialized.
    // Cooking only runs with an attached source, so a null AttachedStage yields the
    // empty result (identity / false / no read).

    /**
     * Local-to-world transform of the object at `primPath`, read through the source.
     */
    GfMatrix4d cookingWorldTransform(const omni::physx::usdparser::AttachedStage* attachedStage, const SdfPath& primPath)
    {
        return attachedStage ?
            omni::physx::internal::getWorldTransform(*attachedStage, attachedStage->keyFor(primPath), UsdTimeCode::Default()) :
            GfMatrix4d(1.0);
    }

    // Source-routed cooking-input geometry read: route an array attribute through
    // AttachedStage's IPhysicsSource, keyed by path.
    template <typename T>
    bool cookingReadArray(const omni::physx::usdparser::AttachedStage* attachedStage,
                          const SdfPath& primPath,
                          const TfToken& attrName,
                          T& out)
    {
        return attachedStage ?
            omni::physx::internal::getArrayValue(*attachedStage, primPath, attrName, UsdTimeCode::Default(), out) :
            false;
    }

    // Source-routed bind-pose point read for a deformable mesh: when
    // bindPoseToken is set, read the OmniPhysicsDeformablePoseAPI instance's
    // points attribute; otherwise read the plain points attribute. A non-empty
    // bindPoseToken guarantees the pose API instance is applied (the parser only
    // emits it then), so no HasAPI re-check is needed. Returns whether geometry
    // was read.
    bool cookingReadBindPoints(const omni::physx::usdparser::AttachedStage* attachedStage,
                               const SdfPath& primPath,
                               const TfToken& bindPoseToken,
                               VtArray<GfVec3f>& out)
    {
        const TfToken attrName = bindPoseToken.IsEmpty() ?
            UsdGeomTokens->points :
            UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, bindPoseToken);
        return cookingReadArray(attachedStage, primPath, attrName, out);
    }

    // Source-routed read of a stored MeshKey CRC blob (authored as a UCharArray
    // marker by storeMeshKey). Mirrors usdparser::loadMeshKey but routes through
    // the source by path — no UsdPrim. Returns true and fills `out` only when the
    // marker is present and its byte count matches sizeof(MeshKey); otherwise
    // leaves `out` at its default (all-zero) value, like loadMeshKey.
    bool cookingLoadMeshKey(const omni::physx::usdparser::AttachedStage* attachedStage,
                            const SdfPath& primPath,
                            const TfToken& crcToken,
                            omni::physx::usdparser::MeshKey& out)
    {
        VtArray<PXR_NS::uchar> bytes;
        if (!cookingReadArray(attachedStage, primPath, crcToken, bytes))
            return false;
        if (bytes.size() != sizeof(omni::physx::usdparser::MeshKey))
            return false;
        std::memcpy(&out, bytes.data(), sizeof(omni::physx::usdparser::MeshKey));
        return true;
    }

    // Source-routed applied-API / prim-type gates, keyed by path.
    bool cookingHasSchema(const omni::physx::usdparser::AttachedStage* attachedStage,
                          const SdfPath& primPath,
                          const TfToken& schemaTypeName)
    {
        if (const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr)
            return omni::physx::internal::hasAppliedSchema(*src, attachedStage->keyFor(primPath), schemaTypeName);
        return false;
    }

    template <typename SchemaT>
    bool cookingIsA(const omni::physx::usdparser::AttachedStage* attachedStage, const SdfPath& primPath)
    {
        if (const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr)
            return omni::physx::internal::isAType<SchemaT>(*src, attachedStage->keyFor(primPath));
        return false;
    }

    void warnTetMeshOrientation(const SdfPath& tetMeshPath,
                                const VtArray<GfVec3f>& points,
                                const VtArray<GfVec4i>& tetVertexIndices,
                                bool expectLeftHanded)
    {
        uint32_t invertedCount = 0;
        for (const GfVec4i& tet : tetVertexIndices)
        {
            if (tet[0] < 0 || tet[1] < 0 || tet[2] < 0 || tet[3] < 0 ||
                size_t(tet[0]) >= points.size() || size_t(tet[1]) >= points.size() ||
                size_t(tet[2]) >= points.size() || size_t(tet[3]) >= points.size())
            {
                continue;
            }

            const GfVec3d a = GfVec3d(points[tet[1]]) - GfVec3d(points[tet[0]]);
            const GfVec3d b = GfVec3d(points[tet[2]]) - GfVec3d(points[tet[0]]);
            const GfVec3d c = GfVec3d(points[tet[3]]) - GfVec3d(points[tet[0]]);
            const double signedVolume6 =
                a[0] * (b[1] * c[2] - b[2] * c[1]) -
                a[1] * (b[0] * c[2] - b[2] * c[0]) +
                a[2] * (b[0] * c[1] - b[1] * c[0]);
            const bool isLeftHanded = signedVolume6 < 0.0;
            if (isLeftHanded != expectLeftHanded)
            {
                ++invertedCount;
            }
        }

        if (invertedCount > 0)
        {
            CARB_LOG_WARN(
                "Cooking: Found %d inverted tets of %d tets in total, relative to UsdGeomGprim orientation %s, %s",
                invertedCount, uint32_t(tetVertexIndices.size()),
                expectLeftHanded ? "leftHanded" : "rightHanded", tetMeshPath.GetText());
        }
    }

    void switchTetsOrientation(VtArray<GfVec4i>& tetVertexIndices)
    {
        for (size_t i = 0; i < tetVertexIndices.size(); ++i)
        {
            GfVec4i& tet = tetVertexIndices[i];
            std::swap(tet[0], tet[1]);
        }
    }

} // namespace

namespace cookingdataasync
{
// These are the collision related attributes that we track in the USD listener.
// If any of these attributes change we *might* have to recook the asset.

static const TfToken particleSamplingCrcToken{ "physxParticleSampling:crc" };
static const TfToken deformableBodyDataCrcToken("physxDeformableBody:deformableBodyDataCrc");
static const TfToken simMeshNumTetsPerElementToken("physxVolumeDeformableSim:numTetsPerElement");
static const TfToken simMeshHexCrcToken("physxVolumeDeformableSim:simMeshHexCrc");


using PrimRefreshSet = std::unordered_set< SdfPath, SdfPath::Hash>;

using MeshKeySet = std::unordered_set< omni::physx::usdparser::MeshKey, omni::physx::usdparser::MeshKeyHash >;

using TokenSet = std::unordered_set< TfToken, TfToken::HashFunctor >;

// This is a small helper class to manage blocking USD update events
// when we are finalizing a cooking task. The finalize step of a cooking
// task usually involves writing some of the cooked data back out to the UsdPrim
// it is associated with. When we write out these cooked data attributes we don't
// want to be triggering other USD notice handlers to react to those changes.
class ScopedBlockUSDUpdates
{
public:
    ScopedBlockUSDUpdates(CookingDataAsync *cda) : m_cookingDataAsync(cda)
    {
        m_cookingDataAsync->blockUSDUpdate(true);
    }
    ~ScopedBlockUSDUpdates(void)
    {
        m_cookingDataAsync->blockUSDUpdate(false);
    }
    CookingDataAsync *m_cookingDataAsync{nullptr};
};

class CookingDataAsyncImpl : public CookingDataAsync, public TfWeakBase
{
public:
    CookingDataAsyncImpl(physx::PxPhysics& physics, omni::physx::IPhysxCookingServicePrivate& cookingServicePrivate, omni::physx::IPhysxCookingService& cookingService, omni::physx::PhysxCookingAsyncContext context):
        mPhysics(physics), m_cookingServicePrivate(cookingServicePrivate), m_cookingService(cookingService), m_asyncContext(context)
    {
        m_settings = carb::getCachedInterface<carb::settings::ISettings>();
        // Change detection is driven by the attached stage's IChangeFeed (ADR-0003),
        // registered at feed creation by AttachedStage via registerOnChangeFeed().
        // No global USD notice listener.

        // Collision-related attributes that may require a recook when changed.
        m_collisionTokens.insert(UsdGeomTokens->points);
        m_collisionTokens.insert(UsdGeomTokens->indices);
        m_collisionTokens.insert(UsdGeomTokens->faceVertexCounts);
        m_collisionTokens.insert(UsdGeomTokens->faceVertexIndices);
        m_collisionTokens.insert(UsdGeomTokens->orientation);
        m_collisionTokens.insert(UsdGeomTokens->holeIndices);
        m_collisionTokens.insert(UsdPhysicsTokens->physicsCollisionEnabled);
        m_collisionTokens.insert(UsdPhysicsTokens->physicsApproximation);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexHullCollisionHullVertexLimit);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexHullCollisionMinThickness);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexDecompositionCollisionErrorPercentage);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexDecompositionCollisionHullVertexLimit);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexDecompositionCollisionMaxConvexHulls);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexDecompositionCollisionMinThickness);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexDecompositionCollisionVoxelResolution);
        m_collisionTokens.insert(PhysxSchemaTokens->physxConvexDecompositionCollisionShrinkWrap);
        m_collisionTokens.insert(PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionMetric);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSDFMeshCollisionSdfResolution);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSDFMeshCollisionSdfSubgridResolution);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSDFMeshCollisionSdfNarrowBandThickness);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSDFMeshCollisionSdfMargin);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSDFMeshCollisionSdfBitsPerSubgridPixel);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSDFMeshCollisionSdfEnableRemeshing);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSDFMeshCollisionSdfTriangleCountReductionFactor);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSphereFillCollisionFillMode);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSphereFillCollisionMaxSpheres);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSphereFillCollisionSeedCount);
        m_collisionTokens.insert(PhysxSchemaTokens->physxSphereFillCollisionVoxelResolution);
        m_collisionTokens.insert(PhysxSchemaTokens->physxDeformableBodyAutoDeformableBodyEnabled);
        m_collisionTokens.insert(PhysxSchemaTokens->physxDeformableBodyResolution);
        m_collisionTokens.insert(PhysxSchemaTokens->physxDeformableBodyAutoDeformableMeshSimplificationEnabled);
        m_collisionTokens.insert(PhysxSchemaTokens->physxDeformableBodyRemeshingEnabled);
        m_collisionTokens.insert(PhysxSchemaTokens->physxDeformableBodyRemeshingResolution);
        m_collisionTokens.insert(PhysxSchemaTokens->physxDeformableBodyTargetTriangleCount);
        m_collisionTokens.insert(PhysxSchemaTokens->physxDeformableBodyForceConforming);
        m_collisionTokens.insert(deformableBodyDataCrcToken);
        m_collisionTokens.insert(simMeshNumTetsPerElementToken);
        m_collisionTokens.insert(simMeshHexCrcToken);
    }

    virtual ~CookingDataAsyncImpl(void)
    {
        m_asyncContext = nullptr;
        m_settings->setBool(PROGRESS_BAR_ENABLED, false);
        {
            lock_guard _lock(m_mutex);
            // The change-feed interest is owned by the (per-stage) feed and dies with
            // it on detach/source-rebuild; nothing to revoke here.
        }
    }

    /**
    * This method is called once per logical 'frame' from the main thread to
    * dispatch new cooking tasks as well as process the results of cooking tasks
    * which have completed.
    *
    * @return : Returns the number of cooking tasks still active/pending
    */
    virtual uint32_t pump(void) final
    {
        carb::extras::Timer timer;
        timer.start();
        const uint32_t pumpResult = m_asyncContext ? getComputeService().pumpAsyncContext(m_asyncContext) : 0;
        if (timer.getElapsedTime<int64_t>(carb::extras::Timer::Scale::eMilliseconds) >= 16) // never take more than 16 ms on the main thread
        {
            return pumpResult;
        }

        CARB_PROFILE_ZONE(0, "CookingDataAsync::pump");
        // We block USD notice handlers here. Any finalized results being written to usd primitives
        // should not trigger other notice handlers, including our own. All classification below is
        // source-routed by ObjectKey; the active attached stage is resolved inside the lock block.

        {
            lock_guard _lock(m_mutex);
            ScopedBlockUSDUpdates _block(this);

            // For each changed prim, find the nearest descendant (incl. self) that
            // carries a cooking-relevant schema and schedule it for a possible
            // recook. Routed through the source's scoped query + hasSchema rather
            // than a direct UsdPrimRange + HasAPI walk. hasSchema matches an
            // authored/applied schema by name; for the directly-applied physics
            // APIs checked here that is equivalent to HasAPI<>. Falls back to the
            // direct USD walk when no attached source is available.
            omni::physx::usdparser::AttachedStage* changeAttached =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
            omni::physics::parse::IPhysicsSource* changeSrc =
                changeAttached ? changeAttached->getSource() : nullptr;

            // Intern the cooking-relevant schema tokens once for the whole pump
            // (shared by every refresh-set loop below). Without an attached source
            // the pump cannot cook (parse + dispatch both key off it), so the loops
            // are guarded on `changeSrc` and otherwise just drain their input sets.
            const omni::physics::parse::TokenId collToken =
                changeSrc ? changeSrc->internToken(
                                UsdSchemaRegistry::GetSchemaTypeName(TfType::Find<UsdPhysicsCollisionAPI>()).GetString()) :
                            omni::physics::parse::TokenId{};
            const omni::physics::parse::TokenId dbToken =
                changeSrc ? changeSrc->internToken(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI.GetString()) :
                            omni::physics::parse::TokenId{};
            const omni::physics::parse::TokenId dpToken =
                changeSrc ? changeSrc->internToken(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformablePoseAPI.GetString()) :
                            omni::physics::parse::TokenId{};

            if (changeSrc)
            {
                // For each api-schema change, find the nearest descendant (incl.
                // self) carrying a cooking-relevant schema and schedule a recook.
                for (const auto& primKey : m_primApiSchemasChangeRefreshSet)
                {
                    const omni::physics::parse::ObjectKey rootKey = changeAttached->keyFor(primKey);
                    if (!rootKey.valid())
                        continue;

                    bool found = false;
                    changeSrc->forEachDescendantPruned(
                        rootKey,
                        [&](omni::physics::parse::ObjectKey k) -> bool
                        {
                            if (found)
                                return true; // already scheduled one; prune the rest
                            if (changeSrc->hasSchema(k, collToken) || changeSrc->hasSchema(k, dbToken) ||
                                changeSrc->hasSchema(k, dpToken))
                            {
                                addPrimRefreshSet(changeAttached->pathFor(k));
                                found = true;
                                return true; // prune this match's descendants
                            }
                            return false;
                        });
                }

                // For added/removed prims, schedule the owning deformable body.
                for (const auto& primKey : m_primAddedRemovedRefreshSet)
                {
                    const omni::physics::parse::ObjectKey parentKey =
                        changeAttached->keyFor(primKey.GetParentPath());
                    if (!parentKey.valid())
                        continue;
                    const omni::physics::parse::ObjectKey bodyKey =
                        findDeformableBodyAncestorKey(*changeSrc, parentKey, dbToken);
                    if (bodyKey.valid())
                        addPrimRefreshSet(changeAttached->pathFor(bodyKey));
                }
            }

            m_primApiSchemasChangeRefreshSet.clear();
            m_primAddedRemovedRefreshSet.clear();

            // Check the primitives which *may* have had collision property changes to see if they
            // actually require a recook.
#if USE_ASYNC_COOKING
            // Time-box this loop; local disk cache hits make it fast, but we still bail after 16ms.
            timer.start();
            bool endedPrematurely = false;
            for(auto it = m_primRefreshSet.begin(); it != m_primRefreshSet.end(); )
            {
                CARB_PROFILE_ZONE(0, "CookingDataAsync::pump primRefreshSet");
                const SdfPath primKey = *it;
                const omni::physics::parse::ObjectKey key =
                    changeSrc ? changeAttached->keyFor(primKey) : omni::physics::parse::ObjectKey{};
                if (changeSrc && key.valid())
                {
                    const bool hasCollisionAPI = changeSrc->hasSchema(key, collToken);
                    const bool hasDeformablePoseAPI = changeSrc->hasSchema(key, dpToken);
                    const bool hasDeformableBodyAPI = changeSrc->hasSchema(key, dbToken);
                    omni::physics::parse::ObjectKey deformableBodyKey;
                    if (hasDeformableBodyAPI)
                        deformableBodyKey = key;
                    else if (hasCollisionAPI || hasDeformablePoseAPI)
                        deformableBodyKey = findDeformableBodyAncestorKey(*changeSrc, key, dbToken);

                    if (hasCollisionAPI && !deformableBodyKey.valid())
                    {
                        // Parse the fully digested PhysXShapeDesc for this prim; if it is a cookable
                        // shape, spawn a background cooking task. parseCollision keys off the stage
                        // id; the cook dispatch resolves `key` back to a path at the service boundary.
                        omni::physx::usdparser::PhysxShapeDesc* shapeDesc =
                            omni::physx::usdparser::parseCollision(*changeAttached, primKey, primKey);
                        if (shapeDesc)
                        {
                            switch (shapeDesc->type)
                            {
                            case omni::physx::usdparser::ObjectType::eConvexMeshShape:
                            {
                                omni::physx::usdparser::ConvexMeshPhysxShapeDesc* desc = static_cast<omni::physx::usdparser::ConvexMeshPhysxShapeDesc*>(shapeDesc);
                                getConvexMeshInternal(*desc, key, *changeAttached, true, nullptr);
                            }
                            break;
                            case omni::physx::usdparser::ObjectType::eSpherePointsShape:
                            {
                                omni::physx::usdparser::SpherePointsPhysxShapeDesc* desc = static_cast<omni::physx::usdparser::SpherePointsPhysxShapeDesc*>(shapeDesc);
                                getSpherePointsInternal(*desc, key, *changeAttached, true, nullptr);
                            }
                            break;
                            case omni::physx::usdparser::ObjectType::eConvexMeshDecompositionShape:
                            {
                                omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc* desc = static_cast<omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc*>(shapeDesc);
                                getConvexMeshDecompositionInternal(*desc, key, *changeAttached, true, nullptr);
                            }
                            break;
                            case omni::physx::usdparser::ObjectType::eTriangleMeshShape:
                            {
                                omni::physx::usdparser::TriangleMeshPhysxShapeDesc* desc = static_cast<omni::physx::usdparser::TriangleMeshPhysxShapeDesc*>(shapeDesc);
                                getTriangleMeshInternal(*desc, key, *changeAttached, true, nullptr, nullptr);
                            }
                            break;
                            default:
                                break;
                            }
                            omni::physx::usdparser::releaseDesc(shapeDesc);
                        }
                    }
                    else if (deformableBodyKey.valid())
                    {
                        cookDeformableBodyInternalAsync(deformableBodyKey, *changeAttached, false);
                    }
                }
                it = m_primRefreshSet.erase(it); // remove processed item and advance to next
                // Make sure m_primXformRefreshSet only contains updates that are exclusive to
                // tranformation changes
                m_primXformRefreshSet.erase(primKey);
                if (timer.getElapsedTime<int64_t>(carb::extras::Timer::Scale::eMilliseconds) >= 16) // never take more than 16 ms on the main thread
                {
                    endedPrematurely = true;
                    break;
                }
            }
            if(!endedPrematurely)
            {
                for (auto &primKey : m_primXformRefreshSet)
                {
                    //TODO we also need to search children to catch transforms
                    //that have an impact to deformable cooking, which might be expensive
                    //without maintaining a SdfPathTable
                    const omni::physics::parse::ObjectKey key =
                        changeSrc ? changeAttached->keyFor(primKey) : omni::physics::parse::ObjectKey{};
                    if (!changeSrc || !key.valid())
                        continue;

                    omni::physics::parse::ObjectKey deformableBodyKey;
                    if (changeSrc->hasSchema(key, dbToken))
                        deformableBodyKey = key;
                    else if (changeSrc->hasSchema(key, collToken) || changeSrc->hasSchema(key, dpToken))
                        deformableBodyKey = findDeformableBodyAncestorKey(*changeSrc, key, dbToken);

                    if (deformableBodyKey.valid())
                    {
                        cookDeformableBodyInternalAsync(deformableBodyKey, *changeAttached, true);
                    }
                }

                m_primRefreshSet.clear();
                m_primXformRefreshSet.clear();
            }
#else
            m_primRefreshSet.clear();
            m_primXformRefreshSet.clear();
#endif
            refreshProgressBarStatus();
            return pumpResult;
        }
    }

    /**
    * This method returns the PhysX PxConvexMesh associated with this USD prim if available.
    * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
    * to compute the result.
    *
    * @param desc : The convex mesh shape descriptor which defines the properties to apply when creating the convex mesh approximation
    * @param usdPrim : The UsdPrim associated with this convex mesh
    * @param asynchronous : If false the convex mesh will be cooked synchronously (blocking) in this thread. If true, it will spawn a background task if necessary.
    *
    * @return : Returns the pointer to the PxConvexMesh if it was available at this time.
    */
    virtual ::physx::PxConvexMesh* getConvexMesh(const omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc, omni::physics::parse::ObjectKey primKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool asynchronous, omni::physx::IPhysxCookingCallback* cb = nullptr) final
    {
        ::physx::PxConvexMesh* ret = nullptr;

        // Block USD notice handlers in case we write out any results during this call
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        return getConvexMeshInternal(desc, primKey, attachedStage, asynchronous, cb);
    }

    static void reportCookingFinished(const uint64_t stageId, const uint64_t primId, omni::physx::IPhysxCookingCallback* cb)
    {
        if (cb && cb->cookingFinishedCallback)
        {
            cb->cookingFinishedCallback(stageId, primId, omni::physx::PhysxCookingResult::eVALID, cb->userData);
        }
    }

    static void reportLegacyCookingFinishedCallbackAndResult(omni::physx::IPhysxCookingCallback* cb, const omni::physx::PhysxCookingComputeResult& result)
    {
        if(cb && cb->cookingResultCallback)
        {
            cb->cookingResultCallback(result.cookedDataCRC, result.cookedData, result.cookedDataNumElements, cb->userData);
        }
             
        if (cb && cb->cookingFinishedCallback)
        {
            cb->cookingFinishedCallback(result.request->primStageId, result.request->primId, result.result, cb->userData);
        }
    }

    void fillRequestPrimMeshView(omni::physx::PhysxCookingComputeRequest& request, const omni::physx::usdparser::MergeMeshDesc& mergeMeshDesc)
    {
        request.dataInputMode = omni::physx::PhysxCookingComputeRequest::DataInputMode::eINPUT_MODE_FROM_PRIM_MESH_VIEW;

        // Storage is std::vector<carb::Float3> / std::vector<int32_t> post US3
        // unification — `data()` is already the right pointer type.
        request.primMeshView.points = { mergeMeshDesc.points.data(), mergeMeshDesc.points.size() };
        request.primMeshView.indices = { mergeMeshDesc.indices.data(), mergeMeshDesc.indices.size() };
        request.primMeshView.faces = { mergeMeshDesc.faces.data(), mergeMeshDesc.faces.size() };
        request.primMeshView.holeIndices = { mergeMeshDesc.holes.data(), mergeMeshDesc.holes.size() };
    }

    /**
    * This method returns the PhysX PxConvexMesh associated with this USD prim if available.
    * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
    * to compute the result.
    *
    * @param desc : The convex mesh shape descriptor which defines the properties to apply when creating the convex mesh approximation
    * @param usdPrim : The UsdPrim associated with this convex mesh
    * @param asynchronous : If false the convex mesh will be cooked synchronously (blocking) in this thread. If true, it will spawn a background task if necessary.
    *
    * @return : Returns the pointer to the PxConvexMesh if it was available at this time.
    */
    virtual ::physx::PxConvexMesh* getConvexMeshInternal(const omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc,
                                                         omni::physics::parse::ObjectKey primKey,
                                                         const omni::physx::usdparser::AttachedStage& attachedStage,
                                                         bool asynchronous, omni::physx::IPhysxCookingCallback* cb)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif

        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        ::physx::PxConvexMesh *ret = nullptr;
        ret = omni::physx::getMeshCache()->getConvexMesh(meshCRC);

        omni::physx::PhysxCookingComputeRequest request;

        // Resolve the source key to a USD path + stage id only here, at the cooking-service boundary.
        const SdfPath primPath = attachedStage.pathFor(primKey);
        request.primStageId = attachedStage.getStageId();
        request.primId = asInt(primPath);

        // If we haven't loaded the mesh from the in memory mesh cache, we go further to see if it is in the local cache or UsdPrim itself
        // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope, since the cooking service copies the view synchronously before any
        // async task is queued (CookingTask::setupTaskFromRequest resets request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
        if (ret == nullptr)
        {
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
            }
            else
            {
                // Stage C: feed the cooking service mesh geometry via IPhysicsSource
                // (no USD read in the service). Falls back to the prim-id path when
                // the source/geometry is unavailable.
                omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey);
            }

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
            request.meshKey = desc.meshKey;

            PxPhysics* pxPhysics = &mPhysics;
            PxConvexMesh*& returnedMesh = ret;
            auto weakPtrToThis = TfCreateWeakPtr(this);

            recordStatisticsRequestFor(request);
            request.onFinished = [pxPhysics, &returnedMesh,
                                         cb, weakPtrToThis](const omni::physx::PhysxCookingComputeResult& result) {
                if(weakPtrToThis)
                    weakPtrToThis->recordStatisticsResultFor(result);
                reportLegacyCookingFinishedCallbackAndResult(cb, result);
                if (result.result != omni::physx::PhysxCookingResult::eVALID)
                {
                    return;
                }
                if(!weakPtrToThis)
                {
                    return; // means that CookingDataAsync destructor was called, so this task is cancelled
                }
                const bool res = omni::physx::getMeshCache()->createRuntimeConvexMesh(
                    *pxPhysics, result.cookedDataCRC, result.cookedData[0],
                    result.isSynchronousResult ? &returnedMesh : nullptr);
                if (!res)
                {
                    SdfPath primKey = intToPath(result.request->primId);
                    CARB_LOG_WARN("Failed to create triangle mesh from cooked data! Prim(%s)\n", primKey.GetText());
                }
            };
            getComputeService().requestConvexMeshCookedData(m_asyncContext, request, desc.convexCookingParams);
            return ret;
        }
        else
        {
            reportCookingFinished(request.primStageId, request.primId, cb);
        }
        return ret; // Return the PxConvexMesh pointer if it could be resolved
    }
    
    /**
    * This method returns the PhysX PxTriangleMesh associated with this USD prim if available.
    * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
    * to compute the result.
    *
    * @param desc : The triangle mesh shape descriptor which defines the properties to apply when creating the triangle mesh approximation
    * @param usdPrim : The UsdPrim associated with this triangle mesh
    * @param asynchronous : If false the triangle mesh will be cooked synchronously (blocking) in this thread. If true, it will spawn a background task if necessary.
    *
    * @return : Returns the pointer to the PxTriangleMesh if it was available at this time.
    */
    virtual ::physx::PxTriangleMesh* getTriangleMesh(const omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
        omni::physics::parse::ObjectKey primKey,
        const omni::physx::usdparser::AttachedStage& attachedStage,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb = nullptr,
        uint16_t* maxMaterialIndex = nullptr) final
    {
        ::physx::PxTriangleMesh *ret = nullptr; // Default return is a null pointer indicating the mesh is either not found or not ready

        // Disable USD notification handlers in case we should do USD writes in this call
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        return getTriangleMeshInternal(desc, primKey, attachedStage, asynchronous, cb, maxMaterialIndex);
    }

    /**
    * This method returns the PhysX PxTriangleMesh associated with this USD prim if available.
    * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
    * to compute the result.
    *
    * @param desc : The triangle mesh shape descriptor which defines the properties to apply when creating the triangle mesh approximation
    * @param usdPrim : The UsdPrim associated with this triangle mesh
    * @param asynchronous : If false the triangle mesh will be cooked synchronously (blocking) in this thread. If true, it will spawn a background task if necessary.
    *
    * @return : Returns the pointer to the PxTriangleMesh if it was available at this time.
    */
    virtual ::physx::PxTriangleMesh* getTriangleMeshInternal(const omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                                                             omni::physics::parse::ObjectKey primKey,
                                                             const omni::physx::usdparser::AttachedStage& attachedStage,
                                                             bool asynchronous,
                                                             omni::physx::IPhysxCookingCallback* cb,
                                                             uint16_t* maxMaterialIndex)
    {

#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        ::physx::PxTriangleMesh* ret = nullptr;
        ret = omni::physx::getMeshCache()->getTriangleMesh(meshCRC);

        omni::physx::PhysxCookingComputeRequest request;

        // Resolve the source key to a USD path + stage id only here, at the cooking-service boundary.
        const SdfPath primPath = attachedStage.pathFor(primKey);
        request.primStageId = attachedStage.getStageId();
        request.primId = asInt(primPath);
        request.primMeshText = { primPath.GetText(), strlen(primPath.GetText()) };

        // If we haven't loaded the mesh from the in memory mesh cache, we go further to see if it is in the local cache
        // or UsdPrim itself
        if (ret == nullptr)
        {
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
            request.meshKey = desc.meshKey;

            // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope, since the cooking service copies the view synchronously before any
        // async task is queued (CookingTask::setupTaskFromRequest resets request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
            }
            else
            {
                // Stage C: feed cooking geometry (incl. per-face materials) via
                // IPhysicsSource (no USD read). Covers triangle + SDF (SDF cooks
                // through this triangle path with sdf params on the desc).
                omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey);
            }

            PxPhysics* pxPhysics = &mPhysics;
            const bool originalTriangles =
                desc.triangleMeshCookingParams.mode == omni::physx::TriangleMeshMode::eORIGINAL_TRIANGLES;
            request.triangulation.needsTriangleFaceMap = originalTriangles;
            request.triangulation.needsMaxMaterialIndex = originalTriangles && maxMaterialIndex != nullptr;
            PxTriangleMesh*& returnedMesh = ret;
            auto weakPtrToThis = TfCreateWeakPtr(this);

            recordStatisticsRequestFor(request);
            request.onFinished = [pxPhysics, originalTriangles, &returnedMesh, cb, maxMaterialIndex,
                                         weakPtrToThis](const omni::physx::PhysxCookingComputeResult& result) {
                if(weakPtrToThis)
                    weakPtrToThis->recordStatisticsResultFor(result);
                reportLegacyCookingFinishedCallbackAndResult(cb, result);

                if (result.result != omni::physx::PhysxCookingResult::eVALID)
                {
                    return;
                }
                if(!weakPtrToThis)
                {
                    return; // means that CookingDataAsync destructor was called, so this task is cancelled
                }
                const bool res = omni::physx::getMeshCache()->createRuntimeTriangleMesh(
                    *pxPhysics, result.cookedDataCRC, originalTriangles, result.cookedData[0],
                    result.triangulationView.trianglesFaceMap, result.isSynchronousResult ? &returnedMesh : nullptr);
                if (!res)
                {
                    SdfPath primKey = intToPath(result.request->primId);
                    CARB_LOG_WARN("Failed to create triangle mesh from cooked data! Prim(%s)\n", primKey.GetText());
                }
                if(result.isSynchronousResult)
                {
                    if(maxMaterialIndex)
                    {
                        *maxMaterialIndex = result.triangulationMaxMaterialIndex;
                    }                    
                }
            };
#if USE_PHYSX_GPU
            PxCudaContextManager* cudaContextManager = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
#else
            PxCudaContextManager* cudaContextManager = nullptr;
#endif
            if(desc.sdfMeshCookingParams.sdfResolution > 0)
            {
                // GPU SDF cooking runs on the ujitso agent, which needs a CUDA context.
                // When no usable CUDA device exists (CPU-only mode, or a GPU-less machine)
                // the agent cannot create one; with kExecuteCookingOnGPU still set (its
                // default in Options), the build is canceled but attach wedges forever in
                // UjitsoProcessContext::waitRequest's infinite wait (NvBugs 6480595). Only
                // force the CPU SDF builder when GPU is genuinely unavailable — do NOT key
                // off the supplied cudaContextManager, which is legitimately null for
                // lazy-context callers (e.g. enableGPUDynamics=false or precook before GPU
                // scene init) that the ujitso service resolves to a real context; disabling
                // GPU cooking there would needlessly move work to the CPU and change the
                // ujitso cache key.
                const omni::physx::IPhysxFoundation& foundation = omni::physx::foundation::getInterface();
                const bool gpuAvailable = foundation.cudaDeviceCheck && foundation.cudaDeviceCheck();
                if (!gpuAvailable)
                {
                    request.options.setFlag(
                        omni::physx::PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU, false);
                }
                m_cookingServicePrivate.requestSdfMeshCookedData(m_asyncContext, request, desc.triangleMeshCookingParams, desc.sdfMeshCookingParams, cudaContextManager);
            }
            else
            {
                getComputeService().requestTriangleMeshCookedData(m_asyncContext, request, desc.triangleMeshCookingParams);
            }
            return ret;
        }
        else
        {
            reportCookingFinished(request.primStageId, request.primId, cb);
        }
        return ret;
    }

    /**
    * This method returns the array of PxConvexMeshes associated with this USD prim if available.
    * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
    * to compute the result.
    *
    * @param desc : The convex decomposition shape descriptor which defines the properties to apply when creating the convex decomposition approximation
    * @param usdPrim : The UsdPrim associated with this convex decomposition
    * @param asynchronous : If false the convex decomposition will be cooked synchronously (blocking) in this thread. If true, it will spawn a background task if necessary.
    *
    * @return : Returns an std::vector of PxConvexMeshes if it was available at this time.
    */
    virtual std::vector<::physx::PxConvexMesh*> getConvexMeshDecomposition(const omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc, omni::physics::parse::ObjectKey primKey, const omni::physx::usdparser::AttachedStage& attachedStage,
        bool asynchronous, omni::physx::IPhysxCookingCallback* cb = nullptr) final
    {
        std::vector<::physx::PxConvexMesh*> ret; // By default returns an empty array

        // Block any USD notification handlers if we make any USD attribute changes in this method
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        return getConvexMeshDecompositionInternal(desc, primKey, attachedStage, asynchronous, cb);
    }

    virtual const omni::physx::usdparser::SpherePointsPhysxShapeDesc * getSpherePoints(const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,
        omni::physics::parse::ObjectKey primKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool asynchronous, omni::physx::IPhysxCookingCallback* cb = nullptr) final
    {
        // Block any USD notification handlers if we make any USD attribute changes in this method
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        return getSpherePointsInternal(desc, primKey, attachedStage, asynchronous, cb);
    }

    virtual const omni::physx::usdparser::SpherePointsPhysxShapeDesc *  getSpherePointsInternal(const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc, omni::physics::parse::ObjectKey primKey, const omni::physx::usdparser::AttachedStage& attachedStage,
        bool asynchronous, omni::physx::IPhysxCookingCallback* cb)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        const omni::physx::usdparser::SpherePointsPhysxShapeDesc *ret = nullptr;

        // Get the convex decomposition STL hash map from the in-memory cache
        const omni::physx::SphereFillMap& sphereFillMap = omni::physx::getMeshCache()->getSphereFillMap();
        // Search to see if this CRC has a representation
        omni::physx::SphereFillMap::const_iterator it = sphereFillMap.find(meshCRC);
        // Resolve the source key to a USD path + stage id only here, at the cooking-service boundary.
        const SdfPath primPath = attachedStage.pathFor(primKey);
        const long stageId = attachedStage.getStageId();
        if ( it != sphereFillMap.end())
        {
            ret = it->second;
            reportCookingFinished(stageId, asInt(primPath), cb);
        }
        else
        {

            omni::physx::PhysxCookingComputeRequest request;

            request.primStageId = stageId;
            request.primId = asInt(primPath);
            request.meshKey = desc.meshKey;

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

            // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope, since the cooking service copies the view synchronously before any
        // async task is queued (CookingTask::setupTaskFromRequest resets request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
            }
            else
            {
                // Stage C: feed cooking geometry via IPhysicsSource (no USD read).
                omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey);
            }

            PxPhysics* pxPhysics = &mPhysics;
            const omni::physx::usdparser::SpherePointsPhysxShapeDesc*& returnedMesh = ret;
            auto weakPtrToThis = TfCreateWeakPtr(this);

            recordStatisticsRequestFor(request);
            request.onFinished = [pxPhysics, &returnedMesh,
                                         cb, weakPtrToThis](const omni::physx::PhysxCookingComputeResult& result) {
                if(weakPtrToThis)
                    weakPtrToThis->recordStatisticsResultFor(result);
                reportLegacyCookingFinishedCallbackAndResult(cb, result);
                if (result.result != omni::physx::PhysxCookingResult::eVALID)
                {
                    return;
                }
                if(!weakPtrToThis)
                {
                    return; // means that CookingDataAsync destructor was called, so this task is cancelled
                }

                const bool res = omni::physx::getMeshCache()->createRuntimeSphereFill(
                    *pxPhysics, result.cookedDataCRC, result.cookedData[0],
                    result.isSynchronousResult ? &returnedMesh : nullptr);
                if (!res)
                {
                    SdfPath primKey = intToPath(result.request->primId);
                    CARB_LOG_WARN("Failed to create sphere fill from cooked data! Prim(%s)\n", primKey.GetText());
                }
            };
            getComputeService().requestSphereFillCookedData(m_asyncContext, request, desc.sphereFillCookingParams);
            return ret;
        }

        return ret;
    }


    /**
    * This method returns the array of PxConvexMeshes associated with this USD prim if available.
    * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
    * to compute the result.
    *
    * @param desc : The convex decomposition shape descriptor which defines the properties to apply when creating the convex decomposition approximation
    * @param usdPrim : The UsdPrim associated with this convex decomposition
    * @param asynchronous : If false the convex decomposition will be cooked synchronously (blocking) in this thread. If true, it will spawn a background task if necessary.
    *
    * @return : Returns an std::vector of PxConvexMeshes if it was available at this time.
    */
    virtual std::vector<::physx::PxConvexMesh*> getConvexMeshDecompositionInternal(const omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc,
        omni::physics::parse::ObjectKey primKey,
        const omni::physx::usdparser::AttachedStage& attachedStage,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb)
    {

#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif

        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        std::vector<::physx::PxConvexMesh*> ret;

        // Get the convex decomposition STL hash map from the in-memory cache
        const omni::physx::ConvexDecompositionMap& convexDecompositionMap = omni::physx::getMeshCache()->getConvexDecompositionMap();
        // Search to see if this CRC has a representation
        omni::physx::ConvexDecompositionMap::const_iterator it = convexDecompositionMap.find(meshCRC);
        // Resolve the source key to a USD path + stage id only here, at the cooking-service boundary.
        const SdfPath primPath = attachedStage.pathFor(primKey);
        const long stageId = attachedStage.getStageId();
        if (it != convexDecompositionMap.end())
        {
            for (auto &i : it->second)
            {
                ret.push_back(i);
            }
            reportCookingFinished(stageId, asInt(primPath), cb);
        }
        else
        {
            omni::physx::PhysxCookingComputeRequest request;

            request.primStageId = stageId;
            request.primId = asInt(primPath);
            request.meshKey = desc.meshKey;

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

            // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope, since the cooking service copies the view synchronously before any
        // async task is queued (CookingTask::setupTaskFromRequest resets request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
            }
            else
            {
                // Stage C: feed cooking geometry via IPhysicsSource (no USD read).
                omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey);
            }

            PxPhysics* pxPhysics = &mPhysics;
            auto weakPtrToThis = TfCreateWeakPtr(this);

            std::vector<::physx::PxConvexMesh*>& returnedMesh = ret;
            recordStatisticsRequestFor(request);
            request.onFinished = [pxPhysics, &returnedMesh,
                                         cb, weakPtrToThis](const omni::physx::PhysxCookingComputeResult& result) {
                if(weakPtrToThis)
                    weakPtrToThis->recordStatisticsResultFor(result);
                reportLegacyCookingFinishedCallbackAndResult(cb, result);
                if (result.result != omni::physx::PhysxCookingResult::eVALID)
                {
                    return;
                }
                if(!weakPtrToThis)
                {
                    return; // means that CookingDataAsync destructor was called, so this task is cancelled
                }

                const bool res = omni::physx::getMeshCache()->createRuntimeConvexDecomposition(
                    *pxPhysics, result.cookedDataCRC, result.cookedData, result.cookedDataNumElements,
                    result.isSynchronousResult ? &returnedMesh : nullptr);
                if (!res)
                {
                    SdfPath primKey = intToPath(result.request->primId);
                    CARB_LOG_WARN(
                        "Failed to create convex decomposition mesh from cooked data! Prim(%s)\n", primKey.GetText());
                }
            };
            getComputeService().requestConvexMeshDecompositionCookedData(m_asyncContext, request, desc.convexDecompositionCookingParams);
            return ret;
        }

        return ret;
    }

#if USE_ASYNC_COOKING
    /**
    * Helper function to schedule both deformable body USD data cooking and physx deformable volume mesh cooking.
    */
    void cookDeformableBodyInternalAsync(omni::physics::parse::ObjectKey bodyKey,
                                         const omni::physx::usdparser::AttachedStage& attachedStage,
                                         bool xformOnly)
    {
        if (!bodyKey.valid())
            return;

        if (attachedStage.pathFor(bodyKey).IsEmpty())
            return;

        omni::physx::usdparser::PhysxDeformableBodyDesc* deformableDesc =
            parseDeformableBody(bodyKey, attachedStage);
        if (!deformableDesc)
        {
            //no error, can be
            CARB_LOG_ERROR("cookDeformableBodyInternalAsync deformable parsing failed");
            return;
        }

        bool dataReady = true;
        if (deformableDesc->type == omni::physx::usdparser::ObjectType::eVolumeDeformableBody)
        {
            const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc* volumeDesc =
                static_cast<const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc*>(deformableDesc);
            if (deformableDesc->hasAutoAPI)
            {
                dataReady = cookVolumeDeformableBodyInternal(*volumeDesc, bodyKey, attachedStage, xformOnly, true);
            }
            if (dataReady)
            {
                // Spawn a deformable volume mesh cooking task
                ::physx::PxDefaultMemoryOutputStream outStream;
                cookDeformableVolumeMeshInternal(outStream, *volumeDesc, bodyKey, attachedStage, true);
            }
        }
        else if (deformableDesc->type == omni::physx::usdparser::ObjectType::eSurfaceDeformableBody)
        {
            const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc* surfaceDesc =
                static_cast<const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc*>(deformableDesc);
            if (deformableDesc->hasAutoAPI)
            {
                cookSurfaceDeformableBodyInternal(*surfaceDesc, bodyKey, attachedStage, xformOnly, true);
            }
        }

        ICE_FREE(deformableDesc);
    }
#endif // USE_ASYNC_COOKING

    virtual omni::physx::usdparser::PhysxDeformableBodyDesc* parseDeformableBody(
        omni::physics::parse::ObjectKey bodyKey, const omni::physx::usdparser::AttachedStage& attachedStage)
    {
        // 6C.7.4 — driven via parse-library scanStage instead of a
        // hand-rolled IUsdPhysicsListener.  The scan produces parse-lib
        // descriptors (ObjectKey / TokenId / Matrix4d fields); the
        // local boundary translator
        // `usdparser::convert::convertScannedDeformableBody` produces a
        // freshly-allocated legacy descriptor with the USD-typed fields
        // the cooking path consumes.
        //
        // scanStage's entry points hold a process-wide mutex around the
        // schema parser's listener-stack mutation (see
        // `omni.physics.usd/StageScan.cpp::scanStage`), so this
        // call is safe to invoke from the cooking refresh pump even
        // when the main parsing pipeline is registered on the same
        // listener stack.

        const SdfPath bodyPath = attachedStage.pathFor(bodyKey);
        if (bodyPath.IsEmpty())
            return nullptr;

        // Subtree re-parse keyed by the body's path. eAll mirrors the native
        // eAllPrims refresh path while still routing through the active scan backend.
        const std::vector<SdfPath> scanRoots{ bodyPath };
        static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
        omni::physics::parse::ScanOptions scanOptions;
        scanOptions.descendantScope = omni::physics::parse::DescendantScope::eAll;
        omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
            attachedStage.attachTarget(), scanRoots, kNoExclude,
            omni::physx::usdparser::iceDescriptorAllocator(), scanOptions);
        if (scanned.deformables.empty())
            return nullptr;

        return omni::physx::usdparser::convert::convertScannedDeformableBody(scanned, 0, attachedStage.getSourceUnits());
    }

    virtual void cookVolumeDeformableBody(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool asynchronous) final
    {
        if (desc.hasAutoAPI)
        {
            // Block USD notification handlers while in this call
            lock_guard _lock(m_mutex);
            ScopedBlockUSDUpdates _block(this);
            if (!bodyKey.valid() || attachedStage.pathFor(bodyKey).IsEmpty())
                return;
            cookVolumeDeformableBodyInternal(desc, bodyKey, attachedStage, false, asynchronous);
        }
    }

    virtual void cookSurfaceDeformableBody(const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool asynchronous) final
    {
        if (desc.hasAutoAPI)
        {
            // Block USD notification handlers while in this call
            lock_guard _lock(m_mutex);
            ScopedBlockUSDUpdates _block(this);
            if (!bodyKey.valid() || attachedStage.pathFor(bodyKey).IsEmpty())
                return;
            cookSurfaceDeformableBodyInternal(desc, bodyKey, attachedStage, false, asynchronous);
        }
    }

    /**
    * Utility method which return true if this path refers to an object corresponding to an active physics object of a given type.
    *
    * @param path : The path of the primitive in question
    *
    * @return : Returns true if this path is already associated with an active physics object
    */
    bool checkParsed(uint64_t stageId, const SdfPath& path, omni::physx::usdparser::ObjectType objectType)
    {
        omni::physx::usdparser::AttachedStage* attachedStage = omni::physx::usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (attachedStage)
        {
            omni::physx::usdparser::ObjectDb* db = attachedStage->getObjectDatabase();
            if (db)
            {
                return db->findEntry(path, objectType) != omni::physx::usdparser::kInvalidObjectId;
            }
        }
        return false;
    }

    /***
    * A method to store the cooked volume deformable data to the corresponding USD primitive attributes
    *
    * @param data : Description of data that needs to be stored to USD
    * @param desc : Parsed volume deformable body 
    * @param bodyPrim : The USD prim root for storing the cooked data
    * @param tetMeshCrc : The unique 128 bit hash key for this tetrahedral mesh configuration
    */
    void storeVolumeDeformableBodyDataToUsd(
        omni::physx::PhysxCookingVolumeDeformableBodyData& data,
        const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey,
        omni::physx::usdparser::AttachedStage& attachedStage,
        const omni::physx::usdparser::MeshKey& tetMeshCrc)
    {
        // Route cooked-array authoring through the source-agnostic output sink. The
        // DeformableBodyAPI gate below early-returns without a live source, and the
        // source / data-write are created in lockstep (AttachedStage::rebuildUsdSource),
        // so by the time any array is written the sink is guaranteed present — there is
        // no direct-USD write fallback. This runs main-thread during the cooking pump
        // (inside ScopedBlockUSDUpdates), so the keyFor intern-table writes are safe.
        omni::physics::usd::UsdPhysicsDataWrite* dataWrite =
            omni::physics::usd::asUsdDataWrite(attachedStage.getDataWrite());
        omni::physics::parse::IPhysicsSource* dataSource = attachedStage.getSource();
        const bool useSink = dataWrite && dataSource;

        // RAII begin/end so every early return below still closes the write batch.
        struct WriteScope
        {
            omni::physics::parse::IPhysicsDataWrite* dw;
            explicit WriteScope(omni::physics::parse::IPhysicsDataWrite* d) : dw(d) { if (dw) dw->beginWrite(); }
            ~WriteScope() { if (dw) dw->endWrite(); }
        } writeScope(useSink ? dataWrite : nullptr);

        auto setArray = [&](const SdfPath& primPath, const PXR_NS::TfToken& attrName, const auto& vtArr)
        {
            if (!useSink)
                return;
            omni::physics::parse::DataWriteView v;
            v.data = vtArr.empty() ? nullptr : vtArr.cdata();
            v.count = vtArr.size();
            v.stride = 0;
            v.device = -1;
            v.type = omni::physics::parse::DataType::e32Bit;
            dataWrite->writeArray(attachedStage.keyFor(primPath),
                                  dataSource->internToken(attrName.GetString()), v);
        };

        if (!cookingHasSchema(&attachedStage, attachedStage.pathFor(bodyKey), OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdPhysicsDeformableBodyAPI applied to %s", attachedStage.pathFor(bodyKey).GetText());
            return;
        }

        //write sim mesh UsdGeomTetMesh
        //we just write the sim points, which are actually bind pose points
        //this will reset the simulation state, if there is one
        if (!cookingIsA<UsdGeomTetMesh>(&attachedStage, desc.simMeshPath))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdGeomTetMesh sim mesh defined at %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshPath, UsdGeomTokens->points, points);

            VtArray<GfVec3f> velocities;
            setArray(desc.simMeshPath, UsdGeomTokens->velocities, velocities);

            VtArray<GfVec4i> indices;
            omni::physx::copyBuffer(indices, data.simIndices, data.simIndicesSize);
            setArray(desc.simMeshPath, UsdGeomTokens->tetVertexIndices, indices);

            // Hex sim mesh: stamp format + signature so a future cook can recover numTetsPerElement
            // from the data, even if the auto API is later removed without changes to the tetmesh.
            if (dataWrite)
            {
                if (data.numTetsPerElement == 5 || data.numTetsPerElement == 6)
                {
                    dataWrite->writeUIntAttribute(attachedStage.keyFor(desc.simMeshPath),
                                                  simMeshNumTetsPerElementToken, data.numTetsPerElement);

                    omni::physx::usdparser::MeshKey hexCrc =
                        computeSimMeshHexCrc(points, indices, data.numTetsPerElement);
                    dataWrite->writeUCharArrayAttribute(attachedStage.keyFor(desc.simMeshPath), simMeshHexCrcToken,
                                                        reinterpret_cast<const uint8_t*>(&hexCrc), sizeof(hexCrc));
                }
                else
                {
                    dataWrite->removeAttribute(attachedStage.keyFor(desc.simMeshPath), simMeshNumTetsPerElementToken);
                    dataWrite->removeAttribute(attachedStage.keyFor(desc.simMeshPath), simMeshHexCrcToken);
                }
            }
        }

        //write rest shape UsdPhysicsVolumeDeformableSimAPI
        //for now we always just write the sim mesh 1:1
        if (!cookingHasSchema(&attachedStage, desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVolumeDeformableSimAPI))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdPhysicsVolumeDeformableSimAPI applied to %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestShapePoints, points);

            VtArray<GfVec4i> indices;
            omni::physx::copyBuffer(indices, data.simIndices, data.simIndicesSize);
            setArray(desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTetVtxIndices, indices);
        }

        //write bind poses
        if (!desc.simMeshBindPoseToken.IsEmpty())
        {
            const TfToken poseAttr = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, desc.simMeshBindPoseToken);
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshPath, poseAttr, points);
        }

        if (desc.collisionMeshPath.IsEmpty())
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: Expected prim with UsdPhysicsCollisionAPI: %s",
                attachedStage.pathFor(bodyKey).GetText());
            return;
        }

        if (desc.collisionMeshPath != desc.simMeshPath)
        {
            //write collision mesh UsdGeomTetMesh
            //we just write the collision points, which are actually bind pose points
            //this will reset the simulation state, if there is one
            if (!cookingIsA<UsdGeomTetMesh>(&attachedStage, desc.collisionMeshPath))
            {
                CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdGeomTetMesh collision mesh defined at %s",
                    desc.collisionMeshPath.GetText());
                return;
            }

            {
                VtArray<GfVec3f> points;
                omni::physx::copyBuffer(points, data.collPoints, data.collPointsSize);
                setArray(desc.collisionMeshPath, UsdGeomTokens->points, points);

                VtArray<GfVec4i> indices;
                omni::physx::copyBuffer(indices, data.collIndices, data.collIndicesSize);
                setArray(desc.collisionMeshPath, UsdGeomTokens->tetVertexIndices, indices);
            }

            if (!desc.collisionMeshBindPoseToken.IsEmpty())
            {
                const TfToken poseAttr = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                    OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, desc.collisionMeshBindPoseToken);
                VtArray<GfVec3f> points;
                omni::physx::copyBuffer(points, data.collPoints, data.collPointsSize);
                setArray(desc.collisionMeshPath, poseAttr, points);
            }
        }

        //writing collision mesh surface indices in any case (even if sim/coll mesh alias)
        {
            VtArray<GfVec3i> indices;
            omni::physx::copyBuffer(indices, data.collSurfaceIndices, data.collSurfaceIndicesSize);
            setArray(desc.collisionMeshPath, UsdGeomTokens->surfaceFaceVertexIndices, indices);
        }

        // reset the skin points to their bind pose
        for (size_t s = 0; s < desc.skinGeomPaths.size(); ++s)
        {
            SdfPath skinGeomPath = desc.skinGeomPaths[s];
            if (cookingIsA<UsdGeomPointBased>(&attachedStage, skinGeomPath))
            {
                VtArray<GfVec3f> bindPoints;
                if (cookingReadBindPoints(&attachedStage, skinGeomPath, desc.skinGeomBindPoseTokens[s], bindPoints))
                {
                    setArray(skinGeomPath, UsdGeomTokens->points, bindPoints);
                }
            }
        }

        //write crc
        if (dataWrite)
        {
            dataWrite->writeUCharArrayAttribute(bodyKey, deformableBodyDataCrcToken,
                                                reinterpret_cast<const uint8_t*>(&tetMeshCrc), sizeof(tetMeshCrc));
        }
    }

    /***
    * A method to store the cooked surface deformable data to the corresponding USD primitive attributes
    *
    * @param data : Description of data that needs to be stored to USD
    * @param desc : Parsed surface deformable body
    * @param bodyPrim : The USD prim root for storing the cooked data
    * @param deformableBodyDataCrc : The unique 128 bit hash key for this mesh configuration
    */
    void storeSurfaceDeformableBodyDataToUsd(
        omni::physx::PhysxCookingSurfaceDeformableBodyData& data,
        const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey,
        omni::physx::usdparser::AttachedStage& attachedStage,
        const omni::physx::usdparser::MeshKey& deformableBodyDataCrc)
    {
        // Cooked arrays are authored through the source-agnostic output sink (no
        // direct-USD write fallback — see storeVolumeDeformableBodyDataToUsd). Runs
        // main-thread during the cooking pump, so keyFor intern-table writes are safe.
        omni::physics::usd::UsdPhysicsDataWrite* dataWrite =
            omni::physics::usd::asUsdDataWrite(attachedStage.getDataWrite());
        omni::physics::parse::IPhysicsSource* dataSource = attachedStage.getSource();
        const bool useSink = dataWrite && dataSource;

        struct WriteScope
        {
            omni::physics::parse::IPhysicsDataWrite* dw;
            explicit WriteScope(omni::physics::parse::IPhysicsDataWrite* d) : dw(d) { if (dw) dw->beginWrite(); }
            ~WriteScope() { if (dw) dw->endWrite(); }
        } writeScope(useSink ? dataWrite : nullptr);

        auto setArray = [&](const SdfPath& primPath, const PXR_NS::TfToken& attrName, const auto& vtArr)
        {
            if (!useSink)
                return;
            omni::physics::parse::DataWriteView v;
            v.data = vtArr.empty() ? nullptr : vtArr.cdata();
            v.count = vtArr.size();
            v.stride = 0;
            v.device = -1;
            v.type = omni::physics::parse::DataType::e32Bit;
            dataWrite->writeArray(attachedStage.keyFor(primPath),
                                  dataSource->internToken(attrName.GetString()), v);
        };

        if (!cookingHasSchema(&attachedStage, attachedStage.pathFor(bodyKey), OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsDeformableBodyAPI applied to %s", attachedStage.pathFor(bodyKey).GetText());
            return;
        }

        //first read crc and abort if still valid, the reason is we are consuming data from the cooking job that doesn't
        //go into USD, so we need the cooking task to return successfully to return the cached data even though
        //we don't need to update to USD.
        omni::physx::usdparser::MeshKey storedDeformableBodyDataCrc;
        cookingLoadMeshKey(&attachedStage, attachedStage.pathFor(bodyKey),
                           deformableBodyDataCrcToken, storedDeformableBodyDataCrc);
        if (storedDeformableBodyDataCrc == deformableBodyDataCrc)
        {
            return;
        }

        //write sim mesh UsdGeomMesh
        //we just write the sim points, which are actually bind pose points
        //this will reset the simulation state, if there is one
        if (!cookingIsA<UsdGeomMesh>(&attachedStage, desc.simMeshPath))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdGeomMesh sim mesh defined at %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshPath, UsdGeomTokens->points, points);

            VtArray<GfVec3f> velocities;
            setArray(desc.simMeshPath, UsdGeomTokens->velocities, velocities);

            size_t numFaces = data.simIndicesSize / 3;
            CARB_ASSERT(numFaces * 3 == data.simIndicesSize);
            VtArray<int32_t> faceVertexCounts(numFaces, 3);
            VtArray<int32_t> faceVertexIndices(numFaces*3);
            std::memcpy(faceVertexIndices.begin(), data.simIndices, sizeof(int32_t)*faceVertexIndices.size());

            setArray(desc.simMeshPath, UsdGeomTokens->faceVertexCounts, faceVertexCounts);
            setArray(desc.simMeshPath, UsdGeomTokens->faceVertexIndices, faceVertexIndices);
        }

        //write rest shape UsdPhysicsSurfaceDeformableSimAPI
        //for now we always just write the sim mesh 1:1
        if (!cookingHasSchema(&attachedStage, desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsSurfaceDeformableSimAPI))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsSurfaceDeformableSimAPI applied to %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestShapePoints, points);

            size_t numFaces = data.simIndicesSize / 3;
            VtArray<GfVec3i> triIndices(numFaces);
            std::memcpy(triIndices.data(), data.simIndices, sizeof(GfVec3i) * triIndices.size());
            setArray(desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTriVtxIndices, triIndices);
        }

        //write bind poses
        if (!desc.simMeshBindPoseToken.IsEmpty())
        {
            const TfToken poseAttr = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, desc.simMeshBindPoseToken);
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshPath, poseAttr, points);
        }

        if (desc.collisionMeshPath.IsEmpty())
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: Expected prim with UsdPhysicsCollisionAPI: %s",
                attachedStage.pathFor(bodyKey).GetText());
            return;
        }

        if (desc.collisionMeshPath != desc.simMeshPath)
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No separate collision mesh supported for surface deformables: %s",
                desc.collisionMeshPath.GetText());
            return;
        }

        // reset the skin points to their bind pose
        for (size_t s = 0; s < desc.skinGeomPaths.size(); ++s)
        {
            SdfPath skinGeomPath = desc.skinGeomPaths[s];
            if (cookingIsA<UsdGeomPointBased>(&attachedStage, skinGeomPath) && !desc.skinGeomBindPoseTokens[s].IsEmpty())
            {
                VtArray<GfVec3f> bindPoints;
                cookingReadBindPoints(&attachedStage, skinGeomPath, desc.skinGeomBindPoseTokens[s], bindPoints);
                setArray(skinGeomPath, UsdGeomTokens->points, bindPoints);
            }
        }

        //write crc
        if (dataWrite)
        {
            dataWrite->writeUCharArrayAttribute(bodyKey, deformableBodyDataCrcToken,
                                                reinterpret_cast<const uint8_t*>(&deformableBodyDataCrc), sizeof(deformableBodyDataCrc));
        }
    }

    /**
    * Fills in ParticlePoissonSamplingParams for cooking input.
    * Decomposes local to world transform into shear/scale and rigid part.
    * The shear/scale becomes part of the cooking parameters, the rigid part is passed back
    * seperately for transforming sample points after cooking into world space.
    *
    * TODO: Maybe quantize shear/scale to make caching more effective. I.e. using the same cooking data for different
    * rigid transform configurations.
    */
    bool setupParticlePoissonSamplingCookingParams(
        const SdfPath& samplerPath,
        const omni::physx::usdparser::ParticleSamplingDesc& desc,
        GfMatrix4d& rigidTransform,
        omni::physx::ParticlePoissonSamplingCookingParams& params)
    {
        GfMatrix3d shearScaleTransform;
        if (!omni::physx::particles::PhysxParticleFactory::getDecomposedTransform(
                samplerPath, desc.particleSetPath,
                rigidTransform, shearScaleTransform))
        {
            return false;
        }

        static_assert(sizeof(params.shearScale) == sizeof(shearScaleTransform));
        memcpy(params.shearScale, shearScaleTransform.data(), sizeof(params.shearScale));

        params.samplingDistance = desc.samplingDistance;
        params.sampleVolume = desc.sampleVolume;
        params.maxSamples = desc.maxSamples;

        return true;
    }

    bool setupVolumeDeformableBodyCookingParams(const omni::physx::usdparser::AttachedStage* attachedStage,
                                                omni::physics::parse::ObjectKey bodyKey,
                                                const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                                omni::physx::VolumeDeformableBodyCookingParams& params,
                                                PXR_NS::SdfPath& srcMeshPath,
                                                PXR_NS::VtArray<PXR_NS::GfVec3f>& pxrSrcPointsInSim,
                                                omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        const PXR_NS::SdfPath bodyPath = attachedStage ? attachedStage->pathFor(bodyKey) : PXR_NS::SdfPath();
        if (!bodyKey.valid() || bodyPath.IsEmpty() ||
            !cookingHasSchema(attachedStage, bodyPath, PXR_NS::OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("PhysX could not find source prim or prim has no UsdPhysicsDeformableBodyAPI!");
            return false;
        }

        PXR_NS::GfMatrix4d simToWorld = cookingWorldTransform(attachedStage, desc.simMeshPath);
        PXR_NS::GfMatrix4d worldToSim = simToWorld.GetInverse();

        PXR_NS::GfMatrix4d simToColl;
        if (desc.collisionMeshPath != desc.simMeshPath)
        {
            PXR_NS::GfMatrix4d collToWorld = cookingWorldTransform(attachedStage, desc.collisionMeshPath);
            PXR_NS::GfMatrix4d worldToColl = collToWorld.GetInverse();
            simToColl = simToWorld * worldToColl;
        }
        else
        {
            simToColl.SetIdentity();
        }

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.", bodyPath.GetText());
            return false;
        }

        srcMeshPath = desc.cookingSrcMeshPath;
        PXR_NS::TfToken srcMeshBindPoseToken = desc.cookingSrcMeshBindPoseToken;
        if (!cookingIsA<PXR_NS::UsdGeomMesh>(attachedStage, srcMeshPath))
        {
            CARB_LOG_WARN(
                "Cooking failed, deformable has no valid source mesh for cooking: %s.", bodyPath.GetText());
            return false;
        }

        PXR_NS::VtArray<int32_t> pxrSrcVertexIndices;
        PXR_NS::VtArray<int32_t> pxrSrcVertexCounts;

        {
            cookingReadBindPoints(attachedStage, srcMeshPath, srcMeshBindPoseToken, pxrSrcPointsInSim);
            cookingReadArray(attachedStage, srcMeshPath, UsdGeomTokens->faceVertexIndices, pxrSrcVertexIndices);
            cookingReadArray(attachedStage, srcMeshPath, UsdGeomTokens->faceVertexCounts, pxrSrcVertexCounts);

            // Transform skin points to sim mesh space
            PXR_NS::GfMatrix4d srcToWorld = cookingWorldTransform(attachedStage, srcMeshPath);
            PXR_NS::GfMatrix4d srcToSim = srcToWorld * worldToSim;
            for (size_t i = 0; i < pxrSrcPointsInSim.size(); ++i)
            {
                PXR_NS::GfVec3f& srcPoint = pxrSrcPointsInSim[i];
                srcPoint = PXR_NS::GfVec3f(srcToSim.Transform(srcPoint));
            }
        }

        // Reject if data is missing
        if (pxrSrcPointsInSim.empty() || pxrSrcVertexIndices.empty() || pxrSrcVertexCounts.empty())
        {
            return false;
        }

        GfMatrix4d simToCookingTransform;
        if (!::computeDeformableCookingTransform(
            &simToCookingTransform, nullptr, nullptr, simToWorld, pxrSrcPointsInSim))
        {
            return false;
        }

        customMeshCrc = computeMeshKey(pxrSrcPointsInSim, pxrSrcVertexIndices, pxrSrcVertexCounts);

        params.srcPointsInSim = { reinterpret_cast<const carb::Float3*>(pxrSrcPointsInSim.data()), pxrSrcPointsInSim.size() };

        static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
        memcpy(params.simToCookingTransform, simToCookingTransform.data(), sizeof(params.simToCookingTransform));

        static_assert(sizeof(params.simToCollTransform) == sizeof(simToColl));
        memcpy(params.simToCollTransform, simToColl.data(), sizeof(params.simToCollTransform));

        params.isAutoMeshSimplificationEnabled = desc.isAutoMeshSimplificationEnabled;
        params.isAutoRemeshingEnabled = desc.isAutoRemeshingEnabled;
        params.hasAutoForceConforming = desc.hasAutoForceConforming;
        params.isAutoHexahedralMeshEnabled = desc.isAutoHexahedralMeshEnabled;
        params.autoRemeshingResolution = desc.autoRemeshingResolution;
        params.autoTriangleTargetCount = desc.autoTriangleTargetCount;
        params.autoHexahedralResolution = desc.autoHexahedralResolution;

        return true;
    }

    /**
    * TODO: rewrite docs here. Performs the cooking operation on a deformable tetrahedral mesh associated with a particular UsdGeomMesh primitive
    *
    * @param usdPrim : The UsdGeomMesh we are cooking
    * @param xFormOnly : If true, the cooking was triggered excusively for a transform update.
    * @param asynchronous : If false, it will cook the tetrahedral mesh synchronously (blocking). If true, it will start a background cooking task for it.
    * @return : Returns true if usd data is ready and PxDeformableVolumeMesh should be cooked
    */
    bool cookVolumeDeformableBodyInternal(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          omni::physics::parse::ObjectKey bodyKey,
                                          const omni::physx::usdparser::AttachedStage& attachedStage,
                                          bool xFormOnly,
                                          bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        const PXR_NS::SdfPath bodyPath = attachedStage.pathFor(bodyKey);
        const uint64_t primStageId = uint64_t(attachedStage.getStageId());
        if (!bodyKey.valid() || bodyPath.IsEmpty())
            return false;

        const bool isSimulated = checkParsed(primStageId, bodyPath, omni::physx::usdparser::eVolumeDeformableBody);
        if (isSimulated)
        {
            // this can be triggered while the deformable body is registered for simulation:
            // PhysXUsdPhysicsInterface::createVolumeDeformableBody
            //     CookingDataAsyncImpl::cookVolumeDeformableBody
            //         CookingDataAsyncImpl::cookVolumeDeformableBodyInternal(asynchronous == false)
            //             CookingDataAsyncImpl::storeVolumeDeformableBodyDataToUsd
            //                 (write deformable body properties to USD)
            //                     CookingDataAsyncImpl::handle
            //     (registered here)
            // CookingDataAsyncImpl::pump
            //     CookingDataAsyncImpl::cookDeformableBodyInternalAsync
            //         CookingDataAsyncImpl::cookVolumeDeformableBodyInternal
            // so we don't issue a warning here.
            return false;
        }

        omni::physx::VolumeDeformableBodyCookingParams params;
        PXR_NS::SdfPath srcMeshPath;
        VtArray<GfVec3f> pxrSrcPointsInSim;
        omni::physx::usdparser::MeshKey customMeshCrc;
        if (!setupVolumeDeformableBodyCookingParams(&attachedStage, bodyKey, desc, params, srcMeshPath, pxrSrcPointsInSim, customMeshCrc))
        {
            CARB_LOG_ERROR("Volume deformable body, failed to setup cooking params, prim: %s", bodyPath.GetText());
            return false;
        }

        omni::physx::usdparser::MeshKey originalCrc;
        cookingLoadMeshKey(&attachedStage, bodyPath, deformableBodyDataCrcToken, originalCrc);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.primId = asInt(srcMeshPath);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
            request.meshKey = customMeshCrc;
            request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
            omni::physx::usdparser::MeshKey cookedDataCRC;
            request.onFinished = [&cookedDataCRC](const omni::physx::PhysxCookingComputeResult& result) {
                cookedDataCRC = result.cookedDataCRC;
            };
            m_cookingServicePrivate.requestVolumeDeformableBodyCookedData(m_asyncContext, request, params);
            if (cookedDataCRC == originalCrc)
            {
                return true;
            }
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
        request.meshKey = customMeshCrc;
        request.primStageId = primStageId;
        request.primId = asInt(srcMeshPath);
        request.deformablePathInfo.bodyPrimId = asInt(bodyPath);
        request.deformablePathInfo.simMeshPrimId = asInt(desc.simMeshPath);
        request.deformablePathInfo.collMeshPrimId = asInt(desc.collisionMeshPath);

        uint64_t bodyPrimId = asInt(bodyPath);
        bool shouldRecookPhysxMeshTooSynchronously = false;
        auto weakPtrToThis = TfCreateWeakPtr(this);
        recordStatisticsRequestFor(request);
        request.onFinished = [weakPtrToThis, &shouldRecookPhysxMeshTooSynchronously, desc,
                              bodyPrimId](const omni::physx::PhysxCookingComputeResult& result) {
            if (!weakPtrToThis)
            {
                return;
            }
            weakPtrToThis->recordStatisticsResultFor(result);
            omni::physx::IPhysxCookingServicePrivate& cookingService = weakPtrToThis->m_cookingServicePrivate;
            omni::physx::usdparser::AttachedStage* as =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->getAttachedStage(long(result.request->primStageId));
            if (as)
            {
                const SdfPath bodyPrimPath = intToPath(bodyPrimId);
                const omni::physics::parse::ObjectKey bodyKey = as->keyFor(bodyPrimPath);
                if (cookingHasSchema(as, bodyPrimPath, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
                {
                    if (result.result == omni::physx::PhysxCookingResult::eVALID)
                    {
                        if (result.cookedData)
                        {
                            omni::physx::PhysxCookingVolumeDeformableBodyData data;
                            cookingService.readVolumeDeformableBodyData(data, *result.cookedData);
                            weakPtrToThis->storeVolumeDeformableBodyDataToUsd(data, desc, bodyKey, *as, result.cookedDataCRC);
                        }
                    }
                    else
                    {
                        omni::physx::usdparser::MeshKey crc_zero;
                        if (auto* dw = omni::physics::usd::asUsdDataWrite(as->getDataWrite()))
                        {
                            dw->writeUCharArrayAttribute(bodyKey, deformableBodyDataCrcToken,
                                                         reinterpret_cast<const uint8_t*>(&crc_zero), sizeof(crc_zero));
                        }
                    }
                }
            }

            if (result.isSynchronousResult)
            {
                shouldRecookPhysxMeshTooSynchronously = true;
            }
        };

        m_cookingServicePrivate.requestVolumeDeformableBodyCookedData(m_asyncContext, request, params);
        return shouldRecookPhysxMeshTooSynchronously;
    }

    bool setupSurfaceDeformableBodyCookingParams(const omni::physx::usdparser::AttachedStage* attachedStage,
                                                 omni::physics::parse::ObjectKey bodyKey,
                                                 const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
                                                 omni::physx::SurfaceDeformableBodyCookingParams& params,
                                                 PXR_NS::SdfPath& srcMeshPath,
                                                 PXR_NS::VtArray<PXR_NS::GfVec3f>& pxrSrcPointsInSim,
                                                 omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        const PXR_NS::SdfPath bodyPath = attachedStage ? attachedStage->pathFor(bodyKey) : PXR_NS::SdfPath();
        if (!bodyKey.valid() || bodyPath.IsEmpty() ||
            !cookingHasSchema(attachedStage, bodyPath, PXR_NS::OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("PhysX could not find source prim or prim has no UsdPhysicsDeformableBodyAPI!");
            return false;
        }

        PXR_NS::GfMatrix4d simToWorld = cookingWorldTransform(attachedStage, desc.simMeshPath);
        PXR_NS::GfMatrix4d worldToSim = simToWorld.GetInverse();

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.", bodyPath.GetText());
            return false;
        }

        srcMeshPath = desc.cookingSrcMeshPath;
        PXR_NS::TfToken srcMeshBindPoseToken = desc.cookingSrcMeshBindPoseToken;
        if (!cookingIsA<PXR_NS::UsdGeomMesh>(attachedStage, srcMeshPath))
        {
            CARB_LOG_WARN(
                "Cooking failed, deformable has no valid source mesh for cooking: %s.", bodyPath.GetText());
            return false;
        }

        PXR_NS::VtArray<int32_t> pxrSrcVertexIndices;
        PXR_NS::VtArray<int32_t> pxrSrcVertexCounts;

        {
            cookingReadBindPoints(attachedStage, srcMeshPath, srcMeshBindPoseToken, pxrSrcPointsInSim);
            cookingReadArray(attachedStage, srcMeshPath, UsdGeomTokens->faceVertexIndices, pxrSrcVertexIndices);
            cookingReadArray(attachedStage, srcMeshPath, UsdGeomTokens->faceVertexCounts, pxrSrcVertexCounts);

            // Transform src points to sim mesh space
            PXR_NS::GfMatrix4d srcToWorld = cookingWorldTransform(attachedStage, srcMeshPath);
            PXR_NS::GfMatrix4d srcToSim = srcToWorld * worldToSim;
            for (size_t i = 0; i < pxrSrcPointsInSim.size(); ++i)
            {
                PXR_NS::GfVec3f& srcPoint = pxrSrcPointsInSim[i];
                srcPoint = PXR_NS::GfVec3f(srcToSim.Transform(srcPoint));
            }
        }

        // Reject if data is missing
        if (pxrSrcPointsInSim.empty() || pxrSrcVertexIndices.empty() || pxrSrcVertexCounts.empty())
        {
            return false;
        }

        GfMatrix4d simToCookingTransform;
        if (!::computeDeformableCookingTransform(
                &simToCookingTransform, nullptr, nullptr, simToWorld, pxrSrcPointsInSim))
        {
            return false;
        }

        params.srcPointsInSim = { reinterpret_cast<const carb::Float3*>(pxrSrcPointsInSim.data()),
                                  pxrSrcPointsInSim.size() };
        static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
        memcpy(params.simToCookingTransform, simToCookingTransform.data(), sizeof(params.simToCookingTransform));
        params.isAutoMeshSimplificationEnabled = desc.isAutoMeshSimplificationEnabled;
        params.isAutoRemeshingEnabled = desc.isAutoRemeshingEnabled;
        params.autoRemeshingResolution = desc.autoRemeshingResolution;
        params.autoTriangleTargetCount = desc.autoTriangleTargetCount;

        customMeshCrc = computeMeshKey(pxrSrcPointsInSim, pxrSrcVertexIndices, pxrSrcVertexCounts);
        return true;
    }

    void cookSurfaceDeformableBodyInternal(const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool xFormOnly, bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        const PXR_NS::SdfPath bodyPath = attachedStage.pathFor(bodyKey);
        const uint64_t primStageId = uint64_t(attachedStage.getStageId());
        if (!bodyKey.valid() || bodyPath.IsEmpty())
            return;

        const bool isSimulated = checkParsed(primStageId, bodyPath, omni::physx::usdparser::eSurfaceDeformableBody);
        if (isSimulated)
        {
            // refer to cookVolumeDeformableBodyInternal why we don't warn here
            return;
        }

        omni::physx::SurfaceDeformableBodyCookingParams params;
        PXR_NS::SdfPath srcMeshPath;
        VtArray<GfVec3f> pxrSrcPointsInSim;
        omni::physx::usdparser::MeshKey customMeshCrc;
        if (!setupSurfaceDeformableBodyCookingParams(&attachedStage, bodyKey, desc, params, srcMeshPath, pxrSrcPointsInSim, customMeshCrc))
        {
            CARB_LOG_ERROR(
                "Surface deformable body, failed to setup cooking params, prim: %s", bodyPath.GetText());
            return;
        }

        omni::physx::usdparser::MeshKey originalCrc;
        cookingLoadMeshKey(&attachedStage, bodyPath, deformableBodyDataCrcToken, originalCrc);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.primId = asInt(srcMeshPath);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
            request.meshKey = customMeshCrc;
            request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
            omni::physx::usdparser::MeshKey cookedDataCRC;
            request.onFinished = [&cookedDataCRC](const omni::physx::PhysxCookingComputeResult& result) {
                cookedDataCRC = result.cookedDataCRC;
            };
            m_cookingServicePrivate.requestSurfaceDeformableBodyCookedData(m_asyncContext, request, params);
            if (cookedDataCRC == originalCrc)
            {
                return;
            }
        }
        omni::physx::PhysxCookingComputeRequest request;
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
        request.meshKey = customMeshCrc;
        request.primStageId = primStageId;
        request.primId = asInt(srcMeshPath);
        request.deformablePathInfo.bodyPrimId = asInt(bodyPath);
        request.deformablePathInfo.simMeshPrimId = asInt(desc.simMeshPath);
        request.deformablePathInfo.collMeshPrimId = asInt(desc.collisionMeshPath);

        uint64_t bodyPrimId = asInt(bodyPath);
        auto weakPtrToThis = TfCreateWeakPtr(this);
        recordStatisticsRequestFor(request);
        request.onFinished = [weakPtrToThis, desc, bodyPrimId](const omni::physx::PhysxCookingComputeResult& result) {
            if (!weakPtrToThis)
            {
                return;
            }
            weakPtrToThis->recordStatisticsResultFor(result);
            if (result.result == omni::physx::PhysxCookingResult::eERROR_CANCELED)
            {
                return;
            }

            omni::physx::IPhysxCookingServicePrivate& cookingService = weakPtrToThis->m_cookingServicePrivate;
            omni::physx::usdparser::AttachedStage* as =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->getAttachedStage(long(result.request->primStageId));
            if (as)
            {
                const SdfPath bodyPrimPath = intToPath(bodyPrimId);
                const omni::physics::parse::ObjectKey bodyKey = as->keyFor(bodyPrimPath);
                if (cookingHasSchema(as, bodyPrimPath, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
                {
                    if (result.result == omni::physx::PhysxCookingResult::eVALID)
                    {
                        if (result.cookedData)
                        {
                            omni::physx::PhysxCookingSurfaceDeformableBodyData data;
                            cookingService.readSurfaceDeformableBodyData(data, *result.cookedData);
                            weakPtrToThis->storeSurfaceDeformableBodyDataToUsd(data, desc, bodyKey, *as, result.cookedDataCRC);
                        }
                    }
                    else
                    {
                        omni::physx::usdparser::MeshKey crc_zero;
                        if (auto* dw = omni::physics::usd::asUsdDataWrite(as->getDataWrite()))
                        {
                            dw->writeUCharArrayAttribute(bodyKey, deformableBodyDataCrcToken,
                                                         reinterpret_cast<const uint8_t*>(&crc_zero), sizeof(crc_zero));
                        }
                    }
                }
            }
        };

        m_cookingServicePrivate.requestSurfaceDeformableBodyCookedData(m_asyncContext, request, params);
        return;
    }

    virtual bool cookDeformableVolumeMesh(::physx::PxDefaultMemoryOutputStream& outStream,
                                          const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          omni::physics::parse::ObjectKey bodyKey,
                                          const omni::physx::usdparser::AttachedStage& attachedStage,
                                          bool asynchronous) final
    {
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        if (!bodyKey.valid() || attachedStage.pathFor(bodyKey).IsEmpty())
            return false;
        return cookDeformableVolumeMeshInternal(outStream, desc, bodyKey, attachedStage, asynchronous);
    }

    virtual bool computeDeformableCookingTransform(GfMatrix4d* simToCookingTransform,
                                                   GfMatrix4d* cookingToWorldTransform,
                                                   double* cookingToWorldScale,
                                                   const GfMatrix4d& simToWorld,
                                                   const VtArray<GfVec3f>& boundsFitPoints) final
    {
        return ::computeDeformableCookingTransform(
            simToCookingTransform, cookingToWorldTransform, cookingToWorldScale, simToWorld, boundsFitPoints);
    }

    bool setupDeformableVolumeMeshCookingParams(const omni::physx::usdparser::AttachedStage* attachedStage,
                                                omni::physics::parse::ObjectKey bodyKey,
                                                const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                                omni::physx::DeformableVolumeMeshCookingParams& params,
                                                VtArray<GfVec3f>& pxrSimPoints,
                                                VtArray<GfVec3f>& pxrSimBindPoints,
                                                VtArray<GfVec4i>& pxrSimIndices,
                                                VtArray<GfVec3f>& pxrCollBindPointsInSim,
                                                VtArray<GfVec4i>& pxrCollIndices,
                                                VtArray<GfVec3i>& pxrCollSurfaceIndices)
    {
        const PXR_NS::SdfPath bodyPath = attachedStage ? attachedStage->pathFor(bodyKey) : PXR_NS::SdfPath();
        if (!bodyKey.valid() || bodyPath.IsEmpty() ||
            !cookingHasSchema(attachedStage, bodyPath, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
        {
            CARB_LOG_WARN("Cooking failed, deformable body prim is invalid.");
            return false;
        }

        if (!cookingIsA<PXR_NS::UsdGeomTetMesh>(attachedStage, desc.simMeshPath) ||
            !cookingHasSchema(attachedStage, desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVolumeDeformableSimAPI))
        {
            CARB_LOG_WARN("Cooking failed, simulation mesh prim %s is invalid.", desc.simMeshPath.GetText());
            return false;
        }

        if (desc.simMeshPath != bodyPath && desc.simMeshPath.GetParentPath() != bodyPath)
        {
            CARB_LOG_WARN(
                "Cooking failed, simulation mesh %s either has to be identical with bodyPrim %s or directly parented under it.",
                desc.simMeshPath.GetText(), bodyPath.GetText());
            return false;
        }

        if (!cookingIsA<PXR_NS::UsdGeomTetMesh>(attachedStage, desc.collisionMeshPath))
        {
            CARB_LOG_WARN("Cooking failed, collision mesh prim %s is invalid.", bodyPath.GetText());
            return false;
        }

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.", bodyPath.GetText());
            return false;
        }
        GfMatrix4d simToWorld = cookingWorldTransform(attachedStage, desc.simMeshPath);
        GfMatrix4d worldToSim = simToWorld.GetInverse();

        // read simulation mesh rest shape for cooking (until the SDK supports a proper rest shape)
        // need to make sure the rest shape is compatible with the tetmesh topology
        params.numTetsPerElement = 1;
        {
            VtArray<GfVec3f> simPoints;
            VtArray<GfVec4i> simTetVertexIndices;
            VtArray<GfVec3f> simRestShapePoints;
            VtArray<GfVec4i> simRestTetVtxIndices;

            cookingReadArray(attachedStage, desc.simMeshPath, UsdGeomTokens->points, simPoints);
            cookingReadArray(attachedStage, desc.simMeshPath, UsdGeomTokens->tetVertexIndices, simTetVertexIndices);
            warnTetMeshOrientation(desc.simMeshPath, simPoints, simTetVertexIndices, desc.simMeshLeftHandedOrientation);

            cookingReadArray(attachedStage, desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestShapePoints, simRestShapePoints);
            cookingReadArray(attachedStage, desc.simMeshPath, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTetVtxIndices, simRestTetVtxIndices);

            // Recover format from the hex signature, verified against the rest shape used for
            // physx cooking (not the live simulation state points).
            // Absent or mismatching signature => treat as plain tet mesh.
            // Markers are read through the source by path (no UsdPrim): numTetsPerElement is a
            // scalar uint, the hex CRC a UCharArray blob. Both must be present (mirrors the prior
            // HasAuthoredValue gate) — storeMeshKey writes/removes them as a pair.
            uint32_t storedNumTets = 0;
            omni::physx::usdparser::MeshKey storedHexCrc;
            if (omni::physx::internal::getValue<uint32_t>(*attachedStage, desc.simMeshPath,
                                                          simMeshNumTetsPerElementToken,
                                                          UsdTimeCode::Default(), storedNumTets) &&
                (storedNumTets == 5 || storedNumTets == 6) &&
                cookingLoadMeshKey(attachedStage, desc.simMeshPath, simMeshHexCrcToken, storedHexCrc))
            {
                omni::physx::usdparser::MeshKey expectedHexCrc =
                    computeSimMeshHexCrc(simRestShapePoints, simRestTetVtxIndices, storedNumTets);
                if (expectedHexCrc == storedHexCrc)
                {
                    params.numTetsPerElement = storedNumTets;
                }
                else
                {
                    CARB_LOG_WARN(
                        "Cooking: sim mesh %s carries a hex signature that doesn't match its "
                        "current rest shape; cooking as plain tet mesh.",
                        desc.simMeshPath.GetText());
                }
            }

            if (desc.simMeshLeftHandedOrientation)
            {
                switchTetsOrientation(simTetVertexIndices);
                switchTetsOrientation(simRestTetVtxIndices);
            }

            bool mismatch = simPoints.size() != simRestShapePoints.size() ||
                            simTetVertexIndices.size() != simRestTetVtxIndices.size() ||
                            std::memcmp(simTetVertexIndices.data(), simRestTetVtxIndices.data(),
                                         sizeof(GfVec4i) * simTetVertexIndices.size()) != 0;

            if (mismatch)
            {
                CARB_LOG_WARN(
                    "Cooking failed, UsdGeomTetMesh not compatible with rest attributes in DeformableVolumeSimAPI, %s",
                     bodyPath.GetText());
                return false;
            }

            pxrSimPoints.swap(simRestShapePoints);
            pxrSimIndices.swap(simTetVertexIndices);
        }

        if (desc.collisionMeshPath != desc.simMeshPath)
        {
            // Need to construct embedding for collision mesh:
            // (simulation bind pose, sim mesh indices == rest shape indices, collision bind pose) -> embedding
            // (embedding, sim rest shape points, rest shape topo> -> collision rest shape
            VtArray<GfVec3f> simMeshBindPoints;
            VtArray<GfVec3f> collMeshBindPoints;
            {
                const bool gotSim = cookingReadBindPoints(attachedStage, desc.simMeshPath, desc.simMeshBindPoseToken, simMeshBindPoints);
                const bool gotColl = cookingReadBindPoints(attachedStage, desc.collisionMeshPath, desc.collisionMeshBindPoseToken, collMeshBindPoints);
                if (!gotSim || !gotColl)
                {
                    CARB_LOG_ERROR("Cooking failed: %s", bodyPath.GetText());
                    return false;
                }
            }

            // Transform collision points to sim space
            GfMatrix4d collToWorld = cookingWorldTransform(attachedStage, desc.collisionMeshPath);
            GfMatrix4d collToSim = collToWorld * worldToSim;
            for (size_t i = 0; i < collMeshBindPoints.size(); ++i)
            {
                collMeshBindPoints[i] = PXR_NS::GfVec3f(collToSim.Transform(collMeshBindPoints[i]));
            }
            pxrCollBindPointsInSim.swap(collMeshBindPoints);
            pxrSimBindPoints.swap(simMeshBindPoints);
            cookingReadArray(attachedStage, desc.collisionMeshPath, UsdGeomTokens->tetVertexIndices, pxrCollIndices);
            warnTetMeshOrientation(desc.collisionMeshPath, pxrCollBindPointsInSim, pxrCollIndices, desc.collisionMeshLeftHandedOrientation);
            if (desc.collisionMeshLeftHandedOrientation)
            {
                switchTetsOrientation(pxrCollIndices);
            }

            if (pxrSimBindPoints.size() != pxrSimPoints.size())
            {
                CARB_LOG_ERROR("Cooking failed, sim mesh bind pose points incompatible with points: %s", bodyPath.GetText());
                return false;
            }
        }

        // read surface face vertices be from the collsion mesh, even if sim and coll mesh alias
        {
            cookingReadArray(attachedStage, desc.collisionMeshPath, UsdGeomTokens->surfaceFaceVertexIndices, pxrCollSurfaceIndices);
            if (pxrCollSurfaceIndices.size() == 0)
            {
                CARB_LOG_WARN("Cooking failed, collision mesh UsdGeomTetMesh needs to have "
                              "surfaceFaceVertexIndices set, %s.", desc.collisionMeshPath.GetText());
                return false;
            }
        }

        GfMatrix4d simToCookingTransform;
        if (!::computeDeformableCookingTransform(&simToCookingTransform, nullptr, nullptr, simToWorld,
                                                 pxrSimBindPoints.size() > 0 ? pxrSimBindPoints : pxrSimPoints))
        {
            return false;
        }

        params.simPoints = { reinterpret_cast<carb::Float3*>(pxrSimPoints.data()), pxrSimPoints.size() };
        params.simBindPoints = { reinterpret_cast<carb::Float3*>(pxrSimBindPoints.data()), pxrSimBindPoints.size() };
        params.simIndices = { reinterpret_cast<carb::Int4*>(pxrSimIndices.data()), pxrSimIndices.size() };
        params.collBindPointsInSim = { reinterpret_cast<carb::Float3*>(pxrCollBindPointsInSim.data()), pxrCollBindPointsInSim.size() };
        params.collIndices = { reinterpret_cast<carb::Int4*>(pxrCollIndices.data()), pxrCollIndices.size() };
        params.collSurfaceIndices = { reinterpret_cast<carb::Int3*>(pxrCollSurfaceIndices.data()), pxrCollSurfaceIndices.size() };
        static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
        memcpy(params.simToCookingTransform, simToCookingTransform.data(), sizeof(params.simToCookingTransform));

        return true;
    }

    bool cookDeformableVolumeMeshInternal(::physx::PxDefaultMemoryOutputStream& outStream,
                                          const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          omni::physics::parse::ObjectKey bodyKey,
                                          const omni::physx::usdparser::AttachedStage& attachedStage,
                                          bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        const PXR_NS::SdfPath bodyPath = attachedStage.pathFor(bodyKey);
        if (!bodyKey.valid() || bodyPath.IsEmpty())
            return false;

        omni::physx::DeformableVolumeMeshCookingParams params;
        VtArray<GfVec3f> pxrSimPoints;
        VtArray<GfVec3f> pxrSimBindPoints;
        VtArray<GfVec4i> pxrSimIndices;
        VtArray<GfVec3f> pxrCollBindPointsInSim;
        VtArray<GfVec4i> pxrCollIndices;
        VtArray<GfVec3i> pxrCollSurfaceIndices;
        if (!setupDeformableVolumeMeshCookingParams(&attachedStage, bodyKey, desc, params, pxrSimPoints, pxrSimBindPoints, pxrSimIndices,
                                                    pxrCollBindPointsInSim, pxrCollIndices, pxrCollSurfaceIndices))
        {
            CARB_LOG_ERROR(
                "Deformable volume mesh, failed to setup cooking params, prim: %s", bodyPath.GetText());
            return false;
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.primStageId = uint64_t(attachedStage.getStageId());
        request.primId = 0; // request without input source
        request.deformablePathInfo.bodyPrimId = asInt(bodyPath);
        request.deformablePathInfo.simMeshPrimId = asInt(desc.simMeshPath);
        request.deformablePathInfo.collMeshPrimId = asInt(desc.collisionMeshPath);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);

        bool resultSynchronous = false;
        auto weakPtrToThis = TfCreateWeakPtr(this);
        recordStatisticsRequestFor(request);
        request.onFinished = [&resultSynchronous,
                                     &outStream, weakPtrToThis](const omni::physx::PhysxCookingComputeResult& result) {
            if (!weakPtrToThis)
            {
                return; // means that CookingDataAsync destructor was called, so this task is cancelled
            }
            weakPtrToThis->recordStatisticsResultFor(result);
            if (result.result != omni::physx::PhysxCookingResult::eVALID)
            {
                return;
            }
            if (result.isSynchronousResult)
            {
                outStream.write(result.cookedData[0].data, (PxU32)result.cookedData[0].sizeInBytes);
                resultSynchronous = true;
            }
        };
        m_cookingServicePrivate.requestDeformableVolumeMeshCookedData(m_asyncContext, request, params);
        return resultSynchronous;
    }

    /**
    * Helper method, that creates a PxDeformableVolumeMesh from the input data stream and parses and parses the surface triangle
    * to tetrahedron map.
    * 
    * @param outCollMeshSurfaceTriToTetMap : output collision mesh surface triangle to tetrahedron map
    * @param inData : input data (@see cookDeformableVolumeMesh)
    *
    * @return : Returns the PxDeformableVolumeMesh instance. The caller of the method is responsible for releasing the PxDeformableVolumeMesh.
    */
    ::physx::PxDeformableVolumeMesh* createDeformableVolumeMesh(
        std::vector<uint32_t>& outCollMeshSurfaceTriToTetMap,
        ::physx::PxDefaultMemoryInputData& inData)
    {
        ::physx::PxDeformableVolumeMesh* deformableVolumeMesh = mPhysics.createDeformableVolumeMesh(inData);

        // read the appended surface triangle to tetrahedron map from the cooked data
        if (inData.tell()+sizeof(uint32_t) <= inData.getLength())
        {
            uint32_t surfaceTriangleCount;
            inData.read(&surfaceTriangleCount,sizeof(uint32_t));
            outCollMeshSurfaceTriToTetMap.resize(surfaceTriangleCount);
            inData.read(outCollMeshSurfaceTriToTetMap.data(),surfaceTriangleCount*sizeof(uint32_t));
        }

        // convert map to physx tets
        {
            const uint32_t* remap = deformableVolumeMesh->getCollisionMesh()->getTetrahedraRemap();
            const uint32_t numTets = deformableVolumeMesh->getCollisionMesh()->getNbTetrahedrons();
            std::vector<uint32_t> remapInv(numTets);
            for (uint32_t i = 0; i < numTets; ++i)
            {
                remapInv[remap[i]] = i;
            }
            for (size_t i = 0; i < outCollMeshSurfaceTriToTetMap.size(); ++i)
            {
                outCollMeshSurfaceTriToTetMap[i] = remapInv[outCollMeshSurfaceTriToTetMap[i]];
            }
        }
        return deformableVolumeMesh;
    }

    /**
     *  Samples particles on a mesh using poisson sampling.
     * 
     *  @param primKey : The UsdGeomMesh/ObjectKey we are sampling.
     *  @param asynchronous : If false, it will sample the mesh synchronously (blocking). If true, it will start a background cooking task for it.
     */
    virtual void poissonSampleMesh(omni::physics::parse::ObjectKey primKey, const omni::physx::usdparser::AttachedStage& attachedStage, const omni::physx::usdparser::ParticleSamplingDesc& desc, bool forceResampling, bool asynchronous) final
    {
        // Block USD notification handlers while in this call
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        if (!primKey.valid() || attachedStage.pathFor(primKey).IsEmpty())
            return;
        poissonSampleMeshInternal(primKey, attachedStage, desc, forceResampling, asynchronous);
    }

    void poissonSampleMeshInternal(omni::physics::parse::ObjectKey primKey,
                                   const omni::physx::usdparser::AttachedStage& attachedStage,
                                   const omni::physx::usdparser::ParticleSamplingDesc& desc,
                                   bool forceResampling,
                                   bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        const SdfPath primPath = attachedStage.pathFor(primKey);
        if (primPath.IsEmpty() || !cookingIsA<UsdGeomMesh>(&attachedStage, primPath))
            return;

        omni::physx::ParticlePoissonSamplingCookingParams params;
        GfMatrix4d rigidTransform;
        if (!setupParticlePoissonSamplingCookingParams(primPath, desc, rigidTransform, params))
        {
            CARB_LOG_ERROR("Particle sampler, failed to setup cooking params, prim: %s", primPath.GetText());
            return;
        }

        omni::physx::usdparser::MeshKey originalCrc;
        cookingLoadMeshKey(&attachedStage, primPath, particleSamplingCrcToken, originalCrc);
        if (!forceResampling)
        {
            // compute data CRC synchronously and exit if the corresponding source value matches.
            // if forceResampling is off, we skip the early out to call processSamplingResults with
            // registerOriginalCount == true
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = uint64_t(attachedStage.getStageId());
            request.primId = asInt(primPath);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
            request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
            omni::physx::usdparser::MeshKey cookedDataCRC;
            request.onFinished = [&cookedDataCRC](const omni::physx::PhysxCookingComputeResult& result) {
                cookedDataCRC = result.cookedDataCRC;
            };
            m_cookingServicePrivate.requestParticlePoissonSamplingCookedData(m_asyncContext, request, params);
            if (cookedDataCRC == originalCrc)
            {
                return;
            }
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.primStageId = uint64_t(attachedStage.getStageId());
        request.primId = asInt(primPath);
        request.primMeshText = { primPath.GetText(), strlen(primPath.GetText()) };
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);

        auto weakPtrToThis = TfCreateWeakPtr(this);
        recordStatisticsRequestFor(request);
        request.onFinished = [desc, params, rigidTransform,
                              originalCrc, weakPtrToThis](const omni::physx::PhysxCookingComputeResult& result) {
            if(!weakPtrToThis)
            {
                return; // means that CookingDataAsync destructor was called, so this task is cancelled
            }
            weakPtrToThis->recordStatisticsResultFor(result);

            if (result.result != omni::physx::PhysxCookingResult::eVALID)
            {
                return;
            }

            omni::physx::IPhysxCookingServicePrivate& cookingService = weakPtrToThis->m_cookingServicePrivate;
            omni::physx::usdparser::AttachedStage* as =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->getAttachedStage(long(result.request->primStageId));
            if (!as)
            {
                return;
            }

            const SdfPath primPath = intToPath(result.request->primId);
            const omni::physics::parse::ObjectKey samplerKey = as->keyFor(primPath);
            const omni::physics::parse::IPhysicsSource* source = as->getSource();
            if (!(source && omni::physx::internal::hasAppliedSchema<PhysxSchemaPhysxParticleSamplingAPI>(*source, samplerKey)))
            {
                return;
            }

            if (result.cookedData)
            {
                omni::physx::PhysxCookingParticlePoissonSamplingData data;
                cookingService.readParticlePoissonSamplingData(data, *result.cookedData);

                // Set the sampling distance through the output sink in case cooking made a correction.
                const omni::physics::parse::TokenId samplingDistanceToken =
                    source->internToken("physxParticleSampling:samplingDistance");
                float samplingDistance = 0.0f;
                if (source->getAttribute(samplerKey, samplingDistanceToken, samplingDistance) &&
                    samplingDistance < desc.samplingDistance && samplingDistance > 0.0f)
                {
                    if (omni::physics::parse::IPhysicsDataWrite* dataWrite = as->getDataWrite())
                    {
                        omni::physics::parse::DataWriteView view;
                        view.data = &desc.samplingDistance;
                        view.count = 1;
                        view.stride = 0;
                        view.device = -1;
                        view.type = omni::physics::parse::DataType::e32Bit;
                        dataWrite->beginWrite();
                        dataWrite->writeData(&samplerKey, 1, samplingDistanceToken, view);
                        dataWrite->endWrite();
                    }
                }

                if (auto* usdDataWrite = omni::physics::usd::asUsdDataWrite(as->getDataWrite()))
                {
                    usdDataWrite->writeUCharArrayAttribute(samplerKey, particleSamplingCrcToken,
                                                           reinterpret_cast<const uint8_t*>(&result.cookedDataCRC),
                                                           sizeof(result.cookedDataCRC));
                }

                const GfVec3f* samples = reinterpret_cast<const GfVec3f*>(data.positions);
                const uint32_t samplesSize = data.positionsSize;
                static_assert(sizeof(GfMatrix3d) == sizeof(params.shearScale));
                const GfMatrix3d& shearScaleTransform = *reinterpret_cast<const GfMatrix3d*>(params.shearScale);

                // if the original source crc matches with the newly computed crc we call processSamplingResults
                // for registering particle counts of newly instantiated samplers without writing particles, in
                // order to avoid overwriting pre-simulated state.
                bool registerOriginalCount = (originalCrc == result.cookedDataCRC);

                omni::physx::particles::PhysxParticleFactory::processSamplingResults(
                    primPath, desc.particleSetPath, samples, samplesSize,
                    desc.pointWidth, rigidTransform, shearScaleTransform,
                    registerOriginalCount);
            }
        };
        m_cookingServicePrivate.requestParticlePoissonSamplingCookedData(m_asyncContext, request, params);
    }

    virtual void release(void) final
    {
        delete this;
    }

    /**
    * Register this cooking driver's interest on an attached stage's change feed
    * (ADR-0003). Called by AttachedStage when it (re)creates its feed, so the
    * registration exists from attach time — no dependency on pump() having run.
    * The registration is owned by the per-stage feed and dies with it on
    * detach/source-rebuild, so there is nothing to revoke. A single wildcard
    * interest delivers every batch to handleSourceChange.
    */
    virtual void registerOnChangeFeed(omni::physx::usdparser::AttachedStage& attachedStage) final
    {
        omni::physics::parse::IChangeFeed* feed = attachedStage.getChangeFeed();
        omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!feed || !src)
            return;

        // `as`/`src` stay valid for the feed's lifetime (the feed is owned by the
        // AttachedStage and owns this lambda), so they never dangle within a live
        // callback. `self` is weak so cooking-driver teardown is safe.
        omni::physx::usdparser::AttachedStage* as = &attachedStage;
        auto self = TfCreateWeakPtr(this);
        feed->registerInterest(
            omni::physics::parse::ObjectKey{}, omni::physics::parse::TokenId{}, -1,
            [self, as, src](const omni::physics::parse::ChangeBatch& batch)
            {
                if (self)
                    self->handleSourceChange(batch, *as, *src);
            },
            0);
    }

    /**
    * Per-batch change handler (replaces the legacy UsdNotice listener `handle`).
    * Schedules recooks from source-delivered ChangeBatches: structural batches
    * (resync / delete) feed the api-schema / added-removed sets; value batches are
    * matched by attribute name against the collision-token / xform-attr gates.
    * Everything is keyed by ObjectKey and converted to SdfPath only at the
    * (SdfPath-based) refresh-set boundary.
    */
    void handleSourceChange(const omni::physics::parse::ChangeBatch& batch,
                            omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physics::parse::IPhysicsSource& src)
    {
        TRACE_FUNCTION();
        CARB_PROFILE_ZONE(0, "CookingDataAsync::changeFeedHandler");

        // Ignore changes we are making ourselves during write-back.
        if (m_blockUsdUpdate > 0)
            return;
        if (batch.keys.type != omni::physics::parse::ColumnType::eObjectKey || !batch.keys.data ||
            batch.keys.count == 0)
            return;
        const omni::physics::parse::ObjectKey* keys =
            static_cast<const omni::physics::parse::ObjectKey*>(batch.keys.data);
        const size_t keyCount = batch.keys.count;

        lock_guard _lock(m_mutex);

        if (!batch.property.valid())
        {
            // Structural change: object removed, or a resync whose changed fields
            // include apiSchemas (vs a plain add / structural edit).
            bool apiSchemasChanged = false;
            if (!batch.isDelete && batch.values.type == omni::physics::parse::ColumnType::eToken &&
                batch.values.data)
            {
                const omni::physics::parse::TokenId* fields =
                    static_cast<const omni::physics::parse::TokenId*>(batch.values.data);
                for (size_t j = 0; j < batch.values.count; ++j)
                {
                    if (src.tokenToString(fields[j]) == UsdTokens->apiSchemas.GetString())
                    {
                        apiSchemasChanged = true;
                        break;
                    }
                }
            }

            for (size_t i = 0; i < keyCount; ++i)
            {
                const omni::physics::parse::ObjectKey key = keys[i];
                const SdfPath primKey = attachedStage.pathFor(key);
                if (primKey.IsEmpty())
                    continue;

                if (apiSchemasChanged)
                {
                    // Schedule the api-schema scan and register the deformable pose
                    // instances' point/purpose attrs as collision tokens (source-
                    // enumerated; replaces prim.GetAppliedSchemas()).
                    m_primApiSchemasChangeRefreshSet.insert(primKey);
                    src.forEachMultiApplyInstance(
                        key, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformablePoseAPI.GetString(),
                        [&](std::string_view instance)
                        {
                            const TfToken inst{ std::string(instance) };
                            m_collisionTokens.insert(UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                                OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, inst));
                            m_collisionTokens.insert(UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                                OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPurposes, inst));
                        });
                }
                else
                {
                    m_primAddedRemovedRefreshSet.insert(primKey);
                    // Be defensive: we don't know exactly what changed, so wipe the
                    // mesh-key cache to keep keys correct.
                    omni::physx::usdparser::notifyStageReset();
                }
            }
        }
        else
        {
            // Value change on a property: classify by attribute name, matching the
            // legacy collision-token / xform-attr gates.
            const TfToken attrName{ std::string(src.tokenToString(batch.property)) };
            const bool isCollision = m_collisionTokens.find(attrName) != m_collisionTokens.cend();
            const bool isXform = !isCollision && UsdGeomXformable::IsTransformationAffectedByAttrNamed(attrName);
            if (!isCollision && !isXform)
                return;
            for (size_t i = 0; i < keyCount; ++i)
            {
                const SdfPath primKey = attachedStage.pathFor(keys[i]);
                if (primKey.IsEmpty())
                    continue;
                if (isCollision)
                    addPrimRefreshSet(primKey);
                else
                    m_primXformRefreshSet.insert(primKey);
            }
        }
    }

    /**
    * Add this path to the list of paths that might need to be recooked
    *
    * @param path : USD prim path that should be inspected to see if it needs to be recooked (because either the cooking data is missing or the MeshKey (hash) has changed).
    */
    virtual void addPrimRefreshSet(const SdfPath &path) final
    {
        m_primRefreshSet.insert(path);
        omni::physx::usdparser::invalidateMeshKeyCache(path);
    }

    /**
    * Returns not only the number of active tasks but those which we have scheduled to be inspected due to property changes as well.
    *
    * @return : Returns the sum of the number of pending cooking tasks plus the number of paths we want to re-inspect
    */
    virtual uint32_t getActiveTaskCount(void) final
    {
        uint32_t ret = 0;
        lock_guard _lock(m_mutex);
        ret += m_cookingServicePrivate.getActiveTaskCount(m_asyncContext);
        ret += uint32_t( m_primRefreshSet.size() );
        ret += uint32_t( m_primXformRefreshSet.size() );
        return ret;
    }

    /**
    * Mark all cooking tasks as being canceled. Their results will be thrown away.
    *
    * @return : Returns the number of active tasks which were marked for cancelation
    */
    virtual uint32_t cancelAllTasks(void) final
    {
        uint32_t res = getComputeService().cancelAllTasks(m_asyncContext);
        m_cookingStatistics = {};
        return res;
    }

    /**
    * Increment or decrement the block USD update counter.
    */
    virtual void blockUSDUpdate(bool blocked) final
    {
        if ( blocked )
        {
            m_blockUsdUpdate++;
        }
        else
        {
            CARB_ASSERT(m_blockUsdUpdate);
            if ( m_blockUsdUpdate > 0 )
            {
                m_blockUsdUpdate--;
            }
        }
    }

    /**
    * Returns the total number of cooking tasks which have been performed since the
    * start of the application. This is used by debug visualization (omni.physx.ui) to
    * know whether or not it should refresh the debug visualization of a primitive because
    * the cooking state has changed since the last time.
    */
    virtual uint32_t getFinishedCookingTasksCount(void) const final
    {
        return m_cookingServicePrivate.getFinishedCookingTasksCount();
    }


    /**
    * Retrieves the unique 128 hash associated with this descriptor.  Takes into account the
    * cooking data version number as well as the signed scale. This method is used by
    * the solid shaded debug visualization to know when it needs to change the debug
    * visualization primitives on property change events.
    *
    * @param desc : The shape descriptor for this primitive.
    * @param meshKey : A reference to return the 128 bit unique hash key corresponding to the cooked data.
    *
    * @return : Returns true if we could generate a hash key from this shape descriptor (only applies to convex hull, triangle mesh, and convex decomposition) other shape types are ignored as they don't have support for the solid shaded debug visualization.
    */
    virtual bool getMeshkey(const omni::physx::usdparser::PhysxShapeDesc& desc,
        omni::physx::usdparser::MeshKey &crc) final
    {
        bool found = false;

        switch (desc.type)
        {
            case omni::physx::usdparser::eConvexMeshShape:
                {
                    const omni::physx::usdparser::ConvexMeshPhysxShapeDesc* convexDesc = (const omni::physx::usdparser::ConvexMeshPhysxShapeDesc*)&desc;
                    found = true;
                    crc = convexDesc->crc;
                }
            break;
            case omni::physx::usdparser::eConvexMeshDecompositionShape:
                {
                    const omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionDesc = (const omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc*)&desc;
                    found = true;
                    crc = convexDecompositionDesc->crc;
                }
                break;
            case omni::physx::usdparser::eTriangleMeshShape:
                {
                    const omni::physx::usdparser::TriangleMeshPhysxShapeDesc* meshDesc = (const omni::physx::usdparser::TriangleMeshPhysxShapeDesc*)&desc;
                    found = true;
                    crc = meshDesc->crc;
                }
                break;
        }
        return found;
    }

    /**
    * Returns the solid shaded debug visualization triangle mesh for the cooked collision
    * representation of this asset. In omni.physx.ui debug visualization we now support
    * the ability to visualize as a solid shaded mesh (represented as a UsdGeomMesh instance(s)
    * on the session layer). In this way the user can get a clear visualization of what the
    * collision representation of the source primitive actually looks like.
    * When a collision mesh is cooked a copy of the debug visualization representation is
    * also stored into the localcache so that it can be easily retrieved here.
    * Since a single UsdPrim can have (n) number of collision meshes (in the case of a
    * convex decomposition) it is possible to get more than just one for the source UsdPrim.
    *
    * @param path : The path of the primitive we are referring to
    * @param desc : The shape descriptor for this UsdPrim
    *
    * @return : If a graphics collision representation exists, it will return a pointer to it. 
    */
    virtual const omni::physx::CollisionRepresentation *getCollisionRepresentation(const SdfPath &path,
                                                                                   const omni::physx::usdparser::PhysxShapeDesc& desc) final
    {

        omni::physx::usdparser::MeshKey crc;
        if (getMeshkey(desc, crc))
        {
            return omni::physx::getMeshCache()->getCollisionRepresentation(crc, desc);
        }
        return nullptr;
    }

    /**
    * Release a previously queried collision representation.
    *
    * @param cr : A pointer to a previously retrieved collision representation. Each call to 'getCollisionRepresentation' should be paired with a call to 'releaseCollisionRepresentation' or you will get a memory leak.
    */
    virtual void releaseCollisionRepresentation(const omni::physx::CollisionRepresentation *cr) final
    {
        if ( cr )
        {
            omni::physx::CollisionRepresentation *c = (omni::physx::CollisionRepresentation *)cr;
            for (uint32_t i=0; i<c->meshCount; i++)
            {
                omni::physx::CollisionMesh &m = cr->meshes[i];
                delete []m.vertices;
                delete []m.indices;
            }
            delete []c->meshes;
            delete c;
        }
    }


    physx::PxPhysics& mPhysics;

    std::atomic_int32_t m_blockUsdUpdate{0};

    carb::settings::ISettings* m_settings = nullptr;

    PrimRefreshSet      m_primRefreshSet;
    PrimRefreshSet      m_primXformRefreshSet;
    PrimRefreshSet      m_primApiSchemasChangeRefreshSet;
    PrimRefreshSet      m_primAddedRemovedRefreshSet;
    TokenSet            m_collisionTokens;
    carb::tasking::MutexWrapper  m_mutex; // mutex lock for thread safety

    omni::physx::PhysxCookingAsyncContext m_asyncContext = nullptr;

    // Progress Bar
    bool m_progressBarEnabled = false;
    float m_currentProgressBarValue = -1.0f;
    uint32_t m_maxCookingTasks = 0;

    omni::physx::PhysxCookingStatistics m_cookingStatistics;

    virtual omni::physx::PhysxCookingStatistics getCookingStatistics() const override
    {
        return m_cookingStatistics;
    }

    void recordStatisticsRequestFor(const omni::physx::PhysxCookingComputeRequest& request)
    {
       m_cookingStatistics.totalScheduledTasks += 1;
    }

    void recordStatisticsResultFor(const omni::physx::PhysxCookingComputeResult& result)
    {
        m_cookingStatistics.totalFinishedTasks += 1;
        if(m_cookingStatistics.totalFinishedTasks > m_cookingStatistics.totalScheduledTasks)
        {
            CARB_LOG_ERROR("Cooking statistics callback has been called unexpected number of times");
            m_cookingStatistics = {};
        }
        else
        {
            if(result.resultSource == omni::physx::PhysxCookingComputeResult::eRESULT_CACHE_MISS)
            {
                m_cookingStatistics.totalFinishedCacheMissTasks += 1;
            }
            else
            {
                m_cookingStatistics.totalFinishedCacheHitTasks += 1;
            }
            if(result.resultWarnings.hasFlag(omni::physx::PhysxCookingComputeResult::ResultWarning::FAILED_GPU_COMPATIBILITY))
            {
                m_cookingStatistics.totalWarningsFailedGPUCompatibility += 1;
            }
            if(result.resultWarnings.hasFlag(omni::physx::PhysxCookingComputeResult::ResultWarning::CONVEX_POLYGON_LIMITS_REACHED))
            {
                m_cookingStatistics.totalWarningsConvexPolygonLimitsReached += 1;
            }
        }
    }

    /**
     * Based on the number of active cooking tasks we refresh the settings which are reflected in the Kit cooking
     * progress bar
     */
    void refreshProgressBarStatus(void)
    {
        const uint32_t taskCount = m_cookingServicePrivate.getActiveTaskCount(m_asyncContext);
        // m_primRefreshSet are tasks that have not yet been submitted to the queue
        const uint32_t totalTaskCount = taskCount + uint32_t(m_primRefreshSet.size());
        if (totalTaskCount)
        {
            if (totalTaskCount > m_maxCookingTasks)
            {
                m_maxCookingTasks = totalTaskCount;
            }
            const float value = 1.0f - (float(totalTaskCount) / float(m_maxCookingTasks));
            if (!m_progressBarEnabled)
            {
                m_settings->setBool(PROGRESS_BAR_ENABLED, true);
                m_settings->setString(PROGRESS_BAR_LABEL, "Physics Tasks");
                m_progressBarEnabled = true;
            }
            if (value != m_currentProgressBarValue)
            {
                m_currentProgressBarValue = value;
                m_settings->setFloat(PROGRESS_BAR_VALUE, value);
            }
        }
        else
        {
            if (m_progressBarEnabled)
            {
                m_currentProgressBarValue = -1;
                m_progressBarEnabled = false;
                m_maxCookingTasks = 0; // reset once all cooking tasks have been exhausted
                m_settings->setBool(PROGRESS_BAR_ENABLED, false);
            }
        }
    }


    virtual void resetLocalMeshCacheContents() override final
    {
        m_cookingServicePrivate.resetMeshCacheContents();
    }

    virtual omni::physx::PhysxCookingAsyncContext getCookingAsyncContext()
    {
        return m_asyncContext;
    }

    omni::physx::IPhysxCookingService& getComputeService(){return m_cookingService; }
    omni::physx::IPhysxCookingServicePrivate& m_cookingServicePrivate;
    omni::physx::IPhysxCookingService& m_cookingService;
};


// Create an instance of the CookingDataAsync implementation class
CookingDataAsync* createCookingDataAsync(physx::PxPhysics& physics, omni::physx::IPhysxCookingServicePrivate& cookingServicePrivate, omni::physx::IPhysxCookingService& cookingService, omni::physx::PhysxCookingAsyncContext context)
{
    auto ret = new CookingDataAsyncImpl(physics, cookingServicePrivate, cookingService, context);
    return static_cast< CookingDataAsync *>(ret);
}

}

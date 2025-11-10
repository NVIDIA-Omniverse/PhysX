// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
#include "PhysXDebugVisualization.h"    // CollisionRepresentation
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Collision.h"
#include "usdLoad/PhysicsBody.h"

#include "particles/PhysXParticleSampling.h"

#include <regex>

// Set this to 1 to enable the USD notice listener to start async tasks
#define USD_USD_NOTICE_LISTENER 1 // Listens for collision related attribute changes to automatically spawn background cooking tasks if needed
#define USE_ASYNC_COOKING 1 // By default we support asynchronous cooking, this is only here for debugging purposes
#define REPORT_PROGRESS 0 // A debug option only, used to track cooking task progress

static constexpr const char* PROGRESS_BAR_ENABLED = "/physics/progressBarEnabled";
static constexpr const char* PROGRESS_BAR_LABEL = "/physics/progressBarLabel";
static constexpr const char* PROGRESS_BAR_VALUE = "/physics/progressBarValue";

// Scoped mutex lock
using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;
using namespace ::physx;
using namespace pxr;

namespace
{
    UsdAttribute getPosePointsAttr(UsdPrim posePrim, const TfType& poseType, TfToken instanceName)
    {
        if (instanceName.IsEmpty())
            return UsdAttribute();

        if (!posePrim.HasAPI(poseType, instanceName))
        {
            CARB_LOG_ERROR("Expected UsdPhysicsDeformablePoseAPI instance %s on %s, but not found.",
                           instanceName.GetText(), posePrim.GetPath().GetText());
            return UsdAttribute();
        }

        TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
            OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, instanceName);
        return posePrim.GetAttribute(attrName);
    }

    UsdAttribute getPosePointsOrPointsAttr(UsdPrim posePrim, const TfType& poseType, TfToken instanceName)
    {
        if (!instanceName.IsEmpty())
        {
            return getPosePointsAttr(posePrim, poseType, instanceName);
        }
        return UsdGeomPointBased(posePrim).GetPointsAttr();
    }

    UsdPrim findDeformableBodyAncestor(UsdPrim prim)
    {
        TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        UsdPrim deformableBodyPrim = UsdPrim();
        while (prim && !deformableBodyPrim)
        {
            if (prim.HasAPI(dbType))
            {
                deformableBodyPrim = prim;
            }
            else if (prim.HasAPI<UsdPhysicsRigidBodyAPI>())
            {
                prim = UsdPrim();
            }
            else
            {
                UsdGeomXformable xformable(prim);
                bool resetXformStack = xformable && xformable.GetResetXformStack();
                prim = resetXformStack ? UsdPrim() : prim.GetParent();
            }
        }
        return deformableBodyPrim;
    }

    pxr::GfVec3d computeQuantizedDir(const pxr::GfVec3d& dir)
    {
        double eps = 1e-7;
        double eps_inv = 1.0 / eps;

        pxr::GfVec3d dirQuant(
            std::round(dir[0] * eps_inv) * eps, std::round(dir[1] * eps_inv) * eps, std::round(dir[2] * eps_inv) * eps);

        return dirQuant.GetNormalized();
    }

    pxr::GfRotation computeQuantizedRotation(const pxr::GfRotation& rotation)
    {
        double eps = 1e-7;
        double eps_inv = 1.0 / eps;

        double angleUnit = rotation.GetAngle() / 360.0;
        double angleUnitQuant = std::round(angleUnit * eps_inv) * eps;
        double angleQuant = angleUnitQuant * 360.0;

        return pxr::GfRotation(computeQuantizedDir(rotation.GetAxis()), angleQuant);
    }

    pxr::GfTransform computeQuantizedSkewTransform(double& scaleAbs, const pxr::GfTransform& transformSkew)
    {
        pxr::GfVec3d scaleNormalized = transformSkew.GetScale();
        scaleAbs = scaleNormalized.Normalize();

        pxr::GfTransform transformSkewQuant(pxr::GfVec3d(0.0), computeQuantizedRotation(transformSkew.GetRotation()),
                                            computeQuantizedDir(scaleNormalized), pxr::GfVec3d(0.0),
                                            computeQuantizedRotation(transformSkew.GetPivotOrientation()));

        return transformSkewQuant;
    }

    void computeFitBounds(pxr::GfVec3d& translation,
                          double& scale,
                          const pxr::VtArray<pxr::GfVec3f>& points,
                          const pxr::GfTransform& transform)
    {
        pxr::GfMatrix4d m = transform.GetMatrix();

        pxr::GfRange3d bounds;
        for (const pxr::GfVec3d& point : points)
        {
            bounds.UnionWith(m.Transform(point));
        }
        pxr::GfVec3d dims = bounds.GetSize();
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
        pxr::GfMatrix4d simToWorldOrtho = simToWorld;
        bool orthonormalized = simToWorldOrtho.Orthonormalize(false);

        if (!orthonormalized)
        {
            return false;
        }

        pxr::GfMatrix4d simToWorldSkew = simToWorld * simToWorldOrtho.GetInverse();

        double scaleAbs;
        pxr::GfTransform skew(simToWorldSkew);
        pxr::GfTransform skewQuant = computeQuantizedSkewTransform(scaleAbs, skew);

        pxr::GfVec3d fbTrans;
        double fbScale;
        computeFitBounds(fbTrans, fbScale, boundsFitPoints, skewQuant);

        pxr::GfRotation pivotOrient = skewQuant.GetPivotOrientation();
        pxr::GfRotation rotation = skewQuant.GetRotation();
        pxr::GfVec3d scale = skewQuant.GetScale();

        if (simToCookingTransform)
        {
            pxr::GfMatrix4d matFbScaleInv;
            matFbScaleInv.SetScale(1.0 / fbScale);
            pxr::GfMatrix4d matFbTransInv;
            matFbTransInv.SetTranslate(-fbTrans);
            pxr::GfMatrix4d matScale;
            matScale.SetScale(scale);
            pxr::GfMatrix4d matOrientInv;
            matOrientInv.SetTransform(pivotOrient.GetInverse(), pxr::GfVec3d(0.0));
            *simToCookingTransform = matOrientInv * matScale * matFbTransInv * matFbScaleInv;
        }

        if (cookingToWorldTransform)
        {
            // this is the corresponding transform from cooking space to world space (ignoring the pre scale factor:
            // scaleAbs*fbScale)
            pxr::GfMatrix4d rigid;
            {
                pxr::GfMatrix4d matRot(pivotOrient * rotation, pxr::GfVec3d(0.0));
                pxr::GfMatrix4d matTrans(pxr::GfRotation(pxr::GfVec3d(1, 0, 0), 0.0), fbTrans * scaleAbs);
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
    omni::physx::usdparser::MeshKey computeMeshKey(const pxr::VtArray<pxr::GfVec3f>& points,
                                                   const pxr::VtArray<int32_t>& vertexIndices,
                                                   const pxr::VtArray<int32_t>& vertexCounts)
    {
        omni::physx::usdparser::MeshKey meshKey;
        meshKey.computeVerticesHash(uint32_t(points.size()), reinterpret_cast<const float*>(points.data()));
        meshKey.computeIndicesHash(
            uint32_t(vertexIndices.size()), reinterpret_cast<const uint32_t*>(vertexIndices.data()));
        meshKey.computeIndicesHash(uint32_t(vertexCounts.size()), reinterpret_cast<const uint32_t*>(vertexCounts.data()));
        return meshKey;
    }

    /**
     * Convenience method to return the local to world scale from a UsdGeomXformable
     */
    GfVec3d getLocalToWorldScale(const UsdGeomXformable& xformable)
    {
        GfMatrix4d localToWorld = xformable.ComputeLocalToWorldTransform(UsdTimeCode::Default());
        return GfTransform(localToWorld).GetScale();
    }

    /**
     * Normalizes the input scale vector and discretizes it by snapping it to a grid with 1e-5f spacing.
     * This is useful to identify if two scale vectors are approximately 'similar' to each other, i.e. whether s1 =
     * a*s2, holds, 'a' being an scalar.
     */
    GfVec3f getNormalizedQuantizedDeprecated(const GfVec3d scale, float scaleFactor)
    {
        GfVec3d scaleNormalized = scale.GetNormalized() * scaleFactor;

        constexpr double eps = 1e-5;
        constexpr double eps_inv = 1.0 / eps;

        double x = std::round(scaleNormalized[0] * eps_inv);
        double y = std::round(scaleNormalized[1] * eps_inv);
        double z = std::round(scaleNormalized[2] * eps_inv);
        return GfVec3f(float(x * eps), float(y * eps), float(z * eps));
    }

} // namespace

namespace cookingdataasync
{
// These are the collision related attributes that we track in the USD listener.
// If any of these attributes change we *might* have to recook the asset.

// DEPRECATED
static const TfToken deformableEnableKinematicToken("physxDeformable:kinematicEnabled");
static const TfToken deformableCollisionSimplificationToken("physxDeformable:collisionSimplification");
static const TfToken deformableCollisionSimplificationRemeshingToken("physxDeformable:collisionSimplificationRemeshing");
static const TfToken deformableCollisionSimplificationRemeshingResolutionToken("physxDeformable:collisionSimplificationRemeshingResolution");
static const TfToken deformableCollisionSimplificationTargetTriangleCountToken("physxDeformable:collisionSimplificationTargetTriangleCount");
static const TfToken deformableCollisionSimplificationForceConformingToken("physxDeformable:collisionSimplificationForceConforming");
static const TfToken deformableSimulationHexahedralResolutionToken("physxDeformable:simulationHexahedralResolution");
static const TfToken deformableNumberOfTetsPerHexToken("physxDeformable:numberOfTetsPerHex");
static const TfToken deformableTetMeshCrcToken("physxDeformable:tetMeshCrc");
static const TfToken deformableCollisionVertexToSimulationTetIndicesToken("physxDeformable:collisionVertexToSimulationTetIndices");
static const TfToken deformableCollisionVertexToSkinTriVertexIndicesToken("physxDeformable:collisionVertexToSkinTriVertexIndices");
static const TfToken deformableCollisionVertexToSkinTriBarycentricsToken("physxDeformable:collisionVertexToSkinTriBarycentrics");
static const TfToken invMassScaleAttributeName("physxDeformable:invMassScale");
static const TfToken physxParticleClothInputCrcToken{ "physxParticle:clothDataInputCrc" };
static const TfToken physxParticleWeldedTriangleIndicesToken{ "physxParticle:weldedTriangleIndices" };
static const TfToken physxParticleWeldedVerticesRemapToWeldToken{ "physxParticle:weldedVerticesRemapToWeld" };
static const TfToken physxParticleWeldedVerticesRemapToOrigToken{ "physxParticle:weldedVerticesRemapToOrig" };
static const TfToken physxParticleInflatableVolumeToken{ "physxParticle:inflatableVolume" };

static const TfToken particleSamplingCrcToken{ "physxParticleSampling:crc" };
static const TfToken deformableBodyDataCrcToken("physxDeformableBody:deformableBodyDataCrc");


// Declare an unordered set of SdfPath names.
using PrimRefreshSet = std::unordered_set< SdfPath, SdfPath::Hash>;

using MeshKeySet = std::unordered_set< omni::physx::usdparser::MeshKey, omni::physx::usdparser::MeshKeyHash >;

// Declare a set of TfTokens
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

// The implementation class for the CookingDataAsync API
class CookingDataAsyncImpl : public CookingDataAsync, public TfWeakBase
{
public:
    CookingDataAsyncImpl(physx::PxPhysics& physics, omni::physx::IPhysxCookingServicePrivate& cookingServicePrivate, omni::physx::IPhysxCookingService& cookingService, omni::physx::PhysxCookingAsyncContext context):
        mPhysics(physics), m_cookingServicePrivate(cookingServicePrivate), m_cookingService(cookingService), m_asyncContext(context)
    {
        m_settings = carb::getCachedInterface<carb::settings::ISettings>();
#if USD_USD_NOTICE_LISTENER
        // Create something called a 'usd notice listener key' so that we can receive notification events when USD attributes change
        m_usdNoticeListenerKey = TfNotice::Register(TfCreateWeakPtr(this), &CookingDataAsyncImpl::handle);
#endif
        // Build the list of USD collision related attributes that we listen for.
        // If any of these attributes change then it indicates that we *might* have
        // to recook the collision data associated with that primitive.
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
        m_collisionTokens.insert(deformableEnableKinematicToken);
        m_collisionTokens.insert(deformableCollisionSimplificationToken);
        m_collisionTokens.insert(deformableCollisionSimplificationRemeshingToken);
        m_collisionTokens.insert(deformableCollisionSimplificationRemeshingResolutionToken);
        m_collisionTokens.insert(deformableCollisionSimplificationTargetTriangleCountToken);
        m_collisionTokens.insert(deformableCollisionSimplificationForceConformingToken);
        m_collisionTokens.insert(deformableSimulationHexahedralResolutionToken);
        m_collisionTokens.insert(deformableNumberOfTetsPerHexToken);
        m_collisionTokens.insert(deformableTetMeshCrcToken);
        m_collisionTokens.insert(PhysxSchemaTokens->physxParticleRestPoints);
        m_collisionTokens.insert(PhysxSchemaTokens->physxAutoParticleClothSpringStretchStiffness);
        m_collisionTokens.insert(PhysxSchemaTokens->physxAutoParticleClothSpringBendStiffness);
        m_collisionTokens.insert(PhysxSchemaTokens->physxAutoParticleClothSpringShearStiffness);
        m_collisionTokens.insert(PhysxSchemaTokens->physxAutoParticleClothSpringDamping);
        m_collisionTokens.insert(PhysxSchemaTokens->physxParticlePressure);
        m_collisionTokens.insert(PhysxSchemaTokens->physxAutoParticleClothDisableMeshWelding);
        m_collisionTokens.insert(PhysxAdditionAttrTokens->autoDeformableBodyEnabled);
        m_collisionTokens.insert(PhysxAdditionAttrTokens->resolution);
        m_collisionTokens.insert(PhysxAdditionAttrTokens->autoDeformableMeshSimplificationEnabled);
        m_collisionTokens.insert(PhysxAdditionAttrTokens->remeshingEnabled);
        m_collisionTokens.insert(PhysxAdditionAttrTokens->remeshingResolution);
        m_collisionTokens.insert(PhysxAdditionAttrTokens->targetTriangleCount);
        m_collisionTokens.insert(PhysxAdditionAttrTokens->forceConforming);
        m_collisionTokens.insert(deformableBodyDataCrcToken);
    }

    virtual ~CookingDataAsyncImpl(void)
    {
        m_asyncContext = nullptr;
        m_settings->setBool(PROGRESS_BAR_ENABLED, false); // Set the progress bar enabled setting to false
        {
            lock_guard _lock(m_mutex);
    #if USD_USD_NOTICE_LISTENER
            TfNotice::Revoke(m_usdNoticeListenerKey); // Release the previously registered notice handler key
    #endif
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
        // should not trigger other notice handlers, including our own.
        UsdStageWeakPtr stage = omni::physx::OmniPhysX::getInstance().getStage();
        UsdSchemaRegistry& reg = UsdSchemaRegistry::GetInstance();
        TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        TfType dpType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);

        {
            // We grab a mutex lock to be thread safe
            lock_guard _lock(m_mutex);
            ScopedBlockUSDUpdates _block(this);

            for (const auto& primPath : m_primApiSchemasChangeRefreshSet)
            {
                // Enumerate all children and look for collision attributes
                UsdPrim prim = stage->GetPrimAtPath(primPath);
                if (prim.IsValid())
                {
                    auto primRange = UsdPrimRange(prim);

                    for (UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
                    {
                        const UsdPrim& subPrim = *iter;

                        if (!subPrim.IsValid() || subPrim.IsPseudoRoot())
                        {
                            continue;
                        }

                        if (subPrim.HasAPI<UsdPhysicsCollisionAPI>())
                        {
                            // Schedule it to possibly be recooked if needed.
                            addPrimRefreshSet(subPrim.GetPath());
                            break;
                        }
                        else if (subPrim.HasAPI(dbType))
                        {
                            addPrimRefreshSet(subPrim.GetPath());
                            break;
                        }
                        else if (subPrim.HasAPI(dpType))
                        {
                            addPrimRefreshSet(subPrim.GetPath());
                            break;
                        }
                    }
                }
            }

            m_primApiSchemasChangeRefreshSet.clear();

            for (const auto& primPath : m_primAddedRemovedRefreshSet)
            {
                UsdPrim parentPrim = stage->GetPrimAtPath(primPath.GetParentPath());
                if (parentPrim.IsValid())
                {
                    UsdPrim deformableBodyPrim = findDeformableBodyAncestor(parentPrim);
                    if (deformableBodyPrim)
                    {
                        addPrimRefreshSet(deformableBodyPrim.GetPath());
                    }
                }
            }

            m_primAddedRemovedRefreshSet.clear();

            // See if some prim collision properties have changed and we need to parse them.
            // We iterate through the list of primitives which *may* have had collision property
            // changes and we need to see if those property changes required that primitive to
            // be re-cooked
#if USE_ASYNC_COOKING
            // We start timing how much time we spend here so that we can exit
            // This mainly happens when the cooking system finds entries in the local disk cache
            timer.start();
            bool endedPrematurely = false;
            for(auto it = m_primRefreshSet.begin(); it != m_primRefreshSet.end(); )
            {
                CARB_PROFILE_ZONE(0, "CookingDataAsync::pump primRefreshSet");
                const SdfPath primPath = *it;
                UsdPrim prim = stage->GetPrimAtPath(primPath); // Look up the UsdPrim for this path name
                if (prim)
                {
                    bool hasCollisionAPI = prim.HasAPI<UsdPhysicsCollisionAPI>();
                    bool hasDeformablePoseAPI = prim.HasAPI(dpType);
                    bool hasDeformableBodyAPI = prim.HasAPI(dbType);
                    UsdPrim deformableBodyPrim;
                    if (hasDeformableBodyAPI)
                    {
                        deformableBodyPrim = prim;
                    }
                    else if(hasCollisionAPI || hasDeformablePoseAPI)
                    {
                        deformableBodyPrim = findDeformableBodyAncestor(prim);
                    }

                    if (hasCollisionAPI && !deformableBodyPrim)
                    {
                        // A.B. this is not correct anymore for mesh childs of collisionAPI

                        // We invoke the usdparser to retrieve the fully digested 'PhysXShapeDesc' for this primitive.
                        // If the result is non-null we continue to inspect it
                        const uint64_t stageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
                        omni::physx::usdparser::PhysxShapeDesc*shapeDesc = omni::physx::usdparser::parseCollision(stageId, primPath, primPath);
                        if (shapeDesc)
                        {
                            // We examine each shape type that we support cooking for.
                            switch (shapeDesc->type)
                            {
                            case omni::physx::usdparser::ObjectType::eConvexMeshShape:
                            {
                                // If it is a convex mesh shape, we spawn a background cooking task for it (if needed)
                                omni::physx::usdparser::ConvexMeshPhysxShapeDesc *desc = static_cast<omni::physx::usdparser::ConvexMeshPhysxShapeDesc *>(shapeDesc);
                                getConvexMeshInternal(*desc, prim, true, nullptr);
                            }
                            break;
                            case omni::physx::usdparser::ObjectType::eSpherePointsShape:
                            {
                                // If it is a convex decomposition shape, we spawn a background cooking task for it (if needed)
                                omni::physx::usdparser::SpherePointsPhysxShapeDesc *desc = static_cast<omni::physx::usdparser::SpherePointsPhysxShapeDesc *>(shapeDesc);
                                getSpherePointsInternal(*desc, prim, true, nullptr);
                            }
                            break;
                            case omni::physx::usdparser::ObjectType::eConvexMeshDecompositionShape:
                            {
                                // If it is a convex decomposition shape, we spawn a background cooking task for it (if needed)
                                omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc *desc = static_cast<omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc *>(shapeDesc);
                                getConvexMeshDecompositionInternal(*desc, prim, true, nullptr);
                            }
                            break;
                            case omni::physx::usdparser::ObjectType::eTriangleMeshShape:
                            {
                                // If it is a triangle mesh shape, we spawn a background cooking task for it (if needed)
                                omni::physx::usdparser::TriangleMeshPhysxShapeDesc *desc = static_cast<omni::physx::usdparser::TriangleMeshPhysxShapeDesc *>(shapeDesc);
                                getTriangleMeshInternal(*desc, prim, true, nullptr, nullptr);
                            }
                            break;
                            default:
                                break;
                            }
                            // Release the shape desc we just parsed
                            omni::physx::usdparser::releaseDesc(shapeDesc);
                        }
                    }
                    else if (prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
                    {
                        cookDeformableBodyInternalAsyncDeprecated(prim, false);
                    }
                    else if (prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
                    {
                        cookParticleClothInternalDeprecated(prim, false, true);
                    }
                    else if (deformableBodyPrim)
                    {
                        cookDeformableBodyInternalAsync(deformableBodyPrim, false);
                    }
                }
                it = m_primRefreshSet.erase(it); // remove processed item and advance to next
                // Make sure m_primXformRefreshSet only contains updates that are exclusive to
                // tranformation changes
                m_primXformRefreshSet.erase(primPath);
                if (timer.getElapsedTime<int64_t>(carb::extras::Timer::Scale::eMilliseconds) >= 16) // never take more than 16 ms on the main thread
                {
                    endedPrematurely = true;
                    break;
                }
            }
            if(!endedPrematurely)
            {
                for (auto &primPath : m_primXformRefreshSet)
                {
                    UsdPrim prim = stage->GetPrimAtPath(primPath); // Look up the UsdPrim for this path name
                    if (prim)
                    {
                        if (prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
                        {
                            cookDeformableBodyInternalAsyncDeprecated(prim, true);
                        }
                        else if (prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
                        {
                            cookParticleClothInternalDeprecated(prim, true, true);
                        }
                        else
                        {
                            //TODO we also need to search children to catch transforms
                            //that have an impact to deformable cooking, which might be expensive
                            //without maintaining a SdfPathTable
                            bool hasCollisionAPI = prim.HasAPI<UsdPhysicsCollisionAPI>();
                            bool hasDeformablePoseAPI = prim.HasAPI(dpType);
                            bool hasDeformableBodyAPI = prim.HasAPI(dbType);
                            UsdPrim deformableBodyPrim;
                            if (hasDeformableBodyAPI)
                            {
                                deformableBodyPrim = prim;
                            }
                            else if (hasCollisionAPI || hasDeformablePoseAPI)
                            {
                                deformableBodyPrim = findDeformableBodyAncestor(prim);
                            }

                            if (deformableBodyPrim)
                            {
                                cookDeformableBodyInternalAsync(deformableBodyPrim, true);
                            }
                        }
                    }
                }

                // Once we have fully processed the refresh set, we can clear it.
                m_primRefreshSet.clear();
                m_primXformRefreshSet.clear();
            }
#else
            m_primRefreshSet.clear();
            m_primXformRefreshSet.clear();
#endif
            // Now we check on the status of outstanding cooking tasks
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
    virtual ::physx::PxConvexMesh* getConvexMesh(const omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc, UsdPrim& usdPrim, bool asynchronous, omni::physx::IPhysxCookingCallback* cb = nullptr) final
    {
        ::physx::PxConvexMesh* ret = nullptr;

        // Block USD notice handlers in case we write out any results during this call
        lock_guard _lock(m_mutex); // Make sure we are thread safe
        ScopedBlockUSDUpdates _block(this);
        return getConvexMeshInternal(desc, usdPrim, asynchronous, cb);
    }

    static void reportCookingFinished(const UsdPrim& usdPrim, omni::physx::IPhysxCookingCallback* cb)
    {
        if (cb && cb->cookingFinishedCallback)
        {
            const uint64_t primpath = asInt(usdPrim.GetPrimPath());
            const uint64_t stageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
            cb->cookingFinishedCallback(stageId, primpath, omni::physx::PhysxCookingResult::eVALID, cb->userData);
        }
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

        request.primMeshView.points = { (carb::Float3*)mergeMeshDesc.points.data(), mergeMeshDesc.points.size() };
        request.primMeshView.indices = { (int32_t*)mergeMeshDesc.indices.data(), mergeMeshDesc.indices.size() };
        request.primMeshView.faces = { (int32_t*)mergeMeshDesc.faces.data(), mergeMeshDesc.faces.size() };
        request.primMeshView.holeIndices = { (int32_t*)mergeMeshDesc.holes.data(), mergeMeshDesc.holes.size() };
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
                                                         UsdPrim& usdPrim,
                                                         bool asynchronous, omni::physx::IPhysxCookingCallback* cb)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // Disable asynchronous cooking for debugging purposes
#endif

        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        ::physx::PxConvexMesh *ret = nullptr;
        ret = omni::physx::getMeshCache()->getConvexMesh(meshCRC);

        omni::physx::PhysxCookingComputeRequest request;

        request.primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
        request.primId = asInt(usdPrim.GetPath());

        // If we haven't loaded the mesh from the in memory mesh cache, we go further to see if it is in the local cache or UsdPrim itself
        if (ret == nullptr)
        {
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
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
                    SdfPath primPath = intToPath(result.request->primId);
                    // Issue a warning message if we were unable to create a mesh from the cooked data stream.
                    CARB_LOG_WARN("Failed to create triangle mesh from cooked data! Prim(%s)\n", primPath.GetText());
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
        UsdPrim& usdPrim,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb = nullptr,
        uint16_t* maxMaterialIndex = nullptr) final
    {
        ::physx::PxTriangleMesh *ret = nullptr; // Default return is a null pointer indicating the mesh is either not found or not ready

        // Disable USD notification handlers in case we should do USD writes in this call
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        return getTriangleMeshInternal(desc, usdPrim, asynchronous, cb, maxMaterialIndex);
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
                                                             UsdPrim& usdPrim,
                                                             bool asynchronous,
                                                             omni::physx::IPhysxCookingCallback* cb,
                                                             uint16_t* maxMaterialIndex)
    {

#if !USE_ASYNC_COOKING
        asynchronous = false; // Disable asynchronous cooking if this debug option is configured
#endif
        // Make a copy of the initial unique MeshKey
        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        ::physx::PxTriangleMesh* ret = nullptr;
        ret = omni::physx::getMeshCache()->getTriangleMesh(meshCRC);

        omni::physx::PhysxCookingComputeRequest request;

        request.primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
        request.primId = asInt(usdPrim.GetPath());

        // If we haven't loaded the mesh from the in memory mesh cache, we go further to see if it is in the local cache
        // or UsdPrim itself
        if (ret == nullptr)
        {
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
            request.meshKey = desc.meshKey;

            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
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
                    SdfPath primPath = intToPath(result.request->primId);
                    // Issue a warning message if we were unable to create a mesh from the cooked data stream.
                    CARB_LOG_WARN("Failed to create triangle mesh from cooked data! Prim(%s)\n", primPath.GetText());
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
    virtual std::vector<::physx::PxConvexMesh*> getConvexMeshDecomposition(const omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc, UsdPrim& usdPrim,
        bool asynchronous, omni::physx::IPhysxCookingCallback* cb = nullptr) final
    {
        std::vector<::physx::PxConvexMesh*> ret; // By default returns an empty array

        // Block any USD notification handlers if we make any USD attribute changes in this method
        lock_guard _lock(m_mutex); // Lock mutex so that multiple-threads don't try to call cooking simultaneously
        ScopedBlockUSDUpdates _block(this);
        // Call the internal implementation version of this method
        return getConvexMeshDecompositionInternal(desc, usdPrim, asynchronous, cb);
    }

    virtual const omni::physx::usdparser::SpherePointsPhysxShapeDesc * getSpherePoints(const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,
        UsdPrim& usdPrim, bool asynchronous, omni::physx::IPhysxCookingCallback* cb = nullptr) final
    {
        // Block any USD notification handlers if we make any USD attribute changes in this method
        lock_guard _lock(m_mutex); // Lock mutex so that multiple-threads don't try to call cooking simultaneously
        ScopedBlockUSDUpdates _block(this);
        // Call the internal implementation version of this method
        return getSpherePointsInternal(desc, usdPrim, asynchronous, cb);
    }

    virtual const omni::physx::usdparser::SpherePointsPhysxShapeDesc *  getSpherePointsInternal(const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,UsdPrim& usdPrim,
        bool asynchronous, omni::physx::IPhysxCookingCallback* cb)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // If this debug option is active disable asynchronous cooking
#endif
        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        const omni::physx::usdparser::SpherePointsPhysxShapeDesc *ret = nullptr;

        // Get the convex decomposition STL hash map from the in-memory cache
        const omni::physx::SphereFillMap& sphereFillMap = omni::physx::getMeshCache()->getSphereFillMap();
        // Search to see if this CRC has a representation
        omni::physx::SphereFillMap::const_iterator it = sphereFillMap.find(meshCRC);
        if ( it != sphereFillMap.end())
        {
            ret = it->second;
            reportCookingFinished(usdPrim, cb);
        }
        else
        {

            omni::physx::PhysxCookingComputeRequest request;

            request.primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
            request.primId = asInt(usdPrim.GetPath());
            request.meshKey = desc.meshKey;

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
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
                    SdfPath primPath = intToPath(result.request->primId);
                    // Issue a warning message if we were unable to create a mesh from the cooked data stream.
                    CARB_LOG_WARN("Failed to create sphere fill from cooked data! Prim(%s)\n", primPath.GetText());
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
        UsdPrim& usdPrim,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb)
    {

#if !USE_ASYNC_COOKING
        asynchronous = false; // If this debug option is active disable asynchronous cooking
#endif

        omni::physx::usdparser::MeshKey meshKey = desc.meshKey;
        omni::physx::usdparser::MeshKey meshCRC = desc.crc;

        std::vector<::physx::PxConvexMesh*> ret;

        // Get the convex decomposition STL hash map from the in-memory cache
        const omni::physx::ConvexDecompositionMap& convexDecompositionMap = omni::physx::getMeshCache()->getConvexDecompositionMap();
        // Search to see if this CRC has a representation
        omni::physx::ConvexDecompositionMap::const_iterator it = convexDecompositionMap.find(meshCRC);
        if (it != convexDecompositionMap.end())
        {
            for (auto &i : it->second)
            {
                ret.push_back(i);
            }
            reportCookingFinished(usdPrim, cb);
        }
        else
        {
            omni::physx::PhysxCookingComputeRequest request;

            request.primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
            request.primId = asInt(usdPrim.GetPath());
            request.meshKey = desc.meshKey;

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, *desc.mergedMesh);
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
                    SdfPath primPath = intToPath(result.request->primId);
                    // Issue a warning message if we were unable to create a mesh from the cooked data stream.
                    CARB_LOG_WARN(
                        "Failed to create convex decomposition mesh from cooked data! Prim(%s)\n", primPath.GetText());
                }
            };
            getComputeService().requestConvexMeshDecompositionCookedData(m_asyncContext, request, desc.convexDecompositionCookingParams);
            return ret;
        }

        return ret;
    }

#if USE_ASYNC_COOKING
    /**
    * Helper function to schedule both deformable tet mesh cooking and physx softbody cooking.
    */
    void cookDeformableBodyInternalAsyncDeprecated(UsdPrim& usdPrim, bool xformOnly)
    {
        // If this is a deformable body, we spawn a cooking task for it if needed.
        bool dataReady = cookDeformableBodyTetMeshInternalDeprecated(usdPrim, xformOnly, true);
        if (dataReady)
        {
            // Parse the descriptor
            const uint64_t stageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
            omni::physx::usdparser::SoftBodyDesc* softBodyDesc = omni::physx::usdparser::parseDeformableBodyDeprecated(stageId, usdPrim.GetPath());
            if (softBodyDesc)
            {
                // Spawn a soft body cooking task
                ::physx::PxDefaultMemoryOutputStream outStream;
                cookSoftBodyMeshInternalDeprecated(outStream, *softBodyDesc, usdPrim, true);
                omni::physx::usdparser::releaseDesc(softBodyDesc);
            }
        }
    }
    
    /**
    * Helper function to schedule both deformable body USD data cooking and physx deformable volume mesh cooking.
    */
    void cookDeformableBodyInternalAsync(UsdPrim bodyPrim, bool xformOnly)
    {
        if (!bodyPrim)
            return;

        omni::physx::usdparser::PhysxDeformableBodyDesc* deformableDesc = parseDeformableBody(bodyPrim);
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
                dataReady = cookVolumeDeformableBodyInternal(*volumeDesc, bodyPrim, xformOnly, true);
            }
            if (dataReady)
            {
                // Spawn a deformable volume mesh cooking task
                ::physx::PxDefaultMemoryOutputStream outStream;
                cookDeformableVolumeMeshInternal(outStream, *volumeDesc, bodyPrim, true);
            }
        }
        else if (deformableDesc->type == omni::physx::usdparser::ObjectType::eSurfaceDeformableBody)
        {
            const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc* surfaceDesc =
                static_cast<const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc*>(deformableDesc);
            if (deformableDesc->hasAutoAPI)
            {
                cookSurfaceDeformableBodyInternal(*surfaceDesc, bodyPrim, xformOnly, true);
            }
        }

        ICE_FREE(deformableDesc);
    }
#endif // USE_ASYNC_COOKING

    /**
    * Performs the cooking operation on a deformable tetrahedral mesh associated with a particular UsdGeomMesh primitive
    *
    * @param usdPrim : The UsdGeomMesh we are cooking
    * @param asynchronous : If false, it will cook the tetrahedral mesh synchronously (blocking). If true, it will start a background cooking task for it.
    */
    virtual void cookDeformableBodyTetMeshDeprecated(UsdPrim& usdPrim, bool asynchronous) final
    {
        // Block USD notification handlers while in this call
        lock_guard _lock(m_mutex); // Lock the API mutex
        ScopedBlockUSDUpdates _block(this);
        cookDeformableBodyTetMeshInternalDeprecated(usdPrim, false, asynchronous);
    }

    virtual omni::physx::usdparser::PhysxDeformableBodyDesc* parseDeformableBody(UsdPrim bodyPrim)
    {
        struct Listener : public omni::physics::schema::IUsdPhysicsListener
        {
            Listener(omni::physx::usdparser::AttachedStage& attachedStage)
                : mAttachedStage(attachedStage)
                , mXfCache(UsdTimeCode::EarliestTime())
            {
            }

            virtual void parsePrim(const UsdPrim& prim, omni::physics::schema::ObjectDesc* objectDesc,
                uint64_t primTypes, const TfTokenVector& appliedApis) override
            {
                if (objectDesc)
                {
                    switch (objectDesc->type)
                    {
                    case omni::physics::schema::ObjectType::eVolumeDeformableBody:
                    case omni::physics::schema::ObjectType::eSurfaceDeformableBody:
                    {
                        mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), omni::physx::usdparser::SchemaAPIFlag::eDeformableBodyAPI);
                        const TfType autoType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableBodyAPI);
                        if (prim.HasAPI(autoType))
                        {
                            mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), omni::physx::usdparser::SchemaAPIFlag::eAutoDeformableBodyAPI);
                            const TfType simpType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableMeshSimplificationAPI);
                            if (prim.HasAPI(simpType))
                            {
                                mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), omni::physx::usdparser::SchemaAPIFlag::eAutoDeformableMeshSimplificationAPI);
                            }
                            if (objectDesc->type == omni::physics::schema::ObjectType::eVolumeDeformableBody)
                            {
                                const TfType hexType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(
                                    PhysxAdditionAPITokens->AutoDeformableHexahedralMeshAPI);
                                if (prim.HasAPI(hexType))
                                {
                                    mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(),
                                        omni::physx::usdparser::SchemaAPIFlag::eAutoDeformableHexahedralMeshAPI);
                                }
                            }
                        }
                    }
                    break;
                    }
                }
            }

            virtual void reportObjectDesc(const SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc) override
            {
                switch (objectDesc->type)
                {
                case omni::physics::schema::ObjectType::eVolumeDeformableBody:
                case omni::physics::schema::ObjectType::eSurfaceDeformableBody:
                {
                    const omni::physics::schema::DeformableBodyDesc* inDesc = (const omni::physics::schema::DeformableBodyDesc*)objectDesc;
                    std::vector<omni::physx::usdparser::CollisionBlockPair> filterPairsStub;
                    SdfPath materialStub;
                    omni::physx::usdparser::PhysxDeformableBodyDesc* desc =
                        omni::physx::usdparser::parseDeformableBody(mAttachedStage, mXfCache, path, *inDesc, filterPairsStub, materialStub);
                    if (desc && !mDesc)
                    {
                        mDesc = desc;
                    }
                }
                break;
                }
            }

            omni::physx::usdparser::PhysxDeformableBodyDesc* mDesc = nullptr;
            omni::physx::usdparser::AttachedStage& mAttachedStage;
            UsdStageWeakPtr mStage;
            UsdGeomXformCache mXfCache;
        };

        omni::physics::schema::IUsdPhysics* usdPhysics = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
        if (!usdPhysics)
        {
            CARB_LOG_ERROR("parseDeformableBody couldn't access IUsdPhysics");
            return nullptr;
        }

        UsdPrimRange range(bodyPrim, pxr::UsdPrimAllPrimsPredicate);
        omni::physics::schema::PrimIteratorRange primIteratorRange(range);

        UsdStageWeakPtr stage = bodyPrim.GetStage();
        omni::physx::usdparser::AttachedStage* attachedStage = new omni::physx::usdparser::AttachedStage(stage, nullptr);
        Listener usdPhysicsListener(*attachedStage);
        UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());

        usdPhysics->registerPhysicsListener(&usdPhysicsListener);
        usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
        usdPhysics->unregisterPhysicsListener(&usdPhysicsListener);
        delete attachedStage;
        return usdPhysicsListener.mDesc;
    }

    virtual void cookVolumeDeformableBody(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
        UsdPrim bodyPrim, bool asynchronous) final
    {
        if (desc.hasAutoAPI)
        {
            // Block USD notification handlers while in this call
            lock_guard _lock(m_mutex); // Lock the API mutex
            ScopedBlockUSDUpdates _block(this);
            cookVolumeDeformableBodyInternal(desc, bodyPrim, false, asynchronous);
        }
    }

    virtual void cookSurfaceDeformableBody(const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
        UsdPrim bodyPrim, bool asynchronous) final
    {
        if (desc.hasAutoAPI)
        {
            // Block USD notification handlers while in this call
            lock_guard _lock(m_mutex); // Lock the API mutex
            ScopedBlockUSDUpdates _block(this);
            cookSurfaceDeformableBodyInternal(desc, bodyPrim, false, asynchronous);
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

    /**
     * Fills in DeformableBodyTetMeshCookingParamsDeprecated for cooking input.
     * The usdPrim is parsed for necessary parameters, pxrSkin arrays are filled in and referenced from the parameters.
     * The customMeshCrc is based on pxrSkin arrays, including pxrSkinEarliestPoints as opposed to the default mesh key.
     */
    bool setupDeformableBodyTetMeshCookingParamsDeprecated(
        UsdPrim& usdPrim,
        omni::physx::DeformableBodyTetMeshCookingParamsDeprecated& params,
        VtArray<GfVec3f>& pxrSkinRestPoints,
        VtArray<GfVec3f>& pxrSkinEarliestPoints,
        omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        if (!usdPrim.HasAPI<PhysxSchemaPhysxDeformableAPI>())
        {
            CARB_LOG_ERROR("PhysX could not find USD prim or prim has no PhysxSchemaPhysxDeformableAPI!");
            return false;
        }

        PhysxSchemaPhysxDeformableAPI deformableBodyAPI(usdPrim);
        UsdGeomMesh geomMesh(usdPrim);

        // load properties from USD
        bool isPreAuthoredDeformable = true;

        UsdAttribute enableKinematicAttr = usdPrim.GetAttribute(deformableEnableKinematicToken);
        if (enableKinematicAttr.HasAuthoredValue())
        {
            enableKinematicAttr.Get(&params.kinematicEnabled);
            isPreAuthoredDeformable = !params.kinematicEnabled;
        }

        // kinematic fallback
        const bool isTransformTimeVarying = primutils::IsTransformTimeVarying(usdPrim);
        const bool isSkinMeshTimeVarying = geomMesh.GetPointsAttr().GetNumTimeSamples() > 1;
        if (isTransformTimeVarying || isSkinMeshTimeVarying)
        {
            // issue no warning here as that's done in the parser
            params.kinematicEnabled = true;
        }

        UsdAttribute simulationResolutionAttr = usdPrim.GetAttribute(deformableSimulationHexahedralResolutionToken);
        if (simulationResolutionAttr.HasAuthoredValue())
        {
            isPreAuthoredDeformable = false;
            simulationResolutionAttr.Get(&params.simulationResolution);
        }

        UsdAttribute numTetsPerHexAttr = usdPrim.GetAttribute(deformableNumberOfTetsPerHexToken);
        if (numTetsPerHexAttr.HasAuthoredValue())
        {
            isPreAuthoredDeformable = false;
            numTetsPerHexAttr.Get(&params.simulationNumTetsPerVoxel);
        }

        UsdAttribute collisionSimplificationAttr = usdPrim.GetAttribute(deformableCollisionSimplificationToken);
        if (collisionSimplificationAttr.HasAuthoredValue())
        {
            isPreAuthoredDeformable = false;
            collisionSimplificationAttr.Get(&params.simpParams.enabled);
            if (params.simpParams.enabled)
            {
                usdPrim.GetAttribute(deformableCollisionSimplificationRemeshingToken)
                    .Get(&params.simpParams.remeshing);
                usdPrim.GetAttribute(deformableCollisionSimplificationRemeshingResolutionToken)
                    .Get(&params.simpParams.remeshingResolution);
                usdPrim.GetAttribute(deformableCollisionSimplificationTargetTriangleCountToken)
                    .Get(&params.simpParams.targetTriangleCount);
                usdPrim.GetAttribute(deformableCollisionSimplificationForceConformingToken)
                    .Get(&params.simpParams.forceConforming);

                if (params.kinematicEnabled)
                {
                    params.simpParams.remeshing = false;
                    params.simpParams.forceConforming = true;
                }
            }
        }

        if (isPreAuthoredDeformable)
        {
            return false;
        }

        // load input and compute crc
        UsdAttribute restPointsAttr = deformableBodyAPI.GetRestPointsAttr();
        if (restPointsAttr.HasAuthoredValue())
        {
            restPointsAttr.Get(&pxrSkinRestPoints);

            // Sanity check to ensure that the number of rest points is the same as mesh points
            VtArray<GfVec3f> pxrMeshPoints;
            geomMesh.GetPointsAttr().Get(&pxrMeshPoints);
            if (pxrSkinRestPoints.size() != pxrMeshPoints.size())
            {
                if (!pxrSkinRestPoints.empty()) // only warn in non-trivial case
                {
                    CARB_LOG_WARN(
                        "PhysxSchemaPhysxDeformableAPI:restPoints and UsdGeomMesh:points "
                        "have inconsistent size. Resetting the restPoints to UsdGeomMesh:points.");
                }
                pxrSkinRestPoints.resize(pxrMeshPoints.size());
                pxrSkinRestPoints.assign(pxrMeshPoints.begin(), pxrMeshPoints.end());
                deformableBodyAPI.CreateRestPointsAttr().Set(pxrSkinRestPoints);
            }
        }
        else
        {
            UsdAttribute pointsAttr = geomMesh.GetPointsAttr();
            geomMesh.GetPointsAttr().Get(&pxrSkinRestPoints);
        }

        if (params.kinematicEnabled)
        {
            if (geomMesh.GetPointsAttr().GetNumTimeSamples() > 1)
            {
                geomMesh.GetPointsAttr().Get(&pxrSkinEarliestPoints, UsdTimeCode::EarliestTime());
            }
            else
            {
                pxrSkinEarliestPoints = pxrSkinRestPoints;
            }
        }

        VtArray<int32_t> pxrSkinVertexIndices;
        VtArray<int32_t> pxrSkinVertexCounts;
        geomMesh.GetFaceVertexIndicesAttr().Get(&pxrSkinVertexIndices);
        geomMesh.GetFaceVertexCountsAttr().Get(&pxrSkinVertexCounts);

        // Reject if data is missing
        if (pxrSkinRestPoints.empty() || pxrSkinVertexIndices.empty() || pxrSkinVertexCounts.empty())
        {
            return false;
        }

        GfVec3d scale = getLocalToWorldScale(geomMesh);
        float scaleFactor = 0.0f;
        {
            PxBounds3 bounds = PxBounds3::empty();
            for (size_t i = 0; i < pxrSkinRestPoints.size(); ++i)
            {
                const GfVec3f& p = pxrSkinRestPoints[i];
                bounds.include(PxVec3(p[0], p[1], p[2]));
            }
            PxVec3 boundsSize = bounds.maximum - bounds.minimum;
            float meshSize = PxMax(boundsSize.x, PxMax(boundsSize.y, boundsSize.z));
            scaleFactor = 1.0f / PxMax(1e-6f, meshSize);
        }

        GfVec3f nqScale = getNormalizedQuantizedDeprecated(scale, scaleFactor);
        params.normalizedQuantizedScale = { nqScale[0], nqScale[1], nqScale[2] };

        params.skinRestPoints = { reinterpret_cast<const carb::Float3*>(pxrSkinRestPoints.data()),
                                  pxrSkinRestPoints.size() };
        params.skinEarliestPoints = { reinterpret_cast<const carb::Float3*>(pxrSkinEarliestPoints.data()),
                                      pxrSkinEarliestPoints.size() };

        customMeshCrc = computeMeshKey(pxrSkinRestPoints, pxrSkinVertexIndices, pxrSkinVertexCounts);
        if (pxrSkinEarliestPoints.size() > 0)
        {
            customMeshCrc.setMiscData(reinterpret_cast<const uint8_t*>(pxrSkinEarliestPoints.data()),
                                      pxrSkinEarliestPoints.size() * sizeof(GfVec3f));
        }
        return true;
    }

    /***
    * A method to store the cooked tetrahedral mesh data to the corresponding USD primitive attributes
    */
    void storeDeformableBodyTetMeshDataToUsdDeprecated(
        const carb::Float3* collisionRestPoints,
        const uint32_t collisionRestPointsSize,
        const uint32_t* collisionIndices,
        const uint32_t collisionIndicesSize,
        const carb::Float3* simulationRestPoints,
        const uint32_t simulationRestPointsSize,
        const uint32_t* simulationIndices,
        const uint32_t simulationIndicesSize,
        const int32_t* collisionVertexToSimulationTetIndices,
        const uint32_t collisionVertexToSimulationTetIndicesSize,
        const uint32_t* collisionVertexToSkinTriVertexIndices,
        const carb::Float3* collisionVertexToSkinTriBarycentrics,
        const uint32_t collisionVertexToSkinTriSize,
        UsdPrim& prim,
        const omni::physx::usdparser::MeshKey& tetMeshCrc)
    {
        PhysxSchemaPhysxDeformableBodyAPI deformableBodyAPI(prim);
        PhysxSchemaPhysxDeformableAPI deformableAPI(deformableBodyAPI);

        //write crc
        VtArray<uchar> dataVt;
        dataVt.resize(sizeof(omni::physx::usdparser::MeshKey));
        memcpy(dataVt.data(), &tetMeshCrc, sizeof(omni::physx::usdparser::MeshKey));
        prim.CreateAttribute(deformableTetMeshCrcToken, SdfValueTypeNames->UCharArray).Set(dataVt);

        //write coll mesh
        VtArray<GfVec3f> pxrCollisionRestPoints;
        omni::physx::copyBuffer(pxrCollisionRestPoints, collisionRestPoints, collisionRestPointsSize);
        deformableBodyAPI.GetCollisionRestPointsAttr().Set(pxrCollisionRestPoints);

        VtArray<int32_t> pxrCollisionIndices;
        pxrCollisionIndices.resize(collisionIndicesSize);
        memcpy(pxrCollisionIndices.data(), collisionIndices, sizeof(int32_t) * pxrCollisionIndices.size());
        deformableBodyAPI.GetCollisionIndicesAttr().Set(pxrCollisionIndices);

        //write sim mesh
        VtArray<GfVec3f> pxrSimulationRestPoints;
        omni::physx::copyBuffer(pxrSimulationRestPoints, simulationRestPoints, simulationRestPointsSize);
        deformableBodyAPI.GetSimulationRestPointsAttr().Set(pxrSimulationRestPoints);

        VtArray<int32_t> pxrSimulationIndices;
        pxrSimulationIndices.resize(simulationIndicesSize);
        memcpy(pxrSimulationIndices.data(), simulationIndices, sizeof(int32_t) * pxrSimulationIndices.size());
        deformableAPI.GetSimulationIndicesAttr().Set(pxrSimulationIndices);

        VtArray<int32_t> pxrCollisionVertexToSimulationTetIndices;
        pxrCollisionVertexToSimulationTetIndices.resize(collisionVertexToSimulationTetIndicesSize);
        memcpy(pxrCollisionVertexToSimulationTetIndices.data(), collisionVertexToSimulationTetIndices, sizeof(int32_t) * pxrCollisionVertexToSimulationTetIndices.size());
        UsdAttribute collisionVertexToSimulationTetIndicesAttr = prim.CreateAttribute(deformableCollisionVertexToSimulationTetIndicesToken, SdfValueTypeNames->IntArray);
        collisionVertexToSimulationTetIndicesAttr.Set(pxrCollisionVertexToSimulationTetIndices);

        VtArray<uint32_t> pxrCollisionVertexToSkinTriVertexIndices;
        pxrCollisionVertexToSkinTriVertexIndices.resize(collisionVertexToSkinTriSize * 3);
        memcpy(pxrCollisionVertexToSkinTriVertexIndices.data(), collisionVertexToSkinTriVertexIndices, sizeof(uint32_t) * pxrCollisionVertexToSkinTriVertexIndices.size());
        UsdAttribute collisionVertexToSkinTriVertexIndicesAttr = prim.CreateAttribute(deformableCollisionVertexToSkinTriVertexIndicesToken, SdfValueTypeNames->UIntArray);
        collisionVertexToSkinTriVertexIndicesAttr.Set(pxrCollisionVertexToSkinTriVertexIndices);

        VtArray<GfVec3f> pxrCollisionVertexToSkinTriBarycentrics;
        pxrCollisionVertexToSkinTriBarycentrics.resize(collisionVertexToSkinTriSize);
        memcpy(pxrCollisionVertexToSkinTriBarycentrics.data(), collisionVertexToSkinTriBarycentrics, sizeof(carb::Float3) * pxrCollisionVertexToSkinTriBarycentrics.size());
        UsdAttribute collisionVertexToSkinTriBarycentricsAttr = prim.CreateAttribute(deformableCollisionVertexToSkinTriBarycentricsToken, SdfValueTypeNames->Vector3fArray);
        collisionVertexToSkinTriBarycentricsAttr.Set(pxrCollisionVertexToSkinTriBarycentrics);

        //reset dependent values (if they have been written already before)
        if (deformableBodyAPI.GetSimulationPointsAttr().HasAuthoredValue())
        {
            deformableBodyAPI.GetSimulationPointsAttr().Set(pxrSimulationRestPoints);
        }

        if (deformableAPI.GetSimulationVelocitiesAttr().HasAuthoredValue())
        {
            VtArray<GfVec3f> pxrSimulationVelocities;
            pxrSimulationVelocities.resize(pxrSimulationRestPoints.size());
            memset(pxrSimulationVelocities.data(), 0, sizeof(GfVec3f) * pxrSimulationVelocities.size());
            deformableAPI.GetSimulationVelocitiesAttr().Set(pxrSimulationVelocities);
        }

        if (deformableBodyAPI.GetCollisionPointsAttr().HasAuthoredValue())
        {
            VtArray<GfVec3f> pxrCollisionRestPoints;
            deformableBodyAPI.GetCollisionRestPointsAttr().Get(&pxrCollisionRestPoints);
            deformableBodyAPI.GetCollisionPointsAttr().Set(pxrCollisionRestPoints);
        }

        //as opposed to other rest data, restPoints are generated from mesh points
        if (deformableAPI.GetRestPointsAttr().HasAuthoredValue())
        {
            VtArray<GfVec3f> pxrRestPoints;
            deformableAPI.GetRestPointsAttr().Get(&pxrRestPoints);
            UsdGeomMesh(prim).GetPointsAttr().Set(pxrRestPoints);
        }

        UsdAttribute invMassScaleAttr = prim.GetAttribute(invMassScaleAttributeName);
        if (invMassScaleAttr)
        {
            invMassScaleAttr.Clear();
        }
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
        UsdPrim& bodyPrim,
        const omni::physx::usdparser::MeshKey& tetMeshCrc)
    {
        UsdStageWeakPtr stage = bodyPrim.GetStage();
        TfType bodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        if (!bodyPrim.HasAPI(bodyType))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdPhysicsDeformableBodyAPI applied to %s", bodyPrim.GetPath().GetText());
            return;
        }

        TfType volumeSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
        TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);

        //write sim mesh UsdGeomTetMesh
        //we just write the sim points, which are actually bind pose points
        //this will reset the simulation state, if there is one
        UsdPrim simMeshPrim = stage->GetPrimAtPath(desc.simMeshPath);
        if (!simMeshPrim.IsA<UsdGeomTetMesh>())
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdGeomTetMesh sim mesh defined at %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            UsdGeomPointBased(simMeshPrim).GetPointsAttr().Set(points);

            VtArray<GfVec3f> velocities;
            UsdGeomPointBased(simMeshPrim).GetVelocitiesAttr().Set(velocities);

            VtArray<GfVec4i> indices;
            omni::physx::copyBuffer(indices, data.simIndices, data.simIndicesSize);
            UsdGeomTetMesh(simMeshPrim).GetTetVertexIndicesAttr().Set(indices);
        }

        //write rest shape UsdPhysicsVolumeDeformableSimAPI
        //for now we always just write the sim mesh 1:1
        if (!simMeshPrim.HasAPI(volumeSimType))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdPhysicsVolumeDeformableSimAPI applied to %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(points);

            VtArray<GfVec4i> indices;
            omni::physx::copyBuffer(indices, data.simIndices, data.simIndicesSize);
            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices).Set(indices);
        }

        //write bind poses
        {
            UsdAttribute posePointsAttr = getPosePointsAttr(simMeshPrim, poseType, desc.simMeshBindPoseToken);
            if (posePointsAttr)
            {
                VtArray<GfVec3f> points;
                omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
                posePointsAttr.Set(points);
            }
        }

        if (desc.collisionMeshPath.IsEmpty())
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: Expected prim with UsdPhysicsCollisionAPI: %s",
                bodyPrim.GetPath().GetText());
            return;
        }

        if (desc.collisionMeshPath != desc.simMeshPath)
        {
            //write collision mesh UsdGeomTetMesh
            //we just write the collision points, which are actually bind pose points
            //this will reset the simulation state, if there is one
            UsdPrim collisionMeshPrim = stage->GetPrimAtPath(desc.collisionMeshPath);
            if (!collisionMeshPrim.IsA<UsdGeomTetMesh>())
            {
                CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdGeomTetMesh collision mesh defined at %s",
                    desc.collisionMeshPath.GetText());
                return;
            }

            {
                VtArray<GfVec3f> points;
                omni::physx::copyBuffer(points, data.collPoints, data.collPointsSize);
                UsdGeomPointBased(collisionMeshPrim).GetPointsAttr().Set(points);

                VtArray<GfVec4i> indices;
                omni::physx::copyBuffer(indices, data.collIndices, data.collIndicesSize);
                UsdGeomTetMesh(collisionMeshPrim).GetTetVertexIndicesAttr().Set(indices);
            }

            {
                UsdAttribute posePointsAttr = getPosePointsAttr(collisionMeshPrim, poseType, desc.collisionMeshBindPoseToken);
                if (posePointsAttr)
                {
                    VtArray<GfVec3f> points;
                    omni::physx::copyBuffer(points, data.collPoints, data.collPointsSize);
                    posePointsAttr.Set(points);
                }
            }
        }

        //writing collision mesh surface indices in any case (even if sim/coll mesh alias)
        {
            UsdPrim collisionMeshPrim = stage->GetPrimAtPath(desc.collisionMeshPath);
            VtArray<GfVec3i> indices;
            omni::physx::copyBuffer(indices, data.collSurfaceIndices, data.collSurfaceIndicesSize);
            UsdGeomTetMesh(collisionMeshPrim).GetSurfaceFaceVertexIndicesAttr().Set(indices);
        }

        // reset the skin points to their bind pose
        for (size_t s = 0; s < desc.skinGeomPaths.size(); ++s)
        {
            SdfPath skinGeomPath = desc.skinGeomPaths[s];
            UsdPrim skinGeomPrim = stage->GetPrimAtPath(skinGeomPath);
            UsdGeomPointBased skinGeom(skinGeomPrim);
            if (skinGeom)
            {
                UsdAttribute posePointsAttr = getPosePointsAttr(skinGeomPrim, poseType, desc.skinGeomBindPoseTokens[s]);
                if (posePointsAttr)
                {
                    VtArray<GfVec3f> bindPoints;
                    posePointsAttr.Get(&bindPoints);
                    skinGeom.GetPointsAttr().Set(bindPoints);
                }
            }
        }

        //write crc
        storeMeshKey(bodyPrim, deformableBodyDataCrcToken, tetMeshCrc);
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
        UsdPrim& bodyPrim,
        const omni::physx::usdparser::MeshKey& deformableBodyDataCrc)
    {
        UsdStageWeakPtr stage = bodyPrim.GetStage();
        TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        if (!bodyPrim.HasAPI(dbType))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsDeformableBodyAPI applied to %s", bodyPrim.GetPath().GetText());
            return;
        }

        //first read crc and abort if still valid, the reason is we are consuming data from the cooking job that doesn't
        //go into USD, so we need the cooking task to return successfully to return the cached data even though
        //we don't need to update to USD.
        omni::physx::usdparser::MeshKey storedDeformableBodyDataCrc = omni::physx::usdparser::loadMeshKey(bodyPrim, deformableBodyDataCrcToken);
        if (storedDeformableBodyDataCrc == deformableBodyDataCrc)
        {
            return;
        }

        //write sim mesh UsdGeomMesh
        //we just write the sim points, which are actually bind pose points
        //this will reset the simulation state, if there is one
        UsdPrim simMeshPrim = stage->GetPrimAtPath(desc.simMeshPath);
        UsdGeomMesh simMesh(simMeshPrim);
        if (!simMesh)
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdGeomMesh sim mesh defined at %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            simMesh.GetPointsAttr().Set(points);

            VtArray<GfVec3f> velocities;
            simMesh.GetVelocitiesAttr().Set(velocities);

            size_t numFaces = data.simIndicesSize / 3;
            CARB_ASSERT(numFaces * 3 == data.simIndicesSize);
            VtArray<int32_t> faceVertexCounts(numFaces, 3);
            VtArray<int32_t> faceVertexIndices(numFaces*3);
            std::memcpy(faceVertexIndices.begin(), data.simIndices, sizeof(int32_t)*faceVertexIndices.size());

            simMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts);
            simMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices);
        }

        TfType surfaceSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
        TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);

        //write rest shape UsdPhysicsSurfaceDeformableSimAPI
        //for now we always just write the sim mesh 1:1
        if (!simMeshPrim.HasAPI(surfaceSimType))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsSurfaceDeformableSimAPI applied to %s", desc.simMeshPath.GetText());
            return;
        }

        {
            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(points);

            size_t numFaces = data.simIndicesSize / 3;
            VtArray<GfVec3i> triIndices(numFaces);
            std::memcpy(triIndices.data(), data.simIndices, sizeof(GfVec3i) * triIndices.size());
            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices).Set(triIndices);
        }

        //write bind poses
        if (!desc.simMeshBindPoseToken.IsEmpty())
        {
            UsdAttribute posePointsAttr = getPosePointsAttr(simMeshPrim, poseType, desc.simMeshBindPoseToken);
            if (!posePointsAttr)
            {
                CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsDeformablePoseAPI:%s applied to %s",
                               desc.simMeshBindPoseToken.GetText(), desc.simMeshPath.GetText());
                return;
            }

            VtArray<GfVec3f> points;
            omni::physx::copyBuffer(points, data.simPoints, data.simPointsSize);
            posePointsAttr.Set(points);
        }

        if (desc.collisionMeshPath.IsEmpty())
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: Expected prim with UsdPhysicsCollisionAPI: %s",
                bodyPrim.GetPath().GetText());
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
            UsdPrim skinGeomPrim = stage->GetPrimAtPath(skinGeomPath);
            UsdGeomPointBased skinGeom(skinGeomPrim);
            if (skinGeom && !desc.skinGeomBindPoseTokens[s].IsEmpty())
            {
                UsdAttribute posePointsAttr = getPosePointsAttr(skinGeomPrim, poseType, desc.skinGeomBindPoseTokens[s]);
                if (!posePointsAttr)
                {
                    CARB_LOG_ERROR(
                        "storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsDeformablePoseAPI:%s applied to %s",
                        desc.skinGeomBindPoseTokens[s].GetText(), skinGeomPath.GetText());
                }
                VtArray<GfVec3f> bindPoints;
                posePointsAttr.Get(&bindPoints);
                skinGeom.GetPointsAttr().Set(bindPoints);
            }
        }

        //write crc
        storeMeshKey(bodyPrim, deformableBodyDataCrcToken, deformableBodyDataCrc);
    }

    /**
    * Performs the cooking operation on a deformable tetrahedral mesh associated with a particular UsdGeomMesh primitive
    *
    * @param usdPrim : The UsdGeomMesh we are cooking
    * @param xFormOnly : If true, the cooking was triggered excusively for a transform update. 
    * @param asynchronous : If false, it will cook the tetrahedral mesh synchronously (blocking). If true, it will start a background cooking task for it.
    * @return : Returns true if usd data is ready and PxSoftBodyMesh should be cooked
    */
    bool cookDeformableBodyTetMeshInternalDeprecated(UsdPrim& usdPrim, bool xFormOnly, bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // If asynchronous cooking has been disabled
#endif
        uint64_t primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();

        const bool isSimulated = checkParsed(primStageId, usdPrim.GetPath(), omni::physx::usdparser::eSoftBody);
        if (isSimulated)
        {
            //note: corresponding warnings are issued in omni::physx::warnDeformableBodyNoRuntimeCooking
            return false;
        }

        omni::physx::DeformableBodyTetMeshCookingParamsDeprecated params;
        omni::physx::usdparser::MeshKey customMeshCrc;
        VtArray<GfVec3f> pxrSkinRestPoints;
        VtArray<GfVec3f> pxrSkinEarliestPoints;
        if (!setupDeformableBodyTetMeshCookingParamsDeprecated(usdPrim, params, 
                pxrSkinRestPoints, pxrSkinEarliestPoints, customMeshCrc))
        {
            CARB_LOG_ERROR("Deformable body tetmesh, failed to setup cooking params, prim: %s", usdPrim.GetPath().GetText());
            return false;
        }

        omni::physx::usdparser::MeshKey originalCrc =
            omni::physx::usdparser::loadMeshKey(usdPrim, deformableTetMeshCrcToken);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.primId = asInt(usdPrim.GetPath());
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
            request.meshKey = customMeshCrc;
            request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
            omni::physx::usdparser::MeshKey cookedDataCRC;
            request.onFinished = [&cookedDataCRC](const omni::physx::PhysxCookingComputeResult& result) {
                cookedDataCRC = result.cookedDataCRC;
            };
            m_cookingServicePrivate.requestDeformableBodyTetMeshCookedDataDeprecated(m_asyncContext, request, params);
            if (cookedDataCRC == originalCrc)
            {
                return true;
            }
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.primStageId = primStageId;
        request.primId = asInt(usdPrim.GetPath());
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
        request.meshKey = customMeshCrc;

        bool shouldRecookSoftBodyTooSynchronously = false;
        auto weakPtrToThis = TfCreateWeakPtr(this);
        recordStatisticsRequestFor(request);
        request.onFinished =
            [weakPtrToThis, &shouldRecookSoftBodyTooSynchronously](const omni::physx::PhysxCookingComputeResult& result) {
                CARB_ASSERT(result.request->dataType == omni::physx::PhysxCookingDataType::eDEFORMABLE_TETRAHEDRAL_MESH_DEPRECATED);
                if (!weakPtrToThis)
                {
                    return;
                }
                weakPtrToThis->recordStatisticsResultFor(result);
                omni::physx::IPhysxCookingServicePrivate& cookingService = weakPtrToThis->m_cookingServicePrivate;
                UsdStageWeakPtr stage = omni::physx::OmniPhysX::getInstance().getStage();
                if (stage)
                {
                    SdfPath primPath = intToPath(result.request->primId);
                    UsdPrim usdPrim = stage->GetPrimAtPath(primPath);
                    if (usdPrim && usdPrim.IsA<UsdGeomMesh>())
                    {
                        if (result.result == omni::physx::PhysxCookingResult::eVALID)
                        {
                            if (result.cookedData)
                            {
                                omni::physx::PhysxCookingDeformableBodyTetMeshDataDeprecated data;
                                cookingService.readDeformableBodyTetMeshDataDeprecated(data, *result.cookedData);

                                weakPtrToThis->storeDeformableBodyTetMeshDataToUsdDeprecated(data.collisionRestPoints, data.collisionRestPointsSize, data.collisionIndices,
                                    data.collisionIndicesSize, data.simulationRestPoints,
                                    data.simulationRestPointsSize, data.simulationIndices, data.simulationIndicesSize,
                                    data.collisionVertexToSimulationTetIndices, data.collisionVertexToSimulationTetIndicesSize,
                                    data.collisionVertexToSkinTriVertexIndices, data.collisionVertexToSkinTriBarycentrics,
                                    data.collisionVertexToSkinTriSize, usdPrim, result.cookedDataCRC);
                            }
                        }
                        else
                        {
                            omni::physx::usdparser::MeshKey crc_zero; // need to write crc 0, so crc test fails for invalid cook
                            weakPtrToThis->storeDeformableBodyTetMeshDataToUsdDeprecated(nullptr, 0, nullptr, 0, nullptr, 0, nullptr, 0, nullptr, 0, nullptr, nullptr, 0,
                                usdPrim, crc_zero);
                        }
                    }
                }

                if (result.isSynchronousResult)
                {
                    shouldRecookSoftBodyTooSynchronously = true;
                }
            };

        m_cookingServicePrivate.requestDeformableBodyTetMeshCookedDataDeprecated(m_asyncContext, request, params);
        return shouldRecookSoftBodyTooSynchronously;
    }

    /**
    * Cooks a PxSoftBodyMesh, creates a surface triangle list and returns the result in a stream.
    *
    * @param outStream : Stream to which the PxSoftBodyMesh data and the surface triangle list is written to if available.
    * @param softBodyDesc : The soft body shape descriptor which defines properties for this mesh.
    * @param usdPrim : The UsdGeomMesh associated with this soft body
    * @param asynchronous : Whether or not to cooking the soft body mesh (if needed) asynchronously.
    *
    * @return : Returns true if data has been written to outStream.
    */
    virtual bool cookSoftBodyMeshDeprecated(::physx::PxDefaultMemoryOutputStream& outStream,
        const omni::physx::usdparser::SoftBodyDesc& desc,
        UsdPrim& usdPrim,
        bool asynchronous) final
    {
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this); // block USD notice handlers
        return cookSoftBodyMeshInternalDeprecated(outStream, desc, usdPrim, asynchronous);
    }
    
    bool setupSoftBodyMeshCookingParamsDeprecated(
        UsdPrim usdPrim,
        const omni::physx::usdparser::SoftBodyDesc& desc,
        omni::physx::SoftBodyMeshCookingParamsDeprecated& params)
    {
        GfVec3d scale = getLocalToWorldScale(UsdGeomXformable(usdPrim));
        float scaleFactor = 0.0f;
        {
            PxBounds3 bounds = PxBounds3::empty();
            for (size_t i = 0; i < desc.points.size(); ++i)
            {
                const carb::Float3& p = desc.points[i];
                bounds.include(PxVec3(p.x, p.y, p.z));
            }
            PxVec3 boundsSize = bounds.maximum - bounds.minimum;
            float meshSize = PxMax(boundsSize.x, PxMax(boundsSize.y, boundsSize.z));
            scaleFactor = 1.0f / PxMax(1e-6f, meshSize);
        }
        GfVec3f nqScale = getNormalizedQuantizedDeprecated(scale, scaleFactor);

        params.collisionRestPoints = omni::span<const carb::Float3>(desc.collisionRestPoints);
        params.collisionIndices = omni::span<const uint32_t>(desc.collisionIndices);
        params.simulationRestPoints = omni::span<const carb::Float3>(desc.simulationRestPoints);
        params.simulationIndices = omni::span<const uint32_t>(desc.simulationIndices);
        params.collisionVertexToSimulationTetIndices = omni::span<const uint32_t>(desc.collisionVertexToSimulationTetIndices);
        params.restPoints = omni::span<const carb::Float3>(desc.restPoints);
        params.softBodyToWorldScaleNormalized = { nqScale[0], nqScale[1], nqScale[2] };
        params.simulationResolution = desc.simulationHexahedralResolution;
        params.simulationNumTetsPerVoxel = desc.numberOfTetsPerHex;
        params.kinematicEnabled = desc.kinematicBody;
        return true;
    }

    /**
    * Cooks a PxSoftBodyMesh, creates a surface triangle list and returns the result in a stream.
    *
    * @param outStream : Stream to which the PxSoftBodyMesh data and the surface triangle list is written to if available.
    * @param desc : The soft body shape descriptor which defines properties for this mesh.
    * @param usdPrim : The UsdGeomMesh associated with this soft body
    * @param asynchronous : Whether or not to cooking the soft body mesh (if needed) asynchronously.
    *
    * @return : Returns true if data has been written to outStream.
    */
    bool cookSoftBodyMeshInternalDeprecated(
        ::physx::PxDefaultMemoryOutputStream& outStream,
        const omni::physx::usdparser::SoftBodyDesc& desc,
        UsdPrim& usdPrim,
        bool asynchronous)
    {
        if (desc.collisionRestPoints.size() == 0 || desc.simulationRestPoints.size() == 0)
        {
            //This configuration is a signal that the deformable mesh cooking failed.
            return false;
        }

#if !USE_ASYNC_COOKING
        asynchronous = false; // If asynchronous cooking is disabled
#endif
        omni::physx::SoftBodyMeshCookingParamsDeprecated params;
        if (!setupSoftBodyMeshCookingParamsDeprecated(usdPrim, desc, params))
        {
            CARB_LOG_ERROR("SoftBody, failed to setup cooking params, prim: %s", usdPrim.GetPath().GetText());
            return false;
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
        request.primId = asInt(usdPrim.GetPath());
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);

        bool resultSynchronous = false;
        auto weakPtrToThis = TfCreateWeakPtr(this);
        recordStatisticsRequestFor(request);
        request.onFinished =
            [&resultSynchronous, &outStream, weakPtrToThis] (const omni::physx::PhysxCookingComputeResult& result) {
            if(!weakPtrToThis)
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
        m_cookingServicePrivate.requestSoftBodyMeshCookedDataDeprecated(m_asyncContext, request, params);
        return resultSynchronous;
    }

    /**
    * Helper method, that creates a PxSoftBodyMesh from the input data stream and parses the surface triangle list.
    *
    * @param outCollMeshSurfaceTriangles : output collision mesh surface triangles
    * @param outNumCollMeshSurfaceVertices : output number of collision mesh surface vertices
    * @param inData : input data (@see cookSoftBodyMesh)
    *
    * @return : Returns the PxSoftBodyMesh instance. The caller of the method is responsible for releasing the PxSoftBodyMesh.
    */
    ::physx::PxSoftBodyMesh* createSoftBodyMeshDeprecated(std::vector<carb::Uint3>& outCollMeshSurfaceTriangles, ::physx::PxDefaultMemoryInputData& inData)
    {
        ::physx::PxSoftBodyMesh* softBodyMesh = mPhysics.createSoftBodyMesh(inData);

        // read the appended surface triangles from the cooked data
        if (inData.tell()+sizeof(uint32_t) <= inData.getLength())
        {
            uint32_t surfaceTriangleCount;
            inData.read(&surfaceTriangleCount,sizeof(uint32_t));
            outCollMeshSurfaceTriangles.resize(surfaceTriangleCount);
            inData.read(&outCollMeshSurfaceTriangles[0],surfaceTriangleCount*sizeof(carb::Uint3));
        }
        return softBodyMesh;
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
        UsdPrim usdPrim,
        const omni::physx::usdparser::ParticleSamplingDesc& desc,
        GfMatrix4d& rigidTransform,
        omni::physx::ParticlePoissonSamplingCookingParams& params)
    {
        GfMatrix3d shearScaleTransform;
        if (!omni::physx::particles::PhysxParticleFactory::getDecomposedTransform(
                usdPrim.GetPath(), desc.particleSetPath,
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

    bool setupVolumeDeformableBodyCookingParams(UsdPrim& bodyPrim,
                                                const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                                omni::physx::VolumeDeformableBodyCookingParams& params,
                                                pxr::UsdPrim& srcMeshPrim,
                                                pxr::VtArray<pxr::GfVec3f>& pxrSrcPointsInSim,
                                                omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        pxr::TfType dbType =
            pxr::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(pxr::OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        if (!bodyPrim.HasAPI(dbType))
        {
            CARB_LOG_ERROR("PhysX could not find USD prim or prim has no UsdPhysicsDeformableBodyAPI!");
            return false;
        }

        pxr::UsdPrim simMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.simMeshPath);
        pxr::UsdPrim collMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.collisionMeshPath);
        pxr::GfMatrix4d simToWorld =
            pxr::UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
        pxr::GfMatrix4d worldToSim = simToWorld.GetInverse();

        pxr::GfMatrix4d simToColl;
        if (collMeshPrim != simMeshPrim)
        {
            pxr::GfMatrix4d collToWorld =
                pxr::UsdGeomXformable(collMeshPrim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            pxr::GfMatrix4d worldToColl = collToWorld.GetInverse();
            simToColl = simToWorld * worldToColl;
        }
        else
        {
            simToColl.SetIdentity();
        }

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.", bodyPrim.GetPath().GetText());
            return false;
        }

        srcMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.cookingSrcMeshPath);
        pxr::UsdGeomMesh srcMesh(srcMeshPrim);
        pxr::TfToken srcMeshBindPoseToken = desc.cookingSrcMeshBindPoseToken;
        if (!srcMesh)
        {
            CARB_LOG_WARN(
                "Cooking failed, deformable has no valid source mesh for cooking: %s.", bodyPrim.GetPath().GetText());
            return false;
        }

        pxr::VtArray<int32_t> pxrSrcVertexIndices;
        pxr::VtArray<int32_t> pxrSrcVertexCounts;

        {
            TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
            UsdAttribute simBindPointsAttr = getPosePointsOrPointsAttr(srcMesh.GetPrim(), poseType, srcMeshBindPoseToken);
            if (simBindPointsAttr)
            {
                simBindPointsAttr.Get(&pxrSrcPointsInSim);
            }
            srcMesh.GetFaceVertexIndicesAttr().Get(&pxrSrcVertexIndices);
            srcMesh.GetFaceVertexCountsAttr().Get(&pxrSrcVertexCounts);

            // Transform skin points to sim mesh space
            pxr::GfMatrix4d srcToWorld = srcMesh.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            pxr::GfMatrix4d srcToSim = srcToWorld * worldToSim;
            for (size_t i = 0; i < pxrSrcPointsInSim.size(); ++i)
            {
                pxr::GfVec3f& srcPoint = pxrSrcPointsInSim[i];
                srcPoint = srcToSim.Transform(srcPoint);
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
    bool cookVolumeDeformableBodyInternal(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc, UsdPrim& bodyPrim, bool xFormOnly, bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // If asynchronous cooking has been disabled
#endif
        uint64_t primStageId = UsdUtilsStageCache::Get().GetId(bodyPrim.GetStage()).ToLongInt();

        const bool isSimulated = checkParsed(primStageId, bodyPrim.GetPath(), omni::physx::usdparser::eVolumeDeformableBody);
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
        pxr::UsdPrim srcMeshPrim;
        VtArray<GfVec3f> pxrSrcPointsInSim;
        omni::physx::usdparser::MeshKey customMeshCrc;
        if (!setupVolumeDeformableBodyCookingParams(bodyPrim, desc, params, srcMeshPrim, pxrSrcPointsInSim, customMeshCrc))
        {
            CARB_LOG_ERROR("Volume deformable body, failed to setup cooking params, prim: %s", bodyPrim.GetPath().GetText());
            return false;
        }

        omni::physx::usdparser::MeshKey originalCrc =
            omni::physx::usdparser::loadMeshKey(bodyPrim, deformableBodyDataCrcToken);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.primId = asInt(srcMeshPrim.GetPath());
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
        request.primId = asInt(srcMeshPrim.GetPath());

        uint64_t bodyPrimId = asInt(bodyPrim.GetPath());
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
            UsdStageWeakPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(long(result.request->primStageId)));
            if (stage)
            {
                SdfPath bodyPrimPath = intToPath(bodyPrimId);
                UsdPrim bodyPrim = stage->GetPrimAtPath(bodyPrimPath);
                TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
                if (bodyPrim && bodyPrim.HasAPI(dbType))
                {
                    if (result.result == omni::physx::PhysxCookingResult::eVALID)
                    {
                        if (result.cookedData)
                        {
                            omni::physx::PhysxCookingVolumeDeformableBodyData data;
                            cookingService.readVolumeDeformableBodyData(data, *result.cookedData);
                            weakPtrToThis->storeVolumeDeformableBodyDataToUsd(data, desc, bodyPrim, result.cookedDataCRC);
                        }
                    }
                    else
                    {
                        omni::physx::usdparser::MeshKey crc_zero;
                        VtArray<uchar> dataVt;
                        dataVt.resize(sizeof(omni::physx::usdparser::MeshKey));
                        memcpy(dataVt.data(), &crc_zero, sizeof(omni::physx::usdparser::MeshKey));
                        bodyPrim.CreateAttribute(deformableBodyDataCrcToken, SdfValueTypeNames->UCharArray).Set(dataVt);
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

    bool setupSurfaceDeformableBodyCookingParams(UsdPrim bodyPrim,
                                                 const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
                                                 omni::physx::SurfaceDeformableBodyCookingParams& params,
                                                 pxr::UsdPrim& srcMeshPrim,
                                                 pxr::VtArray<pxr::GfVec3f>& pxrSrcPointsInSim,
                                                 omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        pxr::TfType dbType =
            pxr::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(pxr::OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        if (!bodyPrim.HasAPI(dbType))
        {
            CARB_LOG_ERROR("PhysX could not find USD prim or prim has no UsdPhysicsDeformableBodyAPI!");
            return false;
        }

        pxr::UsdPrim simMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.simMeshPath);
        pxr::UsdPrim collMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.collisionMeshPath);
        pxr::GfMatrix4d simToWorld =
            pxr::UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
        pxr::GfMatrix4d worldToSim = simToWorld.GetInverse();

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.", bodyPrim.GetPath().GetText());
            return false;
        }

        srcMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.cookingSrcMeshPath);
        pxr::UsdGeomMesh srcMesh(srcMeshPrim);
        pxr::TfToken srcMeshBindPoseToken = desc.cookingSrcMeshBindPoseToken;
        if (!srcMesh)
        {
            CARB_LOG_WARN(
                "Cooking failed, deformable has no valid source mesh for cooking: %s.", bodyPrim.GetPath().GetText());
            return false;
        }

        pxr::VtArray<int32_t> pxrSrcVertexIndices;
        pxr::VtArray<int32_t> pxrSrcVertexCounts;

        {
            pxr::TfType dpType = pxr::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(
                pxr::OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
            bool hasBindPoseAPI = !srcMeshBindPoseToken.IsEmpty() && srcMesh.GetPrim().HasAPI(dpType, srcMeshBindPoseToken);
            if (hasBindPoseAPI)
            {
                pxr::TfToken pointsAttrName = pxr::UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                    pxr::OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, srcMeshBindPoseToken);
                srcMesh.GetPrim().GetAttribute(pointsAttrName).Get(&pxrSrcPointsInSim);
            }
            else
            {
                srcMesh.GetPointsAttr().Get(&pxrSrcPointsInSim);
            }
            srcMesh.GetFaceVertexIndicesAttr().Get(&pxrSrcVertexIndices);
            srcMesh.GetFaceVertexCountsAttr().Get(&pxrSrcVertexCounts);

            // Transform src points to sim mesh space
            pxr::GfMatrix4d srcToWorld = srcMesh.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            pxr::GfMatrix4d srcToSim = srcToWorld * worldToSim;
            for (size_t i = 0; i < pxrSrcPointsInSim.size(); ++i)
            {
                pxr::GfVec3f& srcPoint = pxrSrcPointsInSim[i];
                srcPoint = srcToSim.Transform(srcPoint);
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
        UsdPrim& bodyPrim, bool xFormOnly, bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // If asynchronous cooking has been disabled
#endif
        uint64_t primStageId = UsdUtilsStageCache::Get().GetId(bodyPrim.GetStage()).ToLongInt();

        const bool isSimulated = checkParsed(primStageId, bodyPrim.GetPath(), omni::physx::usdparser::eSurfaceDeformableBody);
        if (isSimulated)
        {
            // refer to cookVolumeDeformableBodyInternal why we don't warn here
            return;
        }

        omni::physx::SurfaceDeformableBodyCookingParams params;
        pxr::UsdPrim srcMeshPrim;
        VtArray<GfVec3f> pxrSrcPointsInSim;
        omni::physx::usdparser::MeshKey customMeshCrc;
        if (!setupSurfaceDeformableBodyCookingParams(bodyPrim, desc, params, srcMeshPrim, pxrSrcPointsInSim, customMeshCrc))
        {
            CARB_LOG_ERROR(
                "Surface deformable body, failed to setup cooking params, prim: %s", bodyPrim.GetPath().GetText());
            return;
        }

        omni::physx::usdparser::MeshKey originalCrc =
            omni::physx::usdparser::loadMeshKey(bodyPrim, deformableBodyDataCrcToken);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.primId = asInt(srcMeshPrim.GetPath());
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
        request.primId = asInt(srcMeshPrim.GetPath());

        uint64_t bodyPrimId = asInt(bodyPrim.GetPath());
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
            UsdStageWeakPtr stage = omni::physx::OmniPhysX::getInstance().getStage();
            if (stage)
            {
                SdfPath bodyPrimPath = intToPath(bodyPrimId);
                UsdPrim bodyPrim = stage->GetPrimAtPath(bodyPrimPath);
                TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
                if (bodyPrim && bodyPrim.HasAPI(dbType))
                {
                    if (result.result == omni::physx::PhysxCookingResult::eVALID)
                    {
                        if (result.cookedData)
                        {
                            omni::physx::PhysxCookingSurfaceDeformableBodyData data;
                            cookingService.readSurfaceDeformableBodyData(data, *result.cookedData);
                            weakPtrToThis->storeSurfaceDeformableBodyDataToUsd(data, desc, bodyPrim, result.cookedDataCRC);
                        }
                    }
                    else
                    {
                        omni::physx::usdparser::MeshKey crc_zero;
                        VtArray<uchar> dataVt;
                        dataVt.resize(sizeof(omni::physx::usdparser::MeshKey));
                        memcpy(dataVt.data(), &crc_zero, sizeof(omni::physx::usdparser::MeshKey));
                        bodyPrim.CreateAttribute(deformableBodyDataCrcToken, SdfValueTypeNames->UCharArray).Set(dataVt);
                    }
                }
            }
        };

        m_cookingServicePrivate.requestSurfaceDeformableBodyCookedData(m_asyncContext, request, params);
        return;
    }

    virtual bool cookDeformableVolumeMesh(::physx::PxDefaultMemoryOutputStream& outStream,
                                          const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          UsdPrim bodyPrim,
                                          bool asynchronous) final
    {
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this); // block USD notice handlers
        return cookDeformableVolumeMeshInternal(outStream, desc, bodyPrim, asynchronous);
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

    bool setupDeformableVolumeMeshCookingParams(UsdPrim bodyPrim,
                                                const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                                omni::physx::DeformableVolumeMeshCookingParams& params,
                                                VtArray<GfVec3f>& pxrSimPoints,
                                                VtArray<GfVec3f>& pxrSimBindPoints,
                                                VtArray<GfVec4i>& pxrSimIndices,
                                                VtArray<GfVec3f>& pxrCollBindPointsInSim,
                                                VtArray<GfVec4i>& pxrCollIndices,
                                                VtArray<GfVec3i>& pxrCollSurfaceIndices)
    {
        // usd validation
        TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        TfType simType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);

        if (!bodyPrim || !bodyPrim.HasAPI(dbType))
        {
            CARB_LOG_WARN("Cooking failed, deformable body prim %s is invalid.", bodyPrim.GetPath().GetText());
            return false;
        }

        pxr::UsdPrim simMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.simMeshPath);
        if (!simMeshPrim || !simMeshPrim.IsA<pxr::UsdGeomTetMesh>() || !simMeshPrim.HasAPI(simType))
        {
            CARB_LOG_WARN("Cooking failed, simulation mesh prim %s is invalid.", bodyPrim.GetPath().GetText());
            return false;
        }

        if (simMeshPrim != bodyPrim && simMeshPrim.GetParent() != bodyPrim)
        {
            CARB_LOG_WARN(
                "Cooking failed, simulation mesh %s either has to be identical with bodyPrim %s or directly parented under it.",
                simMeshPrim.GetPath().GetText(), bodyPrim.GetPath().GetText());
            return false;
        }

        pxr::UsdPrim collMeshPrim = bodyPrim.GetStage()->GetPrimAtPath(desc.collisionMeshPath);

        if (!collMeshPrim || !collMeshPrim.IsA<pxr::UsdGeomTetMesh>())
        {
            CARB_LOG_WARN("Cooking failed, collision mesh prim %s is invalid.", bodyPrim.GetPath().GetText());
            return false;
        }

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.", bodyPrim.GetPath().GetText());
            return false;
        }

        GfMatrix4d simToWorld = UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
        GfMatrix4d worldToSim = simToWorld.GetInverse();

        // read simulation mesh rest shape for cooking (until the SDK supports a proper rest shape)
        // need to make sure the rest shape is compatible with the tetmesh topology
        {
            VtArray<GfVec3f> simPoints;
            VtArray<GfVec4i> simTetVertexIndices;
            VtArray<GfVec3f> simRestShapePoints;
            VtArray<GfVec4i> simRestTetVtxIndices;

            UsdGeomPointBased(simMeshPrim).GetPointsAttr().Get(&simPoints);
            UsdGeomTetMesh(simMeshPrim).GetTetVertexIndicesAttr().Get(&simTetVertexIndices);
            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Get(&simRestShapePoints);
            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices).Get(&simRestTetVtxIndices);

            bool mismatch = simPoints.size() != simRestShapePoints.size() ||
                            simTetVertexIndices.size() != simRestTetVtxIndices.size() ||
                            std::memcmp(simTetVertexIndices.data(), simRestTetVtxIndices.data(),
                                         sizeof(GfVec4i) * simTetVertexIndices.size()) != 0;

            if (mismatch)
            {
                CARB_LOG_WARN(
                    "Cooking failed, UsdGeomTetMesh not compatible with rest attributes in DeformableVolumeSimAPI, %s",
                     bodyPrim.GetPath().GetText());
                return false;
            }

            pxrSimPoints.swap(simRestShapePoints);
            pxrSimIndices.swap(simTetVertexIndices);
        }

        if (collMeshPrim != simMeshPrim)
        {
            // Need to construct embedding for collision mesh:
            // (simulation bind pose, sim mesh indices == rest shape indices, collision bind pose) -> embedding
            // (embedding, sim rest shape points, rest shape topo> -> collision rest shape
            VtArray<GfVec3f> simMeshBindPoints;
            VtArray<GfVec3f> collMeshBindPoints;
            {
                TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
                UsdAttribute simBindPointsAttr = getPosePointsOrPointsAttr(simMeshPrim, poseType, desc.simMeshBindPoseToken);
                UsdAttribute collBindPointsAttr = getPosePointsOrPointsAttr(collMeshPrim, poseType, desc.collisionMeshBindPoseToken);
                if (!simBindPointsAttr || !collBindPointsAttr)
                {
                    CARB_LOG_ERROR("Cooking failed: %s", bodyPrim.GetPath().GetText());
                    return false;
                }
                simBindPointsAttr.Get(&simMeshBindPoints);
                collBindPointsAttr.Get(&collMeshBindPoints);
            }

            // Transform collision points to sim space
            GfMatrix4d collToWorld = UsdGeomImageable(collMeshPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
            GfMatrix4d collToSim = collToWorld * worldToSim;
            for (size_t i = 0; i < collMeshBindPoints.size(); ++i)
            {
                collMeshBindPoints[i] = collToSim.Transform(collMeshBindPoints[i]);
            }
            pxrCollBindPointsInSim.swap(collMeshBindPoints);
            pxrSimBindPoints.swap(simMeshBindPoints);
            UsdGeomTetMesh(collMeshPrim).GetTetVertexIndicesAttr().Get(&pxrCollIndices);

            if (pxrSimBindPoints.size() != pxrSimPoints.size())
            {
                CARB_LOG_ERROR("Cooking failed, sim mesh bind pose points incompatible with points: %s", bodyPrim.GetPath().GetText());
                return false;
            }
        }

        // read surface face vertices be from the collsion mesh, even if sim and coll mesh alias
        {
            pxr::UsdGeomTetMesh(collMeshPrim).GetSurfaceFaceVertexIndicesAttr().Get(&pxrCollSurfaceIndices);
            if (pxrCollSurfaceIndices.size() == 0)
            {
                CARB_LOG_WARN("Cooking failed, collision mesh UsdGeomTetMesh needs to have "
                              "surfaceFaceVertexIndices set, %s.", collMeshPrim.GetPath().GetText());
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
                                          UsdPrim bodyPrim,
                                          bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // If asynchronous cooking is disabled
#endif

        omni::physx::DeformableVolumeMeshCookingParams params;
        VtArray<GfVec3f> pxrSimPoints;
        VtArray<GfVec3f> pxrSimBindPoints;
        VtArray<GfVec4i> pxrSimIndices;
        VtArray<GfVec3f> pxrCollBindPointsInSim;
        VtArray<GfVec4i> pxrCollIndices;
        VtArray<GfVec3i> pxrCollSurfaceIndices;
        if (!setupDeformableVolumeMeshCookingParams(bodyPrim, desc, params, pxrSimPoints, pxrSimBindPoints, pxrSimIndices,
                                                    pxrCollBindPointsInSim, pxrCollIndices, pxrCollSurfaceIndices))
        {
            CARB_LOG_ERROR(
                "Deformable volume mesh, failed to setup cooking params, prim: %s", bodyPrim.GetPath().GetText());
            return false;
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.primStageId = UsdUtilsStageCache::Get().GetId(bodyPrim.GetStage()).ToLongInt();
        request.primId = 0; // request without input source
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
     *  Samples particles on a mesh using poisson sampling
     * 
     *  @param usdPrim : The UsdGeomMesh we are sampling
     *  @param asynchronous : If false, it will sample the mesh synchronously (blocking). If true, it will start a background cooking task for it.
     */
    virtual void poissonSampleMesh(UsdPrim& usdPrim, const omni::physx::usdparser::ParticleSamplingDesc& desc, bool forceResampling, bool asynchronous) final
    {
        // Block USD notification handlers while in this call
        lock_guard _lock(m_mutex); // Lock the API mutex
        ScopedBlockUSDUpdates _block(this);
        poissonSampleMeshInternal(usdPrim, desc, forceResampling, asynchronous);
    }

    void poissonSampleMeshInternal(UsdPrim& usdPrim,
                                   const omni::physx::usdparser::ParticleSamplingDesc& desc,
                                   bool forceResampling, 
                                   bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // If asynchronous cooking has been disabled
#endif
        UsdGeomMesh geomMesh(usdPrim);
        if (!geomMesh)
            return;

        UsdAttribute pointsAttr = geomMesh.GetPointsAttr();
        if (pointsAttr.GetNumTimeSamples() > 0)
        {
            CARB_LOG_ERROR("Particle sampler component does not support meshes with time samples, prim: %s",
                           usdPrim.GetPath().GetText());
            return;
        }

        omni::physx::ParticlePoissonSamplingCookingParams params;
        GfMatrix4d rigidTransform;
        if (!setupParticlePoissonSamplingCookingParams(usdPrim, desc, rigidTransform, params))
        {
            CARB_LOG_ERROR("Particle sampler, failed to setup cooking params, prim: %s",
                           usdPrim.GetPath().GetText());
            return;
        }

        omni::physx::usdparser::MeshKey originalCrc = omni::physx::usdparser::loadMeshKey(usdPrim, particleSamplingCrcToken);
        if (!forceResampling)
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            // if forceResampling is off, we skip the early out to call processSamplingResults with
            // registerOriginalCount == true
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
            request.primId = asInt(usdPrim.GetPath());
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
        request.primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();
        request.primId = asInt(usdPrim.GetPath());
        request.primMeshText = { usdPrim.GetPath().GetText(), strlen(usdPrim.GetPath().GetText()) };
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
            UsdStageWeakPtr stage = omni::physx::OmniPhysX::getInstance().getStage();
            if (stage)
            {
                SdfPath primPath = intToPath(result.request->primId);
                UsdPrim usdPrim = stage->GetPrimAtPath(primPath);
                PhysxSchemaPhysxParticleSamplingAPI samplingAPI(usdPrim);
                if (samplingAPI)
                {
                    if (result.cookedData)
                    {
                        omni::physx::PhysxCookingParticlePoissonSamplingData data;
                        cookingService.readParticlePoissonSamplingData(data, *result.cookedData);

                        // set the sampling distance in USD in case we made a correction
                        float samplingDistance;
                        samplingAPI.GetSamplingDistanceAttr().Get(&samplingDistance); 
                        if (samplingDistance < desc.samplingDistance && samplingDistance > 0.0f)
                        {
                            samplingAPI.CreateSamplingDistanceAttr().Set(desc.samplingDistance);
                        }

                        omni::physx::usdparser::storeMeshKey(usdPrim, particleSamplingCrcToken, result.cookedDataCRC);

                        const GfVec3f* samples = reinterpret_cast<const GfVec3f*>(data.positions);
                        const uint32_t samplesSize = data.positionsSize;
                        static_assert(sizeof(GfMatrix3d) == sizeof(params.shearScale));
                        const GfMatrix3d& shearScaleTransform = *reinterpret_cast<const GfMatrix3d*>(params.shearScale);

                        // if the original USD crc matches with the newly computed crc we call processSamplingResults
                        // for registering particle counts of newly instantiated samplers without writing particles, in
                        // order to avoid overwriting pre-simulated state.
                        bool registerOriginalCount = (originalCrc == result.cookedDataCRC);

                        omni::physx::particles::PhysxParticleFactory::processSamplingResults(
                            primPath, desc.particleSetPath, samples, samplesSize,
                            desc.pointWidth, rigidTransform, shearScaleTransform,
                            registerOriginalCount);
                    }
                }
            }
        };
        m_cookingServicePrivate.requestParticlePoissonSamplingCookedData(m_asyncContext, request, params);
    }

    virtual void cookParticleClothDeprecated(UsdPrim& usdPrim, bool asynchronous) final
    {
        // Block USD notification handlers while in this call
        lock_guard _lock(m_mutex); // Lock the API mutex
        ScopedBlockUSDUpdates _block(this);
        cookParticleClothInternalDeprecated(usdPrim, false, asynchronous);
    }

    /**
     * Fills in ParticleClothMeshCookingParamsDeprecated for cooking input.
     * The usdPrim is parsed for necessary parameters, pxrRestPoints is filled in and referenced from the parameters.
     * The customMeshCrc is based on pxrRestPoints as opposed to the default mesh key.
     */
    bool setupParticleClothMeshCookingParamsDeprecated(
        pxr::UsdPrim& usdPrim,
        omni::physx::ParticleClothMeshCookingParamsDeprecated& params,
        pxr::VtArray<GfVec3f>& pxrRestPoints,
        omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        PhysxSchemaPhysxParticleClothAPI particleCloth(usdPrim);
        PhysxSchemaPhysxAutoParticleClothAPI autoParticleCloth(usdPrim);
        UsdGeomMesh mesh(usdPrim);
        if (!particleCloth || !mesh)
        {
            CARB_LOG_ERROR("PhysX could not find USD prim or prim has no PhysxSchemaPhysxParticleClothAPI!");
            return false;
        }

        // test whether any cooking is required at all
        float pressure;
        particleCloth.GetPressureAttr().Get(&pressure);
        params.isInflatable = pressure > 0.0f;
        params.needsSprings = !!autoParticleCloth;

        // check whether to enable welding
        params.enableWelding = params.needsSprings;
        if (params.enableWelding)
        {
            bool disableWelding;
            autoParticleCloth.GetDisableMeshWeldingAttr().Get(&disableWelding);
            params.enableWelding = !disableWelding;
        }

        // load input and compute crc
        VtArray<int32_t> pxrVertexIndices;
        VtArray<int32_t> pxrVertexCounts;

        UsdAttribute restPointsAttr = particleCloth.GetRestPointsAttr();
        if (restPointsAttr.HasAuthoredValue())
        {
            particleCloth.GetRestPointsAttr().Get(&pxrRestPoints);

            VtArray<GfVec3f> points;
            mesh.GetPointsAttr().Get(&points);
            if (pxrRestPoints.size() != points.size())
            {
                CARB_LOG_ERROR(
                    "PhysxSchemaPhysxParticleClothAPI:restPoints and UsdGeomMesh:points have inconsistent sizes.");
                return false;
            }
        }
        else
        {
            UsdAttribute pointsAttr = mesh.GetPointsAttr();
            if (pointsAttr.GetNumTimeSamples() > 0)
            {
                CARB_LOG_ERROR("Particle cloth component does not support meshes with time samples.");
                return false;
            }
            mesh.GetPointsAttr().Get(&pxrRestPoints);
        }

        mesh.GetFaceVertexIndicesAttr().Get(&pxrVertexIndices);
        mesh.GetFaceVertexCountsAttr().Get(&pxrVertexCounts);

        // Reject if data is missing
        if (pxrRestPoints.empty() || pxrVertexIndices.empty() || pxrVertexCounts.empty())
        {
            return false;
        }

        customMeshCrc = computeMeshKey(pxrRestPoints, pxrVertexIndices, pxrVertexCounts);
        params.restPoints = omni::span<carb::Float3>(reinterpret_cast<carb::Float3*>(pxrRestPoints.data()), pxrRestPoints.size());
        params.springStretchStiffness = 0.0f;
        params.springBendStiffness = 0.0f;
        params.springShearStiffness = 0.0f;
        params.springDamping = 0.0f;
        if (autoParticleCloth)
        {
            autoParticleCloth.GetSpringStretchStiffnessAttr().Get(&params.springStretchStiffness);
            autoParticleCloth.GetSpringBendStiffnessAttr().Get(&params.springBendStiffness);
            autoParticleCloth.GetSpringShearStiffnessAttr().Get(&params.springShearStiffness);
            autoParticleCloth.GetSpringDampingAttr().Get(&params.springDamping);
        }

        return true;
    }

    /***
    * A method to store the cooked particle cloth data to the corresponding USD primitive attributes
    */
    void storeParticleClothDataToUsdDeprecated(const carb::Int2* springIndices, const uint32_t springIndicesSize,
        const float* springStiffnesses, const float* springDampings, const float* springRestLengths,
        const uint32_t* weldedTriangleIndices, const uint32_t weldedTriangleIndicesSize,
        const uint32_t* verticesRemapToWeld, const uint32_t verticesRemapToWeldSize,
        const uint32_t* verticesRemapToOrig, const uint32_t verticesRemapToOrigSize,
        const float inflatableVolume,
        UsdPrim& prim, const omni::physx::usdparser::MeshKey& crc)
    {
        PhysxSchemaPhysxParticleClothAPI particleClothAPI(prim);

        //write crc
        VtArray<uchar> dataVt(sizeof(omni::physx::usdparser::MeshKey));
        memcpy(dataVt.data(), &crc, sizeof(omni::physx::usdparser::MeshKey));
        prim.CreateAttribute(physxParticleClothInputCrcToken, SdfValueTypeNames->UCharArray).Set(dataVt);

        //write data
        //don't overwrite springs if nothing to write
        //for some reason we even run cooking on manually authored cloth to guarantee some SDK limits,
        //in which case no springs are provided, because the original asset already has the springs.
        //if the cook fails for some reason, we still want to nullify the springs (hence the added check for zero crc)
        if (springIndicesSize > 0 || crc == omni::physx::usdparser::MeshKey())
        {
            VtArray<GfVec2i> pxrSpringIndices;
            omni::physx::copyBuffer(pxrSpringIndices, springIndices, springIndicesSize);
            particleClothAPI.GetSpringIndicesAttr().Set(pxrSpringIndices);

            VtArray<float> pxrSpringStiffnesses(springIndicesSize);
            memcpy(pxrSpringStiffnesses.data(), springStiffnesses, sizeof(float) * springIndicesSize);
            particleClothAPI.GetSpringStiffnessesAttr().Set(pxrSpringStiffnesses);

            VtArray<float> pxrSpringDampings(springIndicesSize);
            memcpy(pxrSpringDampings.data(), springDampings, sizeof(float) * springIndicesSize);
            particleClothAPI.GetSpringDampingsAttr().Set(pxrSpringDampings);

            VtArray<float> pxrSpringRestLengths(springIndicesSize);
            memcpy(pxrSpringRestLengths.data(), springRestLengths, sizeof(float) * springIndicesSize);
            particleClothAPI.GetSpringRestLengthsAttr().Set(pxrSpringRestLengths);
        }

        if (weldedTriangleIndicesSize > 0 && verticesRemapToWeldSize > 0 && verticesRemapToOrigSize > 0)
        {
            // store welded info
            UsdAttribute weldedTriangleIndicesAttr = particleClothAPI.GetPrim().GetAttribute(physxParticleWeldedTriangleIndicesToken);
            if (!weldedTriangleIndicesAttr.HasAuthoredValue())
            {
                weldedTriangleIndicesAttr = particleClothAPI.GetPrim().CreateAttribute(physxParticleWeldedTriangleIndicesToken, SdfValueTypeNames->UIntArray);
            }
            VtArray<uint32_t> pxrWeldedTriangleIndices(weldedTriangleIndicesSize);
            memcpy(pxrWeldedTriangleIndices.data(), weldedTriangleIndices, sizeof(uint32_t) * weldedTriangleIndicesSize);
            weldedTriangleIndicesAttr.Set(pxrWeldedTriangleIndices);

            UsdAttribute VerticesRemapToWeldAttr = particleClothAPI.GetPrim().GetAttribute(physxParticleWeldedVerticesRemapToWeldToken);
            if (!VerticesRemapToWeldAttr.HasAuthoredValue())
            {
                VerticesRemapToWeldAttr = particleClothAPI.GetPrim().CreateAttribute(physxParticleWeldedVerticesRemapToWeldToken, SdfValueTypeNames->UIntArray);
            }
            VtArray<uint32_t> pxrVerticesRemapToWeld(verticesRemapToWeldSize);
            memcpy(pxrVerticesRemapToWeld.data(), verticesRemapToWeld, sizeof(uint32_t) * verticesRemapToWeldSize);
            VerticesRemapToWeldAttr.Set(pxrVerticesRemapToWeld);

            UsdAttribute VerticesRemapToOrigAttr = particleClothAPI.GetPrim().GetAttribute(physxParticleWeldedVerticesRemapToOrigToken);
            if (!VerticesRemapToOrigAttr.HasAuthoredValue())
            {
                VerticesRemapToOrigAttr = particleClothAPI.GetPrim().CreateAttribute(physxParticleWeldedVerticesRemapToOrigToken, SdfValueTypeNames->UIntArray);
            }
            VtArray<uint32_t> pxrVerticesRemapToOrig(verticesRemapToOrigSize);
            memcpy(pxrVerticesRemapToOrig.data(), verticesRemapToOrig, sizeof(uint32_t) * verticesRemapToOrigSize);
            VerticesRemapToOrigAttr.Set(pxrVerticesRemapToOrig);
        }
        else
        {
            // delete stale welded info
            particleClothAPI.GetPrim().RemoveProperty(physxParticleWeldedTriangleIndicesToken);
            particleClothAPI.GetPrim().RemoveProperty(physxParticleWeldedVerticesRemapToWeldToken);
            particleClothAPI.GetPrim().RemoveProperty(physxParticleWeldedVerticesRemapToOrigToken);
        }

        UsdAttribute inflatableVolumeAttr = particleClothAPI.GetPrim().GetAttribute(physxParticleInflatableVolumeToken);
        if (!inflatableVolumeAttr.HasAuthoredValue())
        {
            inflatableVolumeAttr = particleClothAPI.GetPrim().CreateAttribute(physxParticleInflatableVolumeToken, SdfValueTypeNames->Float);
        }
        inflatableVolumeAttr.Set(inflatableVolume);

        //reset dependent values (if they have been written already before)
        //not clear yet whether we have to do this for particle cloth
    }

    /**
    * Performs the cooking operation on a particle cloth mesh associated with a particular UsdGeomMesh primitive
    *
    * @param usdPrim : The UsdGeomMesh we are cooking
    * @param xFormOnly : If true, the cooking was triggered excusively for a transform update. 
    * @param asynchronous : If false, it will cook the particle cloth synchronously (blocking). If true, it will start a background cooking task for it.
    * @return : Returns true if usd data has been written
    */
    void cookParticleClothInternalDeprecated(UsdPrim& usdPrim, bool xFormOnly, bool asynchronous)
    {
#if !USE_ASYNC_COOKING
        asynchronous = false; // If asynchronous cooking has been disabled
#endif

        uint64_t primStageId = UsdUtilsStageCache::Get().GetId(usdPrim.GetStage()).ToLongInt();

        const bool isSimulated = checkParsed(primStageId, usdPrim.GetPath(), omni::physx::usdparser::eParticleCloth);
        if (isSimulated)
        {
            //note: corresponding warnings are issued in omni::physx::warnParticleClothNoRuntimeCooking
            return;
        }

        omni::physx::ParticleClothMeshCookingParamsDeprecated params;
        VtArray<GfVec3f> restPoints;
        omni::physx::usdparser::MeshKey customMeshCrc;
        if (!setupParticleClothMeshCookingParamsDeprecated(usdPrim, params, restPoints, customMeshCrc))
        {
            CARB_LOG_ERROR("Particle cloth mesh, failed to setup cooking params, prim: %s", usdPrim.GetPath().GetText());
            return;
        }

        omni::physx::usdparser::MeshKey originalCrc =
            omni::physx::usdparser::loadMeshKey(usdPrim, physxParticleClothInputCrcToken);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.primId = asInt(usdPrim.GetPath());
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
            request.meshKey = customMeshCrc;
            request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
            omni::physx::usdparser::MeshKey cookedDataCRC;
            request.onFinished = [&cookedDataCRC](const omni::physx::PhysxCookingComputeResult& result) {
                cookedDataCRC = result.cookedDataCRC;
            };
            m_cookingServicePrivate.requestParticleClothMeshCookedDataDeprecated(m_asyncContext, request, params);
            if (cookedDataCRC == originalCrc)
            {
                return;
            }
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.primStageId = primStageId;
        request.primId = asInt(usdPrim.GetPath());
        request.primMeshText = { usdPrim.GetPath().GetText(), strlen(usdPrim.GetPath().GetText()) };
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
        request.meshKey = customMeshCrc;

        auto weakPtrToThis = TfCreateWeakPtr(this);

        recordStatisticsRequestFor(request);
        request.onFinished = [weakPtrToThis](const omni::physx::PhysxCookingComputeResult& result) {
            CARB_ASSERT(result.request->dataType == omni::physx::PhysxCookingDataType::ePARTICLE_CLOTH_DEPRECATED);
            if(!weakPtrToThis)
            {
                return; // means that CookingDataAsync destructor was called, so this task is cancelled
            }
            weakPtrToThis->recordStatisticsResultFor(result);

            omni::physx::IPhysxCookingServicePrivate& cookingService = weakPtrToThis->m_cookingServicePrivate;
            UsdStageWeakPtr stage = omni::physx::OmniPhysX::getInstance().getStage();
            if (stage)
            {
                SdfPath primPath = intToPath(result.request->primId);
                UsdPrim usdPrim = stage->GetPrimAtPath(primPath);

                if (usdPrim && usdPrim.IsA<UsdGeomMesh>())
                {
                    if (result.result == omni::physx::PhysxCookingResult::eVALID)
                    {
                        if (result.cookedData)
                        {
                            omni::physx::PhysxCookingParticleClothMeshDataDeprecated data;
                            cookingService.readParticleClothMeshDataDeprecated(data, *result.cookedData);

                            weakPtrToThis->storeParticleClothDataToUsdDeprecated(data.springIndices, data.springIndicesSize,
                                data.springStiffnesses, data.springDampings, data.springRestLengths, data.weldedTriangleIndices, data.weldedTriangleIndicesSize,
                                data.verticesRemapToWeld, data.verticesRemapToWeldSize,
                                data.verticesRemapToOrig, data.verticesRemapToOrigSize,
                                data.inflatableVolume,
                                usdPrim, result.cookedDataCRC);
                        }
                    }
                    else
                    {
                        omni::physx::usdparser::MeshKey crc_zero; // need to write crc 0, so crc test fails for invalid cook
                        weakPtrToThis->storeParticleClothDataToUsdDeprecated(
                            nullptr, 0, nullptr, nullptr, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, 0.0f,
                            usdPrim, crc_zero);
                    }
                }

            }
        };
        m_cookingServicePrivate.requestParticleClothMeshCookedDataDeprecated(m_asyncContext, request, params);
    }

    virtual void release(void) final
    {
        delete this;
    }

    /**
    * This is our USD notice listener callback. It is here that we see if any collision related attributes we react to
    * have been changed and, if so, we post the corresponding USD path to be cooked if necessary
    */
    void handle(const class UsdNotice::ObjectsChanged& objectsChanged)
    {
        TRACE_FUNCTION();
        CARB_PROFILE_ZONE(0, "CookingDataAsync::noticeHandler");

#if USD_USD_NOTICE_LISTENER
        // If we are being notified about changes we ourselves are making, then ignore the changes
        if (m_blockUsdUpdate > 0)
        {
            return;
        }

        if (omni::physx::OmniPhysX::getInstance().getStage() != objectsChanged.GetStage())
        {
            return;
        }

        TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);

        lock_guard _lock(m_mutex);

        for (const SdfPath& path : objectsChanged.GetResyncedPaths())
        {
            if (path.IsAbsoluteRootOrPrimPath())
            {
                // Get the root path relative to this attribute
                const SdfPath primPath =
                    objectsChanged.GetStage()->GetPseudoRoot().GetPath() == path ? path : path.GetPrimPath();

                const auto& changedFields = objectsChanged.GetChangedFields(primPath);
                if (std::find(changedFields.begin(), changedFields.end(), UsdTokens->apiSchemas) !=
                    changedFields.end())
                {
                    // ApiSchemas change
                    // Schedule for processing outside of the notification handler so it's not blocking for huge scenes
                    m_primApiSchemasChangeRefreshSet.insert(primPath);

                    // Can we do this more efficiently?
                    UsdPrim prim = objectsChanged.GetStage()->GetPrimAtPath(primPath);
                    if (prim && prim.HasAPI(poseType))
                    {
                        TfTokenVector allAPIs = prim.GetAppliedSchemas();
                        TfToken poseTypeName = UsdSchemaRegistry::GetAPISchemaTypeName(poseType);
                        for (const auto& api : allAPIs)
                        {
                            std::pair<TfToken, TfToken> typeNameAndInstance = UsdSchemaRegistry::GetTypeNameAndInstance(api);
                            if (typeNameAndInstance.first == poseTypeName)
                            {
                                TfToken pointAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                                    OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, typeNameAndInstance.second);
                                TfToken purposesAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                                    OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, typeNameAndInstance.second);

                                m_collisionTokens.insert(pointAttrName);
                                m_collisionTokens.insert(purposesAttrName);
                            }
                        }
                    }
                }
                else
                {
                    UsdPrim prim = objectsChanged.GetStage()->GetPrimAtPath(primPath);
                    m_primAddedRemovedRefreshSet.insert(primPath);
                }
            }
        }

        // Iterate through objects which have changed
        for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
        {
            // See if it corresponds to an attribute
            const bool isAttributePath = path.IsPropertyPath();
            if ( isAttributePath )
            {
                TfToken const& attrName = path.GetNameToken();

                // If it's an attribute see if it is any of the collision attributes we care about for cooking
                bool isCollisionAttribute = false;
                TokenSet::const_iterator it = m_collisionTokens.find(attrName);
                if (it != m_collisionTokens.cend())
                {
                    isCollisionAttribute = true;
                }

                // Get the root path relative to this attribute
                const SdfPath primPath = objectsChanged.GetStage()->GetPseudoRoot().GetPath() == path ? path : path.GetPrimPath();
                if ( isCollisionAttribute)
                {
                    // Schedule it to possibly be recooked if needed.
                    addPrimRefreshSet(primPath);
                }
                else
                {
                    // Or if it's an xform change
                    const bool isXform = UsdGeomXformable::IsTransformationAffectedByAttrNamed(attrName);
                    if (isXform)
                    {
                        m_primXformRefreshSet.insert(primPath);
                    }
                }
                
            }
        }
#endif
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

#if USD_USD_NOTICE_LISTENER
    TfNotice::Key m_usdNoticeListenerKey;
#endif
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
                // printf("CookingProgress:%0.5f\n", value );
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

    virtual bool isLocalMeshCacheEnabled() const override final { return m_cookingServicePrivate.isLocalMeshCacheEnabled(); }
    
    virtual void setLocalMeshCacheEnabled(bool val) override final { m_cookingServicePrivate.setLocalMeshCacheEnabled(val); }

    // local mesh cache size in MB
    virtual uint32_t getLocalMeshCacheSize() const override final { return m_cookingServicePrivate.getLocalMeshCacheSize(); }

    // local mesh cache size in MB
    virtual void setLocalMeshCacheSize(uint32_t val) override final { m_cookingServicePrivate.setLocalMeshCacheSize(val); }

    virtual void resetLocalMeshCacheContents() override final { m_cookingServicePrivate.resetLocalMeshCacheContents(); }

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

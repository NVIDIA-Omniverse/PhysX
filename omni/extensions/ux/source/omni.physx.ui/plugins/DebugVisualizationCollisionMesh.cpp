// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "DebugVisualizationCollisionMesh.h"
#include "DebugMeshPrim.h"
#include "DebugColors.h"
#include <common/ui/ImguiDrawingUtils.h>

#include "carb/Defines.h"
#include <carb/extras/Hash.h>
#include <carb/profiler/Profile.h>

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxVisualization.h>
#include <common/utilities/Utilities.h>
#include <omni/physx/IPhysxCooking.h>
#include <omni/physx/PhysxTokens.h>

#include <omni/kit/EditorUsd.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <carb/extras/Tokens.h>
#include <omni/kit/PythonInterOpHelper.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/utilities/PrimUtilities.h>

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/SimStageWithHistory.h>


#include <math.h>

#include <unordered_map>
#include <unordered_set>

#include <stdlib.h>
#include <string.h>

#if CARB_COMPILER_MSC
#pragma warning(disable:4996)
#endif

static omni::physx::IPhysxVisualization* gPhysXVisualization = nullptr;
extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;
extern pxr::UsdStageRefPtr gStage;  // Declare external reference to the global stage handle

namespace omni
{
namespace physx
{
    namespace ui
    {
static float gExplodeViewDistance=0;

typedef std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> PathSet;
typedef std::unordered_map<pxr::SdfPath,pxr::SdfPath, pxr::SdfPath::Hash> PathMap;

static pxr::TfToken getCollisionMeshToken(void) // get the token we use to signify a prim is a debug collision mesh
{
    static pxr::TfToken gCollisionMeshToken("CollisionMesh");
    return gCollisionMeshToken;
}

static pxr::TfToken getCollisionMeshParentToken(void) // get the token we use to signify a prim is a debug collision mesh
{
    static pxr::TfToken gCollisionMeshParentToken("CollisionMeshParent");
    return gCollisionMeshParentToken;
}

/**
* Sets a key/value pair on the custom meta data field for this prim.
*
* @param prim : The primitive we wish to modify
* @param key : The key for this metadata expressed as a pxr::TfToken
* @param value : A const char string to assign as the 'data' portion for this metadata
*/
static void setMetaData(pxr::UsdPrim &prim, const pxr::TfToken &key, const char *value)
{
    prim.SetMetadataByDictKey<const char *>(pxr::SdfFieldKeys->CustomData, key, value);
}

static bool isCollisionMesh(const pxr::UsdPrim &prim)
{
    bool ret = false;

    pxr::TfToken ctoken = getCollisionMeshToken();
    std::string metadata;
    if (primutils::getMetaData(prim, ctoken, metadata))
    {
        ret = true;
    }

    return ret;
}

// Don't show this session prim in the main stage window
static void hideSessionPrim(pxr::UsdPrim &prim)
{
    omni::kit::EditorUsd::setHideInStageWindow(prim, true);
    omni::kit::EditorUsd::setNoDelete(prim, true);
    omni::kit::EditorUsd::setAlwaysPickModel(prim, true);
}

// We use this method to copy the matrix transform from the source prim
// to the mirrored xform we created in the session layer
static void copyWorldTransform(const pxr::UsdPrim &source,pxr::UsdGeomXform &dest)
{
    pxr::UsdGeomXformCache xfCache;
    pxr::GfMatrix4d worldRel = xfCache.GetLocalToWorldTransform(source);
    pxr::UsdGeomXformOp xform = dest.MakeMatrixXform();
    if (xform)
    {
        xform.Set(worldRel);
    }
}

static void setVisibility(pxr::UsdPrim &prim,bool visible,bool isParent)
{
    pxr::UsdGeomImageable imageable(prim);
    if ( imageable )
    {
        if ( visible )
        {
            imageable.MakeVisible();
        }
        else
        {
            imageable.MakeInvisible();
            if ( isParent )
            {
                setMetaData(prim,getCollisionMeshParentToken(),"parent");
            }
        }
    }
}

template<typename T>
static T fm_normalize(T* n) // normalize this vector
{
    T dist = (T)sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (dist > 0.0000001f)
    {
        T mag = 1.0f / dist;
        n[0] *= mag;
        n[1] *= mag;
        n[2] *= mag;
    }
    else
    {
        n[0] = 1;
        n[1] = 0;
        n[2] = 0;
    }

    return dist;
}


class CollisionPrim
{
public:
    void syncTransformToUSD(float diagonalDistance,float explodeDistance)
    {
        const carb::Double3 dir = getExplodedVector(diagonalDistance, explodeDistance, mRootCenter, mMeshCenter );
        pxr::UsdPrim p = gStage->GetPrimAtPath(mPrim);
        if ( p )
        {
            omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
            pxr::GfMatrix4d xform(1.0);
            xform.SetTranslate(pxr::GfVec3d(dir.x,dir.y,dir.z));
            omni::usd::UsdUtils::setLocalTransformMatrix(p,xform);
        }
    }

    static carb::Double3 getExplodedVector(double diagonalDistance,
                                           double explodeDistance,
                                           carb::Float3 rootCenter,
                                           carb::Float3 meshCenter)
    {
        double dist = diagonalDistance * explodeDistance;
        carb::Double3 dir;
        dir.x = meshCenter.x - rootCenter.x;
        dir.y = meshCenter.y - rootCenter.y;
        dir.z = meshCenter.z - rootCenter.z;
        fm_normalize(&dir.x);
        dir.x *= dist;
        dir.y *= dist;
        dir.z *= dist;
        return dir;
    }

    pxr::SdfPath    mPrim;
    carb::Float3    mRootCenter{};
    carb::Float3    mMeshCenter{};
};

// Represents a single prim with debug collision shapes that we are
// tracking
class CollisionMesh
{
public:
    void syncTransformToUSD(const pxr::UsdPrim& prim)
    {
        pxr::UsdPrim dest = gStage->GetPrimAtPath(mRootPrim);
        if (prim && dest)
        {
            pxr::UsdGeomXform xform(dest);
            omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
            copyWorldTransform(prim, xform);
            if (mCollisionPrims.size() > 1)
            {
                for (auto& i : mCollisionPrims)
                {
                    i.syncTransformToUSD(mDiagonalAxisLength, gExplodeViewDistance);
                }
            }
        }
    }

    bool setupFabricAttributes(omni::fabric::UsdStageId usdStageId)
    {
        return false;
        using namespace omni::fabric;
        const Token localMatrixToken(gLocalMatrixTokenString);
        const Token meshCenterToken("_collisionMeshDebugViewCenter");
        const Token meshRootToken("_collisionMeshDebugViewRoot");
        const Token meshDiagonalToken("_collisionMeshDebugViewDiagonal");
        const Token meshCollisionTag("_collisionMeshDebugTag");
        const Type typeDouble16(BaseDataType::eDouble, 16, 0, AttributeRole::eMatrix);
        const Type typeFloat3(BaseDataType::eFloat, 3, 0, AttributeRole::eVector);
        const Type typeFloat1(BaseDataType::eFloat);
        const Type tagType(BaseDataType::eTag);

        StageReaderWriterId fabricStageId = { 0 };
        IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<IStageReaderWriter>();
        if (iStageReaderWriter)
        {
            fabricStageId = iStageReaderWriter->get(usdStageId);
        }
        if (fabricStageId.id)
        {
            StageReaderWriter stage = fabricStageId;
            iStageReaderWriter->prefetchPrim(usdStageId, asInt(mRootPrim));
            stage.createAttribute(asInt(mRootPrim), localMatrixToken, typeDouble16);

            for (auto& i : mCollisionPrims)
            {
                Path primPath = asInt(i.mPrim.GetPrimPath());
                iStageReaderWriter->prefetchPrim(usdStageId, primPath);
                stage.createAttributes<5>(primPath, {
                                                        AttrNameAndType(tagType, meshCollisionTag),
                                                        AttrNameAndType(typeDouble16, localMatrixToken),
                                                        AttrNameAndType(typeFloat3, meshCenterToken),
                                                        AttrNameAndType(typeFloat3, meshRootToken),
                                                        AttrNameAndType(typeFloat1, meshDiagonalToken),
                                                    });
                carb::Float3& meshCenter = *stage.getAttributeWr<carb::Float3>(primPath, meshCenterToken);
                meshCenter = i.mMeshCenter;
                carb::Float3& meshRoot = *stage.getAttributeWr<carb::Float3>(primPath, meshRootToken);
                meshRoot = i.mRootCenter;
                float& diagonal = *stage.getAttributeWr<float>(primPath, meshDiagonalToken);
                diagonal = mDiagonalAxisLength;
            }
            return true;
        }
        return false;
    }

    bool syncRootTransformToFabric(omni::fabric::UsdStageId usdStageId, const pxr::UsdPrim& prim)
    {
        return false;
        using namespace omni::fabric;
        const Token localMatrixToken(gLocalMatrixTokenString);
        StageReaderWriterId fabricStageId = { 0 };
        IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<IStageReaderWriter>();
        if (iStageReaderWriter)
        {
            fabricStageId = iStageReaderWriter->get(usdStageId);
        }
        if (fabricStageId.id)
        {
            StageReaderWriter stage = fabricStageId;
            pxr::GfMatrix4d& rootMat = *stage.getAttributeWr<pxr::GfMatrix4d>(asInt(mRootPrim), localMatrixToken);
            if (prim)
            {
                rootMat = pxr::UsdGeomXformable(prim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            }
            return true;
        }
        return false;
    }

    static bool updateAllCollisionMeshesChildrenWithFabric(omni::fabric::UsdStageId usdStageId)
    {
        return false;
        using namespace omni::fabric;
        const Type tagType(BaseDataType::eTag);
        const Token meshCollisionTag("_collisionMeshDebugTag");

        StageReaderWriterId fabricStageId = { 0 };
        IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<IStageReaderWriter>();
        if (iStageReaderWriter)
        {
            fabricStageId = iStageReaderWriter->get(usdStageId);
        }
        if (fabricStageId.id)
        {
            StageReaderWriter stage = fabricStageId;
            // Look for all children collision meshes
            const set<AttrNameAndType_v2> requiredAll = {
                AttrNameAndType_v2(tagType, meshCollisionTag),
            };
            stage.findPrims(requiredAll);

            PrimBucketList primBuckets = stage.findPrims(requiredAll);
            updateTransformBucket(stage, primBuckets);
            return true;
        }
        return false;
    }

    static void updateTransformBucket(omni::fabric::StageReaderWriter& stage, omni::fabric::PrimBucketList& primBuckets)
    {
        using namespace omni::fabric;
        const Token localMatrixToken(gLocalMatrixTokenString);
        const Token meshCenterToken("_collisionMeshDebugViewCenter");
        const Token meshRootToken("_collisionMeshDebugViewRoot");
        const Token meshDiagonalToken("_collisionMeshDebugViewDiagonal");

        // This is a single bucket for sure, considering how it has been setup in setupFabricAttributes, but let's play
        // it safe and loop all buckets anyway
        for (size_t i = 0; i != primBuckets.bucketCount(); i++)
        {
            // All these three arrays have the same number of elements as built in setupFabricAttributes
            // Note that we're writing _localMatrix attributes because parent matrix has been written outside of this
            // loop
            gsl::span<pxr::GfMatrix4d> positions =
                stage.getAttributeArrayWr<pxr::GfMatrix4d>(primBuckets, i, localMatrixToken);
            gsl::span<const carb::Float3> centers =
                stage.getAttributeArrayRd<carb::Float3>(primBuckets, i, meshCenterToken);
            gsl::span<const carb::Float3> roots = stage.getAttributeArrayRd<carb::Float3>(primBuckets, i, meshRootToken);
            gsl::span<const float> diagonals = stage.getAttributeArrayRd<float>(primBuckets, i, meshDiagonalToken);

            // Update the positions of all the prims in the bucket, that will be nicely packed close to each other
            int index = 0;
            const carb::Float3* pCenters = centers.data();
            const carb::Float3* pRoots = roots.data();
            const float* pDiagonals = diagonals.data();
            for (pxr::GfMatrix4d& position : positions)
            {
                carb::Double3 dir = CollisionPrim::getExplodedVector(
                    pDiagonals[index], gExplodeViewDistance, pRoots[index], pCenters[index]);
                position.SetTranslate(pxr::GfVec3d(dir.x, dir.y, dir.z));
                index++;
            }
        }
    }

    // Here we remove all collision prims that are
    // associated with this instance
    void removeCollisionPrims(PathMap &pathMap)
    {
        // Remove all debug visualization collision prims
        // that were created
        for (auto &i:mCollisionPrims)
        {
            PathMap::iterator found = pathMap.find(i.mPrim);
            if ( found != pathMap.end() )
            {
                pathMap.erase(found);
            }
            gStage->RemovePrim(i.mPrim);
        }
        mCollisionPrims.clear();
        // Remove the session xform we created to mirror the
        // prim this debug visualization is associated with
        PathMap::iterator found = pathMap.find(mRootPrim);
        if (found != pathMap.end())
        {
            pathMap.erase(found);
        }
        gStage->RemovePrim(mRootPrim);
    }

    // This is the geom mesh xform we are mirroring
    void setRootPrim(const pxr::SdfPath &parentPrim,    // The prim we are mirroring
                     const pxr::SdfPath &rootPrim,      // the xform we are using to mirror in the session layer
                     float diagonalAxisLength,
                     PathMap &pathMap) // The remapping table from session prims back to the prim they are referencing
    {
        mDiagonalAxisLength = diagonalAxisLength;
        mRootPrim = rootPrim;
        pathMap[mRootPrim] = parentPrim;
    }

    // Add another prim which is associated with the
    // debug visualization of this instance
    void addCollisionPrim(const pxr::SdfPath &parentPrim,
                          const pxr::SdfPath &cprim,
                          const carb::Float3 &rootCenter,
                          const carb::Float3 &meshCenter,
                          PathMap &pathMap)
    {
        CollisionPrim cp;
        cp.mPrim = cprim;
        cp.mRootCenter = rootCenter;
        cp.mMeshCenter = meshCenter;
        mCollisionPrims.push_back(cp);
        pathMap[cprim] = parentPrim;
    }

    static void getShowPrimAndCollision(CollisionMeshDisplayMode mode, bool& showPrim, bool& showCollision)
    {
        showPrim = false;
        showCollision = false;

        switch (mode)
        {
        case CollisionMeshDisplayMode::BOTH:
            showPrim = true;
            showCollision = true;
            break;
        case CollisionMeshDisplayMode::GRAPHICS:
            showPrim = true;
            showCollision = false;
            break;
        case CollisionMeshDisplayMode::COLLISION:
            showPrim = false;
            showCollision = true;
            break;
        }
    }

    bool needsDisplayModeUpdate(CollisionMeshDisplayMode mode) const
    {
        bool showPrim;
        bool showCollision;
        getShowPrimAndCollision(mode, showPrim, showCollision);
        return !mCacheShowIsInited || (mCacheShowPrim != showPrim) || (mCacheShowCollision != showCollision);
    }

    // Set the display mode
    void setDisplayMode(const pxr::SdfPath &primName, // Parent prim
                        CollisionMeshDisplayMode mode)
    {
        bool showPrim;
        bool showCollision;
        getShowPrimAndCollision(mode, showPrim, showCollision);

        pxr::UsdPrim prim = gStage->GetPrimAtPath(primName);
        if ( prim )
        {
            setVisibility(prim,showPrim,true);
        }
        for (auto &i:mCollisionPrims)
        {
            pxr::UsdPrim cprim = gStage->GetPrimAtPath(i.mPrim);
            if ( cprim )
            {
                setVisibility(cprim,showCollision,false);
            }
        }
        // Cache status of showPrim and showCollision to avoid writing back to USD during simulation / fast updates
        mCacheShowIsInited = true;
        mCacheShowPrim = showPrim;
        mCacheShowCollision = showCollision;
    }

    // Each prim with collision APIs has a corresponding unique CRC describing
    // it's collision properties
    omni::physx::usdparser::MeshKey mMeshKey;       // The unique fingerprint for this collision mesh
    pxr::SdfPath                mRootPrim;          // The xform that collision prims are attached to
    std::vector< CollisionPrim > mCollisionPrims;    // The set of debug visualization session layer prims
    float                       mDiagonalAxisLength{0};
    bool mCacheShowIsInited = false;
    bool mCacheShowPrim = false;
    bool mCacheShowCollision = false;
};

using CollisionPrimsMap = std::unordered_map< pxr::SdfPath, CollisionMesh, pxr::SdfPath::Hash>;

// This corresponds to the root prim that has been selected.
// The root prim could (and usually is) a GeomXform with a
// number of child primitives beneath it
class CollisionRoot
{
public:
    // Are there any child prims associated with this CollisionRoot?
    bool isEmpty(void) const
    {
        return mChildren.empty();
    }

    // Propagate the display mode down to all children
    void setDisplayMode(const pxr::SdfPath &primName, CollisionMeshDisplayMode mode)
    {
        for (auto &i:mChildren)
        {
            i.second.setDisplayMode(i.first,mode);
        }
    }

    // Remove all collision prims associated with this CollisionRoot
    void removeCollisionPrims(PathMap &pathMap)
    {
        for (auto &i : mChildren)
        {
            i.second.removeCollisionPrims(pathMap);
        }
        mChildren.clear();
    }

    uint32_t                    mFrameNumber{ 0 };    // used to detect deselected prims
    CollisionPrimsMap mChildren;
};

using CollisionRootMap = std::unordered_map< pxr::SdfPath, CollisionRoot, pxr::SdfPath::Hash>;


        class DebugVisualizationCollisionMeshImpl : public DebugVisualizationCollisionMesh
        {
            omni::physx::IPhysxCooking* mCooking = nullptr;
            carb::settings::ISettings* mSettings = nullptr;
            struct PrecookContext
            {
                IPhysxCookingCallback callback;
                DebugVisualizationCollisionMeshImpl* pself = nullptr;
            };
            std::vector<PrecookContext*> mInFlightContextes; 
            bool mForcedUpdateRequested = false;
            carb::dictionary::SubscriptionId* subId = nullptr;
        public:
            DebugVisualizationCollisionMeshImpl(pxr::UsdGeomXformCache& xformcache) : mXformCache(xformcache)
            {
                gPhysXVisualization = carb::getCachedInterface<omni::physx::IPhysxVisualization>();
                mCooking = carb::getCachedInterface<omni::physx::IPhysxCooking>();
                mSettings = carb::getCachedInterface<carb::settings::ISettings>();
                subId = mSettings->subscribeToNodeChangeEvents(
                    omni::physx::kSettingVisualizationCollisionMesh,
                    [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType changeEventType,
                       void* userData) {
                        auto* self = static_cast<DebugVisualizationCollisionMeshImpl*>(userData);
                        if (changeEventType == carb::dictionary::ChangeEventType::eChanged)
                        {
                            auto* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
                            self->enableCollisionMeshVisualization(dict->getAsBool(changedItem));
                        }
                    },
                    this);
                enableCollisionMeshVisualization(mSettings->getAsBool(omni::physx::kSettingVisualizationCollisionMesh));
            }

            virtual ~DebugVisualizationCollisionMeshImpl(void)
            {
                mSettings->unsubscribeToChangeEvents(subId);
                subId = nullptr;
                //releaseAllCollisionPrims();

                // These are in flight precook request that have not been returning yet
                for(PrecookContext* context: mInFlightContextes)
                {
                    // Let's signal the precook callback lambda that this object doesn't exist anymore.
                    // We cannot delete the context here but it will be done in the precook callback lambda in any case
                    context->pself = nullptr;
                }
                gPhysXVisualization = nullptr;
            }

            virtual void release(void) final
            {
                delete this;
            }

            virtual bool wantsForcedUpdate() const
            {
                return mForcedUpdateRequested;
            }

            virtual void updateDebugVisualization(void) final
            {
                mForcedUpdateRequested = false;

                if ( !mEnabled ) return;
                CARB_PROFILE_ZONE(0, "DebugVisualization::collisionDebugVisUpdate");
                omni::fabric::UsdStageId usdStageId = {};
                if (gStage)
                {
                    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(gStage).ToLongInt();
                    usdStageId.id = stageId;
                }
                for (const pxr::SdfPath& path : mPathCache)
                {
                    const pxr::UsdPrim prim = gStage->GetPrimAtPath(path);
                    if ( prim )
                    {
                        updateCollisionPrimDebugVisualization(usdStageId, prim);
                    }
                }
                if(!mPathCache.empty())
                {
                    // Update transforms for all collision meshes in a single fabric query / loop
                    // This will silently fail if no fabric stage exists
                    CollisionMesh::updateAllCollisionMeshesChildrenWithFabric(usdStageId);
                }
            }

            std::string getFlattenedPrimName(const pxr::UsdPrim &prim)
            {
                const char *scan = prim.GetPath().GetText();
                std::string prefix("/CollisionMeshes/");
                std::string postfix;
                while (*scan)
                {
                    char c = *scan;
                    if (c == '/')
                    {
                        c = '_';
                    }
                    postfix += c;
                    scan++;
                }
                postfix += "_DEBUGVIS";
                std::string ret = prefix + postfix;
                return ret;
            }

            void updateCollisionPrimDebugVisualization(omni::fabric::UsdStageId usdStageId, const pxr::UsdPrim& colPrim)
            {
                const uint64_t stageId = usdStageId.id;

                pxr::UsdPrimRange range(colPrim, pxr::UsdTraverseInstanceProxies());
                // Each selected prim forms the 'root'
                // There can be a large number of children with collision APIs
                // underneath it.
                CollisionRoot croot;
                // See if we are already actively tracking this collision root path or not
                CollisionRootMap::iterator found = mCollisionRoots.find(colPrim.GetPath());
                if ( found != mCollisionRoots.end() )
                {
                    croot = (*found).second;
                }
                bool changed = false;
                // Iterate through all of the children looking for prims with a collision API
                for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
                {
                    pxr::UsdPrim prim = *iter;
                    if (!prim)
                        continue;

                    // We only examine prims with a collision API on them
                    if ( prim.HasAPI<pxr::UsdPhysicsCollisionAPI>() )
                    {
                        // See if this descriptor is already in the descriptor cache
                        usdparser::PhysxShapeDesc* shapeDesc = (usdparser::PhysxShapeDesc*)mDescCache.getDesc(prim.GetPrimPath(), omni::physx::usdparser::eShape);
                        if (!shapeDesc)
                        {
                            CARB_PROFILE_ZONE(0, "DebugVisualization::shapeDescParse");
                            static pxr::TfToken oldConvexPrim("ConvexMesh");

                            if (!(prim.IsA<pxr::UsdGeomGprim>() || prim.GetTypeName() == oldConvexPrim))
                                continue;
                            shapeDesc = gUsdLoad->parseCollision(stageId, prim.GetPrimPath(), prim.GetPrimPath());
                            if (!shapeDesc)
                            {
                                continue;
                            }
                            mDescCache.addDesc(prim.GetPrimPath(), omni::physx::usdparser::eShape, shapeDesc);
                        }

                        if (shapeDesc->collisionEnabled == false)
                        {
                            continue;
                        }
                        // Ok, we have a descriptor and collision is enabled, now get the
                        // unique MeshKey hash (fingerprint) for this prim if it exists
                        // None mesh based prims don't get debug vis
                        omni::physx::usdparser::MeshKey meshKey;
                        // If this is some type of mesh collision, then get the unique meshKey for it
                        if ( gPhysXVisualization->getMeshKey(*shapeDesc,meshKey))
                        {
                            // See if the mesh key is the same as it was before. If it is, then
                            // we only need to synchronize the transform we don't need to generate
                            // the collision proxies as they were already defined
                            bool syncTransform = false;
                            CollisionPrimsMap::iterator found = croot.mChildren.find(prim.GetPath());
                            if ( found != croot.mChildren.end() )
                            {
                                CollisionMesh& collisionMesh = (*found).second;
                                if ( collisionMesh.mMeshKey == meshKey )
                                {
                                    if(collisionMesh.syncRootTransformToFabric(usdStageId, prim))
                                    {
                                        // Fast path, if display mode was not changed, let's skip any USD write and stay Fabric-write only
                                        if(collisionMesh.needsDisplayModeUpdate(mMode))
                                        {
                                            omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                                            collisionMesh.setDisplayMode((*found).first, mMode);
                                        }
                                    }
                                    else
                                    {
                                        // Slow Path, updating with fabric failed let's go with USD
                                        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                                        collisionMesh.syncTransformToUSD(prim);
                                        collisionMesh.setDisplayMode((*found).first, mMode);
                                    }
                                    syncTransform = true; // if the mesh key has not changed, all we have to do is sync the transform
                                }
                            }
                            if ( !syncTransform )
                            {
                                changed = true;
                                // If this collision mesh was already previously defined, then we need
                                // to remove the prims that were associated with it since we are
                                // now going to recreate them.
                                {
                                    CollisionPrimsMap::iterator found = croot.mChildren.find(prim.GetPath());
                                    if ( found != croot.mChildren.end() )
                                    {
                                        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                                        (*found).second.removeCollisionPrims(mCollisionPaths);
                                        croot.mChildren.erase(found);
                                    }
                                }

                                CollisionMesh cmesh;
                                cmesh.mMeshKey = meshKey;
                                const omni::physx::CollisionRepresentation *cr = gPhysXVisualization->getCollisionRepresentation(prim.GetPrimPath(),shapeDesc);
                                if ( cr )
                                {
                                    pxr::SdfPath parentPath = prim.GetPath();
                                    // create the prim here!
                                    {
                                        std::string collisionName = getFlattenedPrimName(prim);
                                        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

                                        {
                                            // If the 'CollisionMeshes' 'scope' does not exist, create it.
                                            // We organize all debug visualization collision meshes underneath a 'scope'
                                            // called 'CollisionMeshes' simply to keep the stage/session view better organized
                                            pxr::SdfPath scopeName("/CollisionMeshes");
                                            pxr::UsdPrim scopePrim = gStage->GetPrimAtPath(scopeName);
                                            if (!scopePrim)
                                            {
                                                pxr::UsdGeomScope::Define(gStage, scopeName);
                                                pxr::UsdPrim prim = gStage->GetPrimAtPath(scopeName);
                                                hideSessionPrim(prim);
                                            }
                                        }

                                        float dx = cr->bmax.x - cr->bmin.x;
                                        float dy = cr->bmax.y - cr->bmin.y;
                                        float dz = cr->bmax.z - cr->bmin.z;
                                        float diagonalAxisLength = (float)sqrt(dx*dx+dy*dy+dz*dz)*0.5f;

                                        cmesh.setRootPrim(prim.GetPath(),
                                                          pxr::SdfPath(collisionName.c_str()),
                                                          diagonalAxisLength,
                                                          mCollisionPaths);

                                        uint32_t colorIndex = 0;
                                        // We compute the hash of the collision primitive name and, from that,
                                        // we compute a starting color for this collision representation.
                                        // In this fashion, the colors of an object do not constantly change
                                        // just because you may be using a different approximation value and/or
                                        // simply because the prim is being rebuilt
                                        {
                                            carb::extras::hash128_t hash;
                                            hash = carb::extras::fnv128hash((const uint8_t *)collisionName.c_str(),collisionName.size());
                                            uint32_t low32 = (uint32_t)hash.d[0]&0xFFFFFFFF;
                                            colorIndex = low32 % uint32_t(DebugColor::NUM_COLORS);
                                        }


                                        // Create the root xform
                                        {
                                            pxr::UsdGeomXform xform = pxr::UsdGeomXform::Define(gStage, cmesh.mRootPrim);
                                            pxr::UsdPrim destPrim = gStage->GetPrimAtPath(cmesh.mRootPrim);
                                            pxr::UsdPrim sourcePrim = prim;
                                            if (sourcePrim && destPrim)
                                            {
                                                hideSessionPrim(destPrim);

                                                pxr::TfToken token = getCollisionMeshToken();
                                                setMetaData(destPrim, token, "collision");

                                                debugmeshprim::DebugMeshPrim *dmp = debugmeshprim::DebugMeshPrim::create();
                                                // Each collision mesh is a child of the root transform
                                                for (uint32_t i=0; i<cr->meshCount; i++)
                                                {
                                                    char scratch[2048];
                                                    snprintf(scratch, sizeof(scratch), "%s/collide%03d", collisionName.c_str(), i+1);
                                                    const omni::physx::CollisionMesh &cm = cr->meshes[i];
                                                    debugmeshprim::DebugMesh mesh;
                                                    mesh.mVertexCount = cm.vertexCount;
                                                    mesh.mVertices = cm.vertices;
                                                    mesh.mTriangleCount = cm.triangleCount;
                                                    mesh.mIndices = cm.indices;

                                                    uint32_t color = getDebugColor((DebugColor)colorIndex);
                                                    colorIndex++;
                                                    if (colorIndex == (uint32_t)DebugColor::NUM_COLORS)
                                                    {
                                                        colorIndex = 0;
                                                    }
                                                    pxr::SdfPath collisionPath(scratch);
                                                    dmp->createDebugMeshPrim(scratch, mesh, color);
                                                    cmesh.addCollisionPrim(prim.GetPath(), collisionPath,cr->rootCenter,cm.meshCenter, mCollisionPaths);
                                                    pxr::UsdPrim meshPrim = gStage->GetPrimAtPath(pxr::SdfPath(scratch));
                                                    hideSessionPrim(meshPrim);
                                                }
                                                bool fabricSucceded = false;
                                                if(cmesh.setupFabricAttributes(usdStageId))
                                                {                                                
                                                    fabricSucceded = cmesh.syncRootTransformToFabric(usdStageId, sourcePrim);
                                                }
                                                
                                                if (!fabricSucceded)
                                                {
                                                    CARB_LOG_WARN("Cannot sync Solid Collision Mesh to Fabric, falling back to USD (slower)");
                                                    cmesh.syncTransformToUSD(prim);
                                                }
                                                dmp->release();
                                            }
                                        }
                                    }
                                    gPhysXVisualization->releaseCollisionRepresentation(cr);

                                    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                                    cmesh.setDisplayMode(prim.GetPath(),mMode);
                                    croot.mChildren[prim.GetPath()] = cmesh;
                                }
                                else
                                {
                                    // If there is no collision mesh, let's ask for precooking the asset and when we
                                    // will receive the callback that it's ready / finished, we will try updating the
                                    // prim again.
                                    scheduleAsyncPrecookFor(asInt(prim.GetPrimPath()), *shapeDesc);
                                }
                            }
                        }
                        else
                        {
                            // There is no mesh key associated with this prim. Make sure there wasn't
                            // one *previously* because if there was we should remove it.
                            CollisionPrimsMap::iterator found = croot.mChildren.find(prim.GetPath());
                            if ( found != croot.mChildren.end() )
                            {
                                omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                                (*found).second.setDisplayMode(prim.GetPath(), CollisionMeshDisplayMode::GRAPHICS);
                                (*found).second.removeCollisionPrims(mCollisionPaths);
                                croot.mChildren.erase(found);
                                changed = true;
                            }
                        }
                    }
                    else
                    {
                        CollisionPrimsMap::iterator found = croot.mChildren.find(prim.GetPath());
                        if (found != croot.mChildren.end())
                        {
                            omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                            (*found).second.setDisplayMode(prim.GetPath(), CollisionMeshDisplayMode::GRAPHICS);
                           (*found).second.removeCollisionPrims(mCollisionPaths);
                           croot.mChildren.erase(found);
                           changed = true;
                        }
                    }
                }
                if ( changed )
                {
                    mCollisionRoots[colPrim.GetPath()] = croot;
                }
            }

            void scheduleAsyncPrecookFor(const uint64_t primId, usdparser::PhysxShapeDesc& shapeDesc)
            {
                const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(gStage).ToLongInt();
                PrecookContext* context = new PrecookContext(); // deleted inside the callback
                context->pself = this;
                context->callback.cookingFinishedCallback = [](uint64_t stageId, uint64_t primPath,
                                                               PhysxCookingResult::Enum result, void* userData) {
                    PrecookContext* context = reinterpret_cast<PrecookContext*>(userData);
                    // if pself == nullptr, then the DebugVisualizationCollisionMeshImpl has been
                    // destroyed (see DebugVisualizationCollisionMeshImpl destructor)
                    if (context->pself)
                    {
                        pxr::UsdStageRefPtr stage =
                            pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(long(stageId)));
                        pxr::UsdPrim prim;
                        if (stage)
                        {
                            // Cannot call updateCollisionPrimDebugVisualization directly here as
                            // this lambda is called from the cooking system that is holding a lock
                            // and we can't call again into the cooking system from here (it would
                            // deadlock) So we just add the prim to the pathCache and it will be
                            // picked up by next update.
                            context->pself->addPathCache(intToPath(primPath));
                            context->pself->mForcedUpdateRequested = true; // signals to call update
                        }
                    }
                    delete context; // deletion must happen in any case
                };
                context->callback.userData = context;
                switch (shapeDesc.type)
                {
                case omni::physx::usdparser::eConvexMeshShape: {
                    auto& desc = *static_cast<omni::physx::usdparser::ConvexMeshPhysxShapeDesc*>(&shapeDesc);
                    mCooking->precookMesh(stageId, primId, desc.convexCookingParams, &context->callback);
                    mInFlightContextes.push_back(context);
                }
                break;
                case omni::physx::usdparser::eConvexMeshDecompositionShape: {
                    auto& desc =
                        *static_cast<omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc*>(&shapeDesc);
                    mCooking->precookMesh(stageId, primId, desc.convexDecompositionCookingParams, &context->callback);
                    mInFlightContextes.push_back(context);
                }
                break;
                case omni::physx::usdparser::eTriangleMeshShape: {
                    auto& desc = *static_cast<omni::physx::usdparser::TriangleMeshPhysxShapeDesc*>(&shapeDesc);
                    if(desc.sdfMeshCookingParams.sdfResolution > 0)
                    {
                        mCooking->precookMesh(stageId, primId, desc.sdfMeshCookingParams, &context->callback);
                    }
                    else
                    {
                        mCooking->precookMesh(stageId, primId, desc.triangleMeshCookingParams, &context->callback);
                    }
                    mInFlightContextes.push_back(context);
                }
                break;
                default: {
                    delete context;
                    context = nullptr;
                }
                break;
                }
            }

            bool addPathCache(const pxr::SdfPath &path)
            {
                bool ret = false;
                PathSet::iterator found = mPathCache.find(path);
                if ( found == mPathCache.end() )
                {
                    mPathCache.insert(path);
                    ret = true;
                }
                return ret;
            }

            bool removePathCache(const pxr::SdfPath &path)
            {
                bool ret = false;

                PathSet::iterator found = mPathCache.find(path);
                if (found != mPathCache.end())
                {
                    mPathCache.erase(found);
                    ret = true;
                }


                return ret;
            }

            virtual bool refreshSelectionSet(void) final
            {
                bool ret = false;

                std::vector<std::string> selection = remapSelection();
                mFrameNumber++; // Increment the logical frame number
                // Iterate through the selected prims
                for (auto &j:selection)
                {
                    pxr::SdfPath prim(j);
                    CollisionRootMap::iterator found = mCollisionRoots.find(prim);
                    if ( found == mCollisionRoots.end() )
                    {
                        ret = true;
                        addPathCache(prim);
                    }
                    else
                    {
                        (*found).second.mFrameNumber = mFrameNumber;
                    }
                }
                // If any collision prims we were previously tracking are now
                // no longer in the selection list, then we remove them.
                // We do this by iterating through all of the collision prims we
                // are currently tracking. If the collision prim was *not* part of
                // the current selection group (we use the 'frame number' to detect this)
                // then those collision primitives are removed and the path removed from
                // the currently active path set
                CollisionRootMap::iterator i = mCollisionRoots.begin();
                while ( i!=mCollisionRoots.end() )
                {
                    if ( (*i).second.mFrameNumber != mFrameNumber )
                    {
                        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                        (*i).second.setDisplayMode((*i).first,CollisionMeshDisplayMode::GRAPHICS);
                        (*i).second.removeCollisionPrims(mCollisionPaths);
                        removePathCache((*i).first);
                        i = mCollisionRoots.erase(i);
                        ret = true;
                    }
                    else
                    {
                        i++;
                    }
                }
                return ret;
            }

            virtual bool releaseDescCache(const pxr::SdfPath& path) final
            {
                bool ret = false;

                pxr::UsdPrim colPrim = gStage->GetPrimAtPath(path);
                if ( colPrim )
                {
                    pxr::UsdPrimRange range(colPrim, pxr::UsdTraverseInstanceProxies());
                    // Iterate through all of the children looking for prims with a collision API
                    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
                    {
                        pxr::UsdPrim prim = *iter;
                        if (!prim)
                            continue;

                        mDescCache.releasePath(prim.GetPrimPath());
                        ret = true;
                    }
                }

                return ret;
            }

            virtual void setCollisionMeshType(const char *type) final
            {
                CollisionMeshDisplayMode current = mMode;
                if ( strcmp(type,"both") == 0 )
                {
                    mMode = CollisionMeshDisplayMode::BOTH;
                }
                else if ( strcmp(type,"collision_only") == 0 )
                {
                    mMode = CollisionMeshDisplayMode::COLLISION;
                }
                else if ( strcmp(type,"graphics_only") == 0 )
                {
                    mMode = CollisionMeshDisplayMode::GRAPHICS;
                }
                else
                {
                    CARB_LOG_WARN("Invalid CollisionMeshType(%s)", type );
                }
                if ( current != mMode && !mCollisionRoots.empty() )
                {
                    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                    for (auto &i:mCollisionRoots)
                    {
                        i.second.setDisplayMode(i.first,mMode);
                    }
                }
            }

            virtual void explodeViewDistance(float distance) final
            {
                gExplodeViewDistance = distance;
            }

            virtual void enableCollisionMeshVisualization(bool enable) final
            {
                if ( enable != mEnabled )
                {
                    mEnabled = enable;
                    if ( !mCollisionRoots.empty() )
                    {
                        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                        for (auto &i : mCollisionRoots)
                        {
                            if ( mEnabled )
                            {
                                i.second.setDisplayMode(i.first,mMode);
                            }
                            else
                            {
                                i.second.setDisplayMode(i.first, CollisionMeshDisplayMode::GRAPHICS);
                            }
                        }
                    }
                }
            }

            void releaseAllCollisionPrims(void)
            {
                if ( !mCollisionRoots.empty() )
                {
                    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
                    // Restore display mode
                    for (auto &i : mCollisionRoots)
                    {
                        i.second.setDisplayMode(i.first,CollisionMeshDisplayMode::GRAPHICS);
                    }
                    for (auto &i:mCollisionRoots)
                    {
                        i.second.removeCollisionPrims(mCollisionPaths);
                    }
                    mCollisionRoots.clear();
                }
            }

            virtual bool isEnabled(void) const final
            {
                return mEnabled;
            }

            virtual bool isXformPathTracked(const pxr::SdfPath &path) const final
            {
                bool ret = false;

                PathSet::const_iterator found = mPathCache.find(path);
                if ( found != mPathCache.end() )
                {
                    ret = true;
                }

                return ret;
            }

            bool addCollisionPrim(const pxr::SdfPath &collisionPath,const pxr::SdfPath &primPath)
            {
                bool ret = false;

                PathMap::iterator found = mCollisionPaths.find(collisionPath);
                if ( found == mCollisionPaths.end() )
                {
                    mCollisionPaths[collisionPath] = primPath;
                    ret = true;
                }

                return ret;
            }

            bool removeCollisionPrim(const pxr::SdfPath &collisionPath)
            {
                bool ret = false;

                PathMap::iterator found = mCollisionPaths.find(collisionPath);
                if ( found != mCollisionPaths.end() )
                {
                    ret = true;
                    mCollisionPaths.erase(found);
                }

                return ret;
            }

            bool collisionToPrim(const pxr::SdfPath &collisionPath,pxr::SdfPath &primPath)
            {
                bool ret = false;

                PathMap::iterator found = mCollisionPaths.find(collisionPath);
                if (found != mCollisionPaths.end())
                {
                    ret = true;
                    primPath = (*found).second;
                }

                return ret;
            }

            std::vector<std::string> remapSelection(void)
            {
                const auto usdContext = omni::usd::UsdContext::getContext();
                const auto selectedPaths = usdContext->getSelection()->getSelectedPrimPaths();
                bool needToRedirect = false;
                PathSet remapped; // names we have already rempapped...
                std::vector<std::string> primPaths;
                for (auto &pathStr : selectedPaths)
                {
                    const pxr::SdfPath path(pathStr);
                    pxr::UsdPrim prim = gStage->GetPrimAtPath(path);
                    if ( isCollisionMesh(prim))
                    {
                        needToRedirect = true;
                        pxr::SdfPath remap;
                        if ( collisionToPrim(path,remap))
                        {
                            PathSet::iterator pfound = remapped.find(remap);
                            if ( pfound == remapped.end() )
                            {
                                primPaths.push_back(remap.GetString());
                                remapped.insert(remap);
                            }
                        }
                    }
                    else
                    {
                        primPaths.push_back(pathStr);
                    }
                }
                if ( needToRedirect )
                {
                    usdContext->getSelection()->setSelectedPrimPaths(primPaths, false);
                }
                return primPaths;
            }

            bool                    mEnabled{false};
            pxr::UsdGeomXformCache& mXformCache;
            PathSet                 mPathCache; // Cache for collider prim paths
            PathMap                 mCollisionPaths; // map collision prims to their parent
            DescCache               mDescCache; // Cache for collider descriptors
            uint32_t                mFrameNumber{0};
            CollisionMeshDisplayMode    mMode{CollisionMeshDisplayMode::BOTH};
            CollisionRootMap        mCollisionRoots;
        };

DebugVisualizationCollisionMesh *DebugVisualizationCollisionMesh::create(pxr::UsdGeomXformCache& xformcache)
{
    auto ret = new DebugVisualizationCollisionMeshImpl(xformcache);
    return static_cast< DebugVisualizationCollisionMesh *>(ret);
}


    } // end of ui namespace
} // end of physx namespace
} // end of omni namespace

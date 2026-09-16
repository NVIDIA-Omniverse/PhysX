// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-2
 *
 * @implements REQ-PARSE-SHAPE-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-6 AC-24
 *
 * @implements REQ-COOK-LIFETIME-001
 * @covers AC-2
 *
 * @implements REQ-COOK-LIFETIME-002
 * @covers AC-1 AC-2
 *
 * @implements REQ-COOK-SOURCE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-MATH-001
 * @covers AC-9
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-5
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-14 AC-16 AC-27 AC-30 AC-44
 *
 * @implements REQ-PUBLICAPI-002
 * @covers AC-10
 */

#include <omni/physics/parse/KnownTokens.h>

#include <carb/logging/Log.h>
#include <carb/tasking/ITasking.h>
#include <carb/profiler/Profile.h>
#include <carb/extras/Timer.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include <common/foundation/CarbPhysXCast.h> // toPhysX/toPhysXd/toFloat3 -- Algorithms.h (below) expects
                                              // these already in scope; nothing pulls them in
                                              // transitively any more.
#include <common/foundation/Algorithms.h>
#include <common/foundation/DeformableCookingTransform.h>

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

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/IChangeFeed.h>
#include <omni/physics/parse/ScanBackend.h>   // parse::scanStage (parseDeformableBody, ADR-0018)
#include <omni/physics/parse/ScannedStage.h>  // parse::ScannedStage (parseDeformableBody, ADR-0018)

#include "usdLoad/IceDescriptorAllocator.h"

#include "particles/PhysXParticleSampling.h"

#include <memory>
#include <regex>
#include <string_view>

// Set this to 1 to enable the USD notice listener to start async tasks
#define USD_USD_NOTICE_LISTENER 1 // Listens for collision related attribute changes to automatically spawn background cooking tasks if needed
#define USE_ASYNC_COOKING 1 // By default we support asynchronous cooking, this is only here for debugging purposes
#define REPORT_PROGRESS 0 // A debug option only, used to track cooking task progress

static constexpr const char* PROGRESS_BAR_ENABLED = "/physics/progressBarEnabled";
static constexpr const char* PROGRESS_BAR_LABEL = "/physics/progressBarLabel";
static constexpr const char* PROGRESS_BAR_VALUE = "/physics/progressBarValue";

using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;
using namespace ::physx;

// Shared PhysX<->Gf bridges (omni.physics.usd/TypeCast.h); this file's
// namespaces sit outside omni::physx.
using omni::physx::toPhysXd;
// carb::Float3 counterpart (common/foundation/CarbPhysXCast.h), overloaded on the
// same names -- toPhysXd(carb::Float3) and toFloat3(PxVec3d) are what the
// read-side helpers below use in place of the retired local toGfVec3f.
using omni::physx::toFloat3;

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
        omni::physics::parse::TokenId deformableBodyToken,
        omni::physics::parse::TokenId rigidBodyToken)
    {
        const omni::physics::parse::ObjectKey root = src.getRootKey();
        while (key.valid() && key != root)
        {
            if (src.hasSchema(key, deformableBodyToken))
                return key;
            if (src.hasSchema(key, rigidBodyToken))
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

    // PhysxCookingComputeRequest's primId/primStageId-adjacent uint64_t correlation fields
    // (IPhysxCookingService.h: "for correlation/logging", not consumed by the cooking service as
    // a real SdfPath) carry the legacy asInt(ObjectKey) encoding (key.handle). Mirrors
    // ContactReport.cpp's keyToLegacyPathInt (same class of pre-existing public/internal wire
    // format, same fix).
    uint64_t keyToLegacyPathInt(const omni::physx::usdparser::AttachedStage* attachedStage,
                                omni::physics::parse::ObjectKey key)
    {
        return attachedStage ? key.handle : 0;
    }

    // Inverse of keyToLegacyPathInt: reconstructs the ObjectKey a completed request's primId
    // named, for the (rare) onFinished continuations that resolve back to an object rather than
    // just logging the id.
    omni::physics::parse::ObjectKey legacyPathIntToKey(const omni::physx::usdparser::AttachedStage& attachedStage,
                                                        uint64_t primId)
    {
        (void)attachedStage;
        return omni::physics::parse::ObjectKey{ primId };
    }

    /**
     * Compute mesh key based on custom mesh arrays
     */
    omni::physx::usdparser::MeshKey computeMeshKey(const std::vector<carb::Float3>& points,
                                                   const std::vector<int32_t>& vertexIndices,
                                                   const std::vector<int32_t>& vertexCounts)
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
    //
    omni::physx::usdparser::MeshKey computeSimMeshHexCrc(const std::vector<carb::Float3>& simPoints,
                                                         const std::vector<carb::Int4>& simIndices,
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

    // Source-agnostic equivalent of pxr's UsdSchemaRegistry::MakeMultipleApplyNameInstance
    // (ChangeRegister.cpp/usdInterface/UsdInterfaceDeformable.cpp carry the identical helper
    // for their own translation units -- small enough, and used differently enough at each
    // call site, that duplicating it locally beats a shared header for this many call sites):
    // substitutes the __INSTANCE_NAME__ placeholder in a multi-apply attribute-name template
    // (e.g. tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints) with the given instance
    // name, and interns the result. Returns an invalid TokenId when `src` is null.
    omni::physics::parse::TokenId makeMultiApplyAttributeToken(const omni::physics::parse::IPhysicsSource* src,
                                                                omni::physics::parse::TokenId nameTemplate,
                                                                omni::physics::parse::TokenId instanceName)
    {
        if (!src)
            return omni::physics::parse::TokenId{};
        static constexpr char kInstanceNamePlaceholder[] = "__INSTANCE_NAME__";
        std::string result(src->tokenToString(nameTemplate));
        const size_t pos = result.find(kInstanceNamePlaceholder);
        if (pos != std::string::npos)
            result.replace(pos, sizeof(kInstanceNamePlaceholder) - 1, std::string(src->tokenToString(instanceName)));
        return src->internToken(result);
    }

    /**
     * Local-to-world transform of the object at `key`, read through the source.
     */
    ::physx::PxMat44d cookingWorldTransform(const omni::physx::usdparser::AttachedStage* attachedStage,
                                            omni::physics::parse::ObjectKey key)
    {
        return attachedStage ?
            omni::physx::internal::getWorldTransform(*attachedStage, key, omni::physics::parse::ReadTime::defaultTime()) :
            ::physx::PxMat44d(::physx::PxIdentity);
    }

    // Local carb flavours of the copyBuffer family that Algorithms.h used to
    // provide before its carb/PhysX retype (PLAN-gf-math-removal bucket B).
    // These three destinations are cooked-geometry *scene-description* arrays,
    // fed to PhysXTools.h's setCookedArrayValue / physxtools_detail::elemTypeOf
    // ladder and to IPhysicsDataWrite::writeArray, both of which now have full
    // carb-typed overloads (2026-08-19). Semantics are unchanged from the prior
    // VtArray-typed overloads -- carb::Float3/Int3/Int4 are memcpy-layout-
    // identical to GfVec3f/GfVec3i/GfVec4i (ADR-0018).
    template <typename SrcVecT>
    void copyBufferCarb(std::vector<carb::Float3>& dst, const SrcVecT* src, unsigned int numElements)
    {
        dst.resize(numElements);
        for (unsigned int i = 0; i < numElements; i++)
        {
            const SrcVecT& srcValue = src[i];
            dst[i] = carb::Float3{ srcValue.x, srcValue.y, srcValue.z };
        }
    }

    template <typename SrcIndexT>
    void copyBufferCarb(std::vector<carb::Int3>& dst, const SrcIndexT* src, unsigned int numSrcElements)
    {
        PX_COMPILE_TIME_ASSERT(3 * sizeof(SrcIndexT) == sizeof(carb::Int3));
        const uint32_t numDstElements = numSrcElements / 3;
        dst.resize(numDstElements);
        std::memcpy(dst.data(), src, numDstElements * sizeof(carb::Int3));
    }

    template <typename SrcIndexT>
    void copyBufferCarb(std::vector<carb::Int4>& dst, const SrcIndexT* src, unsigned int numSrcElements)
    {
        PX_COMPILE_TIME_ASSERT(4 * sizeof(SrcIndexT) == sizeof(carb::Int4));
        const uint32_t numDstElements = numSrcElements / 4;
        dst.resize(numDstElements);
        std::memcpy(dst.data(), src, numDstElements * sizeof(carb::Int4));
    }

    // Source-routed cooking-input geometry read: route an array attribute through
    // AttachedStage's IPhysicsSource, keyed by ObjectKey/TokenId.
    template <typename T>
    bool cookingReadArray(const omni::physx::usdparser::AttachedStage* attachedStage,
                          omni::physics::parse::ObjectKey key,
                          omni::physics::parse::TokenId attrTok,
                          T& out)
    {
        return attachedStage ?
            omni::physx::internal::getArrayValue(
                *attachedStage, key, attrTok, omni::physics::parse::ReadTime::defaultTime(), out) :
            false;
    }

    // Source-routed bind-pose point read for a deformable mesh: when
    // bindPoseToken is set, read the OmniPhysicsDeformablePoseAPI instance's
    // points attribute; otherwise read the plain points attribute. A non-empty
    // bindPoseToken guarantees the pose API instance is applied (the parser only
    // emits it then), so no HasAPI re-check is needed. Returns whether geometry
    // was read.
    bool cookingReadBindPoints(const omni::physx::usdparser::AttachedStage* attachedStage,
                               omni::physics::parse::ObjectKey key,
                               omni::physics::parse::TokenId bindPoseToken,
                               std::vector<carb::Float3>& out)
    {
        const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
        if (!src)
            return false;
        omni::physics::parse::KnownTokens tok;
        tok.intern(*src);
        const omni::physics::parse::TokenId attrTok = bindPoseToken.valid() ?
            makeMultiApplyAttributeToken(src, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints, bindPoseToken) :
            tok.points;
        return cookingReadArray(attachedStage, key, attrTok, out);
    }

    // Source-routed read of a stored MeshKey CRC blob (authored as a UCharArray
    // marker by storeMeshKey). Mirrors usdparser::loadMeshKey but routes through
    // the source by key — no UsdPrim. Returns true and fills `out` only when the
    // marker is present and its byte count matches sizeof(MeshKey); otherwise
    // leaves `out` at its default (all-zero) value, like loadMeshKey. `crcTokenName`
    // is one of the file-local marker names below (not schema-registered, so not in
    // KnownTokens) -- interned fresh against the active source on every call.
    bool cookingLoadMeshKey(const omni::physx::usdparser::AttachedStage* attachedStage,
                            omni::physics::parse::ObjectKey key,
                            const char* crcTokenName,
                            omni::physx::usdparser::MeshKey& out)
    {
        const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
        if (!src)
            return false;
        std::vector<uint8_t> bytes;
        if (!cookingReadArray(attachedStage, key, src->internToken(crcTokenName), bytes))
            return false;
        if (bytes.size() != sizeof(omni::physx::usdparser::MeshKey))
            return false;
        std::memcpy(&out, bytes.data(), sizeof(omni::physx::usdparser::MeshKey));
        return true;
    }

    // Source-routed applied-API / prim-type gates, keyed by ObjectKey. Both take an
    // already-interned KnownTokens TokenId directly (the schema/type name), not
    // a TfToken: `schemaToken`/`typeToken` are pxr-free source vocabulary
    // (IPhysicsSource::hasSchema/isA), so these gates need no UsdSchemaRegistry
    // round-trip -- the KnownTokens field already holds the exact applied-schema
    // or registered-type name string the source expects.
    bool cookingHasSchema(const omni::physx::usdparser::AttachedStage* attachedStage,
                          omni::physics::parse::ObjectKey key,
                          omni::physics::parse::TokenId schemaToken)
    {
        if (const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr)
            return src->hasSchema(key, schemaToken);
        return false;
    }

    bool cookingIsA(const omni::physx::usdparser::AttachedStage* attachedStage,
                    omni::physics::parse::ObjectKey key,
                    omni::physics::parse::TokenId typeToken)
    {
        if (const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr)
            return src->isA(key, typeToken);
        return false;
    }

    // True when the source no longer knows a string for `key` (e.g. removed since the key
    // was captured) -- an extra guard beyond key.valid().
    bool cookingKeyResolves(const omni::physx::usdparser::AttachedStage* attachedStage,
                            omni::physics::parse::ObjectKey key)
    {
        const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
        return src && !src->sourceKeyToString(key).empty();
    }

    void warnTetMeshOrientation(const omni::physx::usdparser::AttachedStage* attachedStage,
                                omni::physics::parse::ObjectKey tetMeshKey,
                                const std::vector<carb::Float3>& points,
                                const std::vector<carb::Int4>& tetVertexIndices,
                                bool expectLeftHanded)
    {
        uint32_t invertedCount = 0;
        for (const carb::Int4& tet : tetVertexIndices)
        {
            if (tet.x < 0 || tet.y < 0 || tet.z < 0 || tet.w < 0 ||
                size_t(tet.x) >= points.size() || size_t(tet.y) >= points.size() ||
                size_t(tet.z) >= points.size() || size_t(tet.w) >= points.size())
            {
                continue;
            }

            const ::physx::PxVec3d a = toPhysXd(points[tet.y]) - toPhysXd(points[tet.x]);
            const ::physx::PxVec3d b = toPhysXd(points[tet.z]) - toPhysXd(points[tet.x]);
            const ::physx::PxVec3d c = toPhysXd(points[tet.w]) - toPhysXd(points[tet.x]);
            const double signedVolume6 = a.dot(b.cross(c));
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
                expectLeftHanded ? "leftHanded" : "rightHanded",
                attachedStage ? attachedStage->textFor(tetMeshKey) : "");
        }
    }

    // Mirrors pxr::UsdGeomXformable::IsTransformationAffectedByAttrNamed, which delegates to
    // UsdGeomXformOp::IsXformOp: true for the `xformOpOrder` attribute itself, or any attribute
    // in the `xformOp:` namespace (verified against the OpenUSD source, usdGeom/xformable.cpp +
    // usdGeom/xformOp.cpp). A pure name predicate, so it is reimplemented here on a plain string
    // rather than linking usdGeom for this one check -- mirrors usdLoad/PrimUpdate.cpp's own
    // identical (currently-unused) reimplementation.
    bool isTransformOpAttributeName(std::string_view name)
    {
        static constexpr std::string_view kXformOpOrder = "xformOpOrder";
        static constexpr std::string_view kXformOpPrefix = "xformOp:";
        return name == kXformOpOrder ||
               (name.size() >= kXformOpPrefix.size() && name.compare(0, kXformOpPrefix.size(), kXformOpPrefix) == 0);
    }

    void switchTetsOrientation(std::vector<carb::Int4>& tetVertexIndices)
    {
        for (size_t i = 0; i < tetVertexIndices.size(); ++i)
        {
            carb::Int4& tet = tetVertexIndices[i];
            std::swap(tet.x, tet.y);
        }
    }

} // namespace

namespace cookingdataasync
{
// These are the collision related attributes that we track in the USD listener.
// If any of these attributes change we *might* have to recook the asset.

static constexpr const char* kParticleSamplingCrcTokenName = "physxParticleSampling:crc";
static constexpr const char* kDeformableBodyDataCrcTokenName = "physxDeformableBody:deformableBodyDataCrc";
static constexpr const char* kSimMeshNumTetsPerElementTokenName = "physxVolumeDeformableSim:numTetsPerElement";
static constexpr const char* kSimMeshHexCrcTokenName = "physxVolumeDeformableSim:simMeshHexCrc";
// All four markers above are ad hoc (non-schema) attribute names: the
// IPhysicsDataWrite methods that author them are keyed by std::string_view.


// ObjectKey-keyed (not SdfPath-keyed): every producer below already has an
// ObjectKey in hand, so the sets hold that directly -- SdfPath is only
// materialized where a downstream call (parseCollision, invalidateMeshKeyCache)
// still requires it, at the point of use.
using PrimRefreshSet = std::unordered_set< omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;

using MeshKeySet = std::unordered_set< omni::physx::usdparser::MeshKey, omni::physx::usdparser::MeshKeyHash >;

// TokenId-keyed (not TfToken-keyed). Unlike TfToken (globally interned, valid
// to compare across any Source), a TokenId is only valid for comparison
// against the Source that minted it (IPhysicsSource::internToken: "cross-Source
// comparison is undefined") -- so this set is rebuilt from scratch whenever the
// active source instance changes, see ensureCollisionTokens().
using TokenSet = std::unordered_set< omni::physics::parse::TokenId, omni::physics::parse::TokenId::Hash >;

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

class CookingDataAsyncImpl;

// pxr-free replacement for TfWeakPtr<CookingDataAsyncImpl>/TfCreateWeakPtr/TfWeakBase:
// async cooking-service completion callbacks capture one of these (by value) instead of
// `this` directly, so a callback that fires after the driver's destructor has already run
// (attach/detach or PhysX-reset teardown mid-flight cook) can detect that and skip touching
// freed memory -- the same liveness contract TfWeakPtr provided, without linking pxr for a
// generic (non-USD) lifetime-safety idiom. `operator bool`/`operator->` mirror TfWeakPtr's
// own interface exactly, so every existing "if (weakPtrToThis) weakPtrToThis->foo()" call
// site needs no change beyond how the value is constructed.
class CookingDataAsyncWeakSelf
{
public:
    CookingDataAsyncWeakSelf(CookingDataAsyncImpl* self, std::weak_ptr<bool> aliveFlag)
        : m_self(self), m_aliveFlag(std::move(aliveFlag))
    {
    }
    explicit operator bool() const
    {
        return m_self && !m_aliveFlag.expired();
    }
    CookingDataAsyncImpl* operator->() const
    {
        return m_self;
    }

private:
    CookingDataAsyncImpl* m_self;
    std::weak_ptr<bool> m_aliveFlag;
};

class CookingDataAsyncImpl : public CookingDataAsync
{
public:
    CookingDataAsyncImpl(physx::PxPhysics& physics, omni::physx::IPhysxCookingServicePrivate& cookingServicePrivate, omni::physx::IPhysxCookingService& cookingService, omni::physx::PhysxCookingAsyncContext context):
        mPhysics(physics), m_cookingServicePrivate(cookingServicePrivate), m_cookingService(cookingService), m_asyncContext(context)
    {
        m_settings = carb::getCachedInterface<carb::settings::ISettings>();
        // Change detection is driven by the attached stage's IChangeFeed (ADR-0003),
        // registered at feed creation by AttachedStage via registerOnChangeFeed().
        // No global USD notice listener.

        // Collision-related attributes that may require a recook when changed:
        // populated lazily by ensureCollisionTokens() the first time a source is
        // available (there is none yet at construction time -- TokenId, unlike the
        // former TfToken, has to be minted BY a source).
    }

    // (Re)builds m_collisionTokens and m_changeTokens from `src`'s vocabulary. A TokenId is only
    // valid for comparison against the Source that minted it, so this rebuilds
    // whenever the active source instance changes (attach/reattach) rather than
    // once at construction, when no source exists yet. Per-instance deformable-
    // pose tokens registered during a source's lifetime (handleSourceChange,
    // below) are intentionally dropped on rebuild: they name prim/instance
    // combinations specific to the previous source and are meaningless -- unsafe
    // to compare, per internToken's own contract -- against a new one.
    //
    // Keyed by attachHandle, not by `&src`: this cache (m_collisionTokens) lives
    // on the singleton CookingDataAsyncImpl, which survives detach, while `src`
    // is destroyed with its AttachedStage. A later attach's allocator can reuse
    // the freed source's address, which would make `m_collisionTokensSource ==
    // &src` a false positive and retain TokenIds minted by the previous source.
    // attachHandle (AttachHandle.h) is process-unique and never reused.
    void ensureCollisionTokens(omni::physx::AttachHandle attachHandle, const omni::physics::parse::IPhysicsSource& src)
    {
        if (m_collisionTokensAttachHandle == attachHandle)
            return;
        m_collisionTokensAttachHandle = attachHandle;
        m_collisionTokens.clear();

        // The one KnownTokens::intern() this driver pays per attach. Retained in
        // m_changeTokens so handleSourceChange's structural branch can read its
        // TokenIds without re-interning the whole vocabulary per change batch.
        omni::physics::parse::KnownTokens& tok = m_changeTokens;
        tok = omni::physics::parse::KnownTokens{};
        tok.intern(src);
        m_collisionTokens.insert(tok.points);
        m_collisionTokens.insert(tok.indices);
        m_collisionTokens.insert(tok.faceVertexCounts);
        m_collisionTokens.insert(tok.faceVertexIndices);
        m_collisionTokens.insert(tok.orientation);
        m_collisionTokens.insert(tok.holeIndices);
        m_collisionTokens.insert(tok.physicsCollisionEnabled);
        m_collisionTokens.insert(tok.physicsApproximation);
        m_collisionTokens.insert(tok.physxConvexHullCollisionHullVertexLimit);
        m_collisionTokens.insert(tok.physxConvexHullCollisionMinThickness);
        m_collisionTokens.insert(tok.physxConvexDecompositionCollisionErrorPercentage);
        m_collisionTokens.insert(tok.physxConvexDecompositionCollisionHullVertexLimit);
        m_collisionTokens.insert(tok.physxConvexDecompositionCollisionMaxConvexHulls);
        m_collisionTokens.insert(tok.physxConvexDecompositionCollisionMinThickness);
        m_collisionTokens.insert(tok.physxConvexDecompositionCollisionVoxelResolution);
        m_collisionTokens.insert(tok.physxConvexDecompositionCollisionShrinkWrap);
        m_collisionTokens.insert(tok.physxTriangleMeshSimplificationCollisionMetric);
        m_collisionTokens.insert(tok.physxSDFMeshCollisionSdfResolution);
        m_collisionTokens.insert(tok.physxSDFMeshCollisionSdfSubgridResolution);
        m_collisionTokens.insert(tok.physxSDFMeshCollisionSdfNarrowBandThickness);
        m_collisionTokens.insert(tok.physxSDFMeshCollisionSdfMargin);
        m_collisionTokens.insert(tok.physxSDFMeshCollisionSdfBitsPerSubgridPixel);
        m_collisionTokens.insert(tok.physxSDFMeshCollisionSdfEnableRemeshing);
        m_collisionTokens.insert(tok.physxSDFMeshCollisionSdfTriangleCountReductionFactor);
        m_collisionTokens.insert(tok.physxSphereFillCollisionFillMode);
        m_collisionTokens.insert(tok.physxSphereFillCollisionMaxSpheres);
        m_collisionTokens.insert(tok.physxSphereFillCollisionSeedCount);
        m_collisionTokens.insert(tok.physxSphereFillCollisionVoxelResolution);
        m_collisionTokens.insert(tok.physxDeformableBodyAutoDeformableBodyEnabled);
        m_collisionTokens.insert(tok.physxDeformableBodyResolution);
        m_collisionTokens.insert(tok.physxDeformableBodyAutoDeformableMeshSimplificationEnabled);
        m_collisionTokens.insert(tok.physxDeformableBodyRemeshingEnabled);
        m_collisionTokens.insert(tok.physxDeformableBodyRemeshingResolution);
        m_collisionTokens.insert(tok.physxDeformableBodyTargetTriangleCount);
        m_collisionTokens.insert(tok.physxDeformableBodyForceConforming);
        // File-local cooking markers (not schema-registered attributes, so not in
        // KnownTokens): interned straight from their plain-string names.
        m_collisionTokens.insert(src.internToken(kDeformableBodyDataCrcTokenName));
        m_collisionTokens.insert(src.internToken(kSimMeshNumTetsPerElementTokenName));
        m_collisionTokens.insert(src.internToken(kSimMeshHexCrcTokenName));
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
            omni::physics::parse::KnownTokens changeTok;
            if (changeSrc)
                changeTok.intern(*changeSrc);
            const omni::physics::parse::TokenId collToken =
                changeSrc ? changeTok.physicsCollisionAPI : omni::physics::parse::TokenId{};
            const omni::physics::parse::TokenId dbToken =
                changeSrc ? changeTok.omniphysicsDeformableBodyAPI : omni::physics::parse::TokenId{};
            const omni::physics::parse::TokenId dpToken =
                changeSrc ? changeTok.OmniPhysicsDeformablePoseAPI : omni::physics::parse::TokenId{};
            const omni::physics::parse::TokenId rigidBodyToken =
                changeSrc ? changeTok.physicsRigidBodyAPI : omni::physics::parse::TokenId{};

            if (changeSrc)
            {
                // For each api-schema change, find the nearest descendant (incl.
                // self) carrying a cooking-relevant schema and schedule a recook.
                for (const auto& rootKey : m_primApiSchemasChangeRefreshSet)
                {
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
                                addPrimRefreshSet(k, *changeAttached);
                                found = true;
                                return true; // prune this match's descendants
                            }
                            return false;
                        });
                }

                // For added/removed prims, schedule the owning deformable body. The
                // set already holds the still-existing PARENT key (resolved by
                // handleSourceChange, at a point where it was guaranteed valid) --
                // not the added/removed child's own key, which may no longer
                // resolve by the time pump() drains this set.
                for (const auto& parentKey : m_primAddedRemovedRefreshSet)
                {
                    if (!parentKey.valid())
                        continue;
                    const omni::physics::parse::ObjectKey bodyKey =
                        findDeformableBodyAncestorKey(*changeSrc, parentKey, dbToken, rigidBodyToken);
                    if (bodyKey.valid())
                        addPrimRefreshSet(bodyKey, *changeAttached);
                }
            }

            m_primApiSchemasChangeRefreshSet.clear();
            m_primAddedRemovedRefreshSet.clear();

            // Check the primitives which *may* have had collision property changes to see if they
            // actually require a recook.
            //
            // Re-derives a fresh PhysxShapeDesc from a live USD change notification (collision
            // branch via parseCollision(), deformable branch via
            // cookDeformableBodyInternalAsync()->parseDeformableBody()). Known gap without USD:
            // parseDeformableBody re-keys through omni::physics::usd::ScannedStage, so editing a
            // collision/pose property on an already-loaded deformable body does not auto-recook
            // there; the caller must re-request cooking explicitly.
#if USE_ASYNC_COOKING
            // Time-box this loop; local disk cache hits make it fast, but we still bail after 16ms.
            timer.start();
            bool endedPrematurely = false;
            for(auto it = m_primRefreshSet.begin(); it != m_primRefreshSet.end(); )
            {
                CARB_PROFILE_ZONE(0, "CookingDataAsync::pump primRefreshSet");
                const omni::physics::parse::ObjectKey key = *it;
                if (changeSrc && key.valid() && cookingKeyResolves(changeAttached, key))
                {
                    const bool hasCollisionAPI = changeSrc->hasSchema(key, collToken);
                    const bool hasDeformablePoseAPI = changeSrc->hasSchema(key, dpToken);
                    const bool hasDeformableBodyAPI = changeSrc->hasSchema(key, dbToken);
                    omni::physics::parse::ObjectKey deformableBodyKey;
                    if (hasDeformableBodyAPI)
                        deformableBodyKey = key;
                    else if (hasCollisionAPI || hasDeformablePoseAPI)
                        deformableBodyKey = findDeformableBodyAncestorKey(*changeSrc, key, dbToken, rigidBodyToken);

                    if (hasCollisionAPI && !deformableBodyKey.valid())
                    {
                        // Parse the fully digested PhysXShapeDesc for this prim; if it is a cookable
                        // shape, spawn a background cooking task. parseCollision keys off the stage
                        // id; the cook dispatch resolves `key` back to a path at the service boundary.
                        omni::physx::usdparser::PhysxShapeDesc* shapeDesc =
                            omni::physx::usdparser::parseCollision(*changeAttached, key, key);
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
                m_primXformRefreshSet.erase(key);
                if (timer.getElapsedTime<int64_t>(carb::extras::Timer::Scale::eMilliseconds) >= 16) // never take more than 16 ms on the main thread
                {
                    endedPrematurely = true;
                    break;
                }
            }
            if(!endedPrematurely)
            {
                for (auto &key : m_primXformRefreshSet)
                {
                    //TODO we also need to search children to catch transforms
                    //that have an impact to deformable cooking, which might be expensive
                    //without maintaining a SdfPathTable
                    if (!changeSrc || !key.valid())
                        continue;

                    omni::physics::parse::ObjectKey deformableBodyKey;
                    if (changeSrc->hasSchema(key, dbToken))
                        deformableBodyKey = key;
                    else if (changeSrc->hasSchema(key, collToken) || changeSrc->hasSchema(key, dpToken))
                        deformableBodyKey = findDeformableBodyAncestorKey(*changeSrc, key, dbToken, rigidBodyToken);

                    if (deformableBodyKey.valid())
                    {
                        cookDeformableBodyInternalAsync(deformableBodyKey, *changeAttached, true);
                    }
                }

                m_primRefreshSet.clear();
                m_primXformRefreshSet.clear();
            }
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

    // The cache-hit twin of reportLegacyCookingFinishedCallbackAndResult: no request was ever
    // submitted, so the attach has to be named by the caller. PhysxCookingFinishedCallback reports
    // an AttachHandle, not a stage id (ADR-0016 Decision 6) -- a stage id cannot name a stageless
    // attach, and the callback's whole purpose is to tell the consumer which attach finished.
    static void reportCookingFinished(const omni::physx::AttachHandle attachHandle,
                                      const uint64_t primId,
                                      omni::physx::IPhysxCookingCallback* cb)
    {
        if (cb && cb->cookingFinishedCallback)
        {
            cb->cookingFinishedCallback(attachHandle, primId, omni::physx::PhysxCookingResult::eVALID, cb->userData);
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
            // The request's attachHandle, not its primStageId: the two fields answer different
            // questions (ADR-0016 Decision 6) and this one is "which attach issued this cook".
            cb->cookingFinishedCallback(result.request->attachHandle, result.request->primId, result.result, cb->userData);
        }
    }

    void fillRequestPrimMeshView(omni::physx::PhysxCookingComputeRequest& request,
                                 const omni::physx::usdparser::AttachedStage& attachedStage,
                                 const omni::physx::usdparser::MergeMeshDesc& mergeMeshDesc)
    {
        // Every request is mesh-view mode now (eINPUT_MODE_FROM_PRIM_ID removed); the struct
        // default for metersPerUnit is 1.0 unless it is set here.
        // Every cooking tolerance derived from PxTolerancesScale would otherwise be wrong by
        // 1/metersPerUnit on a stage that is not authored in metres (100x on a centimetre
        // stage). Read from the source so it holds with or without a backing UsdStage.
        request.primMeshMetersPerUnit = double(attachedStage.getSourceUnits().metersPerUnit);

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
        // Two identities, two jobs (ADR-0016 Decision 6). primStageId/primId are correlation keys
        // only now (REQ-COOK-SOURCE-001 AC-1) -- the geometry always comes from IPhysicsSource, no
        // USD re-read fallback. attachHandle is the attach this cook belongs to, and is what the
        // completion side resolves and reports. The handle is nonzero even for a stageless attach,
        // where the stage id is 0.
        request.primStageId = attachedStage.getStageId();
        request.attachHandle = attachedStage.getAttachHandle();
        request.primId = keyToLegacyPathInt(&attachedStage, primKey);

        // If we haven't loaded the mesh from the in memory mesh cache, we go further to see if it is in the local cache or UsdPrim itself
        // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope, since the cooking service copies the view synchronously before any
        // async task is queued (CookingTask::setupTaskFromRequest resets request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
        if (ret == nullptr)
        {
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, attachedStage, *desc.mergedMesh);
            }
            else if (!omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey))
            {
                // REQ-COOK-SOURCE-001 AC-4: unreadable input fails loudly, before any submission --
                // no more falling back to the cooking service resolving primStageId/primId itself.
                CARB_LOG_ERROR("Convex mesh cooking: could not read source geometry for prim %s", attachedStage.textFor(primKey));
                return ret;
            }

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
            request.meshKey = desc.meshKey;

            PxPhysics* pxPhysics = &mPhysics;
            PxConvexMesh*& returnedMesh = ret;
            CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);

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
                    CARB_LOG_WARN("Failed to create triangle mesh from cooked data! Prim(%llu)\n",
                                  static_cast<unsigned long long>(result.request->primId));
                }
            };
            getComputeService().requestConvexMeshCookedData(m_asyncContext, request, desc.convexCookingParams);
            return ret;
        }
        else
        {
            reportCookingFinished(request.attachHandle, request.primId, cb);
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
        // primStageId names the USD stage to read from; attachHandle names the attach to resolve and
        // report on completion (ADR-0016 Decision 6).
        request.primStageId = attachedStage.getStageId();
        request.attachHandle = attachedStage.getAttachHandle();
        request.primId = keyToLegacyPathInt(&attachedStage, primKey);
        const std::string_view primMeshTextView = attachedStage.textViewFor(primKey);
        request.primMeshText = { primMeshTextView.data(), primMeshTextView.size() };

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
                fillRequestPrimMeshView(request, attachedStage, *desc.mergedMesh);
            }
            else if (!omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey))
            {
                // REQ-COOK-SOURCE-001 AC-4: unreadable input fails loudly, before any submission --
                // no more falling back to the cooking service resolving primStageId/primId itself.
                // Covers triangle + SDF (SDF cooks through this triangle path with sdf params on the desc).
                CARB_LOG_ERROR("Triangle mesh cooking: could not read source geometry for prim %s", attachedStage.textFor(primKey));
                return ret;
            }

            PxPhysics* pxPhysics = &mPhysics;
            const bool originalTriangles =
                desc.triangleMeshCookingParams.mode == omni::physx::TriangleMeshMode::eORIGINAL_TRIANGLES;
            request.triangulation.needsTriangleFaceMap = originalTriangles;
            request.triangulation.needsMaxMaterialIndex = originalTriangles && maxMaterialIndex != nullptr;
            PxTriangleMesh*& returnedMesh = ret;
            CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);

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
                    CARB_LOG_WARN("Failed to create triangle mesh from cooked data! Prim(%llu)\n",
                                  static_cast<unsigned long long>(result.request->primId));
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
            reportCookingFinished(request.attachHandle, request.primId, cb);
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
        // The stage id is the service's mesh-data input; the handle is the attach identity the
        // finished callback reports (ADR-0016 Decision 6).
        const long stageId = attachedStage.getStageId();
        const omni::physx::AttachHandle attachHandle = attachedStage.getAttachHandle();
        if ( it != sphereFillMap.end())
        {
            ret = it->second;
            reportCookingFinished(attachHandle, keyToLegacyPathInt(&attachedStage, primKey), cb);
        }
        else
        {

            omni::physx::PhysxCookingComputeRequest request;

            request.primStageId = stageId;
            request.attachHandle = attachHandle;
            request.primId = keyToLegacyPathInt(&attachedStage, primKey);
            request.meshKey = desc.meshKey;

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

            // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope, since the cooking service copies the view synchronously before any
        // async task is queued (CookingTask::setupTaskFromRequest resets request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, attachedStage, *desc.mergedMesh);
            }
            else if (!omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey))
            {
                // REQ-COOK-SOURCE-001 AC-4: unreadable input fails loudly, before any submission --
                // no more falling back to the cooking service resolving primStageId/primId itself.
                CARB_LOG_ERROR("Sphere fill cooking: could not read source geometry for prim %s", attachedStage.textFor(primKey));
                return ret;
            }

            PxPhysics* pxPhysics = &mPhysics;
            const omni::physx::usdparser::SpherePointsPhysxShapeDesc*& returnedMesh = ret;
            CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);

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
                    CARB_LOG_WARN("Failed to create sphere fill from cooked data! Prim(%llu)\n",
                                  static_cast<unsigned long long>(result.request->primId));
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
        // The stage id is the service's mesh-data input; the handle is the attach identity the
        // finished callback reports (ADR-0016 Decision 6).
        const long stageId = attachedStage.getStageId();
        const omni::physx::AttachHandle attachHandle = attachedStage.getAttachHandle();
        if (it != convexDecompositionMap.end())
        {
            for (auto &i : it->second)
            {
                ret.push_back(i);
            }
            reportCookingFinished(attachHandle, keyToLegacyPathInt(&attachedStage, primKey), cb);
        }
        else
        {
            omni::physx::PhysxCookingComputeRequest request;

            request.primStageId = stageId;
            request.attachHandle = attachHandle;
            request.primId = keyToLegacyPathInt(&attachedStage, primKey);
            request.meshKey = desc.meshKey;

            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

            // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope, since the cooking service copies the view synchronously before any
        // async task is queued (CookingTask::setupTaskFromRequest resets request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
            if (desc.mergedMesh)
            {
                fillRequestPrimMeshView(request, attachedStage, *desc.mergedMesh);
            }
            else if (!omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey))
            {
                // REQ-COOK-SOURCE-001 AC-4: unreadable input fails loudly, before any submission --
                // no more falling back to the cooking service resolving primStageId/primId itself.
                CARB_LOG_ERROR("Convex decomposition cooking: could not read source geometry for prim %s", attachedStage.textFor(primKey));
                return ret;
            }

            PxPhysics* pxPhysics = &mPhysics;
            CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);

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
                    CARB_LOG_WARN(
                        "Failed to create convex decomposition mesh from cooked data! Prim(%llu)\n",
                        static_cast<unsigned long long>(result.request->primId));
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
    * Only called from pump()'s USD-notify reclassification loop; see that loop's comment
    * for the deformable-recook gap without USD.
    */
    void cookDeformableBodyInternalAsync(omni::physics::parse::ObjectKey bodyKey,
                                         const omni::physx::usdparser::AttachedStage& attachedStage,
                                         bool xformOnly)
    {
        if (!bodyKey.valid())
            return;

        if (attachedStage.textViewFor(bodyKey).empty())
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
        const std::string bodyPathText(attachedStage.textViewFor(bodyKey));
        if (bodyPathText.empty())
            return nullptr;

        // Subtree re-parse keyed by the body's path. eAll mirrors the native
        // eAllPrims refresh path while still routing through the active scan backend.
        const std::vector<std::string> scanRoots{ bodyPathText };
        static const std::vector<std::string> kNoExclude;
        omni::physics::parse::ScanOptions scanOptions;
        scanOptions.descendantScope = omni::physics::parse::DescendantScope::eAll;
        omni::physics::parse::ScannedStage scanned = omni::physics::parse::scanStage(
            attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
            omni::physx::usdparser::iceDescriptorAllocator());
        if (scanned.deformables.empty())
            return nullptr;

        return omni::physx::usdparser::convert::convertScannedDeformableBody(scanned, 0, attachedStage.getSourceUnits(), attachedStage);
    }

    virtual bool cookVolumeDeformableBody(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool asynchronous) final
    {
        if (!desc.hasAutoAPI)
            return false;

        // Block USD notification handlers while in this call
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        if (!bodyKey.valid())
            return false;
        // Extra guard beyond bodyKey.valid(): a valid key can still fail to resolve to a live
        // object (e.g. removed from the source since bodyKey was captured).
        if (!cookingKeyResolves(&attachedStage, bodyKey))
            return false;

        bool cookedDataAvailable = false;
        cookVolumeDeformableBodyInternal(desc, bodyKey, attachedStage, false, asynchronous, &cookedDataAvailable);
        return cookedDataAvailable;
    }

    virtual bool cookSurfaceDeformableBody(const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool asynchronous) final
    {
        if (!desc.hasAutoAPI)
            return false;

        // Block USD notification handlers while in this call
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        if (!bodyKey.valid())
            return false;
        // Extra guard beyond bodyKey.valid(): a valid key can still fail to resolve to a live
        // object (e.g. removed from the source since bodyKey was captured).
        if (!cookingKeyResolves(&attachedStage, bodyKey))
            return false;

        bool cookedDataAvailable = false;
        cookSurfaceDeformableBodyInternal(desc, bodyKey, attachedStage, false, asynchronous, &cookedDataAvailable);
        return cookedDataAvailable;
    }

    /**
    * Utility method which return true if this key refers to an object corresponding to an active physics object of a given type.
    *
    * @param key : The ObjectKey of the primitive in question
    *
    * @return : Returns true if this key is already associated with an active physics object
    */
    bool checkParsed(uint64_t stageId, omni::physics::parse::ObjectKey key, omni::physx::usdparser::ObjectType objectType)
    {
        omni::physx::usdparser::AttachedStage* attachedStage = omni::physx::usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (attachedStage)
        {
            omni::physx::usdparser::ObjectDb* db = attachedStage->getObjectDatabase();
            if (db)
            {
                return db->findEntry(key, objectType) != omni::physx::usdparser::kInvalidObjectId;
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
    *
    * @return : Returns true if the cooked arrays were published. False means nothing was written
    * and no deformable can be built from this cook - the caller must report that upwards rather
    * than treating the cook as done.
    */
    bool storeVolumeDeformableBodyDataToUsd(
        omni::physx::PhysxCookingVolumeDeformableBodyData& data,
        const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey,
        omni::physx::usdparser::AttachedStage& attachedStage,
        const omni::physx::usdparser::MeshKey& tetMeshCrc)
    {
        // Record always, publish when possible (ADR-0022).
        //
        // Cooked geometry is runtime SCRATCH: the runtime derives it from authored input and must
        // not depend on reading it back out of the scene description. It does read it back -- the
        // PhysX-mesh cook, object creation, the attachments and the tensor views all re-read these
        // arrays later and independently -- so every array below is recorded into the attached
        // stage's cooked-geometry carrier, which those re-reads consult through
        // internal::getArrayValue when there is no write sink.
        //
        // Publishing to the scene description stays a best-effort SIDE EFFECT. It is not
        // guaranteed just because the source exists: a parse backend may vend a live
        // IPhysicsSource and a null write sink, which is exactly what OvstageParseBackend does by
        // design. A missing sink is therefore no longer a failure -- it only means the cooked mesh
        // does not land in a layer. A missing SOURCE still is: without one there is no token
        // vocabulary to record against and nothing downstream can resolve.
        //
        // This runs main-thread during the cooking pump (inside ScopedBlockUSDUpdates), so the
        // keyFor intern-table writes and the carrier writes need no synchronization.
        omni::physics::parse::IPhysicsDataWrite* dataWrite = attachedStage.getDataWrite();
        omni::physics::parse::IPhysicsSource* dataSource = attachedStage.getSource();
        if (!dataSource)
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: no parse source for %s, the cooked sim and collision "
                           "meshes cannot be recorded and no deformable volume will be created.",
                           attachedStage.textFor(bodyKey));
            return false;
        }

        // RAII begin/end so every early return below still closes the write batch.
        struct WriteScope
        {
            omni::physics::parse::IPhysicsDataWrite* dw;
            explicit WriteScope(omni::physics::parse::IPhysicsDataWrite* d) : dw(d) { if (dw) dw->beginWrite(); }
            ~WriteScope() { if (dw) dw->endWrite(); }
        } writeScope(dataWrite);

        // KnownTokens field lookup for the schema/attribute-name tokens below
        // (dataSource is confirmed non-null above).
        omni::physics::parse::KnownTokens tok;
        tok.intern(*dataSource);

        auto setArray = [&](omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId attrTok, const auto& arr)
        {
            omni::physx::internal::setCookedArrayValue(attachedStage, primKey, attrTok, arr);
            if (!dataWrite)
                return;
            omni::physics::parse::DataWriteView v;
            v.data = arr.empty() ? nullptr : omni::physx::internal::physxtools_detail::arrayElemData(arr);
            v.count = arr.size();
            v.stride = 0;
            v.device = -1;
            v.type = omni::physics::parse::DataType::e32Bit;
            dataWrite->writeArray(primKey, attrTok, v);
        };

        if (!cookingHasSchema(&attachedStage, bodyKey, tok.omniphysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdPhysicsDeformableBodyAPI applied to %s", attachedStage.textFor(bodyKey));
            return false;
        }

        //write sim mesh UsdGeomTetMesh
        //we just write the sim points, which are actually bind pose points
        //this will reset the simulation state, if there is one
        // isTetMeshLike, not cookingIsA(..., tok.tetMeshType): ovstage reports a UsdGeomTetMesh as
        // plain "Mesh", so the concrete-type gate is unconditionally false there and rejects this
        // write-back for every volume deformable loaded from a non-USD source -- on any ovstage
        // attach, with or without a backing stage, which is why no stageless A/B could see it.
        // Same remedy already applied to the cooking-params gates below. See
        // PhysXTools.h::isTetMeshLike.
        if (!omni::physx::internal::isTetMeshLike(attachedStage, desc.simMeshKey))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdGeomTetMesh sim mesh defined at %s", attachedStage.textFor(desc.simMeshKey));
            return false;
        }

        //write rest shape UsdPhysicsVolumeDeformableSimAPI
        //for now we always just write the sim mesh 1:1
        if (!cookingHasSchema(&attachedStage, desc.simMeshKey, tok.OmniPhysicsVolumeDeformableSimAPI))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdPhysicsVolumeDeformableSimAPI applied to %s", attachedStage.textFor(desc.simMeshKey));
            return false;
        }

        if (!desc.collisionMeshKey.valid())
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: Expected prim with UsdPhysicsCollisionAPI: %s",
                attachedStage.textFor(bodyKey));
            return false;
        }

        // isTetMeshLike on the collision mesh must be checked before the clear below: once the
        // clear drops the carrier subtree, a collision mesh whose tets only ever existed in the
        // carrier (auto-generated, never authored as a UsdGeomTetMesh) would trip this gate on
        // every recook, since isTetMeshLike falls back to reading the very data the clear just
        // erased. See PhysXTools.h::isTetMeshLike / getArrayValue.
        if (desc.collisionMeshKey != desc.simMeshKey && !omni::physx::internal::isTetMeshLike(attachedStage, desc.collisionMeshKey))
        {
            CARB_LOG_ERROR("storeVolumeDeformableBodyDataToUsd: No UsdGeomTetMesh collision mesh defined at %s",
                attachedStage.textFor(desc.collisionMeshKey));
            return false;
        }

        // A re-cook must not leave anything behind from the previous one: the body's whole subtree
        // (sim mesh, collision mesh and skins all sit under it) is dropped from the carrier before
        // the first record below. Mirrors clearGeneratedDeformableAttachmentDataUnderPath, and is
        // placed after every reject gate above (all pure predicates over desc/schema state) so a
        // rejected store leaves the previous cook's scratch intact rather than half-erasing it.
        // (ADR-0022)
        attachedStage.clearCookedGeometryUnderPath(bodyKey);

        {
            const omni::physics::parse::ObjectKey simMeshKey = desc.simMeshKey;

            std::vector<carb::Float3> points;
            copyBufferCarb(points, data.simPoints, data.simPointsSize);
            setArray(simMeshKey, tok.points, points);

            std::vector<carb::Float3> velocities;
            setArray(simMeshKey, tok.velocities, velocities);

            std::vector<carb::Int4> indices;
            copyBufferCarb(indices, data.simIndices, data.simIndicesSize);
            setArray(simMeshKey, tok.tetVertexIndices, indices);

            // Hex sim mesh: stamp format + signature so a future cook can recover numTetsPerElement
            // from the data, even if the auto API is later removed without changes to the tetmesh.
            //
            // These two cannot travel through the neutral writeArray - DataWriteView::type is
            // {e16Bit, e32Bit}, with no 8-bit form, and numTetsPerElement is a scalar, not an
            // array - so they go out-of-band via the carrier and IPhysicsDataWrite's ad hoc
            // attribute methods (ADR-0022 addendum).
            const omni::physics::parse::TokenId simMeshNumTetsPerElementTok =
                dataSource->internToken(kSimMeshNumTetsPerElementTokenName);
            const omni::physics::parse::TokenId simMeshHexCrcTok = dataSource->internToken(kSimMeshHexCrcTokenName);
            if (data.numTetsPerElement == 5 || data.numTetsPerElement == 6)
            {
                const omni::physx::usdparser::MeshKey hexCrc =
                    computeSimMeshHexCrc(points, indices, data.numTetsPerElement);

                omni::physx::internal::setCookedUIntValue(attachedStage, simMeshKey, simMeshNumTetsPerElementTok,
                                                          data.numTetsPerElement);
                omni::physx::internal::setCookedBlobValue(attachedStage, simMeshKey, simMeshHexCrcTok, &hexCrc,
                                                          sizeof(hexCrc));

                if (dataWrite)
                {
                    dataWrite->writeUIntAttribute(simMeshKey, kSimMeshNumTetsPerElementTokenName, data.numTetsPerElement);
                    dataWrite->writeByteArrayAttribute(simMeshKey, kSimMeshHexCrcTokenName,
                                                       reinterpret_cast<const uint8_t*>(&hexCrc), sizeof(hexCrc));
                }
            }
            else
            {
                omni::physx::internal::clearCookedValue(attachedStage, simMeshKey, simMeshNumTetsPerElementTok);
                omni::physx::internal::clearCookedValue(attachedStage, simMeshKey, simMeshHexCrcTok);

                if (dataWrite)
                {
                    dataWrite->removeAttribute(simMeshKey, kSimMeshNumTetsPerElementTokenName);
                    dataWrite->removeAttribute(simMeshKey, kSimMeshHexCrcTokenName);
                }
            }
        }

        {
            std::vector<carb::Float3> points;
            copyBufferCarb(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshKey, tok.omniphysicsRestShapePoints, points);

            std::vector<carb::Int4> indices;
            copyBufferCarb(indices, data.simIndices, data.simIndicesSize);
            setArray(desc.simMeshKey, tok.omniphysicsRestTetVtxIndices, indices);
        }

        //write bind poses
        if (desc.simMeshBindPoseToken.valid())
        {
            const omni::physics::parse::TokenId poseAttrTok = makeMultiApplyAttributeToken(
                dataSource, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints, desc.simMeshBindPoseToken);
            std::vector<carb::Float3> points;
            copyBufferCarb(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshKey, poseAttrTok, points);
        }

        if (desc.collisionMeshKey != desc.simMeshKey)
        {
            //write collision mesh UsdGeomTetMesh
            //we just write the collision points, which are actually bind pose points
            //this will reset the simulation state, if there is one
            {
                std::vector<carb::Float3> points;
                copyBufferCarb(points, data.collPoints, data.collPointsSize);
                setArray(desc.collisionMeshKey, tok.points, points);

                std::vector<carb::Int4> indices;
                copyBufferCarb(indices, data.collIndices, data.collIndicesSize);
                setArray(desc.collisionMeshKey, tok.tetVertexIndices, indices);
            }

            if (desc.collisionMeshBindPoseToken.valid())
            {
                const omni::physics::parse::TokenId poseAttrTok = makeMultiApplyAttributeToken(
                    dataSource, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints, desc.collisionMeshBindPoseToken);
                std::vector<carb::Float3> points;
                copyBufferCarb(points, data.collPoints, data.collPointsSize);
                setArray(desc.collisionMeshKey, poseAttrTok, points);
            }
        }

        //writing collision mesh surface indices in any case (even if sim/coll mesh alias)
        {
            std::vector<carb::Int3> indices;
            copyBufferCarb(indices, data.collSurfaceIndices, data.collSurfaceIndicesSize);
            setArray(desc.collisionMeshKey, tok.surfaceFaceVertexIndices, indices);
        }

        // reset the skin points to their bind pose
        for (size_t s = 0; s < desc.skinGeomPaths.size(); ++s)
        {
            const omni::physics::parse::ObjectKey skinGeomKey = desc.skinGeomPaths[s];
            if (cookingIsA(&attachedStage, skinGeomKey, tok.pointBasedType))
            {
                std::vector<carb::Float3> bindPoints;
                if (cookingReadBindPoints(&attachedStage, skinGeomKey, desc.skinGeomBindPoseTokens[s], bindPoints))
                {
                    setArray(skinGeomKey, tok.points, bindPoints);
                }
            }
        }

        //write crc
        const omni::physics::parse::TokenId deformableBodyDataCrcTok =
            dataSource->internToken(kDeformableBodyDataCrcTokenName);
        omni::physx::internal::setCookedBlobValue(attachedStage, bodyKey, deformableBodyDataCrcTok, &tetMeshCrc,
                                                  sizeof(tetMeshCrc));
        // Ad hoc marker outside IPhysicsSource's token vocabulary; authored through
        // writeByteArrayAttribute, as for the numTetsPerElement/hexCrc pair above.
        if (dataWrite)
        {
            dataWrite->writeByteArrayAttribute(bodyKey, kDeformableBodyDataCrcTokenName,
                                               reinterpret_cast<const uint8_t*>(&tetMeshCrc), sizeof(tetMeshCrc));
        }
        return true;
    }

    /***
    * A method to store the cooked surface deformable data to the corresponding USD primitive attributes
    *
    * @param data : Description of data that needs to be stored to USD
    * @param desc : Parsed surface deformable body
    * @param bodyPrim : The USD prim root for storing the cooked data
    * @param deformableBodyDataCrc : The unique 128 bit hash key for this mesh configuration
    *
    * @return : Returns true if the cooked arrays were published (or were already up to date).
    * @see storeVolumeDeformableBodyDataToUsd
    */
    bool storeSurfaceDeformableBodyDataToUsd(
        omni::physx::PhysxCookingSurfaceDeformableBodyData& data,
        const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey,
        omni::physx::usdparser::AttachedStage& attachedStage,
        const omni::physx::usdparser::MeshKey& deformableBodyDataCrc)
    {
        // Record always, publish when possible — see storeVolumeDeformableBodyDataToUsd for why a
        // missing sink is a dropped side effect rather than a failure, and a missing source is not.
        // Runs main-thread during the cooking pump, so keyFor intern-table and carrier writes are
        // safe. (ADR-0022)
        omni::physics::parse::IPhysicsDataWrite* dataWrite = attachedStage.getDataWrite();
        omni::physics::parse::IPhysicsSource* dataSource = attachedStage.getSource();
        if (!dataSource)
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: no parse source for %s, the cooked sim mesh cannot "
                           "be recorded and no deformable surface will be created.",
                           attachedStage.textFor(bodyKey));
            return false;
        }

        struct WriteScope
        {
            omni::physics::parse::IPhysicsDataWrite* dw;
            explicit WriteScope(omni::physics::parse::IPhysicsDataWrite* d) : dw(d) { if (dw) dw->beginWrite(); }
            ~WriteScope() { if (dw) dw->endWrite(); }
        } writeScope(dataWrite);

        // KnownTokens field lookup for the schema/attribute-name tokens below
        // (dataSource is confirmed non-null above).
        omni::physics::parse::KnownTokens tok;
        tok.intern(*dataSource);

        auto setArray = [&](omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId attrTok, const auto& arr)
        {
            omni::physx::internal::setCookedArrayValue(attachedStage, primKey, attrTok, arr);
            if (!dataWrite)
                return;
            omni::physics::parse::DataWriteView v;
            v.data = arr.empty() ? nullptr : omni::physx::internal::physxtools_detail::arrayElemData(arr);
            v.count = arr.size();
            v.stride = 0;
            v.device = -1;
            v.type = omni::physics::parse::DataType::e32Bit;
            dataWrite->writeArray(primKey, attrTok, v);
        };

        if (!cookingHasSchema(&attachedStage, bodyKey, tok.omniphysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsDeformableBodyAPI applied to %s", attachedStage.textFor(bodyKey));
            return false;
        }

        //first read crc and abort if still valid, the reason is we are consuming data from the cooking job that doesn't
        //go into USD, so we need the cooking task to return successfully to return the cached data even though
        //we don't need to update to USD.
        omni::physx::usdparser::MeshKey storedDeformableBodyDataCrc;
        cookingLoadMeshKey(&attachedStage, bodyKey, kDeformableBodyDataCrcTokenName, storedDeformableBodyDataCrc);
        if (storedDeformableBodyDataCrc == deformableBodyDataCrc)
        {
            // Already published and still valid - nothing to write, but the data is there.
            return true;
        }

        //write sim mesh UsdGeomMesh
        //we just write the sim points, which are actually bind pose points
        //this will reset the simulation state, if there is one
        if (!cookingIsA(&attachedStage, desc.simMeshKey, tok.meshType))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdGeomMesh sim mesh defined at %s", attachedStage.textFor(desc.simMeshKey));
            return false;
        }

        //write rest shape UsdPhysicsSurfaceDeformableSimAPI
        //for now we always just write the sim mesh 1:1
        if (!cookingHasSchema(&attachedStage, desc.simMeshKey, tok.OmniPhysicsSurfaceDeformableSimAPI))
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No UsdPhysicsSurfaceDeformableSimAPI applied to %s", attachedStage.textFor(desc.simMeshKey));
            return false;
        }

        if (!desc.collisionMeshKey.valid())
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: Expected prim with UsdPhysicsCollisionAPI: %s",
                attachedStage.textFor(bodyKey));
            return false;
        }

        if (desc.collisionMeshKey != desc.simMeshKey)
        {
            CARB_LOG_ERROR("storeSurfaceDeformableBodyDataToUsd: No separate collision mesh supported for surface deformables: %s",
                attachedStage.textFor(desc.collisionMeshKey));
            return false;
        }

        // Drop the previous cook's scratch for this body before recording the new one; placed
        // after the CRC early-return above and every reject gate (all pure predicates over
        // desc/schema state) so a still-valid or rejected cook keeps what it published
        // (ADR-0022).
        attachedStage.clearCookedGeometryUnderPath(bodyKey);

        {
            std::vector<carb::Float3> points;
            copyBufferCarb(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshKey, tok.points, points);

            std::vector<carb::Float3> velocities;
            setArray(desc.simMeshKey, tok.velocities, velocities);

            size_t numFaces = data.simIndicesSize / 3;
            CARB_ASSERT(numFaces * 3 == data.simIndicesSize);
            std::vector<int32_t> faceVertexCounts(numFaces, 3);
            std::vector<int32_t> faceVertexIndices(numFaces*3);
            std::memcpy(faceVertexIndices.data(), data.simIndices, sizeof(int32_t)*faceVertexIndices.size());

            setArray(desc.simMeshKey, tok.faceVertexCounts, faceVertexCounts);
            setArray(desc.simMeshKey, tok.faceVertexIndices, faceVertexIndices);
        }

        {
            std::vector<carb::Float3> points;
            copyBufferCarb(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshKey, tok.omniphysicsRestShapePoints, points);

            size_t numFaces = data.simIndicesSize / 3;
            std::vector<carb::Int3> triIndices(numFaces);
            std::memcpy(triIndices.data(), data.simIndices, sizeof(carb::Int3) * triIndices.size());
            setArray(desc.simMeshKey, tok.omniphysicsRestTriVtxIndices, triIndices);
        }

        //write bind poses
        if (desc.simMeshBindPoseToken.valid())
        {
            const omni::physics::parse::TokenId poseAttrTok = makeMultiApplyAttributeToken(
                dataSource, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints, desc.simMeshBindPoseToken);
            std::vector<carb::Float3> points;
            copyBufferCarb(points, data.simPoints, data.simPointsSize);
            setArray(desc.simMeshKey, poseAttrTok, points);
        }

        // reset the skin points to their bind pose
        for (size_t s = 0; s < desc.skinGeomPaths.size(); ++s)
        {
            const omni::physics::parse::ObjectKey skinGeomKey = desc.skinGeomPaths[s];
            if (cookingIsA(&attachedStage, skinGeomKey, tok.pointBasedType) && desc.skinGeomBindPoseTokens[s].valid())
            {
                std::vector<carb::Float3> bindPoints;
                cookingReadBindPoints(&attachedStage, skinGeomKey, desc.skinGeomBindPoseTokens[s], bindPoints);
                setArray(skinGeomKey, tok.points, bindPoints);
            }
        }

        //write crc
        const omni::physics::parse::TokenId deformableBodyDataCrcTok =
            dataSource->internToken(kDeformableBodyDataCrcTokenName);
        omni::physx::internal::setCookedBlobValue(attachedStage, bodyKey, deformableBodyDataCrcTok,
                                                  &deformableBodyDataCrc, sizeof(deformableBodyDataCrc));
        if (dataWrite)
        {
            dataWrite->writeByteArrayAttribute(bodyKey, kDeformableBodyDataCrcTokenName,
                                               reinterpret_cast<const uint8_t*>(&deformableBodyDataCrc), sizeof(deformableBodyDataCrc));
        }
        return true;
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
        omni::physics::parse::ObjectKey samplerKey,
        const omni::physx::usdparser::AttachedStage& attachedStage,
        const omni::physx::usdparser::ParticleSamplingDesc& desc,
        ::physx::PxMat44d& rigidTransform,
        omni::physx::ParticlePoissonSamplingCookingParams& params)
    {
        ::physx::PxMat33d shearScaleTransform(::physx::PxIdentity);
        if (!omni::physx::particles::PhysxParticleFactory::getDecomposedTransform(
                samplerKey, desc.particleSetKey, rigidTransform, shearScaleTransform))
        {
            return false;
        }

        // CACHE-KEY INVARIANCE: PxMat33d is an element copy of the GfMatrix3d this
        // used to be, so these nine doubles are byte-identical to what
        // GfMatrix3d::data() produced. CookingHashing.h hashes them verbatim and
        // ParticlePoissonSamplingCookingTask.cpp reinterpret_casts them back to a
        // GfMatrix3d -- both keep working unchanged.
        static_assert(sizeof(params.shearScale) == sizeof(shearScaleTransform));
        memcpy(params.shearScale, &shearScaleTransform.column0.x, sizeof(params.shearScale));

        params.samplingDistance = desc.samplingDistance;
        params.sampleVolume = desc.sampleVolume;
        params.maxSamples = desc.maxSamples;

        return true;
    }

    bool setupVolumeDeformableBodyCookingParams(const omni::physx::usdparser::AttachedStage* attachedStage,
                                                omni::physics::parse::ObjectKey bodyKey,
                                                const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                                omni::physx::VolumeDeformableBodyCookingParams& params,
                                                omni::physics::parse::ObjectKey& srcMeshKey,
                                                std::vector<carb::Float3>& pxrSrcPointsInSim,
                                                omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        // KnownTokens field lookup for the schema/attribute-name tokens below;
        // left un-interned (all fields invalid) when there is no source -- every
        // cookingHasSchema/cookingIsA call below already null-checks attachedStage/
        // source internally and returns false in that case, regardless of the
        // TokenId value passed.
        omni::physics::parse::KnownTokens tok;
        const omni::physics::parse::IPhysicsSource* tokSrc = attachedStage ? attachedStage->getSource() : nullptr;
        if (tokSrc)
            tok.intern(*tokSrc);

        if (!bodyKey.valid() || !cookingHasSchema(attachedStage, bodyKey, tok.omniphysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("PhysX could not find source prim or prim has no UsdPhysicsDeformableBodyAPI!");
            return false;
        }

        const ::physx::PxMat44d simToWorld = cookingWorldTransform(attachedStage, desc.simMeshKey);
        const ::physx::PxMat44d worldToSim = omni::physx::affineInverse(simToWorld);

        ::physx::PxMat44d simToColl(::physx::PxIdentity);
        if (desc.collisionMeshKey != desc.simMeshKey)
        {
            const ::physx::PxMat44d collToWorld = cookingWorldTransform(attachedStage, desc.collisionMeshKey);
            const ::physx::PxMat44d worldToColl = omni::physx::affineInverse(collToWorld);
            // Gf `simToWorld * worldToColl` -- operands swap under the PhysX convention.
            simToColl = worldToColl * simToWorld;
        }

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.",
                attachedStage ? attachedStage->textFor(bodyKey) : "");
            return false;
        }

        srcMeshKey = desc.cookingSrcMeshKey;
        if (!cookingIsA(attachedStage, srcMeshKey, tok.meshType))
        {
            CARB_LOG_WARN(
                "Cooking failed, deformable has no valid source mesh for cooking: %s.",
                attachedStage ? attachedStage->textFor(bodyKey) : "");
            return false;
        }

        std::vector<int32_t> pxrSrcVertexIndices;
        std::vector<int32_t> pxrSrcVertexCounts;

        {
            cookingReadBindPoints(attachedStage, srcMeshKey, desc.cookingSrcMeshBindPoseToken, pxrSrcPointsInSim);
            cookingReadArray(attachedStage, srcMeshKey, tok.faceVertexIndices, pxrSrcVertexIndices);
            cookingReadArray(attachedStage, srcMeshKey, tok.faceVertexCounts, pxrSrcVertexCounts);

            // Transform skin points to sim mesh space
            const ::physx::PxMat44d srcToWorld = cookingWorldTransform(attachedStage, srcMeshKey);
            // Gf `srcToWorld * worldToSim` -- operands swap under the PhysX convention.
            const ::physx::PxMat44d srcToSim = worldToSim * srcToWorld;
            for (size_t i = 0; i < pxrSrcPointsInSim.size(); ++i)
            {
                carb::Float3& srcPoint = pxrSrcPointsInSim[i];
                srcPoint = toFloat3(srcToSim.transform(toPhysXd(srcPoint)));
            }
        }

        // Reject if data is missing
        if (pxrSrcPointsInSim.empty() || pxrSrcVertexIndices.empty() || pxrSrcVertexCounts.empty())
        {
            return false;
        }

        ::physx::PxMat44d simToCookingTransform;
        if (!omni::physx::computeDeformableCookingTransform(
                &simToCookingTransform, nullptr, nullptr, simToWorld,
                pxrSrcPointsInSim.data(), pxrSrcPointsInSim.size()))
        {
            return false;
        }

        customMeshCrc = computeMeshKey(pxrSrcPointsInSim, pxrSrcVertexIndices, pxrSrcVertexCounts);

        params.srcPointsInSim = { pxrSrcPointsInSim.data(), pxrSrcPointsInSim.size() };

        static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
        memcpy(params.simToCookingTransform, simToCookingTransform.front(), sizeof(params.simToCookingTransform));

        static_assert(sizeof(params.simToCollTransform) == sizeof(simToColl));
        memcpy(params.simToCollTransform, simToColl.front(), sizeof(params.simToCollTransform));

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
    * @param outCookedDataAvailable : Optional. Set to true if the cooked data is available in the
    * scene description once this returns, false if the cook could not run or could not publish. This
    * is deliberately NOT the return value: the return value answers "should a PxDeformableVolumeMesh
    * be cooked now", which is false both when nothing needs doing and when everything failed.
    * @return : Returns true if usd data is ready and PxDeformableVolumeMesh should be cooked
    */
    bool cookVolumeDeformableBodyInternal(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          omni::physics::parse::ObjectKey bodyKey,
                                          const omni::physx::usdparser::AttachedStage& attachedStage,
                                          bool xFormOnly,
                                          bool asynchronous,
                                          bool* outCookedDataAvailable = nullptr)
    {
        auto reportCookedDataAvailable = [outCookedDataAvailable](bool available)
        {
            if (outCookedDataAvailable)
                *outCookedDataAvailable = available;
        };
        reportCookedDataAvailable(false);

#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        const uint64_t primStageId = uint64_t(attachedStage.getStageId());
        const omni::physx::AttachHandle attachHandle = attachedStage.getAttachHandle();
        if (!bodyKey.valid())
            return false;
        // Extra guard beyond bodyKey.valid(): a valid key can still fail to resolve to a live
        // object (e.g. removed from the source since bodyKey was captured).
        if (!cookingKeyResolves(&attachedStage, bodyKey))
            return false;

        const bool isSimulated = checkParsed(primStageId, bodyKey, omni::physx::usdparser::eVolumeDeformableBody);
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
            // so we don't issue a warning here. It is a re-entrancy skip and not a failure either:
            // the body could only be registered for simulation because an earlier cook already
            // published its data, so the cooked data IS available even though this call did nothing.
            reportCookedDataAvailable(true);
            return false;
        }

        omni::physx::VolumeDeformableBodyCookingParams params;
        omni::physics::parse::ObjectKey srcMeshKey;
        std::vector<carb::Float3> pxrSrcPointsInSim;
        omni::physx::usdparser::MeshKey customMeshCrc;
        if (!setupVolumeDeformableBodyCookingParams(&attachedStage, bodyKey, desc, params, srcMeshKey, pxrSrcPointsInSim, customMeshCrc))
        {
            CARB_LOG_ERROR("Volume deformable body, failed to setup cooking params, prim: %s", attachedStage.textFor(bodyKey));
            return false;
        }

        // Cooking input from the parse source, not from a stage lookup.
        //
        // Both requests below used to run in the default eINPUT_MODE_FROM_PRIM_ID, which makes the
        // cooking service resolve `primStageId` through UsdUtilsStageCache and re-read the cooking
        // source (skin) mesh itself -- its triangles for the triangulation, and its bind-pose
        // points transformed into sim space (fillUSDVolumeDeformableBodyMeshView). That round-trip
        // cannot succeed on an attach with no backing USD stage (stageless ovstage:
        // getStageId() == 0), and while the service does log "PhysX could not find USD stage", the
        // failure arrives here as SUCCESS: the CRC probe's onFinished still fires, leaving
        // cookedDataCRC default, an unauthored originalCrc is default too, so the "cached data is
        // still valid" comparison holds and this returns true -- telling the caller to go on and
        // cook a volume mesh from a sim mesh that was never cooked.
        //
        // Everything the service would re-read is already in hand: the skin triangles come from
        // IPhysicsSource through the same helper every collision cook uses, and srcPointsInSim was
        // read through the source and transformed into sim space by
        // setupVolumeDeformableBodyCookingParams above. Feeding both to the request removes the
        // stage lookup and the duplicate read at once. `srcScope` owns the skin buffers the view
        // points at and must outlive both submissions.
        //
        // One deliberate consequence: the ujitso request key drops PRIM_MESH_TEXT (only
        // getStageAndPrim sets primMeshText) and now carries metersPerUnit from the source rather
        // than from UsdGeomGetStageMetersPerUnit, so a warm cache misses once and re-cooks. This is
        // the same trade the already-merged collision port made.
        omni::physx::usdparser::SourceMeshGeometryScope srcScope;
        omni::physx::PhysxCookingComputeRequest srcInput;
        if (!omni::physx::usdparser::fillCookingMeshViewFromSource(srcInput, srcScope, attachedStage, srcMeshKey) ||
            params.srcPointsInSim.empty())
        {
            CARB_LOG_ERROR("Volume deformable body, cooking source mesh %s is unreadable through the parse "
                           "source (skin points %zu, skin indices %zu, sim-space points %zu), prim: %s",
                           attachedStage.textFor(srcMeshKey), srcInput.primMeshView.points.size(),
                           srcInput.primMeshView.indices.size(), params.srcPointsInSim.size(), attachedStage.textFor(bodyKey));
            return false;
        }

        // Apply the source-read input to a request. The deformable-body view is the second half:
        // the service fills it from USD on the prim-id path (fillUSDVolumeDeformableBodyMeshView),
        // and it is what the ujitso input container actually serializes -- leaving it empty here
        // would cook successfully from nothing.
        auto applySourceInput = [&](omni::physx::PhysxCookingComputeRequest& request)
        {
            request.primMeshView = srcInput.primMeshView;
            request.primMeshMetersPerUnit = srcInput.primMeshMetersPerUnit;
            request.volumeDeformableBodyView.srcPointsInSim = params.srcPointsInSim;
        };

        omni::physx::usdparser::MeshKey originalCrc;
        cookingLoadMeshKey(&attachedStage, bodyKey, kDeformableBodyDataCrcTokenName, originalCrc);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.attachHandle = attachHandle;
            request.primId = keyToLegacyPathInt(&attachedStage, srcMeshKey);
            applySourceInput(request);
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
                // Cached data is still valid - nothing to re-publish, but it is there.
                reportCookedDataAvailable(true);
                return true;
            }
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
        request.meshKey = customMeshCrc;
        request.primStageId = primStageId;
        // onFinished below resolves the attach from this handle, not from primStageId
        // (ADR-0016 Decision 6): a stageless attach has primStageId 0, which names nothing.
        request.attachHandle = attachHandle;
        request.primId = keyToLegacyPathInt(&attachedStage, srcMeshKey);
        applySourceInput(request);
        request.deformablePathInfo.bodyPrimId = keyToLegacyPathInt(&attachedStage, bodyKey);
        request.deformablePathInfo.simMeshPrimId = keyToLegacyPathInt(&attachedStage, desc.simMeshKey);
        request.deformablePathInfo.collMeshPrimId = keyToLegacyPathInt(&attachedStage, desc.collisionMeshKey);

        uint64_t bodyPrimId = keyToLegacyPathInt(&attachedStage, bodyKey);
        bool shouldRecookPhysxMeshTooSynchronously = false;
        // Whether the write-back below actually published anything. Captured by reference like
        // shouldRecookPhysxMeshTooSynchronously: only meaningful for a synchronous submission,
        // where onFinished runs inline before this function returns.
        bool published = false;
        CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);
        recordStatisticsRequestFor(request);
        request.onFinished = [weakPtrToThis, &shouldRecookPhysxMeshTooSynchronously, &published, desc,
                              bodyPrimId](const omni::physx::PhysxCookingComputeResult& result) {
            if (!weakPtrToThis)
            {
                return;
            }
            weakPtrToThis->recordStatisticsResultFor(result);
            omni::physx::IPhysxCookingServicePrivate& cookingService = weakPtrToThis->m_cookingServicePrivate;
            // Resolve the attach by handle, not by primStageId (ADR-0016 Decision 6). The stage-id
            // lookup only ever worked on a stageless attach because getAttachedStage(0) falls back
            // to "the lone attach" -- which silently picks the wrong one under multi-attach and
            // cannot tell a detach/reattach from the original. resolveAttach() returns null for a
            // stale handle and for kNoAttach, and the null branch below already handles that.
            omni::physx::usdparser::AttachedStage* as =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->resolveAttach(result.request->attachHandle);
            if (as)
            {
                const omni::physics::parse::ObjectKey bodyKey = legacyPathIntToKey(*as, bodyPrimId);
                omni::physics::parse::KnownTokens tok;
                const omni::physics::parse::IPhysicsSource* asSrc = as->getSource();
                if (asSrc)
                    tok.intern(*asSrc);
                if (cookingHasSchema(as, bodyKey, tok.omniphysicsDeformableBodyAPI))
                {
                    if (result.result == omni::physx::PhysxCookingResult::eVALID)
                    {
                        if (result.cookedData)
                        {
                            omni::physx::PhysxCookingVolumeDeformableBodyData data;
                            cookingService.readVolumeDeformableBodyData(data, *result.cookedData);
                            const bool storedThisCall = weakPtrToThis->storeVolumeDeformableBodyDataToUsd(
                                data, desc, bodyKey, *as, result.cookedDataCRC);
                            // Only touch the reference-captured local for a synchronous result: for a
                            // genuinely deferred callback this function has already returned and its
                            // stack frame (and `published`) no longer exists.
                            if (result.isSynchronousResult)
                            {
                                published = storedThisCall;
                            }
                        }
                    }
                    else
                    {
                        // The cook failed: invalidate the stored CRC so the next attempt does not
                        // read it back as "cached data is still valid". Recorded in the
                        // cooked-geometry carrier as well as the USD sink, because on a sink-less
                        // backend the carrier is where the CRC lives (ADR-0022).
                        omni::physx::usdparser::MeshKey crc_zero;
                        if (asSrc)
                        {
                            omni::physx::internal::setCookedBlobValue(
                                *as, bodyKey, asSrc->internToken(kDeformableBodyDataCrcTokenName), &crc_zero,
                                sizeof(crc_zero));
                        }
                        if (auto* dw = as->getDataWrite())
                        {
                            dw->writeByteArrayAttribute(bodyKey, kDeformableBodyDataCrcTokenName,
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
        // An asynchronous submission cannot be judged here - its write-back has not run yet - so it
        // counts as accepted. A synchronous one is judged on what the write-back actually published.
        reportCookedDataAvailable(asynchronous || published);
        return shouldRecookPhysxMeshTooSynchronously;
    }

    bool setupSurfaceDeformableBodyCookingParams(const omni::physx::usdparser::AttachedStage* attachedStage,
                                                 omni::physics::parse::ObjectKey bodyKey,
                                                 const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
                                                 omni::physx::SurfaceDeformableBodyCookingParams& params,
                                                 omni::physics::parse::ObjectKey& srcMeshKey,
                                                 std::vector<carb::Float3>& pxrSrcPointsInSim,
                                                 omni::physx::usdparser::MeshKey& customMeshCrc)
    {
        // See setupVolumeDeformableBodyCookingParams: left un-interned (all fields
        // invalid) when there is no source -- every cookingHasSchema/cookingIsA
        // call below already null-checks and returns false in that case.
        omni::physics::parse::KnownTokens tok;
        if (const omni::physics::parse::IPhysicsSource* tokSrc = attachedStage ? attachedStage->getSource() : nullptr)
            tok.intern(*tokSrc);

        if (!bodyKey.valid() || !cookingHasSchema(attachedStage, bodyKey, tok.omniphysicsDeformableBodyAPI))
        {
            CARB_LOG_ERROR("PhysX could not find source prim or prim has no UsdPhysicsDeformableBodyAPI!");
            return false;
        }

        const ::physx::PxMat44d simToWorld = cookingWorldTransform(attachedStage, desc.simMeshKey);
        const ::physx::PxMat44d worldToSim = omni::physx::affineInverse(simToWorld);

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.",
                attachedStage ? attachedStage->textFor(bodyKey) : "");
            return false;
        }

        srcMeshKey = desc.cookingSrcMeshKey;
        if (!cookingIsA(attachedStage, srcMeshKey, tok.meshType))
        {
            CARB_LOG_WARN(
                "Cooking failed, deformable has no valid source mesh for cooking: %s.",
                attachedStage ? attachedStage->textFor(bodyKey) : "");
            return false;
        }

        std::vector<int32_t> pxrSrcVertexIndices;
        std::vector<int32_t> pxrSrcVertexCounts;

        {
            cookingReadBindPoints(attachedStage, srcMeshKey, desc.cookingSrcMeshBindPoseToken, pxrSrcPointsInSim);
            cookingReadArray(attachedStage, srcMeshKey, tok.faceVertexIndices, pxrSrcVertexIndices);
            cookingReadArray(attachedStage, srcMeshKey, tok.faceVertexCounts, pxrSrcVertexCounts);

            // Transform src points to sim mesh space
            const ::physx::PxMat44d srcToWorld = cookingWorldTransform(attachedStage, srcMeshKey);
            // Gf `srcToWorld * worldToSim` -- operands swap under the PhysX convention.
            const ::physx::PxMat44d srcToSim = worldToSim * srcToWorld;
            for (size_t i = 0; i < pxrSrcPointsInSim.size(); ++i)
            {
                carb::Float3& srcPoint = pxrSrcPointsInSim[i];
                srcPoint = toFloat3(srcToSim.transform(toPhysXd(srcPoint)));
            }
        }

        // Reject if data is missing
        if (pxrSrcPointsInSim.empty() || pxrSrcVertexIndices.empty() || pxrSrcVertexCounts.empty())
        {
            return false;
        }

        ::physx::PxMat44d simToCookingTransform;
        if (!omni::physx::computeDeformableCookingTransform(
                &simToCookingTransform, nullptr, nullptr, simToWorld,
                pxrSrcPointsInSim.data(), pxrSrcPointsInSim.size()))
        {
            return false;
        }

        params.srcPointsInSim = { pxrSrcPointsInSim.data(), pxrSrcPointsInSim.size() };
        static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
        memcpy(params.simToCookingTransform, simToCookingTransform.front(), sizeof(params.simToCookingTransform));
        params.isAutoMeshSimplificationEnabled = desc.isAutoMeshSimplificationEnabled;
        params.isAutoRemeshingEnabled = desc.isAutoRemeshingEnabled;
        params.autoRemeshingResolution = desc.autoRemeshingResolution;
        params.autoTriangleTargetCount = desc.autoTriangleTargetCount;

        customMeshCrc = computeMeshKey(pxrSrcPointsInSim, pxrSrcVertexIndices, pxrSrcVertexCounts);
        return true;
    }

    /**
    * @param outCookedDataAvailable : Optional, @see cookVolumeDeformableBodyInternal.
    */
    void cookSurfaceDeformableBodyInternal(const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
        omni::physics::parse::ObjectKey bodyKey, const omni::physx::usdparser::AttachedStage& attachedStage, bool xFormOnly,
        bool asynchronous, bool* outCookedDataAvailable = nullptr)
    {
        auto reportCookedDataAvailable = [outCookedDataAvailable](bool available)
        {
            if (outCookedDataAvailable)
                *outCookedDataAvailable = available;
        };
        reportCookedDataAvailable(false);

#if !USE_ASYNC_COOKING
        asynchronous = false;
#endif
        const uint64_t primStageId = uint64_t(attachedStage.getStageId());
        const omni::physx::AttachHandle attachHandle = attachedStage.getAttachHandle();
        if (!bodyKey.valid())
            return;
        // Extra guard beyond bodyKey.valid(): a valid key can still fail to resolve to a live
        // object (e.g. removed from the source since bodyKey was captured).
        if (!cookingKeyResolves(&attachedStage, bodyKey))
            return;

        const bool isSimulated = checkParsed(primStageId, bodyKey, omni::physx::usdparser::eSurfaceDeformableBody);
        if (isSimulated)
        {
            // refer to cookVolumeDeformableBodyInternal why we don't warn here, and why this
            // re-entrancy skip counts as the cooked data being available
            reportCookedDataAvailable(true);
            return;
        }

        omni::physx::SurfaceDeformableBodyCookingParams params;
        omni::physics::parse::ObjectKey srcMeshKey;
        std::vector<carb::Float3> pxrSrcPointsInSim;
        omni::physx::usdparser::MeshKey customMeshCrc;
        if (!setupSurfaceDeformableBodyCookingParams(&attachedStage, bodyKey, desc, params, srcMeshKey, pxrSrcPointsInSim, customMeshCrc))
        {
            CARB_LOG_ERROR(
                "Surface deformable body, failed to setup cooking params, prim: %s", attachedStage.textFor(bodyKey));
            return;
        }

        // Cooking input from the parse source, not from a stage lookup -- see the long note in
        // cookVolumeDeformableBodyInternal for why the prim-id path cannot work without a backing
        // USD stage and why its failure is silent rather than loud. `srcScope` owns the skin
        // buffers the view points at and must outlive both submissions.
        omni::physx::usdparser::SourceMeshGeometryScope srcScope;
        omni::physx::PhysxCookingComputeRequest srcInput;
        if (!omni::physx::usdparser::fillCookingMeshViewFromSource(srcInput, srcScope, attachedStage, srcMeshKey) ||
            params.srcPointsInSim.empty())
        {
            CARB_LOG_ERROR("Surface deformable body, cooking source mesh %s is unreadable through the parse "
                           "source (skin points %zu, skin indices %zu, sim-space points %zu), prim: %s",
                           attachedStage.textFor(srcMeshKey), srcInput.primMeshView.points.size(),
                           srcInput.primMeshView.indices.size(), params.srcPointsInSim.size(), attachedStage.textFor(bodyKey));
            return;
        }

        // The surface deformable body view is what the ujitso input container serializes; the
        // service fills it from USD on the prim-id path, so it has to be supplied here.
        auto applySourceInput = [&](omni::physx::PhysxCookingComputeRequest& request)
        {
            request.primMeshView = srcInput.primMeshView;
            request.primMeshMetersPerUnit = srcInput.primMeshMetersPerUnit;
            request.surfaceDeformableBodyView.srcPointsInSim = params.srcPointsInSim;
        };

        omni::physx::usdparser::MeshKey originalCrc;
        cookingLoadMeshKey(&attachedStage, bodyKey, kDeformableBodyDataCrcTokenName, originalCrc);
        {
            // compute data CRC synchronously and exit if the corresponding USD value matches.
            omni::physx::PhysxCookingComputeRequest request;
            request.primStageId = primStageId;
            request.attachHandle = attachHandle;
            request.primId = keyToLegacyPathInt(&attachedStage, srcMeshKey);
            applySourceInput(request);
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
                // Cached data is still valid - nothing to re-publish, but it is there.
                reportCookedDataAvailable(true);
                return;
            }
        }
        omni::physx::PhysxCookingComputeRequest request;
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
        request.meshKey = customMeshCrc;
        request.primStageId = primStageId;
        // onFinished below resolves the attach from this handle, not from primStageId
        // (ADR-0016 Decision 6): a stageless attach has primStageId 0, which names nothing.
        request.attachHandle = attachHandle;
        request.primId = keyToLegacyPathInt(&attachedStage, srcMeshKey);
        applySourceInput(request);
        request.deformablePathInfo.bodyPrimId = keyToLegacyPathInt(&attachedStage, bodyKey);
        request.deformablePathInfo.simMeshPrimId = keyToLegacyPathInt(&attachedStage, desc.simMeshKey);
        request.deformablePathInfo.collMeshPrimId = keyToLegacyPathInt(&attachedStage, desc.collisionMeshKey);

        uint64_t bodyPrimId = keyToLegacyPathInt(&attachedStage, bodyKey);
        // Whether the write-back below actually published anything; only meaningful for a
        // synchronous submission, where onFinished runs inline before this function returns.
        bool published = false;
        CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);
        recordStatisticsRequestFor(request);
        request.onFinished = [weakPtrToThis, &published, desc, bodyPrimId](const omni::physx::PhysxCookingComputeResult& result) {
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
            // Resolve the attach by handle, not by primStageId (ADR-0016 Decision 6). The stage-id
            // lookup only ever worked on a stageless attach because getAttachedStage(0) falls back
            // to "the lone attach" -- which silently picks the wrong one under multi-attach and
            // cannot tell a detach/reattach from the original. resolveAttach() returns null for a
            // stale handle and for kNoAttach, and the null branch below already handles that.
            omni::physx::usdparser::AttachedStage* as =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->resolveAttach(result.request->attachHandle);
            if (as)
            {
                const omni::physics::parse::ObjectKey bodyKey = legacyPathIntToKey(*as, bodyPrimId);
                omni::physics::parse::KnownTokens tok;
                const omni::physics::parse::IPhysicsSource* asSrc = as->getSource();
                if (asSrc)
                    tok.intern(*asSrc);
                if (cookingHasSchema(as, bodyKey, tok.omniphysicsDeformableBodyAPI))
                {
                    if (result.result == omni::physx::PhysxCookingResult::eVALID)
                    {
                        if (result.cookedData)
                        {
                            omni::physx::PhysxCookingSurfaceDeformableBodyData data;
                            cookingService.readSurfaceDeformableBodyData(data, *result.cookedData);
                            const bool storedThisCall = weakPtrToThis->storeSurfaceDeformableBodyDataToUsd(
                                data, desc, bodyKey, *as, result.cookedDataCRC);
                            // Only touch the reference-captured local for a synchronous result: for a
                            // genuinely deferred callback this function has already returned and its
                            // stack frame (and `published`) no longer exists.
                            if (result.isSynchronousResult)
                            {
                                published = storedThisCall;
                            }
                        }
                    }
                    else
                    {
                        // The cook failed: invalidate the stored CRC so the next attempt does not
                        // read it back as "cached data is still valid". Recorded in the
                        // cooked-geometry carrier as well as the USD sink, because on a sink-less
                        // backend the carrier is where the CRC lives (ADR-0022).
                        omni::physx::usdparser::MeshKey crc_zero;
                        if (asSrc)
                        {
                            omni::physx::internal::setCookedBlobValue(
                                *as, bodyKey, asSrc->internToken(kDeformableBodyDataCrcTokenName), &crc_zero,
                                sizeof(crc_zero));
                        }
                        if (auto* dw = as->getDataWrite())
                        {
                            dw->writeByteArrayAttribute(bodyKey, kDeformableBodyDataCrcTokenName,
                                                        reinterpret_cast<const uint8_t*>(&crc_zero), sizeof(crc_zero));
                        }
                    }
                }
            }
        };

        m_cookingServicePrivate.requestSurfaceDeformableBodyCookedData(m_asyncContext, request, params);
        // @see cookVolumeDeformableBodyInternal: an asynchronous submission counts as accepted, a
        // synchronous one is judged on what the write-back actually published.
        reportCookedDataAvailable(asynchronous || published);
    }

    virtual bool cookDeformableVolumeMesh(::physx::PxDefaultMemoryOutputStream& outStream,
                                          const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          omni::physics::parse::ObjectKey bodyKey,
                                          const omni::physx::usdparser::AttachedStage& attachedStage,
                                          bool asynchronous) final
    {
        lock_guard _lock(m_mutex);
        ScopedBlockUSDUpdates _block(this);
        if (!bodyKey.valid())
            return false;
        // Extra guard beyond bodyKey.valid(): a valid key can still fail to resolve to a live
        // object (e.g. removed from the source since bodyKey was captured).
        if (!cookingKeyResolves(&attachedStage, bodyKey))
            return false;
        return cookDeformableVolumeMeshInternal(outStream, desc, bodyKey, attachedStage, asynchronous);
    }

    virtual bool computeDeformableCookingTransform(::physx::PxMat44d* simToCookingTransform,
                                                   ::physx::PxMat44d* cookingToWorldTransform,
                                                   double* cookingToWorldScale,
                                                   const ::physx::PxMat44d& simToWorld,
                                                   const carb::Float3* boundsFitPoints,
                                                   size_t boundsFitPointCount) final
    {
        return omni::physx::computeDeformableCookingTransform(simToCookingTransform, cookingToWorldTransform,
                                                     cookingToWorldScale, simToWorld, boundsFitPoints,
                                                     boundsFitPointCount);
    }

    bool setupDeformableVolumeMeshCookingParams(const omni::physx::usdparser::AttachedStage* attachedStage,
                                                omni::physics::parse::ObjectKey bodyKey,
                                                const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                                omni::physx::DeformableVolumeMeshCookingParams& params,
                                                std::vector<carb::Float3>& pxrSimPoints,
                                                std::vector<carb::Float3>& pxrSimBindPoints,
                                                std::vector<carb::Int4>& pxrSimIndices,
                                                std::vector<carb::Float3>& pxrCollBindPointsInSim,
                                                std::vector<carb::Int4>& pxrCollIndices,
                                                std::vector<carb::Int3>& pxrCollSurfaceIndices)
    {
        if (!bodyKey.valid())
        {
            CARB_LOG_WARN("Cooking failed, deformable body prim is invalid.");
            return false;
        }
        // KnownTokens field lookup for the schema/attribute-name tokens below.
        // attachedStage is non-null past this point (bodyKey.valid() already
        // filtered a null attachedStage out above -- setupDeformableVolumeMeshCookingParams'
        // caller only passes a valid key from a valid attachedStage).
        omni::physics::parse::KnownTokens tok;
        if (const omni::physics::parse::IPhysicsSource* tokSrc = attachedStage->getSource())
            tok.intern(*tokSrc);
        if (!cookingHasSchema(attachedStage, bodyKey, tok.omniphysicsDeformableBodyAPI))
        {
            CARB_LOG_WARN("Cooking failed, deformable body prim is invalid.");
            return false;
        }

        // isTetMeshLike, not cookingIsA(..., tok.tetMeshType): ovstage reports a UsdGeomTetMesh as
        // plain "Mesh" (its populator has no TetMesh mapping), so the concrete-type gate rejected
        // every volume deformable loaded from a non-USD source. See PhysXTools.h::isTetMeshLike.
        if (!omni::physx::internal::isTetMeshLike(*attachedStage, desc.simMeshKey) ||
            !cookingHasSchema(attachedStage, desc.simMeshKey, tok.OmniPhysicsVolumeDeformableSimAPI))
        {
            CARB_LOG_WARN("Cooking failed, simulation mesh prim %s is invalid.", attachedStage->textFor(desc.simMeshKey));
            return false;
        }

        if (desc.simMeshKey != bodyKey)
        {
            const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
            if (!src || src->getParent(desc.simMeshKey) != bodyKey)
            {
                CARB_LOG_WARN(
                    "Cooking failed, simulation mesh %s either has to be identical with bodyPrim %s or directly parented under it.",
                    attachedStage->textFor(desc.simMeshKey), attachedStage->textFor(bodyKey));
                return false;
            }
        }

        if (!omni::physx::internal::isTetMeshLike(*attachedStage, desc.collisionMeshKey))
        {
            CARB_LOG_WARN("Cooking failed, collision mesh prim %s is invalid.", attachedStage->textFor(bodyKey));
            return false;
        }

        if (desc.kinematicBody)
        {
            CARB_LOG_WARN(
                "Cooking failed, kinematic deformables are currently not supported: %s.", attachedStage->textFor(bodyKey));
            return false;
        }
        const ::physx::PxMat44d simToWorld = cookingWorldTransform(attachedStage, desc.simMeshKey);
        const ::physx::PxMat44d worldToSim = omni::physx::affineInverse(simToWorld);

        // read simulation mesh rest shape for cooking (until the SDK supports a proper rest shape)
        // need to make sure the rest shape is compatible with the tetmesh topology
        params.numTetsPerElement = 1;
        {
            std::vector<carb::Float3> simPoints;
            std::vector<carb::Int4> simTetVertexIndices;
            std::vector<carb::Float3> simRestShapePoints;
            std::vector<carb::Int4> simRestTetVtxIndices;

            cookingReadArray(attachedStage, desc.simMeshKey, tok.points, simPoints);
            cookingReadArray(attachedStage, desc.simMeshKey, tok.tetVertexIndices, simTetVertexIndices);
            warnTetMeshOrientation(attachedStage, desc.simMeshKey, simPoints, simTetVertexIndices, desc.simMeshLeftHandedOrientation);

            cookingReadArray(attachedStage, desc.simMeshKey, tok.omniphysicsRestShapePoints, simRestShapePoints);
            cookingReadArray(attachedStage, desc.simMeshKey, tok.omniphysicsRestTetVtxIndices, simRestTetVtxIndices);

            // Recover format from the hex signature, verified against the rest shape used for
            // physx cooking (not the live simulation state points).
            // Absent or mismatching signature => treat as plain tet mesh.
            // Markers are read through the source by key (no UsdPrim): numTetsPerElement is a
            // scalar uint, the hex CRC a UCharArray blob. Both must be present (mirrors the prior
            // HasAuthoredValue gate) — storeMeshKey writes/removes them as a pair.
            uint32_t storedNumTets = 0;
            omni::physx::usdparser::MeshKey storedHexCrc;
            const omni::physics::parse::IPhysicsSource* tetSrc = attachedStage->getSource();
            if (tetSrc &&
                omni::physx::internal::getValue<uint32_t>(*attachedStage, desc.simMeshKey,
                                                          tetSrc->internToken(kSimMeshNumTetsPerElementTokenName),
                                                          omni::physics::parse::ReadTime::defaultTime(), storedNumTets) &&
                (storedNumTets == 5 || storedNumTets == 6) &&
                cookingLoadMeshKey(attachedStage, desc.simMeshKey, kSimMeshHexCrcTokenName, storedHexCrc))
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
                        attachedStage->textFor(desc.simMeshKey));
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
                                         sizeof(carb::Int4) * simTetVertexIndices.size()) != 0;

            if (mismatch)
            {
                CARB_LOG_WARN(
                    "Cooking failed, UsdGeomTetMesh not compatible with rest attributes in DeformableVolumeSimAPI, %s",
                     attachedStage->textFor(bodyKey));
                return false;
            }

            pxrSimPoints.swap(simRestShapePoints);
            pxrSimIndices.swap(simTetVertexIndices);
        }

        if (desc.collisionMeshKey != desc.simMeshKey)
        {
            // Need to construct embedding for collision mesh:
            // (simulation bind pose, sim mesh indices == rest shape indices, collision bind pose) -> embedding
            // (embedding, sim rest shape points, rest shape topo> -> collision rest shape
            std::vector<carb::Float3> simMeshBindPoints;
            std::vector<carb::Float3> collMeshBindPoints;
            {
                const bool gotSim = cookingReadBindPoints(attachedStage, desc.simMeshKey, desc.simMeshBindPoseToken, simMeshBindPoints);
                const bool gotColl = cookingReadBindPoints(attachedStage, desc.collisionMeshKey, desc.collisionMeshBindPoseToken, collMeshBindPoints);
                if (!gotSim || !gotColl)
                {
                    CARB_LOG_ERROR("Cooking failed: %s", attachedStage->textFor(bodyKey));
                    return false;
                }
            }

            // Transform collision points to sim space
            const ::physx::PxMat44d collToWorld = cookingWorldTransform(attachedStage, desc.collisionMeshKey);
            // Gf `collToWorld * worldToSim` -- operands swap under the PhysX convention.
            const ::physx::PxMat44d collToSim = worldToSim * collToWorld;
            for (size_t i = 0; i < collMeshBindPoints.size(); ++i)
            {
                collMeshBindPoints[i] = toFloat3(collToSim.transform(toPhysXd(collMeshBindPoints[i])));
            }
            pxrCollBindPointsInSim.swap(collMeshBindPoints);
            pxrSimBindPoints.swap(simMeshBindPoints);
            cookingReadArray(attachedStage, desc.collisionMeshKey, tok.tetVertexIndices, pxrCollIndices);
            warnTetMeshOrientation(attachedStage, desc.collisionMeshKey, pxrCollBindPointsInSim, pxrCollIndices, desc.collisionMeshLeftHandedOrientation);
            if (desc.collisionMeshLeftHandedOrientation)
            {
                switchTetsOrientation(pxrCollIndices);
            }

            if (pxrSimBindPoints.size() != pxrSimPoints.size())
            {
                CARB_LOG_ERROR("Cooking failed, sim mesh bind pose points incompatible with points: %s", attachedStage->textFor(bodyKey));
                return false;
            }
        }

        // read surface face vertices be from the collsion mesh, even if sim and coll mesh alias
        {
            cookingReadArray(attachedStage, desc.collisionMeshKey, tok.surfaceFaceVertexIndices, pxrCollSurfaceIndices);
            if (pxrCollSurfaceIndices.size() == 0)
            {
                CARB_LOG_WARN("Cooking failed, collision mesh UsdGeomTetMesh needs to have "
                              "surfaceFaceVertexIndices set, %s.", attachedStage->textFor(desc.collisionMeshKey));
                return false;
            }
        }

        const std::vector<carb::Float3>& boundsFitPoints = pxrSimBindPoints.size() > 0 ? pxrSimBindPoints : pxrSimPoints;
        ::physx::PxMat44d simToCookingTransform;
        if (!omni::physx::computeDeformableCookingTransform(
                &simToCookingTransform, nullptr, nullptr, simToWorld,
                boundsFitPoints.data(), boundsFitPoints.size()))
        {
            return false;
        }

        params.simPoints = { pxrSimPoints.data(), pxrSimPoints.size() };
        params.simBindPoints = { pxrSimBindPoints.data(), pxrSimBindPoints.size() };
        params.simIndices = { pxrSimIndices.data(), pxrSimIndices.size() };
        params.collBindPointsInSim = { pxrCollBindPointsInSim.data(), pxrCollBindPointsInSim.size() };
        params.collIndices = { pxrCollIndices.data(), pxrCollIndices.size() };
        params.collSurfaceIndices = { pxrCollSurfaceIndices.data(), pxrCollSurfaceIndices.size() };
        static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
        memcpy(params.simToCookingTransform, simToCookingTransform.front(), sizeof(params.simToCookingTransform));

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
        if (!bodyKey.valid())
            return false;
        // Extra guard beyond bodyKey.valid(): a valid key can still fail to resolve to a live
        // object (e.g. removed from the source since bodyKey was captured).
        if (!cookingKeyResolves(&attachedStage, bodyKey))
            return false;

        omni::physx::DeformableVolumeMeshCookingParams params;
        std::vector<carb::Float3> pxrSimPoints;
        std::vector<carb::Float3> pxrSimBindPoints;
        std::vector<carb::Int4> pxrSimIndices;
        std::vector<carb::Float3> pxrCollBindPointsInSim;
        std::vector<carb::Int4> pxrCollIndices;
        std::vector<carb::Int3> pxrCollSurfaceIndices;
        if (!setupDeformableVolumeMeshCookingParams(&attachedStage, bodyKey, desc, params, pxrSimPoints, pxrSimBindPoints, pxrSimIndices,
                                                    pxrCollBindPointsInSim, pxrCollIndices, pxrCollSurfaceIndices))
        {
            CARB_LOG_ERROR(
                "Deformable volume mesh, failed to setup cooking params, prim: %s", attachedStage.textFor(bodyKey));
            return false;
        }

        omni::physx::PhysxCookingComputeRequest request;
        request.primStageId = uint64_t(attachedStage.getStageId());
        request.attachHandle = attachedStage.getAttachHandle();
        request.primId = 0; // request without input source
        request.deformablePathInfo.bodyPrimId = keyToLegacyPathInt(&attachedStage, bodyKey);
        request.deformablePathInfo.simMeshPrimId = keyToLegacyPathInt(&attachedStage, desc.simMeshKey);
        request.deformablePathInfo.collMeshPrimId = keyToLegacyPathInt(&attachedStage, desc.collisionMeshKey);

        // Hand the cooking service the tet geometry setupDeformableVolumeMeshCookingParams just
        // read through IPhysicsSource, instead of letting it resolve primStageId in
        // UsdUtilsStageCache and re-read the prims itself. That round-trip cannot succeed when the
        // attach has no backing USD stage (stageless ovstage: getStageId() == 0), which is what
        // stopped every volume deformable from being created. primStageId stays set because it is
        // still the service's stage input on the prim-id path; the attach identity now travels
        // separately in attachHandle (ADR-0016 Decision 6), which this request's onFinished happens
        // not to need -- it only writes to the caller's stream.
        //
        // The view aliases exactly the arrays `params` already aliases, so this adds no lifetime
        // requirement beyond the one those spans already impose. It also removes a real divergence:
        // the setup above applies switchTetsOrientation and the numTetsPerElement hex signature,
        // while the service's USD re-read does neither -- so for a left-handed tet mesh the CRC and
        // the cooked input disagreed.
        request.volumeMeshView.simPoints = params.simPoints;
        request.volumeMeshView.simBindPoints = params.simBindPoints;
        request.volumeMeshView.simIndices = params.simIndices;
        request.volumeMeshView.collBindPointsInSim = params.collBindPointsInSim;
        request.volumeMeshView.collIndices = params.collIndices;
        request.volumeMeshView.collSurfaceIndices = params.collSurfaceIndices;
        // Units from the source rather than the stage, so the value -- and the ujitso request key
        // it feeds -- is unchanged for a stage-backed attach and correct without a stage.
        request.primMeshMetersPerUnit = double(attachedStage.getSourceUnits().metersPerUnit);

        // Loud, not silent: the service's emptiness gate was the only thing standing between an
        // unreadable source and a cook that quietly produces nothing.
        if (request.volumeMeshView.isEmpty())
        {
            CARB_LOG_ERROR("Deformable volume mesh, cooking input is empty (sim points %zu, sim tets %zu, "
                           "collision surface tris %zu), prim: %s",
                           request.volumeMeshView.simPoints.size(), request.volumeMeshView.simIndices.size(),
                           request.volumeMeshView.collSurfaceIndices.size(), attachedStage.textFor(bodyKey));
            return false;
        }

        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);

        bool resultSynchronous = false;
        CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);
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
        if (!primKey.valid() || !cookingKeyResolves(&attachedStage, primKey))
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
        omni::physics::parse::KnownTokens tok;
        if (const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource())
            tok.intern(*src);
        if (!cookingIsA(&attachedStage, primKey, tok.meshType))
            return;

        omni::physx::ParticlePoissonSamplingCookingParams params;
        ::physx::PxMat44d rigidTransform(::physx::PxIdentity);
        if (!setupParticlePoissonSamplingCookingParams(primKey, attachedStage, desc, rigidTransform, params))
        {
            CARB_LOG_ERROR("Particle sampler, failed to setup cooking params, prim: %s", attachedStage.textFor(primKey));
            return;
        }

        omni::physx::usdparser::MeshKey originalCrc;
        cookingLoadMeshKey(&attachedStage, primKey, kParticleSamplingCrcTokenName, originalCrc);
        if (!forceResampling)
        {
            // compute data CRC synchronously and exit if the corresponding source value matches.
            // if forceResampling is off, we skip the early out to call processSamplingResults with
            // registerOriginalCount == true
            omni::physx::PhysxCookingComputeRequest request;
            // primStageId/primId stay set as the correlation keys the onFinished continuation and
            // debug logging use (ADR-0016 Decision 6) -- the actual geometry now comes from
            // IPhysicsSource via fillCookingMeshViewFromSource below, same as every other cook.
            // primId is the ObjectKey<->uint64_t correlation encoding (keyToLegacyPathInt).
            request.primStageId = uint64_t(attachedStage.getStageId());
            request.attachHandle = attachedStage.getAttachHandle();
            request.primId = keyToLegacyPathInt(&attachedStage, primKey);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
            request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
            request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
            omni::physx::usdparser::SourceMeshGeometryScope probeGeomScope;
            if (!omni::physx::usdparser::fillCookingMeshViewFromSource(request, probeGeomScope, attachedStage, primKey))
            {
                // REQ-COOK-SOURCE-001 AC-4: unreadable input fails loudly, before any submission.
                CARB_LOG_ERROR("Particle Poisson sampling: could not read source geometry for prim %s", attachedStage.textFor(primKey));
                return;
            }
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
        request.attachHandle = attachedStage.getAttachHandle();
        request.primId = keyToLegacyPathInt(&attachedStage, primKey);
        const std::string_view primText = attachedStage.textViewFor(primKey);
        request.primMeshText = { primText.data(), primText.size() };
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, asynchronous);
        request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, false);
        // geomScope owns the mesh buffers request.primMeshView points at; safe to release once it
        // goes out of scope (function return), since the cooking service copies the view
        // synchronously before any async task is queued (CookingTask::setupTaskFromRequest resets
        // request.primMeshView after).
        omni::physx::usdparser::SourceMeshGeometryScope geomScope;
        if (!omni::physx::usdparser::fillCookingMeshViewFromSource(request, geomScope, attachedStage, primKey))
        {
            // REQ-COOK-SOURCE-001 AC-4: unreadable input fails loudly, before any submission.
            CARB_LOG_ERROR("Particle Poisson sampling: could not read source geometry for prim %s", attachedStage.textFor(primKey));
            return;
        }

        CookingDataAsyncWeakSelf weakPtrToThis(this, m_aliveFlag);
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
            // Resolve the attach by handle, not by primStageId (ADR-0016 Decision 6). The stage-id
            // lookup only ever worked on a stageless attach because getAttachedStage(0) falls back
            // to "the lone attach" -- which silently picks the wrong one under multi-attach and
            // cannot tell a detach/reattach from the original. resolveAttach() returns null for a
            // stale handle and for kNoAttach, and the null branch below already handles that.
            omni::physx::usdparser::AttachedStage* as =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->resolveAttach(result.request->attachHandle);
            if (!as)
            {
                return;
            }

            const omni::physics::parse::ObjectKey samplerKey = legacyPathIntToKey(*as, result.request->primId);
            const omni::physics::parse::IPhysicsSource* source = as->getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);
            if (!(source && source->hasSchema(samplerKey, tok.physxParticleSamplingAPI)))
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
                    if (omni::physics::parse::IPhysicsDataWrite* dataWrite = as->getAuthoringDataWrite())
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

                // Ad hoc marker; the primary record is the neutral write above.
                if (auto* markerWrite = as->getAuthoringDataWrite())
                {
                    markerWrite->writeByteArrayAttribute(samplerKey, kParticleSamplingCrcTokenName,
                                                         reinterpret_cast<const uint8_t*>(&result.cookedDataCRC),
                                                         sizeof(result.cookedDataCRC));
                }

                const carb::Float3* samples = data.positions;
                const uint32_t samplesSize = data.positionsSize;
                // Same nine doubles that were memcpy'd in above; PxMat33d is the
                // element copy of the GfMatrix3d this used to reinterpret to.
                static_assert(sizeof(::physx::PxMat33d) == sizeof(params.shearScale));
                const ::physx::PxMat33d& shearScaleTransform =
                    *reinterpret_cast<const ::physx::PxMat33d*>(params.shearScale);

                // if the original source crc matches with the newly computed crc we call processSamplingResults
                // for registering particle counts of newly instantiated samplers without writing particles, in
                // order to avoid overwriting pre-simulated state.
                bool registerOriginalCount = (originalCrc == result.cookedDataCRC);

                omni::physx::particles::PhysxParticleFactory::processSamplingResults(
                    samplerKey, desc.particleSetKey, samples, samplesSize,
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
        CookingDataAsyncWeakSelf self(this, m_aliveFlag);
        feed->registerInterest(
            omni::physics::parse::ObjectKey{}, omni::physics::parse::TokenId{}, -1,
            [self, as, src](const omni::physics::parse::ChangeBatch& batch)
            {
                if (self)
                    self->handleSourceChange(batch, *as, *src);
                // Recook scheduling never commits solver state, so it can never force a redelivery.
                return true;
            },
            0);

        // Mint this attach's TokenIds now rather than on the first batch: a full
        // KnownTokens intern is a measured cost on the incremental change path.
        // kNoAttach is not a usable cache key, so that case falls back to the lazy build.
        if (attachedStage.getAttachHandle() != omni::physx::kNoAttach)
        {
            lock_guard _lock(m_mutex);
            ensureCollisionTokens(attachedStage.getAttachHandle(), *src);
        }
    }

    /**
    * Per-batch change handler (replaces the legacy UsdNotice listener `handle`).
    * Schedules recooks from source-delivered ChangeBatches: structural batches
    * (resync / delete) feed the api-schema / added-removed sets; value batches are
    * matched by attribute name against the collision-token / xform-attr gates.
    * Everything is keyed by ObjectKey (and TokenId); SdfPath/TfToken are
    * materialized only where a call genuinely still requires one (a PXR API, or
    * a call into a not-yet-retyped helper elsewhere in the plugin).
    */
    void handleSourceChange(const omni::physics::parse::ChangeBatch& batch,
                            omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physics::parse::IPhysicsSource& src)
    {
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
        ensureCollisionTokens(attachedStage.getAttachHandle(), src);

        if (!batch.property.valid())
        {
            // Structural change: object removed, or a resync whose changed fields
            // include apiSchemas (vs a plain add / structural edit). Reads the
            // per-attach cache ensureCollisionTokens() just refreshed above.
            const omni::physics::parse::KnownTokens& tok = m_changeTokens;
            bool apiSchemasChanged = false;
            if (!batch.isDelete && batch.values.type == omni::physics::parse::ColumnType::eToken &&
                batch.values.data)
            {
                const omni::physics::parse::TokenId* fields =
                    static_cast<const omni::physics::parse::TokenId*>(batch.values.data);
                for (size_t j = 0; j < batch.values.count; ++j)
                {
                    if (fields[j] == tok.apiSchemas)
                    {
                        apiSchemasChanged = true;
                        break;
                    }
                }
            }

            for (size_t i = 0; i < keyCount; ++i)
            {
                const omni::physics::parse::ObjectKey key = keys[i];
                if (!key.valid())
                    continue;

                if (apiSchemasChanged)
                {
                    // Schedule the api-schema scan and register the deformable pose
                    // instances' point/purpose attrs as collision tokens (source-
                    // enumerated; replaces prim.GetAppliedSchemas()).
                    m_primApiSchemasChangeRefreshSet.insert(key);
                    src.forEachMultiApplyInstance(
                        key, src.tokenToString(tok.OmniPhysicsDeformablePoseAPI),
                        [&](std::string_view instance)
                        {
                            const omni::physics::parse::TokenId instTok = src.internToken(instance);
                            m_collisionTokens.insert(makeMultiApplyAttributeToken(
                                &src, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints, instTok));
                            m_collisionTokens.insert(makeMultiApplyAttributeToken(
                                &src, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPurposes, instTok));
                        });
                }
                else
                {
                    // Resolve the still-existing PARENT now, while `key` (and therefore
                    // its parent) is guaranteed resolvable -- by the time pump() drains
                    // this set the object named by `key` may already be gone from the
                    // source (a genuine delete), so its own key must not be relied on
                    // later. The parent survives (only the child was removed/added) and
                    // is what the pump()-side deformable-ancestor walk actually needs.
                    const omni::physics::parse::ObjectKey parentKey = src.getParent(key);
                    if (parentKey.valid())
                        m_primAddedRemovedRefreshSet.insert(parentKey);
                    // Be defensive: we don't know exactly what changed, so wipe the
                    // mesh-key cache to keep keys correct.
                    omni::physx::usdparser::notifyStageReset();
                }
            }
        }
        else
        {
            // Value change on a property: classify by TokenId against the
            // collision-token gate directly (both batch.property and
            // m_collisionTokens are already scoped to this same `src`), and by
            // attribute name (materialized only when needed) against the
            // xform-attr gate via the pxr-free isTransformOpAttributeName below
            // (a pure name predicate, reimplemented rather than linking usdGeom
            // just for this one check -- mirrors usdLoad/PrimUpdate.cpp's own
            // identical reimplementation).
            const bool isCollision = m_collisionTokens.find(batch.property) != m_collisionTokens.cend();
            const bool isXform = !isCollision && isTransformOpAttributeName(src.tokenToString(batch.property));
            if (!isCollision && !isXform)
                return;
            for (size_t i = 0; i < keyCount; ++i)
            {
                if (!keys[i].valid())
                    continue;
                if (isCollision)
                    addPrimRefreshSet(keys[i], attachedStage);
                else
                    m_primXformRefreshSet.insert(keys[i]);
            }
        }
    }

    /**
    * Add this object to the list of prims that might need to be recooked
    *
    * @param key : ObjectKey that should be inspected to see if it needs to be recooked (because either the cooking data is missing or the MeshKey (hash) has changed).
    * @param attachHandle : the attach that key was resolved from (ADR-0016 Decision 4). Resolved
    * through the single resolution point UsdLoad::resolveAttach rather than the lone-attach
    * getActiveAttachedStage() fallback, which silently picks the wrong attach under multi-attach.
    * An unresolvable handle is an error with a diagnostic, never a silent no-op.
    */
    virtual void addPrimRefreshSet(omni::physics::parse::ObjectKey key, omni::physics::AttachHandle attachHandle) final
    {
        omni::physx::usdparser::AttachedStage* attachedStage =
            omni::physx::usdparser::UsdLoad::getUsdLoad()->resolveAttach(attachHandle);
        if (!attachedStage)
        {
            CARB_LOG_ERROR(
                "addPrimRefreshSet: could not resolve attach handle %llu, key %llu is not scheduled for recook.",
                static_cast<unsigned long long>(attachHandle), static_cast<unsigned long long>(key.handle));
            return;
        }
        addPrimRefreshSet(key, *attachedStage);
    }

    /**
    * Overload for internal call sites that already hold the AttachedStage& the key was resolved
    * from, so they need not round-trip through an AttachHandle lookup.
    */
    void addPrimRefreshSet(omni::physics::parse::ObjectKey key, omni::physx::usdparser::AttachedStage& attachedStage)
    {
        if (!key.valid())
            return;
        // m_primRefreshSet and invalidateMeshKeyCache are both ObjectKey-keyed
        // (usdLoad/Collision.h/.cpp) -- no path materialization needed. A key that
        // does not resolve on the active attached stage is a safe, pre-existing
        // no-op both here and downstream: pump()'s own consumer loop re-validates
        // `!primKey.IsEmpty()` per entry before doing anything with it.
        m_primRefreshSet.insert(key);
        omni::physx::usdparser::invalidateMeshKeyCache(key);
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
    * @param key : The ObjectKey of the primitive we are referring to
    * @param desc : The shape descriptor for this UsdPrim
    *
    * @return : If a graphics collision representation exists, it will return a pointer to it.
    */
    virtual const omni::physx::CollisionRepresentation *getCollisionRepresentation(omni::physics::parse::ObjectKey key,
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

    // Liveness canary for CookingDataAsyncWeakSelf (see its own comment) -- every async
    // cooking-service completion callback captures a std::weak_ptr to this, not `this`
    // directly. Never reset false explicitly: it goes false automatically when the last
    // shared_ptr (this member) is destroyed, i.e. exactly at ~CookingDataAsyncImpl.
    std::shared_ptr<bool> m_aliveFlag = std::make_shared<bool>(true);

    std::atomic_int32_t m_blockUsdUpdate{0};

    carb::settings::ISettings* m_settings = nullptr;

    PrimRefreshSet      m_primRefreshSet;
    PrimRefreshSet      m_primXformRefreshSet;
    PrimRefreshSet      m_primApiSchemasChangeRefreshSet;
    PrimRefreshSet      m_primAddedRemovedRefreshSet;
    TokenSet            m_collisionTokens;
    // Full KnownTokens for the same attach as m_collisionTokens, interned once by
    // ensureCollisionTokens(). Only valid while m_collisionTokensAttachHandle names
    // a live attach; read only from handleSourceChange, after that call.
    omni::physics::parse::KnownTokens m_changeTokens;
    // The attach m_collisionTokens' TokenIds were last interned under; see
    // ensureCollisionTokens(). Not a raw Source address: this cache outlives
    // detach (CookingDataAsyncImpl is a singleton), so a pointer key could be
    // matched against a later, unrelated source that reused the same address.
    omni::physx::AttachHandle m_collisionTokensAttachHandle = omni::physx::kNoAttach;
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

// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-ATTACH-001
 * @covers AC-20 AC-18 AC-17 AC-16 AC-15 AC-14 AC-13 AC-12 AC-11 AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9
 * @implements REQ-CACHE-INITIAL-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-7
 * @implements REQ-CACHE-UPDATE-001
 * @covers AC-2 AC-3 AC-4 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11 AC-13 AC-14
 */
#include "AttachmentCache.h"
#include "CompiledCacheBuild.h"
#include "../usdLoad/AttachedStage.h"
#include "../OmniPhysX.h"
#include "../Setup.h"
#include "../usdLoad/LoadUsd.h"
#include "../usdLoad/LoadStage.h"
#include "../MeshCache.h"
#include "../internal/InternalScene.h"
#include "../internal/InternalPhysXDatabase.h"
#include "../PhysXScene.h"
#include "PhysXFoundation.h" // foundation::getInterface (CUDA availability)
#include "../../../omni.physics.ovstage/OvstageSource.h"
#include "../../../omni.physics.ovstage/OvstageWalker.h"
#include <omni/physics/ovstage/StageFingerprint.h>
#include <private/omni/physx/RuntimeArtifactService.h>
#include <carb/Framework.h>
#include <carb/datastore/DataStoreConfigurator.h>
#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>
#include <carb/settings/ISettings.h>
#include <carb/ujitso/IDefaultUjitso.h>
#include <set>
#include <omni/physics/parse/CustomTokens.h>

#include <algorithm>
#include <atomic>
#include <future>
#include <optional>
#include <unordered_set>

namespace omni::physx::cache
{
// Replicator environment ids on attach are eligible: hydration gives restored
// objects the ids ordinary loading would (creationEnvironmentId).
bool eligibleOptions(const usdparser::AttachedStage& owner)
{
    return owner.hasExternalSource() && !owner.getStageId() && owner.getExcludedPaths().empty() &&
           owner.getParseOptions().forceSingleScenePath.empty() &&
           owner.getParseOptions().forcedSceneId == usdparser::kInvalidObjectId;
}
namespace
{
using namespace omni::physics::parse;
using namespace omni::physics::parse::cache;
using Result = PhysxCompiledStageCacheResult;
using Mode = PhysxCompiledStageCacheMode;
// One-shot fault injection: the next whole-stage put or insertion publication
// fails without writing.
std::atomic<bool> failNextStore{ false };
std::atomic<carb::datastore::IDataStore*> testDataStore{ nullptr };
std::atomic<uint32_t> testDataStoreFeatures{ carb::datastore::DataStoreConfigurator::Features::Local };
// Native serialization and deserialization need only their own attachment idle,
// which the caller's admission guarantees. PhysX registers deserialized objects
// under the same locks as objects created while other scenes simulate, and
// serialization reads only this attachment's objects, so other attachments keep
// running. Shared authoring admission, as cooking takes, keeps recording and
// process-wide configuration changes out; datastore writes take the store's mutex.
struct SerializationAdmission
{
    RuntimeAuthoringScope authoring;
    bool admitted = bool(authoring);
};
// Scoped use of the source's current hierarchy edges.
struct HierarchyRead
{
    const omni::physics::ovstage::OvstageSource& source;
    explicit HierarchyRead(const omni::physics::ovstage::OvstageSource& value) : source(value)
    {
        source.beginHierarchyBulkRead();
    }
    ~HierarchyRead()
    {
        source.endHierarchyBulkRead();
    }
};
// The prim types and API schemas the population contract marks compiled_cache, and
// those it marks neither that nor "ignores": the parser creates for them what the
// archive cannot hold. Untyped prims carry no physics.
const std::set<std::string_view> supportedTypes = {
    "",
#define POPULATION_PRIM_TYPE(type) type,
#include <omni/physics/ovstage/PopulationReads.inl>
};
const std::set<std::string_view> supportedSchemas = {
#define POPULATION_API_SCHEMA(api) api,
#include <omni/physics/ovstage/PopulationReads.inl>
};
const std::set<std::string_view> uncachedTypes = {
#define POPULATION_UNCACHED_PRIM_TYPE(type) type,
#include <omni/physics/ovstage/PopulationReads.inl>
};
const std::set<std::string_view> uncachedSchemas = {
#define POPULATION_UNCACHED_API_SCHEMA(api) api,
#include <omni/physics/ovstage/PopulationReads.inl>
};
// Whether a prim type or applied-schema name keeps an attachment out of the cache.
// The contract classifies the names it declares. The parser compares any other name
// only as a plugin's registered custom token, whose callbacks are not serialized; a
// name that is neither cannot change what the parser creates and is ignored.
bool uncachedType(std::string_view type)
{
    if (supportedTypes.count(type))
        return false;
    if (uncachedTypes.count(type))
        return true;
    // Native-instancing roots carry ovstage's `<type>Instance` types; their content is
    // the prototype's, captured under stable names. A root with a usd-metadata:typeName
    // is classified by that name instead.
    constexpr std::string_view suffix = "Instance";
    if (type.size() > suffix.size() && type.compare(type.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
        const std::string_view base = type.substr(0, type.size() - suffix.size());
        if (supportedTypes.count(base))
            return false;
        if (uncachedTypes.count(base))
            return true;
    }
    return isCustomToken(CustomTokenKind::eShape, type) || isCustomToken(CustomTokenKind::eJoint, type) ||
           isCustomToken(CustomTokenKind::ePhysicsInstancer, type);
}
// A multi-apply schema is classified by its base name.
bool uncachedSchema(std::string_view schema)
{
    const std::string_view base = schema.substr(0, schema.find(':'));
    if (supportedSchemas.count(base))
        return false;
    return uncachedSchemas.count(base) || isCustomToken(CustomTokenKind::eShape, schema) ||
           isCustomToken(CustomTokenKind::eShape, base);
}
// A scene may hold its default material: the archive restores that binding.
// Any other child of the physics scene keeps the ordinary path.
bool sceneChildrenSupported(const IPhysicsSource& source, ObjectKey scene)
{
    bool supported = true;
    source.forEachChild(scene, [&](ObjectKey child) {
        if (source.tokenToString(source.getTypeName(child)) != "Material")
            supported = false;
    });
    return supported;
}
// A whole stage reports why it is not cached once per attach, at INFO. An inserted
// unit, which can repeat for every environment, reports at VERBOSE.
void logUnsupported(bool insertion, const char* kind, std::string_view name)
{
    if (insertion)
        CARB_LOG_VERBOSE("Compiled stage cache bypassed: unsupported %s %.*s", kind, int(name.size()), name.data());
    else
        CARB_LOG_INFO("Compiled stage cache bypassed: unsupported %s %.*s", kind, int(name.size()), name.data());
}
// The selected snapshot already validated representation, sealing and prototype
// exclusion. Classify the distinct type and schema names its capture recorded while
// hashing, without visiting rows or rebuilding source caches before the parser runs.
bool eligibleCaptured(const usdparser::AttachedStage& owner,
                      const omni::physics::ovstage::PhysicsInputSnapshot& inputs,
                      std::string& scenePath,
                      bool insertion = false)
{
    using TokenName = omni::physics::ovstage::PhysicsInputSnapshot::TokenName;
    if (!eligibleOptions(owner) || !inputs.complete())
        return false;
    // A prim's type is its usd-metadata:typeName where it has one, as an instance root
    // does, else its usd-prim-type. The capture counts usd-prim-type's names over the
    // other prims only, so each prim counts once, under the type the parser reads.
    struct SceneRows
    {
        uint64_t rows = 0, first = 0;
    } typed, authored;
    auto acceptTypes = [&](std::string_view column, SceneRows& sceneRows) {
        return inputs.visitTokenNames(column, [&](const TokenName& type) {
            if (uncachedType(type.name))
            {
                logUnsupported(insertion, "prim type", type.name);
                return false;
            }
            if (type.name == "PhysicsScene")
                sceneRows = { type.rows, type.firstPath };
            return true;
        });
    };
    if (!acceptTypes("usd-metadata:typeName", authored) || !acceptTypes("usd-prim-type", typed))
        return false;
    if (typed.rows + authored.rows != 1)
        return false;
    const uint64_t scene = typed.rows ? typed.first : authored.first;
    if (!inputs.visitTokenNames("usd-schemas", [&](const TokenName& schema) {
            if (schema.name.empty() || !uncachedSchema(schema.name))
                return true;
            logUnsupported(insertion, "schema", schema.name.substr(0, schema.name.find(':')));
            return false;
        }))
        return false;
    const auto* source = dynamic_cast<const omni::physics::ovstage::OvstageSource*>(owner.getSource());
    if (!source)
        return false;
    ovx_string_t text{};
    if (ovx_path_dictionary_path_to_string(ovstage_get_path_dictionary(source->instance()), ovx_primpath_t{scene}, &text) != OVX_OK ||
        !text.ptr)
        return false;
    scenePath.assign(text.ptr, text.length);
    const auto key = source->findByPath(scenePath);
    if (!key.valid())
        return false;
    const auto descendants = inputs.hasDescendants(scene);
    if (descendants.has_value() && !*descendants)
        return true;
    // A subtree snapshot includes scene context, but does not claim that it
    // captured the scene's descendants. Inspect that small boundary live.
    if (!descendants.has_value() && !insertion)
        return false;
    const HierarchyRead read{ *source };
    return sceneChildrenSupported(*source, key);
}
} // namespace
// Decode inside the datastore lease: runtime metadata owns its values, and the
// native section gets its one writable aligned copy for later deserialization.
// Both finish before the borrowed envelope can be released.
bool decodeStoredBundle(const BundleView& view,
                        RuntimeArchive& archive,
                        std::unique_ptr<DecodedCollection>& native,
                        Bytes* deferredRuntime)
{
    if (deferredRuntime)
    {
        deferredRuntime->assign(view.runtime.data, view.runtime.data + view.runtime.size);
        native = DecodedCollection::prepare(view.native.data, view.native.size);
        return native != nullptr;
    }
    auto runtimeDecode = std::async(view.runtime.size > 1024 * 1024 ? std::launch::async : std::launch::deferred, [&]() {
        return decodeRuntimeArchive(view.runtime.data, view.runtime.size, archive, true);
    });
    native = DecodedCollection::prepare(view.native.data, view.native.size);
    const bool runtimeValid = runtimeDecode.get();
    return native && runtimeValid;
}
CookedTriangleMesh cookedTriangleMeshes(bool insertion, bool* missing)
{
    return [insertion, missing](const ::physx::PxTriangleMesh& mesh) {
        std::shared_ptr<const Bytes> cooked = getMeshCache()->getTriangleMeshCookedData(&mesh);
        if (cooked)
            return cooked;
        if (missing)
            *missing = true;
        if (insertion)
            CARB_LOG_VERBOSE("Compiled stage cache: not storing the inserted unit, a triangle mesh has no cooked stream");
        else
            CARB_LOG_INFO("Compiled stage cache: not storing the attachment, a triangle mesh has no cooked stream");
        return cooked;
    };
}
Digest attachmentCompatibility(const usdparser::AttachedStage& owner)
{
    Writer w;
    w.string("ovruntime.compiled-attachment.rigid-serializable.v19.runtime-v16.envelope-v2");
    for (const auto kind : { CustomTokenKind::eJoint, CustomTokenKind::eShape, CustomTokenKind::ePhysicsInstancer })
    {
        auto names = customTokens(kind);
        std::sort(names.begin(), names.end());
        w.u64(names.size());
        for (const auto& name : names)
            w.string(name);
    }
    w.string(OVRUNTIME_COMPILED_CACHE_BUILD);
    w.string(PX_BINARY_SERIAL_VERSION);
    w.u32(sizeof(void*));
    const uint32_t endian = 1;
    w.u8(*reinterpret_cast<const uint8_t*>(&endian));
    const PhysxAttachmentSettings& s = owner.getCachedSettings();
    w.f32(s.viewportGizmoScale);
    w.u32(s.visualizationDisplayParticles);
    w.u8(s.updateToUsd);
    w.u8(s.updateToUsdUsingXformCommonAPI);
    w.u8(s.updateVelocitiesToUsd);
    w.u8(s.outputVelocitiesLocalSpace);
    w.u8(s.updateParticlesToUsd);
    w.u32(s.minFrameRate);
    w.f32(s.jointBodyTransformCheckTolerance);
    w.u8(s.simulateEmptyScene);
    w.u8(s.disableContactProcessing);
    w.u8(s.approximateCones);
    w.u8(s.approximateCylinders);
    w.u8(s.enableExtendedJointAngles);
    w.u8(s.resetOnStop);
    w.u8(s.disableResetOnStop);
    w.u32(s.gpuPipelineOverride);
    w.u32(s.solverTypeOverride);
    w.u8(s.suppressReadback);
    w.u8(s.replicatorEnvIdsOnAttach);
    const PhysxSharedSettings& shared = owner.getSharedSettings();
    w.u32(shared.threadCount);
    w.u8(shared.usePhysxDispatcher);
    w.u32(shared.cudaDevice);
    w.u8(shared.useActiveCudaContext);
    w.u8(shared.synchronousCudaLaunches);
    w.u32(shared.sceneMultiGpuMode);
    w.u8(owner.getParseOptions().exposePrimNames);
    // SDF meshes cook on the GPU when a CUDA device is usable and on the CPU
    // otherwise (CookingDataAsync), and the two builders produce different SDFs.
    // Devices do not come or go while the process runs, and CPU mode, once set,
    // stays set; ask about the device once, since the check warns on every call
    // on a machine without one.
    const omni::physx::IPhysxFoundation& foundation = omni::physx::foundation::getInterface();
    static const bool cudaDevice = foundation.cudaDeviceCheck && foundation.cudaDeviceCheck();
    w.u8(cudaDevice && !(foundation.isCpuMode && foundation.isCpuMode()));
    // Stage units scale what the parser derives (default densities and masses,
    // gravity), and an inserted unit's selected inputs omit the prim authoring them.
    const omni::physics::parse::SourceUnits& units = owner.getSourceUnits();
    w.f32(units.metersPerUnit);
    w.f32(units.kilogramsPerUnit);
    w.u32(uint32_t(units.upAxis));
    const ::physx::PxTolerancesScale scale = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale();
    w.f32(scale.length);
    w.f32(scale.speed);
    return digest(w.data().data(), w.data().size());
}
namespace
{
// A whole stage of at least this many prims, whose scene asks for the GPU, captures its
// inputs on a worker that starts before the attach acquires PhysX resources, while
// restore() creates the scene. That overlap saves time only when the capture is long and
// creating the scene is slow, as a GPU scene with its CUDA context is; for a smaller stage
// or a CPU scene the worker and the scene read ahead of the snapshot cost more than they
// save, so it captures on the attaching thread.
constexpr size_t kEarlyCapturePrims = 10000;
std::atomic<size_t> earlyCapturePrims{ kEarlyCapturePrims };
// Tests capture a CPU scene's inputs early too, as a GPU scene's.
std::atomic<bool> earlyCaptureOnCpu{ false };
// Whole-stage captures started on a worker ahead of the attach's resources, and the
// scenes created ahead of the lookup.
std::atomic<size_t> earlyCaptureCount{ 0 }, earlySceneCount{ 0 };
// The descriptor of the scene a whole-stage load creates for the stage's only
// PhysicsScene prim, read as hydration and the ordinary scene pass read it. False
// when there is none or several: hydration then creates the bundle's scene itself.
// False too when a read failed, which leaves its value unset as if the stage did not
// author it: hydration or the ordinary scene pass then reads the captured inputs.
// Eligible options rule out excluded paths and the single-scene filter.
bool readWholeStageScene(usdparser::AttachedStage& owner,
                         const omni::physics::ovstage::OvstageSource& source,
                         usdparser::PhysxSceneDesc& desc)
{
    std::vector<omni::physics::parse::ObjectKey> scenes;
    if (!source.queryPrimTypeKeys("PhysicsScene", scenes) || scenes.size() != 1 ||
        !source.isA(scenes.front(), source.internToken("PhysicsScene")))
        return false;
    const size_t failed = source.failedSinglePrimReads();
    return usdparser::readSceneFromStage(owner, owner.keyFor(source.sourceKeyToString(scenes.front())), desc) &&
           source.failedSinglePrimReads() == failed;
}
// Creates the scene readWholeStageScene read, as hydration creates it: announced
// later, with the restored objects or by the ordinary scene pass that takes it.
void prepareWholeStageScene(usdparser::AttachedStage& owner, usdparser::PhysxSceneDesc& desc, bool& changing)
{
    CARB_PROFILE_ZONE(0, "Compiled cache: create the scene");
    struct Suppress
    {
        bool& flag;
        bool previous;
        explicit Suppress(bool& value) : flag(value), previous(value)
        {
            flag = true;
        }
        ~Suppress()
        {
            flag = previous;
        }
    } suppress(owner.objectNotificationState().suppressed);
    const size_t first = owner.getInternalPhysXDatabase().getRecords().size();
    // A failure while creating leaves the candidate half changed: discard it.
    changing = true;
    const usdparser::ObjectId id = usdparser::createSceneFromDesc(owner, desc);
    if (id == usdparser::kInvalidObjectId)
        throw std::runtime_error("the compiled cache could not create the stage's scene");
    usdparser::AttachedStage::PreparedScene scene{ desc.primKey, id, {} };
    for (size_t record = first; record < owner.getInternalPhysXDatabase().getRecords().size(); ++record)
        scene.records.push_back(usdparser::ObjectId(record));
    owner.preparedScene = std::move(scene);
    changing = false;
    earlySceneCount.fetch_add(1, std::memory_order_relaxed);
}
} // namespace
struct AttachmentCache::Impl
{
    usdparser::AttachedStage& owner;
    const PhysxCompiledStageCacheOptions* options;
    const omni::physics::ovstage::OvstageSource* source = nullptr;
    omni::physics::ovstage::StageFingerprint fingerprint;
    Digest profile{};
    RuntimeArtifactService* store = nullptr;
    std::shared_ptr<omni::physics::ovstage::PhysicsInputSnapshot> inputs;
    // Started by beginRestore(): the capture's first query, or for a large stage the scene
    // it read, created by restore(), and the capture continuing on a worker.
    bool begun = false;
    std::shared_ptr<omni::physics::ovstage::PhysicsInputSnapshot::StageQuery> stageQuery;
    std::optional<usdparser::PhysxSceneDesc> sceneDesc;
    std::future<std::shared_ptr<omni::physics::ovstage::PhysicsInputSnapshot>> capture;
    // The owner's creation failures before this attachment loaded anything.
    uint32_t creationFailures = owner.creationFailures();
    void result(Result value)
    {
        if (options && options->result)
            *options->result = value;
    }
    // Nothing will be stored: ordinary loading then reads the stage exactly as
    // with the cache disabled, not through the selected snapshot.
    void bypass()
    {
        store = nullptr;
        result(Result::eBypassed);
        if (inputs && source)
            source->usePhysicsInputSnapshot(nullptr);
        inputs.reset();
    }
};
AttachmentCache::AttachmentCache(usdparser::AttachedStage& owner, const PhysxCompiledStageCacheOptions* options)
    : mImpl(new Impl{ owner, options })
{
    owner.configureCompiledInsertion(options);
    // The attach acquires PhysX resources after this and before restore(); a failure
    // there reports the enabled cache as bypassed.
    mImpl->result(Result::eBypassed);
}
AttachmentCache::~AttachmentCache()
{
    if (mImpl->inputs && mImpl->source)
        mImpl->source->usePhysicsInputSnapshot(nullptr);
}
bool bundleAtFault(const std::exception_ptr& error)
{
    try
    {
        std::rethrow_exception(error);
    }
    catch (const BundleDefect&)
    {
        return true;
    }
    catch (...)
    {
        return false;
    }
}
void AttachmentCache::beginRestore()
{
    Impl& c = *mImpl;
    if (c.begun)
        return;
    c.begun = true;
    try
    {
        // restore() repeats these checks and reports what they decide.
        if (!c.options || (c.options->mode != Mode::eReadOnly && c.options->mode != Mode::eReadWrite) ||
            !eligibleOptions(c.owner))
            return;
        // An unusable datastore costs no identity work (restore() reports eUnavailable).
        const auto* source = dynamic_cast<const omni::physics::ovstage::OvstageSource*>(c.owner.getSource());
        if (!source || !c.owner.compiledInsertionStore())
            return;
        // The capture's own first query counts the stage's prims. A smaller stage, or one
        // whose scene simulates on the CPU, continues from it in restore(), on the
        // attaching thread.
        size_t prims = 0;
        c.stageQuery = omni::physics::ovstage::PhysicsInputSnapshot::queryStage(source->instance(), prims);
        const bool cpuToo = earlyCaptureOnCpu.load(std::memory_order_relaxed);
        if (!c.stageQuery || prims < earlyCapturePrims.load(std::memory_order_relaxed) ||
            !(cpuToo || attachmentMayUseGpu(c.owner)))
            return;
        // The bundle holds no scene: a hit creates it from the stage, as ordinary
        // loading does before anything else. Read it before the capture's reads;
        // restore() creates it while they run, and hydration or the ordinary scene
        // pass takes it.
        c.sceneDesc.emplace();
        if (!readWholeStageScene(c.owner, *source, *c.sceneDesc))
            c.sceneDesc.reset();
        else if (!(cpuToo || sceneRequestsGpu(c.owner, *c.sceneDesc)))
        {
            c.sceneDesc.reset();
            return;
        }
        c.capture = std::async(std::launch::async, [source, stage = std::move(c.stageQuery)]() mutable {
            return omni::physics::ovstage::PhysicsInputSnapshot::capture(source->instance(), source->readOrdinal(),
                                                                         omni::physics::ovstage::physicsInputColumns(),
                                                                         {}, {}, nullptr, std::move(stage));
        });
        earlyCaptureCount.fetch_add(1, std::memory_order_relaxed);
    }
    catch (...)
    {
        // restore() then captures on its own.
        c.stageQuery.reset();
        c.sceneDesc.reset();
    }
}
AttachmentCache::Restore AttachmentCache::restore()
{
    CARB_PROFILE_ZONE(0, "Compiled cache: restore attachment");
    Impl& c = *mImpl;
    if (!c.options || c.options->mode == Mode::eDisabled)
        return Restore::eMiss;
    c.result(Result::eBypassed);
    // Set once hydration starts changing the attachment.
    bool changing = false;
    // A delivered bundle at fault is removed from the cache: later lookups skip it,
    // and this attach's own store, or a later one, replaces it.
    std::optional<Digest> delivered;
    RuntimeArtifactService* rejecting = nullptr;
    auto reject = [&]() {
        try
        {
            if (delivered && rejecting)
                rejecting->remove(
                    RuntimeArtifactService::ArtifactKind::eWholeStage, c.fingerprint.value, c.profile, *delivered);
        }
        catch (...)
        {
        }
    };
    std::vector<usdparser::ObjectId> created;
    try
    {
        if ((c.options->mode != Mode::eReadOnly && c.options->mode != Mode::eReadWrite) || !eligibleOptions(c.owner))
            return Restore::eMiss;
        c.source = dynamic_cast<const omni::physics::ovstage::OvstageSource*>(c.owner.getSource());
        if (!c.source)
            return Restore::eMiss;
        // Open the store first: an unusable datastore costs no identity capture.
        RuntimeArtifactService* store = c.owner.compiledInsertionStore();
        if (!store)
        {
            c.bypass();
            c.result(Result::eUnavailable);
            return Restore::eMiss;
        }
        // The selected identity covers type/schema and hierarchy changes. A
        // matched artifact carries the capability verdict stored with it; audit
        // capabilities only when compiling a miss. beginRestore() started the
        // capture before the attach acquired PhysX resources; for a large stage
        // the scene is created while it runs on a worker.
        beginRestore();
        if (c.sceneDesc)
            prepareWholeStageScene(c.owner, *c.sceneDesc, changing);
        c.sceneDesc.reset();
        c.inputs = c.capture.valid() ?
                       c.capture.get() :
                       omni::physics::ovstage::PhysicsInputSnapshot::capture(
                           c.source->instance(), c.source->readOrdinal(), omni::physics::ovstage::physicsInputColumns(),
                           {}, {}, nullptr, std::move(c.stageQuery));
        c.fingerprint = c.inputs->fingerprint();
        if (c.fingerprint)
            c.source->usePhysicsInputSnapshot(c.inputs);
        if (!c.fingerprint)
        {
            CARB_LOG_INFO("Compiled stage cache bypassed: %s", c.fingerprint.bypassReason.c_str());
            // The ordinary load runs without the snapshot and its read groups.
            c.bypass();
            return Restore::eMiss;
        }
        c.store = store;
        rejecting = c.store;
        c.profile = attachmentCompatibility(c.owner);
        c.result(Result::eMiss);
        // A matching input/build artifact was admitted by this exact capability
        // profile when stored. Only misses need to repeat the full source audit.
        auto miss = [&]() {
            std::string scenePath;
            if (!eligibleCaptured(c.owner, *c.inputs, scenePath))
                c.bypass();
            return Restore::eMiss;
        };
        using Kind = RuntimeArtifactService::ArtifactKind;
        RuntimeArchive archive;
        std::unique_ptr<DecodedCollection> collection;
        Bytes runtime;
        // Read and validate outside serialization admission: a datastore wait must
        // not hold other attachments. Both sections are copied; no native object exists yet.
        const bool copied = c.store->visit(
            Kind::eWholeStage, c.fingerprint.value, c.profile,
            [&](const BundleView& bundle) {
                return decodeStoredBundle(bundle, archive, collection, &runtime);
            },
            &delivered);
        if (!copied)
        {
            reject();
            return miss();
        }
        auto& registry = *OmniPhysX::getInstance().getPhysXSetup().getSerializationRegistry();
        Hydration hydration = Hydration::eRejected;
        {
            // The runtime archive decodes while the native section deserializes.
            auto runtimeDecode =
                std::async(runtime.size() > 1024 * 1024 ? std::launch::async : std::launch::deferred,
                           [&]() { return decodeRuntimeArchive(runtime.data(), runtime.size(), archive, true); });
            SerializationAdmission admission;
            if (!admission.admitted)
            {
                c.bypass();
                return Restore::eMiss;
            }
            // Native objects that hydration did not take are destroyed under
            // admission, also when decoding or path resolution throws.
            struct ReleaseUnused
            {
                std::unique_ptr<DecodedCollection>& collection;
                ~ReleaseUnused()
                {
                    collection.reset();
                }
            } releaseUnused{ collection };
            const bool deserialized = collection->deserialize(registry);
            const bool decoded = runtimeDecode.get();
            // Archived prototype geometry names resolve through this population's
            // instancing graph, which the identity already matched.
            if (deserialized && decoded &&
                omni::physics::ovstage::resolvePrototypePaths(c.source->instance(), archive.paths, c.inputs.get()))
                hydration =
                    hydrateAttachment(c.owner, archive, std::move(collection), false, nullptr, &created, &changing);
        }
        // A bundle that does not decode or does not fit this attachment is rejected
        // before the attachment changes, so that a store replaces it, and the
        // attachment loads ordinarily as on a miss. So is one whose prototype names
        // this population cannot resolve, which the identity's prototype mapping
        // should rule out.
        if (hydration == Hydration::eRejected)
        {
            reject();
            return miss();
        }
        // The caller discards a candidate that hydration changed and attaches
        // again from a fresh one without the lookup, as on a miss. A bundle that
        // contradicted itself leaves the cache; one the live stage or process
        // failed to apply stays for later attaches.
        if (hydration == Hydration::eFailed || hydration == Hydration::eInterrupted)
        {
            if (hydration == Hydration::eFailed)
                reject();
            c.result(Result::eRestoreFailed);
            return Restore::eFailed;
        }
        c.result(Result::eHit);
    }
    catch (...)
    {
        // Before the attachment changed, it loads ordinarily without storing.
        c.bypass();
        if (bundleAtFault(std::current_exception()))
            reject();
        if (!changing)
            return Restore::eMiss;
        c.result(Result::eRestoreFailed);
        return Restore::eFailed;
    }
    // Opted-in initial-population subscribers see the restored objects, as
    // ordinary loading announces each object it creates. A throwing subscriber
    // fails the attach as it fails ordinary loading; the bundle is not to blame.
    notifyRestoredObjects(c.owner, created);
    return Restore::eHit;
}
bool AttachmentCache::preparedSceneTaken()
{
    Impl& c = *mImpl;
    if (!c.owner.preparedScene)
        return true;
    CARB_LOG_INFO("Compiled stage cache: ordinary loading did not take the scene created ahead at %s",
                  std::string(c.owner.textViewFor(c.owner.preparedScene->key)).c_str());
    c.bypass();
    c.result(Result::eRestoreFailed);
    return false;
}
void AttachmentCache::store()
{
    CARB_PROFILE_ZONE(0, "Compiled cache: store attachment");
    Impl& c = *mImpl;
    if (!c.store || c.options->mode != Mode::eReadWrite)
        return;
    try
    {
        if (!c.inputs->complete())
        {
            c.result(Result::eBypassed);
            return;
        }
        // A shape or material the load failed to create is missing from the result.
        if (const uint32_t failed = c.owner.creationFailures() - c.creationFailures)
        {
            CARB_LOG_INFO(
                "Compiled stage cache: not storing the attachment, %u shapes or materials failed to load", failed);
            c.result(Result::eStoreFailed);
            return;
        }
        c.owner.getPhysXPhysicsInterface()->finishSetup(c.owner);
        RuntimeArchive archive;
        CollectionPtr roots(PxCreateCollection());
        Bundle bundle;
        bundle.input = c.fingerprint.value;
        bundle.compatibility = c.profile;
        auto& registry = *OmniPhysX::getInstance().getPhysXSetup().getSerializationRegistry();
        auto nativeWork = [&]() { return encodeCollection(*roots, registry, cookedTriangleMeshes(), bundle.native); };
        {
            // Capture and native serialization read this attachment's SDK objects
            // under admission; the datastore write runs outside it.
            SerializationAdmission admission;
            if (!admission.admitted || !roots ||
                !captureAttachmentData(c.owner, archive, *roots, {}, nativeWork, &bundle.runtime, nullptr, c.inputs.get()))
            {
                c.result(Result::eBypassed);
                return;
            }
        }
        if (!c.inputs->complete())
        {
            c.result(Result::eBypassed);
            return;
        }
        const bool stored = !failNextStore.exchange(false, std::memory_order_relaxed) &&
                            c.store->put(RuntimeArtifactService::ArtifactKind::eWholeStage, bundle);
        c.result(stored ? Result::eStored : Result::eStoreFailed);
    }
    catch (...)
    {
        c.result(Result::eStoreFailed);
    }
}

namespace
{
bool under(std::string_view path, std::string_view root)
{
    return path == root || (path.size() > root.size() && path.substr(0, root.size()) == root && path[root.size()] == '/');
}
// A plain unit is a few rigid bodies with implicit colliders: transform and
// primitive-geometry prims only. Its ordinary parse costs less than input
// capture plus serialization, and nothing in it cooks, links bodies or adds
// resources a later hit could share. Joints, meshes, materials and every other
// type keep the cached path, as does any prim without an exact type row from
// this update's change feed. Bypassing never changes the loaded result.
constexpr size_t kPlainInsertionPrimLimit = 16; // prims below the unit root
std::atomic<bool> plainInsertionBypass{ true }; // cache-mechanics tests may cache plain units
// keys[0] is the unit root; the descendant walk may visit it again.
bool hasPhysicsSchema(const omni::physics::ovstage::OvstageSource& source, ObjectKey key)
{
    bool physics = false;
    source.forEachAppliedSchema(key, [&](TokenId token) {
        const std::string_view schema = source.tokenToString(token);
        physics |= schema.rfind("Physics", 0) == 0 || schema.rfind("Physx", 0) == 0;
    });
    return physics;
}
// A prim ordinary loading creates objects for: physics schemas or a physics type.
bool physicsPrim(const omni::physics::ovstage::OvstageSource& source, ObjectKey key)
{
    const std::string_view type = source.tokenToString(source.getTypeName(key));
    return type.rfind("Physics", 0) == 0 || type.rfind("Physx", 0) == 0 || hasPhysicsSchema(source, key);
}
bool plainInsertionUnit(const omni::physics::ovstage::OvstageSource& source, const std::vector<ObjectKey>& keys)
{
    if (keys.empty() || !plainInsertionBypass.load(std::memory_order_relaxed))
        return false;
    const ObjectKey root = keys.front();
    if (size_t(std::count_if(keys.begin() + 1, keys.end(), [&](ObjectKey key) { return key != root; })) >
        kPlainInsertionPrimLimit)
        return false;
    static constexpr std::string_view kPlainTypes[] = { "Xform",     "Scope",    "Cube",       "Sphere", "Capsule",
                                                        "Capsule_1", "Cylinder", "Cylinder_1", "Cone",   "Plane" };
    uint64_t plainTokens[std::size(kPlainTypes)];
    for (size_t i = 0; i < std::size(kPlainTypes); ++i)
        plainTokens[i] = source.ovxToken(kPlainTypes[i]).raw_id;
    for (const ObjectKey key : keys)
    {
        omni::physics::ovstage::OvstageSource::SharedInputTypeRow row;
        if (!source.sharedPhysicsInputType(source.rawHandle(key), row) || !row.token ||
            std::find(std::begin(plainTokens), std::end(plainTokens), row.token) == std::end(plainTokens))
            return false;
    }
    return true;
}
} // namespace
bool loadCompiledInsertion(usdparser::AttachedStage& owner,
                           const std::vector<std::string>& roots,
                           const std::function<bool(const std::string&)>& load)
{
    CARB_PROFILE_ZONE(0, "Compiled cache: load insertion");
    bool ordinaryStarted = false, insertionCommitted = false;
    try
    {
        using Kind = RuntimeArtifactService::ArtifactKind;
        owner.compiledInsertionResult = Result::eDisabled;
        if (owner.compiledInsertionMode() == Mode::eDisabled)
            return false;
        owner.compiledInsertionResult = Result::eBypassed;
        // A unit must be absent and self-contained. Cross-root batches retain the
        // ordinary parser's dependency ordering until they have a joint unit plan.
        if (roots.empty() || !eligibleOptions(owner) || owner.getPhysXScenes().size() != 1)
            return false;
        std::string root = roots.front();
        for (const auto& path : roots)
            while (!root.empty() && !under(path, root))
                root = root.substr(0, root.find_last_of('/'));
        if (root.empty() || root == "/")
            return false;
        auto* source = dynamic_cast<omni::physics::ovstage::OvstageSource*>(owner.getSource());
        if (!source)
            return false;
        // The identity is captured at the source's ordinal and ordinary loading
        // scans at the attachment's. A drain that moved the source and then failed
        // leaves them apart until its range is retried: storing then would file one
        // ordinal's scan under the other's identity.
        if (source->readOrdinal() != owner.attachTarget().readOrdinal)
            return false;
        const ObjectKey rootKey = source->findByPath(root);
        if (!rootKey.valid())
            return false;
        InsertionCaptureSelection selection;
        selection.keys.push_back(rootKey);
        const auto gatherKeys = [&]() {
            source->forEachDescendant(rootKey, [&](ObjectKey key) { selection.keys.push_back(key); });
        };
        if (source->hierarchyCacheBuilt())
        {
            // The change feed patched (or invalidated) this cache before dispatch,
            // exactly as for ordinary incremental loading; reuse its current edges.
            const HierarchyRead read{ *source };
            gatherKeys();
        }
        else
            source->withSubtreeHierarchy(rootKey, gatherKeys);
        // Decide before the scene-wide preflight and any input capture, so a
        // bypassed unit costs no more than this subtree walk.
        if (plainInsertionUnit(*source, selection.keys))
            return false;
        // Instance proxies read prototype rows outside the unit's selected inputs.
        for (const ObjectKey key : selection.keys)
            if (source->isInstance(key) || source->isInstanceProxy(key))
                return false;
        // Ordinary loading parses only the notified roots. A physics prim elsewhere
        // under their common root that never loaded, such as a joint authored before
        // its bodies, stays absent there, while parsing the whole unit would create it.
        if (roots.size() > 1)
            for (const ObjectKey key : selection.keys)
            {
                if (!physicsPrim(*source, key))
                    continue;
                const std::string_view path = source->sourceKeyToString(key);
                if (std::none_of(
                        roots.begin(), roots.end(), [&](const std::string& notified) { return under(path, notified); }))
                    return false;
            }
        // Insertion never creates or resets the scene. Its settings come from the
        // scene prim, which is part of the unit's identity; its default materials take
        // part in loading the unit, so their prims join the identity too.
        const PhysxSceneDesc& sceneDesc = owner.getPhysXScenes().begin()->second->getInternalScene()->getSceneDesc();
        if (sceneDesc.synthetic || sceneDesc.enableQuasistatic || !sceneDesc.quasistaticActors.empty())
            return false;
        const std::string scenePath = owner.textFor(sceneDesc.primKey);
        // An unusable datastore costs no preflight and no capture.
        auto* store = owner.compiledInsertionStore();
        if (!store)
        {
            owner.compiledInsertionResult = Result::eUnavailable;
            return false;
        }
        // Ordinary loading attaches a unit to a physics ancestor: a collider joins
        // the ancestor body and updates its mass, a link joins its articulation.
        // A unit captures only its own objects, so no existing record may belong to
        // its prims or their ancestors, and no ancestor may carry physics schemas.
        std::vector<ObjectKey> ancestors;
        for (ObjectKey parent = source->getParent(rootKey); parent.valid(); parent = source->getParent(parent))
        {
            if (hasPhysicsSchema(*source, parent))
                return false;
            ancestors.push_back(parent);
            if (parent == source->getRootKey())
                break;
        }
        const auto& database = owner.getInternalPhysXDatabase();
        for (const auto* keys : { &selection.keys, &ancestors })
            for (const ObjectKey key : *keys)
                if (database.hasLiveRecord(key))
                    return false;
        // A body that a resync removed in this flush leaves the joints from outside
        // its unit waiting for its ordinary re-creation to reattach them; a restored
        // body would leave them detached.
        const auto& waitingJoints = database.getPxJointMap();
        if (!waitingJoints.empty())
            for (const ObjectKey key : selection.keys)
                if (waitingJoints.count(key))
                    return false;
        // Runtime state keyed by a unit prim can outlive its removal (collision-group
        // membership, registrations, schema flags); ordinary loading would apply it.
        const auto& objects = *owner.getObjectDatabase();
        for (const ObjectKey key : selection.keys)
        {
            const auto* entries = objects.getEntries(key);
            if ((entries && !entries->empty()) || objects.getSchemaAPIs(key) ||
                owner.getCollisionGroupMap().count(key) || owner.getAnimatedKinematicBodies().count(key))
                return false;
            for (const auto& map : owner.getAdditionalCollisionGroupMaps())
                if (map.count(key))
                    return false;
        }
        std::vector<std::string> paths{ root, scenePath };
        paths.reserve(selection.keys.size() + 8);
        selection.paths = &paths;
        for (size_t i = 1; i < selection.keys.size(); ++i)
            paths.emplace_back(source->sourceKeyToString(selection.keys[i]));
        for (const ObjectKey parent : ancestors)
            paths.emplace_back(source->sourceKeyToString(parent));
        for (const ObjectKey material :
             { sceneDesc.defaultMaterialDesc.materialKey, sceneDesc.defaultDeformableMaterialDesc.materialKey,
               sceneDesc.defaultSurfaceDeformableMaterialDesc.materialKey, sceneDesc.defaultPBDMaterialDesc.materialKey })
            if (material.valid())
                paths.emplace_back(owner.textFor(material));
        std::sort(paths.begin(), paths.end());
        paths.erase(std::unique(paths.begin(), paths.end()), paths.end());
        auto inputs = omni::physics::ovstage::PhysicsInputSnapshot::capture(
            source->instance(), source->readOrdinal(), omni::physics::ovstage::physicsInputColumns(), paths, root,
            source);
        if (!inputs->complete())
            return false;
        // Ordinary loading resolves a target outside the unit against runtime
        // registrations the identity does not cover: a joint body, a filtered pair,
        // a collision group, a physics material. A binding to an outside material
        // without physics schemas is the exception: it resolves to the scene default,
        // and hashing the target below changes the identity if it gains one.
        std::vector<std::string> outsideMaterials;
        auto* dictionary = ovstage_get_path_dictionary(source->instance());
        if (!dictionary || !inputs->visitPathTargets([&](uint64_t prim, std::string_view column, std::string_view target) {
                if (under(target, root))
                    return true;
                ovx_string_t text{};
                if (ovx_path_dictionary_path_to_string(dictionary, ovx_primpath_t{prim}, &text) != OVX_OK || !text.ptr)
                    return false;
                // The scene's own bindings (its default materials) are profile inputs.
                if (std::string_view(text.ptr, text.length) == scenePath)
                    return true;
                if (target == scenePath && column == "physics:simulationOwner")
                    return true;
                if (column.rfind("material:binding", 0) != 0)
                    return false;
                const ObjectKey material = source->findByPath(std::string(target));
                if (material.valid() && hasPhysicsSchema(*source, material))
                    return false;
                outsideMaterials.emplace_back(target);
                return true;
            }))
            return false;
        if (!outsideMaterials.empty())
        {
            paths.insert(paths.end(), outsideMaterials.begin(), outsideMaterials.end());
            std::sort(paths.begin(), paths.end());
            paths.erase(std::unique(paths.begin(), paths.end()), paths.end());
            inputs = omni::physics::ovstage::PhysicsInputSnapshot::capture(source->instance(), source->readOrdinal(),
                                                                           omni::physics::ovstage::physicsInputColumns(),
                                                                           paths, root, source);
            if (!inputs->complete())
                return false;
        }
        Writer profileWriter;
        profileWriter.string("ovruntime.compiled-insertion.v3");
        const auto compatibility = attachmentCompatibility(owner);
        profileWriter.bytes(compatibility.data(), compatibility.size());
        // Creation also reads live scene state that can differ from the scene prim
        // the identity hashes. Scene-query support and the iteration clamps are
        // taken once when the scene is created, gravity changes through the tensor
        // API without touching the prim, and default contact offsets follow gravity
        // and the time step. Debug visualization flags what is created.
        const PhysXScene& liveScene = *owner.getPhysXScenes().begin()->second;
        profileWriter.u8(liveScene.isSceneQuerySupported());
        profileWriter.u32(sceneDesc.minPosIterationCount);
        profileWriter.u32(sceneDesc.maxPosIterationCount);
        profileWriter.u32(sceneDesc.minVelIterationCount);
        profileWriter.u32(sceneDesc.maxVelIterationCount);
        const ::physx::PxVec3 gravity = liveScene.getScene()->getGravity();
        profileWriter.f32(gravity.x);
        profileWriter.f32(gravity.y);
        profileWriter.f32(gravity.z);
        profileWriter.u32(liveScene.getTimeStepsPerSeconds());
        profileWriter.u8(owner.isDebugVisualizationEnabled());
        const auto profile = digest(profileWriter.data().data(), profileWriter.data().size());
        auto& registry = *OmniPhysX::getInstance().getPhysXSetup().getSerializationRegistry();
        source->usePhysicsInputSnapshot(inputs);
        struct ClearInputs
        {
            const omni::physics::ovstage::OvstageSource& source;
            ~ClearInputs()
            {
                source.usePhysicsInputSnapshot(nullptr);
            }
        } clear{ *source };
        owner.compiledInsertionResult = Result::eMiss;
        // A failure once hydration changed the attachment rolls the unit back and
        // stays reported as eRestoreFailed. A delivered bundle at fault (see
        // BundleDefect) is removed from the cache, and the ordinary load below then
        // stores its replacement; a failure of the live stage or process keeps it.
        bool restoreFailed = false, changing = false;
        std::optional<Digest> delivered;
        auto settle = [&](bool changed, bool atFault) {
            if (changed)
            {
                restoreFailed = true;
                owner.compiledInsertionResult = Result::eRestoreFailed;
            }
            try
            {
                if (atFault && delivered)
                    store->remove(Kind::eReferenceSubtree, inputs->fingerprint().value, profile, *delivered);
            }
            catch (...)
            {
            }
        };
        try
        {
            // A matching input/build bundle already passed the capability audit.
            // Read, validate and decode outside serialization admission.
            RuntimeArchive archive;
            std::unique_ptr<DecodedCollection> native;
            const bool decoded = store->visit(
                Kind::eReferenceSubtree, inputs->fingerprint().value, profile,
                [&](const BundleView& view) {
                    return decodeStoredBundle(view, archive, native);
                },
                &delivered);
            bool validPaths = decoded;
            for (auto& path : archive.paths)
            {
                if (!validPaths)
                    break;
                if (under(path, omni::physics::parse::cache::kUnitRoot))
                    path = root + path.substr(omni::physics::parse::cache::kUnitRoot.size());
                else if (path != scenePath)
                    validPaths = false;
            }
            if (delivered && !validPaths)
                settle(false, true);
            else if (validPaths)
            {
                std::vector<usdparser::ObjectId> created;
                bool admitted = false;
                Hydration hydration = Hydration::eRejected;
                {
                    // The update holds this attachment's admission.
                    SerializationAdmission admission;
                    admitted = admission.admitted;
                    if (admitted && native->deserialize(registry))
                        hydration = hydrateAttachment(
                            owner, archive, std::move(native), true, &insertionCommitted, &created, &changing);
                }
                if (hydration == Hydration::eHydrated)
                {
                    notifyRestoredObjects(owner, created);
                    owner.compiledInsertionResult = Result::eHit;
                    return true;
                }
                // A collection PhysX would not deserialize stays eRejected.
                if (admitted)
                    settle(hydration != Hydration::eRejected, hydration != Hydration::eInterrupted);
            }
        }
        catch (...)
        {
            // Observer dispatch can throw after publication. Never create a second
            // copy of an already committed unit through the ordinary fallback.
            if (insertionCommitted)
                throw;
            settle(changing, bundleAtFault(std::current_exception()));
        }
        // Audit only the captured unit and its declared context. Unrelated
        // unsupported objects do not affect a self-contained insertion; runtime
        // bindings and the native closure still reject external dependencies.
        std::string auditedScene;
        if (!eligibleCaptured(owner, *inputs, auditedScene, true) || auditedScene != scenePath)
        {
            owner.compiledInsertionResult = Result::eBypassed;
            return false;
        }
        source->usePhysicsInputSnapshot(inputs);
        // Failure before commit rolls back only this unit. Ordinary loading still
        // receives the same retained source buffers and the untouched live scene.
        std::optional<internal::InternalDatabase::CreationJournal> created;
        if (owner.compiledInsertionMode() == Mode::eReadWrite)
        {
            created.emplace(owner.getInternalPhysXDatabase());
            created->records.reserve(selection.keys.size() * 3);
        }
        // A failed ordinary load is the caller's to report; nothing of it is stored.
        const uint32_t creationFailures = owner.creationFailures();
        // From here a failure is the ordinary load's: the caller must not load again.
        ordinaryStarted = true;
        if (!load(root) || owner.compiledInsertionMode() != Mode::eReadWrite || !inputs->complete())
            return true;
        // Nor is a unit with a shape or material the load failed to create.
        if (const uint32_t failed = owner.creationFailures() - creationFailures)
        {
            CARB_LOG_INFO(
                "Compiled stage cache: not storing the inserted unit, %u shapes or materials failed to load", failed);
            if (!restoreFailed)
                owner.compiledInsertionResult = Result::eStoreFailed;
            return true;
        }
        try
        {
            owner.getPhysXPhysicsInterface()->finishSetup(owner);
            selection.records = std::move(created->records);
            CollectionPtr collection(PxCreateCollection());
            RuntimeArchive archive;
            Bundle bundle;
            bundle.input = inputs->fingerprint().value;
            bundle.compatibility = profile;
            bool missingStream = false;
            auto nativeWork = [&]() {
                return encodeCollection(
                    *collection, registry, cookedTriangleMeshes(true, &missingStream), bundle.native, true);
            };
            {
                SerializationAdmission admission;
                if (!admission.admitted)
                {
                    if (!restoreFailed)
                        owner.compiledInsertionResult = Result::eBypassed;
                    return true;
                }
                if (!collection || !captureAttachmentData(owner, archive, *collection, root, nativeWork,
                                                          &bundle.runtime, &selection, inputs.get()))
                {
                    // A unit with a triangle mesh created without its cooked stream
                    // is never stored; a whole stage reports the same as bypassed.
                    if (missingStream && !restoreFailed)
                        owner.compiledInsertionResult = Result::eBypassed;
                    return true;
                }
            }
            if (!inputs->complete())
                return true;
            // The owned bundle is checksummed and written beside later work; the
            // next lookup joins it. A failed asynchronous write only loses reuse.
            const bool published = !failNextStore.exchange(false, std::memory_order_relaxed) &&
                                   store->publish(Kind::eReferenceSubtree, std::move(bundle));
            // A failed restore stays reported; the published unit replaces its bundle.
            if (!restoreFailed)
                owner.compiledInsertionResult = published ? Result::eStored : Result::eStoreFailed;
        }
        catch (...)
        {
            if (!restoreFailed)
                owner.compiledInsertionResult = Result::eStoreFailed;
        }
        return true;
    }
    catch (...)
    {
        // Cache preparation is optional. Preserve ordinary parser exceptions and
        // post-publication errors instead of retrying a partially handled load.
        if (ordinaryStarted || insertionCommitted)
            throw;
        owner.compiledInsertionResult = Result::eBypassed;
        return false;
    }
}
void setPlainInsertionBypassForTest(bool enabled)
{
    plainInsertionBypass.store(enabled, std::memory_order_relaxed);
}
// One-shot: the next publication fails before it is queued. False withdraws it.
void failNextCompiledStoreForTest(bool enabled)
{
    failNextStore.store(enabled, std::memory_order_relaxed);
}
// Whole stages of at least `prims` prims capture early, CPU scenes too when `cpu`; 0
// restores the defaults.
void setEarlyCaptureMinimumPrimsForTest(size_t prims, bool cpu)
{
    earlyCapturePrims.store(prims ? prims : kEarlyCapturePrims, std::memory_order_relaxed);
    earlyCaptureOnCpu.store(prims && cpu, std::memory_order_relaxed);
}
size_t getEarlyCompiledCaptureCountForTest()
{
    return earlyCaptureCount.load(std::memory_order_relaxed);
}
size_t getEarlyCompiledSceneCountForTest()
{
    return earlySceneCount.load(std::memory_order_relaxed);
}
void useCompiledStageDataStoreForTest(carb::datastore::IDataStore* dataStore)
{
    useCompiledStageDataStoreForTest(dataStore, carb::datastore::DataStoreConfigurator::Features::Local);
}
void useCompiledStageDataStoreForTest(carb::datastore::IDataStore* dataStore, uint32_t features)
{
    testDataStoreFeatures.store(features);
    testDataStore.store(dataStore);
    CompiledStageStore::forgetProcessRejections();
}
carb::datastore::IDataStore* compiledStageDataStoreForTest()
{
    return testDataStore.load();
}
bool RuntimeArtifactService::localOnly(uint32_t features)
{
    using Features = carb::datastore::DataStoreConfigurator::Features;
    return (features & Features::Local) && !(features & (Features::OmniHub | Features::HighSpeed | Features::Network));
}
std::unique_ptr<RuntimeArtifactService> RuntimeArtifactService::create(Mode mode)
{
    if (mode == Mode::eDisabled)
        return nullptr;
    auto* framework = carb::getFramework();
    auto* keys = framework ? framework->tryAcquireInterface<omni::blobkey::IBlobKey>() : nullptr;
    if (!keys)
        return nullptr;
    omni::core::ObjectPtr<carb::datastore::IDataStore> data;
    if (carb::datastore::IDataStore* test = compiledStageDataStoreForTest())
    {
        // A test datastore passes the same local-only rule, with the features its
        // test reports for it.
        if (!localOnly(testDataStoreFeatures.load()))
            return nullptr;
        data = omni::core::borrow(test);
    }
    else
    {
        carb::ujitso::DefaultState* state = carb::ujitso::getDefaultState();
        carb::datastore::DataStoreConfigurator* configurator = state ? state->dataStoreConfigurator : nullptr;
        if (!configurator || !localOnly(configurator->getFeatures()))
            return nullptr;
        // The reference keeps this datastore alive if UJITSO replaces it.
        data = omni::core::borrow(configurator->getDataStore());
    }
    if (!data)
        return nullptr;
    return std::unique_ptr<RuntimeArtifactService>(new RuntimeArtifactService(std::move(data), *keys, mode));
}
} // namespace omni::physx::cache

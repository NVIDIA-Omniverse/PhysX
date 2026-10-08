// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-5
 *
 * @implements REQ-PARSE-JOINT-005
 * @covers AC-1 AC-2
 */

#pragma once

#include "Allocator.h"
#include "Handles.h"
#include "Descriptors.h"
#include "IPhysicsSource.h"
#include "KnownTokens.h"

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::parse
{

// ---------------------------------------------------------------------------
// ObjectIdMap — multimap of ObjectCategory → ObjectId for a single key.
// USD-free counterpart of the consumer-side ObjectIdMap.
// ---------------------------------------------------------------------------

using ObjectIdMap = std::multimap<ObjectCategory, ObjectId>;

// ---------------------------------------------------------------------------
// SchemaAPIFlag — bitfield for schema API presence on a prim.
// ---------------------------------------------------------------------------

struct SchemaAPIFlag
{
    enum Enum : uint64_t
    {
        eRigidBodyAPI = (1 << 0),
        eCollisionAPI = (1 << 1),
        eParticleIsosurfaceAPI = (1 << 2),
        eDiffuseParticlesAPI = (1 << 3),
        eParticleSetAPI = (1 << 4),
        eParticleAnisotropyAPI = (1 << 5),
        eParticleSmoothingAPI = (1 << 6),
        ePhysxForceAPI = (1 << 7),
        eFilteredPairsAPI = (1 << 8),
        eMimicJointRotXAPI = (1 << 9),
        eMimicJointRotYAPI = (1 << 10),
        eMimicJointRotZAPI = (1 << 11),
        eContactReportAPI = (1 << 12),
        eDrivePerformanceEnvelopeAngularAPI = (1 << 13),
        eDrivePerformanceEnvelopeLinearAPI = (1 << 14),
        eDrivePerformanceEnvelopeRotXAPI = (1 << 15),
        eDrivePerformanceEnvelopeRotYAPI = (1 << 16),
        eDrivePerformanceEnvelopeRotZAPI = (1 << 17),
        eJointAxisAngularAPI = (1 << 18),
        eJointAxisLinearAPI = (1 << 19),
        eJointAxisRotXAPI = (1 << 20),
        eJointAxisRotYAPI = (1 << 21),
        eJointAxisRotZAPI = (1 << 22),
        eDeformableBodyAPI = (1 << 23),
        eVolumeDeformableSimAPI = (1 << 24),
        eSurfaceDeformableSimAPI = (1 << 25),
        eDeformablePoseAPI = (1 << 26),
        eAutoDeformableBodyAPI = (1 << 27),
        eAutoDeformableHexahedralMeshAPI = (1 << 28),
        eAutoDeformableMeshSimplificationAPI = (1 << 29),
        eNewtonMimicAPI = (1 << 30),
    };
};

// ---------------------------------------------------------------------------
// ObjectDatabase — USD-free equivalent of ObjectDb.
// Maps ObjectKey → ObjectIdMap (category → ObjectId) and tracks schema APIs.
// ---------------------------------------------------------------------------

class ObjectDatabase
{
public:
    using Map = std::unordered_map<ObjectKey, ObjectIdMap, ObjectKey::Hash>;
    using SchemaApiMap = std::unordered_map<ObjectKey, uint64_t, ObjectKey::Hash>;

    void findOrCreateEntry(ObjectKey key, ObjectCategory category, ObjectId id);
    const ObjectIdMap* getEntries(ObjectKey key) const;
    ObjectIdMap* getEntries(ObjectKey key);
    ObjectId findEntry(ObjectKey key, ObjectCategory category) const;
    bool removeEntries(ObjectKey key);
    void removeEntry(ObjectKey key, ObjectCategory category, ObjectId id);
    bool empty() const { return mKeyMap.empty(); }

    void addSchemaAPI(ObjectKey key, SchemaAPIFlag::Enum api);
    void setSchemaAPI(ObjectKey key, uint64_t flags);
    void removeSchemaAPIs(ObjectKey key);
    void removeSchemaAPI(ObjectKey key, SchemaAPIFlag::Enum api);
    uint64_t getSchemaAPIs(ObjectKey key) const;

    const Map& map() const { return mKeyMap; }

private:
    Map mKeyMap;
    SchemaApiMap mSchemaAPIMap;
};

// ---------------------------------------------------------------------------
// Parse-time container typedefs (ObjectKey-based, USD-free)
// ---------------------------------------------------------------------------

using CollisionGroupsMap = std::unordered_map<ObjectKey, std::vector<ObjectKey>, ObjectKey::Hash>;

// Scalar fields resolved from a particle system that downstream particle
// sets / samplers copy (scene owner + autocompleted rest offsets). Cached per
// system ObjectKey so the owning system is parsed at most once per pass rather
// than re-read O(sets + 2*samplers) times.
struct ParticleSystemResolved
{
    ObjectKey sceneKey;
    float solidRestOffset = 0.0f;
    float fluidRestOffset = 0.0f;
    float particleContactOffset = 0.0f;
};
using ParticleSystemResolvedCache = std::unordered_map<ObjectKey, ParticleSystemResolved, ObjectKey::Hash>;
using DeformableAttachmentHistoryMap = std::unordered_multimap<ObjectKey, ObjectKey, ObjectKey::Hash>;
using DeformableCollisionFilterHistoryMap = std::unordered_multimap<ObjectKey, ObjectKey, ObjectKey::Hash>;
using EnvIdMap = std::unordered_map<TokenId, uint32_t, TokenId::Hash>;

// ---------------------------------------------------------------------------
// JointSchemaPresence: stage-wide "does any prim apply this joint sub-schema" flags
// (exact schema membership). A backend that knows the answer sets them so parseJoint
// skips the per-joint hasSchema probe; all default to true (USD/native unchanged).
// ---------------------------------------------------------------------------

struct JointSchemaPresence
{
    bool physxLimit = true;         // PhysxLimitAPI:<instance>
    bool physxJointAxis = true;     // PhysxJointAxisAPI:<instance>
    bool physxJointApi = true;      // PhysxJointAPI (concrete)
    bool drivePerfEnvelope = true;  // PhysxDrivePerformanceEnvelopeAPI:<instance>
    bool jointState = true;         // PhysicsJointStateAPI:<instance>
    bool physicsDrive = true;       // PhysicsDriveAPI:<instance> (walker-side)
    bool physxDistanceJoint = true; // PhysxPhysicsDistanceJointAPI (concrete)
};

// ---------------------------------------------------------------------------
// ParseContext — parse-time state container.
// Owns the ObjectDatabase, collision group maps, deformable history maps,
// and environment-ID tables. All keyed by ObjectKey/TokenId (USD-free).
// ---------------------------------------------------------------------------

class ParseContext
{
public:
    explicit ParseContext(IPhysicsSource& source, IDescriptorAllocator& allocator);
    ~ParseContext();

    IPhysicsSource& source() { return mSource; }
    const IPhysicsSource& source() const { return mSource; }
    SourceUnits units() const { return mUnits; }

    // Lazily resolved because some short-lived contexts never parse a concept
    // that consumes the well-known vocabulary. References the source's own batch
    // when IPhysicsSource::knownTokens() provides one (no copy, no intern);
    // otherwise interns once into mKnownTokens (REQ-LOAD-TOKENS-001 AC-5).
    const KnownTokens& knownTokens();

    // Seed from a batch already interned for the same source (e.g. the attach-scoped
    // AttachedStage::getKnownTokens()) so a per-object short-lived context does not
    // re-intern the whole vocabulary (REQ-LOAD-TOKENS-001). When the source caches
    // its own batch that one is referenced instead -- same source, same ids -- so
    // there is exactly one KnownTokens instance per such source.
    void adoptKnownTokens(const KnownTokens& tok)
    {
        if (const KnownTokens* cached = mSource.knownTokens())
        {
            mKnownTokensRef = cached;
            return;
        }
        mKnownTokens = tok;
        mKnownTokensRef = &mKnownTokens;
    }

    // Allocator the parse-lib `parseX` entry points (and the USD walker)
    // use to mint descriptor allocations. Supplied by the consumer at
    // ctor time.
    IDescriptorAllocator& descriptorAllocator() const { return mAllocator; }

    // Default value for `localSpaceVelocities` on newly-parsed dynamic bodies.
    // Comes from the consumer's global setting (e.g. omni.physx's
    // /physics/outputVelocitiesLocalSpace). Per-prim metadata overrides this.
    bool outputVelocitiesLocalSpaceDefault() const { return mOutputVelocitiesLocalSpaceDefault; }
    void setOutputVelocitiesLocalSpaceDefault(bool v) { mOutputVelocitiesLocalSpaceDefault = v; }

    // --- Joint sub-schema stage presence (perf gate; see JointSchemaPresence) ---
    JointSchemaPresence& jointSchemaPresence() { return mJointSchemaPresence; }
    const JointSchemaPresence& jointSchemaPresence() const { return mJointSchemaPresence; }

    // --- Object database ---
    ObjectDatabase& objects() { return mObjects; }
    const ObjectDatabase& objects() const { return mObjects; }

    // --- Collision groups ---
    CollisionGroupsMap& collisionGroups() { return mCollisionGroupsMap; }
    const CollisionGroupsMap& collisionGroups() const { return mCollisionGroupsMap; }

    std::vector<CollisionGroupsMap>& additionalCollisionGroups() { return mAdditionalCollisionGroupMaps; }
    const std::vector<CollisionGroupsMap>& additionalCollisionGroups() const { return mAdditionalCollisionGroupMaps; }

    // --- Deformable history ---
    DeformableAttachmentHistoryMap& deformableAttachmentHistory() { return mDeformableAttachmentHistoryMap; }
    const DeformableAttachmentHistoryMap& deformableAttachmentHistory() const { return mDeformableAttachmentHistoryMap; }

    DeformableCollisionFilterHistoryMap& deformableCollisionFilterHistory() { return mDeformableCollisionFilterHistoryMap; }
    const DeformableCollisionFilterHistoryMap& deformableCollisionFilterHistory() const { return mDeformableCollisionFilterHistoryMap; }

    // --- Particle system memoization ---
    ParticleSystemResolvedCache& particleSystemCache() { return mParticleSystemCache; }

    // --- Environment IDs ---
    uint32_t registerEnvId(TokenId token);
    uint32_t getEnvId(TokenId token) const;
    uint32_t envIdCounter() const { return mEnvIdCounter; }

    // --- Buffer access ---
    // Resolve a BufferHandle to a typed zero-copy span. The source backend
    // owns the underlying memory; the span is valid as long as the source
    // hasn't released the buffer (typically the lifetime of the parse run).
    // No type checking is performed against handle.type — the caller chooses
    // the element type based on the attribute they're reading.
    template <typename T>
    BufferSpan<T> getBuffer(BufferHandle h) const
    {
        if (!h.valid())
            return {};
        size_t byteCount = 0;
        const void* data = mSource.resolveBuffer(h, byteCount);
        if (!data || byteCount == 0)
            return {};
        return { static_cast<const T*>(data), byteCount / sizeof(T) };
    }

private:
    IPhysicsSource& mSource;
    IDescriptorAllocator& mAllocator;
    SourceUnits mUnits;
    ObjectDatabase mObjects;
    CollisionGroupsMap mCollisionGroupsMap;
    std::vector<CollisionGroupsMap> mAdditionalCollisionGroupMaps;
    DeformableAttachmentHistoryMap mDeformableAttachmentHistoryMap;
    DeformableCollisionFilterHistoryMap mDeformableCollisionFilterHistoryMap;
    ParticleSystemResolvedCache mParticleSystemCache;
    uint32_t mEnvIdCounter = 0;
    EnvIdMap mEnvIdMap;
    bool mOutputVelocitiesLocalSpaceDefault = false;
    JointSchemaPresence mJointSchemaPresence;
    // Fallback storage, used only for sources whose knownTokens() is nullptr.
    KnownTokens mKnownTokens;
    // The resolved batch: mSource.knownTokens() or &mKnownTokens; nullptr until first use.
    // A pointer into the source is safe: the source outlives every ParseContext built on
    // it (mSource is a reference) and its batch lives as long as the source.
    const KnownTokens* mKnownTokensRef = nullptr;
};

} // namespace omni::physics::parse

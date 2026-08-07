// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-4
 */

#pragma once

#include <omni/physics/parse/IPhysicsSource.h>

#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace omni::physics::parse
{

// ---------------------------------------------------------------------------
// ProceduralSource — USD-free IPhysicsSource for programmatic scene creation.
//
// Proves the parsing library abstraction: create physics scenes entirely in
// code, parse them through the same API that handles USD stages.
//
// Unlike MockSource (test-only), ProceduralSource is a full backend suitable
// for integration testing and procedural physics scene generation.
// ---------------------------------------------------------------------------

class ProceduralSource final : public IPhysicsSource
{
public:
    ProceduralSource();
    ~ProceduralSource() override;

    // --- Scene construction API ---

    struct PrimBuilder
    {
        ObjectKey key;
        ProceduralSource& source;

        PrimBuilder& schema(const std::string& schemaName);
        PrimBuilder& attr(const std::string& name, AttrValue value);
        PrimBuilder& rel(const std::string& name, ObjectKey target);
    };

    PrimBuilder addPrim(const std::string& path);
    void setSceneUnits(float metersPerUnit, UpAxis upAxis);

    // --- IPhysicsSource overrides ---

    std::string_view sourceKeyToString(ObjectKey key) const override;
    TokenId internToken(std::string_view token) const override;
    std::string_view tokenToString(TokenId id) const override;

    ObjectKey getRootKey() const override;
    void forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const override;
    ObjectKey findByPath(std::string_view path) const override;

    bool hasSchema(ObjectKey key, TokenId schemaToken) const override;
    AttrValue getAttribute(ObjectKey key, TokenId attr) const override;
    using IPhysicsSource::getAttribute;  // bring in typed overloads
    bool hasAuthoredAttribute(ObjectKey key, TokenId attr) const override;
    bool isAttributeTimeSampled(ObjectKey key, TokenId attr) const override;
    void getLocalToWorldTransform(ObjectKey key, Matrix4d& outMatrix) const override;
    void getLocalToWorldRotationAndScale(ObjectKey key,
                                         Matrix3d& outRotation,
                                         carb::Float3& outScale) const override;
    ObjectKey getParent(ObjectKey key) const override;
    void getRelationshipTargets(ObjectKey key, TokenId rel, std::vector<ObjectKey>& out) const override;

    const void* resolveBuffer(BufferHandle handle, size_t& byteCount) const override;
    MeshGeometry getMeshAttributes(ObjectKey key) const override;
    SourceUnits getSourceUnits() const override;
    void resolveCollection(ObjectKey primKey,
                           TokenId collectionName,
                           std::vector<ObjectKey>& members) const override;

    void forEachMultiApplyInstance(
        ObjectKey key,
        std::string_view baseSchema,
        std::function<void(std::string_view instance)> cb) const override;

    ObjectKey getMaterialBinding(ObjectKey primKey) const override;

    // Setters used by procedurally-built scenes.
    void setMetadata(ObjectKey key, const std::string& metaName, AttrValue value);
    void setLocalToWorldTransform(ObjectKey key, const Matrix4d& m);

private:
    struct PrimData
    {
        std::string path;
        ObjectKey parent;
        std::vector<ObjectKey> children;
        std::vector<TokenId> schemas;
        std::map<uint32_t, AttrValue> attributes;
        std::map<uint32_t, std::vector<ObjectKey>> relationships;
        std::map<uint32_t, AttrValue> metadata;
        Matrix4d localToWorld;
    };

    std::vector<PrimData> mPrims;
    std::unordered_map<std::string, ObjectKey> mPathToKey;

    mutable std::unordered_map<std::string, TokenId> mStringToToken;
    mutable std::vector<std::string> mTokenStrings;

    SourceUnits mUnits{1.0f, 1.0f, UpAxis::eZ};

    TokenId doInternToken(std::string_view token) const;
    ObjectKey findParent(const std::string& path) const;

    friend struct PrimBuilder;
};

} // namespace omni::physics::parse

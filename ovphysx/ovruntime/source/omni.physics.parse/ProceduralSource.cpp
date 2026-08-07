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

#include "ProceduralSource.h"

#include <algorithm>

namespace omni::physics::parse
{

ProceduralSource::ProceduralSource()
{
    mTokenStrings.emplace_back();

    PrimData root;
    root.path = "/";
    root.parent = {};
    mPrims.push_back(std::move(root));
    mPathToKey["/"] = ObjectKey{1};
}

ProceduralSource::~ProceduralSource() = default;

// ---------------------------------------------------------------------------
// Scene construction
// ---------------------------------------------------------------------------

ProceduralSource::PrimBuilder ProceduralSource::addPrim(const std::string& path)
{
    auto it = mPathToKey.find(path);
    if (it != mPathToKey.end())
        return PrimBuilder{it->second, *this};

    ObjectKey key{static_cast<uint64_t>(mPrims.size() + 1)};
    PrimData prim;
    prim.path = path;

    ObjectKey parentKey = findParent(path);
    prim.parent = parentKey;
    if (parentKey.handle > 0 && parentKey.handle <= mPrims.size())
        mPrims[parentKey.handle - 1].children.push_back(key);

    mPrims.push_back(std::move(prim));
    mPathToKey[path] = key;
    return PrimBuilder{key, *this};
}

void ProceduralSource::setSceneUnits(float metersPerUnit, UpAxis upAxis)
{
    mUnits.metersPerUnit = metersPerUnit;
    mUnits.upAxis = upAxis;
}

ProceduralSource::PrimBuilder& ProceduralSource::PrimBuilder::schema(const std::string& schemaName)
{
    TokenId tok = source.doInternToken(schemaName);
    auto& schemas = source.mPrims[key.handle - 1].schemas;
    if (std::find(schemas.begin(), schemas.end(), tok) == schemas.end())
        schemas.push_back(tok);
    return *this;
}

ProceduralSource::PrimBuilder& ProceduralSource::PrimBuilder::attr(const std::string& name, AttrValue value)
{
    TokenId tok = source.doInternToken(name);
    source.mPrims[key.handle - 1].attributes[tok.id] = std::move(value);
    return *this;
}

ProceduralSource::PrimBuilder& ProceduralSource::PrimBuilder::rel(const std::string& name, ObjectKey target)
{
    TokenId tok = source.doInternToken(name);
    source.mPrims[key.handle - 1].relationships[tok.id].push_back(target);
    return *this;
}

// ---------------------------------------------------------------------------
// IPhysicsSource implementation
// ---------------------------------------------------------------------------

std::string_view ProceduralSource::sourceKeyToString(ObjectKey key) const
{
    if (key.handle == 0 || key.handle > mPrims.size())
        return {};
    return mPrims[key.handle - 1].path;
}

TokenId ProceduralSource::internToken(std::string_view token) const
{
    return doInternToken(token);
}

std::string_view ProceduralSource::tokenToString(TokenId id) const
{
    if (id.id == 0 || id.id >= mTokenStrings.size())
        return {};
    return mTokenStrings[id.id];
}

ObjectKey ProceduralSource::getRootKey() const
{
    return ObjectKey{1};
}

void ProceduralSource::forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const
{
    if (parent.handle == 0 || parent.handle > mPrims.size())
        return;
    for (ObjectKey child : mPrims[parent.handle - 1].children)
        cb(child);
}

ObjectKey ProceduralSource::findByPath(std::string_view path) const
{
    auto it = mPathToKey.find(std::string{path});
    if (it != mPathToKey.end())
        return it->second;
    return {};
}

bool ProceduralSource::hasSchema(ObjectKey key, TokenId schemaToken) const
{
    if (key.handle == 0 || key.handle > mPrims.size())
        return false;
    const auto& schemas = mPrims[key.handle - 1].schemas;
    return std::find(schemas.begin(), schemas.end(), schemaToken) != schemas.end();
}

AttrValue ProceduralSource::getAttribute(ObjectKey key, TokenId attr) const
{
    if (key.handle == 0 || key.handle > mPrims.size())
        return {};
    const auto& prim = mPrims[key.handle - 1];
    auto ait = prim.attributes.find(attr.id);
    if (ait != prim.attributes.end())
        return ait->second;
    // Metadata fallback — mirrors the UsdSource customData fallback so
    // callers don't need to know which kind of storage a value lives
    // in. `setMetadata` is the builder-side way to populate this map.
    auto mit = prim.metadata.find(attr.id);
    if (mit != prim.metadata.end())
        return mit->second;
    return {};
}

void ProceduralSource::getRelationshipTargets(ObjectKey key, TokenId rel, std::vector<ObjectKey>& out) const
{
    if (key.handle == 0 || key.handle > mPrims.size())
        return;
    const auto& rels = mPrims[key.handle - 1].relationships;
    auto it = rels.find(rel.id);
    if (it != rels.end())
        out.insert(out.end(), it->second.begin(), it->second.end());
}

bool ProceduralSource::isAttributeTimeSampled(ObjectKey, TokenId) const
{
    // Procedurally-built scenes are static by construction — no time samples.
    return false;
}

bool ProceduralSource::hasAuthoredAttribute(ObjectKey key, TokenId attr) const
{
    // Procedural sources have no schema-fallback resolution; an attribute
    // present in the table is "authored" by the builder.
    if (key.handle == 0 || key.handle > mPrims.size())
        return false;
    const auto& attrs = mPrims[key.handle - 1].attributes;
    return attrs.find(attr.id) != attrs.end();
}

void ProceduralSource::getLocalToWorldTransform(ObjectKey key, Matrix4d& outMatrix) const
{
    outMatrix = Matrix4d{};
    if (key.handle == 0 || key.handle > mPrims.size())
        return;
    outMatrix = mPrims[key.handle - 1].localToWorld;
}

void ProceduralSource::setMetadata(ObjectKey key, const std::string& metaName, AttrValue value)
{
    if (key.handle == 0 || key.handle > mPrims.size())
        return;
    TokenId tok = doInternToken(metaName);
    mPrims[key.handle - 1].metadata[tok.id] = std::move(value);
}

void ProceduralSource::setLocalToWorldTransform(ObjectKey key, const Matrix4d& m)
{
    if (key.handle == 0 || key.handle > mPrims.size())
        return;
    mPrims[key.handle - 1].localToWorld = m;
}

void ProceduralSource::getLocalToWorldRotationAndScale(ObjectKey /*key*/, Matrix3d& outRotation, carb::Float3& outScale) const
{
    // Procedural scenes don't yet model affine decomposition; return identity.
    outRotation = Matrix3d{};
    outScale = { 1.0f, 1.0f, 1.0f };
}

ObjectKey ProceduralSource::getParent(ObjectKey key) const
{
    if (key.handle == 0 || key.handle > mPrims.size())
        return {};
    return mPrims[key.handle - 1].parent;
}

const void* ProceduralSource::resolveBuffer(BufferHandle, size_t& byteCount) const
{
    byteCount = 0;
    return nullptr;
}

MeshGeometry ProceduralSource::getMeshAttributes(ObjectKey) const
{
    // ProceduralSource has no mesh data wired up yet; return empty geometry.
    return {};
}

SourceUnits ProceduralSource::getSourceUnits() const
{
    return mUnits;
}

void ProceduralSource::resolveCollection(ObjectKey, TokenId, std::vector<ObjectKey>&) const
{
}

void ProceduralSource::forEachMultiApplyInstance(
    ObjectKey, std::string_view, std::function<void(std::string_view)>) const
{
    // ProceduralSource doesn't model multi-apply schemas. An explicit
    // helper to register multi-apply instances would be a separate
    // setter; for now, no instances reported.
}

ObjectKey ProceduralSource::getMaterialBinding(ObjectKey) const
{
    // Procedural sources don't model UsdShade material bindings. An
    // explicit "material on this prim" relationship would be a separate
    // helper API; for now, return invalid (no binding).
    return {};
}

// ---------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------

TokenId ProceduralSource::doInternToken(std::string_view token) const
{
    std::string key{token};
    auto it = mStringToToken.find(key);
    if (it != mStringToToken.end())
        return it->second;

    TokenId id{static_cast<uint32_t>(mTokenStrings.size())};
    mTokenStrings.push_back(key);
    mStringToToken[key] = id;
    return id;
}

ObjectKey ProceduralSource::findParent(const std::string& path) const
{
    if (path == "/" || path.empty())
        return {};

    size_t lastSlash = path.rfind('/');
    if (lastSlash == std::string::npos)
        return ObjectKey{1};

    std::string parentKey = (lastSlash == 0) ? "/" : path.substr(0, lastSlash);
    auto it = mPathToKey.find(parentKey);
    if (it != mPathToKey.end())
        return it->second;
    return {};
}

} // namespace omni::physics::parse

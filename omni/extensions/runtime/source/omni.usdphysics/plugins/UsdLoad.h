// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <vector>
#include <map>

#include <private/omni/physics/schema/DescCache.h>

namespace omni
{
namespace physics
{
namespace schema
{

using SceneMap = pxr::TfHashMap<pxr::SdfPath, SceneDesc*, pxr::SdfPath::Hash>;
using CollisionGroupMap = pxr::TfHashMap<pxr::SdfPath, CollisionGroupDesc*, pxr::SdfPath::Hash>;
using MaterialMap = pxr::TfHashMap<pxr::SdfPath, ObjectDesc*, pxr::SdfPath::Hash>;
using ShapeMap = std::vector<std::pair<pxr::SdfPath, ShapeDesc*>>;
using BodyMap = std::map<pxr::SdfPath, ObjectDesc*>;
using JointMap = std::map<pxr::SdfPath, JointDesc*>;
using ArticulationMap = std::map<pxr::SdfPath, ArticulationDesc*>;
using AttachmentMap = std::map<pxr::SdfPath, AttachmentDesc*>;
using CollisionFilterMap = std::map<pxr::SdfPath, ElementCollisionFilterDesc*>;
using TokenVector = std::vector<pxr::TfToken>;

class IUsdPhysicsListener;
class PrimIteratorBase;

class PhysicsSchemaUsdLoad
{
public:
    PhysicsSchemaUsdLoad();

    ~PhysicsSchemaUsdLoad();

    bool loadFromRange(const pxr::UsdStageWeakPtr stage,
                       pxr::UsdGeomXformCache& xfCache,
                       PrimIteratorBase& range,
                       bool reset = true);

    void registerPhysicsListener(IUsdPhysicsListener* listener);

    void unregisterPhysicsListener(IUsdPhysicsListener* listener);

    void addCustomShapeToken(const pxr::TfToken& token)
    {
        mCustomShapeTokens.push_back(token);
    }

    void addCustomJointToken(const pxr::TfToken& token)
    {
        mCustomJointTokens.push_back(token);
    }

    void removeCustomJointToken(const pxr::TfToken& token)
    {
        for (size_t i = 0; i < mCustomJointTokens.size(); i++)
        {
            const pxr::TfToken& t = mCustomJointTokens[i];
            if (t == token)
            {
                mCustomJointTokens[i] = mCustomJointTokens.back();
                mCustomJointTokens.pop_back();
                return;
            }
        }
    }

    void addCustomPhysicsInstancerToken(const pxr::TfToken& token)
    {
        mCustomPhysicsInstancerTokens.push_back(token);
    }

    void removeCustomPhysicsInstancerToken(const pxr::TfToken& token)
    {
        for (size_t i = 0; i < mCustomPhysicsInstancerTokens.size(); i++)
        {
            const pxr::TfToken& t = mCustomPhysicsInstancerTokens[i];
            if (t == token)
            {
                mCustomPhysicsInstancerTokens[i] = mCustomPhysicsInstancerTokens.back();
                mCustomPhysicsInstancerTokens.pop_back();
                return;
            }
        }
    }

    void removeCustomShapeToken(const pxr::TfToken& token)
    {
        for (size_t i = 0; i < mCustomShapeTokens.size(); i++)
        {
            const pxr::TfToken& t = mCustomShapeTokens[i];
            if (t == token)
            {
                mCustomShapeTokens[i] = mCustomShapeTokens.back();
                mCustomShapeTokens.pop_back();
                return;
            }
        }
    }

private:
    void reportPrimDesc(const pxr::UsdPrim& prim,
                        ObjectDesc* desc,
                        uint64_t typeFlags,
                        const pxr::TfTokenVector& appliedApis);
    void reportObjectDesc(const pxr::SdfPath& path, const ObjectDesc* desc);

private:
    std::vector<IUsdPhysicsListener*> mListeners;

    TokenVector mCustomShapeTokens;
    TokenVector mCustomJointTokens;
    TokenVector mCustomPhysicsInstancerTokens;

    DescCache mDescCache;
};

} // namespace schema
} // namespace physics
} // namespace omni

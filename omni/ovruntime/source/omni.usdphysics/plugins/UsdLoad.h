// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

using SceneMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, SceneDesc*, PXR_NS::SdfPath::Hash>;
using CollisionGroupMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, CollisionGroupDesc*, PXR_NS::SdfPath::Hash>;
using MaterialMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, ObjectDesc*, PXR_NS::SdfPath::Hash>;
using ShapeMap = std::vector<std::pair<PXR_NS::SdfPath, ShapeDesc*>>;
using BodyMap = std::map<PXR_NS::SdfPath, ObjectDesc*>;
using JointMap = std::map<PXR_NS::SdfPath, JointDesc*>;
using ArticulationMap = std::map<PXR_NS::SdfPath, ArticulationDesc*>;
using AttachmentMap = std::map<PXR_NS::SdfPath, AttachmentDesc*>;
using CollisionFilterMap = std::map<PXR_NS::SdfPath, ElementCollisionFilterDesc*>;
using TokenVector = std::vector<PXR_NS::TfToken>;

class IUsdPhysicsListener;
class PrimIteratorBase;

class PhysicsSchemaUsdLoad
{
public:
    PhysicsSchemaUsdLoad();

    ~PhysicsSchemaUsdLoad();

    bool loadFromRange(const PXR_NS::UsdStageWeakPtr stage,
                       PXR_NS::UsdGeomXformCache& xfCache,
                       PrimIteratorBase& range,
                       bool reset = true);

    void registerPhysicsListener(IUsdPhysicsListener* listener);

    void unregisterPhysicsListener(IUsdPhysicsListener* listener);

    void addCustomShapeToken(const PXR_NS::TfToken& token)
    {
        mCustomShapeTokens.push_back(token);
    }

    void addCustomJointToken(const PXR_NS::TfToken& token)
    {
        mCustomJointTokens.push_back(token);
    }

    void removeCustomJointToken(const PXR_NS::TfToken& token)
    {
        for (size_t i = 0; i < mCustomJointTokens.size(); i++)
        {
            const PXR_NS::TfToken& t = mCustomJointTokens[i];
            if (t == token)
            {
                mCustomJointTokens[i] = mCustomJointTokens.back();
                mCustomJointTokens.pop_back();
                return;
            }
        }
    }

    void addCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
    {
        mCustomPhysicsInstancerTokens.push_back(token);
    }

    void removeCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
    {
        for (size_t i = 0; i < mCustomPhysicsInstancerTokens.size(); i++)
        {
            const PXR_NS::TfToken& t = mCustomPhysicsInstancerTokens[i];
            if (t == token)
            {
                mCustomPhysicsInstancerTokens[i] = mCustomPhysicsInstancerTokens.back();
                mCustomPhysicsInstancerTokens.pop_back();
                return;
            }
        }
    }

    void removeCustomShapeToken(const PXR_NS::TfToken& token)
    {
        for (size_t i = 0; i < mCustomShapeTokens.size(); i++)
        {
            const PXR_NS::TfToken& t = mCustomShapeTokens[i];
            if (t == token)
            {
                mCustomShapeTokens[i] = mCustomShapeTokens.back();
                mCustomShapeTokens.pop_back();
                return;
            }
        }
    }

private:
    void reportPrimDesc(const PXR_NS::UsdPrim& prim,
                        ObjectDesc* desc,
                        uint64_t typeFlags,
                        const PXR_NS::TfTokenVector& appliedApis);
    void reportObjectDesc(const PXR_NS::SdfPath& path, const ObjectDesc* desc);

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

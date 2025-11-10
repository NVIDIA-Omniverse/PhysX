// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
#include "BindingsImpl.h"

#include <stack>
#include <carb/profiler/Profile.h>

using namespace pxr;

static const TfType& rigidBodyAPIType = TfType::Find<UsdPhysicsRigidBodyAPI>();

bool descendantHasAPI_WRet(const TfType& name, const UsdPrim& prim, UsdPrim& ret, bool check_prim = true)
{
    CARB_PROFILE_ZONE(carb::profiler::kCaptureMaskDefault, "descendantHasAPI");

    if (check_prim && prim.HasAPI(name))
    {
        ret = prim;
        return true;
    }

    auto descendants = prim.GetDescendants();

    if (name == rigidBodyAPIType)
    {
        for (const auto& d : descendants)
        {
            if (UsdGeomXformable(d).GetResetXformStack())
            {
                return false;
            }

            if (d.HasAPI(name))
            {
                ret = d;
                return true;
            }
        }
    }
    else
    {
        for (const auto& d : descendants)
        {
            if (d.HasAPI(name))
            {
                ret = d;
                return true;
            }
        }
    }

    return false;
}

bool descendantHasAPI(const TfType& name, const UsdPrim& prim)
{
    UsdPrim ret;
    return descendantHasAPI_WRet(name, prim, ret);
}

bool ancestorHasAPI_RB(const TfType& name, const UsdPrim& prim, UsdPrim& ret, bool check_prim = true)
{
    if (check_prim && prim.HasAPI(name))
    {
        ret = prim;
        return true;
    }

    UsdPrim parent = prim.GetParent();
    if (parent.IsValid())
    {
        if (UsdGeomXformable(prim).GetResetXformStack())
        {
            return false;
        }
        return ancestorHasAPI_RB(name, parent, ret);
    }

    return false;
}

bool ancestorHasAPI_Others(const TfType& name, const UsdPrim& prim, UsdPrim& ret, bool check_prim = true)
{
    if (check_prim && prim.HasAPI(name))
    {
        ret = prim;
        return true;
    }

    UsdPrim parent = prim.GetParent();
    if (parent.IsValid())
    {
        return ancestorHasAPI_Others(name, parent, ret);
    }

    return false;
}

bool ancestorHasAPI_WRet(const TfType& name, const UsdPrim& prim, UsdPrim& ret, bool check_prim = true)
{
    if (name == rigidBodyAPIType)
    {
        return ancestorHasAPI_RB(name, prim, ret, check_prim);
    }
    else
    {
        return ancestorHasAPI_Others(name, prim, ret, check_prim);
    }
}

bool ancestorHasAPI(const TfType& name, const UsdPrim& prim)
{
    UsdPrim ret;
    return ancestorHasAPI_WRet(name, prim, ret);
}

bool do_check_itself(const TfType& self_api, const UsdPrim& prim, UsdPrim& ret, bool check_prim = true)
{
    if (check_prim)
    {
        if (prim.HasAPI(self_api))
        {
            ret = prim;
            return true;
        }
    }

    if (ancestorHasAPI_WRet(self_api, prim, ret, check_prim) || descendantHasAPI_WRet(self_api, prim, ret, check_prim))
    {
        return true;
    }

    return false;
}

template <std::size_t N>
bool hasconflictingapis(const TfType itselfType, const std::array<TfType, N>& conflict_dict,
                               const UsdPrim& prim, UsdPrim& ret,
                               bool check_itself = false, bool check_prim = true)
{
    if (check_itself)
    {
        if (do_check_itself(itselfType, prim, ret, check_prim))
        {
            return true;
        }
    }

    for (auto& c : conflict_dict)
    {
        if (ancestorHasAPI_WRet(c, prim, ret) || descendantHasAPI_WRet(c, prim, ret))
        {
            return true;
        }
    }

    return false;
}

bool hasconflictingapis_RigidBodyAPI_WRet(const UsdPrim& prim, UsdPrim& ret, bool check_itself, bool check_prim)
{
    static std::array<TfType, 7> conflict_dict = {
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->BaseDeformableBodyAPI),
        TfType::Find<PhysxSchemaPhysxDeformableAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
        TfType::Find<PhysxSchemaPhysxCharacterControllerAPI>(),
    };

    static const TfType rbType = TfType::Find<UsdPhysicsRigidBodyAPI>();
    return hasconflictingapis<conflict_dict.size()>(rbType, conflict_dict, prim, ret, check_itself, check_prim);
}

bool hasconflictingapis_RigidBodyAPI(const UsdPrim& prim, bool check_itself/* = false*/)
{
    UsdPrim ret;
    return hasconflictingapis_RigidBodyAPI_WRet(prim, ret, check_itself, true);
}

bool hasconflictingapis_CollisionAPI_WRet(const UsdPrim& prim, UsdPrim& ret, bool check_itself, bool check_prim)
{
    static std::array<TfType, 6> conflict_dict = {
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->BaseDeformableBodyAPI),
        TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
    };

    static const TfType cType = TfType::Find<UsdPhysicsCollisionAPI>();
    return hasconflictingapis<conflict_dict.size()>(cType, conflict_dict, prim, ret, check_itself, check_prim);
}

bool hasconflictingapis_CollisionAPI(const UsdPrim& prim, bool check_itself/* = false*/)
{
    UsdPrim ret;
    return hasconflictingapis_CollisionAPI_WRet(prim, ret, check_itself, true);
}

bool hasconflictingapis_ArticulationRoot_WRet(const UsdPrim& prim, UsdPrim& ret, bool check_itself, bool check_prim)
{
    static std::array<TfType, 6> conflict_dict = {
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->BaseDeformableBodyAPI),
        TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
    };

    static const TfType arType = TfType::Find<UsdPhysicsArticulationRootAPI>();
    return hasconflictingapis<conflict_dict.size()>(arType, conflict_dict, prim, ret, check_itself, check_prim);
}

bool hasconflictingapis_ArticulationRoot(const UsdPrim& prim, bool check_itself/* = false*/)
{
    UsdPrim ret;
    return hasconflictingapis_ArticulationRoot_WRet(prim, ret, check_itself, true);
}

bool hasconflictingapis_PhysxDeformableBodyAPI_deprecated(const UsdPrim& prim, bool check_itself/* = false*/)
{
    UsdPrim ret;
    static std::array<TfType, 8> conflict_dict = {
        TfType::Find<UsdPhysicsRigidBodyAPI>(),
        TfType::Find<UsdPhysicsCollisionAPI>(),
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
        TfType::Find<UsdPhysicsArticulationRootAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>(),
    };

    static const TfType dbType = TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>();
    return hasconflictingapis<conflict_dict.size()>(dbType, conflict_dict, prim, ret, check_itself);
}

bool hasconflictingapis_PhysxDeformableSurfaceAPI_deprecated(const UsdPrim& prim, bool check_itself /* = false*/)
{
    UsdPrim ret;
    static std::array<TfType, 8> conflict_dict = {
        TfType::Find<UsdPhysicsRigidBodyAPI>(),
        TfType::Find<UsdPhysicsCollisionAPI>(),
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
        TfType::Find<UsdPhysicsArticulationRootAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>(),
    };

    static const TfType dsType = TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>();
    return hasconflictingapis<conflict_dict.size()>(dsType, conflict_dict, prim, ret, check_itself);
}

bool hasconflictingapis_DeformableBodyAPI(const UsdPrim& prim,  bool check_itself/* = false*/)
{
    UsdPrim ret;
    static std::array<TfType, 8> conflict_dict = {
        TfType::Find<UsdPhysicsRigidBodyAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
        TfType::Find<UsdPhysicsArticulationRootAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>(),
    };

    static const TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    return hasconflictingapis<conflict_dict.size()>(dbType, conflict_dict, prim, ret, check_itself);
}

bool hasconflictingapis_PhysxParticleSamplingAPI(const UsdPrim& prim, bool check_itself/* = false*/)
{
    UsdPrim ret;
    static std::array<TfType, 9> conflict_dict = {
        TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI),
        TfType::Find<UsdPhysicsRigidBodyAPI>(),
        TfType::Find<UsdPhysicsCollisionAPI>(),
        TfType::Find<UsdPhysicsArticulationRootAPI>(),
    };

    static const TfType psType = TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>();
    return hasconflictingapis<conflict_dict.size()>(psType, conflict_dict, prim, ret, check_itself);
}

bool hasconflictingapis_PhysxParticleClothAPI_deprecated(const UsdPrim& prim, bool check_itself /* = false*/)
{
    UsdPrim ret;
    static std::array<TfType, 9> conflict_dict = {
        TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>(),
        TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSetAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleClothAPI>(),
        TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>(),
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI),
        TfType::Find<UsdPhysicsRigidBodyAPI>(),
        TfType::Find<UsdPhysicsCollisionAPI>(),
        TfType::Find<UsdPhysicsArticulationRootAPI>(),
    };

    static const TfType pcType = TfType::Find<PhysxSchemaPhysxParticleClothAPI>();
    return hasconflictingapis<conflict_dict.size()>(pcType, conflict_dict, prim, ret, check_itself);
}

bool isOverConflictingApisSubtreeLimit(const UsdPrim& prim, unsigned int limit)
{
    CARB_PROFILE_ZONE(carb::profiler::kCaptureMaskDefault, "doConflictingApisLimitCheck");
    unsigned int num = 0;
    auto descendants = prim.GetDescendants();
    for (const auto& d : descendants)
    {
        num++;
        if (num >= limit)
        {
            return true;
        }
    }

    return false;
}

std::array<bool, 3> hasconflictingapis_Precompute(const UsdPrim& prim)
{
    CARB_PROFILE_ZONE(carb::profiler::kCaptureMaskDefault, "hasconflictingapis_Precompute");

    // compute hasconflictingapis for
    // CollisionAPI, ArticulationRoot, RigidBodyAPI
    // more efficiently together
    static TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);

    static std::array<TfType, 3> list_common = { TfType::Find<PhysxSchemaPhysxDeformableAPI>(),
                                                 TfType::Find<PhysxSchemaPhysxParticleAPI>(),
                                                 TfType::Find<PhysxSchemaPhysxParticleSamplingAPI>() };

    static std::array<TfType, 1> list_cct = { TfType::Find<PhysxSchemaPhysxCharacterControllerAPI>() };
    static std::array<TfType, 1> list_db = { dbType };

    // first param is not used with check_itself = false
    static const TfType empty_type = TfType();
    UsdPrim ret;
    bool res_common = hasconflictingapis<list_common.size()>(empty_type, list_common, prim, ret);
    bool res_cct = hasconflictingapis<list_cct.size()>(empty_type, list_cct, prim, ret);
    bool res_db = hasconflictingapis<list_db.size()>(empty_type, list_db, prim, ret);

    static const TfType rb_api_type = TfType::Find<UsdPhysicsRigidBodyAPI>();
    static const TfType root_api_type = TfType::Find<UsdPhysicsArticulationRootAPI>();
    bool res_self_rb = do_check_itself(rb_api_type, prim, ret);
    bool res_self_root = do_check_itself(root_api_type, prim, ret);

    return {
        res_common, // CollisionAPI
        res_common || res_self_root || res_db, // ArticulationRoot
        res_common || res_self_rb || res_cct || res_db // RigidBodyAPI
    };
}

// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "Mass.h"

#include "Collision.h"
#include "LoadTools.h"
#include "MassProperties.h"
#include "Material.h"
#include "LoadUsd.h"
#include "SoftBodyDeprecated.h"
#include "Particles.h"

#include <PhysXDefines.h>
#include <common/utilities/UsdMaterialParsing.h>

#include <carb/Types.h>
#include <carb/logging/Log.h>

using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

float convertSiValueToStageUnits(const pxr::UsdStageWeakPtr stage, float siValue, int distanceExponent, int massExponent)
{
    // use double to avoid compounding precision issues
    const double metersPerUnit = UsdGeomGetStageMetersPerUnit(stage);
    const double kilogramsPerUnit = UsdPhysicsGetStageKilogramsPerUnit(stage);
    double val = siValue;
    if(distanceExponent != 0)
    {
        val *= std::pow(metersPerUnit, -distanceExponent);
    }
    if(massExponent != 0)
    {
        val *= std::pow(kilogramsPerUnit, -massExponent);
    }
    return static_cast<float>(val);
}

MassApiData parseMassApi(const UsdStageWeakPtr stage, const UsdPrim& usdPrim)
{
    MassApiData result;

    if (usdPrim.HasAPI<UsdPhysicsMassAPI>())
    {

        // density
        const UsdPhysicsMassAPI massAPI = UsdPhysicsMassAPI::Get(stage, usdPrim.GetPath());
        const UsdAttribute densityAttribute = massAPI.GetDensityAttr();
        const UsdAttribute massAttribute = massAPI.GetMassAttr();
        const UsdAttribute diagonalInertia = massAPI.GetDiagonalInertiaAttr();

        {
            float d;
            densityAttribute.Get(&d);
            if (d > 0.0f)
                result.density = d;
        }

        {
            float m;
            massAttribute.Get(&m);
            if (m > 0.0f)
                result.mass = m;
        }

        {
            GfVec3f dg;
            diagonalInertia.Get(&dg);
            if (dg[0] > 0.0f || dg[1] > 0.0f || dg[2] > 0.0f)
            {
                result.hasInertia = true;
                result.diagonalInertia = dg;
            }
        }
    }

    return result;
}

bool getCoM(const UsdStageWeakPtr stage, const UsdPrim& usdPrim, carb::Float3& com)
{
    bool comSet = false;
    if (usdPrim.HasAPI<UsdPhysicsMassAPI>())
    {
        // com
        const UsdPhysicsMassAPI massAPI = UsdPhysicsMassAPI::Get(stage, usdPrim.GetPath());
        const UsdAttribute comAttribute = massAPI.GetCenterOfMassAttr();

        GfVec3f v;
        comAttribute.Get(&v);

        // -inf -inf -inf is the sentinel value, though any inf works
        if (isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]))
        {
            UsdGeomXformCache xfCache;

            GfMatrix4d mat = xfCache.GetLocalToWorldTransform(usdPrim);
            const GfTransform tr(mat);
            const GfVec3f sc = GfVec3f(tr.GetScale());

            com.x = v[0] * sc[0];
            com.y = v[1] * sc[1];
            com.z = v[2] * sc[2];

            comSet = true;
        }
    }
    return comSet;
}

bool getPrincipalAxes(const UsdStageWeakPtr stage, const UsdPrim& usdPrim, carb::Float4& pa)
{
    bool comSet = false;
    if (usdPrim.HasAPI<UsdPhysicsMassAPI>())
    {
        // principal exes
        const UsdPhysicsMassAPI massAPI = UsdPhysicsMassAPI::Get(stage, usdPrim.GetPath());
        const UsdAttribute paAttribute = massAPI.GetPrincipalAxesAttr();

        // 0 0 0 0 is the sentinel value
        GfQuatf v;
        paAttribute.Get(&v);

        if (!GfIsClose(v.GetImaginary(), GfVec3f(0.0f), kAlmostZero) || fabsf(v.GetReal()) > kAlmostZero)
        {
            pa.x = v.GetImaginary()[0];
            pa.y = v.GetImaginary()[1];
            pa.z = v.GetImaginary()[2];
            pa.w = v.GetReal();

            comSet = true;
        }
    }
    return comSet;
}

struct InternalMassAccumulationData
{
    bool accumulateMass; // if true it indicates we are summing up the mass from density/child mass calculations
    float mass = -1.0f; //-1.0 means it is not set yet
    GfVec3f diagonalizedInertiaTensor = { 0.0f, 0.0f, 0.0f };
    GfVec3f centerOfMass = { 0.0f, 0.0f, 0.0f };
    GfQuatf principalAxes;
    float density = -1.0f;
};

MassApiData getCollisionShapeMassAPIData(const UsdStageWeakPtr stage, const UsdPrim& prim, float bodyDensity, float& density)
{
    CARB_ASSERT(isCollisionShape(stage, prim));
    const SdfPath& primPath = prim.GetPrimPath();
    SdfPath materialPath = usdmaterialutils::getMaterialBinding(prim);

    MassApiData shapeMassInfo = parseMassApi(stage, prim);

    if (shapeMassInfo.density <= 0.0)
    {
        shapeMassInfo.density = bodyDensity; // use parent density if shape doesn't have one specified
    }

    // handle material
    density = shapeMassInfo.density;
    if (density <= 0.0f) // density not set, so we take it from the materials
    {
        if (materialPath != SdfPath::EmptyPath())
        {
            const UsdPhysicsMaterialAPI materialAPI = UsdPhysicsMaterialAPI::Get(stage, materialPath);

            if (materialAPI)
            {
                UsdAttribute densityAttribute = materialAPI.GetDensityAttr();
                if (densityAttribute.HasAuthoredValue())
                {
                    float materialDensity;
                    densityAttribute.Get(&materialDensity);
                    density = materialDensity;
                }
            }
        }
    }

    return shapeMassInfo;
}

MassProperties parseCollisionShapeForMass(UsdStageWeakPtr stage,
                                          const UsdPrim& prim, ObjectId shapeObjectId,
                                          const MassApiData& inShapeMassInfo, float density,
                                          GfMatrix4f& transform,
                                          AbstractComputeRigidBodyMass* crbmInterface)
{
    MassApiData shapeMassInfo = inShapeMassInfo;
    const SdfPath& primPath = prim.GetPrimPath();
    GfMatrix3f inertia;
    PhysXUsdPhysicsInterface::MassInformation massInfo;
    if (shapeObjectId != kInvalidObjectId)
    {
        massInfo = crbmInterface->getShapeMassInfo(primPath, shapeObjectId);
        memcpy(inertia.data(), &massInfo.inertia[0], sizeof(float) * 9);
    }
    else
    {
        massInfo.volume = 1.0f;
        inertia = GfMatrix3f(0.0f);
        inertia[0][0] = 1.0f;
        inertia[1][1] = 1.0f;
        inertia[2][2] = 1.0f;
        massInfo.centerOfMass.x = 0.0f;
        massInfo.centerOfMass.y = 0.0f;
        massInfo.centerOfMass.z = 0.0f;
        massInfo.localPos = { 0.0f, 0.0f, 0.0f };
        massInfo.localRot = { 0.0f, 0.0f, 0.0f, 1.0f };
    }

    // if no density was set, use 1000 as default
    if (density <= 0.0f) {
        density = getScaledDensity(stage, 1000.0f);
    }

    carb::Float3 centerOfMass = { 0.0f, 0.0f, 0.0f };
    carb::Float4 principalAxes = { 0.0f, 0.0f, 0.0f, 1.0f };
    const bool hasCoM = getCoM(stage, prim, centerOfMass);

    if (shapeMassInfo.mass > 0.0f)
    {
        inertia = inertia * (shapeMassInfo.mass / massInfo.volume);
    }
    else if (massInfo.volume >= 0.0f)
    {
        shapeMassInfo.mass = massInfo.volume * density;
        inertia = inertia * density;
    }

    if (shapeMassInfo.hasInertia)
    {
        const pxr::GfQuatf pa(principalAxes.w, principalAxes.x, principalAxes.y, principalAxes.z);
        const pxr::GfMatrix3f rotMatr(pa);
        pxr::GfMatrix3f inMatr(0.0f);
        inMatr[0][0] = shapeMassInfo.diagonalInertia[0];
        inMatr[1][1] = shapeMassInfo.diagonalInertia[1];
        inMatr[2][2] = shapeMassInfo.diagonalInertia[2];
        inertia = inMatr * rotMatr;
    }

    if (hasCoM)
    {
        if (!shapeMassInfo.hasInertia)
        {
            // update inertia if we override the CoM but use the computed inertia
            MassProperties massProps(
                shapeMassInfo.mass, inertia,
                GfVec3f(massInfo.centerOfMass.x, massInfo.centerOfMass.y, massInfo.centerOfMass.z));
            const GfVec3f newCenterOfMass(centerOfMass.x, centerOfMass.y, centerOfMass.z);
            massProps.translate(newCenterOfMass - massProps.centerOfMass);
            inertia = massProps.inertiaTensor;
        }
        massInfo.centerOfMass.x = centerOfMass.x;
        massInfo.centerOfMass.y = centerOfMass.y;
        massInfo.centerOfMass.z = centerOfMass.z;
    }

    transform.SetTranslate(GfVec3f(massInfo.localPos.x, massInfo.localPos.y, massInfo.localPos.z));
    transform.SetRotateOnly(
        GfQuatd(massInfo.localRot.w, massInfo.localRot.x, massInfo.localRot.y, massInfo.localRot.z));

    return MassProperties(shapeMassInfo.mass, inertia,
                            GfVec3f(massInfo.centerOfMass.x, massInfo.centerOfMass.y, massInfo.centerOfMass.z));
}


struct UsdLoadRigidBodyMass : public AbstractComputeRigidBodyMass
{
    UsdLoadRigidBodyMass(const AttachedStage& attachedStage)
        : mAttachedStage(attachedStage)
    {
    }

    virtual bool getRigidBodyShapes(usdparser::ObjectId rbId, usdparser::ObjectIdUsdPrimMap& shapes) override
    {
        return mAttachedStage.getPhysXPhysicsInterface()->getRigidBodyShapes(mAttachedStage, rbId, shapes);
    }

    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(const pxr::SdfPath& path, usdparser::ObjectId objectId) override
    {
        return mAttachedStage.getPhysXPhysicsInterface()->getShapeMassInfo(path, objectId);
    }

private:
    const AttachedStage& mAttachedStage;
};

void RequestRigidBodyMassUpdate(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim)
{
    const SdfPath primPath = usdPrim.GetPrimPath();
    const ObjectIdMap* entries = attachedStage.getObjectIds(primPath);
    UsdLoadRigidBodyMass crbmInterface(attachedStage);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (it->first == eBody || it->first == eArticulationLink)
            {
                // We assume that usdPrim is a dynamic body. Callers responsibility to check
                RigidBodyMass physicsMassInfo = computeRigidBodyMass(&crbmInterface, attachedStage.getStage(), usdPrim, it->second);
                attachedStage.getPhysXPhysicsInterface()->updateMass(
                    primPath, it->second, physicsMassInfo.mass, physicsMassInfo.inertia, physicsMassInfo.centerOfMass, physicsMassInfo.principalAxes);
            }
        it++;
        }
    }
}

RigidBodyMass computeRigidBodyMass(AbstractComputeRigidBodyMass* crbmInterface, const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim, usdparser::ObjectId rbId)
{
    // Triple indentation is here to minimize diff of existing code that has been extracted
    {
        {
            {
                const SdfPath primPath = usdPrim.GetPrimPath();
                InternalMassAccumulationData massDesc;
                massDesc.principalAxes = GfQuatf(1.0f);

                // Parse dynamic body mass data
                MassApiData massInfo = parseMassApi(stage, usdPrim);
                massDesc.density = massInfo.density;
                massDesc.mass = massInfo.mass;
                massDesc.diagonalizedInertiaTensor = massInfo.diagonalInertia;
                massDesc.accumulateMass = massDesc.mass <= 0.0f;

                // check for CoM
                carb::Float3 centerOfMass = { 0.0f, 0.0f, 0.0f };
                carb::Float4 principalAxes = { 0.0f, 0.0f, 0.0f, 1.0f };
                const bool hasCoM = getCoM(stage, usdPrim, centerOfMass);
                const bool hasPa = getPrincipalAxes(stage, usdPrim, principalAxes);

                if (massDesc.accumulateMass || !massInfo.hasInertia || !hasCoM)
                {
                    std::vector<MassProperties> massProps;
                    std::vector<GfMatrix4f> massTransf;
                    ObjectIdUsdPrimMap shapeIds;

                    const bool hasTriggers = crbmInterface->getRigidBodyShapes(rbId, shapeIds);
                    const size_t numShapes = shapeIds.size();
                    massProps.reserve(numShapes);
                    massTransf.reserve(numShapes);

                    for (const std::pair<usdparser::ObjectId, pxr::UsdPrim>& shapePair : shapeIds)
                    {
                        float shapeDensity = 0.0f;
                        const UsdPrim& shapePrim = shapePair.second;

                        if (!shapePrim)
                            continue;

                        MassApiData massAPIdata = getCollisionShapeMassAPIData(stage, shapePrim, massDesc.density, shapeDensity);

                        GfMatrix4f matrix;
                        massProps.push_back(parseCollisionShapeForMass(stage, shapePrim, shapePair.first, massAPIdata, shapeDensity, matrix, crbmInterface));
                        massTransf.push_back(matrix);
                    }

                    if (!massProps.empty())
                    {
                        MassProperties accumulatedMassProps =
                            MassProperties::sum(massProps.data(), massTransf.data(), uint32_t(massProps.size()));

                        // if we had to compute mass, set the new mass
                        if (massDesc.accumulateMass)
                        {
                            massDesc.mass = accumulatedMassProps.mass;
                        }
                        else
                        {
                            const double massDiff = massDesc.mass / accumulatedMassProps.mass;
                            accumulatedMassProps.mass = massDesc.mass;
                            accumulatedMassProps.inertiaTensor = accumulatedMassProps.inertiaTensor * massDiff;
                        }

                        if (!hasCoM)
                        {
                            centerOfMass.x = accumulatedMassProps.centerOfMass[0];
                            centerOfMass.y = accumulatedMassProps.centerOfMass[1];
                            centerOfMass.z = accumulatedMassProps.centerOfMass[2];
                        }
                        else
                        {
                            const GfVec3f newCenterOfMass(centerOfMass.x, centerOfMass.y, centerOfMass.z);
                            accumulatedMassProps.translate(newCenterOfMass - accumulatedMassProps.centerOfMass);
                        }

                        GfQuatf accPa;
                        const GfVec3f accInertia = MassProperties::getMassSpaceInertia(accumulatedMassProps.inertiaTensor, accPa);

                        // check for inertia override
                        if (!massInfo.hasInertia)
                        {
                            massDesc.diagonalizedInertiaTensor = accInertia;
                        }

                        if (!hasPa)
                        {
                            principalAxes.x = accPa.GetImaginary()[0];
                            principalAxes.y = accPa.GetImaginary()[1];
                            principalAxes.z = accPa.GetImaginary()[2];
                            principalAxes.w = accPa.GetReal();
                        }
                    }
                    else
                    {
                        // no shape provided check inertia
                        if (!hasTriggers && !massInfo.hasInertia)
                        {
                            bool kinematic = false;
                            if (usdPrim.HasAPI<UsdPhysicsRigidBodyAPI>())
                            {
                                UsdPhysicsRigidBodyAPI rbAPI(usdPrim);
                                rbAPI.GetKinematicEnabledAttr().Get(&kinematic);
                            }

                            // In the absence of collision shapes and a specified inertia tensor, approximate
                            // the tensor using a sphere. If the mass is not specified
                            // throw a warning instead. Equation for spherical intertial tensor is (2/5 or
                            // 0.4)*mass*radius^2, where we use 0.1 radius to imitate point.
                            const float metersPerUnit = float(UsdGeomGetStageMetersPerUnit(stage));
                            const float radius = 0.1f / metersPerUnit;
                            const float inertiaVal = massDesc.mass > 0.0f ? 0.4f * massDesc.mass * radius * radius : 0.4f * radius * radius;
                            massDesc.diagonalizedInertiaTensor[0] = inertiaVal;
                            massDesc.diagonalizedInertiaTensor[1] = inertiaVal;
                            massDesc.diagonalizedInertiaTensor[2] = inertiaVal;
                            if (massDesc.mass > 0.0f)
                            {
                                CARB_LOG_INFO(
                                    "The rigid body at %s has a possibly invalid inertia tensor of {1.0, 1.0, 1.0}, small sphere approximated inertia was used. %s",
                                    primPath.GetString().c_str(),
                                    "Either specify correct values in the mass properties, or add collider(s) to any UsdGeom p(s) that you wish to automatically compute mass properties for.");
                            }
                            else
                            {
                                if (!kinematic)
                                {
                                    CARB_LOG_WARN(
                                        "The rigid body at %s has a possibly invalid inertia tensor of {1.0, 1.0, 1.0}%s, small sphere approximated inertia was used. %s",
                                        primPath.GetString().c_str(), (massDesc.mass < 0.0f) ? " and a negative mass" : "",
                                        "Either specify correct values in the mass properties, or add collider(s) to any shape(s) that you wish to automatically compute mass properties for.");
                                }
                            }
                        }
                        else  if (hasTriggers && massDesc.mass <= 0.0f)
                        {
                            massDesc.mass = 1.0f;
                        }
                    }
                }

                // Set mass to 1.0f as UsdPhysics spec says
                if (massDesc.mass < 0.0f)
                {                    
                    massDesc.mass = 1.0f;
                }

                const float tolerance = 0.1f;
                for (int i = 0; i < 3; i++)
                {
                    if (massDesc.diagonalizedInertiaTensor[i] < 0.0f)
                    {
                        if (massDesc.diagonalizedInertiaTensor[i] < -tolerance)
                        {
                            CARB_LOG_WARN("Physics mass: computed mass inertia tensor on a prim (%s) does have a negative diagonal value.", primPath.GetText());
                            massDesc.diagonalizedInertiaTensor[i] = fabsf(massDesc.diagonalizedInertiaTensor[i]);
                        }
                        else
                        {
                            massDesc.diagonalizedInertiaTensor[i] = fabsf(massDesc.diagonalizedInertiaTensor[i]);
                        }
                    }
                }

                const carb::Float3 diagInertia = { massDesc.diagonalizedInertiaTensor[0], massDesc.diagonalizedInertiaTensor[1],
                                                   massDesc.diagonalizedInertiaTensor[2] };

                RigidBodyMass physicsMassInfo;
                physicsMassInfo.mass = massDesc.mass;
                physicsMassInfo.inertia = diagInertia;
                physicsMassInfo.centerOfMass = centerOfMass;
                physicsMassInfo.principalAxes = principalAxes;
                return physicsMassInfo;
            }
        }
    }
}

void RequestDeformableBodyMassUpdateDeprecated(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim)
{
    const SdfPath primPath = usdPrim.GetPrimPath();
    const ObjectIdMap* entries = attachedStage.getObjectIds(primPath);

    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (it->first == eSoftBody)
            {
                SoftBodyDesc* softBodyDesc = parseDeformableBodyDeprecated(attachedStage.getStageId(), primPath);
                if (softBodyDesc)
                {
                    attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyMassDeprecated(primPath, it->second, *softBodyDesc);
                    omni::physx::usdparser::releaseDesc(softBodyDesc);
                }
            }
            it++;
        }
    }
}

void RequestDeformableSurfaceMassUpdateDeprecated(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim)
{
    const SdfPath primPath = usdPrim.GetPrimPath();
    const ObjectIdMap* entries = attachedStage.getObjectIds(primPath);

    if (entries && !entries->empty())
    {
        auto it = entries->begin();

        while (it != entries->end())
        {
            if (it->first == eFEMCloth)
            {
                FEMClothDesc* clothDesc = parseDeformableSurfaceDeprecated(attachedStage.getStageId(), primPath);
                if (clothDesc)
                {
                    attachedStage.getPhysXPhysicsInterface()->updateDeformableSurfaceMassDeprecated(primPath, it->second, *clothDesc);
                    omni::physx::usdparser::releaseDesc(clothDesc);
                }
            }

            it++;
        }
    }
}

void RequestParticleMassUpdate(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim)
{
    const SdfPath primPath = usdPrim.GetPrimPath();
    const ObjectIdMap* entries = attachedStage.getObjectIds(primPath);

    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (it->first == eParticleSet)
            {
                ParticleSetDesc* desc = parseParticleSet(attachedStage.getStageId(), primPath);
                if (desc)
                {
                    attachedStage.getPhysXPhysicsInterface()->updateParticleMass(primPath, it->second, *desc);
                    omni::physx::usdparser::releaseDesc(desc);
                }
            }

            if (it->first == eParticleCloth)
            {
                ParticleClothDesc* desc = parseParticleClothDeprecated(attachedStage.getStageId(), primPath);
                if (desc)
                {
                    attachedStage.getPhysXPhysicsInterface()->updateParticleMass(primPath, it->second, *desc);
                    omni::physx::usdparser::releaseDesc(desc);
                }
            }

            it++;
        }
    }
}


} // namespace usdparser
} // namespace physx
} // namespace omni

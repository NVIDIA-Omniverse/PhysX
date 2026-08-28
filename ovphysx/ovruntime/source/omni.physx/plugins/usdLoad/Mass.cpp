// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-MASS-001
 * @covers AC-2 AC-3
 */

#include "UsdPCH.h"

#include "Mass.h"

#include "Collision.h"
#include "LoadTools.h"
#include "PhysXTools.h"
#include "MassProperties.h"
#include "Material.h"
#include "LoadUsd.h"
#include "Particles.h"

#include <omni/physx/IPhysxSettings.h>
#include <OmniPhysX.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include "IceDescriptorAllocator.h"
#include "UsdSource.h"

#include <PhysXDefines.h>
#include <common/utilities/UsdMaterialParsing.h>

#include <carb/Types.h>
#include <carb/logging/Log.h>

using namespace PXR_NS;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{
namespace pp = omni::physics::parse; // backend-agnostic mass reads (ADR-0002 M2c-D)

float convertSiValueToStageUnits(const omni::physics::parse::SourceUnits& units, float siValue, int distanceExponent, int massExponent)
{
    // use double to avoid compounding precision issues
    const double metersPerUnit = units.metersPerUnit;
    const double kilogramsPerUnit = units.kilogramsPerUnit;
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
        const UsdPhysicsMassAPI massAPI = UsdPhysicsMassAPI::Get(stage, usdPrim.GetPath());
        const UsdAttribute comAttribute = massAPI.GetCenterOfMassAttr();

        GfVec3f v;
        comAttribute.Get(&v);

        // -inf -inf -inf is the sentinel value, though any inf works
        if (isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]))
        {
            // World transform is resolved via the source when a stage is attached;
            // falls back to direct USD compute otherwise.
            const long stageId = stage ? UsdUtilsStageCache::Get().GetId(stage).ToLongInt() : 0;
            omni::physx::usdparser::AttachedStage* as =
                omni::physx::usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
            const GfMatrix4d mat = as
                ? omni::physx::internal::getWorldTransform(*as, as->keyFor(usdPrim.GetPath()), UsdTimeCode::Default())
                : UsdGeomXformable(usdPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
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

// ---------------------------------------------------------------------------
// Source-based MassAPI reads (ADR-0002 M2c-D). Backend-agnostic replacements for
// the USD parseMassApi/getCoM/getPrincipalAxes above: they read UsdPhysicsMassAPI
// through the parse library (works for USD and ovstage alike, no UsdPrim).
// ---------------------------------------------------------------------------
namespace
{
pp::MassApiData parseSourceMassApi(pp::IPhysicsSource& source, pp::ObjectKey key)
{
    pp::ParseContext ctx(source, iceDescriptorAllocator());
    return pp::parseMassApi(ctx, key);
}

MassApiData toLocalMassApi(const pp::MassApiData& m)
{
    MassApiData out;
    out.mass = m.mass;
    out.density = m.density;
    out.hasInertia = m.hasInertia;
    out.diagonalInertia = GfVec3f(m.diagonalInertia.x, m.diagonalInertia.y, m.diagonalInertia.z);
    out.hasCenterOfMass = m.hasCenterOfMass;
    out.centerOfMass = m.centerOfMass;
    out.hasPrincipalAxes = m.hasPrincipalAxes;
    out.principalAxes = m.principalAxes;
    return out;
}

// Center of mass. parse-lib parseMassApi already applies the prim's local-to-world
// per-axis scale (matching the legacy getCoM), so use its value directly — do NOT
// re-scale here (that was a double-scale bug).
bool getCoMSource(pp::IPhysicsSource& source, pp::ObjectKey key, carb::Float3& com)
{
    const pp::MassApiData m = parseSourceMassApi(source, key);
    if (!m.hasCenterOfMass)
        return false;
    com = m.centerOfMass;
    return true;
}

bool getPrincipalAxesSource(pp::IPhysicsSource& source, pp::ObjectKey key, carb::Float4& pa)
{
    const pp::MassApiData m = parseSourceMassApi(source, key);
    if (!m.hasPrincipalAxes)
        return false;
    pa = m.principalAxes;
    return true;
}
} // namespace

MassApiData getCollisionShapeMassAPIData(pp::IPhysicsSource& source, pp::ObjectKey shapeKey, float bodyDensity, float& density)
{
    MassApiData shapeMassInfo = toLocalMassApi(parseSourceMassApi(source, shapeKey));

    if (shapeMassInfo.density <= 0.0)
    {
        shapeMassInfo.density = bodyDensity; // use parent density if shape doesn't have one specified
    }

    // handle material — bound physics material's density via the source.
    density = shapeMassInfo.density;
    if (density <= 0.0f) // density not set, so we take it from the materials
    {
        const pp::ObjectKey materialKey = source.getMaterialBinding(shapeKey);
        if (materialKey.valid())
        {
            pp::ParseContext ctx(source, iceDescriptorAllocator());
            if (pp::DescPtr<pp::PhysxMaterialDesc> mat = pp::parseMaterial(ctx, materialKey))
            {
                if (mat->density > 0.0f)
                    density = mat->density;
            }
        }
    }

    return shapeMassInfo;
}

MassProperties parseCollisionShapeForMass(pp::IPhysicsSource& source,
                                          const SdfPath& path, pp::ObjectKey shapeKey, ObjectId shapeObjectId,
                                          const MassApiData& inShapeMassInfo, float density,
                                          GfMatrix4f& transform,
                                          AbstractComputeRigidBodyMass* crbmInterface)
{
    MassApiData shapeMassInfo = inShapeMassInfo;
    GfMatrix3f inertia;
    PhysXUsdPhysicsInterface::MassInformation massInfo;
    if (shapeObjectId != kInvalidObjectId)
    {
        massInfo = crbmInterface->getShapeMassInfo(path, shapeObjectId);
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
        density = getScaledDensity(source.getSourceUnits(), 1000.0f);
    }

    carb::Float3 centerOfMass = { 0.0f, 0.0f, 0.0f };
    carb::Float4 principalAxes = { 0.0f, 0.0f, 0.0f, 1.0f };
    const bool hasCoM = shapeMassInfo.hasCenterOfMass;
    if (hasCoM)
        centerOfMass = shapeMassInfo.centerOfMass;
    if (shapeMassInfo.hasPrincipalAxes)
        principalAxes = shapeMassInfo.principalAxes;

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
        const PXR_NS::GfQuatf pa(principalAxes.w, principalAxes.x, principalAxes.y, principalAxes.z);
        const PXR_NS::GfMatrix3f rotMatr(pa);
        PXR_NS::GfMatrix3f inMatr(0.0f);
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

    virtual bool getRigidBodyShapes(usdparser::ObjectId rbId, usdparser::ObjectIdPathMap& shapes) override
    {
        return mAttachedStage.getPhysXPhysicsInterface()->getRigidBodyShapes(mAttachedStage, rbId, shapes);
    }

    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(const PXR_NS::SdfPath& path, usdparser::ObjectId objectId) override
    {
        return mAttachedStage.getPhysXPhysicsInterface()->getShapeMassInfo(path, objectId);
    }

private:
    const AttachedStage& mAttachedStage;
};

void RequestRigidBodyMassUpdate(AttachedStage& attachedStage, pp::ObjectKey bodyKey)
{
    pp::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;
    const SdfPath primKey = attachedStage.pathFor(bodyKey);
    const ObjectIdMap* entries = attachedStage.getObjectIds(primKey);
    UsdLoadRigidBodyMass crbmInterface(attachedStage);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (it->first == eBody || it->first == eArticulationLink)
            {
                // We assume that the body is dynamic. Caller's responsibility to check.
                RigidBodyMass physicsMassInfo = computeRigidBodyMass(&crbmInterface, *source, bodyKey, it->second);
                attachedStage.getPhysXPhysicsInterface()->updateMass(
                    primKey, it->second, physicsMassInfo.mass, physicsMassInfo.inertia, physicsMassInfo.centerOfMass, physicsMassInfo.principalAxes);
            }
        it++;
        }
    }
}

RigidBodyMass computeRigidBodyMass(AbstractComputeRigidBodyMass* crbmInterface, pp::IPhysicsSource& source, pp::ObjectKey bodyKey, usdparser::ObjectId rbId)
{
    // Triple indentation is here to minimize diff of existing code that has been extracted
    {
        {
            {
                const SdfPath primKey(std::string(source.sourceKeyToString(bodyKey)));
                InternalMassAccumulationData massDesc;
                massDesc.principalAxes = GfQuatf(1.0f);

                // Parse dynamic body mass data via the source (backend-agnostic).
                MassApiData massInfo = toLocalMassApi(parseSourceMassApi(source, bodyKey));
                massDesc.density = massInfo.density;
                massDesc.mass = massInfo.mass;
                massDesc.diagonalizedInertiaTensor = massInfo.diagonalInertia;
                massDesc.accumulateMass = massDesc.mass <= 0.0f;

                // check for CoM
                carb::Float3 centerOfMass = { 0.0f, 0.0f, 0.0f };
                carb::Float4 principalAxes = { 0.0f, 0.0f, 0.0f, 1.0f };
                const bool hasCoM = massInfo.hasCenterOfMass;
                if (hasCoM)
                    centerOfMass = massInfo.centerOfMass;
                const bool hasPa = massInfo.hasPrincipalAxes;
                if (hasPa)
                    principalAxes = massInfo.principalAxes;

                if (massDesc.accumulateMass || !massInfo.hasInertia || !hasCoM)
                {
                    std::vector<MassProperties> massProps;
                    std::vector<GfMatrix4f> massTransf;
                    ObjectIdPathMap shapeIds;

                    const bool hasTriggers = crbmInterface->getRigidBodyShapes(rbId, shapeIds);
                    const size_t numShapes = shapeIds.size();
                    massProps.reserve(numShapes);
                    massTransf.reserve(numShapes);

                    for (const std::pair<usdparser::ObjectId, PXR_NS::SdfPath>& shapePair : shapeIds)
                    {
                        float shapeDensity = 0.0f;
                        const SdfPath& shapePath = shapePair.second;

                        if (shapePath.IsEmpty())
                            continue;

                        const pp::ObjectKey shapeKey = source.findByPath(shapePath.GetString());
                        MassApiData massAPIdata = getCollisionShapeMassAPIData(source, shapeKey, massDesc.density, shapeDensity);

                        GfMatrix4f matrix;
                        massProps.push_back(parseCollisionShapeForMass(source, shapePath, shapeKey, shapePair.first, massAPIdata, shapeDensity, matrix, crbmInterface));
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
                            source.getAttribute(bodyKey, source.internToken("physics:kinematicEnabled"), kinematic);

                            // In the absence of collision shapes and a specified inertia tensor, approximate
                            // the tensor using a sphere. If the mass is not specified
                            // throw a warning instead. Equation for spherical intertial tensor is (2/5 or
                            // 0.4)*mass*radius^2, where we use 0.1 radius to imitate point.
                            const float metersPerUnit = source.getSourceUnits().metersPerUnit;
                            const float radius = 0.1f / metersPerUnit;
                            const float inertiaVal = massDesc.mass > 0.0f ? 0.4f * massDesc.mass * radius * radius : 0.4f * radius * radius;
                            massDesc.diagonalizedInertiaTensor[0] = inertiaVal;
                            massDesc.diagonalizedInertiaTensor[1] = inertiaVal;
                            massDesc.diagonalizedInertiaTensor[2] = inertiaVal;
                            if (massDesc.mass > 0.0f)
                            {
                                CARB_LOG_INFO(
                                    "The rigid body at %s has a possibly invalid inertia tensor of {1.0, 1.0, 1.0}, small sphere approximated inertia was used. %s",
                                    primKey.GetString().c_str(),
                                    "Either specify correct values in the mass properties, or add collider(s) to any UsdGeom p(s) that you wish to automatically compute mass properties for.");
                            }
                            else
                            {
                                if (!kinematic)
                                {
                                    CARB_LOG_WARN(
                                        "The rigid body at %s has a possibly invalid inertia tensor of {1.0, 1.0, 1.0}%s, small sphere approximated inertia was used. %s",
                                        primKey.GetString().c_str(), (massDesc.mass < 0.0f) ? " and a negative mass" : "",
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
                            CARB_LOG_WARN("Physics mass: computed mass inertia tensor on a prim (%s) does have a negative diagonal value.", primKey.GetText());
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

void RequestParticleMassUpdate(AttachedStage& attachedStage, omni::physics::parse::ObjectKey particleKey)
{
    const SdfPath primKey = attachedStage.pathFor(particleKey);
    if (primKey.IsEmpty())
        return;

    const ObjectIdMap* entries = attachedStage.getObjectIds(primKey);

    if (entries && !entries->empty())
    {
        ObjectIdMap::const_iterator it = entries->begin();
        while (it != entries->end())
        {
            if (it->first == eParticleSet)
            {
                // Re-read the set through the parse library, then build the
                // engine descriptor for the mass update (source-backed).
                const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
                if (src)
                {
                    omni::physics::parse::ParseContext ctx(const_cast<omni::physics::parse::IPhysicsSource&>(*src), iceDescriptorAllocator());
                    if (omni::physics::parse::DescPtr<omni::physics::parse::ParticleSetDesc> scanDesc =
                            omni::physics::parse::parseParticleSet(ctx, particleKey))
                    {
                        ParticleSetDesc* desc = buildParticleSetDescRuntime(attachedStage, *scanDesc);
                        attachedStage.getPhysXPhysicsInterface()->updateParticleMass(primKey, it->second, *desc);
                        omni::physx::usdparser::releaseDesc(desc);
                    }
                }
            }

            it++;
        }
    }
}


} // namespace usdparser
} // namespace physx
} // namespace omni

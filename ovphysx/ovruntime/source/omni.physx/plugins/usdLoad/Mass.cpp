// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-MASS-001
 * @covers AC-2 AC-3
 *
 * @implements REQ-PARSE-MASS-002
 * @covers AC-1 AC-2
 *
 * @implements REQ-SIM-OBJECTDB-001
 * @covers AC-3
 *
 * @implements REQ-PARSE-MASS-003
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-2
 * computeRigidBodyMass and RequestParticleMassUpdate adopt the attach batch.
 */

#include "Mass.h"
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

#include <PhysXDefines.h>

#include <carb/Types.h>
#include <carb/logging/Log.h>

using namespace carb;
using namespace ::physx;

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

struct InternalMassAccumulationData
{
    bool accumulateMass; // if true it indicates we are summing up the mass from density/child mass calculations
    float mass = -1.0f; //-1.0 means it is not set yet
    ::physx::PxVec3 diagonalizedInertiaTensor = { 0.0f, 0.0f, 0.0f };
    ::physx::PxVec3 centerOfMass = { 0.0f, 0.0f, 0.0f };
    ::physx::PxQuat principalAxes;
    float density = -1.0f;
};

// ---------------------------------------------------------------------------
// Source-based MassAPI reads (ADR-0002 M2c-D): UsdPhysicsMassAPI is read through
// the parse library (works for USD and ovstage alike, no UsdPrim).
// ---------------------------------------------------------------------------
namespace
{
MassApiData toLocalMassApi(const pp::MassApiData& m)
{
    MassApiData out;
    out.mass = m.mass;
    out.density = m.density;
    out.hasInertia = m.hasInertia;
    out.diagonalInertia = toPhysX(m.diagonalInertia);
    out.hasCenterOfMass = m.hasCenterOfMass;
    out.centerOfMass = m.centerOfMass;
    out.hasPrincipalAxes = m.hasPrincipalAxes;
    out.principalAxes = m.principalAxes;
    return out;
}
} // namespace

MassApiData getCollisionShapeMassAPIData(pp::ParseContext& ctx,
                                         pp::ObjectKey shapeKey,
                                         float bodyDensity,
                                         float& density)
{
    pp::IPhysicsSource& source = ctx.source();
    MassApiData shapeMassInfo = toLocalMassApi(pp::parseMassApi(ctx, shapeKey));

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
                                          pp::ObjectKey shapeKey, ObjectId shapeObjectId,
                                          const MassApiData& inShapeMassInfo, float density,
                                          PxTransform& transform,
                                          AbstractComputeRigidBodyMass* crbmInterface)
{
    MassApiData shapeMassInfo = inShapeMassInfo;
    PxMat33 inertia(PxZero);
    PhysXUsdPhysicsInterface::MassInformation massInfo;
    if (shapeObjectId != kInvalidObjectId)
    {
        massInfo = crbmInterface->getShapeMassInfo(shapeObjectId);
        // PxMat33 stores the same nine floats in the same order the Gf matrix did,
        // so this raw copy lands identically whether or not the tensor is symmetric.
        memcpy(&inertia.column0.x, &massInfo.inertia[0], sizeof(float) * 9);
    }
    else
    {
        massInfo.volume = 1.0f;
        inertia = PxMat33(PxIdentity);
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

    // Authored center of mass and inertia axes are source-collider-local. If either is present,
    // express both computed values in that frame before mixing authored and computed properties.
    const bool usesSourceFrame = hasCoM || shapeMassInfo.hasInertia;
    const PxTransform geometryToSource = toPhysX(massInfo.geometryToSourcePos, massInfo.geometryToSourceRot);
    if (usesSourceFrame)
    {
        massInfo.centerOfMass = fromPhysX(geometryToSource.transform(toPhysX(massInfo.centerOfMass)));
        if (!shapeMassInfo.hasInertia)
            inertia = MassProperties::rotateInertia(inertia, geometryToSource.q);
    }

    if (shapeMassInfo.hasInertia)
    {
        const PxQuat pa = toPhysXQuat(principalAxes);
        // PxMat33(pa) and GfMatrix3f(pa) hold the same nine floats, but PhysX
        // multiplies column-vector style, so the operands swap to keep the
        // product identical to the row-vector `inMatr * rotMatr`.
        const PxMat33 rotMatr(pa);
        PxMat33 inMatr(PxZero);
        inMatr[0][0] = shapeMassInfo.diagonalInertia[0];
        inMatr[1][1] = shapeMassInfo.diagonalInertia[1];
        inMatr[2][2] = shapeMassInfo.diagonalInertia[2];
        inertia = rotMatr * inMatr;
    }

    if (hasCoM)
    {
        if (!shapeMassInfo.hasInertia)
        {
            // update inertia if we override the CoM but use the computed inertia
            MassProperties massProps(shapeMassInfo.mass, inertia, toPhysX(massInfo.centerOfMass));
            const PxVec3 newCenterOfMass = toPhysX(centerOfMass);
            massProps.translate(newCenterOfMass - massProps.centerOfMass);
            inertia = massProps.inertiaTensor;
        }
        massInfo.centerOfMass.x = centerOfMass.x;
        massInfo.centerOfMass.y = centerOfMass.y;
        massInfo.centerOfMass.z = centerOfMass.z;
    }

    const PxTransform geometryToBody = toPhysX(massInfo.localPos, massInfo.localRot);
    transform = usesSourceFrame ? geometryToBody * geometryToSource.getInverse() : geometryToBody;

    return MassProperties(shapeMassInfo.mass, inertia, toPhysX(massInfo.centerOfMass));
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

    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(usdparser::ObjectId objectId) override
    {
        return mAttachedStage.getPhysXPhysicsInterface()->getShapeMassInfo(objectId);
    }

private:
    const AttachedStage& mAttachedStage;
};

void RequestRigidBodyMassUpdate(AttachedStage& attachedStage, pp::ObjectKey bodyKey)
{
    pp::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;
    // bodyKey is already attachedStage's own ObjectKey -- look entries up
    // directly rather than round-tripping through an SdfPath (updateMass's
    // ObjectKey overload applies mass purely by ObjectId, no path needed).
    const ObjectIdMap* entries = attachedStage.getObjectIds(bodyKey);
    UsdLoadRigidBodyMass crbmInterface(attachedStage);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            // kInvalidObjectId would be used unchecked as an index into the internal record array
            // (getRigidBodyShapes/getShapeMassInfo), reading far out of bounds in release builds.
            if ((it->first == eBody || it->first == eArticulationLink) && it->second != kInvalidObjectId)
            {
                // We assume that the body is dynamic. Caller's responsibility to check.
                RigidBodyMass physicsMassInfo =
                    computeRigidBodyMass(&crbmInterface, *source, bodyKey, it->second, &attachedStage.getKnownTokens());
                attachedStage.getPhysXPhysicsInterface()->updateMass(
                    bodyKey, it->second, physicsMassInfo.mass, physicsMassInfo.inertia, physicsMassInfo.centerOfMass, physicsMassInfo.principalAxes);
            }
        it++;
        }
    }
}

// @implements REQ-LOAD-TOKENS-001
RigidBodyMass computeRigidBodyMass(AbstractComputeRigidBodyMass* crbmInterface, pp::IPhysicsSource& source, pp::ObjectKey bodyKey, usdparser::ObjectId rbId,
                                   const pp::KnownTokens* knownTokens)
{
    pp::ParseContext ctx(source, iceDescriptorAllocator());
    // Runs once per dynamic body: adopt the attach's batch instead of re-interning per body.
    if (knownTokens)
        ctx.adoptKnownTokens(*knownTokens);
    // Triple indentation is here to minimize diff of existing code that has been extracted
    {
        {
            {
                const std::string primKey(source.sourceKeyToString(bodyKey));
                InternalMassAccumulationData massDesc;
                massDesc.principalAxes = PxQuat(PxIdentity);

                // Parse dynamic body mass data via the source (backend-agnostic).
                MassApiData massInfo = toLocalMassApi(pp::parseMassApi(ctx, bodyKey));
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
                    std::vector<PxTransform> massTransf;
                    ObjectIdPathMap shapeIds;

                    const bool hasTriggers = crbmInterface->getRigidBodyShapes(rbId, shapeIds);
                    const size_t numShapes = shapeIds.size();
                    massProps.reserve(numShapes);
                    massTransf.reserve(numShapes);

                    for (const std::pair<usdparser::ObjectId, pp::ObjectKey>& shapePair : shapeIds)
                    {
                        float shapeDensity = 0.0f;
                        const pp::ObjectKey shapeKey = shapePair.second;

                        if (!shapeKey.valid())
                            continue;

                        MassApiData massAPIdata =
                            getCollisionShapeMassAPIData(ctx, shapeKey, massDesc.density, shapeDensity);

                        PxTransform matrix(PxIdentity);
                        massProps.push_back(parseCollisionShapeForMass(source, shapeKey, shapePair.first, massAPIdata, shapeDensity, matrix, crbmInterface));
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
                            accumulatedMassProps.inertiaTensor = accumulatedMassProps.inertiaTensor * float(massDiff);
                        }

                        if (!hasCoM)
                        {
                            centerOfMass = toFloat3(accumulatedMassProps.centerOfMass);
                        }
                        else
                        {
                            const PxVec3 newCenterOfMass = toPhysX(centerOfMass);
                            accumulatedMassProps.translate(newCenterOfMass - accumulatedMassProps.centerOfMass);
                        }

                        PxQuat accPa;
                        const PxVec3 accInertia = MassProperties::getMassSpaceInertia(accumulatedMassProps.inertiaTensor, accPa);

                        // check for inertia override
                        if (!massInfo.hasInertia)
                        {
                            massDesc.diagonalizedInertiaTensor = accInertia;
                        }

                        if (!hasPa)
                        {
                            principalAxes = toFloat4(accPa);
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
                                    primKey.c_str(),
                                    "Either specify correct values in the mass properties, or add collider(s) to any UsdGeom p(s) that you wish to automatically compute mass properties for.");
                            }
                            else
                            {
                                if (!kinematic)
                                {
                                    CARB_LOG_WARN(
                                        "The rigid body at %s has a possibly invalid inertia tensor of {1.0, 1.0, 1.0}%s, small sphere approximated inertia was used. %s",
                                        primKey.c_str(), (massDesc.mass < 0.0f) ? " and a negative mass" : "",
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
                            CARB_LOG_WARN("Physics mass: computed mass inertia tensor on a prim (%s) does have a negative diagonal value.", primKey.c_str());
                            massDesc.diagonalizedInertiaTensor[i] = fabsf(massDesc.diagonalizedInertiaTensor[i]);
                        }
                        else
                        {
                            massDesc.diagonalizedInertiaTensor[i] = fabsf(massDesc.diagonalizedInertiaTensor[i]);
                        }
                    }
                }

                const carb::Float3 diagInertia = toFloat3(massDesc.diagonalizedInertiaTensor);

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
    if (!particleKey.valid())
        return;

    // particleKey is already attachedStage's own ObjectKey -- look entries up
    // directly rather than round-tripping through an SdfPath (updateParticleMass's
    // key parameter is already unused by the body -- mass is applied purely by ObjectId).
    const ObjectIdMap* entries = attachedStage.getObjectIds(particleKey);

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
                    ctx.adoptKnownTokens(attachedStage.getKnownTokens());
                    if (omni::physics::parse::DescPtr<omni::physics::parse::ParticleSetDesc> scanDesc =
                            omni::physics::parse::parseParticleSet(ctx, particleKey))
                    {
                        ParticleSetDesc* desc = buildParticleSetDescRuntime(attachedStage, *scanDesc);
                        attachedStage.getPhysXPhysicsInterface()->updateParticleMass(particleKey, it->second, *desc);
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

// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>
#include <common/utilities/PrimUtilities.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "CollisionGroup.h"
#include "Material.h"
#include <physxSchema/tetrahedralMesh.h>

using namespace pxr;
using namespace carb;

namespace
{
    template <typename T>
    bool SafeGetAttributeP(T* out, UsdAttribute const& attribute)
    {
        if (attribute.HasValue())
        {
            attribute.Get(out);

            return true;
        }

        return false;
    }

    template <typename T>
    bool SafeGetAttributeP(T* out, UsdAttribute const& attribute, T defaultValue)
    {
        if (attribute.HasValue())
        {
            attribute.Get(out);
        }
        else
        {
            *out = defaultValue;
        }

        return true;
    }

    template <>
    bool SafeGetAttributeP<carb::Float3>(carb::Float3* out, UsdAttribute const& attribute)
    {
        if (attribute.HasValue())
        {
            GfVec3f v;
            attribute.Get(&v);
            out->x = v[0];
            out->y = v[1];
            out->z = v[2];

            return true;
        }

        return false;
    }

    void convert(std::vector<carb::Uint4>& out, VtArray<GfVec4i> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i].x = in[i][0];
            out[i].y = in[i][1];
            out[i].z = in[i][2];
            out[i].w = in[i][3];
        }
    }

    void convert(std::vector<carb::Float3>& out, VtArray<GfVec3f> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i].x = in[i][0];
            out[i].y = in[i][1];
            out[i].z = in[i][2];
        }
    }

    void convert(std::vector<carb::Int2>& out, VtArray<GfVec2i> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i].x = in[i][0];
            out[i].y = in[i][1];
        }
    }

    template<typename TIn, typename TOut>
    void convert(std::vector<TIn>& out, VtArray<TOut> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i] = in[i];
        }
    }

    template <typename T>
    bool GetAuthoredValueOrDefault(T* value, const UsdAttribute& attribute, const T& defaultValue)
    {
        if (attribute.HasAuthoredValue())
        {
            attribute.Get(value);
            return true;
        }
        *value = defaultValue;
        return false;
    }

    template <typename SrcT, typename DstT>
    bool convertIndexBuffer(DstT* out, const SrcT* in, size_t size, uint32_t multiple, SrcT range)
    {
        if (size % multiple != 0)
        {
            return false;
        }
        for (size_t i = 0; i < size; ++i)
        {
            if (in[i] >= range)
            {
                return false;
            }
            out[i] = in[i];
        }
        return true;
    }

    template <typename SrcT, typename DstT>
    bool convertIndexBuffer(std::vector<DstT>& out, const VtArray<SrcT>& in, uint32_t multiple, SrcT range)
    {
        out.resize(in.size());
        return convertIndexBuffer(out.data(), in.data(), out.size(), multiple, range);
    }

}

namespace omni
{
    namespace physx
    {
        namespace usdparser
        {
            void ParseDeformableMaterialDeprecated(FemMaterialDesc* mat, const UsdPrim& usdPrim)
            {
                float metersPerUnit = float(UsdGeomGetStageMetersPerUnit(usdPrim.GetStage()));
                float kilogramsPerUnit = float(UsdPhysicsGetStageKilogramsPerUnit(usdPrim.GetStage()));

                if (usdPrim.HasAPI<PhysxSchemaPhysxDeformableBodyMaterialAPI>())
                {
                    PhysxSchemaPhysxDeformableBodyMaterialAPI materialAPI(usdPrim);
                    FemSoftBodyMaterialDesc* out = (FemSoftBodyMaterialDesc*)mat;

                    // preist: Default to 0.0 such that density logic in createDeformableBody works
                    // default density of 1000kg/m3 will be set there in case neither the material nor the MassAPI
                    // define density or mass.
                    GetAuthoredValueOrDefault<float>(&out->density, materialAPI.GetDensityAttr(),
                                                     0.0f * metersPerUnit * metersPerUnit * metersPerUnit / kilogramsPerUnit);

                    // common material parameters
                    GetAuthoredValueOrDefault<float>(&out->youngs, materialAPI.GetYoungsModulusAttr(),
                                                     50000000.0f * metersPerUnit / kilogramsPerUnit);

                    materialAPI.GetPoissonsRatioAttr().Get(&out->poissons);
                    materialAPI.GetDynamicFrictionAttr().Get(&out->dynamicFriction);
                    materialAPI.GetElasticityDampingAttr().Get(&out->damping);

                    // softbody material parameters
                    materialAPI.GetDampingScaleAttr().Get(&out->dampingScale);

                    out->materialPath = usdPrim.GetPath();
                }

                if (usdPrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceMaterialAPI>())
                {
                    PhysxSchemaPhysxDeformableSurfaceMaterialAPI materialAPI(usdPrim);
                    FemClothMaterialDesc* out = (FemClothMaterialDesc*)mat;

                    GetAuthoredValueOrDefault<float>(&out->density, materialAPI.GetDensityAttr(), 0.0f);

                    // common material parameters
                    GetAuthoredValueOrDefault<float>(
                        &out->youngs, materialAPI.GetYoungsModulusAttr(), 1.0e+7f * metersPerUnit / kilogramsPerUnit);

                    materialAPI.GetPoissonsRatioAttr().Get(&out->poissons);
                    materialAPI.GetDynamicFrictionAttr().Get(&out->dynamicFriction);

                    static TfToken bendDampingToken("physxDeformableSurfaceMaterial:bendDamping");
                    static TfToken elasticityDampingToken("physxDeformableSurfaceMaterial:elasticityDamping");
                    static TfToken bendStiffnessToken("physxDeformableSurfaceMaterial:bendStiffness");
                    GetAuthoredValueOrDefault<float>(&out->bendDamping, usdPrim.GetAttribute(bendDampingToken), 0.0f);
                    GetAuthoredValueOrDefault<float>(&out->elasticityDamping, usdPrim.GetAttribute(elasticityDampingToken), 0.0f);
                    GetAuthoredValueOrDefault<float>(&out->bendStiffness, usdPrim.GetAttribute(bendStiffnessToken), 0.0f);

                    // FEM cloth material parameters
                    // Note: If the fallback changes in the schema, we have to update accordingly.
                    GetAuthoredValueOrDefault<float>(
                        &out->thickness, materialAPI.GetThicknessAttr(), 0.005f / metersPerUnit);

                    out->materialPath = usdPrim.GetPath();
                }
            }

            FemSoftBodyMaterialDesc* ParseDeformableBodyMaterialDeprecated(const UsdPrim& usdPrim)
            {
                if (!usdPrim.HasAPI<PhysxSchemaPhysxDeformableBodyMaterialAPI>())
                {
                    CARB_LOG_WARN("ParseDeformableBodyMaterial: UsdPrim doesn't have a PhysxSchemaPhysxDeformableBodyMaterialAPI\n");
                    return nullptr;
                }

                // create and fill in descriptor
                FemSoftBodyMaterialDesc* softBodyMaterialDesc = ICE_PLACEMENT_NEW(FemSoftBodyMaterialDesc)();

                ParseDeformableMaterialDeprecated(softBodyMaterialDesc, usdPrim);

                return softBodyMaterialDesc;
            }

            FemClothMaterialDesc* ParseDeformableSurfaceMaterialDeprecated(const UsdPrim& usdPrim)
            {
                if (!usdPrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceMaterialAPI>())
                {
                    CARB_LOG_WARN("ParseDeformableSurfaceMaterial: UsdPrim doesn't have a PhysxSchemaPhysxDeformableSurfaceMaterialAPI\n");
                    return nullptr;
                }

                // create and fill in descriptor
                FemClothMaterialDesc* FEMClothMaterialDesc = ICE_PLACEMENT_NEW(FemClothMaterialDesc)();

                ParseDeformableMaterialDeprecated(FEMClothMaterialDesc, usdPrim);

                return FEMClothMaterialDesc;
            }

            bool ParseDeformableDescDeprecated(AttachedStage& attachedStage, DeformableDesc& deformableDesc, const UsdPrim& deformablePrim, const ObjectType materialType,
                                     const uint32_t numVertsPerElement, const uint32_t numSimPoints)
            {
                float metersPerUnit = float(UsdGeomGetStageMetersPerUnit(deformablePrim.GetStage()));
                float unitsPerMeter = 1.f / metersPerUnit;

                PhysxSchemaPhysxDeformableAPI deformableApi(deformablePrim);
                UsdGeomMesh mesh(deformablePrim);

                deformableApi.GetSolverPositionIterationCountAttr().Get(&deformableDesc.solverPositionIterations);

                deformableApi.GetVertexVelocityDampingAttr().Get(&deformableDesc.velocityDamping);

                deformableApi.GetSleepDampingAttr().Get(&deformableDesc.sleepDamping);

                GetAuthoredValueOrDefault<float>(&deformableDesc.sleepThreshold, deformableApi.GetSleepThresholdAttr(), 0.05f * unitsPerMeter);

                GetAuthoredValueOrDefault<float>(&deformableDesc.settlingThreshold, deformableApi.GetSettlingThresholdAttr(), 0.1f * unitsPerMeter);

                deformableApi.GetSelfCollisionAttr().Get(&deformableDesc.selfCollision);

                deformableApi.GetSelfCollisionFilterDistanceAttr().Get(&deformableDesc.selfCollisionFilterDistance);

                deformableApi.GetEnableCCDAttr().Get(&deformableDesc.enableCCD);

                if(deformableApi.GetMaxDepenetrationVelocityAttr().IsAuthored())
                {
                    CARB_LOG_WARN(
                        "PhysxSchemaPhysxDeformableAPI:maxDepenetrationVelocity on %s is set but but the corresponding feature is not yet supported.",
                        deformablePrim.GetPath().GetText());
                }

                VtArray<GfVec3f> points;
                SafeGetAttributeP(&points, mesh.GetPointsAttr());
                convert(deformableDesc.points, points);

                UsdAttribute restPointsAttr = deformableApi.GetRestPointsAttr();
                VtArray<GfVec3f> restPoints;
                if (restPointsAttr.HasAuthoredValue())
                {
                    SafeGetAttributeP(&restPoints, restPointsAttr);
                }

                if (restPoints.size() != points.size())
                {
                    if (!restPoints.empty()) // only warn in non-trivial case
                    {
                        CARB_LOG_WARN(
                            "PhysxSchemaPhysxDeformableAPI:restPoints and UsdGeomMesh:points "
                            "have inconsistent size. Resetting the restPoints to UsdGeomMesh:points.");
                    }
                    deformableApi.CreateRestPointsAttr().Set(points);
                    deformableDesc.restPoints = deformableDesc.points;
                }
                else
                {
                    convert(deformableDesc.restPoints, restPoints);
                }

                UsdAttribute simulationVelocitiesAttr = deformableApi.GetSimulationVelocitiesAttr();
                if (simulationVelocitiesAttr.HasAuthoredValue())
                {
                    VtArray<GfVec3f> simulationVelocities;
                    SafeGetAttributeP(&simulationVelocities, simulationVelocitiesAttr);
                    if (simulationVelocities.size() > 0 && uint32_t(simulationVelocities.size()) != numSimPoints)
                    {
                        CARB_LOG_WARN("PhysxSchemaPhysxDeformableAPI:simulationVelocities and simulation points have inconsistent sizes.");
                        return false;
                    }
                    convert(deformableDesc.simulationVelocities, simulationVelocities);
                }

                VtArray<int32_t> simulationIndices;
                SafeGetAttributeP(&simulationIndices, deformableApi.GetSimulationIndicesAttr());
                if (!convertIndexBuffer(deformableDesc.simulationIndices, simulationIndices, numVertsPerElement, int32_t(numSimPoints)))
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableAPI:simulationindices has invalid size or contains out of range indices.");
                    return false;
                }

                deformableDesc.scenePath = SdfPath();
                if (UsdRelationship rel = deformableApi.GetSimulationOwnerRel())
                {
                    SdfPathVector paths;
                    if (rel.GetTargets(&paths) && paths.size() > 0)
                    {
                        deformableDesc.scenePath = paths[0].GetPrimPath();
                        if (paths.size() > 1)
                        {
                            // Warn only one scene is supported for deformables
                            CARB_LOG_WARN("PhysxSchemaPhysxDeformableAPI: Only 1 simulation owner is supported. Scene %s will be used as the default.", deformableDesc.scenePath.GetText());
                        }
                    }
                }

                // Collision contact and rest offsets are handled in the createDeformableBody function
                // If either one is -inf, the default values are computed
                if (!deformablePrim.HasAPI<PhysxSchemaPhysxCollisionAPI>())
                {
                    PhysxSchemaPhysxCollisionAPI::Apply(deformablePrim);
                }
                const PhysxSchemaPhysxCollisionAPI collisionParameters(deformablePrim);
                collisionParameters.GetContactOffsetAttr().Get(&deformableDesc.collisionContactOffset);
                collisionParameters.GetRestOffsetAttr().Get(&deformableDesc.collisionRestOffset);

                deformableDesc.collisionGroup = getCollisionGroup(attachedStage, deformablePrim.GetPrimPath());

                // filteredPairs
                UsdPhysicsFilteredPairsAPI filteredPairsAPI(deformablePrim);
                if (filteredPairsAPI && filteredPairsAPI.GetFilteredPairsRel())
                {
                    filteredPairsAPI.GetFilteredPairsRel().GetTargets(&deformableDesc.filteredCollisions);
                }

                // materials
                const SdfPath& materialPath = usdmaterialutils::getMaterialBinding(deformablePrim);
                if (materialPath != SdfPath())
                {
                    deformableDesc.materials.push_back(getMaterial(attachedStage, materialPath, materialType));
                }
                return true;
            }

            SoftBodyDesc* ParseDeformableBodyDeprecated(AttachedStage& attachedStage, const UsdPrim& usdPrim)
            {
                if (!usdPrim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI parsing: UsdPrim doesn't have a PhysxSchemaPhysxDeformableBodyAPI.");
                    return nullptr;
                }

                if (!usdPrim.HasAPI<PhysxSchemaPhysxDeformableAPI>())
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI parsing: UsdPrim doesn't have a PhysxSchemaPhysxDeformableAPI.");
                    return nullptr;
                }

                if (!usdPrim.IsA<UsdGeomMesh>())
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI parsing: UsdPrim isn't a UsdGeomMesh.");
                    return nullptr;
                }

                PhysxSchemaPhysxDeformableBodyAPI deformableBody(usdPrim);

                // create and fill in descriptor
                SoftBodyDesc* softBodyDesc = ICE_PLACEMENT_NEW(SoftBodyDesc)();
                if (!softBodyDesc)
                {
                    return nullptr;
                }

                bool isValid = true;

                // load simulation mesh data
                VtArray<GfVec3f> simulationRestPoints;
                SafeGetAttributeP(&simulationRestPoints, deformableBody.GetSimulationRestPointsAttr());
                convert(softBodyDesc->simulationRestPoints, simulationRestPoints);

                UsdAttribute simulationPointsAttr = deformableBody.GetSimulationPointsAttr();
                if (simulationPointsAttr.HasAuthoredValue())
                {
                    VtArray<GfVec3f> simulationPoints;
                    SafeGetAttributeP(&simulationPoints, simulationPointsAttr);
                    convert(softBodyDesc->simulationPoints, simulationPoints);
                }
                else
                {
                    deformableBody.CreateSimulationPointsAttr().Set(simulationRestPoints);
                    softBodyDesc->simulationPoints = softBodyDesc->simulationRestPoints;
                }

                if (softBodyDesc->simulationRestPoints.size() != softBodyDesc->simulationPoints.size())
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI:simulationRestPoints and :simulationPoints have inconsistent sizes.");
                    isValid = false;
                }

                if (!ParseDeformableDescDeprecated(attachedStage, *softBodyDesc, usdPrim, eSoftBodyMaterial,
                                         4, uint32_t(softBodyDesc->simulationPoints.size())))
                {
                    isValid = false;
                }

                // only parse mass property if it was provided
                UsdPhysicsMassAPI massApi(usdPrim);
                const UsdAttribute massAttribute = massApi.GetMassAttr();
                if (massAttribute.HasAuthoredValue())
                {
                    massAttribute.Get(&softBodyDesc->mass);
                }
                const UsdAttribute densityAttribute = massApi.GetDensityAttr();
                if (densityAttribute.HasAuthoredValue())
                {
                    densityAttribute.Get(&softBodyDesc->density);
                }

                // load collision mesh data
                VtArray<GfVec3f> collisionRestPoints;
                SafeGetAttributeP(&collisionRestPoints, deformableBody.GetCollisionRestPointsAttr());
                convert(softBodyDesc->collisionRestPoints, collisionRestPoints);

                UsdAttribute collisionPointsAttr = deformableBody.GetCollisionPointsAttr();
                if (collisionPointsAttr.HasAuthoredValue())
                {
                    VtArray<GfVec3f> collisionPoints;
                    SafeGetAttributeP(&collisionPoints, collisionPointsAttr);
                    convert(softBodyDesc->collisionPoints, collisionPoints);
                }
                else
                {
                    deformableBody.CreateCollisionPointsAttr().Set(collisionRestPoints);
                    softBodyDesc->collisionPoints = softBodyDesc->collisionRestPoints;
                }

                if (softBodyDesc->collisionRestPoints.size() != softBodyDesc->collisionPoints.size())
                {
                    CARB_LOG_WARN(
                        "PhysxSchemaPhysxDeformableBodyAPI:collisionRestPoints and collisionPoints have inconsistent sizes.");
                    isValid = false;
                }

                VtArray<int32_t> collisionIndices;
                SafeGetAttributeP(&collisionIndices, deformableBody.GetCollisionIndicesAttr());
                bool collMeshValid = convertIndexBuffer(softBodyDesc->collisionIndices, collisionIndices, 4, int32_t(softBodyDesc->collisionPoints.size()));
                if (!collMeshValid)
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI:collisionindices has invalid size or contains out of range indices.");
                    isValid = false;
                }

                static const TfToken kinematicEnabledToken("physxDeformable:kinematicEnabled");
                SafeGetAttributeP(&softBodyDesc->kinematicBody, usdPrim.GetAttribute(kinematicEnabledToken));

                static const TfToken deformableCollisionVertexToSimulationTetIndicesToken("physxDeformable:collisionVertexToSimulationTetIndices");
                VtArray<int> collisionVertexToSimulationTetIndices;
                SafeGetAttributeP(&collisionVertexToSimulationTetIndices, usdPrim.GetAttribute(deformableCollisionVertexToSimulationTetIndicesToken));

                bool embeddingValid = true;
                if (collisionVertexToSimulationTetIndices.size())
                {
                    const size_t numColl = softBodyDesc->collisionPoints.size();
                    const size_t numSkin = softBodyDesc->points.size();
                    const size_t numMap = softBodyDesc->kinematicBody ? numColl : numColl + numSkin;
                    embeddingValid = (collisionVertexToSimulationTetIndices.size() == numMap);

                    const int32_t numSimTets = int32_t(softBodyDesc->simulationIndices.size() / 4);
                    embeddingValid &= convertIndexBuffer(softBodyDesc->collisionVertexToSimulationTetIndices,
                                                         collisionVertexToSimulationTetIndices, 1, numSimTets);
                }
                if (!embeddingValid)
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI:collisionVertexToSimulationTetIndices: Embedding is invalid.");
                    isValid = false;
                }

                bool kinematicValid = true;
                if (softBodyDesc->kinematicBody)
                {
                    if (softBodyDesc->simulationPoints.size() != softBodyDesc->collisionPoints.size() ||
                        softBodyDesc->simulationIndices.size() != softBodyDesc->collisionIndices.size())
                    {
                        CARB_LOG_WARN(
                            "PhysxSchemaPhysxDeformableBodyAPI kinematic configuration requires simulation mesh to be equal to collision mesh.");
                        isValid = false;
                    }

                    static const TfToken deformableCollisionVertexToSkinTriVertexIndicesToken("physxDeformable:collisionVertexToSkinTriVertexIndices");
                    VtArray<uint32_t> collisionVertexToSkinTriVertexIndices;
                    SafeGetAttributeP(&collisionVertexToSkinTriVertexIndices, usdPrim.GetAttribute(deformableCollisionVertexToSkinTriVertexIndicesToken));

                    static const TfToken deformableCollisionVertexToSkinTriBarycentricsToken("physxDeformable:collisionVertexToSkinTriBarycentrics");
                    VtArray<GfVec3f> collisionVertexToSkinTriBarycentrics;
                    SafeGetAttributeP(&collisionVertexToSkinTriBarycentrics, usdPrim.GetAttribute(deformableCollisionVertexToSkinTriBarycentricsToken));

                    if (collisionVertexToSkinTriVertexIndices.size()/3 != collisionVertexToSkinTriBarycentrics.size())
                    {
                        CARB_LOG_WARN(
                            "PhysxSchemaPhysxDeformableBodyAPI:collisionVertexToSkinTriVertexIndices and :collisionVertexToSkinTriBarycentrics have inconsistent sizes.");
                        isValid = false;
                    }

                    bool mapValid = true;
                    if (collisionVertexToSkinTriVertexIndices.size() > 0)
                    {
                        const size_t numColl = softBodyDesc->collisionPoints.size();
                        // only surface collision vertices are mapped, internal vertices not.
                        mapValid &= (collisionVertexToSkinTriVertexIndices.size() / 3 <= numColl);

                        softBodyDesc->collisionVertexToSkinTriVertexIndices.resize(collisionVertexToSkinTriVertexIndices.size() / 3);
                        mapValid &= convertIndexBuffer((uint32_t*)softBodyDesc->collisionVertexToSkinTriVertexIndices.data(),
                                                       collisionVertexToSkinTriVertexIndices.data(),
                                                       softBodyDesc->collisionVertexToSkinTriVertexIndices.size() * 3, 3,
                                                       uint32_t(softBodyDesc->points.size()));


                        convert(softBodyDesc->collisionVertexToSkinTriBarycentrics, collisionVertexToSkinTriBarycentrics);
                    }
                    else
                    {
                        mapValid = (softBodyDesc->collisionPoints.size() == softBodyDesc->points.size());
                    }
                    if (!mapValid)
                    {
                        CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI:collisionVertexToSkinTriVertexIndices mapping is invalid.");
                        isValid = false;
                    }
                }

                static const TfToken simulationHexahedralResolutionToken("physxDeformable:simulationHexahedralResolution");
                SafeGetAttributeP(&softBodyDesc->simulationHexahedralResolution, usdPrim.GetAttribute(simulationHexahedralResolutionToken));

                static const TfToken deformableNumberOfTetsPerHexToken("physxDeformable:numberOfTetsPerHex");
                SafeGetAttributeP(&softBodyDesc->numberOfTetsPerHex, usdPrim.GetAttribute(deformableNumberOfTetsPerHexToken), 6u);

                SafeGetAttributeP(&softBodyDesc->deformableEnabled, PhysxSchemaPhysxDeformableAPI(deformableBody).GetDeformableEnabledAttr());

                // Parse for custom attribute inv mass scale
                static const TfToken invMassScaleToken("physxDeformable:invMassScale");
                VtArray<float> invMassScale;
                SafeGetAttributeP(&invMassScale, usdPrim.GetAttribute(invMassScaleToken));
                if (invMassScale.size() != collisionRestPoints.size())
                {
                    invMassScale.assign(collisionRestPoints.size(), 1.0f);
                }
                convert(softBodyDesc->invMassScale, invMassScale);

                // checking for time varying
                const bool isTransformTimeVarying = primutils::IsTransformTimeVarying(usdPrim);
                const UsdGeomMesh usdMesh(usdPrim);
                const bool isMeshTimeVarying = usdMesh.GetPointsAttr().GetNumTimeSamples() > 1;

                if (isTransformTimeVarying || isMeshTimeVarying)
                {
                    if (!softBodyDesc->kinematicBody)
                    {
                        CARB_LOG_WARN(
                            "Detected deformable body that is not kinematic but does have parents with animated xformOps or animated mesh vertices. Prim: %s, converting the deformable body to kinematic.", usdPrim.GetPrimPath().GetText());
                        softBodyDesc->kinematicBody = true;
                    }
                    attachedStage.getAnimatedKinematicDeformableBodies()[usdPrim.GetPrimPath()] = usdPrim;
                }

                if (!isValid)
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableBodyAPI parsing failed: %s.", usdPrim.GetPrimPath().GetText());
                    ICE_FREE(softBodyDesc);
                    return nullptr;
                }
                return softBodyDesc;
            }

            FEMClothDesc* ParseDeformableSurfaceDeprecated(AttachedStage& attachedStage, const UsdPrim& usdPrim)
            {
                if (!usdPrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableSurfaceAPI parsing: UsdPrim doesn't have a PhysxSchemaPhysxDeformableSurfaceAPI.");
                    return nullptr;
                }

                if (!usdPrim.HasAPI<PhysxSchemaPhysxDeformableAPI>())
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableSurfaceAPI parsing: UsdPrim doesn't have a PhysxSchemaPhysxDeformableAPI.");
                    return nullptr;
                }

                if (!usdPrim.IsA<UsdGeomMesh>())
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableSurfaceAPI parsing: UsdPrim isn't a UsdGeomMesh.");
                    return nullptr;
                }

                // create and fill in descriptor
                FEMClothDesc* femClothDesc = ICE_PLACEMENT_NEW(FEMClothDesc)();
                if (!femClothDesc)
                {
                    return nullptr;
                }

                bool isValid = true;

                PhysxSchemaPhysxDeformableAPI deformableApi(usdPrim);
                PhysxSchemaPhysxDeformableSurfaceAPI femClothApi(usdPrim);

                SafeGetAttributeP(&femClothDesc->deformableEnabled, PhysxSchemaPhysxDeformableAPI(femClothApi).GetDeformableEnabledAttr());

                // bending
                femClothApi.GetFlatteningEnabledAttr().Get(&femClothDesc->flatteningEnabled);
                
                // collision substepping
                femClothApi.GetCollisionPairUpdateFrequencyAttr().Get(&femClothDesc->collisionPairUpdateFrequency);
                femClothApi.GetCollisionIterationMultiplierAttr().Get(&femClothDesc->collisionIterationMultiplier);
                
                // velocity clamping
                femClothApi.GetMaxVelocityAttr().Get(&femClothDesc->maxVelocity);
                deformableApi.GetMaxDepenetrationVelocityAttr().Get(&femClothDesc->maxDepenetrationVelocity);

                //For deformable surfaces we don't have explicit sim points in USD, sim velocities should also not exist, but they
                //are part of the base schema. For now we just store sim velocities corresponding to the mesh points, although
                //by right we should store the velocities in the point based velocities...
                //The new schema has a separate sim mesh if welding takes place, which removes this ambiguity.
                VtArray<GfVec3f> points;
                UsdGeomMesh(usdPrim).GetPointsAttr().Get(&points);
                if (!ParseDeformableDescDeprecated(attachedStage, *femClothDesc, usdPrim, eFEMClothMaterial, 3, uint32_t(points.size())))
                {
                    isValid = false;
                }

                // only parse mass property if it was provided
                UsdPhysicsMassAPI massApi(usdPrim);
                const UsdAttribute massAttribute = massApi.GetMassAttr();
                if (massAttribute.HasAuthoredValue())
                {
                    massAttribute.Get(&femClothDesc->mass);
                }
                const UsdAttribute densityAttribute = massApi.GetDensityAttr();
                if (densityAttribute.HasAuthoredValue())
                {
                    densityAttribute.Get(&femClothDesc->density);
                }

                femClothDesc->collisionGroup = getCollisionGroup(attachedStage, usdPrim.GetPrimPath());

                if (!isValid)
                {
                    CARB_LOG_WARN("PhysxSchemaPhysxDeformableSurfaceAPI parsing failed: %s.", usdPrim.GetPrimPath().GetText());
                    ICE_FREE(femClothDesc);
                    return nullptr;
                }

                return femClothDesc;
            }

        } // namespace usdparser
    } // namespace physx
} // namespace omni

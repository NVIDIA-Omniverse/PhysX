// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/usd/PathConversion.h>

#include "ImmediateNode.h"
#include "ImmediateShared.h"

#include <omni/graph/core/BundlePrims.h>
#include <omni/graph/core/ogn/Bundle.h>

using namespace omni::graph::core;
using namespace ::physx;
using namespace omni::physx::graph;
using namespace pxr;
using namespace omni::fabric;


ImmediateShared& ImmediateNode::getGlobals()
{
    static ImmediateShared globals;
    return globals;
}

bool ImmediateNode::fillMeshInputViewFromBundle(ogn::OmniGraphDatabase& db,
                                                IConstBundle2* inputBundle,
                                                std::vector<MeshInputView>& inputMeshView,
                                                std::unordered_map<omni::graph::core::NameToken, size_t>* primToIndicesMap)
{
    const GraphContextObj& context = db.abi_context();
    const long stageId = context.iContext->getStageId(context);

    omni::graph::core::BackendId backendId;
    omni::graph::core::GraphObj graphObj = context.iContext->getGraph(context);
    graphObj.iGraph->getBackendId(graphObj, backendId);
    omni::fabric::FabricId fabricId(backendId.id);

    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
    const double metersPerUnit = UsdGeomGetStageMetersPerUnit(stage);

    const size_t childrenCount = inputBundle->getChildBundleCount();
    inputMeshView.clear();
    for (size_t i = 0; i < childrenCount; ++i)
    {
        ConstBundleHandle child = inputBundle->getConstChildBundle(i);
        if (!child.isValid())
        {
            db.logError("Input must be a valid child bundle!");
            return false;
        }
        ogn::BundleContents<ogn::kOgnInput, ogn::kCpu> childBundle(context, child);

        if (!validateMeshBundle(db, childBundle))
        {
            return false;
        }

        const auto primPathAttribute = childBundle.attributeByName(omni::fabric::Token::createImmortal("sourcePrimPath"));
        Token primPath = *primPathAttribute.getCpu<Token>();
        NameToken tokenPath = primPath;

        ConstAttributeDataHandle pointsAttr = getAttributeR(context, child, omni::fabric::Token::createImmortal("points"));
        ConstAttributeDataHandle facesAttr = getAttributeR(context, child, omni::fabric::Token::createImmortal("faceVertexCounts"));
        ConstAttributeDataHandle indicesAttr = getAttributeR(context, child, omni::fabric::Token::createImmortal("faceVertexIndices"));
        ConstAttributeDataHandle holesAttr = getAttributeR(context, child, omni::fabric::Token::createImmortal("holeIndices"));
        ConstAttributeDataHandle matrixAttr = getAttributeR(context, child, omni::fabric::Token::createImmortal("worldMatrix"));
        ConstAttributeDataHandle approximationAttr =
            getAttributeR(context, child, omni::fabric::Token::createImmortal("physics:approximation"));
        ConstAttributeDataHandle orientationAttr = getAttributeR(context, child, omni::fabric::Token::createImmortal("orientation"));
        ConstAttributeDataHandle meshKeyAttr = getAttributeR(context, child, omni::fabric::Token::createImmortal("meshKey"));

        if (pointsAttr.isValid() && facesAttr.isValid() && indicesAttr.isValid() && matrixAttr.isValid())
        {
            // Mandatory attributes
            const carb::Float3* const* points = getDataR<carb::Float3*>(context, pointsAttr);
            const int32_t* const* faces = getDataR<int32_t*>(context, facesAttr);
            const int32_t* const* indices = getDataR<int32_t*>(context, indicesAttr);
            const auto worldMatrix = getDataR<double[16]>(context, matrixAttr);
            // Optional attributes
            const int32_t* const* holeIndices = getDataR<int32_t*>(context, holesAttr);
            const auto orientation = getDataR<NameToken>(context, orientationAttr);
            const auto approximation = getDataR<NameToken>(context, approximationAttr);
            const auto meshKey = getDataR<uint64_t*>(context, meshKeyAttr);
            if (points && faces && indices && worldMatrix)
            {
                int index = (int)inputMeshView.size();
                MeshInputView mesh;
                mesh.meshView.primStageId = stageId;
                mesh.meshView.primTokenId = omni::fabric::fabricTokenToHandle(tokenPath);
                mesh.meshView.primId = omni::fabric::convertToPathType<omni::fabric::Path>(fabricId, tokenPath.getText());
                if (approximation)
                {
                    mesh.meshCollision.collisionApproximation = omni::fabric::fabricTokenToHandle(*approximation);
                }
                mesh.meshCollision.metersPerUnit = metersPerUnit;

                if (meshKey)
                {
                    size_t count = 0;
                    context.iAttributeData->getElementCount(&count, context, &meshKeyAttr, 1);
                    if (count == 2)
                    {
                        memcpy(&mesh.meshKey, *meshKey, sizeof(MeshKey));
                    }
                }

                size_t elementCount = 0;
                context.iAttributeData->getElementCount(&elementCount, context, &pointsAttr, 1);
                mesh.meshView.cookingMeshView.points = { *points, elementCount };
                context.iAttributeData->getElementCount(&elementCount, context, &facesAttr, 1);
                mesh.meshView.cookingMeshView.faces = { *faces, elementCount };
                context.iAttributeData->getElementCount(&elementCount, context, &indicesAttr, 1);
                mesh.meshView.cookingMeshView.indices = { *indices, elementCount };

                // Matrix
                memcpy(&mesh.worldMatrix, *worldMatrix, 16 * sizeof(double));
                GfTransform transform(mesh.worldMatrix);
                GfVec3d meshScale = transform.GetScale();

                if (meshScale[0] < 0 || meshScale[1] < 0 || meshScale[2] < 0)
                {
                    // If we have negative mesh scale, let's track it in signScale
                    mesh.meshView.signScale = { meshScale[0] >= 0 ? 1.f : -1.f, // x
                                                meshScale[1] >= 0 ? 1.f : -1.f, // y
                                                meshScale[2] >= 0 ? 1.f : -1.f }; // z
                    transform.SetScale(meshScale);
                    GfMatrix4d newMat = transform.GetMatrix();
                    memcpy(&mesh.worldMatrix, newMat.data(), 16 * sizeof(double));
                }

                // Mesh Orientation
                mesh.meshView.cookingMeshView.rightHandedOrientation = true;
                if (orientationAttr.isValid())
                {
                    mesh.meshView.cookingMeshView.rightHandedOrientation =
                        *orientation == omni::fabric::Token::createImmortal("rightHanded");
                }
                if (holesAttr.isValid())
                {
                    context.iAttributeData->getElementCount(&elementCount, context, &holesAttr, 1);
                    mesh.meshView.cookingMeshView.holeIndices = { *holeIndices, elementCount };
                }
                if (primToIndicesMap)
                {
                    (*primToIndicesMap)[tokenPath] = inputMeshView.size();
                }     
                inputMeshView.push_back(mesh);
            }
            else
            {
                db.logError(
                    "Attributes (points, faceVertexCounts, faceVertexIndices, worldMatrix) for \"%s\" are empty or have wrong type",
                    db.tokenToString(primPath));
                return false;
            }
        }
        else
        {
            db.logError(
                "Input child prim \"%s\" is missing one following attributes: points, faceVertexCounts, faceVertexIndices, worldMatrix",
                db.tokenToString(primPath));
            return false;
        }
    }
    return true;
}


bool ImmediateNode::fillMeshInputViewFromTargets(ogn::OmniGraphDatabase& db,
                                                omni::graph::core::ogn::ArrayInput<const TargetPath, ogn::kCpu>& targets,
                                                std::vector<MeshInputView>& inputMeshView)
{
    const GraphContextObj& context = db.abi_context();
    const long stageId = context.iContext->getStageId(context);

    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
    const double metersPerUnit = UsdGeomGetStageMetersPerUnit(stage);

    UsdGeomXformCache xfCache;
    inputMeshView.clear();
    const size_t numTargets = targets.size();
    for (size_t index = 0; index < numTargets; index++)
    {
        const TargetPath& target = targets()[index];
        const UsdPrim targetPrim = stage->GetPrimAtPath(toSdfPath(target));
        if(!targetPrim)
        {
            db.logError("Unable to locate prim at path: %s", toSdfPath(target).GetName());
            continue;
        }
        if(!targetPrim.IsValid())
        {
            db.logError("Input prim is invalid: %s", toSdfPath(target).GetName());
            continue;
        }
        if(!targetPrim.IsA<UsdGeomMesh>())
        {
            db.logError("Input prim is not a mesh: %s", toSdfPath(target).GetName());
            continue;
        }

        MeshInputView mesh;
        mesh.meshView.usePrimID = true;
        mesh.meshView.primStageId = stageId;
        mesh.meshView.primId = target;
        mesh.meshCollision.metersPerUnit = metersPerUnit;

        // Get approximation type if specified.
        TfToken approximationType = UsdPhysicsTokens.Get()->none;
        const UsdPhysicsMeshCollisionAPI colMeshAPI = UsdPhysicsMeshCollisionAPI::Get(stage, toSdfPath(target));
        if (colMeshAPI)
        {
            colMeshAPI.GetApproximationAttr().Get(&approximationType);
            mesh.meshCollision.collisionApproximation = omni::fabric::tfTokenToHandle(approximationType);
        }
        
        mesh.worldMatrix = xfCache.GetLocalToWorldTransform(targetPrim);
        GfTransform transform(mesh.worldMatrix);
        GfVec3d meshScale = transform.GetScale();

        if (meshScale[0] < 0 || meshScale[1] < 0 || meshScale[2] < 0)
        {
            // If we have negative mesh scale, let's track it in signScale
            mesh.meshView.signScale = { meshScale[0] >= 0 ? 1.f : -1.f, // x
                                        meshScale[1] >= 0 ? 1.f : -1.f, // y
                                        meshScale[2] >= 0 ? 1.f : -1.f }; // z
            transform.SetScale(meshScale);
            GfMatrix4d newMat = transform.GetMatrix();
            memcpy(&mesh.worldMatrix, newMat.data(), 16 * sizeof(double));
        }

        inputMeshView.push_back(mesh);
    }
    return true;
}


bool ImmediateNode::fillBoxOverlapPairsVector(omni::graph::core::ogn::OmniGraphDatabase& db,
                                              omni::graph::core::ogn::ArrayInput<const NameToken, ogn::kCpu>& overlapsPair0,
                                              omni::graph::core::ogn::ArrayInput<const NameToken, ogn::kCpu>& overlapsPair1,
                                              std::vector<PxGeomIndexPair>& boxesOverlapPairs,
                                              const std::unordered_map<NameToken, size_t>& namesToIndices)
{
    const size_t numOverlapTests = overlapsPair0.size();
    const size_t numOverlapTests1 = overlapsPair1.size();
    if (numOverlapTests != numOverlapTests1)
    {
        db.logError("overlapsPair0 and overlapsPair1 input arrays must have same size");
        return false;
    }
    boxesOverlapPairs.resize(numOverlapTests);

    for (size_t idx = 0; idx < numOverlapTests; ++idx)
    {
        NameToken body0 = overlapsPair0()[idx];
        NameToken body1 = overlapsPair1()[idx];
        // Body0 and body1 are input as Tokens, so we search their corresponding index in the clash detection flat
        // mesh array
        auto it0 = namesToIndices.find(body0);
        auto it1 = namesToIndices.find(body1);
        if (it0 == namesToIndices.end())
        {
            db.logError("Mesh '%s' was not found in input bundle", db.tokenToString(body0));
            return false;
        }
        else if (it1 == namesToIndices.end())
        {
            db.logError("Mesh '%s' was not found in input bundle", db.tokenToString(body1));
            return false;
        }
        else
        {
            boxesOverlapPairs[idx] = PxGeomIndexPair((PxU32)it0->second, (PxU32)it1->second);
        }
    }
    return true;
}


bool ImmediateNode::validateMeshBundle(ogn::OmniGraphDatabase& db,
                                       const ogn::BundleContents<ogn::kOgnInput, ogn::kCpu>& childBundle)
{
    const auto primPathAttribute = childBundle.attributeByName(omni::fabric::Token::createImmortal("sourcePrimPath"));
    const auto primTypeAttribute = childBundle.attributeByName(omni::fabric::Token::createImmortal("sourcePrimType"));

    if (!primPathAttribute.isValid() || !primTypeAttribute.isValid())
    {
        db.logError("Input is not a bundle of child bundles!");
        return false;
    }
    Token primType = *primTypeAttribute.getCpu<Token>();
    Token primPath = *primPathAttribute.getCpu<Token>();
    NameToken tokenPath = primPath;
    if (primType != omni::fabric::Token::createImmortal("Mesh"))
    {
        db.logError("'%s' is not a Mesh Prim type", db.tokenToString(primPath));
        return false;
    }

    return true;
}


void ImmediateNode::checkCookingWarnings(omni::graph::core::ogn::OmniGraphDatabase& db,
                                         const std::vector<MeshInputView>& meshes,
                                         const std::vector<MeshCookedData>& meshCookedData)
{
    for (size_t idx = 0; idx < meshes.size(); ++idx)
    {
        const MeshCookedData& mcd = meshCookedData[idx];
        if (!mcd.isValid)
        {
            const SdfPath path = omni::fabric::toSdfPath(meshes[idx].meshView.primId);
            db.logError("Cooking for Prim '%s' has failed", path.GetName());
        }
    }
}

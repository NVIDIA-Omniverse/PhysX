// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>
#include <common/utilities/UsdMaterialParsing.h>

#include "Collision.h"
#include "CollisionGroup.h"
#include "FilteredPairs.h"

using namespace pxr;

namespace omni
{
namespace physics
{
namespace schema
{
const double tolerance = 1e-4;

void checkNonUniformScale(const GfVec3d& scale, const SdfPath& primPath)
{
    if (abs(scale[0] - scale[1]) > tolerance || abs(scale[0] - scale[2]) > tolerance || abs(scale[2] - scale[1]) > tolerance)
    {
        CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", primPath.GetText());
    }
}

ShapeDesc* parseCollisionInternal(const UsdStageWeakPtr stage, UsdGeomXformCache& xfCache,
        const UsdPrim& usdPrim, const TokenVector& customTokens, const TfTokenVector& apis)
{
    ShapeDesc* desc = nullptr;

    // custom shape handling
    bool customShape = false;
    {
        const TfToken& primType = usdPrim.GetTypeName();
        for (size_t i = 0; i < customTokens.size(); i++)
        {
            for (size_t j = 0; j < apis.size(); j++)
            {
                if (apis[j] == customTokens[i])
                {
                    CustomShapeDesc* cd = new CustomShapeDesc();
                    desc = cd;
                    cd->customGeometryToken = apis[j];
                    break;
                }
            }
            if(desc)
            {
                break;
            }
            if (primType == customTokens[i])
            {
                CustomShapeDesc* cd = new CustomShapeDesc();
                desc = cd;
                cd->customGeometryToken = primType;
                break;
            }
        }
    }

    // if we are getting stuff in a desc cache we scale later once instanced
    const bool isMaster = usdPrim.IsInPrototype();
    const GfTransform tr(isMaster ? GfMatrix4d(1.0) : xfCache.GetLocalToWorldTransform(usdPrim));

    // geomgprim that belong to that collision
    if (!desc && usdPrim.IsA<UsdGeomGprim>())
    {
        // If the primitive is a UsdGeomPoints *and* it has a widths attribute
        // corresponding to the positions attribute, then we treat it as an
        // array of spheres corresponding to the 'SpherePointsShapeDesc'
        if (usdPrim.IsA<UsdGeomPoints>())
        {
            UsdGeomPoints shape(usdPrim);
            VtArray<float> widths;
            VtArray<GfVec3f> positions;
            shape.GetWidthsAttr().Get(&widths);
            if ( widths.size() )
            {
                shape.GetPointsAttr().Get(&positions);
                if ( positions.size() == widths.size() )
                {
                    float sphereScale = 1.0f;
                    {
                        const GfVec3d sc = tr.GetScale();
                        // as we don't support scale in physics and scale can be non uniform
                        // we pick the largest scale value as the sphere radius base
                        checkNonUniformScale(sc, usdPrim.GetPrimPath());
                        sphereScale = fmaxf(fmaxf(fabsf(float(sc[1])), fabsf(float(sc[0]))), fabsf(float(sc[2])));
                    }
                    SpherePointsShapeDesc *sdesc = new SpherePointsShapeDesc;
                    size_t scount = positions.size();
                    sdesc->spherePoints.resize(scount);
                    for (size_t i=0; i<scount; i++)
                    {
                        sdesc->spherePoints[i].radius = sphereScale*widths[i]*0.5f;
                        sdesc->spherePoints[i].center = positions[i];
                    }
                    desc = static_cast< ShapeDesc *>(sdesc);
                }
            }
        }
        else if (usdPrim.IsA<UsdGeomCube>())
        {
            GfVec3f halfExtents;

            {
                const GfVec3d sc = tr.GetScale();
                // scale is taken, its a part of the cube size, as the physics does not support scale
                halfExtents = GfVec3f(sc);
            }

            // Get shape parameters
            {
                UsdGeomCube shape(usdPrim);
                double sizeAttr;
                shape.GetSizeAttr().Get(&sizeAttr);
                sizeAttr = abs(sizeAttr) * 0.5f; // convert cube edge length to half extend
                halfExtents *= (float)sizeAttr;
            }

            desc = new CubeShapeDesc(halfExtents);
        }
        else if (usdPrim.IsA<UsdGeomSphere>())
        {
            float radius = 1.0f;

            {
                const GfVec3d sc = tr.GetScale();
                // as we dont support scale in physics and scale can be non uniform
                // we pick the largest scale value as the sphere radius base
                checkNonUniformScale(sc, usdPrim.GetPrimPath());
                radius = fmaxf(fmaxf(fabsf(float(sc[1])), fabsf(float(sc[0]))), fabsf(float(sc[2])));
            }

            // Get shape parameters
            {
                UsdGeomSphere shape(usdPrim);
                double radiusAttr;
                shape.GetRadiusAttr().Get(&radiusAttr);
                radius *= (float)radiusAttr;
            }

            desc = new SphereShapeDesc(fabsf(radius));
        }
        else if (usdPrim.IsA<UsdGeomCapsule>())
        {
            float radius = 1.0f;
            float halfHeight = 1.0f;
            Axis::Enum axis = Axis::eX;

            // Get shape parameters
            {
                UsdGeomCapsule shape(usdPrim);
                double radiusAttr;
                shape.GetRadiusAttr().Get(&radiusAttr);
                double heightAttr;
                shape.GetHeightAttr().Get(&heightAttr);
                radius = (float)radiusAttr;
                halfHeight = (float)heightAttr * 0.5f;

                TfToken capAxis;
                if (shape.GetAxisAttr())
                {
                    shape.GetAxisAttr().Get(&capAxis);
                    if (capAxis == UsdPhysicsTokens.Get()->y)
                        axis = Axis::eY;
                    else if (capAxis == UsdPhysicsTokens.Get()->z)
                        axis = Axis::eZ;
                }
            }

            {
                // scale the radius and height based on the given axis token
                const GfVec3d sc = tr.GetScale();
                checkNonUniformScale(sc, usdPrim.GetPrimPath());
                if (axis == Axis::eX)
                {
                    halfHeight *= float(sc[0]);
                    radius *= fmaxf(fabsf(float(sc[1])), fabsf(float(sc[2])));
                }
                else if (axis == Axis::eY)
                {
                    halfHeight *= float(sc[1]);
                    radius *= fmaxf(fabsf(float(sc[0])), fabsf(float(sc[2])));
                }
                else
                {
                    halfHeight *= float(sc[2]);
                    radius *= fmaxf(fabsf(float(sc[1])), fabsf(float(sc[0])));
                }
            }


            desc = new CapsuleShapeDesc(fabsf(radius), fabsf(halfHeight), axis);
        }
        else if (usdPrim.IsA<UsdGeomCylinder>())
        {
            float radius = 1.0f;
            float halfHeight = 1.0f;
            Axis::Enum axis = Axis::eX;

            // Get shape parameters
            {
                UsdGeomCylinder shape(usdPrim);
                double radiusAttr;
                shape.GetRadiusAttr().Get(&radiusAttr);
                double heightAttr;
                shape.GetHeightAttr().Get(&heightAttr);
                radius = (float)radiusAttr;
                halfHeight = (float)heightAttr * 0.5f;

                TfToken capAxis;
                if (shape.GetAxisAttr())
                {
                    shape.GetAxisAttr().Get(&capAxis);
                    if (capAxis == UsdPhysicsTokens.Get()->y)
                        axis = Axis::eY;
                    else if (capAxis == UsdPhysicsTokens.Get()->z)
                        axis = Axis::eZ;
                }
            }

            {
                // scale the radius and height based on the scale
                const GfVec3d sc = tr.GetScale();

                if (axis == Axis::eX)
                {
                    halfHeight *= float(sc[0]);
                    if (abs(sc[2] - sc[1]) > tolerance)
                    {
                        CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", usdPrim.GetPrimPath().GetText());
                    }
                    radius *= fmaxf(fabsf(float(sc[1])), fabsf(float(sc[2])));
                }
                else if (axis == Axis::eY)
                {
                    halfHeight *= float(sc[1]);
                    if (abs(sc[2] - sc[0]) > tolerance)
                    {
                        CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", usdPrim.GetPrimPath().GetText());
                    }
                    radius *= fmaxf(fabsf(float(sc[0])), fabsf(float(sc[2])));
                }
                else
                {
                    halfHeight *= float(sc[2]);
                    if (abs(sc[0] - sc[1]) > tolerance)
                    {
                        CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", usdPrim.GetPrimPath().GetText());
                    }
                    radius *= fmaxf(fabsf(float(sc[1])), fabsf(float(sc[0])));
                }
            }


            desc = new CylinderShapeDesc(fabsf(radius), fabsf(halfHeight), axis);
        }
        else if (usdPrim.IsA<UsdGeomCone>())
        {
            float radius = 1.0f;
            float halfHeight = 1.0f;
            Axis::Enum axis = Axis::eX;

            // Get shape parameters
            {
                UsdGeomCone shape(usdPrim);
                double radiusAttr;
                shape.GetRadiusAttr().Get(&radiusAttr);
                double heightAttr;
                shape.GetHeightAttr().Get(&heightAttr);
                radius = (float)radiusAttr;
                halfHeight = (float)heightAttr * 0.5f;

                TfToken capAxis;
                if (shape.GetAxisAttr())
                {
                    shape.GetAxisAttr().Get(&capAxis);
                    if (capAxis == UsdPhysicsTokens.Get()->y)
                        axis = Axis::eY;
                    else if (capAxis == UsdPhysicsTokens.Get()->z)
                        axis = Axis::eZ;
                }
            }

            {
                // scale the radius and height based on the scale
                const GfVec3d sc = tr.GetScale();
                checkNonUniformScale(sc, usdPrim.GetPrimPath());
                if (axis == Axis::eX)
                {
                    halfHeight *= float(sc[0]);
                    radius *= fmaxf(fabsf(float(sc[1])), fabsf(float(sc[2])));
                }
                else if (axis == Axis::eY)
                {
                    halfHeight *= float(sc[1]);
                    radius *= fmaxf(fabsf(float(sc[0])), fabsf(float(sc[2])));
                }
                else
                {
                    halfHeight *= float(sc[2]);
                    radius *= fmaxf(fabsf(float(sc[1])), fabsf(float(sc[0])));
                }
            }

            desc = new ConeShapeDesc(fabsf(radius), fabsf(halfHeight), axis);
        }
        else if (usdPrim.IsA<UsdGeomMesh>())
        {
            const UsdGeomMesh& usdMesh = (const UsdGeomMesh&)(usdPrim);

            UsdTimeCode time = UsdTimeCode::Default();
            VtArray<GfVec3f>* pointsValue = new VtArray<GfVec3f>();
            {
                // test if the verts are there or if its time sampled
                {
                    usdMesh.GetPointsAttr().Get(pointsValue);
                    if (!pointsValue->size())
                    {
                        time = UsdTimeCode::EarliestTime();
                        usdMesh.GetPointsAttr().Get(pointsValue, time);
                    }
                }

                const uint32_t pointCount = uint32_t(pointsValue->size());
                if (pointCount < 3)
                {
                    delete pointsValue;
                    CARB_LOG_ERROR("Provided mesh geom with a PhysicsCollisionAPI does not have points, collision will not be created. Prim: %s", usdPrim.GetPrimPath().GetText());
                    return nullptr;
                }
            }

            VtArray<int>* indicesValue = new VtArray<int>();
            {
                usdMesh.GetFaceVertexIndicesAttr().Get(indicesValue, time);

                const uint32_t indicesCount = uint32_t(indicesValue->size());
                if (indicesCount < 3)
                {
                    delete indicesValue;
                    delete pointsValue;
                    CARB_LOG_ERROR("Provided mesh geom with a PhysicsCollisionAPI does not have indices, collision will not be created. Prim: %s", usdPrim.GetPrimPath().GetText());
                    return nullptr;
                }
            }

            VtArray<int>* facesValue = new VtArray<int>();
            {
                usdMesh.GetFaceVertexCountsAttr().Get(facesValue, time);

                const uint32_t facesCount = uint32_t(facesValue->size());
                if (facesCount == 0)
                {
                    delete facesValue;
                    delete indicesValue;
                    delete pointsValue;
                    CARB_LOG_ERROR("Provided mesh geom with a PhysicsCollisionAPI does not have faces, collision will not be created. Prim: %s", usdPrim.GetPrimPath().GetText());
                    return nullptr;
                }
            }

            MeshShapeDesc* meshDesc = new MeshShapeDesc();

            const GfVec3d sc = tr.GetScale();
            meshDesc->meshScale = GfVec3f(sc);

            meshDesc->points = pointsValue;
            meshDesc->faces = facesValue;
            meshDesc->indices = indicesValue;

            // Get approximation type
            meshDesc->approximation = UsdPhysicsTokens.Get()->none;
            UsdPhysicsMeshCollisionAPI physicsColMeshAPI = UsdPhysicsMeshCollisionAPI::Get(stage, usdPrim.GetPrimPath());
            if (physicsColMeshAPI)
                physicsColMeshAPI.GetApproximationAttr().Get(&meshDesc->approximation);

            usdMesh.GetDoubleSidedAttr().Get(&meshDesc->doubleSided);

            // gather materials though subsets
            const std::vector<UsdGeomSubset> subsets = UsdGeomSubset::GetGeomSubsets(usdMesh, UsdGeomTokens->face);
            if (!subsets.empty())
            {
                for (const UsdGeomSubset& subset : subsets)
                {
                    const SdfPath material = usdmaterialutils::getMaterialBinding(subset.GetPrim());
                    if (material != SdfPath())
                    {
                        const UsdPrim materialPrim = stage->GetPrimAtPath(material);
                        if (materialPrim && materialPrim.HasAPI<UsdPhysicsMaterialAPI>())
                            meshDesc->materials.push_back(material);
                    }
                }
            }

            desc = meshDesc;
        }
    }

    return desc;
}

void finalizeDesc(const UsdStageWeakPtr stage, const UsdPrim& colPrim, const UsdPhysicsCollisionAPI& colAPI, ShapeDesc& desc)
{
    // set the collider material as last
    // set SdfPath() anyway, this would indicate default material should be used, this is required for trimesh subset materials
    // as not alway all faces are covered with a subset material
    const SdfPath& materialPath = usdmaterialutils::getMaterialBinding(colPrim);
    if (materialPath != SdfPath())
    {
        const UsdPrim materialPrim = stage->GetPrimAtPath(materialPath);
        if (materialPrim && materialPrim.HasAPI<UsdPhysicsMaterialAPI>())
            desc.materials.push_back(materialPath);
        else
            desc.materials.push_back(SdfPath());
    }
    else
    {
        desc.materials.push_back(SdfPath());
    }

    parseFilteredPairs(stage, colPrim, desc.filteredCollisions);
    colAPI.GetCollisionEnabledAttr().Get(&desc.collisionEnabled);
    const UsdRelationship ownerRel = colAPI.GetSimulationOwnerRel();
    if (ownerRel)
    {
        ownerRel.GetTargets(&desc.simulationOwners);
    }
    desc.sourceGprim = colPrim;
}

// parse the given prim with collisionAPI
// simple geom types like sphere, cylinder, cube are parsed and send directly to physics
// complex geom type like a mesh is parsed based on additional meshCollisionAPI settings
// based on the approximation
void parseCollision(const UsdStageWeakPtr stage, UsdGeomXformCache& xfCache,
                          const UsdPrim& colPrim, const TokenVector& customTokens, const TfTokenVector& apis, std::vector<ShapeDesc*>& shapes)
{
    CARB_ASSERT(colPrim.HasAPI<UsdPhysicsCollisionAPI>());
    const UsdPhysicsCollisionAPI colAPI = UsdPhysicsCollisionAPI::Get(stage, colPrim.GetPrimPath());

    bool customgeom = false;
    const TfToken& primType = colPrim.GetTypeName();
    for (size_t i = 0; i < customTokens.size(); i++)
    {
        for (size_t j = 0; j < apis.size(); j++)
        {
            if (apis[j] == customTokens[i])
            {
                customgeom = true;
                break;
            }
        }
        if(customgeom)
        {
            break;
        }
        if (primType == customTokens[i])
        {
            customgeom = true;
            break;
        }
    }

    if (customgeom || colPrim.IsA<UsdGeomGprim>())
    {
        ShapeDesc* desc = parseCollisionInternal(stage, xfCache, colPrim, customTokens, apis);

        if (desc)
        {
            finalizeDesc(stage, colPrim, colAPI, *desc);

            shapes.push_back(desc);
        }
    }
    else
    {
        // all gprims
        UsdPrimRange range(colPrim, UsdTraverseInstanceProxies());
        for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const UsdPrim& usdPrim = (*iter);
            if (!usdPrim)
                continue;

            ShapeDesc* desc = parseCollisionInternal(stage, xfCache, usdPrim, customTokens, apis);

            // handle material
            if (desc)
            {
                finalizeDesc(stage, usdPrim, colAPI, *desc);

                shapes.push_back(desc);
            }
        }
    }

}

void finalizeCollision(const UsdStageWeakPtr stage, const RigidBodyDesc* bodyDesc,
    UsdGeomXformCache& xfCache, ShapeDesc* shapeDesc)
{
    // get shape local pose
    if (!shapeDesc->masterDesc)
    {
        getCollisionShapeLocalTransfrom(xfCache, shapeDesc->sourceGprim, bodyDesc ? bodyDesc->usdPrim : stage->GetPseudoRoot(),
            shapeDesc->localPos, shapeDesc->localRot, shapeDesc->localScale);
    }

    if (bodyDesc)
    {
        shapeDesc->rigidBody = bodyDesc->usdPrim.GetPrimPath();
    }
}


} // namespace schema
} // namespace physics
} // namespace omni

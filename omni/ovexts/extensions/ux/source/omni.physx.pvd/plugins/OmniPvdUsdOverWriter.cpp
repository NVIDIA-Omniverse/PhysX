// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdUsdOverWriter.h"
#include <algorithm>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/dictionary.h>

void processTranslationOver(
    PXR_NS::UsdGeomXformCache &xformCache,
    PXR_NS::UsdPrim* ancestorPrim,
    PXR_NS::UsdPrim* overPrim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    static PXR_NS::TfToken const &attribToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::Type::TypeTranslate);
    PXR_NS::UsdAttribute usdAttrib = overPrim->GetAttribute(attribToken);
    PXR_NS::UsdGeomXformable *xformable = (PXR_NS::UsdGeomXformable*)overPrim;
    if (!usdAttrib) {
        PXR_NS::UsdGeomXformOp translation = xformable->AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
    }
    float *pos = (float*)attrib->mData;
    usdAttrib.Set(PXR_NS::GfVec3f(pos[0], pos[1], pos[2]), (double)attrib->mTimeStamp);
}

void processRotationOver(
    PXR_NS::UsdGeomXformCache &xformCache,
    PXR_NS::UsdPrim* ancestorPrim,
    PXR_NS::UsdPrim* overPrim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    static PXR_NS::TfToken const &attribToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::Type::TypeOrient);
    PXR_NS::UsdAttribute usdAttrib = overPrim->GetAttribute(attribToken);
    PXR_NS::UsdGeomXformable *xformable = (PXR_NS::UsdGeomXformable*)overPrim;
    if (!usdAttrib) {
        PXR_NS::UsdGeomXformOp translation = xformable->AddOrientOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
    }
    float *quat = (float*)attrib->mData;
    usdAttrib.Set(PXR_NS::GfQuatf(quat[3], quat[0], quat[1], quat[2]), (double)attrib->mTimeStamp);
}

void processScaleOver(
    PXR_NS::UsdGeomXformCache &xformCache,
    PXR_NS::UsdPrim* ancestorPrim,
    PXR_NS::UsdPrim* overPrim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    static PXR_NS::TfToken const &attribToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::Type::TypeScale);
    PXR_NS::UsdAttribute usdAttrib = overPrim->GetAttribute(attribToken);
    PXR_NS::UsdGeomXformable *xformable = (PXR_NS::UsdGeomXformable*)overPrim;
    if (!usdAttrib) {
        PXR_NS::UsdGeomXformOp scaleOp = xformable->AddScaleOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
    }
    float *scale = (float*)attrib->mData;
    usdAttrib.Set(PXR_NS::GfVec3f(scale[0], scale[1], scale[2]), (double)attrib->mTimeStamp);
}

PXR_NS::UsdPrim getTformAncestor(
    PXR_NS::UsdPrim& prim
)
{
    PXR_NS::UsdPrim primAncestor = prim.GetParent();
    while (primAncestor && (!primAncestor.IsA<PXR_NS::UsdGeomXformable>()))
    {
        primAncestor = primAncestor.GetParent();
    }
    return primAncestor;
}
/*
void createPrimPassOverMirroredInOVD(
    PXR_NS::UsdStageRefPtr usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    std::unordered_map<std::string, PXR_NS::TfToken*> &tokenMap,
    int isUSDA
)
{
    int32_t pxActorNameAttribIndex = -1;
    int32_t pxActorNameClassIndex = -1;
    std::list<OmniPvdObject*>::iterator it;
    for (it = objectCreations.begin(); it != objectCreations.end(); it++)
    {
        OmniPvdObject* omniPvdObject = *it;
        OmniPvdClass *omniPvdClass = omniPvdObject->mOmniPvdClass;
        if (omniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor)
        {
            char* primPath = (char*)getAttribData(pxActorNameAttribIndex, pxActorNameClassIndex, "name", omniPvdObject);
            if (primPath)
            {
                PXR_NS::SdfPath primPathSdf = PXR_NS::SdfPath(primPath);
                PXR_NS::UsdPrim simPrim = usdStage->GetPrimAtPath(primPathSdf);
                if (simPrim)
                {
                    PXR_NS::UsdPrim overPrim = usdStage->OverridePrim(primPathSdf);
                }
            }
        }
    }
}

void createPrimPassOverHasPhysXAPIsOrIsJoint(
    PXR_NS::UsdStageRefPtr usdStage
)
{
    // Make sure we create an overriding Prim for each Prim that has a PhysX or UsdPhysics API
    const PXR_NS::UsdPrimRange range = usdStage->Traverse(PXR_NS::UsdTraverseInstanceProxies());
    for (PXR_NS::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const PXR_NS::UsdPrim& prim = *iter;
        if (!prim)
            continue;        
        const PXR_NS::SdfPath& primPath = prim.GetPath();
        if (hasPhysicsAPIs((PXR_NS::UsdPrim&)prim) || prim.IsA<PXR_NS::UsdPhysicsJoint>())
        {
            PXR_NS::UsdPrim overPrim = usdStage->OverridePrim(primPath);
        }
    }
}
*/

static bool isPhysicsSchema(const std::string& schemaStr)
{
    std::string lower = schemaStr;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return lower.find("physx") != std::string::npos || lower.find("physics") != std::string::npos;
}

bool hasPhysicsAPIs(const PXR_NS::UsdPrim& prim)
{
    const PXR_NS::TfTokenVector apiSchemas = prim.GetAppliedSchemas();
    for (const PXR_NS::TfToken& schema : apiSchemas)
    {
        if (isPhysicsSchema(schema.GetString()))
        {
            return true;
        }
    }
    return false;
}

void removePhysicsAPIs(PXR_NS::UsdPrim& prim)
{
    if (prim.IsInstanceProxy()) return;
    const PXR_NS::TfTokenVector apiSchemas = prim.GetAppliedSchemas();
    for (const PXR_NS::TfToken& schema : apiSchemas)
    {
        const std::string schemaStr = schema.GetString();
        if (isPhysicsSchema(schemaStr))
        {
            // Multi-apply schemas use "SchemaName:instanceName" format
            size_t colonPos = schemaStr.find(':');
            if (colonPos != std::string::npos)
            {
                PXR_NS::TfToken schemaFamily(schemaStr.substr(0, colonPos));
                PXR_NS::TfToken instanceName(schemaStr.substr(colonPos + 1));
                prim.RemoveAPI(schemaFamily, instanceName);
            }
            else
            {
                prim.RemoveAPI(schema);
            }
        }
    }
}

bool clearPhysicsAPIsAndDisableJoints(PXR_NS::UsdStageRefPtr stage)
{
    PXR_NS::TfToken jointEnabledToken = PXR_NS::TfToken("physics:jointEnabled");
    const PXR_NS::UsdPrimRange range = stage->Traverse(PXR_NS::UsdTraverseInstanceProxies());
    for (PXR_NS::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const PXR_NS::UsdPrim& prim = *iter;
        if (!prim)
            continue;
        removePhysicsAPIs((PXR_NS::UsdPrim&)prim);
        if (prim.IsA<PXR_NS::UsdPhysicsJoint>())
        {
            prim.SetActive(false);
        }
    }
    return true;
}

void createAttribPassOver(
    PXR_NS::UsdStageRefPtr* usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    int isUSDA
)
{
    int32_t pxActorNameAttribIndex = -1;
    int32_t pxActorNameClassIndex = -1;

    int32_t pxActorGlobalPoseAttribIndex = -1;
    int32_t pxActorGlobalPoseClassIndex = -1;
   
    PXR_NS::UsdGeomXformCache xformCache;
    static PXR_NS::TfToken const tformToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::Type::TypeTransform);    
    
    std::list<OmniPvdObject*>::iterator it;
    for (it = objectCreations.begin(); it != objectCreations.end(); it++)
    {
        OmniPvdObject* omniPvdObject = *it;
        OmniPvdClass *omniPvdClass = omniPvdObject->mOmniPvdClass;
        if (omniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor)
        {
            char* primPath = (char*)getAttribData(pxActorNameAttribIndex, pxActorNameClassIndex, "name", omniPvdObject);
            if (primPath)
            {
                PXR_NS::SdfPath primPathSdf = PXR_NS::SdfPath(primPath);
                PXR_NS::UsdPrim simPrim = (*usdStage)->GetPrimAtPath(primPathSdf);
                if (simPrim)
                {
                    PXR_NS::UsdPrim overPrim = (*usdStage)->OverridePrim(primPathSdf);
                    if (overPrim)
                    {
                        ////////////////////////////////////////////////////////////////////////////////
                        // Extract the local Tform of the primitive by finding the first ancestor that
                        // is a UsdGeomXformable, take the ancestors tform and calculate the local tform as
                        //   prim.TformLocal = ancestorPrim.Tform(Inv) * prim.TformGlobal
                        ////////////////////////////////////////////////////////////////////////////////

                        OmniPvdAttributeInstList* globalPoses = getAttribList(pxActorGlobalPoseAttribIndex, pxActorGlobalPoseClassIndex, "globalPose", omniPvdObject);

                        if (globalPoses)
                        {
                            PXR_NS::UsdGeomXformable xformableSimPrim = PXR_NS::UsdGeomXformable(simPrim);
                            if (xformableSimPrim)
                            {
                                
                                OmniPvdAttributeSample *omniPvdGlobalPoseAttrib = (OmniPvdAttributeSample*)globalPoses->mFirst;

                                ////////////////////////////////////////////////////////////////////////////////
                                // Get the precisions of the transforms already on the simulated prim
                                ////////////////////////////////////////////////////////////////////////////////
                                PXR_NS::UsdGeomXformOp::Precision tformPrecision = PXR_NS::UsdGeomXformOp::Precision::PrecisionDouble;

                                bool hasScaleOp = false;
                                PXR_NS::UsdGeomXformOp scaleOp;

                                bool hasTformOp = false;
                                PXR_NS::UsdGeomXformOp tformOp;

                                
                                bool resetsXformStack;
                                std::vector<PXR_NS::UsdGeomXformOp> orderedOps = xformableSimPrim.GetOrderedXformOps(&resetsXformStack);
                                
                                // Iterate through existing ops and keep only scaling ones
                                for (const auto& op : orderedOps) {
                                    if (op.GetOpType() == PXR_NS::UsdGeomXformOp::TypeScale)
                                    {
                                        scaleOp = op;
                                        hasScaleOp = true;
                                    }
                                    else if (op.GetOpType() == PXR_NS::UsdGeomXformOp::TypeTransform)
                                    {
                                        tformOp = op;
                                        hasTformOp = true;                                    
                                    }
                                }
                                // Set the new xform op order with only scaling and tform
                                PXR_NS::UsdGeomXformable xformableOverPrim = (PXR_NS::UsdGeomXformable) overPrim;
                                if (!hasTformOp)
                                {
                                    tformOp = xformableOverPrim.AddTransformOp(tformPrecision);
                                }

                                std::vector<PXR_NS::UsdGeomXformOp> newOps;
                                if (hasScaleOp)
                                {
                                    newOps = std::vector<PXR_NS::UsdGeomXformOp>{tformOp, scaleOp};
                                }
                                else
                                {
                                    newOps = std::vector<PXR_NS::UsdGeomXformOp>{tformOp};
                                }
                                                            
                                xformableOverPrim.SetXformOpOrder(newOps);
                                ////////////////////////////////////////////////////////////////////////////////
                                // Extract ancestorPrim.Tform(Inv)
                                ////////////////////////////////////////////////////////////////////////////////
                                PXR_NS::UsdPrim ancestorWithTform = getTformAncestor(simPrim);
                                bool hasAncestor = ancestorWithTform ? true : false;
                                PXR_NS::GfMatrix4d parentTformInv;
                                if (hasAncestor)
                                {
                                    parentTformInv = xformCache.GetLocalToWorldTransform(ancestorWithTform).GetInverse();
                                }
                                else
                                {
                                    parentTformInv = PXR_NS::GfMatrix4d(1.0);
                                }

                                while (omniPvdGlobalPoseAttrib)
                                {
                                    const double attribTimeStamp = (double)omniPvdGlobalPoseAttrib->mTimeStamp;

                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Extract prim.TformGlobal from the OmniPvd DOM state mData field.
                                    // The data is stored as a quaternion and translation vector, as 7 floats
                                    // The quaternion is stored as x, y, z, w order. So we need to convert this to
                                    // the Pixar standard w, x, y, z order.
                                    ////////////////////////////////////////////////////////////////////////////////
                                    const float* quatGlobalDat = (float*)omniPvdGlobalPoseAttrib->mData;
                                    const float* transGlobalDat = &((float*)omniPvdGlobalPoseAttrib->mData)[4];

                                    const PXR_NS::GfMatrix4d rotGlobalMat = PXR_NS::GfMatrix4d(1.0).SetRotate(PXR_NS::GfRotation(PXR_NS::GfQuatd(quatGlobalDat[3], PXR_NS::GfVec3d(quatGlobalDat[0], quatGlobalDat[1], quatGlobalDat[2]))));
                                    const PXR_NS::GfMatrix4d transGlobalMat = PXR_NS::GfMatrix4d(1.0).SetTranslate(PXR_NS::GfVec3d(transGlobalDat[0], transGlobalDat[1], transGlobalDat[2]));

                                    const PXR_NS::GfMatrix4d primGlobalTform = rotGlobalMat * transGlobalMat;

                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Calculate : prim.TformLocal = ancestorPrim.Tform(Inv) * prim.TformGlobal
                                    // Note - Pixar does right to left order.
                                    ////////////////////////////////////////////////////////////////////////////////                                
                                    const PXR_NS::GfMatrix4d primLocaTform = primGlobalTform * parentTformInv;

                                    PXR_NS::UsdAttribute tformAttrib = overPrim.GetAttribute(tformToken);

                                    // The time stamp is in time codes and is in fact doubled, so we need to divide by 2
                                    // to get the correct time stamp. The exception is for timestamps that are 1.0, which
                                    // are the initial values and should not be divided by 2. Time stamps start at 1.0
                                    // and are incremented by 1.0 each frame. Each odd value is a pre-simulation frame and
                                    // each even value is a post-simulation frame. This collapses the time stamps to a single
                                    // frame per time code except for the initial frame.
                                    double timeStamp = 0.0;
                                    if (attribTimeStamp > 1.0)
                                    {
                                        timeStamp = std::ceil(attribTimeStamp / 2.0);
                                    }
                                    tformAttrib.Set(primLocaTform, timeStamp);

                                    omniPvdGlobalPoseAttrib = (OmniPvdAttributeSample*)omniPvdGlobalPoseAttrib->mNextAttribute;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void createParticleAttribPassOver(
    PXR_NS::UsdStageRefPtr* usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    int isUSDA
)
{
    int32_t particleBufferNameAttribIndex = -1;
    int32_t particleBufferNameClassIndex = -1;

    int32_t particleBufferPosAttribIndex = -1;
    int32_t particleBufferPosClassIndex = -1;

    PXR_NS::UsdGeomXformCache xformCache;

    std::list<OmniPvdObject*>::iterator it;
    for (it = objectCreations.begin(); it != objectCreations.end(); it++)
    {
        OmniPvdObject* omniPvdObject = *it;
        OmniPvdClass *omniPvdClass = omniPvdObject->mOmniPvdClass;

        // Check if this is a PxParticleBuffer or derived class
        if (omniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxParticleBuffer)
        {
            // Get the Points prim path from name attribute
            char* primPath = (char*)getAttribData(particleBufferNameAttribIndex, particleBufferNameClassIndex, "name", omniPvdObject);
            if (primPath)
            {
                PXR_NS::SdfPath primPathSdf = PXR_NS::SdfPath(primPath);
                PXR_NS::UsdPrim simPrim = (*usdStage)->GetPrimAtPath(primPathSdf);
                if (simPrim)
                {
                    PXR_NS::UsdPrim overPrim = (*usdStage)->OverridePrim(primPathSdf);
                    if (overPrim)
                    {
                        // Get positionInvMasses samples
                        OmniPvdAttributeInstList* positions = getAttribList(particleBufferPosAttribIndex, particleBufferPosClassIndex, "positionInvMasses", omniPvdObject);

                        if (positions)
                        {
                            // Get transform for world-to-local conversion
                            PXR_NS::GfMatrix4d localToWorld = xformCache.GetLocalToWorldTransform(simPrim);
                            PXR_NS::GfMatrix4d worldToLocal = localToWorld.GetInverse();

                            // Get points attribute - works for both UsdGeomPoints and UsdGeomPointInstancer
                            // Check PointInstancer first since it derives from PointBased
                            PXR_NS::UsdGeomPointInstancer pointInstancer(simPrim);
                            PXR_NS::UsdGeomPointBased pointBased(simPrim);
                            PXR_NS::UsdAttribute pointsAttr;
                            if (pointInstancer)
                            {
                                pointsAttr = pointInstancer.CreatePositionsAttr();
                            }
                            else if (pointBased)
                            {
                                pointsAttr = pointBased.CreatePointsAttr();
                            }

                            if (pointsAttr)
                            {
                                // Process each frame's positions
                                OmniPvdAttributeSample* posAttrib = (OmniPvdAttributeSample*)positions->mFirst;
                                while (posAttrib)
                                {
                                    // positionInvMasses is array of floats: (x, y, z, invMass) per particle
                                    float* posData = (float*)posAttrib->mData;
                                    uint32_t numFloats = posAttrib->mDataLen / sizeof(float);
                                    uint32_t numParticles = numFloats / 4;

                                    PXR_NS::VtArray<PXR_NS::GfVec3f> localPoints(numParticles);
                                    for (uint32_t i = 0; i < numParticles; i++)
                                    {
                                        PXR_NS::GfVec3d worldPos(posData[i*4], posData[i*4+1], posData[i*4+2]);
                                        PXR_NS::GfVec3d localPos = worldToLocal.Transform(worldPos);
                                        localPoints[i] = PXR_NS::GfVec3f(localPos);
                                    }

                                    // Calculate timecode (same logic as rigid bodies)
                                    double timeStamp = 0.0;
                                    double attribTimeStamp = (double)posAttrib->mTimeStamp;
                                    if (attribTimeStamp > 1.0)
                                    {
                                        timeStamp = std::ceil(attribTimeStamp / 2.0);
                                    }

                                    pointsAttr.Set(localPoints, timeStamp);

                                    posAttrib = (OmniPvdAttributeSample*)posAttrib->mNextAttribute;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

// Navigate from a deformable actor to its first collision mesh OVD object.
// Hierarchy: DeformableActor -> Shape -> Geometry (mReferenceObject -> ref -> mReferenceObject = shared mesh)
static OmniPvdObject* findDeformableMeshObject(OmniPvdObject* deformableActor)
{
    // Navigate: actor -> first shape child -> first geom child -> ref -> shared mesh
    OmniPvdObject* shapeChild = deformableActor->mFirstChild;
    while (shapeChild)
    {
        OmniPvdObject* geomChild = shapeChild->mFirstChild;
        while (geomChild)
        {
            // The geometry has mReferenceObject pointing to its ref child (e.g., tetmesh_ref)
            // The ref child's mReferenceObject points to the actual shared mesh
            if (geomChild->mReferenceObject && geomChild->mReferenceObject->mReferenceObject)
            {
                return geomChild->mReferenceObject->mReferenceObject;
            }
            geomChild = geomChild->mNextSibling;
        }
        shapeChild = shapeChild->mNextSibling;
    }
    return nullptr;
}

void createDeformableAttribPassOver(
    PXR_NS::UsdStageRefPtr* usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    int isUSDA
)
{
    int32_t deformableNameAttribIndex = -1;
    int32_t deformableNameClassIndex = -1;

    PXR_NS::UsdGeomXformCache xformCache;

    std::list<OmniPvdObject*>::iterator it;
    for (it = objectCreations.begin(); it != objectCreations.end(); it++)
    {
        OmniPvdObject* omniPvdObject = *it;
        OmniPvdClass *omniPvdClass = omniPvdObject->mOmniPvdClass;

        // Check if this is a deformable volume or surface
        if (omniPvdClass->mPhysXBaseProcessingClassId != OmniPvdPhysXClassEnum::ePxDeformableVolume &&
            omniPvdClass->mPhysXBaseProcessingClassId != OmniPvdPhysXClassEnum::ePxDeformableSurface)
        {
            continue;
        }

        // Get the prim path from the name attribute
        char* primPath = (char*)getAttribData(deformableNameAttribIndex, deformableNameClassIndex, "name", omniPvdObject);
        if (!primPath)
        {
            continue;
        }

        PXR_NS::SdfPath primPathSdf = PXR_NS::SdfPath(primPath);
        PXR_NS::UsdPrim simPrim = (*usdStage)->GetPrimAtPath(primPathSdf);
        if (!simPrim)
        {
            continue;
        }

        PXR_NS::UsdPrim overPrim = (*usdStage)->OverridePrim(primPathSdf);
        if (!overPrim)
            continue;

        PXR_NS::UsdGeomMesh meshPrim(overPrim);
        if (!meshPrim)
        {
            // Deformable actor names reference the root Xform;
            // navigate to the first renderable mesh child (e.g., skin mesh)
            for (auto child : simPrim.GetChildren())
            {
                PXR_NS::UsdGeomMesh childMesh(child);
                if (childMesh)
                {
                    PXR_NS::UsdGeomImageable imageable(child);
                    if (imageable)
                    {
                        PXR_NS::TfToken purpose;
                        imageable.GetPurposeAttr().Get(&purpose);
                        if (purpose == PXR_NS::UsdGeomTokens->guide)
                            continue;
                    }
                    simPrim = child;
                    overPrim = (*usdStage)->OverridePrim(child.GetPath());
                    meshPrim = PXR_NS::UsdGeomMesh(overPrim);
                    break;
                }
            }
            if (!meshPrim)
                continue;
        }

        // Get transform for world-to-local conversion
        PXR_NS::GfMatrix4d localToWorld = xformCache.GetLocalToWorldTransform(simPrim);
        PXR_NS::GfMatrix4d worldToLocal = localToWorld.GetInverse();

        // Find position data: navigate from the deformable actor to its mesh object
        // via the shape hierarchy (actor -> shape -> geometry -> ref -> shared mesh)
        OmniPvdObject* meshObject = findDeformableMeshObject(omniPvdObject);

        // Determine position attribute name: try mesh object first, then fall back to actor
        const char* posAttrName = "positions";
        OmniPvdObject* posSource = meshObject;

        if (!posSource)
        {
            // Fallback for old-style streams where positions are on the actor
            posSource = omniPvdObject;
            posAttrName = (omniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxDeformableVolume)
                ? "collisionMeshPositions" : "positions";
        }

        // Get triangle indices for topology (static, set once)
        // For the over layer, only use triangle topology since the target is a renderable skin mesh.
        // Prefer tris from mesh object, fall back to actor-level attributes for old-style streams.
        // Skip tet topology (4 indices per element) — let the original mesh topology stand.
        int32_t triAttribIndex = -1;
        int32_t triClassIndex = -1;
        OmniPvdAttributeInstList* triList = nullptr;

        if (meshObject)
        {
            triList = getAttribList(triAttribIndex, triClassIndex, "tris", meshObject);
        }
        if (!triList)
        {
            // Fallback for old-style streams
            const char* triAttrName = (omniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxDeformableVolume)
                ? "collisionMeshTriangleIndices" : "triangleIndices";
            triList = getAttribList(triAttribIndex, triClassIndex, triAttrName, omniPvdObject);
        }

        if (triList)
        {
            OmniPvdAttributeSample* triSample = (OmniPvdAttributeSample*)triList->mFirst;
            if (triSample && triSample->mDataLen > 0)
            {
                int* triData = (int*)triSample->mData;
                int nbrTris = triSample->mDataLen / (sizeof(int) * 3);
                if (nbrTris > 0)
                {
                    PXR_NS::VtArray<int> vertexCounts(nbrTris, 3);
                    PXR_NS::VtArray<int> vertexIndices;
                    vertexIndices.assign(triData, triData + nbrTris * 3);
                    meshPrim.CreateFaceVertexCountsAttr().Set(vertexCounts);
                    meshPrim.CreateFaceVertexIndicesAttr().Set(vertexIndices);
                }
            }
        }
        meshPrim.CreateSubdivisionSchemeAttr().Set(PXR_NS::TfToken("none"));
        meshPrim.CreateDoubleSidedAttr().Set(true);

        // Get position samples (animated per-frame)
        int32_t posAttribIndex = -1;
        int32_t posClassIndex = -1;
        OmniPvdAttributeInstList* positions = getAttribList(posAttribIndex, posClassIndex, posAttrName, posSource);
        if (!positions)
            continue;

        PXR_NS::UsdAttribute pointsAttr = meshPrim.CreatePointsAttr();

        OmniPvdAttributeSample* posAttrib = (OmniPvdAttributeSample*)positions->mFirst;
        while (posAttrib)
        {
            // Positions are PxVec4 array: x,y,z,invMass per vertex
            float* posData = (float*)posAttrib->mData;
            uint32_t numFloats = posAttrib->mDataLen / sizeof(float);
            uint32_t numVerts = numFloats / 4;

            PXR_NS::VtArray<PXR_NS::GfVec3f> localPoints(numVerts);
            for (uint32_t i = 0; i < numVerts; i++)
            {
                PXR_NS::GfVec3d worldPos(posData[i*4], posData[i*4+1], posData[i*4+2]);
                PXR_NS::GfVec3d localPos = worldToLocal.Transform(worldPos);
                localPoints[i] = PXR_NS::GfVec3f(localPos);
            }

            // Calculate timecode (same logic as rigid bodies and particles)
            double timeStamp = 0.0;
            double attribTimeStamp = (double)posAttrib->mTimeStamp;
            if (attribTimeStamp > 1.0)
            {
                timeStamp = std::ceil(attribTimeStamp / 2.0);
            }

            pointsAttr.Set(localPoints, timeStamp);

            posAttrib = (OmniPvdAttributeSample*)posAttrib->mNextAttribute;
        }
    }
}

bool getTimePerFrame(OmniPvdDOMState &domState, float& timePerFrame)
{
    OmniPvdObject* sceneRoot = domState.mSceneRoot;
    if (!sceneRoot)
    {
        return false;
    }
    OmniPvdObject* scene = sceneRoot->mLastChild;
    while (scene && scene->mOmniPvdClass->mClassName != "PxScene")
    {
        scene = scene->mPrevSibling;
    }
    if (!scene)
    {
        return false;
    }
    while (scene)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Must be set to < 0 otherwise the geAttribData function assumes you know the
        // index of the attribute in the class definition vector.
        ////////////////////////////////////////////////////////////////////////////////
        int32_t attribIndex = -1;
        int32_t classIndex = -1;
        OmniPvdAttributeInstList* timePerFrameList = getAttribList(attribIndex, classIndex, "elapsedTime", scene);
        if (!timePerFrameList)
        {
            return false;
        }
        ////////////////////////////////////////////////////////////////////////////////
        // To be sure go through all attribute instances of elapsedTime the scene
        ////////////////////////////////////////////////////////////////////////////////
        OmniPvdAttributeSample* attrib = (OmniPvdAttributeSample*)timePerFrameList->mFirst;
        while (attrib) {
            const float timePerFrameTmp = *(float*)attrib->mData;
            if (timePerFrameTmp > 0.0f)
            {
                timePerFrame = timePerFrameTmp;
                return true;
            }
            attrib = (OmniPvdAttributeSample*)(attrib->mNextAttribute);
        }
        scene = scene->mNextSibling;
    }
    return false;
}

bool verifyOverLayerForDomState(
    OmniPvdDOMState &domState,
    PXR_NS::UsdStageRefPtr stage,
    PXR_NS::SdfLayerHandle overLayer
)
{
   static PXR_NS::TfToken const tformToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::Type::TypeTransform);

    // Verify that the objects found in the domState objectCreations list are present in the over layer
    int32_t pxActorNameAttribIndex = -1;
    int32_t pxActorNameClassIndex = -1;

    int32_t pxActorGlobalPoseAttribIndex = -1;
    int32_t pxActorGlobalPoseClassIndex = -1;

    PXR_NS::UsdGeomXformCache xformCacheStart(0.0);
    PXR_NS::UsdGeomXformCache xformCacheStop(100000.0); // Large enough to not be a problem

    std::list<OmniPvdObject*>::iterator it;
    for (it = domState.mObjectCreations.begin(); it != domState.mObjectCreations.end(); it++)
    {
        OmniPvdObject* omniPvdObject = *it;
        OmniPvdClass *omniPvdClass = omniPvdObject->mOmniPvdClass;
        if (omniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor)
        {
            char* primPathString = (char*)getAttribData(pxActorNameAttribIndex, pxActorNameClassIndex, "name", omniPvdObject);
            if (primPathString)
            {
                PXR_NS::SdfPath primPathSdf = PXR_NS::SdfPath(primPathString);
                PXR_NS::UsdPrim simPrim = stage->GetPrimAtPath(primPathSdf);
                if (simPrim)
                {
                    PXR_NS::SdfPrimSpecHandle primSpec = overLayer->GetPrimAtPath(primPathSdf);
                    if (!primSpec)
                    {
                        CARB_LOG_ERROR("Prim does not have an override: %s\n", primPathSdf.GetAsString().c_str());
                        return false;
                    }
                    if (primSpec->GetSpecifier() != PXR_NS::SdfSpecifierOver)
                    {
                        CARB_LOG_ERROR("Prim is not an override: %s\n", primPathSdf.GetAsString().c_str());
                        return false;
                    }
                    // Test if the transform is the same as the OVD transform for the start and stop timecodes
                    OmniPvdAttributeInstList* globalPoses = getAttribList(pxActorGlobalPoseAttribIndex, pxActorGlobalPoseClassIndex, "globalPose", omniPvdObject);
                    if (globalPoses)
                    {
                        OmniPvdAttributeSample *firstPoseAttrib = (OmniPvdAttributeSample*)globalPoses->mFirst;
                        OmniPvdAttributeSample *lastPoseAttrib = (OmniPvdAttributeSample*)globalPoses->mLast;
                        if (firstPoseAttrib && lastPoseAttrib)
                        {
                            const float* startTranslation = &((float*)firstPoseAttrib->mData)[4];
                            const float* stopTranslation = &((float*)lastPoseAttrib->mData)[4];
                            
                            PXR_NS::GfMatrix4d startTform = xformCacheStart.GetLocalToWorldTransform(simPrim);
                            PXR_NS::GfVec3d translationStart = startTform.ExtractTranslation();
                            
                            PXR_NS::GfMatrix4d stopTform = xformCacheStop.GetLocalToWorldTransform(simPrim);
                            PXR_NS::GfVec3d translationStop = stopTform.ExtractTranslation();

                            // Compare the start translation values, with a tolerance of 0.001
                            if (fabs(translationStart[0] - startTranslation[0]) > 0.001 ||
                                fabs(translationStart[1] - startTranslation[1]) > 0.001 ||
                                fabs(translationStart[2] - startTranslation[2]) > 0.001)
                            {
                                CARB_LOG_ERROR("Translation is not the same as the OVD transform for the start timecode: %s\n", primPathSdf.GetAsString().c_str());
                                return false;
                            }                            

                            // Compare the stop translation values, with a tolerance of 0.001
                            if (fabs(translationStop[0] - stopTranslation[0]) > 0.001 ||
                                fabs(translationStop[1] - stopTranslation[1]) > 0.001 ||
                                fabs(translationStop[2] - stopTranslation[2]) > 0.001)
                            {   
                                CARB_LOG_ERROR("Translation is not the same as the OVD transform for the stop timecode: %s\n", primPathSdf.GetAsString().c_str());
                                return false;   
                            }

                        }                 
                    }
                }
            }
        }
    }

    // Verify that there are no PhysX or UsdPhysics APIs on any of the prims in the stage
    const PXR_NS::UsdPrimRange range = stage->Traverse(PXR_NS::UsdTraverseInstanceProxies());
    for (PXR_NS::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const PXR_NS::UsdPrim& prim = *iter;
        if (!prim)
            continue;        
        const PXR_NS::SdfPath& primPath = prim.GetPath();
        if (hasPhysicsAPIs((PXR_NS::UsdPrim&)prim))
        {
            CARB_LOG_ERROR("Prim still has physics schemas: %s\n", primPath.GetAsString().c_str());
            return false;
        }
    }
    return true;
}

void writeUSDFileOver(
    OmniPvdDOMState &domState
)
{
    PXR_NS::UsdStageRefPtr stage = omni::usd::UsdContext::getContext()->getStage();

    // Check if the current edit target is writable
    PXR_NS::UsdEditTarget editTarget = stage->GetEditTarget();
    PXR_NS::SdfLayerHandle editLayer = editTarget.GetLayer();
    if (!editLayer || !editLayer->PermissionToEdit())
    {
        CARB_LOG_ERROR("[Bake] Current edit target is not writable. Please create or select an edit layer before baking.");
        return;
    }

    //createPrimPassOverMirroredInOVD(stage, domState.mObjectCreations, 1);
    //createPrimPassOverHasPhysXAPIsOrIsJoint(stage);
    {
        PXR_NS::SdfChangeBlock block;
        createAttribPassOver(&stage, domState.mObjectCreations, 1);
    }
    {
        PXR_NS::SdfChangeBlock block;
        createParticleAttribPassOver(&stage, domState.mObjectCreations, 1);
    }
    {
        PXR_NS::SdfChangeBlock block;
        createDeformableAttribPassOver(&stage, domState.mObjectCreations, 1);
    }
    {
        PXR_NS::SdfChangeBlock block;
        clearPhysicsAPIsAndDisableJoints(stage);
    }
    // Set time codes - prefer root layer, fall back to session layer
    // Note: USD only allows stage metadata on root or session layer, not sublayers
    PXR_NS::SdfLayerHandle rootLayer = stage->GetRootLayer();
    if (rootLayer && !rootLayer->IsAnonymous() && rootLayer->PermissionToEdit())
    {
        PXR_NS::UsdEditContext editCtx(stage, rootLayer);
        stage->SetStartTimeCode((double)domState.mMinFrame);
        stage->SetEndTimeCode((double)domState.mMaxFrame);
    }
    else
    {
        // Fall back to session layer for time codes
        PXR_NS::SdfLayerHandle sessionLayer = stage->GetSessionLayer();
        if (sessionLayer && sessionLayer->PermissionToEdit())
        {
            PXR_NS::UsdEditContext editCtx(stage, sessionLayer);
            stage->SetStartTimeCode((double)domState.mMinFrame);
            stage->SetEndTimeCode((double)domState.mMaxFrame);
        }
        else
        {
            CARB_LOG_WARN("[Bake] Neither root nor session layer is writable, skipping time code setup");
        }
    }
}

bool writeUSDFileOverWithLayerCreation(
    OmniPvdDOMState &domState,
    const std::string& inputStageFileAbsolutePath,
    const std::string& outputDir,
    const std::string& outputStageFile,
    float startTime,
    float stopTime,
    bool newLayersAreASCII,
    bool verifyOverLayer
)
{
    ////////////////////////////////////////////////////////////////////////////////
    // The structure of the New Stage is as follows:
    //  New Stage
    //    metadata from Incoming Stage
    //    layers
    //      over layer with the new over Prims - layer_over.usdc
    //      root layer prims from Incoming Stage as a new sublayer - layer_root.usdc
    //      layers in the Incoming Stage - whatever was in the incoming stage
    ////////////////////////////////////////////////////////////////////////////////

    // Get just the base filename without any directory path
    std::string outputStagFilename = PXR_NS::TfGetBaseName(outputStageFile);
    
    // Normalize the output directory path
    std::string normalizedOutputDir = PXR_NS::TfGetPathName(outputDir);

    // Remove any existing extension and append .usda
    size_t dotPos = outputStagFilename.find_last_of('.');
    if (dotPos != std::string::npos) {
        outputStagFilename = outputStagFilename.substr(0, dotPos);
    }
    outputStagFilename += ".usda";

    std::string outputLayerFileTypeStr;
    if (newLayersAreASCII)
    {
        outputLayerFileTypeStr = ".usda";
    }
    else
    {     
       outputLayerFileTypeStr = ".usdc";
    }

    // Load the input USD Stage
    PXR_NS::UsdStageRefPtr inputStage = PXR_NS::UsdStage::Open(inputStageFileAbsolutePath);

    // Create the new stage and set up its layers : outputStage
    std::string outputStagePath = normalizedOutputDir + outputStagFilename;
    PXR_NS::UsdStageRefPtr outputStage = PXR_NS::UsdStage::CreateNew(outputStagePath);

    PXR_NS::SdfLayerRefPtr outputStageRootLayer = outputStage->GetRootLayer();

    // Create the output over layer : outputOverLayer
    std::string outputOverLayerName = "layer_over" + outputLayerFileTypeStr;
    std::string outputOverLayerPath = normalizedOutputDir + outputOverLayerName;
    PXR_NS::SdfLayerRefPtr outputOverLayer = PXR_NS::SdfLayer::CreateNew(outputOverLayerPath);

            
    // Create the output root layer : outputRootLayer
    std::string outputRootLayerName = "layer_root" + outputLayerFileTypeStr;
    std::string outputRootLayerPath = normalizedOutputDir + outputRootLayerName;
    PXR_NS::SdfLayerRefPtr outputRootLayer = PXR_NS::SdfLayer::CreateNew(outputRootLayerPath);
    
    // Create the new layer stack with
    //   overLayer
    //   rootLayer
    //   followed by previous sublayers of the input stage

    std::vector<std::string> newSubLayers;
    newSubLayers.push_back(outputOverLayerName);
    newSubLayers.push_back(outputRootLayerName);

    // Get the sublayer paths from the input stage : subLayers
    PXR_NS::SdfLayerHandle inputRootLayer = inputStage->GetRootLayer();
    std::vector<std::string> subLayers = inputRootLayer->GetSubLayerPaths();

    // Keep sublayer paths exactly as they are from the input stage
    for (const std::string& subLayerPath : subLayers)
    {
        newSubLayers.push_back(subLayerPath);
    }    
    outputStage->GetRootLayer()->SetSubLayerPaths(newSubLayers);

    // Copy the meta data from the inputStage into the root layer of the outputStage
    std::vector<PXR_NS::TfToken> fields = inputRootLayer->ListFields(PXR_NS::SdfPath::AbsoluteRootPath());
    for (const PXR_NS::TfToken& field : fields)
    {
        // Skip fields that require special handling
        if (field == PXR_NS::SdfFieldKeys->SubLayers || field == PXR_NS::SdfFieldKeys->Owner)
        {
            continue;
        }
        PXR_NS::VtValue value;
        if (inputRootLayer->HasField(PXR_NS::SdfPath::AbsoluteRootPath(), field))
        {
            std::string fieldStr = field.GetText();
            if ((fieldStr != "subLayerOffsets") && (fieldStr != "primChildren"))
            {
                value = inputRootLayer->GetField(PXR_NS::SdfPath::AbsoluteRootPath(), field);
                outputStageRootLayer->SetField(PXR_NS::SdfPath::AbsoluteRootPath(), field, value);
            }
        }
    }

    // Copy the root layer prims from the inputStage root layer to the outputStage's separate root layer
    for (const auto& primSpec : inputRootLayer->GetRootPrims()) {
        PXR_NS::SdfPath primPath = primSpec->GetPath();        
        // Verify prim exists in root layer (not just session/payloads)
        if (inputStage->GetPrimAtPath(primPath).HasAuthoredReferences() ||
            inputStage->GetPrimAtPath(primPath).HasPayload()) {
            continue;
        }
        PXR_NS::SdfCopySpec(inputRootLayer, primPath, outputRootLayer, primPath);
    }

    // Now set up the over layer of the outpuStage
    {
        PXR_NS::UsdEditContext editCtx(outputStage, outputOverLayer);
        {
            PXR_NS::SdfChangeBlock block;
            createAttribPassOver(&outputStage, domState.mObjectCreations, 1);
        }
        {
            PXR_NS::SdfChangeBlock block;
            createParticleAttribPassOver(&outputStage, domState.mObjectCreations, 1);
        }
        {
            PXR_NS::SdfChangeBlock block;
            createDeformableAttribPassOver(&outputStage, domState.mObjectCreations, 1);
        }
        {
            PXR_NS::SdfChangeBlock block;
            clearPhysicsAPIsAndDisableJoints(outputStage);
        }
    }

    {
        PXR_NS::UsdEditContext editCtx(outputStage, outputStage->GetRootLayer());
        // Set the time range based on the provided parameters
        
        float timePerFrame = 0.0f;
        double timeCodePerSecond = 1.0;
        if (getTimePerFrame(domState, timePerFrame))
        {
            timeCodePerSecond = 1.0/(double)timePerFrame;
        }
        else
        {
            CARB_LOG_WARN("Failed to get time codes per second, setting to 1.0f");
        }
        outputStage->SetTimeCodesPerSecond(ceil(timeCodePerSecond));

        double maxTimeCode = ceil((((double)domState.mMaxFrame)-1.0)/2.0);
        double maxTime = maxTimeCode / timeCodePerSecond;

        if (startTime < 0.0f)
        {
            startTime = 0.0f;
        }
        if (startTime > maxTime)
        {
            startTime = 0.0f;
        }
        if (stopTime <= startTime)
        {
            stopTime = maxTime;
        }
        outputStage->SetStartTimeCode(ceil( (double)startTime * timeCodePerSecond ));
        outputStage->SetEndTimeCode(ceil( (double)stopTime * timeCodePerSecond ));
    }

    outputStage->SetInterpolationType(PXR_NS::UsdInterpolationType::UsdInterpolationTypeHeld);

    // Save the stage and all its layers to disk
    outputStage->Save();

    if (verifyOverLayer)
    {
        if (!verifyOverLayerForDomState(domState, outputStage, outputOverLayer))
        {
            CARB_LOG_ERROR("Failed to verify the over layer");
            return false;
        }
    }
    return true;
}

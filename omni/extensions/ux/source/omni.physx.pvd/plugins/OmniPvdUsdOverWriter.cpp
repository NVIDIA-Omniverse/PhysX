// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdUsdOverWriter.h"
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/dictionary.h>

void processTranslationOver(
    pxr::UsdGeomXformCache &xformCache,
    pxr::UsdPrim* ancestorPrim,
    pxr::UsdPrim* overPrim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    static pxr::TfToken const &attribToken = pxr::UsdGeomXformOp::GetOpName(pxr::UsdGeomXformOp::Type::TypeTranslate);
    pxr::UsdAttribute usdAttrib = overPrim->GetAttribute(attribToken);
    pxr::UsdGeomXformable *xformable = (pxr::UsdGeomXformable*)overPrim;
    if (!usdAttrib) {
        pxr::UsdGeomXformOp translation = xformable->AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat);
    }
    float *pos = (float*)attrib->mData;
    usdAttrib.Set(pxr::GfVec3f(pos[0], pos[1], pos[2]), (double)attrib->mTimeStamp);
}

void processRotationOver(
    pxr::UsdGeomXformCache &xformCache,
    pxr::UsdPrim* ancestorPrim,
    pxr::UsdPrim* overPrim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    static pxr::TfToken const &attribToken = pxr::UsdGeomXformOp::GetOpName(pxr::UsdGeomXformOp::Type::TypeOrient);
    pxr::UsdAttribute usdAttrib = overPrim->GetAttribute(attribToken);
    pxr::UsdGeomXformable *xformable = (pxr::UsdGeomXformable*)overPrim;
    if (!usdAttrib) {
        pxr::UsdGeomXformOp translation = xformable->AddOrientOp(pxr::UsdGeomXformOp::PrecisionFloat);
    }
    float *quat = (float*)attrib->mData;
    usdAttrib.Set(pxr::GfQuatf(quat[3], quat[0], quat[1], quat[2]), (double)attrib->mTimeStamp);
}

void processScaleOver(
    pxr::UsdGeomXformCache &xformCache,
    pxr::UsdPrim* ancestorPrim,
    pxr::UsdPrim* overPrim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    static pxr::TfToken const &attribToken = pxr::UsdGeomXformOp::GetOpName(pxr::UsdGeomXformOp::Type::TypeScale);
    pxr::UsdAttribute usdAttrib = overPrim->GetAttribute(attribToken);
    pxr::UsdGeomXformable *xformable = (pxr::UsdGeomXformable*)overPrim;
    if (!usdAttrib) {
        pxr::UsdGeomXformOp scaleOp = xformable->AddScaleOp(pxr::UsdGeomXformOp::PrecisionFloat);
    }
    float *scale = (float*)attrib->mData;
    usdAttrib.Set(pxr::GfVec3f(scale[0], scale[1], scale[2]), (double)attrib->mTimeStamp);
}

pxr::UsdPrim getTformAncestor(
    pxr::UsdPrim& prim
)
{
    pxr::UsdPrim primAncestor = prim.GetParent();
    while (primAncestor && (!primAncestor.IsA<pxr::UsdGeomXformable>()))
    {
        primAncestor = primAncestor.GetParent();
    }
    return primAncestor;
}
/*
void createPrimPassOverMirroredInOVD(
    pxr::UsdStageRefPtr usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    std::unordered_map<std::string, pxr::TfToken*> &tokenMap,
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
                pxr::SdfPath primPathSdf = pxr::SdfPath(primPath);
                pxr::UsdPrim simPrim = usdStage->GetPrimAtPath(primPathSdf);
                if (simPrim)
                {
                    pxr::UsdPrim overPrim = usdStage->OverridePrim(primPathSdf);
                }
            }
        }
    }
}

void createPrimPassOverHasPhysXAPIsOrIsJoint(
    pxr::UsdStageRefPtr usdStage
)
{
    // Make sure we create an overriding Prim for each Prim that has a PhysX or UsdPhysics API
    const pxr::UsdPrimRange range = usdStage->Traverse(pxr::UsdTraverseInstanceProxies());
    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;        
        const pxr::SdfPath& primPath = prim.GetPath();
        if (hasPhysicsAPIs((pxr::UsdPrim&)prim) || prim.IsA<pxr::UsdPhysicsJoint>())
        {
            pxr::UsdPrim overPrim = usdStage->OverridePrim(primPath);
        }
    }
}
*/

bool hasPhysicsAPIs(const pxr::UsdPrim& prim)
{
    const pxr::TfTokenVector apiSchemas = prim.GetAppliedSchemas();
    //printf("Removing schemas from: %s\n", prim.GetPath().GetAsString().c_str());
    for (const pxr::TfToken& schema : apiSchemas) 
    {
        const std::string schemaStr = schema.GetString();
        if (schemaStr.find("PhysX") != std::string::npos || schemaStr.find("Physics") != std::string::npos) 
        {
            return true;
        }
    }
    return false;;
}

void removePhysicsAPIs(pxr::UsdPrim& prim)
{
    if (prim.IsInstanceProxy()) return;
    const pxr::TfTokenVector apiSchemas = prim.GetAppliedSchemas();
    for (const pxr::TfToken& schema : apiSchemas) 
    {
        const std::string schemaStr = schema.GetString();
        if (schemaStr.find("PhysX") != std::string::npos || schemaStr.find("Physics") != std::string::npos) 
        {
            if (prim.HasAPI(schema))
            {
                prim.RemoveAPI(schema);
            }
        }
    }
}

bool clearPhysicsAPIsAndDisableJoints(pxr::UsdStageRefPtr stage)
{
    pxr::TfToken jointEnabledToken = pxr::TfToken("physics:jointEnabled");
    const pxr::UsdPrimRange range = stage->Traverse(pxr::UsdTraverseInstanceProxies());
    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;
        removePhysicsAPIs((pxr::UsdPrim&)prim);
        if (prim.IsA<pxr::UsdPhysicsJoint>())
        {
            //prim.CreateAttribute(jointEnabledToken, pxr::SdfValueTypeNames->Bool).Set(false);
            prim.SetActive(false);
        }
    }
    return true;
}

void createAttribPassOver(
    pxr::UsdStageRefPtr* usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    std::unordered_map<std::string, pxr::TfToken*> &tokenMap,
    int isUSDA
)
{
    int32_t pxActorNameAttribIndex = -1;
    int32_t pxActorNameClassIndex = -1;

    int32_t pxActorGlobalPoseAttribIndex = -1;
    int32_t pxActorGlobalPoseClassIndex = -1;
   
    pxr::UsdGeomXformCache xformCache;
    static pxr::TfToken const tformToken = pxr::UsdGeomXformOp::GetOpName(pxr::UsdGeomXformOp::Type::TypeTransform);    
    
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
                pxr::SdfPath primPathSdf = pxr::SdfPath(primPath);
                pxr::UsdPrim simPrim = (*usdStage)->GetPrimAtPath(primPathSdf);
                if (simPrim)
                {
                    pxr::UsdPrim overPrim = (*usdStage)->OverridePrim(primPathSdf);
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
                            pxr::UsdGeomXformable xformableSimPrim = pxr::UsdGeomXformable(simPrim);
                            if (xformableSimPrim)
                            {
                                
                                OmniPvdAttributeSample *omniPvdGlobalPoseAttrib = (OmniPvdAttributeSample*)globalPoses->mFirst;

                                ////////////////////////////////////////////////////////////////////////////////
                                // Get the precisions of the transforms already on the simulated prim
                                ////////////////////////////////////////////////////////////////////////////////
                                pxr::UsdGeomXformOp::Precision tformPrecision = pxr::UsdGeomXformOp::Precision::PrecisionDouble;

                                bool hasScaleOp = false;
                                pxr::UsdGeomXformOp scaleOp;

                                bool hasTformOp = false;
                                pxr::UsdGeomXformOp tformOp;

                                
                                bool resetsXformStack;
                                std::vector<pxr::UsdGeomXformOp> orderedOps = xformableSimPrim.GetOrderedXformOps(&resetsXformStack);
                                
                                // Iterate through existing ops and keep only scaling ones
                                for (const auto& op : orderedOps) {
                                    if (op.GetOpType() == pxr::UsdGeomXformOp::TypeScale)
                                    {
                                        scaleOp = op;
                                        hasScaleOp = true;
                                    }
                                    else if (op.GetOpType() == pxr::UsdGeomXformOp::TypeTransform)
                                    {
                                        tformOp = op;
                                        hasTformOp = true;                                    
                                    }
                                }
                                // Set the new xform op order with only scaling and tform
                                pxr::UsdGeomXformable xformableOverPrim = (pxr::UsdGeomXformable) overPrim;
                                if (!hasTformOp)
                                {
                                    tformOp = xformableOverPrim.AddTransformOp(tformPrecision);
                                }

                                std::vector<pxr::UsdGeomXformOp> newOps;
                                if (hasScaleOp)
                                {
                                    newOps = std::vector<pxr::UsdGeomXformOp>{tformOp, scaleOp};
                                }
                                else
                                {
                                    newOps = std::vector<pxr::UsdGeomXformOp>{tformOp};
                                }
                                                            
                                xformableOverPrim.SetXformOpOrder(newOps);
                                ////////////////////////////////////////////////////////////////////////////////
                                // Extract ancestorPrim.Tform(Inv)
                                ////////////////////////////////////////////////////////////////////////////////
                                pxr::UsdPrim ancestorWithTform = getTformAncestor(simPrim);
                                bool hasAncestor = ancestorWithTform ? true : false;
                                pxr::GfMatrix4d parentTformInv;
                                if (hasAncestor)
                                {
                                    parentTformInv = xformCache.GetLocalToWorldTransform(ancestorWithTform).GetInverse();
                                }
                                else
                                {
                                    parentTformInv = pxr::GfMatrix4d(1.0);
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

                                    const pxr::GfMatrix4d rotGlobalMat = pxr::GfMatrix4d(1.0).SetRotate(pxr::GfRotation(pxr::GfQuatd(quatGlobalDat[3], pxr::GfVec3d(quatGlobalDat[0], quatGlobalDat[1], quatGlobalDat[2]))));
                                    const pxr::GfMatrix4d transGlobalMat = pxr::GfMatrix4d(1.0).SetTranslate(pxr::GfVec3d(transGlobalDat[0], transGlobalDat[1], transGlobalDat[2]));

                                    const pxr::GfMatrix4d primGlobalTform = rotGlobalMat * transGlobalMat;

                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Calculate : prim.TformLocal = ancestorPrim.Tform(Inv) * prim.TformGlobal
                                    // Note - Pixar does right to left order.
                                    ////////////////////////////////////////////////////////////////////////////////                                
                                    const pxr::GfMatrix4d primLocaTform = primGlobalTform * parentTformInv;

                                    pxr::UsdAttribute tformAttrib = overPrim.GetAttribute(tformToken);

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

bool getTimePerFrame(OmniPvdDOMState &domState, float& timePerFrame)
{
    OmniPvdObject* sceneRoot = domState.mSceneLayerRoot;
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
    pxr::UsdStageRefPtr stage,
    pxr::SdfLayerHandle overLayer
)
{
   static pxr::TfToken const tformToken = pxr::UsdGeomXformOp::GetOpName(pxr::UsdGeomXformOp::Type::TypeTransform);

    // Verify that the objects found in the domState objectCreations list are present in the over layer
    int32_t pxActorNameAttribIndex = -1;
    int32_t pxActorNameClassIndex = -1;

    int32_t pxActorGlobalPoseAttribIndex = -1;
    int32_t pxActorGlobalPoseClassIndex = -1;

    pxr::UsdGeomXformCache xformCacheStart(0.0);
    pxr::UsdGeomXformCache xformCacheStop(100000.0); // Large enough to not be a problem

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
                pxr::SdfPath primPathSdf = pxr::SdfPath(primPathString);
                pxr::UsdPrim simPrim = stage->GetPrimAtPath(primPathSdf);
                if (simPrim)
                {
                    pxr::SdfPrimSpecHandle primSpec = overLayer->GetPrimAtPath(primPathSdf);
                    if (!primSpec)
                    {
                        CARB_LOG_ERROR("Prim does not have an override: %s\n", primPathSdf.GetAsString().c_str());
                        return false;
                    }
                    if (primSpec->GetSpecifier() != pxr::SdfSpecifierOver)
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
                            
                            pxr::GfMatrix4d startTform = xformCacheStart.GetLocalToWorldTransform(simPrim);
                            pxr::GfVec3d translationStart = startTform.ExtractTranslation();
                            
                            pxr::GfMatrix4d stopTform = xformCacheStop.GetLocalToWorldTransform(simPrim);
                            pxr::GfVec3d translationStop = stopTform.ExtractTranslation();

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
    const pxr::UsdPrimRange range = stage->Traverse(pxr::UsdTraverseInstanceProxies());
    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;        
        const pxr::SdfPath& primPath = prim.GetPath();
        if (hasPhysicsAPIs((pxr::UsdPrim&)prim))
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
    pxr::UsdStageRefPtr stage = omni::usd::UsdContext::getContext()->getStage();

    //createPrimPassOverMirroredInOVD(stage, domState.mObjectCreations, domState.mTokenMap, 1);
    //createPrimPassOverHasPhysXAPIsOrIsJoint(stage);
    {
        pxr::SdfChangeBlock block;
        createAttribPassOver(&stage, domState.mObjectCreations, domState.mTokenMap, 1);
    }
    {
        pxr::SdfChangeBlock block;
        clearPhysicsAPIsAndDisableJoints(stage);
    }
    {
        PXR_NS::UsdEditContext editCtx(stage, stage->GetRootLayer());
        stage->SetStartTimeCode((double)domState.mMinFrame);
        stage->SetEndTimeCode((double)domState.mMaxFrame);
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
    std::string outputStagFilename = pxr::TfGetBaseName(outputStageFile);
    
    // Normalize the output directory path
    std::string normalizedOutputDir = pxr::TfGetPathName(outputDir);

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
    pxr::UsdStageRefPtr inputStage = pxr::UsdStage::Open(inputStageFileAbsolutePath);

    // Create the new stage and set up its layers : outputStage
    std::string outputStagePath = normalizedOutputDir + outputStagFilename;
    pxr::UsdStageRefPtr outputStage = pxr::UsdStage::CreateNew(outputStagePath);

    pxr::SdfLayerRefPtr outputStageRootLayer = outputStage->GetRootLayer();

    // Create the output over layer : outputOverLayer
    std::string outputOverLayerName = "layer_over" + outputLayerFileTypeStr;
    std::string outputOverLayerPath = normalizedOutputDir + outputOverLayerName;
    pxr::SdfLayerRefPtr outputOverLayer = pxr::SdfLayer::CreateNew(outputOverLayerPath);

            
    // Create the output root layer : outputRootLayer
    std::string outputRootLayerName = "layer_root" + outputLayerFileTypeStr;
    std::string outputRootLayerPath = normalizedOutputDir + outputRootLayerName;
    pxr::SdfLayerRefPtr outputRootLayer = pxr::SdfLayer::CreateNew(outputRootLayerPath);
    
    // Create the new layer stack with
    //   overLayer
    //   rootLayer
    //   followed by previous sublayers of the input stage

    std::vector<std::string> newSubLayers;
    newSubLayers.push_back(outputOverLayerName);
    newSubLayers.push_back(outputRootLayerName);

    // Get the sublayer paths from the input stage : subLayers
    pxr::SdfLayerHandle inputRootLayer = inputStage->GetRootLayer();
    std::vector<std::string> subLayers = inputRootLayer->GetSubLayerPaths();

    for (const std::string& subLayerPath : subLayers)
    {
        pxr::SdfLayerRefPtr subLayer = pxr::SdfLayer::Find(subLayerPath);
        if (subLayer)
        {
            newSubLayers.push_back(subLayerPath);
        }
    }    
    outputStage->GetRootLayer()->SetSubLayerPaths(newSubLayers);

    // Copy the meta data from the inputStage into the root layer of the outputStage
    std::vector<pxr::TfToken> fields = inputRootLayer->ListFields(pxr::SdfPath::AbsoluteRootPath());
    for (const pxr::TfToken& field : fields)
    {
        // Skip fields that require special handling
        if (field == pxr::SdfFieldKeys->SubLayers || field == pxr::SdfFieldKeys->Owner)
        {
            continue;
        }
        pxr::VtValue value;
        if (inputRootLayer->HasField(pxr::SdfPath::AbsoluteRootPath(), field))
        {
            std::string fieldStr = field.GetText();
            if ((fieldStr != "subLayerOffsets") && (fieldStr != "primChildren"))
            {
                value = inputRootLayer->GetField(pxr::SdfPath::AbsoluteRootPath(), field);
                outputStageRootLayer->SetField(pxr::SdfPath::AbsoluteRootPath(), field, value);
            }
        }
    }

    // Copy the root layer prims from the inputStage root layer to the outputStage's separate root layer
    for (const auto& primSpec : inputRootLayer->GetRootPrims()) {
        pxr::SdfPath primPath = primSpec->GetPath();        
        // Verify prim exists in root layer (not just session/payloads)
        if (inputStage->GetPrimAtPath(primPath).HasAuthoredReferences() ||
            inputStage->GetPrimAtPath(primPath).HasPayload()) {
            continue;
        }
        pxr::SdfCopySpec(inputRootLayer, primPath, outputRootLayer, primPath);
    }

    // Now set up the over layer of the outpuStage
    {
        pxr::UsdEditContext editCtx(outputStage, outputOverLayer);
        {
            pxr::SdfChangeBlock block;
            createAttribPassOver(&outputStage, domState.mObjectCreations, domState.mTokenMap, 1);
        }
        {
            pxr::SdfChangeBlock block;
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

    outputStage->SetInterpolationType(pxr::UsdInterpolationType::UsdInterpolationTypeHeld);

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

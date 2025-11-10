// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "ParticlesVisualizationManager.h"
#include <omni/kit/EditorUsd.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <private/omni/physx/IPhysxVisualizationPrivate.h>
#include <omni/renderer/IDebugDraw.h>

#include <usdrt/scenegraph/usd/usd/stage.h>


using namespace omni::physx::ui;
using namespace pxr;
using namespace omni::physx::usdparser;
using namespace omni::renderer;

extern UsdStageRefPtr gStage;
extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;

static omni::renderer::IDebugDraw* gDebugDraw;
static omni::physx::IPhysxVisualizationPrivate* gPhysXVisualizationPrivate = nullptr;

static const TfToken transformMatrixAttrName{ "xformOp:transform" };
static const TfToken scaleAttrName{ "xformOp:scale" };
static const TfToken physxParticleClothInputCrcToken{ "physxParticle:clothDataInputCrc" };

static const GfVec3f kColorFluidParticle(0.026f, 0.083f, 0.13f); // lighter blue
static const GfVec3f kColorFluidMesh(0.02f, 0.034f, 0.1f); // darker blue
static const GfVec3f kColorSolidParticle(0.13f, 0.026f, 0.026f); // brown
static const GfVec3f kColorClothParticle(0.061f, 0.13f, 0.026f); // lighter green
static const GfVec3f kColorClothMesh(0.02f, 0.1f, 0.067f); // darker green
static const GfVec3f kColorDiffuse(1.0f, 1.0f, 1.0f); // white

uint32_t convertColor(uint32_t inColor);

namespace
{
    SdfPath appendPath(SdfPath basePath, const std::string& name)
    {
        return basePath.IsEmpty() ? SdfPath() : basePath.AppendElementString(name);
    }

    SdfPath getParticleSystemSurfaceProxyPath(SdfPath particleSystemProxyPath)
    {
        return appendPath(particleSystemProxyPath, std::string("surface"));
    }

    SdfPath getParticleSystemDiffuseProxyPath(SdfPath particleSystemProxyPath)
    {
        return appendPath(particleSystemProxyPath, std::string("diffuse"));
    }

    SdfPath getParticleObjectParticleProxyPath(SdfPath particleObjectProxyPath)
    {
        return appendPath(particleObjectProxyPath, std::string("particle"));
    }

    SdfPath getParticleObjectParticleProtoProxyPath(SdfPath particleObjectParticleProxyPath)
    {
        return appendPath(particleObjectParticleProxyPath, std::string("prototype"));
    }

    SdfPath getParticleClothMeshProxyPath(SdfPath particleClothProxyPath)
    {
        return appendPath(particleClothProxyPath, std::string("mesh"));
    }

    SdfPath getParticleSystemSurfaceActualPath(SdfPath actualParticleSystemPath)
    {
        return appendPath(actualParticleSystemPath, std::string("Isosurface"));
    }

    SdfPath getParticleSystemDiffuseActualPath(SdfPath actualParticleSystemPath)
    {
        return appendPath(actualParticleSystemPath, std::string("DiffuseParticles"));
    }

    SdfPath parseParticleSystemIsosurface(SdfPath actualParticleSystemPath)
    {
        SdfPath actualSurfacePath;
        PhysxSchemaPhysxParticleIsosurfaceAPI particleIsosurface(gStage->GetPrimAtPath(actualParticleSystemPath));
        if (particleIsosurface)
        {
            bool enabled;
            particleIsosurface.GetIsosurfaceEnabledAttr().Get(&enabled);
            if (enabled)
            {
                actualSurfacePath = getParticleSystemSurfaceActualPath(actualParticleSystemPath);
            }
        }
        return actualSurfacePath;
    }

    bool parseParticleSystemHasSmoothing(SdfPath actualParticleSystemPath)
    {
        bool enabled = false;
        PhysxSchemaPhysxParticleSmoothingAPI particleSmoothing(gStage->GetPrimAtPath(actualParticleSystemPath));
        if (particleSmoothing)
        {
            particleSmoothing.GetParticleSmoothingEnabledAttr().Get(&enabled);
        }
        return enabled;
    }

    bool parseParticleSystemHasAnisotropy(SdfPath actualParticleSystemPath)
    {
        bool enabled = false;
        PhysxSchemaPhysxParticleAnisotropyAPI particleAnisotropy(gStage->GetPrimAtPath(actualParticleSystemPath));
        if (particleAnisotropy)
        {
            particleAnisotropy.GetParticleAnisotropyEnabledAttr().Get(&enabled);
        }
        return enabled;
    }

    SdfPath parseParticleObjectParticleSystem(SdfPath actualParticleObjectPath)
    {
        SdfPath actualParticleSystemPath;
        UsdPrim particlePrim = gStage->GetPrimAtPath(actualParticleObjectPath);
        if (particlePrim && particlePrim.HasAPI<PhysxSchemaPhysxParticleAPI>())
        {
            SdfPathVector targets;
            PhysxSchemaPhysxParticleAPI(particlePrim).GetParticleSystemRel().GetTargets(&targets);
            if (!targets.empty() && !targets[0].IsEmpty())
            {
                PhysxSchemaPhysxParticleSystem ps = PhysxSchemaPhysxParticleSystem(gStage->GetPrimAtPath(targets[0]));
                if (ps)
                {
                    actualParticleSystemPath = targets[0];
                }
            }
        }
        return actualParticleSystemPath;
    }

    bool parseParticleSetIsFluid(SdfPath actualParticleSetPath)
    {
        bool fluid = false;
        PhysxSchemaPhysxParticleSetAPI particleSet = PhysxSchemaPhysxParticleSetAPI::Get(gStage, actualParticleSetPath);
        if (particleSet)
        {
            particleSet.GetFluidAttr().Get(&fluid);
        }
        return fluid;
    }

    bool parseParticleSetHasDiffuse(SdfPath actualParticleSetPath)
    {
        bool enabled = false;
        PhysxSchemaPhysxDiffuseParticlesAPI diffuseParticles =  PhysxSchemaPhysxDiffuseParticlesAPI::Get(gStage, actualParticleSetPath);
        if (diffuseParticles)
        {
            diffuseParticles.GetDiffuseParticlesEnabledAttr().Get(&enabled);
        }
        return enabled;
    }

    float getParticleRadius(omni::physx::usdparser::ParticleSystemDesc* particleSystemDesc, ParticleRadiusType radiusType, bool forFluid)
    {
        float sphereRadius = 0.0f;
        if (particleSystemDesc)
        {
            switch (radiusType)
            {
            case ParticleRadiusType::eContactOffset:
                sphereRadius = particleSystemDesc->contactOffset;
                break;
            case ParticleRadiusType::eParticleContactOffset:
                sphereRadius = particleSystemDesc->particleContactOffset;
                break;
            case ParticleRadiusType::eRestOffset:
                sphereRadius = particleSystemDesc->restOffset;
                break;
            case ParticleRadiusType::eParticleRestOffset:
                sphereRadius = forFluid ? particleSystemDesc->fluidRestOffset : particleSystemDesc->solidRestOffset;
                break;
            case ParticleRadiusType::eAnisotropy: // for aniso, get the particle rest offsets since those resemble "particle size" closest.
                sphereRadius = particleSystemDesc->fluidRestOffset;
                break;
            case ParticleRadiusType::eRenderGeometry: // render geometry will just hide the debug viz, so we don't really care about the radius.
            default:
                sphereRadius = particleSystemDesc->particleContactOffset;
                break;
            }
        }
        return sphereRadius;
    }

    void getActualParticleSetAttributes(SdfPath actualPath, ParticlePositionType positionType, bool hasSmoothing,
        UsdAttribute& pointsAttr, UsdAttribute& orientationsAttr, UsdAttribute& scalesAttr)
    {
        UsdPrim prim = gStage->GetPrimAtPath(actualPath);
        UsdGeomPoints points(prim);
        UsdGeomPointInstancer instancer(prim);

        // check if the simPoints attribute is set, and lookup the setting to see what position buffer to select.
        bool flipBuffers = false;
        PhysxSchemaPhysxParticleSetAPI particleSet(prim);
        if (particleSet)
        {
            bool hasSimPositions = particleSet.GetSimulationPointsAttr().HasAuthoredValue();
            bool smoothingReady = hasSimPositions && hasSmoothing;
            flipBuffers = smoothingReady && (positionType == ParticlePositionType::eSimPositions);
        }

        if (points)
        {
            pointsAttr = flipBuffers ? particleSet.GetSimulationPointsAttr() : points.GetPointsAttr();
        }
        else if (instancer)
        {
            pointsAttr = flipBuffers ? particleSet.GetSimulationPointsAttr() : instancer.GetPositionsAttr();
            orientationsAttr = instancer.GetOrientationsAttr();
            scalesAttr = instancer.GetScalesAttr();
        }
    }

    void releaseParticleSystemDesc(omni::physx::usdparser::ParticleSystemDesc* desc)
    {
        gUsdLoad->releaseDesc(desc);
    }

    omni::physx::usdparser::ParticleSystemDesc* createParticleSystemDesc(SdfPath particleSystemPath)
    {
        omni::physx::usdparser::ParticleSystemDesc* desc = nullptr;
        if (gStage && !particleSystemPath.IsEmpty())
        {
            const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(gStage).ToLongInt();
            desc = gUsdLoad->parseParticleSystem(stageId, particleSystemPath);
        }
        return desc;
    }

#define ON_ATTRIBUTE_CHANGE_WRAPPER(name)\
    void name(ProxyVisualizationClient& client, pxr::SdfPath path, pxr::TfToken attributeName)\
    {   static_cast<ParticlesVisualizationManager&>(client).name(path, attributeName);    }

    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_ParticleSystemParams)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_SurfaceMesh)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_DiffusePoints)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_ParticleObjectParams)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_ParticleClothCooking)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_ParticleObjectPoints)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_ParticleObjectMesh)
}

ParticlesVisualizationManager::ParticlesVisualizationManager(
    ProxyVisualizationManager& proxyVisualizationManager,
    VisualizerMode mode,
    ParticlePositionType positionType,
    ParticleRadiusType radiusType,
    bool showClothMeshLines,
    bool showParticleSetParticles,
    bool showFluidSurface,
    bool showDeformableParticles,
    bool showDeformableMesh,
    bool showDiffuseParticles)
    : mMode(mode),
      mShowParticleSetParticles(showParticleSetParticles),
      mShowFluidSurface(showFluidSurface),
      mShowDeformableParticles(showDeformableParticles),
      mShowDeformableMesh(showDeformableMesh),
      mShowDiffuseParticles(showDiffuseParticles),
      mBufferModeDirty(false),
      mRadiusType(radiusType),
      mParticlePositionType(positionType),
      mMeshLineVisualizationEnabled(showClothMeshLines),
      mProxyManager(proxyVisualizationManager)
{
    gPhysXVisualizationPrivate = carb::getCachedInterface<omni::physx::IPhysxVisualizationPrivate>();
    gDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();

    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->contactOffset, *this, &::onAttributeChange_ParticleSystemParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->restOffset, *this, &::onAttributeChange_ParticleSystemParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->solidRestOffset, *this, &::onAttributeChange_ParticleSystemParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->fluidRestOffset, *this, &::onAttributeChange_ParticleSystemParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->particleContactOffset, *this, &::onAttributeChange_ParticleSystemParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->physxParticleIsosurfaceIsosurfaceEnabled, *this, &::onAttributeChange_ParticleSystemParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->physxParticleSmoothingParticleSmoothingEnabled, *this, &::onAttributeChange_ParticleSystemParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->physxParticleAnisotropyParticleAnisotropyEnabled, *this, &::onAttributeChange_ParticleSystemParams);

    mProxyManager.registerAttribute(UsdGeomTokens.Get()->points, *this, &::onAttributeChange_SurfaceMesh);
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->faceVertexCounts, *this, &::onAttributeChange_SurfaceMesh);
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->faceVertexIndices, *this, &::onAttributeChange_SurfaceMesh);

    mProxyManager.registerAttribute(UsdGeomTokens.Get()->points, *this, &::onAttributeChange_DiffusePoints);

    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->physxParticleParticleSystem, *this, &::onAttributeChange_ParticleObjectParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->physxDiffuseParticlesDiffuseParticlesEnabled, *this, &::onAttributeChange_ParticleObjectParams);
    mProxyManager.registerAttribute(PhysxSchemaTokens.Get()->physxParticleFluid, *this, &::onAttributeChange_ParticleObjectParams);

    mProxyManager.registerAttribute(physxParticleClothInputCrcToken, *this, &::onAttributeChange_ParticleClothCooking);

    mProxyManager.registerAttribute(UsdGeomTokens.Get()->points, *this, &::onAttributeChange_ParticleObjectPoints);
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->positions, *this, &::onAttributeChange_ParticleObjectPoints);
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->scales, *this, &::onAttributeChange_ParticleObjectPoints);
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->orientations, *this, &::onAttributeChange_ParticleObjectPoints);

    mProxyManager.registerAttribute(UsdGeomTokens.Get()->faceVertexCounts, *this, &::onAttributeChange_ParticleObjectMesh);
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->faceVertexIndices, *this, &::onAttributeChange_ParticleObjectMesh);
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->widths, *this, &::onAttributeChange_ParticleObjectMesh);
}

ParticlesVisualizationManager::~ParticlesVisualizationManager()
{
    release();

    mDescCache.release();
    gPhysXVisualizationPrivate = nullptr;
    gDebugDraw = nullptr;
}

void ParticlesVisualizationManager::setMode(VisualizerMode mode)
{
    if (mMode != mode)
    {
        VisualizerMode oldMode = mMode;
        mMode = mode; // mode needs to be set before handlePrimResync is executed!
        if (oldMode == VisualizerMode::eNone)
        {
            if (gStage)
            {
                // reparse the stage if we are activating the visualizer
                mProxyManager.handlePrimResync(gStage->GetPseudoRoot().GetPath());
                mProxyManager.selectionChanged();
            }
        }
        else if (mode == VisualizerMode::eNone)
        {
            clear(true);
        }
        mBufferModeDirty = true;
    }
}

void ParticlesVisualizationManager::setParticleRadiusType(ParticleRadiusType type)
{
    mRadiusType = type;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::setParticlePositionType(ParticlePositionType type)
{
    mParticlePositionType = type;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::enableMeshLineVisualization(bool enable)
{
    mMeshLineVisualizationEnabled = enable;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::showParticleSetParticles(bool show)
{
    mShowParticleSetParticles = show;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::showFluidSurface(bool show)
{
    mShowFluidSurface = show;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::showDeformableParticles(bool show)
{ 
    mShowDeformableParticles = show;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::showDeformableMesh(bool show)
{
    mShowDeformableMesh = show;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::showDiffuseParticles(bool show)
{
    mShowDiffuseParticles = show;
    mBufferModeDirty = true;
}

void ParticlesVisualizationManager::draw()
{
    CARB_PROFILE_ZONE(0, "ParticlesVisualizationManager::draw()");

    if (!gDebugDraw || !mMeshLineVisualizationEnabled || !showDeformableMesh() || !gStage)
        return;

    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(gStage).ToLongInt();

    for (SdfPath particleClothPath : mParticleCloths)
    {
        ParticleClothProxyInfo* particleClothProxyInfo = getParticleClothProxyInfo(particleClothPath);
        if (particleClothProxyInfo && checkCompleteness(*particleClothProxyInfo))
        {
            if (!mProxyManager.isActive(particleClothPath))
                continue;

            bool invalidateCache = false;
            ParticleClothDesc* clothDesc = (usdparser::ParticleClothDesc*)mDescCache.getDesc(particleClothPath, eParticleCloth);
            if (!clothDesc)
            {
                // No Cloth descriptor in cache parse the USD to get mesh info
                clothDesc = gUsdLoad->parseParticleCloth(stageId, particleClothPath);
                // Deposit it to DescCache so we don't have to parse the USD again
                mDescCache.addDesc(particleClothPath, eParticleCloth, clothDesc);
                // Cloth descriptor has changed, invalidate the debug viz cache
                invalidateCache = true;
            }

            if (clothDesc)
            {
                UsdGeomMesh mesh = UsdGeomMesh::Get(gStage, particleClothPath);
                VtVec3fArray weldedPoints;

                if (clothDesc->isWelded)
                {
                    VtVec3fArray points;
                    mesh.GetPointsAttr().Get(&points);

                    // Get welded points through remap table
                    const std::vector<uint32_t>& verticesRemapToOrig = clothDesc->verticesRemapToOrig;
                    int numParticles = (int)verticesRemapToOrig.size();

                    weldedPoints.resize(numParticles);
                    for (int weldedIndex = 0; weldedIndex < numParticles; ++weldedIndex)
                    {
                        const uint32_t origIndex = verticesRemapToOrig[weldedIndex];
                        weldedPoints[weldedIndex] = points[origIndex];
                    }
                }
                else
                {
                    mesh.GetPointsAttr().Get(&weldedPoints);
                }

                UsdGeomXform xform = UsdGeomXform::Get(gStage, particleClothPath);
                pxr::GfMatrix4f localToWorld = pxr::GfMatrix4f(xform.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default()));

                pxr::VtArray<uint32_t> vertices(0);
                gPhysXVisualizationPrivate->getParticleClothDebugDraw(particleClothPath, clothDesc, vertices, invalidateCache);

                const size_t weldedPointsSize = weldedPoints.size();
                for (uint32_t i = 0; i < vertices.size(); i += 2)
                {
                    if (vertices[i] < weldedPointsSize && vertices[i + 1] < weldedPointsSize)
                    {
                        GfVec3f p1 = localToWorld.Transform(weldedPoints[vertices[i]]);
                        GfVec3f p2 = localToWorld.Transform(weldedPoints[vertices[i + 1]]);
                        gDebugDraw->drawLine(carb::Float3{ p1[0], p1[1], p1[2] }, convertColor(0xBB4444FF), 1.0f,
                            carb::Float3{ p2[0], p2[1], p2[2] }, convertColor(0xBB4444FF), 1.0f);
                    }
                }
            }
        }
    }
}

void ParticlesVisualizationManager::release()
{
    clear(false);
}

bool ParticlesVisualizationManager::needsStageParse(pxr::UsdStageWeakPtr stage)
{
    if (!isActive())
        return false;

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(stage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxParticleClothAPI"));
        if (!paths.empty())
            return true;
    }
    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxParticleSetAPI"));
        if (!paths.empty())
            return true;
    }
    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("PhysxParticleSystem"));
        if (!paths.empty())
            return true;
    }

    return false;
}

void ParticlesVisualizationManager::handlePrimResync(SdfPath path)
{
    if (!isActive())
        return;

    // check for updates
    if (mParticleSets.count(path) || mParticleCloths.count(path) || mBufferParticleObjectsToRemove.count(path) > 0)
    {
        mBufferParticleObjectsToRemove.erase(path);
        mBufferParticleObjectsToUpdate.insert(path);
    }
    else if (mParticleSystems.count(path) || mBufferParticleSystemsToRemove.count(path) > 0)
    {
        mBufferParticleSystemsToRemove.erase(path);
        mBufferParticleSystemsToUpdate.insert(path);
    }
    else if (gStage)
    {
        UsdPrim prim = gStage->GetPrimAtPath(path);
        if (prim.HasAPI<PhysxSchemaPhysxParticleSetAPI>() || prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
        {
            mBufferParticleObjectsToAdd.insert(path);
        }
        else if (prim.IsA<PhysxSchemaPhysxParticleSystem>())
        {
            mBufferParticleSystemsToAdd.insert(path);
        }
        else if (prim.GetParent() && prim.GetParent().IsA<PhysxSchemaPhysxParticleSystem>())
        {
            mBufferParticleSystemsToUpdate.insert(path.GetParentPath());
        }
    }
}

void ParticlesVisualizationManager::handlePrimRemove(SdfPath path)
{
    mBufferParticleObjectsToAdd.erase(path);
    mBufferParticleSystemsToAdd.erase(path);

    if (mParticleSets.count(path) || mParticleCloths.count(path))
    {
        mBufferParticleObjectsToRemove.insert(path);
        mBufferParticleObjectsToUpdate.erase(path);
    }
    else if (mParticleSystems.count(path))
    {
        mBufferParticleSystemsToRemove.insert(path);
        mBufferParticleSystemsToUpdate.erase(path);
    }
}

void ParticlesVisualizationManager::handleTransformChange(SdfPath path)
{
    if (mParticleSets.count(path) > 0 || mParticleCloths.count(path) > 0)
    {
        mBufferParticleObjectsToUpdateTransform.insert(path);
    }
}

void ParticlesVisualizationManager::handleVisibilityChange(SdfPath path)
{
    if (mParticleSets.count(path) > 0 || mParticleCloths.count(path) > 0 || mParticleSystems.count(path) > 0)
    {
        mProxyManager.bufferUpdateActive(path);
    }
}

void ParticlesVisualizationManager::onAttributeChange_ParticleSystemParams(SdfPath path, TfToken attributeName)
{
    if (mParticleSystems.count(path))
    {
        mBufferParticleSystemsToUpdate.insert(path);
    }
}

void ParticlesVisualizationManager::onAttributeChange_SurfaceMesh(SdfPath path, TfToken attributeName)
{
    if (mParticleSystems.count(path.GetParentPath()) && path.GetName() == "Isosurface")
    {
        mBufferParticleSystemsToUpdateSurfaceMesh.insert(path.GetParentPath());
    }
}

void ParticlesVisualizationManager::onAttributeChange_DiffusePoints(SdfPath path, TfToken attributeName)
{
    if (mParticleSystems.count(path.GetParentPath()) && path.GetName() == "DiffuseParticles")
    {
        mBufferParticleSystemsToUpdateDiffusePoints.insert(path.GetParentPath());
    }
}

void ParticlesVisualizationManager::onAttributeChange_ParticleObjectParams(SdfPath path, TfToken attributeName)
{
    if (mParticleSets.count(path) > 0 || mParticleCloths.count(path) > 0)
    {
        mBufferParticleObjectsToUpdate.insert(path);
    }
}

void ParticlesVisualizationManager::onAttributeChange_ParticleClothCooking(SdfPath path, TfToken attributeName)
{
    if (mParticleSets.count(path) > 0 || mParticleCloths.count(path) > 0)
    {
        // invalidate caches
        mDescCache.releasePath(path);
    }
}

void ParticlesVisualizationManager::onAttributeChange_ParticleObjectPoints(SdfPath path, TfToken attributeName)
{
    if (mParticleCloths.count(path) > 0)
    {
        mBufferParticleObjectsToUpdatePoints.insert(path);
    }
    else if (mParticleSets.count(path) > 0)
    {
        mBufferParticleObjectsToUpdatePoints.insert(path);
        ParticleSetProxyInfo* particleSetProxyInfo = getParticleSetProxyInfo(path);
        if (particleSetProxyInfo->hasDiffuse)
        {
            //trigger particle system diffuse update based on particle set points in order to catch reset
            mBufferParticleSystemsToUpdateDiffusePoints.insert(particleSetProxyInfo->actualParticleSystemPath);
        }
    }
}

void ParticlesVisualizationManager::onAttributeChange_ParticleObjectMesh(SdfPath path, TfToken attributeName)
{
    if (mParticleCloths.count(path) > 0 || mParticleSets.count(path) > 0)
    {
        mBufferParticleObjectsToUpdateMesh.insert(path);
    }
}

void ParticlesVisualizationManager::updateTracking()
{
    if (!isActive())
    {
        return;
    }

    // track new particle systems
    for (SdfPath path : mBufferParticleSystemsToAdd)
    {
        addParticleSystem(path);
    }

    // track new particle objects
    for (SdfPath path : mBufferParticleObjectsToAdd)
    {
        addParticleObject(path);
    }

    // track particle object removals
    for (SdfPath path : mBufferParticleObjectsToRemove)
    {
        removeParticleObject(path, true);
    }

    // track particle object updates, including removal of APIs
    for (SdfPath path : mBufferParticleObjectsToUpdate)
    {
        const UsdPrim prim = gStage->GetPrimAtPath(path);
        if (!prim || !prim.HasAPI<PhysxSchemaPhysxParticleAPI>())
        {
            // remove session prims if the underlying particle is not a particle prim/does not exist anymore
            removeParticleObject(path, true);
        }
        else
        {
            updateParticleObject(path);
        }
    }

    // track particle system removals
    for (SdfPath path : mBufferParticleSystemsToRemove)
    {
        removeParticleSystem(path, true);
    }

    // track particle system updates
    for (SdfPath path : mBufferParticleSystemsToUpdate)
    {
        const UsdPrim prim = gStage->GetPrimAtPath(path);
        if (!prim)
        {
            removeParticleSystem(path, true);
        }
        else
        {
            updateParticleSystem(path);
        }
    }

}

void ParticlesVisualizationManager::updateModeDirty()
{
    if (mBufferModeDirty)
    {
        for (SdfPath particleSystemPath : mParticleSystems)
        {
            mProxyManager.bufferUpdateActive(particleSystemPath);
        }

        for (SdfPath particleSetPath : mParticleSets)
        {
            mProxyManager.bufferUpdateActive(particleSetPath);
        }

        for (SdfPath particleClothPath : mParticleCloths)
        {
            mProxyManager.bufferUpdateActive(particleClothPath);
        }
    }
}

bool ParticlesVisualizationManager::checkMode(SdfPath actualPath)
{
    if (mMode == VisualizerMode::eSelected)
    {
        return mProxyManager.isSelected(actualPath);
    }

    //checkMode shouldn't be called if VisualizerMode is eNone, but handle this case for robustness
    return mMode != VisualizerMode::eNone;
}

bool ParticlesVisualizationManager::checkCompleteness(ProxyInfo& proxyInfo)
{
    if (proxyInfo.type == ProxyInfoType::eParticleSet || proxyInfo.type == ProxyInfoType::eParticleCloth)
    {
        ParticleObjectProxyInfo& particleObjectProxyInfo = static_cast<ParticleObjectProxyInfo&>(proxyInfo);
        if (mParticleSystemToParticleMap.count(particleObjectProxyInfo.actualParticleSystemPath) == 0)
        {
            return false;
        }
    }
    return true;
}

uint32_t ParticlesVisualizationManager::updateActiveProxies(ProxyInfo& proxyInfo, SdfPath actualPath)
{
    uint32_t numActiveProxies = 0;
    if (proxyInfo.type == ProxyInfoType::eParticleSet || proxyInfo.type == ProxyInfoType::eParticleCloth)
    {
        ParticleObjectProxyInfo& particleObjectProxyInfo = static_cast<ParticleObjectProxyInfo&>(proxyInfo);
        bool showParticles = (proxyInfo.type == ProxyInfoType::eParticleSet) ? showParticleSetParticles() : showDeformableParticles();
        if (showParticles)
        {
            particleObjectProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(actualPath);
            SdfPath particleProxyPath = getParticleObjectParticleProxyPath(particleObjectProxyInfo.proxyRootPath);
            UsdPrim particleProxyPrim = gStage->GetPrimAtPath(particleProxyPath);
            if (!particleProxyPrim)
            {
                particleProxyPrim = createParticleInstancerProxyPrim(particleProxyPath);

                mProxyManager.addProxyPrim(actualPath, particleProxyPath);
                mProxyManager.addProxyPrim(actualPath, getParticleObjectParticleProtoProxyPath(particleProxyPath));
            }

            mBufferParticleObjectsToUpdateTransform.insert(actualPath);
            mBufferParticleObjectsToUpdatePoints.insert(actualPath);
            mBufferParticleObjectsToUpdateMesh.insert(actualPath);
            numActiveProxies++;
        }
        else
        {
            SdfPath particleProxyPath = getParticleObjectParticleProxyPath(particleObjectProxyInfo.proxyRootPath);
            if (!particleProxyPath.IsEmpty())
            {
                SdfPath particleProtoProxyPath = getParticleObjectParticleProtoProxyPath(particleProxyPath);
                mProxyManager.removeProxyPrim(actualPath, particleProtoProxyPath);
                mProxyManager.removeProxyPrim(actualPath, particleProxyPath);
            }
        }
    }

    if (proxyInfo.type == ProxyInfoType::eParticleCloth)
    {
        ParticleClothProxyInfo& particleClothProxyInfo = static_cast<ParticleClothProxyInfo&>(proxyInfo);
        if (showDeformableMesh())
        {
            particleClothProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(actualPath);
            SdfPath meshProxyPath = getParticleClothMeshProxyPath(particleClothProxyInfo.proxyRootPath);
            UsdPrim meshProxyPrim = gStage->GetPrimAtPath(meshProxyPath);
            if (!meshProxyPrim)
            {
                meshProxyPrim = createMeshProxyPrim(meshProxyPath);
                mProxyManager.addProxyPrim(actualPath, meshProxyPath);
            }

            mBufferParticleObjectsToUpdateTransform.insert(actualPath);
            mBufferParticleObjectsToUpdatePoints.insert(actualPath);
            mBufferParticleObjectsToUpdateMesh.insert(actualPath);
            numActiveProxies++;
        }
        else
        {
            SdfPath meshProxyPath = getParticleClothMeshProxyPath(particleClothProxyInfo.proxyRootPath);
            mProxyManager.removeProxyPrim(actualPath, meshProxyPath);
        }
    }
    else if (proxyInfo.type == ProxyInfoType::eParticleSystem)
    {
        ParticleSystemProxyInfo& particleSystemProxyInfo = static_cast<ParticleSystemProxyInfo&>(proxyInfo);

        if (showFluidSurface() && !particleSystemProxyInfo.actualIsosurfacePath.IsEmpty())
        {
            particleSystemProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(actualPath);
            SdfPath meshProxyPath = getParticleSystemSurfaceProxyPath(particleSystemProxyInfo.proxyRootPath);
            UsdPrim meshProxyPrim = gStage->GetPrimAtPath(meshProxyPath);
            if (!meshProxyPrim)
            {
                meshProxyPrim = createMeshProxyPrim(meshProxyPath);
                mProxyManager.addProxyPrim(actualPath, meshProxyPath);
            }

            mBufferParticleSystemsToUpdateSurfaceMesh.insert(actualPath);
            numActiveProxies++;
        }
        else
        {
            SdfPath meshProxyPath = getParticleSystemSurfaceProxyPath(particleSystemProxyInfo.proxyRootPath);
            mProxyManager.removeProxyPrim(actualPath, meshProxyPath);
        }

        if (showDiffuseParticles())
        {
            particleSystemProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(actualPath);
            SdfPath pointsProxyPath = getParticleSystemDiffuseProxyPath(particleSystemProxyInfo.proxyRootPath);
            UsdPrim pointsProxyPrim = gStage->GetPrimAtPath(pointsProxyPath);
            if (!pointsProxyPrim)
            {
                pointsProxyPrim = createPointsProxyPrim(pointsProxyPath);
                mProxyManager.addProxyPrim(actualPath, pointsProxyPath);
            }

            mBufferParticleSystemsToUpdateDiffusePoints.insert(actualPath);
            numActiveProxies++;
        }
        else
        {
            SdfPath pointsProxyPath = getParticleSystemDiffuseProxyPath(particleSystemProxyInfo.proxyRootPath);
            mProxyManager.removeProxyPrim(actualPath, pointsProxyPath);
        }
    }

    return numActiveProxies;
}

void ParticlesVisualizationManager::updateProxyProperties(UsdGeomXformCache& xformCache)
{
    for (SdfPath actualPath : mBufferParticleObjectsToUpdateTransform)
    {
        updateParticleObjectTransform(actualPath, xformCache);
    }

    for (SdfPath actualPath : mBufferParticleObjectsToUpdatePoints)
    {
        updateParticleObjectPoints(actualPath);
    }

    for (SdfPath actualPath : mBufferParticleObjectsToUpdateMesh)
    {
        updateParticleObjectMesh(actualPath);
    }

    for (SdfPath actualPath : mBufferParticleSystemsToUpdateSurfaceMesh)
    {
        updateParticleSystemSurfaceMesh(actualPath, xformCache);
    }

    for (SdfPath actualPath : mBufferParticleSystemsToUpdateDiffusePoints)
    {
        updateParticleSystemDiffusePoints(actualPath, xformCache);
    }
}

void ParticlesVisualizationManager::updateActualPurpose(SdfPath actualPath, bool active)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(actualPath);
    if (!proxyInfo || !gStage)
    {
        return;
    }

    if (proxyInfo->type == ProxyInfoType::eParticleSet)
    {
        ParticleSetProxyInfo& particleSetProxyInfo = static_cast<ParticleSetProxyInfo&>(*proxyInfo);
        ParticleSystemProxyInfo* particleSystemProxyInfo = getParticleSystemProxyInfo(particleSetProxyInfo.actualParticleSystemPath);
        bool isFluidSetWithSurface = particleSetProxyInfo.isFluid && particleSystemProxyInfo && !particleSystemProxyInfo->actualIsosurfacePath.IsEmpty();

        UsdGeomImageable actualImg = UsdGeomImageable::Get(gStage, actualPath);
        if (actualImg)
        {
            //hide the actual particle set if proxy is shown (active) or if the actual surface rendering should surpress the particle rendering
            bool hideActual = active || isFluidSetWithSurface;
            TfToken setPurpose = hideActual ? UsdGeomTokens->proxy : UsdGeomTokens->default_;
            actualImg.GetPurposeAttr().Set(setPurpose);

            if (isFluidSetWithSurface)
            {
                UsdGeomImageable actualSurfaceImg = UsdGeomImageable::Get(gStage, particleSystemProxyInfo->actualIsosurfacePath);
                if (actualSurfaceImg)
                {
                    //hide the actual surface
                    //either if proxy surface is shown (active)
                    //or if there are any shown (active) proxy particle sets that also participate in the surface (to avoid obscuring particles)
                    bool activeSurface = showFluidSurface() && mProxyManager.isActive(particleSetProxyInfo.actualParticleSystemPath);
                    bool hideActualSurface = activeSurface || (active || hasActiveFluidParticleSet(particleSetProxyInfo.actualParticleSystemPath));
                    TfToken surfacePurpose = hideActualSurface ? UsdGeomTokens->proxy : UsdGeomTokens->default_;
                    actualSurfaceImg.GetPurposeAttr().Set(surfacePurpose);
                }
            }
        }
    }
    else if (proxyInfo->type == ProxyInfoType::eParticleCloth)
    {
        UsdGeomImageable actualImg = UsdGeomImageable::Get(gStage, actualPath);
        if (actualImg)
        {
            TfToken purpose = active ? UsdGeomTokens->proxy : UsdGeomTokens->default_;
            actualImg.GetPurposeAttr().Set(purpose);
        }
    }
    else if (proxyInfo->type == ProxyInfoType::eParticleSystem)
    {
        UsdGeomImageable actualSurfaceImg = UsdGeomImageable::Get(gStage, getParticleSystemSurfaceActualPath(actualPath));
        if (actualSurfaceImg)
        {
            //hide the actual surface
            //either if proxy surface is shown (active)
            //or if there are any shown (active) proxy particle sets that also participate in the surface (to avoid obscuring particles)
            bool activeSurface = showFluidSurface() && active;
            bool hideActualSurface = activeSurface || (hasActiveFluidParticleSet(actualPath));
            TfToken surfacePurpose = hideActualSurface ? UsdGeomTokens->proxy : UsdGeomTokens->default_;
            actualSurfaceImg.GetPurposeAttr().Set(surfacePurpose);
        }
    }
}

void ParticlesVisualizationManager::clearBuffers(void)
{
    mBufferParticleObjectsToAdd.clear();
    mBufferParticleObjectsToRemove.clear();
    mBufferParticleObjectsToUpdate.clear();
    mBufferParticleObjectsToUpdateTransform.clear();
    mBufferParticleObjectsToUpdatePoints.clear();
    mBufferParticleObjectsToUpdateMesh.clear();

    mBufferParticleSystemsToAdd.clear();
    mBufferParticleSystemsToRemove.clear();
    mBufferParticleSystemsToUpdate.clear();
    mBufferParticleSystemsToUpdateSurfaceMesh.clear();
    mBufferParticleSystemsToUpdateDiffusePoints.clear();

    mBufferModeDirty = false;
}

UsdPrim ParticlesVisualizationManager::createParticleInstancerProxyPrim(SdfPath particleInstancerProxyPath)
{
    UsdGeomPointInstancer points = UsdGeomPointInstancer::Define(gStage, particleInstancerProxyPath);
    CARB_ASSERT(points);

    // create prototype
    SdfPath prototypePath = getParticleObjectParticleProtoProxyPath(particleInstancerProxyPath);
    UsdGeomSphere sphereGeom = UsdGeomSphere::Define(gStage, prototypePath);

    UsdGeomXformOp opScale;
    opScale = sphereGeom.AddScaleOp();
    // workaround for hydra update not working if op value is set in later update.
    opScale.Set(GfVec3f(1, 1, 1)); 

    sphereGeom.CreateRadiusAttr().Set(1.0);
    VtArray<GfVec3f> colorArray(1, GfVec3f(1.0f, 1.0f, 1.0f));
    sphereGeom.CreateDisplayColorPrimvar().Set(colorArray);

    VtArray<int> indices;
    VtArray<GfVec3f> positions;
    indices.resize(0);

    points.GetPrototypesRel().AddTarget(prototypePath);

    points.CreateProtoIndicesAttr().Set(indices);
    points.CreatePositionsAttr().Set(positions);

    omni::kit::EditorUsd::setNoDelete(points.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(points.GetPrim(), true);
    omni::kit::EditorUsd::setNoSelectionOutline(points.GetPrim(), true);

    omni::kit::EditorUsd::setNoDelete(sphereGeom.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(sphereGeom.GetPrim(), true);
    omni::kit::EditorUsd::setNoSelectionOutline(sphereGeom.GetPrim(), true);
    return points.GetPrim();
}

UsdPrim ParticlesVisualizationManager::createMeshProxyPrim(SdfPath meshProxyPath)
{
    UsdGeomMesh mesh = UsdGeomMesh::Define(gStage, meshProxyPath);
    CARB_ASSERT(mesh);

    VtArray<GfVec3f> points;
    VtArray<int> faceVertexCounts;
    VtArray<int> faceVertexIndices;

    mesh.CreatePointsAttr().Set(points);
    mesh.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
    mesh.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);

    omni::kit::EditorUsd::setNoDelete(mesh.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(mesh.GetPrim(), true);
    omni::kit::EditorUsd::setNoSelectionOutline(mesh.GetPrim(), true);
    return mesh.GetPrim();
}

UsdPrim ParticlesVisualizationManager::createPointsProxyPrim(SdfPath pointsProxyPath)
{
    UsdGeomPoints geomPoints = UsdGeomPoints::Define(gStage, pointsProxyPath);
    CARB_ASSERT(geomPoints);

    VtArray<GfVec3f> points;

    geomPoints.CreatePointsAttr().Set(points);
    geomPoints.SetWidthsInterpolation(UsdGeomTokens.Get()->constant);

    omni::kit::EditorUsd::setNoDelete(geomPoints.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(geomPoints.GetPrim(), true);
    omni::kit::EditorUsd::setNoSelectionOutline(geomPoints.GetPrim(), true);
    return geomPoints.GetPrim();
}

ParticlesVisualizationManager::ParticleSystemProxyInfo* ParticlesVisualizationManager::createParticleSystemProxyInfo(SdfPath actualParticleSystemPath)
{
    ParticleSystemProxyInfo* proxyInfo = new ParticleSystemProxyInfo();
    proxyInfo->client = this;
    proxyInfo->type = ProxyInfoType::eParticleSystem;
    proxyInfo->actualIsosurfacePath = parseParticleSystemIsosurface(actualParticleSystemPath);
    proxyInfo->hasSmoothing = parseParticleSystemHasSmoothing(actualParticleSystemPath);
    proxyInfo->hasAnisotropy = parseParticleSystemHasAnisotropy(actualParticleSystemPath);
    proxyInfo->desc = createParticleSystemDesc(actualParticleSystemPath);
    return proxyInfo;
}

ParticlesVisualizationManager::ParticleSetProxyInfo* ParticlesVisualizationManager::createParticleSetProxyInfo(SdfPath actualParticleSetPath)
{
    ParticleSetProxyInfo* proxyInfo = new ParticleSetProxyInfo();
    proxyInfo->client = this;
    proxyInfo->type = ProxyInfoType::eParticleSet;
    proxyInfo->isFluid = parseParticleSetIsFluid(actualParticleSetPath);
    proxyInfo->hasDiffuse = parseParticleSetHasDiffuse(actualParticleSetPath);
    return proxyInfo;
}

ParticlesVisualizationManager::ParticleClothProxyInfo* ParticlesVisualizationManager::createParticleClothProxyInfo(SdfPath actualParticleClothPath)
{
    ParticleClothProxyInfo* proxyInfo = new ParticleClothProxyInfo();
    proxyInfo->client = this;
    proxyInfo->type = ProxyInfoType::eParticleCloth;
    return proxyInfo;
}

void ParticlesVisualizationManager::addParticleSystem(SdfPath path)
{
    if (mParticleSystems.count(path) == 0)
    {
        //update tracking
        mParticleSystems.insert(path);
        mParticleSystemToParticleMap.insert({path, SdfPathSet()});
        ParticleSystemProxyInfo* particleSystemProxyInfo = createParticleSystemProxyInfo(path);
        mProxyManager.addProxy(path, *particleSystemProxyInfo);

        //update events
        mProxyManager.bufferUpdateActive(path);
    }
}

void ParticlesVisualizationManager::removeParticleSystem(SdfPath path, bool updateSkin)
{
    //update events
    SdfPathSet* particleObjectPaths = getParticleSystemParticleObjects(path);
    if (particleObjectPaths)
    {
        for (SdfPath particleObjectPath : *particleObjectPaths)
        {
            mProxyManager.bufferUpdateActive(particleObjectPath);
        }
    }

    if (updateSkin)
    {
        updateActualPurpose(path, false);
    }

    //update tracking
    mParticleSystems.erase(path);
    mParticleSystemToParticleMap.erase(path);
    ParticleSystemProxyInfo* particleSystemProxyInfo = getParticleSystemProxyInfo(path);
    if (particleSystemProxyInfo)
    {
        releaseParticleSystemDesc(particleSystemProxyInfo->desc);
        mProxyManager.removeProxy(path, particleSystemProxyInfo);
    }
}

void ParticlesVisualizationManager::updateParticleSystem(SdfPath path)
{
    ParticleSystemProxyInfo* particleSystemProxyInfo = getParticleSystemProxyInfo(path);
    if (particleSystemProxyInfo)
    {
        particleSystemProxyInfo->actualIsosurfacePath = parseParticleSystemIsosurface(path);
        particleSystemProxyInfo->actualDiffusePath = getParticleSystemDiffuseActualPath(path);
        particleSystemProxyInfo->hasSmoothing = parseParticleSystemHasSmoothing(path);
        particleSystemProxyInfo->hasAnisotropy = parseParticleSystemHasAnisotropy(path);
        releaseParticleSystemDesc(particleSystemProxyInfo->desc);
        particleSystemProxyInfo->desc = createParticleSystemDesc(path);

        SdfPathSet* particleObjectPaths = getParticleSystemParticleObjects(path);
        if (particleObjectPaths)
        {
            for (SdfPath particleObjectPath : *particleObjectPaths)
            {
                mProxyManager.bufferUpdateActive(particleObjectPath);
            }
        }
    }
    mProxyManager.bufferUpdateActive(path);
}

void ParticlesVisualizationManager::addParticleObject(SdfPath path)
{
    ParticleObjectProxyInfo* particleObjectProxyInfo = getParticleObjectProxyInfo(path);
    if (particleObjectProxyInfo)
    {
        return;
    }

    PhysxSchemaPhysxParticleAPI particleAPI = PhysxSchemaPhysxParticleAPI::Get(gStage, path);
    if (!particleAPI)
    {
        return;
    }

    PhysxSchemaPhysxParticleSetAPI particleSet = PhysxSchemaPhysxParticleSetAPI(particleAPI.GetPrim());

    //update tracking
    if (particleSet)
    {
        mParticleSets.insert(path);
        particleObjectProxyInfo = createParticleSetProxyInfo(path);
    }
    else // particle cloth
    {
        mParticleCloths.insert(path);
        particleObjectProxyInfo =  createParticleClothProxyInfo(path);
    }
    mProxyManager.addProxy(path, *particleObjectProxyInfo);

    SdfPath particleSystemPath = parseParticleObjectParticleSystem(path);
    particleObjectProxyInfo->actualParticleSystemPath = particleSystemPath;
    SdfPathSet* particleObjectPaths = getParticleSystemParticleObjects(particleSystemPath);
    if (particleObjectPaths)
    {
        particleObjectPaths->insert(path);
    }

    //update events
    mProxyManager.bufferUpdateActive(path);
    mProxyManager.bufferUpdateActive(particleSystemPath);
}

void ParticlesVisualizationManager::removeParticleObject(SdfPath path, bool updateSkin)
{
    ParticleObjectProxyInfo* particleObjectProxyInfo = getParticleObjectProxyInfo(path);
    if (!particleObjectProxyInfo)
    {
        return;
    }

    //update events
    mProxyManager.bufferUpdateActive(particleObjectProxyInfo->actualParticleSystemPath);

    if (updateSkin && gStage)
    {
        updateActualPurpose(path, false);
    }

    //update tracking
    SdfPathSet* particleObjectPaths = getParticleSystemParticleObjects(particleObjectProxyInfo->actualParticleSystemPath);
    if (particleObjectPaths)
    {
        particleObjectPaths->erase(path);
    }

    if (particleObjectProxyInfo->type == ProxyInfoType::eParticleSet)
    {
        mParticleSets.erase(path);
    }
    else // particleObjectProxyInfo->type == ProxyInfoType::eParticleCloth
    {
        mParticleCloths.erase(path);
    }

    mProxyManager.removeProxy(path, particleObjectProxyInfo);
}

void ParticlesVisualizationManager::updateParticleObject(SdfPath path)
{
    ParticleObjectProxyInfo* particleObjectProxyInfo = getParticleObjectProxyInfo(path);
    if (!particleObjectProxyInfo)
    {
        return;
    }

    //first update owner if necessary
    {
        SdfPath particleSystemPath = parseParticleObjectParticleSystem(path);
        if (particleSystemPath != particleObjectProxyInfo->actualParticleSystemPath)
        {
            mProxyManager.bufferUpdateActive(particleSystemPath);
            mProxyManager.bufferUpdateActive(particleObjectProxyInfo->actualParticleSystemPath);

            SdfPathSet* particleObjectPathsOrig = getParticleSystemParticleObjects(particleObjectProxyInfo->actualParticleSystemPath);
            if (particleObjectPathsOrig)
            {
                for (SdfPath particleObjectPathOrig : *particleObjectPathsOrig)
                {
                    mProxyManager.bufferUpdateActive(particleObjectPathOrig);
                }
                particleObjectPathsOrig->erase(path);
            }

            particleObjectProxyInfo->actualParticleSystemPath = particleSystemPath;

            SdfPathSet* particleObjectPathsNew = getParticleSystemParticleObjects(particleSystemPath);
            if (particleObjectPathsNew)
            {
                for (SdfPath particleObjectPathNew : *particleObjectPathsNew)
                {
                    mProxyManager.bufferUpdateActive(particleObjectPathNew);
                }
                particleObjectPathsNew->insert(path);
            }
        }
    }

    // update other properties
    if (particleObjectProxyInfo->type == ProxyInfoType::eParticleSet)
    {
        ParticleSetProxyInfo& particleSetProxyInfo = static_cast<ParticleSetProxyInfo&>(*particleObjectProxyInfo);
        particleSetProxyInfo.isFluid = parseParticleSetIsFluid(path);

        bool hasDiffuse = parseParticleSetHasDiffuse(path);
        if (hasDiffuse != particleSetProxyInfo.hasDiffuse)
        {
            mBufferParticleSystemsToUpdateDiffusePoints.insert(particleSetProxyInfo.actualParticleSystemPath);
            particleSetProxyInfo.hasDiffuse = hasDiffuse;
        }
    }
    else if (particleObjectProxyInfo->type == ProxyInfoType::eParticleCloth)
    {
        // could be a tranform related attribute removal
        mBufferParticleObjectsToUpdateTransform.insert(path);
    }
    mProxyManager.bufferUpdateActive(path);
}

void ParticlesVisualizationManager::updateParticleObjectTransform(SdfPath actualPath, pxr::UsdGeomXformCache& xformCache)
{
    ParticleObjectProxyInfo* particleObjectProxyInfo = getParticleObjectProxyInfo(actualPath);
    if (!particleObjectProxyInfo || particleObjectProxyInfo->proxyRootPath.IsEmpty())
    {
        return;
    }

    //source
    const UsdPrim particleSetPrim = gStage->GetPrimAtPath(actualPath);
    CARB_ASSERT(particleSetPrim);
    const GfMatrix4d particlesToWorld = xformCache.GetLocalToWorldTransform(particleSetPrim);

    //target
    UsdPrim rootProxyPrim = gStage->GetPrimAtPath(particleObjectProxyInfo->proxyRootPath);
    CARB_ASSERT(rootProxyPrim);

    // sphere scaling - we invert the scaling because we don't want the actual spheres to be larger.
    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, getParticleObjectParticleProxyPath(particleObjectProxyInfo->proxyRootPath));
    if (pointInstancer)
    {
        SdfPath protoPath = getParticleObjectParticleProtoProxyPath(pointInstancer.GetPath());
        if (!protoPath.IsEmpty())
        {
            UsdGeomSphere spherePrim = UsdGeomSphere::Get(gStage, protoPath);
            if (spherePrim) // only do this if the original particle render gfx is a sphere, otherwise let the user handle it.
            {
                GfVec3d scale = GfTransform(particlesToWorld).GetScale();
                for (int i = 0; i < 3; ++i)
                {
                    if (std::abs(scale[i]) <= 1e-6)
                        // sanitize in case of near zero:
                        scale[i] = std::signbit(scale[i]) ? -1e6 : 1e6;
                    else
                        scale[i] = 1.0 / scale[i];
                }
                // set new scale
                const UsdAttribute scaleAttr = spherePrim.GetPrim().CreateAttribute(scaleAttrName, SdfValueTypeNames->Float3);
                if (scaleAttr.IsValid())
                {
                    scaleAttr.Set(GfVec3f(scale));
                }
            }
        }
    }

    UsdAttribute localToWorldAttr;
    localToWorldAttr = rootProxyPrim.CreateAttribute(transformMatrixAttrName, SdfValueTypeNames->Matrix4d);
    CARB_ASSERT(localToWorldAttr);
    localToWorldAttr.Set(particlesToWorld);
}

void ParticlesVisualizationManager::updateParticleObjectPoints(SdfPath actualPath)
{
    ParticleObjectProxyInfo* particleObjectProxyInfo = getParticleObjectProxyInfo(actualPath);
    if (!particleObjectProxyInfo || particleObjectProxyInfo->proxyRootPath.IsEmpty())
    {
        return;
    }
    ParticleSystemProxyInfo* particleSystemProxyInfo = getParticleSystemProxyInfo(particleObjectProxyInfo->actualParticleSystemPath);
    if (!particleSystemProxyInfo || !particleSystemProxyInfo->desc)
    {
        return;
    }

    if (particleObjectProxyInfo->type == ProxyInfoType::eParticleSet)
    {
        ParticleSetProxyInfo& particleSetProxyInfo = static_cast<ParticleSetProxyInfo&>(*particleObjectProxyInfo);

        UsdAttribute pointsAttr;
        UsdAttribute orientationsAttr;
        UsdAttribute scalesAttr;
        getActualParticleSetAttributes(actualPath, mParticlePositionType, particleSystemProxyInfo->hasSmoothing, pointsAttr, orientationsAttr, scalesAttr);

        SdfPath proxyPointInstancerPath = getParticleObjectParticleProxyPath(particleSetProxyInfo.proxyRootPath);
        UsdGeomPointInstancer proxyPointInstancer = UsdGeomPointInstancer::Get(gStage, proxyPointInstancerPath);
        if (proxyPointInstancer)
        {
            VtArray<GfVec3f> points;
            pointsAttr.Get(&points);

            if (points.size() > 0)
            {
                proxyPointInstancer.CreatePositionsAttr().Set(points);
                VtArray<GfQuath> orientations;
                VtArray<GfVec3f> scales;

                // if anisotropy should be visualized we need to get scales/orientations.
                if (mRadiusType == ParticleRadiusType::eAnisotropy)
                {
                    if (orientationsAttr)
                    {
                        orientationsAttr.Get(&orientations);
                        proxyPointInstancer.CreateOrientationsAttr().Set(orientations);
                    }

                    if (scalesAttr)
                    {
                        scalesAttr.Get(&scales);
                        proxyPointInstancer.CreateScalesAttr().Set(scales);
                    }
                }
                else
                {
                    // reset the scales and orientations - we assume unit scaling for all of the particles.
                    {
                        scales = VtArray<GfVec3f>(points.size(), GfVec3f(1.0f));
                        orientations = VtArray<GfQuath>(points.size(), GfQuath(1.0f));
                        proxyPointInstancer.CreateScalesAttr().Set(scales);
                        proxyPointInstancer.CreateOrientationsAttr().Set(orientations);
                    }
                }

                {
                    VtArray<int> indices;
                    proxyPointInstancer.GetProtoIndicesAttr().Get(&indices);
                    size_t oldSize = indices.size();
                    size_t newSize = points.size();
                    indices.resize(newSize);
                    if (newSize > oldSize)
                    {
                        std::memset(indices.data()+oldSize, 0, sizeof(int)*(newSize-oldSize));
                    }
                    proxyPointInstancer.CreateProtoIndicesAttr().Set(indices);
                }
            }
            else
            {
                proxyPointInstancer.CreatePositionsAttr().Set(VtArray<GfVec3f>());
                proxyPointInstancer.CreateProtoIndicesAttr().Set(VtArray<int>());
                proxyPointInstancer.CreateOrientationsAttr().Set(VtArray<GfQuath>());
                proxyPointInstancer.CreateScalesAttr().Set(VtArray<GfVec3f>());
            }
        }
    }
    else if (particleObjectProxyInfo->type == ProxyInfoType::eParticleCloth)
    {
        ParticleClothProxyInfo& particleClothProxyInfo = static_cast<ParticleClothProxyInfo&>(*particleObjectProxyInfo);
        UsdGeomMesh mesh = UsdGeomMesh::Get(gStage, actualPath);
        if (mesh)
        {
            VtArray<GfVec3f> points;
            mesh.GetPointsAttr().Get(&points);

            SdfPath proxyPointInstancerPath = getParticleObjectParticleProxyPath(particleClothProxyInfo.proxyRootPath);
            UsdGeomPointInstancer proxyPointInstancer = UsdGeomPointInstancer::Get(gStage, proxyPointInstancerPath);
            if (proxyPointInstancer)
            {
                if (points.size() > 0)
                {
                    proxyPointInstancer.CreatePositionsAttr().Set(points);

                    {
                        VtArray<int> indices;
                        proxyPointInstancer.GetProtoIndicesAttr().Get(&indices);
                        size_t oldSize = indices.size();
                        size_t newSize = points.size();
                        indices.resize(newSize);
                        if (newSize > oldSize)
                        {
                            std::memset(indices.data()+oldSize, 0, sizeof(int)*(newSize-oldSize));
                        }
                        proxyPointInstancer.CreateProtoIndicesAttr().Set(indices);
                    }
                }
                else
                {
                    proxyPointInstancer.CreatePositionsAttr().Set(VtArray<GfVec3f>());
                    proxyPointInstancer.CreateProtoIndicesAttr().Set(VtArray<int>());
                }
            }

            SdfPath proxyMeshPath = getParticleClothMeshProxyPath(particleClothProxyInfo.proxyRootPath);
            UsdGeomMesh proxyMesh = UsdGeomMesh::Get(gStage, proxyMeshPath);
            if (proxyMesh)
            {
                proxyMesh.CreatePointsAttr().Set(points);
            }
        }
    }
}

//Cloth topology, colors and radii
void ParticlesVisualizationManager::updateParticleObjectMesh(SdfPath actualPath)
{
    ParticleObjectProxyInfo* particleObjectProxyInfo = getParticleObjectProxyInfo(actualPath);
    if (!particleObjectProxyInfo || particleObjectProxyInfo->proxyRootPath.IsEmpty())
    {
        return;
    }
    ParticleSystemProxyInfo* particleSystemProxyInfo = getParticleSystemProxyInfo(particleObjectProxyInfo->actualParticleSystemPath);
    if (!particleSystemProxyInfo)
    {
        return;
    }

    SdfPath proxyPointInstancerPath = getParticleObjectParticleProxyPath(particleObjectProxyInfo->proxyRootPath);
    UsdGeomPointInstancer proxyPointInstancer = UsdGeomPointInstancer::Get(gStage, proxyPointInstancerPath);
    if (proxyPointInstancer)
    {
        bool isFluid = false;
        if (particleObjectProxyInfo->type == ProxyInfoType::eParticleSet)
        {
            isFluid = static_cast<ParticleSetProxyInfo*>(particleObjectProxyInfo)->isFluid;
        }

        SdfPath protoPath = getParticleObjectParticleProtoProxyPath(proxyPointInstancerPath);
        UsdGeomSphere sphere = UsdGeomSphere::Get(gStage, protoPath);

        //color
        GfVec3f color = (particleObjectProxyInfo->type == ProxyInfoType::eParticleSet) ? (isFluid ? kColorFluidParticle : kColorSolidParticle) : kColorClothParticle;
        VtArray<GfVec3f> colorArray(1, color);
        sphere.CreateDisplayColorPrimvar().Set(colorArray);

        //radius
        float radius = getParticleRadius(particleSystemProxyInfo->desc, mRadiusType, isFluid);
        sphere.GetRadiusAttr().Set(double(radius));

        // need to set targets again because otherwise color won't be updated.
        SdfPathVector targets;
        targets.push_back(protoPath);
        proxyPointInstancer.CreatePrototypesRel().SetTargets(targets);
    }

    if (particleObjectProxyInfo->type == ProxyInfoType::eParticleCloth)
    {
        SdfPath proxyMeshPath = getParticleClothMeshProxyPath(particleObjectProxyInfo->proxyRootPath);
        UsdGeomMesh proxyMesh = UsdGeomMesh::Get(gStage, proxyMeshPath);
        if (proxyMesh)
        {
            //topology
            UsdGeomMesh mesh = UsdGeomMesh::Get(gStage, actualPath);
            if (mesh)
            {
                VtArray<int> faceVertexCounts;
                VtArray<int> faceVertexIndices;

                mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);
                mesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);

                proxyMesh.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
                proxyMesh.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
            }

            //color
            VtArray<GfVec3f> colorArray(1, kColorClothMesh);
            proxyMesh.CreateDisplayColorPrimvar().Set(colorArray);
        }
    }
}

void ParticlesVisualizationManager::updateParticleSystemSurfaceMesh(SdfPath actualPath, pxr::UsdGeomXformCache& xformCache)
{
    ParticleSystemProxyInfo* particleSystemProxyInfo = getParticleSystemProxyInfo(actualPath);
    if (!particleSystemProxyInfo || particleSystemProxyInfo->actualIsosurfacePath.IsEmpty() || particleSystemProxyInfo->proxyRootPath.IsEmpty())
    {
        return;
    }

    UsdGeomMesh actualIsosurfaceMesh = UsdGeomMesh::Get(gStage, particleSystemProxyInfo->actualIsosurfacePath);
    if (actualIsosurfaceMesh)
    {
        const GfMatrix4d localToWorld = xformCache.GetLocalToWorldTransform(actualIsosurfaceMesh.GetPrim());

        SdfPath proxyMeshPath = getParticleSystemSurfaceProxyPath(particleSystemProxyInfo->proxyRootPath);
        UsdGeomMesh proxyMesh = UsdGeomMesh::Get(gStage, proxyMeshPath);
        if (proxyMesh)
        {
            VtArray<GfVec3f> points;
            VtArray<int> faceVertexCounts;
            VtArray<int> faceVertexIndices;
            actualIsosurfaceMesh.GetPointsAttr().Get(&points);
            actualIsosurfaceMesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);
            actualIsosurfaceMesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);
            proxyMesh.GetPointsAttr().Set(points);
            proxyMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts);
            proxyMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices);

            VtArray<GfVec3f> colorArray(1, kColorFluidMesh);
            proxyMesh.CreateDisplayColorPrimvar().Set(colorArray);

            UsdAttribute localToWorldAttr;
            localToWorldAttr = proxyMesh.GetPrim().GetParent().CreateAttribute(transformMatrixAttrName, SdfValueTypeNames->Matrix4d);
            CARB_ASSERT(localToWorldAttr);
            localToWorldAttr.Set(localToWorld);
        }
    }
}

void ParticlesVisualizationManager::updateParticleSystemDiffusePoints(SdfPath actualPath, pxr::UsdGeomXformCache& xformCache)
{
    ParticleSystemProxyInfo* particleSystemProxyInfo = getParticleSystemProxyInfo(actualPath);
    if (!particleSystemProxyInfo || !particleSystemProxyInfo->desc || particleSystemProxyInfo->actualDiffusePath.IsEmpty() || particleSystemProxyInfo->proxyRootPath.IsEmpty())
    {
        return;
    }

    SdfPath proxyPointsPath = getParticleSystemDiffuseProxyPath(particleSystemProxyInfo->proxyRootPath);
    UsdGeomPoints proxyPoints = UsdGeomPoints::Get(gStage, proxyPointsPath);
    if (proxyPoints)
    {
        UsdGeomPoints actualDiffusePoints = UsdGeomPoints::Get(gStage, particleSystemProxyInfo->actualDiffusePath);
        if (actualDiffusePoints)
        {
            const GfMatrix4d localToWorld = xformCache.GetLocalToWorldTransform(actualDiffusePoints.GetPrim());

            VtArray<GfVec3f> points;
            actualDiffusePoints.GetPointsAttr().Get(&points);
            proxyPoints.GetPointsAttr().Set(points);

            float width = particleSystemProxyInfo->desc->restOffset * 2.0f;
            VtArray<float> widths ={width};
            proxyPoints.GetWidthsAttr().Set(widths);

            VtArray<GfVec3f> colorArray(1, kColorDiffuse);
            proxyPoints.CreateDisplayColorPrimvar().Set(colorArray);

            UsdAttribute localToWorldAttr;
            localToWorldAttr = proxyPoints.GetPrim().GetParent().CreateAttribute(transformMatrixAttrName, SdfValueTypeNames->Matrix4d);
            CARB_ASSERT(localToWorldAttr);
            localToWorldAttr.Set(localToWorld);
        }
        else
        {
            VtArray<GfVec3f> points;
            proxyPoints.GetPointsAttr().Set(points);
            //also setting the width to zero, as empty points array updates fail to synchronize with hydra.
            VtArray<float> widths ={0.0f};
            proxyPoints.GetWidthsAttr().Set(widths);
        }
    }
}

ParticlesVisualizationManager::ParticleSystemProxyInfo* ParticlesVisualizationManager::getParticleSystemProxyInfo(SdfPath actualParticleSystemPath)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(actualParticleSystemPath);
    return (proxyInfo && proxyInfo->type == ProxyInfoType::eParticleSystem) ? static_cast<ParticleSystemProxyInfo*>(proxyInfo) : nullptr;
}

ParticlesVisualizationManager::ParticleObjectProxyInfo* ParticlesVisualizationManager::getParticleObjectProxyInfo(SdfPath actualParticleObjectPath)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(actualParticleObjectPath);
    return (proxyInfo && (proxyInfo->type == ProxyInfoType::eParticleSet || proxyInfo->type == ProxyInfoType::eParticleCloth)) ? static_cast<ParticleObjectProxyInfo*>(proxyInfo) : nullptr;
}

ParticlesVisualizationManager::ParticleSetProxyInfo* ParticlesVisualizationManager::getParticleSetProxyInfo(SdfPath actualParticleSetPath)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(actualParticleSetPath);
    return (proxyInfo && proxyInfo->type == ProxyInfoType::eParticleSet) ? static_cast<ParticleSetProxyInfo*>(proxyInfo) : nullptr;
}

ParticlesVisualizationManager::ParticleClothProxyInfo* ParticlesVisualizationManager::getParticleClothProxyInfo(SdfPath actualParticleClothPath)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(actualParticleClothPath);
    return (proxyInfo && proxyInfo->type == ProxyInfoType::eParticleCloth) ? static_cast<ParticleClothProxyInfo*>(proxyInfo) : nullptr;
}

SdfPathSet* ParticlesVisualizationManager::getParticleSystemParticleObjects(SdfPath actualParticleSystemPath)
{
    auto it = mParticleSystemToParticleMap.find(actualParticleSystemPath);
    return it != mParticleSystemToParticleMap.end() ? &it->second : nullptr;
}

bool ParticlesVisualizationManager::hasActiveFluidParticleSet(SdfPath actualParticleSystemPath)
{
    SdfPathSet* particleObjectPaths = getParticleSystemParticleObjects(actualParticleSystemPath);
    if (particleObjectPaths)
    {
        for (SdfPath particleObjectPath : *particleObjectPaths)
        {
            if (mProxyManager.isActive(particleObjectPath))
            {
                ParticleSetProxyInfo* particleSetProxyInfo = getParticleSetProxyInfo(particleObjectPath);
                if (particleSetProxyInfo && particleSetProxyInfo->isFluid)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void ParticlesVisualizationManager::clear(bool updateSkin)
{
    if (gStage)
    {
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        while (mParticleSets.size())
        {
            // updating the skin causes flatcache crash on stage close
            removeParticleObject(*mParticleSets.begin(), updateSkin);
        }

        while (mParticleCloths.size())
        {
            // updating the skin causes flatcache crash on stage close
            removeParticleObject(*mParticleCloths.begin(), updateSkin);
        }

        while (mParticleSystems.size())
        {
            // updating the skin causes flatcache crash on stage close
            removeParticleSystem(*mParticleSystems.begin(), updateSkin);
        }
    }

    clearBuffers();
}

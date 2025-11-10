// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/ui/ParticleVisualizationModes.h>
#include "ProxyVisualizationManager.h"
#include "DescCache.h"

namespace omni
{

namespace physx
{

namespace ui
{

class ParticlesVisualizationManager : public ProxyVisualizationClient
{
public:
    explicit ParticlesVisualizationManager(ProxyVisualizationManager& proxyVisualizationManager,
                                           VisualizerMode mode,
                                           ParticlePositionType positionType,
                                           ParticleRadiusType radiusType,
                                           bool showClothMeshLines,
                                           bool showParticleSetParticles,
                                           bool showFluidSurface,
                                           bool showDeformableParticles,
                                           bool showDeformableSurface,
                                           bool showDiffuseParticles);

    void setMode(VisualizerMode mode);
    void setParticleRadiusType(ParticleRadiusType radiusType);
    void setParticlePositionType(ParticlePositionType positionType);
    void enableMeshLineVisualization(bool enable);
    void showParticleSetParticles(bool show);
    void showFluidSurface(bool show);
    void showDeformableParticles(bool show);
    void showDeformableMesh(bool show);
    void showDiffuseParticles(bool show);

    // draw trimesh visualization
    void draw();

    void onAttributeChange_ParticleSystemParams(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_SurfaceMesh(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_DiffusePoints(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_ParticleObjectParams(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_ParticleClothCooking(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_ParticleObjectPoints(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_ParticleObjectMesh(pxr::SdfPath path, pxr::TfToken attributeName);

protected:
    virtual ~ParticlesVisualizationManager();

    // ProxyVisualizationClient implementation
    virtual void release();

    virtual bool isActive()
    {
        return mMode != VisualizerMode::eNone;
    }
    virtual bool isEmpty()
    {
        return mParticleSystems.size() == 0 && mParticleSets.size() == 0 && mParticleCloths.size() == 0;
    }
    virtual bool needsStageParse(pxr::UsdStageWeakPtr stage);

    virtual void handlePrimResync(pxr::SdfPath path);
    virtual void handlePrimRemove(pxr::SdfPath path);
    virtual void handleTransformChange(pxr::SdfPath path);
    virtual void handleVisibilityChange(pxr::SdfPath path);

    virtual void updateTracking();
    virtual void updateModeDirty();
    virtual bool checkMode(pxr::SdfPath actualPath);
    virtual bool checkCompleteness(ProxyInfo& proxyInfo);
    virtual uint32_t updateActiveProxies(ProxyInfo& proxyInfo, pxr::SdfPath actualPath);
    virtual void updateProxyProperties(pxr::UsdGeomXformCache& xformCache);
    virtual void updateActualPurpose(pxr::SdfPath actualPath, bool active);
    virtual void notifyReleaseProxyPrim(pxr::SdfPath proxyPrimPath)
    {
    }
    virtual void clearBuffers();
    //~ProxyVisualizationClient implementation

private:
    struct ParticleSystemProxyInfo : public ProxyInfo
    {
        pxr::SdfPath actualIsosurfacePath;
        pxr::SdfPath actualDiffusePath;
        bool hasSmoothing = false;
        bool hasAnisotropy = false;
        omni::physx::usdparser::ParticleSystemDesc* desc = nullptr;

        virtual ~ParticleSystemProxyInfo()
        {
        }
    };

    // incomplete type
    struct ParticleObjectProxyInfo : public ProxyInfo
    {
        pxr::SdfPath actualParticleSystemPath;

    protected:
        ParticleObjectProxyInfo()
        {
        }
    };

    struct ParticleSetProxyInfo : public ParticleObjectProxyInfo
    {
        bool isFluid = false;
        bool hasDiffuse = false;

        virtual ~ParticleSetProxyInfo()
        {
        }
    };

    struct ParticleClothProxyInfo : public ParticleObjectProxyInfo
    {
        virtual ~ParticleClothProxyInfo()
        {
        }
    };

    // show wrappers
    bool showParticleSetParticles()
    {
        return mShowParticleSetParticles && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }
    bool showFluidSurface()
    {
        return mShowFluidSurface && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }
    bool showDeformableParticles()
    {
        return mShowDeformableParticles && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }
    bool showDeformableMesh()
    {
        return mShowDeformableMesh && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }
    bool showDiffuseParticles()
    {
        return mShowDiffuseParticles && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }

    // create/set debug prims
    pxr::UsdPrim createParticleInstancerProxyPrim(pxr::SdfPath particleInstancerProxyPath);
    pxr::UsdPrim createMeshProxyPrim(pxr::SdfPath meshProxyPath);
    pxr::UsdPrim createPointsProxyPrim(pxr::SdfPath pointsProxyPath);

    ParticleSystemProxyInfo* createParticleSystemProxyInfo(pxr::SdfPath actualParticleSystemPath);
    ParticleSetProxyInfo* createParticleSetProxyInfo(pxr::SdfPath actualParticleSetPath);
    ParticleClothProxyInfo* createParticleClothProxyInfo(pxr::SdfPath actualParticleClothPath);

    // update tracking
    void addParticleSystem(pxr::SdfPath path);
    void removeParticleSystem(pxr::SdfPath path, bool updateSkin);
    void updateParticleSystem(pxr::SdfPath path);
    void addParticleObject(pxr::SdfPath path);
    void removeParticleObject(pxr::SdfPath path, bool updateSkin);
    void updateParticleObject(pxr::SdfPath path);


    // update helpers
    void updateParticleObjectTransform(pxr::SdfPath actualPath, pxr::UsdGeomXformCache& xformCache);
    void updateParticleObjectPoints(pxr::SdfPath actualPath);
    void updateParticleObjectMesh(pxr::SdfPath actualPath);
    void updateParticleSystemSurfaceMesh(pxr::SdfPath actualPath, pxr::UsdGeomXformCache& xformCache);
    void updateParticleSystemDiffusePoints(pxr::SdfPath actualPath, pxr::UsdGeomXformCache& xformCache);

    // helper functions
    ParticleSystemProxyInfo* getParticleSystemProxyInfo(pxr::SdfPath actualParticleSystemPath);
    ParticleObjectProxyInfo* getParticleObjectProxyInfo(pxr::SdfPath actualParticleObjectPath);
    ParticleSetProxyInfo* getParticleSetProxyInfo(pxr::SdfPath actualParticleSetPath);
    ParticleClothProxyInfo* getParticleClothProxyInfo(pxr::SdfPath actualParticleClothPath);
    pxr::SdfPathSet* getParticleSystemParticleObjects(pxr::SdfPath actualParticleSystemPath);
    bool hasActiveFluidParticleSet(pxr::SdfPath actualParticleSystemPath);

    // called by release
    void clear(bool updateSkin);

private:
    // class-scope using:
    using SdfPathToPathSetMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;

    // ----------------
    // tracking
    // ----------------
    // applies to all actual objects of interest in the stage, and is maintained if visualizer mode is "Selected" or
    // "All".
    pxr::SdfPathSet mParticleSets; // set of all particle sets in the stage
    pxr::SdfPathSet mParticleCloths; // set of all particle cloths in the stage
    pxr::SdfPathSet mParticleSystems; // set of all particle systems in the stage

    SdfPathToPathSetMap mParticleSystemToParticleMap; // map from particle systems to particle instances

    // ----------------
    // active state
    // ----------------
    // data structures that are maintained only for active objects that are being visualized with session proxies
    // or which is relevant for filter evaluation
    VisualizerMode mMode; // mode for visualization [none, all, selected]
    bool mShowParticleSetParticles; // show particle set particles
    bool mShowFluidSurface; // show fluid surface
    bool mShowDeformableParticles; // show deformance particles
    bool mShowDeformableMesh; // show deformable surface
    bool mShowDiffuseParticles; // show diffuse particles

    // ----------------
    // event buffering
    // ----------------
    // buffers to store notifications until next stage update, or batch notifications in the update itself
    pxr::SdfPathSet mBufferParticleObjectsToAdd;
    pxr::SdfPathSet mBufferParticleObjectsToRemove;
    pxr::SdfPathSet mBufferParticleObjectsToUpdate;
    pxr::SdfPathSet mBufferParticleObjectsToUpdateTransform;
    pxr::SdfPathSet mBufferParticleObjectsToUpdatePoints;
    pxr::SdfPathSet mBufferParticleObjectsToUpdateMesh;

    pxr::SdfPathSet mBufferParticleSystemsToAdd;
    pxr::SdfPathSet mBufferParticleSystemsToRemove;
    pxr::SdfPathSet mBufferParticleSystemsToUpdate;
    pxr::SdfPathSet mBufferParticleSystemsToUpdateSurfaceMesh;
    pxr::SdfPathSet mBufferParticleSystemsToUpdateDiffusePoints;

    bool mBufferModeDirty;

    ParticleRadiusType mRadiusType; // Which type of the radius should we use
    ParticlePositionType mParticlePositionType; // which type of particle positions we should use
    bool mMeshLineVisualizationEnabled; // if mesh line drawing is enabled

    DescCache mDescCache; // Cache for cloth descriptors

    ProxyVisualizationManager& mProxyManager;
};

} // namespace ui
} // namespace physx
} // namespace omni

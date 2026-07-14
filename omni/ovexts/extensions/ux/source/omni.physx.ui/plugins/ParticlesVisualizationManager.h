// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
                                           bool showParticleSetParticles,
                                           bool showFluidSurface,
                                           bool showDiffuseParticles);

    void setMode(VisualizerMode mode);
    void setParticleRadiusType(ParticleRadiusType radiusType);
    void setParticlePositionType(ParticlePositionType positionType);
    void showParticleSetParticles(bool show);
    void showFluidSurface(bool show);
    void showDiffuseParticles(bool show);

    void onAttributeChange_ParticleSystemParams(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_SurfaceMesh(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_DiffusePoints(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_ParticleObjectParams(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_ParticleObjectPoints(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_ParticleObjectMesh(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);

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
        return mParticleSystems.size() == 0 && mParticleSets.size() == 0;
    }
    virtual bool needsStageParse(PXR_NS::UsdStageWeakPtr stage);

    virtual void handlePrimResync(PXR_NS::SdfPath path);
    virtual void handlePrimRemove(PXR_NS::SdfPath path);
    virtual void handleTransformChange(PXR_NS::SdfPath path);
    virtual void handleVisibilityChange(PXR_NS::SdfPath path);

    virtual void updateTracking();
    virtual void updateModeDirty();
    virtual bool checkMode(PXR_NS::SdfPath actualPath);
    virtual bool checkCompleteness(ProxyInfo& proxyInfo);
    virtual uint32_t updateActiveProxies(ProxyInfo& proxyInfo, PXR_NS::SdfPath actualPath);
    virtual void updateProxyProperties(PXR_NS::UsdGeomXformCache& xformCache);
    virtual void updateActualPurpose(PXR_NS::SdfPath actualPath, bool active);
    virtual void notifyReleaseProxyPrim(PXR_NS::SdfPath proxyPrimPath)
    {
    }
    virtual void clearBuffers();
    //~ProxyVisualizationClient implementation

private:
    struct ParticleSystemProxyInfo : public ProxyInfo
    {
        PXR_NS::SdfPath actualIsosurfacePath;
        PXR_NS::SdfPath actualDiffusePath;
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
        PXR_NS::SdfPath actualParticleSystemPath;

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

    // show wrappers
    bool showParticleSetParticles()
    {
        return mShowParticleSetParticles && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }
    bool showFluidSurface()
    {
        return mShowFluidSurface && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }
    bool showDiffuseParticles()
    {
        return mShowDiffuseParticles && mRadiusType != ParticleRadiusType::eRenderGeometry;
    }

    // create/set debug prims
    PXR_NS::UsdPrim createParticleInstancerProxyPrim(PXR_NS::SdfPath particleInstancerProxyPath);
    PXR_NS::UsdPrim createMeshProxyPrim(PXR_NS::SdfPath meshProxyPath);
    PXR_NS::UsdPrim createPointsProxyPrim(PXR_NS::SdfPath pointsProxyPath);

    ParticleSystemProxyInfo* createParticleSystemProxyInfo(PXR_NS::SdfPath actualParticleSystemPath);
    ParticleSetProxyInfo* createParticleSetProxyInfo(PXR_NS::SdfPath actualParticleSetPath);

    // update tracking
    void addParticleSystem(PXR_NS::SdfPath path);
    void removeParticleSystem(PXR_NS::SdfPath path, bool updateSkin);
    void updateParticleSystem(PXR_NS::SdfPath path);
    void addParticleObject(PXR_NS::SdfPath path);
    void removeParticleObject(PXR_NS::SdfPath path, bool updateSkin);
    void updateParticleObject(PXR_NS::SdfPath path);


    // update helpers
    void updateParticleObjectTransform(PXR_NS::SdfPath actualPath, PXR_NS::UsdGeomXformCache& xformCache);
    void updateParticleObjectPoints(PXR_NS::SdfPath actualPath);
    void updateParticleObjectMesh(PXR_NS::SdfPath actualPath);
    void updateParticleSystemSurfaceMesh(PXR_NS::SdfPath actualPath, PXR_NS::UsdGeomXformCache& xformCache);
    void updateParticleSystemDiffusePoints(PXR_NS::SdfPath actualPath, PXR_NS::UsdGeomXformCache& xformCache);

    // helper functions
    ParticleSystemProxyInfo* getParticleSystemProxyInfo(PXR_NS::SdfPath actualParticleSystemPath);
    ParticleObjectProxyInfo* getParticleObjectProxyInfo(PXR_NS::SdfPath actualParticleObjectPath);
    ParticleSetProxyInfo* getParticleSetProxyInfo(PXR_NS::SdfPath actualParticleSetPath);
    PXR_NS::SdfPathSet* getParticleSystemParticleObjects(PXR_NS::SdfPath actualParticleSystemPath);
    bool hasActiveFluidParticleSet(PXR_NS::SdfPath actualParticleSystemPath);

    // called by release
    void clear(bool updateSkin);

private:
    // class-scope using:
    using SdfPathToPathSetMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPathSet, PXR_NS::SdfPath::Hash>;

    // ----------------
    // tracking
    // ----------------
    // applies to all actual objects of interest in the stage, and is maintained if visualizer mode is "Selected" or
    // "All".
    PXR_NS::SdfPathSet mParticleSets; // set of all particle sets in the stage
    PXR_NS::SdfPathSet mParticleSystems; // set of all particle systems in the stage

    SdfPathToPathSetMap mParticleSystemToParticleMap; // map from particle systems to particle instances

    // ----------------
    // active state
    // ----------------
    // data structures that are maintained only for active objects that are being visualized with session proxies
    // or which is relevant for filter evaluation
    VisualizerMode mMode; // mode for visualization [none, all, selected]
    bool mShowParticleSetParticles; // show particle set particles
    bool mShowFluidSurface; // show fluid surface
    bool mShowDiffuseParticles; // show diffuse particles

    // ----------------
    // event buffering
    // ----------------
    // buffers to store notifications until next stage update, or batch notifications in the update itself
    PXR_NS::SdfPathSet mBufferParticleObjectsToAdd;
    PXR_NS::SdfPathSet mBufferParticleObjectsToRemove;
    PXR_NS::SdfPathSet mBufferParticleObjectsToUpdate;
    PXR_NS::SdfPathSet mBufferParticleObjectsToUpdateTransform;
    PXR_NS::SdfPathSet mBufferParticleObjectsToUpdatePoints;
    PXR_NS::SdfPathSet mBufferParticleObjectsToUpdateMesh;

    PXR_NS::SdfPathSet mBufferParticleSystemsToAdd;
    PXR_NS::SdfPathSet mBufferParticleSystemsToRemove;
    PXR_NS::SdfPathSet mBufferParticleSystemsToUpdate;
    PXR_NS::SdfPathSet mBufferParticleSystemsToUpdateSurfaceMesh;
    PXR_NS::SdfPathSet mBufferParticleSystemsToUpdateDiffusePoints;

    bool mBufferModeDirty;

    ParticleRadiusType mRadiusType; // Which type of the radius should we use
    ParticlePositionType mParticlePositionType; // which type of particle positions we should use
    DescCache mDescCache;

    ProxyVisualizationManager& mProxyManager;
};

} // namespace ui
} // namespace physx
} // namespace omni

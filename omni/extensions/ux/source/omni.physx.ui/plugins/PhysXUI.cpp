// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

#define CARB_EXPORTS
#include "PhysXJointAuthoring.h"

#include <omni/physx/IPhysxUI.h>
#include <private/omni/physx/IPhysxUIPrivate.h>
#include <omni/physx/IPhysxStatistics.h>

#include "DebugVisualization.h"
#include "DebugVisualizationPhysX.h"
#include "DebugVisualizationFixedTendon.h"
#include "DeformableBodyVisualizationManagerDeprecated.h"
#include "DeformableSurfaceVisualizationManagerDeprecated.h"
#include "DeformableBodyVisualizationManager.h"
#include "ParticleAuthoring.h"
#include "ProxyVisualizationManager.h"
#include "ParticlesVisualizationManager.h"
#include "AttachmentsVisualizationManagerDeprecated.h"
#include "AttachmentAuthoringDeprecated.h"
#include "AttachmentAuthoring.h"
#include "SpatialTendonAuthoring.h"
#include "NoticeHandler.h"
#include "InputManager.h"
#include <omni/physx/IPhysxVisualization.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxAttachmentPrivate.h>
#include <private/omni/physx/IPhysxParticlesPrivate.h>
#include <omni/physx/IPhysxFabric.h>
#include <private/omni/physx/ui/VisualizerMode.h>
#include <private/omni/physx/ui/ParticleVisualizationModes.h>

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/events/EventsUtils.h>
#include <carb/settings/ISettings.h>
#include <imgui.h>
#include <omni/renderer/IDebugDraw.h>
#include <carb/eventdispatcher/IEventDispatcher.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/kit/IViewport.h>
#include <omni/kit/IAppWindow.h>
#include <omni/ui/IGlyphManager.h>
#include <carb/tokens/ITokens.h>
#include <omni/kit/ViewportWindowUtils.h>
#include <omni/kit/KitUpdateOrder.h>
#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <carb/tasking/ITasking.h>
#include <omni/ext/IExt.h>
#include <omni/ext/ExtensionsUtils.h>
#include <omni/kit/IApp.h>

#include <string>

using namespace omni::physx::ui;
using namespace pxr;

const static carb::RStringKey kObserverName("omni.physxui");

DebugVisualization* gDebugVisualization = nullptr;
DebugVisualizationPhysX* gDebugVisualizationPhysX = nullptr;
PhysXJointAuthoring* gPhysXJointAuthoring = nullptr;
FixedTendonVisualizer* gFixedTendonVisualization = nullptr;
ParticleAuthoring* gParticleAuthoring = nullptr;
ProxyVisualizationManager* gProxyVisualizationManager = nullptr;
ParticlesVisualizationManager* gParticlesVisualizationManager = nullptr;
AttachmentAuthoringDeprecated* gAttachmentAuthoringDeprecated = nullptr;
AttachmentAuthoring* gAttachmentAuthoring = nullptr;
AttachmentsVisualizationManagerDeprecated* gAttachmentsVisualizationManagerDeprecated = nullptr;
DeformableBodyVisualizationManagerDeprecated* gDeformableBodyVisualizationManagerDeprecated = nullptr;
DeformableSurfaceVisualizationManagerDeprecated* gDeformableSurfaceVisualizationManagerDeprecated = nullptr;
DeformableBodyVisualizationManager* gDeformableVisualizationManager = nullptr;
SpatialTendonManager* gSpatialTendonManager = nullptr;
InputManager* gInputManager = nullptr;
pxr::TfNotice::Key gUsdNoticeListenerKey;
omni::physx::ui::UsdNoticeListener* gUsdNoticeListener = nullptr;
omni::physx::IPhysx* gPhysX = nullptr;
omni::physx::IPhysxCooking* gPhysXCooking = nullptr;
omni::physx::IPhysxCookingPrivate* gPhysXCookingPrivate = nullptr;
omni::physx::IPhysxAttachmentPrivate* gPhysXAttachmentPrivate = nullptr;
omni::physx::IPhysxParticlesPrivate* gPhysXParticlesPrivate = nullptr;

omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad = nullptr;

UsdStageRefPtr gStage = nullptr;
extern bool gBlockNoticeHandle;
carb::settings::ISettings* gSettings = nullptr;
omni::kit::IViewportWindow* gViewportWindow = nullptr;
omni::timeline::TimelinePtr gTimeline = nullptr;
carb::input::IInput* gInput = nullptr;

std::array<carb::eventdispatcher::ObserverGuard, 3> gStageEvtSub;
carb::events::ISubscriptionPtr gTimelineEvtSub;
carb::events::ISubscriptionPtr gViewportUiEvtSub;
omni::kit::StageUpdateNode* gStageUpdateNode = nullptr;

uint8_t gProxySelectionGroup = 0;

static std::vector<carb::dictionary::SubscriptionId*> uiSettingCallbackSubscriptions;

std::string gExtensionPath;

class ExtensionImpl : public omni::ext::IExt
{
public:
    void onStartup(const char* extId) override
    {
        omni::kit::IApp* app = carb::getCachedInterface<omni::kit::IApp>();
        gExtensionPath = omni::ext::getExtensionPath(app->getExtensionManager(), extId);
        gPhysXJointAuthoring = new PhysXJointAuthoring();
        gPhysXJointAuthoring->init(gExtensionPath);
    }

    void onShutdown() override
    {
        gPhysXJointAuthoring->shutdown();
        delete gPhysXJointAuthoring;
    }
};

const struct carb::PluginImplDesc kPluginImpl = { "omni.physxui.plugin", "PhysXUI", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, ExtensionImpl, omni::physx::ui::IPhysxUI, omni::physx::ui::IPhysxUIPrivate)
CARB_PLUGIN_IMPL_DEPS(carb::settings::ISettings,
                      omni::kit::IStageUpdate,
                      omni::timeline::ITimeline,
                      omni::physx::IPhysxVisualization,
                      omni::physx::IPhysx,
                      omni::physx::IPhysxCooking,
                      carb::dictionary::IDictionary,
                      omni::physx::usdparser::IPhysxUsdLoad,
                      omni::physx::IPhysxAttachmentPrivate,
                      omni::physx::IPhysxParticlesPrivate,
                      omni::physx::IPhysxStatistics,
                      omni::renderer::IDebugDraw,
                      carb::tokens::ITokens,
                      omni::physics::ui::IUsdPhysicsUI,
                      omni::ui::IGlyphManager,
                      carb::tasking::ITasking,
                      carb::input::IInput,
                      omni::kit::IApp
)

void setVisualizationDistance(float distance)
{
    if (gDebugVisualizationPhysX)
        gDebugVisualizationPhysX->setVisualizationDistance(distance);
}

void enableDebugVisualization(bool enable)
{
    if (gDebugVisualizationPhysX)
        gDebugVisualizationPhysX->enableDebugVisualization(enable);
}

void enableDebugNormalsVisualization(bool enable)
{
    if (gDebugVisualization)
        gDebugVisualization->enableNormalsVisualization(enable);
}

void setCollsionMeshType(const char* type)
{
    if (gDebugVisualization)
        gDebugVisualization->setCollisionMeshType(type);
}

void enableCollisionMeshVisualization(bool enable)
{
    if (gDebugVisualization)
        gDebugVisualization->enableCollisionMeshVisualization(enable);
}

void explodeViewDistance(float distance)
{
    if ( gDebugVisualization )
    {
        gDebugVisualization->explodeViewDistance(distance);
    }
}

void selectSpatialTendonAttachmentHelper(const pxr::SdfPath linkBodyPath, const pxr::TfToken instanceName)
{
    gSpatialTendonManager->selectSpatialTendonAttachmentHelper(linkBodyPath, instanceName);
}

void setTendonVisualizationFilter(const char* instanceName)
{
    gFixedTendonVisualization->setInstanceFilter(instanceName);
}

void setVehicleVisualization(PhysXVehicleVisualizationParameter::Enum param, bool enable)
{
    if (gDebugVisualization)
        gDebugVisualization->setVehicleVisualization(param, enable);
}

bool getVehicleVisualization(PhysXVehicleVisualizationParameter::Enum param)
{
    if (gDebugVisualization)
        return gDebugVisualization->getVehicleVisualization(param);
    else
        return false;
}

void getAttachmentsDeprecated(size_t*& dstPathLengths,
                              uint32_t& dstPathLengthsSize,
                              uint8_t*& dstPathData,
                              size_t& dstPathDataSize,
                              const pxr::SdfPath& primPath,
                              void* (*allocateBytes)(size_t))
{
    if (gAttachmentAuthoringDeprecated)
    {
        SdfPathSet attachmentPaths;
        gAttachmentAuthoringDeprecated->getAttachments(attachmentPaths, primPath);

        dstPathLengthsSize = (uint32_t)attachmentPaths.size();
        dstPathLengths = (size_t*)allocateBytes(dstPathLengthsSize * sizeof(size_t));

        // count memory
        uint32_t index = 0;
        size_t dataByteSize = 0;
        for (SdfPathSet::const_iterator cit = attachmentPaths.cbegin(); cit != attachmentPaths.cend(); ++cit)
        {
            const std::string string = cit->GetString();
            size_t strSize = string.length() + 1;
            dstPathLengths[index++] = strSize;
            dataByteSize += strSize;
        }

        // allocate and copy
        dstPathDataSize = dataByteSize;
        dstPathData = (uint8_t*)allocateBytes(dstPathDataSize);
        dataByteSize = 0;
        for (SdfPathSet::const_iterator cit = attachmentPaths.cbegin(); cit != attachmentPaths.cend(); ++cit)
        {
            const std::string string = cit->GetString();
            size_t strSize = string.length();
            std::memcpy(dstPathData + dataByteSize, string.c_str(), strSize + 1);
            dataByteSize += strSize + 1;
        }
    }
    else
    {
        dstPathLengths = nullptr;
        dstPathLengthsSize = 0;
        dstPathData = nullptr;
        dstPathDataSize = 0;
    }
}

void refreshAttachment(pxr::SdfPath attachmentPath)
{
    if (gAttachmentAuthoringDeprecated)
    {
        gAttachmentAuthoringDeprecated->refreshAttachment(attachmentPath);
    }
    if (gAttachmentAuthoring)
    {
        gAttachmentAuthoring->refreshAttachment(attachmentPath);
    }
}

void setCameraPos(const carb::Float3& pos)
{
    if (gDebugVisualization)
    {
        gDebugVisualization->setCameraPos(pxr::GfVec3f(pos.x, pos.y, pos.z));
    }
}

void enableRedrawOptimizations(bool enable)
{
    if (gDebugVisualization)
        gDebugVisualization->enableRedrawOptimizations(enable);
}

struct DrawCustomText
{
    std::string text;
    float x, y;
    uint32_t color;
};


// convert from argb to abgr
uint32_t convertColor(uint32_t inColor)
{
    uint32_t outColor = (inColor & 0xFF000000) |
        ((inColor & 0x00FF0000) >> 16) |
        (inColor & 0x0000FF00) |
        ((inColor & 0x000000FF) << 16);
    return outColor;
}

void internalDrawOmniSceneUIOverlays(GfMatrix4d viewMatrix, GfMatrix4d projMatrix, carb::Float4 viewportRect, std::function<void(bool)>& enableViewport2Picking)
{
    if (gViewportWindow)
    {
        // scristiano: we are only drawing omni.scene.ui compatible overlays under viewport 2
        // because we must keep the old onUIDraw callback for backward compatibility (and we don't want to draw 2 times)
        // In theory we could use this code path for both Viewport 1 and 2 but we need to fix visual tests first
        // as hideUi setting is also hiding somehow omni.scene.ui overlays (to be investigated)
        return;
    }

    if (gDebugVisualizationPhysX)
    {
        gDebugVisualizationPhysX->setCameraMatrix(viewMatrix);
    }
    if (gParticlesVisualizationManager)
    {
        gParticlesVisualizationManager->draw();
    }
    if (gDeformableSurfaceVisualizationManagerDeprecated)
    {
        gDeformableSurfaceVisualizationManagerDeprecated->draw();
    }
}

void onUIDraw(carb::events::IEvent* e)
{
    if (!gStage)
        return;

    if ( !gDebugVisualization && !gSpatialTendonManager)
    {
        return;
    }

    GfMatrix4d viewMatrix;
    GfMatrix4d projMatrix;
    carb::Float4 viewportRect;
    omni::kit::getUiDrawPayloadFromEvent(*e, viewMatrix.data(), projMatrix.data(), viewportRect);

    if (gFixedTendonVisualization)
    {
        gFixedTendonVisualization->draw(viewMatrix, projMatrix, viewportRect);
    }
    if (gParticlesVisualizationManager)
    {
        gParticlesVisualizationManager->draw();
    }
    if (gDeformableSurfaceVisualizationManagerDeprecated)
    {
        gDeformableSurfaceVisualizationManagerDeprecated->draw();
    }
}

static bool selectableLeftCheck(std::string label,
                                bool checked,
                                bool selected = false,
                                ImGuiSelectableFlags flags = 0)
{
    auto selectGlyph = carb::getCachedInterface<omni::ui::IGlyphManager>()->getGlyphInfo(checked ? "${glyphs}/check_solid.svg" : "${glyphs}/none.svg");
    label = std::string(selectGlyph.code) + " " + label;
    return ImGui::Selectable(label.c_str(), selected, flags, ImVec2(0.0f, 0.0f));
}

bool onMenuDraw()
{
    auto selectableFlags = ImGuiSelectableFlags_DontClosePopups;
    const char* glyphName = "${glyphs}/none.svg";
    auto showtypeGlyph = carb::getCachedInterface<omni::ui::IGlyphManager>()->getGlyphInfo(glyphName);

    if (ImGui::BeginMenu((std::string(showtypeGlyph.code) + " Physics###ViewportWindowPhysics").c_str(), true))
    {
        bool showJoints = gSettings->get<bool>(omni::physx::kSettingDisplayJoints);
        if (selectableLeftCheck("Joints", showJoints, false, selectableFlags))
        {
            showJoints = !showJoints;
            gSettings->set(omni::physx::kSettingDisplayJoints, showJoints);
            // settings changed callbacks handle update of joint authoring
        }

        if (ImGui::BeginMenu((std::string(showtypeGlyph.code) + " Colliders###ViewportWindowColliders").c_str(), true))
        {
            auto MenuItem = [&](VisualizerMode menuMode, const char* title) {
                VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayColliders));
                if (selectableLeftCheck(title, vizMode == menuMode, false, selectableFlags))
                {
                    gSettings->setInt(omni::physx::kSettingDisplayColliders, int32_t(menuMode));
                    // settings changed callbacks handle update of debug viz
                }
            };

            MenuItem(VisualizerMode::eNone, "None");
            MenuItem(VisualizerMode::eSelected, "Selected");
            MenuItem(VisualizerMode::eAll, "All");

            ImGui::Separator();

            bool showNormals = gSettings->get<bool>(omni::physx::kSettingDisplayColliderNormals);
            if (selectableLeftCheck("Normals", showNormals, false, selectableFlags))
            {
                showNormals = !showNormals;
                gSettings->set(omni::physx::kSettingDisplayColliderNormals, showNormals);
                enableDebugNormalsVisualization(showNormals);
                // settings changed callbacks handle update of joint authoring
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu((std::string(showtypeGlyph.code) + " Tendons###ViewportWindowTendons").c_str(), true))
        {
            auto MenuItem = [&](VisualizerMode menuMode, const char* title) {
                VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayTendons));
                if (selectableLeftCheck(title, vizMode == menuMode, false, selectableFlags))
                {
                    gSettings->setInt(omni::physx::kSettingDisplayTendons, int32_t(menuMode));
                    // settings changed callbacks handle update of tendon viz
                }
            };

            MenuItem(VisualizerMode::eNone, "None");
            MenuItem(VisualizerMode::eSelected, "Selected");
            MenuItem(VisualizerMode::eAll, "All");
            ImGui::EndMenu();
        }

        bool deformableBetaEnabled = gSettings->getAsBool(omni::physx::kSettingEnableDeformableBeta);

        // DEPRECATED
        if (!deformableBetaEnabled)
        {
            if (ImGui::BeginMenu(
                    (std::string(showtypeGlyph.code) + " Deformable Body (deprecated)###ViewportWindowDeformableBody(deprecated)").c_str(), true))
            {
                auto MenuItemMode = [&](VisualizerMode menuMode, const char* title) {
                    VisualizerMode vizMode =
                        VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableBodies));
                    if (selectableLeftCheck(title, vizMode == menuMode, false, selectableFlags))
                    {
                        gSettings->setInt(omni::physx::kSettingDisplayDeformableBodies, int32_t(menuMode));
                    }
                };

                MenuItemMode(VisualizerMode::eNone, "None");
                MenuItemMode(VisualizerMode::eSelected, "Selected");
                MenuItemMode(VisualizerMode::eAll, "All");

                ImGui::Separator();

                auto MenuItemType = [&](DeformableBodyVisualizerTypeDeprecated menuType, const char* title) {
                    DeformableBodyVisualizerTypeDeprecated vizType = DeformableBodyVisualizerTypeDeprecated(
                        gSettings->getAsInt(omni::physx::kSettingDisplayDeformableBodyType));
                    if (selectableLeftCheck(title, vizType == menuType, false, selectableFlags))
                    {
                        gSettings->setInt(omni::physx::kSettingDisplayDeformableBodyType, int32_t(menuType));
                        // settings changed callbacks handle update of SB viz manager
                    }
                };
                MenuItemType(DeformableBodyVisualizerTypeDeprecated::eSimulation, "Simulation");
                MenuItemType(DeformableBodyVisualizerTypeDeprecated::eCollision, "Collision");

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu(
                    (std::string(showtypeGlyph.code) + " Attachments (deprecated)###ViewportWindowAttachments(deprecated)").c_str(), true))
            {
                auto MenuItem = [&](VisualizerMode menuMode, const char* title) {
                    VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayAttachments));
                    if (selectableLeftCheck(title, vizMode == menuMode, false, selectableFlags))
                    {
                        gSettings->setInt(omni::physx::kSettingDisplayAttachments, int32_t(menuMode));
                        // settings changed callbacks handle update of tendon viz
                    }
                };

                MenuItem(VisualizerMode::eNone, "None");
                MenuItem(VisualizerMode::eSelected, "Selected");
                MenuItem(VisualizerMode::eAll, "All");

                ImGui::Separator();

                bool hideActor0 = gSettings->get<bool>(omni::physx::kSettingDisplayAttachmentsHideActor0);
                if (selectableLeftCheck("Hide Actor 0", hideActor0, false, selectableFlags))
                {
                    hideActor0 = !hideActor0;
                    gSettings->set(omni::physx::kSettingDisplayAttachmentsHideActor0, hideActor0);
                }

                bool hideActor1 = gSettings->get<bool>(omni::physx::kSettingDisplayAttachmentsHideActor1);
                if (selectableLeftCheck("Hide Actor 1", hideActor0, false, selectableFlags))
                {
                    hideActor1 = !hideActor1;
                    gSettings->set(omni::physx::kSettingDisplayAttachmentsHideActor1, hideActor1);
                }

                ImGui::EndMenu();
            }
        }
        //~DEPRECATED

        if (deformableBetaEnabled)
        {
            if (ImGui::BeginMenu(
                    (std::string(showtypeGlyph.code) + " Deformables (beta)###ViewportWindowDeformables(beta)").c_str(), true))
            {
                auto MenuItemMode = [&](VisualizerMode menuMode, const char* title) {
                    VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformables));
                    if (selectableLeftCheck(title, vizMode == menuMode, false, selectableFlags))
                    {
                        gSettings->setInt(omni::physx::kSettingDisplayDeformables, int32_t(menuMode));
                    }
                };

                MenuItemMode(VisualizerMode::eNone, "None");
                MenuItemMode(VisualizerMode::eSelected, "Selected");
                MenuItemMode(VisualizerMode::eAll, "All");

                ImGui::Separator();

                auto MenuItemType = [&](DeformableVisualizerMeshType menuType, const char* title) {
                    DeformableVisualizerMeshType vizType =
                        DeformableVisualizerMeshType(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableMeshType));
                    if (selectableLeftCheck(title, vizType == menuType, false, selectableFlags))
                    {
                        gSettings->setInt(omni::physx::kSettingDisplayDeformableMeshType, int32_t(menuType));
                        // settings changed callbacks handle update of SB viz manager
                    }
                };

                MenuItemType(DeformableVisualizerMeshType::eSimulationDefault, "Simulation: Default Pose");
                MenuItemType(DeformableVisualizerMeshType::eSimulationBind, "Simulation: Bind Pose");
                MenuItemType(DeformableVisualizerMeshType::eSimulationRestShape, "Simulation: Rest Shape");
                MenuItemType(DeformableVisualizerMeshType::eCollisionDefault, "Collision: Default Pose");
                MenuItemType(DeformableVisualizerMeshType::eCollisionBind, "Collision: Bind Pose");

                ImGui::Separator();

                // display attachments visualization
                bool displayDeformableAttachments =
                    gSettings->get<bool>(omni::physx::kSettingDisplayDeformableAttachments);
                if (selectableLeftCheck("Display Attachments Visualization", displayDeformableAttachments, false,
                                        selectableFlags))
                {
                    displayDeformableAttachments = !displayDeformableAttachments;
                    gSettings->set(omni::physx::kSettingDisplayDeformableAttachments, displayDeformableAttachments);
                }

                ImGui::EndMenu();
            }
        }

        if (ImGui::BeginMenu(
                (std::string(showtypeGlyph.code) + " Particles###ViewportWindowParticles").c_str(), true))
        {
            auto MenuItemMode = [&](VisualizerMode menuMode, const char* title)
            {
                VisualizerMode vizMode =
                    VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayParticles));
                if (selectableLeftCheck(title, vizMode == menuMode, false, selectableFlags))
                {
                    gSettings->setInt(omni::physx::kSettingDisplayParticles, int32_t(menuMode));
                }
            };

            MenuItemMode(VisualizerMode::eNone, "None");
            MenuItemMode(VisualizerMode::eSelected, "Selected");
            MenuItemMode(VisualizerMode::eAll, "All");

            ImGui::Separator();

            // Particle set particles
            bool showParticleSetParticles = gSettings->get<bool>(omni::physx::kSettingDisplayParticlesShowParticleSetParticles);
            if (selectableLeftCheck("Particles", showParticleSetParticles, false, selectableFlags))
            {
                showParticleSetParticles = !showParticleSetParticles;
                gSettings->set(omni::physx::kSettingDisplayParticlesShowParticleSetParticles, showParticleSetParticles);
            }

            // Fluid surface
            bool showFluidSurface = gSettings->get<bool>(omni::physx::kSettingDisplayParticlesShowFluidSurface);
            if (selectableLeftCheck("Fluid Surface", showFluidSurface, false, selectableFlags))
            {
                showFluidSurface = !showFluidSurface;
                gSettings->set(omni::physx::kSettingDisplayParticlesShowFluidSurface, showFluidSurface);
            }

			// DEPRECATED
            // Deformable particles
            bool showDeformableParticles = gSettings->get<bool>(omni::physx::kSettingDisplayParticlesShowDeformableParticles);
            if (selectableLeftCheck("Cloth Particles (deprecated)", showDeformableParticles, false, selectableFlags))
            {
                showDeformableParticles = !showDeformableParticles;
                gSettings->set(omni::physx::kSettingDisplayParticlesShowDeformableParticles, showDeformableParticles);
            }

            // Deformable mesh
            bool showDeformableMesh = gSettings->get<bool>(omni::physx::kSettingDisplayParticlesShowDeformableMesh);
            if (selectableLeftCheck("Cloth Mesh", showDeformableMesh, false, selectableFlags))
            {
                showDeformableMesh = !showDeformableMesh;
                gSettings->set(omni::physx::kSettingDisplayParticlesShowDeformableMesh, showDeformableMesh);
            }
            //~DEPRECATED

            ImGui::Separator();

            // show diffuseParticles
            bool showDiffuseParticles = gSettings->get<bool>(omni::physx::kSettingDisplayParticlesShowDiffuseParticles);
            if (selectableLeftCheck("Diffuse Particles", showDiffuseParticles, false, selectableFlags))
            {
                showDiffuseParticles = !showDiffuseParticles;
                gSettings->set(omni::physx::kSettingDisplayParticlesShowDiffuseParticles, showDiffuseParticles);
            }

            ImGui::EndMenu();
        }

        ImGui::EndMenu();
    }

    return true;
}

// assume de-clutter mode is off at startup
static bool isDeclutterMode = false;

bool onMenuReset()
{
    // this is actually called from toggleGlobalVisibilitySettings and we need to toggle between vis and no vis
    static bool vizEnabledJointsPrev = false;
    static VisualizerMode vizModeCollidersPrev = VisualizerMode::eNone;
    static VisualizerMode vizModeTendonsPrev = VisualizerMode::eNone;
    static VisualizerMode vizModeDeformableBodiesPrev = VisualizerMode::eNone;
    static VisualizerMode vizModeDeformableSurfacesPrev = VisualizerMode::eNone;
    static VisualizerMode vizModeDeformablesPrev = VisualizerMode::eNone;
    static VisualizerMode vizModeParticlesPrev = VisualizerMode::eNone;
    static VisualizerMode vizModeAttachmentsPrev = VisualizerMode::eNone;

    bool vizEnabledJointsNew;
    VisualizerMode vizModeCollidersNew;
    VisualizerMode vizModeTendonsNew;
    VisualizerMode vizModeDeformableBodiesNew;
    VisualizerMode vizModeDeformableSurfacesNew;
    VisualizerMode vizModeDeformablesNew;
    VisualizerMode vizModeParticlesNew;
    VisualizerMode vizModeAttachmentsNew;

    if (isDeclutterMode)
    {
        // restore settings
        vizEnabledJointsNew = vizEnabledJointsPrev;
        vizModeCollidersNew = vizModeCollidersPrev;
        vizModeTendonsNew = vizModeTendonsPrev;
        vizModeDeformableBodiesNew = vizModeDeformableBodiesPrev;
        vizModeDeformableSurfacesNew = vizModeDeformableSurfacesPrev;
        vizModeDeformablesNew = vizModeDeformablesPrev;
        vizModeParticlesNew = vizModeParticlesPrev;
        vizModeAttachmentsNew = vizModeAttachmentsPrev;
    }
    else
    {
        // backup settings
        vizEnabledJointsPrev = gSettings->get<bool>(omni::physx::kSettingDisplayJoints);
        vizModeCollidersPrev = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayColliders));
        vizModeTendonsPrev = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayTendons));
        vizModeDeformableBodiesPrev = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableBodies));
        vizModeDeformableSurfacesPrev = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableSurfaces));
        vizModeDeformablesPrev = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformables));
        vizModeParticlesPrev = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayParticles));
        vizModeAttachmentsPrev = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayAttachments));

        // declutter
        vizEnabledJointsNew = false;
        vizModeCollidersNew = VisualizerMode::eNone;
        vizModeTendonsNew = VisualizerMode::eNone;
        vizModeDeformableBodiesNew = VisualizerMode::eNone;
        vizModeDeformableSurfacesNew = VisualizerMode::eNone;
        vizModeDeformablesNew = VisualizerMode::eNone;
        vizModeParticlesNew = VisualizerMode::eNone;
        vizModeAttachmentsNew = VisualizerMode::eNone;
    }

    isDeclutterMode = !isDeclutterMode;

    gSettings->set<bool>(omni::physx::kSettingDisplayJoints, vizEnabledJointsNew);
    gSettings->setInt(omni::physx::kSettingDisplayColliders, int32_t(vizModeCollidersNew));
    gSettings->setInt(omni::physx::kSettingDisplayTendons, int32_t(vizModeTendonsNew));
    gSettings->setInt(omni::physx::kSettingDisplayDeformableBodies, int32_t(vizModeDeformableBodiesNew));
    gSettings->setInt(omni::physx::kSettingDisplayDeformableSurfaces, int32_t(vizModeDeformableSurfacesNew));
    gSettings->setInt(omni::physx::kSettingDisplayDeformables, int32_t(vizModeDeformablesNew));
    gSettings->setInt(omni::physx::kSettingDisplayParticles, int32_t(vizModeParticlesNew));
    gSettings->setInt(omni::physx::kSettingDisplayAttachments, int32_t(vizModeAttachmentsNew));

    if (gDebugVisualization)
    {
        gDebugVisualization->setColliderVisualization(vizModeCollidersNew);
    }
    if (gFixedTendonVisualization)
    {
        gFixedTendonVisualization->setMode(vizModeTendonsNew);
    }
    if (gSpatialTendonManager)
    {
        gSpatialTendonManager->setMode(vizModeTendonsNew);
    }
    if (gDeformableBodyVisualizationManagerDeprecated)
    {
        gDeformableBodyVisualizationManagerDeprecated->setMode(vizModeDeformableBodiesNew);
    }
    if (gDeformableSurfaceVisualizationManagerDeprecated)
    {
        gDeformableSurfaceVisualizationManagerDeprecated->setMode(vizModeDeformableSurfacesNew);
    }
    if (gDeformableVisualizationManager)
    {
        gDeformableVisualizationManager->setMode(vizModeDeformablesNew);
    }
    if (gParticlesVisualizationManager)
    {
        gParticlesVisualizationManager->setMode(vizModeParticlesNew);
    }
    if (gAttachmentsVisualizationManagerDeprecated)
    {
        gAttachmentsVisualizationManagerDeprecated->setMode(vizModeAttachmentsNew);
    }
    return true;
}

bool isMenuReset()
{
    return !isDeclutterMode;
}

void update()
{
    if (gFixedTendonVisualization)
    {
        gFixedTendonVisualization->update();
    }
    if(gPhysXJointAuthoring)
    {
        gPhysXJointAuthoring->update();
    }
}

void stageUpdate(float, float, const omni::kit::StageUpdateSettings* updateSettings, void*)
{
    CARB_PROFILE_ZONE(0, "omni.physx.ui::stageUpdate");
    if (!gStage)
        return;

    if (gDebugVisualization)
    {
        gDebugVisualization->draw();
    }

    if (gDebugVisualizationPhysX)
    {
        gDebugVisualizationPhysX->stageUpdate();
    }

    //the order of updates for
    //gAttachmentAuthoringDeprecated, gAttachmentsVisualizationManagerDeprecated and gDeformableBodyVisualizationManager
    //gAttachmentAuthoring, gProxyVisualizationManager
    //is currently relevant, don't change!
    if (gAttachmentAuthoringDeprecated)
    {
        gAttachmentAuthoringDeprecated->update();
    }
    
    if (gAttachmentAuthoring)
    {
        gAttachmentAuthoring->update();
    }

    if (gAttachmentsVisualizationManagerDeprecated)
    {
        gAttachmentsVisualizationManagerDeprecated->update();
    }

    if (gDeformableBodyVisualizationManagerDeprecated)
    {
        gDeformableBodyVisualizationManagerDeprecated->update();
    }

    if (gDeformableSurfaceVisualizationManagerDeprecated)
    {
        gDeformableSurfaceVisualizationManagerDeprecated->update();
    }

    if (gSpatialTendonManager)
    {
        gSpatialTendonManager->update();
    }

    if (gFixedTendonVisualization)
    {
        gFixedTendonVisualization->update();
    }

    if(gPhysXJointAuthoring)
    {
        gPhysXJointAuthoring->update();
    }

    if (gParticleAuthoring)
    {
        gParticleAuthoring->update();
    }

    if (gProxyVisualizationManager)
    {
        gProxyVisualizationManager->update();
    }

    if (gInputManager)
    {
        gInputManager->update(updateSettings->isPlaying);
    }
}

void onResume(float currentTime, void*)
{
    if (gParticleAuthoring)
    {
        gParticleAuthoring->onResume();
    }

    if (gInputManager)
    {
        gInputManager->onResume();
    }
}

void onPause(void*)
{
    if (gParticleAuthoring)
    {
        gParticleAuthoring->onPause();
    }

    if (gInputManager)
    {
        gInputManager->onPause();
    }
}

void onStop(void*)
{
    if (gParticleAuthoring)
    {
        gParticleAuthoring->onStop();
    }

    if (gInputManager)
    {
        gInputManager->onStop();
    }
}

static void unsubscribeFromSettingsCallbacks(void)
{
    for (auto sub : uiSettingCallbackSubscriptions)
    {
        gSettings->unsubscribeToChangeEvents(sub);
    }
    uiSettingCallbackSubscriptions.clear();
}

static void subscribeToSettingsCallbacks(void)
{
    unsubscribeFromSettingsCallbacks();

	// DEPRECATED
    auto onDisplayDeformableBodiesChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDeformableBodyVisualizationManagerDeprecated)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(VisualizerMode::eAll) || mode < 0)
            mode = 0;
        gDeformableBodyVisualizationManagerDeprecated->setMode(VisualizerMode(mode));
    };
    
    auto onDisplayDeformableBodyTypeChange = [](const carb::dictionary::Item* changedItem,
                                                carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDeformableBodyVisualizationManagerDeprecated)
            return;
        int type = dict->getAsInt(changedItem);
        if (type > int(DeformableBodyVisualizerTypeDeprecated::eCollision) || type < 0)
            type = 0;
        gDeformableBodyVisualizationManagerDeprecated->setType(DeformableBodyVisualizerTypeDeprecated(type));
    };

    auto onDisplayDeformableSurfacesChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDeformableSurfaceVisualizationManagerDeprecated)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(VisualizerMode::eAll) || mode < 0)
            mode = 0;
        gDeformableSurfaceVisualizationManagerDeprecated->setMode(VisualizerMode(mode));
    };

    auto onDisplayAttachmentsChange = [](const carb::dictionary::Item* changedItem,
        carb::dictionary::ChangeEventType changeEventType, void* userData) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            if (!dict || !gAttachmentsVisualizationManagerDeprecated)
                return;
            int mode = dict->getAsInt(changedItem);
            if (mode > int(VisualizerMode::eAll) || mode < 0)
                mode = 0;
            gAttachmentsVisualizationManagerDeprecated->setMode(VisualizerMode(mode));
    };

    auto onDisplayAttachmentsHideActor0Change = [](const carb::dictionary::Item* changedItem,
        carb::dictionary::ChangeEventType changeEventType, void* userData) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            if (!dict || !gAttachmentsVisualizationManagerDeprecated)
                return;
            bool hideActor0 = dict->getAsBool(changedItem);
            gAttachmentsVisualizationManagerDeprecated->setHideActor0(hideActor0);
    };

    auto onDisplayAttachmentsHideActor1Change = [](const carb::dictionary::Item* changedItem,
        carb::dictionary::ChangeEventType changeEventType, void* userData) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            if (!dict || !gAttachmentsVisualizationManagerDeprecated)
                return;
            bool hideActor1 = dict->getAsBool(changedItem);
            gAttachmentsVisualizationManagerDeprecated->setHideActor1(hideActor1);
    };
	//~DEPRECATED

    auto onDisplayDeformablesChange = [](const carb::dictionary::Item* changedItem,
        carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDeformableVisualizationManager)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(VisualizerMode::eAll) || mode < 0)
            mode = 0;
        gDeformableVisualizationManager->setMode(VisualizerMode(mode));
    };
    
    auto onDisplayDeformableMeshTypeChange = [](const carb::dictionary::Item* changedItem,
                                                carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDeformableVisualizationManager)
            return;
        int type = dict->getAsInt(changedItem);
        if (type > int(DeformableVisualizerMeshType::eCollisionBind) || type < 0)
            type = 0;
        gDeformableVisualizationManager->setMeshType(DeformableVisualizerMeshType(type));
    };
    
    auto onDisplayDeformableAttachmentsChange = [](const carb::dictionary::Item* changedItem,
        carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDeformableVisualizationManager)
            return;
        bool visAttachments = dict->getAsBool(changedItem);
        gDeformableVisualizationManager->displayDeformableAttachments(visAttachments);
    };

    auto onDisplayCollidersChange = [](const carb::dictionary::Item* changedItem,
                                       carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDebugVisualization)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(VisualizerMode::eAll) || mode < 0)
            mode = 0;
        VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayColliders));
        gDebugVisualization->setColliderVisualization(VisualizerMode(mode));
    };

    auto onDisplayColliderNormalsChange = [](const carb::dictionary::Item* changedItem,
                                             carb::dictionary::ChangeEventType changeEventType, void* userData) {
        if (!gDebugVisualization)
            return;
        gDebugVisualization->updateDebugVisualization();
    };

    auto onSimplifyDebugVisDistanceChange = [](const carb::dictionary::Item* changedItem,
                                               carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDebugVisualization)
            return;
        float simplifyDebugVisAtDistance = dict->getAsFloat(changedItem);
        gDebugVisualization->setSimplifyDebugVisDistance(simplifyDebugVisAtDistance);
    };

    auto onDisplayTendonsChange = [](const carb::dictionary::Item* changedItem,
                                     carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gSpatialTendonManager || !gFixedTendonVisualization)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(VisualizerMode::eAll) || mode < 0)
            mode = 0;
        gSpatialTendonManager->setMode(VisualizerMode(mode));
        gFixedTendonVisualization->setMode(VisualizerMode(mode));
    };

    auto onDisplayParticlesChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(VisualizerMode::eAll) || mode < 0)
            mode = 0;
        gParticlesVisualizationManager->setMode(VisualizerMode(mode));
    };

    auto onDisplayParticlesParticleRadiusChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(ParticleRadiusType::eRenderGeometry) || mode < 0)
            mode = 0;
        gParticlesVisualizationManager->setParticleRadiusType(ParticleRadiusType(mode));
    };

    auto onDisplayParticlesParticlePositionsChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        int mode = dict->getAsInt(changedItem);
        if (mode > int(ParticlePositionType::eSmoothedPositions) || mode < 0)
            mode = 0;
        gParticlesVisualizationManager->setParticlePositionType(ParticlePositionType(mode));
    };

    auto onDisplayParticlesDrawClothMeshLinesChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        bool mode = dict->getAsBool(changedItem);
        gParticlesVisualizationManager->enableMeshLineVisualization(mode);
    };

    auto onDisplayParticlesShowParticleSetParticlesChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        bool mode = dict->getAsBool(changedItem);
        gParticlesVisualizationManager->showParticleSetParticles(mode);
    };

    auto onDisplayParticlesShowFluidSurfaceChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        bool mode = dict->getAsBool(changedItem);
        gParticlesVisualizationManager->showFluidSurface(mode);
    };

    auto onDisplayParticlesShowDeformableParticlesChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        bool mode = dict->getAsBool(changedItem);
        gParticlesVisualizationManager->showDeformableParticles(mode);
    };

    auto onDisplayParticlesShowDeformableMeshChange = [](const carb::dictionary::Item* changedItem,
                                              carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gParticlesVisualizationManager)
            return;
        bool mode = dict->getAsBool(changedItem);
        gParticlesVisualizationManager->showDeformableMesh(mode);
    };

    auto onDisplayParticlesShowDiffuseParticlesChange = [](const carb::dictionary::Item* changedItem,
        carb::dictionary::ChangeEventType changeEventType, void* userData) {
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            if (!dict || !gParticlesVisualizationManager)
                return;
            bool mode = dict->getAsBool(changedItem);
            gParticlesVisualizationManager->showDiffuseParticles(mode);
    };

    auto onFabricEnabledChange = [](const carb::dictionary::Item* changedItem,
                                    carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDebugVisualization)
            return;
        bool fabricEnabled = dict->getAsBool(changedItem);
        gDebugVisualization->setQueryFabricWhileSimEnabled(fabricEnabled);
    };

    auto onUsdrtEnabledChange = [](const carb::dictionary::Item* changedItem,
                                    carb::dictionary::ChangeEventType changeEventType, void* userData) {
        carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!dict || !gDebugVisualization)
            return;
        bool queryUsdrtForTraversal = dict->getAsBool(changedItem);
        gDebugVisualization->setQueryUsdrtForTraversalEnabled(queryUsdrtForTraversal);
    };

    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayColliders, onDisplayCollidersChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayColliderNormals, onDisplayColliderNormalsChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDebugVisSimplifyAtDistance, onSimplifyDebugVisDistanceChange, nullptr));
    // DEPRECATED
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayDeformableBodies, onDisplayDeformableBodiesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayDeformableBodyType, onDisplayDeformableBodyTypeChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayDeformableSurfaces, onDisplayDeformableSurfacesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayAttachments, onDisplayAttachmentsChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayAttachmentsHideActor0, onDisplayAttachmentsHideActor0Change, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayAttachmentsHideActor1, onDisplayAttachmentsHideActor1Change, nullptr));
    //~DEPRECATED
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayDeformables, onDisplayDeformablesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayDeformableMeshType, onDisplayDeformableMeshTypeChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayDeformableAttachments, onDisplayDeformableAttachmentsChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(
        gSettings->subscribeToNodeChangeEvents(omni::physx::kSettingDisplayTendons, onDisplayTendonsChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(
        gSettings->subscribeToNodeChangeEvents(omni::physx::kSettingDisplayParticles, onDisplayParticlesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesShowDiffuseParticles, onDisplayParticlesShowDiffuseParticlesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesParticlePositions, onDisplayParticlesParticlePositionsChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesParticleRadius, onDisplayParticlesParticleRadiusChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesClothMeshLines, onDisplayParticlesDrawClothMeshLinesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesShowParticleSetParticles, onDisplayParticlesShowParticleSetParticlesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesShowFluidSurface, onDisplayParticlesShowFluidSurfaceChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesShowDeformableParticles, onDisplayParticlesShowDeformableParticlesChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingDisplayParticlesShowDeformableMesh, onDisplayParticlesShowDeformableMeshChange, nullptr));

    uiSettingCallbackSubscriptions.push_back(
        gSettings->subscribeToNodeChangeEvents(omni::physx::kSettingFabricEnabled, onFabricEnabledChange, nullptr));
    uiSettingCallbackSubscriptions.push_back(
        gSettings->subscribeToNodeChangeEvents(omni::physx::kSettingDebugVisQueryUsdrtForTraversal, onUsdrtEnabledChange, nullptr));
}

void stageOpened()
{
    CARB_PROFILE_ZONE(0, "PhysXUi stageOpened");

    gStage = omni::usd::UsdContext::getContext()->getStage();

    if (gDebugVisualization)
    {
        gDebugVisualization->setSceneLoaded(true);
        gDebugVisualization->setSceneDirty();
    }
    if (gSpatialTendonManager)
        gSpatialTendonManager->parseStage();
    if (gFixedTendonVisualization)
        gFixedTendonVisualization->parseStage();
    if (gParticleAuthoring)
    {
        const bool enable = (gSettings ? gSettings->getAsBool(omni::physx::kSettingEnableParticleAuthoring) : true);
        gParticleAuthoring->setEnabled(enable);
        gParticleAuthoring->parseStage();
    }
    if (gProxyVisualizationManager)
        gProxyVisualizationManager->parseStage();
    if (gAttachmentAuthoringDeprecated)
    {
        const bool enable = (gSettings ? gSettings->getAsBool(omni::physx::kSettingEnableAttachmentAuthoring) : true);
        gAttachmentAuthoringDeprecated->setEnabled(enable);
        gAttachmentAuthoringDeprecated->parseStage();
    }
    if (gAttachmentsVisualizationManagerDeprecated)
        gAttachmentsVisualizationManagerDeprecated->parseStage();
    if (gDeformableBodyVisualizationManagerDeprecated)
        gDeformableBodyVisualizationManagerDeprecated->parseStage();
    if (gDeformableSurfaceVisualizationManagerDeprecated)
        gDeformableSurfaceVisualizationManagerDeprecated->parseStage();
    if (gAttachmentAuthoring)
    {
        const bool enable = (gSettings ? gSettings->getAsBool(omni::physx::kSettingEnableAttachmentAuthoring) : true);
        gAttachmentAuthoring->setEnabled(enable);
        gAttachmentAuthoring->parseStage();
    }
}

void stageClosed()
{
    if (gDebugVisualization)
        gDebugVisualization->setSceneLoaded(false);
    if (gSpatialTendonManager)
        gSpatialTendonManager->release();
    if (gFixedTendonVisualization)
        gFixedTendonVisualization->release();
    if (gParticleAuthoring)
        gParticleAuthoring->release();
    if (gProxyVisualizationManager)
        gProxyVisualizationManager->release();
    if (gAttachmentAuthoringDeprecated)
        gAttachmentAuthoringDeprecated->release();
    if (gAttachmentsVisualizationManagerDeprecated)
        gAttachmentsVisualizationManagerDeprecated->release();
    if (gDeformableBodyVisualizationManagerDeprecated)
        gDeformableBodyVisualizationManagerDeprecated->release();
    if (gDeformableSurfaceVisualizationManagerDeprecated)
        gDeformableSurfaceVisualizationManagerDeprecated->release();
    if (gAttachmentAuthoring)
        gAttachmentAuthoring->release();
    gStage = nullptr;
}

void tryRefreshStage()
{
    gStage = omni::usd::UsdContext::getContext()->getStage();
    if (gStage)
    {
        stageOpened();
    }
}

void blockUsdNoticeHandler(bool enable)
{
    gBlockNoticeHandle = enable;
}

bool isUsdNoticeHandlerEnabled()
{
    return !gBlockNoticeHandle;
}


CARB_EXPORT void carbOnPluginStartup()
{
    carb::Framework* framework = carb::getFramework();
    auto viewport = framework->tryAcquireInterface<omni::kit::IViewport>();
    if(viewport)
        gViewportWindow = viewport->getViewportWindow(nullptr);
    else
        gViewportWindow = nullptr;
    gInput = carb::getCachedInterface<carb::input::IInput>();
    gPhysX = carb::getCachedInterface<omni::physx::IPhysx>();
    gPhysXCooking = carb::getCachedInterface<omni::physx::IPhysxCooking>();
    gPhysXCookingPrivate = carb::getCachedInterface<omni::physx::IPhysxCookingPrivate>();
    gPhysXAttachmentPrivate = carb::getCachedInterface<omni::physx::IPhysxAttachmentPrivate>();
    gPhysXParticlesPrivate = carb::getCachedInterface<omni::physx::IPhysxParticlesPrivate>();
    gUsdLoad = carb::getCachedInterface<omni::physx::usdparser::IPhysxUsdLoad>();
    gTimeline = omni::timeline::getTimeline();
    gTimelineEvtSub = carb::events::createSubscriptionToPop(
        gTimeline->getTimelineEventStream(),
        [](carb::events::IEvent* e) {
        if (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::ePlay)
        {
            if (gParticleAuthoring)
                gParticleAuthoring->setIsPlaying(true);

            if (gDebugVisualization)
            {
                gDebugVisualization->setIsPlaying(true);
            }
        }
        else if (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::eStop)
        {
            if (gParticleAuthoring)
                gParticleAuthoring->setIsPlaying(false);

            if (gDebugVisualization)
            {
                gDebugVisualization->setIsPlaying(false);
            }
        }
        },
        0, "PhysX::UI stage update");

    tryRefreshStage();

    auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
    omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
    if (usdContext)
    {
        gStageEvtSub = {
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eOpened),
                             [](const auto&) { stageOpened(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eClosing),
                             [](const auto&) { stageClosed(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eSelectionChanged),
                             [](const auto&) {
                                 if (!gStage)
                                     stageOpened();

                                 if (gDebugVisualization)
                                 {
                                     gDebugVisualization->selectionChanged();
                                 }
                                 if (gSpatialTendonManager)
                                 {
                                     gSpatialTendonManager->selectionChanged();
                                 }
                                 if (gFixedTendonVisualization)
                                 {
                                     gFixedTendonVisualization->selectionChanged();
                                 }
                                 if (gDeformableBodyVisualizationManagerDeprecated)
                                 {
                                     gDeformableBodyVisualizationManagerDeprecated->selectionChanged();
                                 }
                                 if (gDeformableSurfaceVisualizationManagerDeprecated)
                                 {
                                     gDeformableSurfaceVisualizationManagerDeprecated->selectionChanged();
                                 }
                                 if (gAttachmentsVisualizationManagerDeprecated)
                                 {
                                     gAttachmentsVisualizationManagerDeprecated->selectionChanged();
                                 }
                                 if (gProxyVisualizationManager)
                                 {
                                     gProxyVisualizationManager->selectionChanged();
                                 }
                             }),
        };
    }

    omni::kit::StageUpdatePtr stageUpdatePtr =  omni::kit::getStageUpdate();

    omni::kit::StageUpdateNodeDesc desc = { 0 };
    desc.displayName = "PhysXUI";
    desc.order = omni::kit::update::eIUsdStageUpdatePhysicsUI;
    desc.onPrimAdd = nullptr;
    desc.onPrimOrPropertyChange = nullptr;
    desc.onPrimRemove = nullptr;
    desc.onUpdate = stageUpdate;
    desc.onStop = onStop;
    desc.onResume = onResume;
    desc.onPause = onPause;
    if (stageUpdatePtr)
        gStageUpdateNode = stageUpdatePtr->createStageUpdateNode(desc);

    if (viewport && gViewportWindow)
    {
        gViewportUiEvtSub = carb::events::createSubscriptionToPop(
            gViewportWindow->getUiDrawEventStream().get(),
            [](carb::events::IEvent* e) { onUIDraw(e); }, 0, "PhysX::UI viewport ui update");
    }

    gSettings = carb::getCachedInterface<carb::settings::ISettings>();
    gDebugVisualization = new DebugVisualization();
    gDebugVisualizationPhysX = new DebugVisualizationPhysX();
    gInputManager = new InputManager();

    const float simplifyDebugVisAtDistance = gSettings->getAsFloat(omni::physx::kSettingDebugVisSimplifyAtDistance);
    gDebugVisualization->setSimplifyDebugVisDistance(simplifyDebugVisAtDistance);

    gDebugVisualization->setQueryUsdrtForTraversalEnabled(gSettings->getAsBool(omni::physx::kSettingDebugVisQueryUsdrtForTraversal));
    gDebugVisualization->setQueryFabricWhileSimEnabled(gSettings->getAsBool(omni::physx::kSettingFabricEnabled));

    const VisualizerMode tendonsMode =
        (gSettings ? VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayTendons)) :
                     VisualizerMode::eSelected);
	// DEPRECATED
    const VisualizerMode deformableBodiesMode =
        (gSettings ? VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableBodies)) :
                     VisualizerMode::eSelected);
    const DeformableBodyVisualizerTypeDeprecated deformableBodyType =
        (gSettings ? DeformableBodyVisualizerTypeDeprecated(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableBodyType)) :
                     DeformableBodyVisualizerTypeDeprecated::eSimulation);
    const VisualizerMode deformableSurfaceMode =
        (gSettings ? VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableSurfaces)) :
                     VisualizerMode::eNone);
    //~DEPRECATED

   const VisualizerMode deformablesMode =
        (gSettings ? VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayDeformables)) :
            VisualizerMode::eSelected);
    const DeformableVisualizerMeshType deformableMeshType =
        (gSettings ? DeformableVisualizerMeshType(gSettings->getAsInt(omni::physx::kSettingDisplayDeformableMeshType)) :
                     DeformableVisualizerMeshType::eSimulationDefault);
    const bool displayDeformableAttachments =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayDeformableAttachments) : false);
    const VisualizerMode particlesMode =
        (gSettings ? VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayParticles)) :
                     VisualizerMode::eSelected);

    // DEPRECATED
    const VisualizerMode attachmentsMode =
        (gSettings ? VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayAttachments)) :
            VisualizerMode::eSelected);
    const bool attachmentsHideActor0 = (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayAttachmentsHideActor0) : false);
    const bool attachmentsHideActor1 = (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayAttachmentsHideActor1) : false);
    //~DEPRECATED
    
    const ParticlePositionType particlesPosition =
        (gSettings ? ParticlePositionType(gSettings->getAsInt(omni::physx::kSettingDisplayParticlesParticlePositions)) :
            ParticlePositionType::eSimPositions);
    const ParticleRadiusType particlesRadius =
        (gSettings ? ParticleRadiusType(gSettings->getAsInt(omni::physx::kSettingDisplayParticlesParticleRadius)) :
            ParticleRadiusType::eParticleContactOffset);
            
    // DEPRECATED
    const bool particlesClothMeshLines =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayParticlesClothMeshLines) : false);
    //~DEPRECATED

    const bool showParticleSetParticles =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayParticlesShowParticleSetParticles) : true);
    const bool showFluidSurface =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayParticlesShowFluidSurface) : false);
    // DEPRECATED
    const bool showDeformableParticles =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayParticlesShowDeformableParticles) : true);
    const bool showDeformableMesh =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayParticlesShowDeformableMesh) : false);
    //~DEPRECATED
    const bool showDiffuseParticles =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingDisplayParticlesShowDiffuseParticles) : false);
    const bool enableParticleAuthoring =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingEnableParticleAuthoring) : true);
    const bool enableAttachmentAuthoring =
        (gSettings ? gSettings->getAsBool(omni::physx::kSettingEnableAttachmentAuthoring) : true);

    gSpatialTendonManager = new SpatialTendonManager(tendonsMode);
    gFixedTendonVisualization = new FixedTendonVisualizer(tendonsMode);
    gAttachmentAuthoringDeprecated = new AttachmentAuthoringDeprecated(enableAttachmentAuthoring);
    gAttachmentsVisualizationManagerDeprecated = new AttachmentsVisualizationManagerDeprecated(attachmentsMode, attachmentsHideActor0, attachmentsHideActor1);
    gDeformableBodyVisualizationManagerDeprecated = new DeformableBodyVisualizationManagerDeprecated(deformableBodiesMode, deformableBodyType);
    gDeformableSurfaceVisualizationManagerDeprecated = new DeformableSurfaceVisualizationManagerDeprecated(deformableSurfaceMode);
    gAttachmentAuthoring = new AttachmentAuthoring(enableAttachmentAuthoring);
    
    gAttachmentsVisualizationManagerDeprecated->setDeformableBodyVisualizationManager(*gDeformableBodyVisualizationManagerDeprecated);
    gDeformableBodyVisualizationManagerDeprecated->setAttachmentsVisualizationManager(*gAttachmentsVisualizationManagerDeprecated);

    gProxyVisualizationManager = new ProxyVisualizationManager();

    gParticlesVisualizationManager = new ParticlesVisualizationManager(*gProxyVisualizationManager, particlesMode,
        particlesPosition, particlesRadius, particlesClothMeshLines, showParticleSetParticles, showFluidSurface,
        showDeformableParticles, showDeformableMesh, showDiffuseParticles);
    
    gDeformableVisualizationManager = new DeformableBodyVisualizationManager(*gProxyVisualizationManager, deformablesMode, deformableMeshType, displayDeformableAttachments);

    gProxyVisualizationManager->addClient(*gDeformableVisualizationManager);
    gProxyVisualizationManager->addClient(*gParticlesVisualizationManager);

    gParticleAuthoring = new ParticleAuthoring(enableParticleAuthoring);


    subscribeToSettingsCallbacks();

    gUsdNoticeListener = new omni::physx::ui::UsdNoticeListener();
    gUsdNoticeListenerKey =
        pxr::TfNotice::Register(pxr::TfCreateWeakPtr(gUsdNoticeListener), &omni::physx::ui::UsdNoticeListener::handle);

    {
        if(gViewportWindow)
        {
            CARB_LOG_WARN("Compatibility mode: Viewport 1.0 is now deprecated and will be removed in a future release");
            gViewportWindow->addShowByTypeSubmenu("ShowPhysics", &onMenuDraw, &onMenuReset, &isMenuReset);
        }
        if (gDebugVisualization)
        {
            VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayColliders));
            gDebugVisualization->setColliderVisualization(vizMode);
        }
        if (gFixedTendonVisualization)
        {
            VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayTendons));
            gFixedTendonVisualization->setMode(vizMode);
        }
        if (gSpatialTendonManager)
        {
            VisualizerMode vizMode = VisualizerMode(gSettings->getAsInt(omni::physx::kSettingDisplayTendons));
            gSpatialTendonManager->setMode(vizMode);
        }
    }

    //create a shared selection group for custom outlines for session layer proxy geometry
    gProxySelectionGroup = usdContext->registerSelectionGroup();
    usdContext->setSelectionGroupOutlineColor(gProxySelectionGroup, {1.0f, 0.6f, 0.0f, 1.0f});
    usdContext->setSelectionGroupShadeColor(gProxySelectionGroup, {1.0f, 1.0f, 1.0f, 0.0f});

    // turn on physx stats
    omni::physx::IPhysxStatistics* physxStatistics = carb::getCachedInterface<omni::physx::IPhysxStatistics>();
    if (!physxStatistics)
    {
        CARB_LOG_ERROR("Acquiring omni::physx::IPhysxStatistics interface failed.");
    }
    physxStatistics->enableCarbStatsUpload(true);
}

CARB_EXPORT void carbOnPluginShutdown()
{
    if(gStage != nullptr)
        stageClosed();

    if (gViewportUiEvtSub)
    {
        gViewportUiEvtSub->unsubscribe();
        gViewportUiEvtSub = nullptr;
    }

    gStageEvtSub = {};

    if (gTimelineEvtSub)
    {
        gTimelineEvtSub->unsubscribe();
        gTimelineEvtSub = nullptr;
    }

    unsubscribeFromSettingsCallbacks();

    omni::kit::StageUpdatePtr stageUpdatePtr =  omni::kit::getStageUpdate();
    if (gStageUpdateNode && stageUpdatePtr)
    {
        stageUpdatePtr->destroyStageUpdateNode(gStageUpdateNode);
        gStageUpdateNode = nullptr;
    }

    if (gDebugVisualization)
    {
        delete gDebugVisualization;
        gDebugVisualization = nullptr;
    }

    if (gDebugVisualizationPhysX)
    {
        delete gDebugVisualizationPhysX;
        gDebugVisualizationPhysX = nullptr;
    }

    if (gSpatialTendonManager)
    {
        delete gSpatialTendonManager;
        gSpatialTendonManager = nullptr;
    }

    if (gFixedTendonVisualization)
    {
        delete gFixedTendonVisualization;
        gFixedTendonVisualization = nullptr;
    }

    if (gParticleAuthoring)
    {
        delete gParticleAuthoring;
        gParticleAuthoring = nullptr;
    }

    if (gProxyVisualizationManager)
    {
        delete gProxyVisualizationManager;
        gProxyVisualizationManager = nullptr;
    }
    gParticlesVisualizationManager = nullptr;
    gDeformableVisualizationManager = nullptr;

    if (gAttachmentAuthoringDeprecated)
    {
        delete gAttachmentAuthoringDeprecated;
        gAttachmentAuthoringDeprecated = nullptr;
    }
    
    if (gAttachmentAuthoring)
    {
        delete gAttachmentAuthoring;
        gAttachmentAuthoring = nullptr;
    }

    if (gDeformableBodyVisualizationManagerDeprecated)
    {
        delete gDeformableBodyVisualizationManagerDeprecated;
        gDeformableBodyVisualizationManagerDeprecated = nullptr;
    }

    if (gDeformableSurfaceVisualizationManagerDeprecated)
    {
        delete gDeformableSurfaceVisualizationManagerDeprecated;
        gDeformableSurfaceVisualizationManagerDeprecated = nullptr;
    }

    if (gAttachmentsVisualizationManagerDeprecated)
    {
        delete gAttachmentsVisualizationManagerDeprecated;
        gAttachmentsVisualizationManagerDeprecated = nullptr;
    }

    if (gInputManager)
    {
        delete gInputManager;
        gInputManager = nullptr;
    }

    gPhysX = nullptr;
    gPhysXCooking = nullptr;
    gPhysXAttachmentPrivate = nullptr;
    gPhysXParticlesPrivate = nullptr;
    gUsdLoad = nullptr;
    gStage = nullptr;

    gViewportWindow = nullptr;

    pxr::TfNotice::Revoke(gUsdNoticeListenerKey);
    delete gUsdNoticeListener;
    gUsdNoticeListener = nullptr;
}

void fillInterface(IPhysxUIPrivate& iface);

void fillInterface(IPhysxUI& iface)
{
    iface.setVisualizationDistance = setVisualizationDistance;
    iface.enableDebugVisualization = enableDebugVisualization;
    iface.selectSpatialTendonAttachmentHelper = selectSpatialTendonAttachmentHelper;
    iface.setTendonVisualizationFilter = setTendonVisualizationFilter;
    iface.setVehicleVisualization = setVehicleVisualization;
    iface.getVehicleVisualization = getVehicleVisualization;
    iface.setCollisionMeshType = setCollsionMeshType;
    iface.enableCollisionMeshVisualization = enableCollisionMeshVisualization;
    iface.explodeViewDistance = explodeViewDistance;
    iface.update = update;
    iface.getAttachments = getAttachmentsDeprecated;
    iface.blockUsdNoticeHandler = blockUsdNoticeHandler;
    iface.isUsdNoticeHandlerEnabled = isUsdNoticeHandlerEnabled;
    iface.refreshAttachment = refreshAttachment;
    iface.setCameraPos = setCameraPos;
    iface.enableRedrawOptimizations = enableRedrawOptimizations;
}

void fillInterface(ExtensionImpl& iface)
{
}

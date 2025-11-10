// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physics
{
namespace ui
{

struct IUsdPhysicsUICustomJointAuthoring
{
    using RegistrationID = uint64_t;
    /// Invalid Registration ID that may be returned by @ref IUsdPhysicsUI::registerCustomJointAuthoring
    static const RegistrationID InvalidRegistrationID = 0;

    /// Return Joint custom billboard function
    /// Allows returning a custom billboard for joint, previously registered through @ref
    /// IUsdPhysicsUI::registerCustomJointBillboard
    ///
    /// \param[in]  stageID         USD stageId
    /// \param[in]  jointUSDPath    USD path to joint
    /// \param[out] billboardID     Handle to the billboard created with @ref IUsdPhysicsUI::registerCustomJointBillboard
	/// \param[in]  userData        User data passed during registration as @ref IUsdPhysicsUICustomJointAuthoring::userData
    typedef bool (*OnJointGetCustomBillboard)(uint64_t stageID,
                                              uint64_t jointUSDPath,
                                              uint32_t& billboardID,
                                              void* userData);
    /// Create Joint function
    /// Callback called when Joint Authoring Object is created for given path
    ///
    /// \param[in]  stageID         USD stageId
    /// \param[in]  jointUSDPath    USD path to joint
    /// \param[in]  userData        User data passed during registration as @ref IUsdPhysicsUICustomJointAuthoring::userData
    typedef void (*OnJointCreate)(uint64_t stageID, uint64_t jointUSDPath, void* userData);


    /// Create Joint function
    /// Callback called when Joint Authoring Object is deleted for given path
    ///
    /// \param[in]  stageID         USD stageId
    /// \param[in]  jointUSDPath    USD path to joint
    /// \param[in]  userData        User data passed during registration as @ref IUsdPhysicsUICustomJointAuthoring::userData
    typedef void (*OnJointDelete)(uint64_t stageID, uint64_t jointUSDPath, void* userData);
    struct JointScaleData
    {
        carb::Float3 scaleRatio;
    };

    /// Scale Joint function
    /// Callback called when a scaling operation is applied to custom joint. Caller will need to apply scaling
    /// to usd in order for it to persist
    ///
    /// \param[in]  stageID         USD stageId
    /// \param[in]  jointUSDPath    USD path to joint
    /// \param[in]  scaleData       Scaling ratio to be applied to custom joint
    /// \param[in]  userData        User data passed during registration as @ref IUsdPhysicsUICustomJointAuthoring::userData
    typedef bool (*OnJointScale)(uint64_t stageID, uint64_t jointUSDPath, const JointScaleData& scaleData, void* userData);

    void* userData = { nullptr }; //!< Custom user data pointer passed back to all callbacks

    OnJointCreate onJointAuthoringCreate = { nullptr }; //!<  When Joint Authoring Object is created for given path
    OnJointDelete onJointAuthoringDelete = { nullptr }; //!<  When Joint Authoring Object is deleted for given path
    OnJointScale onJointAuthoringScale = { nullptr }; //!<  When Joint Authoring Object is scaled
    OnJointGetCustomBillboard onJointGetCustomBillboard = { nullptr }; //!<  When Joint Authoring needs billboards
};

struct VisibilityFlags
{
    enum Enum
    {
        eHIDE_ALL = 0, //!< Disables all drawing inside IUsdPhysicsUI
        eSHOW_ALL = 0xffffffff, //!< Enables all drawing inside IUsdPhysicsUI
        eSHOW_JOINTS = 1 << 0, //!< Enables drawing only joints
    };
};

struct IUsdPhysicsUI
{
    CARB_PLUGIN_INTERFACE("omni::physics::ui::IUsdPhysicsUI", 0, 1)
    /// Register a new custom joint authoring set of callbacks
    ///
    /// \param customJointAuthoring IUsdPhysicsUICustomJointAuthoring object with callbacks for each event
    /// \param name                 A user supplied string used to refer to the registered client in log messages.
    ///
    /// \returns A JointAuthoringRegistrationID (uint64_t) to be used to de-register the callbacks OR @ref
    /// IUsdPhysicsUICustomJointAuthoring::InvalidRegistrationID
    IUsdPhysicsUICustomJointAuthoring::RegistrationID(CARB_ABI* registerCustomJointAuthoring)(
        IUsdPhysicsUICustomJointAuthoring& customJointAuthoring, const char* name);

    /// Unregister a custom joint authoring set of callbacks
    /// It will automatically unregister all custom joint billboards registered with registerCustomJointBillboard
    ///
    /// \param registrationID         RegistrationID previously returned by call to registerCustomJointAuthoring
    ///
    /// \returns true if the object was successfully unregistered
    bool(CARB_ABI* unregisterCustomJointAuthoring)(IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID);

    /// Registers a customJointBillboard
    ///
    /// \param registrationID  RegistrationID previously returned by call to registerCustomJointAuthoring
    /// \param pngPath         c-string with path to a png that contains png asset for the billboard
    ///
    /// \returns the joint billboard handle that can be used later inside the onJointGetCustomBillboard callback
    uint32_t(CARB_ABI* registerCustomJointBillboard)(IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID,
                                                     const char* pngPath);

    /// Set visibility to enable/disable drawing specific helpers.
    /// For example caller can hide all draw helpers when simulation is running.
    ///
    /// \param visibilityFlags         Flags bitmask as from VisibilityFlags::Enum
    ///
    void(CARB_ABI* setVisibilityFlags)(VisibilityFlags::Enum visibilityFlags);

    /// Gets current visibility of drawing helpers.
    ///
    /// \returns Combination of flags from VisibilityFlags::Enum
    ///
    VisibilityFlags::Enum(CARB_ABI* getVisibilityFlags)();

    /// Block USD notice handler.
    /// \param enable Enabled or disable the notice handler.
    void(CARB_ABI* blockUsdNoticeHandler)(bool enable);

    /// Check if USD notice handler is blocked.
    /// \return If the notice handler is enabled or not.
    bool(CARB_ABI* isUsdNoticeHandlerEnabled)();
};

} // namespace ui
} // namespace physics
} // namespace omni

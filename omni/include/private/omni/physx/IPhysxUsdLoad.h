// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

typedef size_t VehicleComponentTrackerHandle;


struct IPhysxUsdLoad
{
    CARB_PLUGIN_INTERFACE("omni::physx::usdparser::IPhysxUsdLoad", 1, 0)

    /// Parse joint desc from a given Usd path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Joint usd path
    /// return Joint descriptor, note that memory is owned by omni.physx
    usdparser::PhysxJointDesc*(CARB_ABI* parseJoint)(uint64_t stageId, const pxr::SdfPath& path);

    /// Parse collision desc from a given Usd path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Collision usd path
    /// return Collision descriptor, note that memory is owned by omni.physx
    usdparser::PhysxShapeDesc*(CARB_ABI* parseCollision)(uint64_t stageId,
                                                         const pxr::SdfPath& path,
                                                         const pxr::SdfPath& gprimPath);

    /// Parse spatial tendon attachment descs from a given Usd parent path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Parent path of subtree with spatial tendon components
    /// return Collection of tendon attachment descriptors, note that memory is owned by omni.physx
    std::vector<usdparser::PhysxTendonAttachmentHierarchyDesc*>(CARB_ABI* parseSpatialTendons)(
        uint64_t stageId, const pxr::SdfPath& parentPath);

    /// Parse fixed tendon axes descs from a given Usd parent path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Parent path of subtree with fixed tendon components
    /// return Collection of tendon axis descriptors, note that memory is owned by omni.physx
    std::vector<usdparser::PhysxTendonAxisHierarchyDesc*>(CARB_ABI* parseFixedTendons)(uint64_t stageId,
                                                                                       const pxr::SdfPath& parentPath);

    /// Release desc memory
    ///
    /// \param[in] objectDesc Desc memory to release
    void(CARB_ABI* releaseDesc)(usdparser::PhysxObjectDesc* objectDesc);

    /// Parse softbody desc from a given Usd path
    ///
    /// DEPRECATED: Will be replaced by new deformable implementation in future release.
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path softbody usd path
    /// return softbody descriptor, note that memory is owned by omni.physx
    usdparser::SoftBodyDesc*(CARB_ABI* parseDeformableBody)(uint64_t stageId, const pxr::SdfPath& path);

    /// Parse FEM cloth desc from a given Usd path
    ///
    /// DEPRECATED: Will be replaced by new deformable implementation in future release.
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path FEM cloth usd path
    /// return FEM cloth descriptor, note that memory is owned by omni.physx
    usdparser::FEMClothDesc*(CARB_ABI* parseDeformableSurface)(uint64_t stageId, const pxr::SdfPath& path);

    /// Parse cloth desc from a given Usd path
    ///
    /// DEPRECATED: Will be replaced by new deformable implementation in future release.
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path cloth usd path
    /// return cloth descriptor, note that memory is owned by omni.physx
    usdparser::ParticleClothDesc*(CARB_ABI* parseParticleCloth)(uint64_t stageId, const pxr::SdfPath& path);

    /// Create a vehicle component tracker instance.
    ///
    /// A vehicle component tracker is needed to parse a vehicle Usd prim.
    /// It avoids that shared components are parsed multiple times when parsing
    /// multiple vehicles.
    /// The vehicle component tracker must not be released as long as dependent
    /// vehicle descriptors need to be available for reading.
    ///
    /// \return handle of vehicle component tracker
    ///
    VehicleComponentTrackerHandle(CARB_ABI* createVehicleComponentTracker)();

    /// Release a vehicle component tracker instance.
    ///
    /// \param[in] handle the handle of the vehicle component tracker to release
    ///
    void(CARB_ABI* releaseVehicleComponentTracker)(VehicleComponentTrackerHandle handle);

    /// Parse vehicle descriptor from a given Usd path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path vehicle Usd path
    /// \param[in] handle the handle of the vehicle component tracker to use
    /// \return vehicle descriptor, note that memory is owned by omni.physx
    ///
    usdparser::VehicleDesc*(CARB_ABI* parseVehicle)(uint64_t stageId,
                                                    const pxr::SdfPath& path,
                                                    VehicleComponentTrackerHandle handle);

    /// parse a particle system from a given Usd path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path particle system Usd path
    /// \return particle system descriptor, note that memory is owned by omni.physx
    usdparser::ParticleSystemDesc*(CARB_ABI* parseParticleSystem)(uint64_t stageId, const pxr::SdfPath& path);

    /// Parse articulations descs from a given Usd parent path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Parent path of subtree with articulatino roots
    /// return Collection of articulation desc, note that memory is owned by omni.physx
    std::vector<usdparser::PhysxArticulationDesc*>(CARB_ABI* parseArticulations)(uint64_t stageId,
                                                                                 const pxr::SdfPath& parentPath);
};

} // namespace usdparser
} // namespace physx
} // namespace omni

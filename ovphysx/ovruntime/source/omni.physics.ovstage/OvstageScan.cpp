// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CCT-001
 * @covers AC-5
 *
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-11 AC-12
 */

#include <omni/physics/ovstage/OvstageScan.h>
#include <omni/physics/ovstage/OvstageParseBackend.h> // OvstageAttach

#include "OvstageSource.h"
#include "OvstageWalker.h"

#include <carb/logging/Log.h>

#include <memory>

namespace omni::physics::ovstage
{

omni::physics::parse::ScannedStage scanStageOvstage(ovstage_instance_t* instance,
                                                    ovx_path_dictionary_t* dict,
                                                    parse::IDescriptorAllocator& allocator,
                                                    ovstage_ordinal_t readOrdinal,
                                                    const OvstageScanFilter* filter,
                                                    uint64_t usdStageId,
                                                    OvstageSource* attached)
{
    if (!dict && instance)
        dict = ovstage_get_path_dictionary(instance);
    if (!instance || !dict)
        return {};

    OvstageScanResult scan = scanOvstage(instance, dict, allocator, readOrdinal, filter, usdStageId, attached);

    // TokenIds are source-local, so retain the source that minted the descriptor handles: the
    // attach's own source when the scan borrowed it, else the fresh one the scan built.
    parse::ScannedStage out = scan.borrowedSource ? parse::makeScannedStageBorrowingSource(*scan.borrowedSource) :
                                                    parse::makeScannedStageFromSource(std::move(scan.source));
    out.scenes = std::move(scan.scenes);
    out.materials = std::move(scan.materials);
    out.pbdMaterials = std::move(scan.pbdMaterials);
    out.deformableMaterials = std::move(scan.deformableMaterials);
    out.bodies = std::move(scan.bodies);
    out.shapes = std::move(scan.shapes);
    // Bounding-sphere/cube shapes carry a mergedMesh pointer into these owned
    // MergeMeshDescs; hand ownership to the ScannedStage so they outlive the shapes.
    for (parse::DescPtr<parse::MergeMeshDesc>& mm : scan.ownedMeshes)
        out.retainOwnedMesh(std::move(mm));
    out.joints = std::move(scan.joints);
    out.mimicJoints = std::move(scan.mimicJoints);
    out.collisionGroups = std::move(scan.collisionGroups);
    out.articulations = std::move(scan.articulations);
    out.deformables = std::move(scan.deformables);
    out.attachments = std::move(scan.attachments);
    out.deformableCollisionFilters = std::move(scan.deformableCollisionFilters);
    out.spatialTendonAttachments = std::move(scan.spatialTendonAttachments);
    out.fixedTendonAxes = std::move(scan.fixedTendonAxes);
    out.fixedTendons = std::move(scan.fixedTendons);
    out.tireFrictionTables = std::move(scan.tireFrictionTables);
    out.vehicleContexts = std::move(scan.vehicleContexts);
    out.vehicleWheels = std::move(scan.vehicleWheels);
    out.vehicleTires = std::move(scan.vehicleTires);
    out.vehicleSuspensions = std::move(scan.vehicleSuspensions);
    out.vehicleEngines = std::move(scan.vehicleEngines);
    out.vehicleGears = std::move(scan.vehicleGears);
    out.vehicleClutches = std::move(scan.vehicleClutches);
    out.vehicleGearsPaths = std::move(scan.vehicleGearsPaths);
    out.vehicleClutchPaths = std::move(scan.vehicleClutchPaths);
    out.vehicleDrivesBasic = std::move(scan.vehicleDrivesBasic);
    out.vehicleMultiWheelDifferentials = std::move(scan.vehicleMultiWheelDifferentials);
    out.vehicleTankDifferentials = std::move(scan.vehicleTankDifferentials);
    out.vehicleAutoGearBoxes = std::move(scan.vehicleAutoGearBoxes);
    out.vehicleMultiWheelDifferentialPaths = std::move(scan.vehicleMultiWheelDifferentialPaths);
    out.vehicleTankDifferentialPaths = std::move(scan.vehicleTankDifferentialPaths);
    out.vehicleAutoGearBoxPaths = std::move(scan.vehicleAutoGearBoxPaths);
    out.vehicleBrakes = std::move(scan.vehicleBrakes);
    out.vehicleBrakesPaths = std::move(scan.vehicleBrakesPaths);
    out.vehicleBrakesInstanceTokens = std::move(scan.vehicleBrakesInstanceTokens);
    out.vehicleSteeringBasic = std::move(scan.vehicleSteeringBasic);
    out.vehicleSteeringAckermann = std::move(scan.vehicleSteeringAckermann);
    out.vehicleSteeringBasicPaths = std::move(scan.vehicleSteeringBasicPaths);
    out.vehicleSteeringAckermannPaths = std::move(scan.vehicleSteeringAckermannPaths);
    out.vehicleNonlinearCmdResponses = std::move(scan.vehicleNonlinearCmdResponses);
    out.vehicleNonlinearCmdResponsePaths = std::move(scan.vehicleNonlinearCmdResponsePaths);
    out.vehicleNonlinearCmdResponseInstanceTokens = std::move(scan.vehicleNonlinearCmdResponseInstanceTokens);
    out.vehicleDrivesStandard = std::move(scan.vehicleDrivesStandard);
    out.vehicleDrivesStandardPaths = std::move(scan.vehicleDrivesStandardPaths);
    out.vehicleDrivesStandardCrossRefs = std::move(scan.vehicleDrivesStandardCrossRefs);
    out.vehicleWheelAttachments = std::move(scan.vehicleWheelAttachments);
    out.vehicleWheelAttachmentInfos = std::move(scan.vehicleWheelAttachmentInfos);
    out.vehicleWheelAttachmentOwners = std::move(scan.vehicleWheelAttachmentOwners);
    out.vehicles = std::move(scan.vehicles);
    out.vehiclePaths = std::move(scan.vehiclePaths);
    out.vehicleSuspensionCompliances = std::move(scan.vehicleSuspensionCompliances);
    out.vehicleSuspensionCompliancePaths = std::move(scan.vehicleSuspensionCompliancePaths);
    out.particleSystems = std::move(scan.particleSystems);
    out.particleAnisotropies = std::move(scan.particleAnisotropies);
    out.particleSmoothings = std::move(scan.particleSmoothings);
    out.particleIsosurfaces = std::move(scan.particleIsosurfaces);
    out.particleSets = std::move(scan.particleSets);
    out.particleSamplers = std::move(scan.particleSamplers);
    out.particleSamplerKeys = std::move(scan.particleSamplerKeys);
    out.ccts = std::move(scan.ccts);
    out.hasPointInstancerPrims = scan.hasPointInstancerPrims;
    return out;
}

namespace
{
// IScanBackend over ovstage. nativeStage is the same const OvstageAttach* the
// ovstage parse backend consumes, so a consumer sets one payload and registers
// both backends.
class OvstageScanBackend final : public omni::physics::parse::IScanBackend
{
public:
    explicit OvstageScanBackend(const OvstageAttach* attachPayload) : mAttachPayload(attachPayload)
    {
    }

    omni::physics::parse::ScannedStage scan(const parse::AttachTarget& target,
                                            const std::vector<std::string>& scanRoots,
                                            const std::vector<std::string>& excludePaths,
                                            const parse::ScanOptions& options,
                                            parse::IDescriptorAllocator& allocator) override
    {
        if (!target.nativeStage)
            return {};

        // `nativeStage` is backend-opaque and carries no tag saying which backend
        // minted it, so a target built for another backend — a USD attach's
        // `UsdStageWeakPtr*`, say — is bit-indistinguishable from ours at this
        // point. Reinterpreting one is undefined behaviour, and in practice a
        // SIGSEGV two frames down in OvstageSource. So accept only the payload this
        // backend was registered for; anything else is IScanBackend::scan's
        // documented "target this backend does not understand", reported rather
        // than dereferenced. (A registration with no payload matches nothing, which
        // is the safe direction.)
        if (target.nativeStage != mAttachPayload)
        {
            CARB_LOG_ERROR("ovstage scan backend received a target it does not own (nativeStage %p, expected %p); "
                           "returning an empty scan. A scan target must come from the attach this backend was "
                           "registered for -- parse a USD stage through the native USD walk instead.",
                           target.nativeStage, static_cast<const void*>(mAttachPayload));
            return {};
        }

        const OvstageAttach* attach = mAttachPayload;
        OvstageScanFilter filter;
        filter.scanRoots = scanRoots;
        filter.excludePaths = excludePaths;
        filter.prunePointInstancerDescendants = options.prunePointInstancerDescendants;
        filter.descendantScope = options.descendantScope;
        ovx_path_dictionary_t* dict = attach->dict ? attach->dict : ovstage_get_path_dictionary(attach->instance);
        const ovstage_ordinal_t readOrdinal =
            target.readOrdinal ? static_cast<ovstage_ordinal_t>(target.readOrdinal) : 1;
        // Read through the attach's live source when it is ours: the change feed keeps its
        // memos exact across drains, so the scan does not start cold (REQ-PARSE-BACKEND-001 AC-12).
        auto* warm = dynamic_cast<OvstageSource*>(target.attachedSource);
        if (warm && warm->instance() != attach->instance)
            warm = nullptr;
        return scanStageOvstage(
            attach->instance, dict, allocator, readOrdinal, &filter, target.residentBackingStageId, warm);
    }

private:
    // The one attach payload this backend serves. Not owned: the producer keeps it
    // alive for the attach, and the backend is registered and released with it.
    const OvstageAttach* mAttachPayload = nullptr;
};
} // namespace

std::unique_ptr<omni::physics::parse::IScanBackend> makeOvstageScanBackend(const void* attachPayload)
{
    return std::make_unique<OvstageScanBackend>(static_cast<const OvstageAttach*>(attachPayload));
}

} // namespace omni::physics::ovstage

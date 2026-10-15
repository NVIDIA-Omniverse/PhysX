// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-BATCH-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include "OvstageWalker.h"
#include "DescriptorMemoryStats.h"
#include <omni/physics/parse/PreparedVehicle.h>
#include <omni/physics/parse/DescriptorBatchAllocator.h>
#include <stdexcept>
namespace omni::physics::ovstage
{
using namespace omni::physics::parse;
struct PreparedOvstageVehicles
{
    std::vector<PreparedTireFrictionTable> tireFrictionTables;
    std::vector<PreparedVehicleContext> vehicleContexts;
    std::vector<PreparedWheel> vehicleWheels;
    std::vector<PreparedTire> vehicleTires;
    std::vector<PreparedSuspension> vehicleSuspensions;
    std::vector<PreparedEngine> vehicleEngines;
    std::vector<PreparedGears> vehicleGears;
    std::vector<PreparedClutch> vehicleClutches;
    std::vector<PreparedDriveBasic> vehicleDrivesBasic;
    std::vector<PreparedDriveStandard> vehicleDrivesStandard;
    std::vector<PreparedMultiWheelDifferential> vehicleMultiWheelDifferentials;
    std::vector<PreparedTankDifferential> vehicleTankDifferentials;
    std::vector<PreparedAutoGearBox> vehicleAutoGearBoxes;
    std::vector<PreparedBrakes> vehicleBrakes;
    std::vector<PreparedSteeringBasic> vehicleSteeringBasic;
    std::vector<PreparedSteeringAckermann> vehicleSteeringAckermann;
    std::vector<PreparedNonlinearCmdResponse> vehicleNonlinearCmdResponses;
    std::vector<PreparedSuspensionCompliance> vehicleSuspensionCompliances;
    std::vector<PreparedWheelAttachment> vehicleWheelAttachments;
    std::vector<PreparedVehicle> vehicles;
    std::vector<std::pair<ObjectKey, ObjectKey>> vehicleWheelAttachmentOwners;
};

template <typename Input, typename Desc, typename Storage, typename Fill, typename Finish, typename Publish>
void fillVehicleBatch(std::vector<Input>& inputs, std::vector<DescPtr<Desc>>& outputs,
                      DescriptorBatchAllocator& pool, const DescriptorExecutionOptions& execution,
                      Storage storage, Fill fill, Finish finish, Publish publish)
{
    if (inputs.empty()) return;
    DescriptorPipelineStats* stats = execution.statistics;
    DescriptorPhaseTimer phase(stats ? &stats->constructSeconds : nullptr);
    if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
    // Exactly one slot per classified applied instance is the bounded upper
    // count. Validation can reject a slot; only the coordinator destroys it.
    std::vector<DescPtr<Desc>> batch = allocateDescriptorBatch<Desc>(pool, inputs.size());
    std::vector<VehicleFillStatus> statuses(inputs.size());
    std::vector<uint8_t> valid(inputs.size());
    for (size_t i = 0; i < inputs.size(); ++i) storage(inputs[i], *batch[i]);
    phase.finish();
    fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
    {
        for (size_t i = begin; i < end; ++i) valid[i] = fill(inputs[i], *batch[i], statuses[i]);
    });
    phase.transition(stats ? &stats->finalizeSeconds : nullptr);
    for (const VehicleFillStatus& status : statuses)
        if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        statuses[i].report();
        if (valid[i])
        {
            finish(*batch[i], statuses[i]);
            outputs.push_back(std::move(batch[i]));
            publish(inputs[i]);
        }
    }
    std::vector<Input>().swap(inputs); // Release the wave before preparing the next family.
}

inline void flushPreparedVehicles(PreparedOvstageVehicles& inputs, OvstageScanResult& out,
                                   DescriptorBatchAllocator& pool, const DescriptorExecutionOptions& execution)
{
    fillVehicleBatch(inputs.tireFrictionTables, out.tireFrictionTables, pool, execution, prepareTireFrictionTableStorage, fillTireFrictionTable, finishTireFrictionTableStorage,
        [&](const PreparedTireFrictionTable& input) { (void)input; });
    fillVehicleBatch(inputs.vehicleContexts, out.vehicleContexts, pool, execution, prepareVehicleContextStorage, fillVehicleContext, finishVehicleContextStorage,
        [&](const PreparedVehicleContext& input) { (void)input; });
    fillVehicleBatch(inputs.vehicleWheels, out.vehicleWheels, pool, execution, prepareWheelStorage, fillWheel, finishWheelStorage,
        [&](const PreparedWheel& input) { (void)input; });
    fillVehicleBatch(inputs.vehicleTires, out.vehicleTires, pool, execution, prepareTireStorage, fillTire, finishTireStorage,
        [&](const PreparedTire& input) { (void)input; });
    fillVehicleBatch(inputs.vehicleSuspensions, out.vehicleSuspensions, pool, execution, prepareSuspensionStorage, fillSuspension, finishSuspensionStorage,
        [&](const PreparedSuspension& input) { (void)input; });
    fillVehicleBatch(inputs.vehicleEngines, out.vehicleEngines, pool, execution, prepareEngineStorage, fillEngine, finishEngineStorage,
        [&](const PreparedEngine& input) { (void)input; });
    fillVehicleBatch(inputs.vehicleGears, out.vehicleGears, pool, execution, prepareGearsStorage, fillGears, finishGearsStorage,
        [&](const PreparedGears& input) { out.vehicleGearsPaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleClutches, out.vehicleClutches, pool, execution, prepareClutchStorage, fillClutch, finishClutchStorage,
        [&](const PreparedClutch& input) { out.vehicleClutchPaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleDrivesBasic, out.vehicleDrivesBasic, pool, execution, prepareDriveBasicStorage, fillDriveBasic, finishDriveBasicStorage,
        [&](const PreparedDriveBasic& input) { (void)input; });
    fillVehicleBatch(inputs.vehicleDrivesStandard, out.vehicleDrivesStandard, pool, execution, prepareDriveStandardStorage, fillDriveStandard, finishDriveStandardStorage,
        [&](const PreparedDriveStandard& input) { out.vehicleDrivesStandardPaths.push_back(input.key); out.vehicleDrivesStandardCrossRefs.push_back(input.info); });
    fillVehicleBatch(inputs.vehicleMultiWheelDifferentials, out.vehicleMultiWheelDifferentials, pool, execution, prepareMultiWheelDifferentialStorage, fillMultiWheelDifferential, finishMultiWheelDifferentialStorage,
        [&](const PreparedMultiWheelDifferential& input) { out.vehicleMultiWheelDifferentialPaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleTankDifferentials, out.vehicleTankDifferentials, pool, execution, prepareTankDifferentialStorage, fillTankDifferential, finishTankDifferentialStorage,
        [&](const PreparedTankDifferential& input) { out.vehicleTankDifferentialPaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleAutoGearBoxes, out.vehicleAutoGearBoxes, pool, execution, prepareAutoGearBoxStorage, fillAutoGearBox, finishAutoGearBoxStorage,
        [&](const PreparedAutoGearBox& input) { out.vehicleAutoGearBoxPaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleBrakes, out.vehicleBrakes, pool, execution, prepareBrakesStorage, fillBrakes, finishBrakesStorage,
        [&](const PreparedBrakes& input) { out.vehicleBrakesPaths.push_back(input.key); out.vehicleBrakesInstanceTokens.push_back(input.instanceToken); });
    fillVehicleBatch(inputs.vehicleSteeringBasic, out.vehicleSteeringBasic, pool, execution, prepareSteeringBasicStorage, fillSteeringBasic, finishSteeringBasicStorage,
        [&](const PreparedSteeringBasic& input) { out.vehicleSteeringBasicPaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleSteeringAckermann, out.vehicleSteeringAckermann, pool, execution, prepareSteeringAckermannStorage, fillSteeringAckermann, finishSteeringAckermannStorage,
        [&](const PreparedSteeringAckermann& input) { out.vehicleSteeringAckermannPaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleNonlinearCmdResponses, out.vehicleNonlinearCmdResponses, pool, execution, prepareNonlinearCmdResponseStorage, fillNonlinearCmdResponse, finishNonlinearCmdResponseStorage,
        [&](const PreparedNonlinearCmdResponse& input) { out.vehicleNonlinearCmdResponsePaths.push_back(input.ownerKey); out.vehicleNonlinearCmdResponseInstanceTokens.push_back(input.instanceToken); });
    fillVehicleBatch(inputs.vehicleSuspensionCompliances, out.vehicleSuspensionCompliances, pool, execution, prepareSuspensionComplianceStorage, fillSuspensionCompliance, finishSuspensionComplianceStorage,
        [&](const PreparedSuspensionCompliance& input) { out.vehicleSuspensionCompliancePaths.push_back(input.key); });
    fillVehicleBatch(inputs.vehicleWheelAttachments, out.vehicleWheelAttachments, pool, execution, prepareWheelAttachmentStorage, fillWheelAttachment, finishWheelAttachmentStorage,
        [&](const PreparedWheelAttachment& input) { out.vehicleWheelAttachmentInfos.push_back(input.info); });
    fillVehicleBatch(inputs.vehicles, out.vehicles, pool, execution, prepareVehicleStorage, fillVehicle, finishVehicleStorage,
        [&](const PreparedVehicle& input) { out.vehiclePaths.push_back(input.key); });
    out.vehicleWheelAttachmentOwners.insert(out.vehicleWheelAttachmentOwners.end(),
        inputs.vehicleWheelAttachmentOwners.begin(), inputs.vehicleWheelAttachmentOwners.end());
    inputs.vehicleWheelAttachmentOwners.clear();

}
}

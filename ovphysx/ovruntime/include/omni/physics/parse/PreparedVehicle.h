// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include "ParseApi.h"
#include "PreparedVehicleSupport.h"
namespace omni::physics::parse
{
struct PreparedSuspension
{
    PreparedVehicleReads reads;
    ObjectKey key{};
};
PreparedSuspension prepareSuspension(ParseContext& ctx, ObjectKey key);
void prepareSuspensionStorage(const PreparedSuspension& input, SuspensionDesc& output);
bool fillSuspension(const PreparedSuspension& input, SuspensionDesc& output, VehicleFillStatus& status) noexcept;
void finishSuspensionStorage(SuspensionDesc& output, const VehicleFillStatus& status);

struct PreparedSteeringAckermann
{
    PreparedVehicleReads reads;
    ObjectKey key{};
};
PreparedSteeringAckermann prepareSteeringAckermann(ParseContext& ctx, ObjectKey key);
void prepareSteeringAckermannStorage(const PreparedSteeringAckermann& input, SteeringAckermannDesc& output);
bool fillSteeringAckermann(const PreparedSteeringAckermann& input, SteeringAckermannDesc& output, VehicleFillStatus& status) noexcept;
void finishSteeringAckermannStorage(SteeringAckermannDesc& output, const VehicleFillStatus& status);

struct PreparedSteeringBasic
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    SteeringBasicInfo info{};
};
PreparedSteeringBasic prepareSteeringBasic(ParseContext& ctx, ObjectKey key, const SteeringBasicInfo& info);
void prepareSteeringBasicStorage(const PreparedSteeringBasic& input, SteeringBasicDesc& output);
bool fillSteeringBasic(const PreparedSteeringBasic& input, SteeringBasicDesc& output, VehicleFillStatus& status) noexcept;
void finishSteeringBasicStorage(SteeringBasicDesc& output, const VehicleFillStatus& status);

struct PreparedWheelAttachment
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    WheelAttachmentInfo info{};
};
PreparedWheelAttachment prepareWheelAttachment(ParseContext& ctx, ObjectKey key, const WheelAttachmentInfo& info);
void prepareWheelAttachmentStorage(const PreparedWheelAttachment& input, WheelAttachmentDesc& output);
bool fillWheelAttachment(const PreparedWheelAttachment& input, WheelAttachmentDesc& output, VehicleFillStatus& status) noexcept;
void finishWheelAttachmentStorage(WheelAttachmentDesc& output, const VehicleFillStatus& status);

struct PreparedAutoGearBox
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    AutoGearBoxInfo info{};
};
PreparedAutoGearBox prepareAutoGearBox(ParseContext& ctx, ObjectKey key, const AutoGearBoxInfo& info);
void prepareAutoGearBoxStorage(const PreparedAutoGearBox& input, AutoGearBoxDesc& output);
bool fillAutoGearBox(const PreparedAutoGearBox& input, AutoGearBoxDesc& output, VehicleFillStatus& status) noexcept;
void finishAutoGearBoxStorage(AutoGearBoxDesc& output, const VehicleFillStatus& status);

struct PreparedTire
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    TireInfo info{};
};
PreparedTire prepareTire(ParseContext& ctx, ObjectKey key, const TireInfo& info);
void prepareTireStorage(const PreparedTire& input, TireDesc& output);
bool fillTire(const PreparedTire& input, TireDesc& output, VehicleFillStatus& status) noexcept;
void finishTireStorage(TireDesc& output, const VehicleFillStatus& status);

struct PreparedBrakes
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    std::string instanceName{};
    uint8_t brakesIndex{};
    BrakesInfo info{};
    TokenId instanceToken;
};
PreparedBrakes prepareBrakes(ParseContext& ctx, ObjectKey key, std::string_view instanceName, uint8_t brakesIndex, const BrakesInfo& info);
void prepareBrakesStorage(const PreparedBrakes& input, BrakesDesc& output);
bool fillBrakes(const PreparedBrakes& input, BrakesDesc& output, VehicleFillStatus& status) noexcept;
void finishBrakesStorage(BrakesDesc& output, const VehicleFillStatus& status);

struct PreparedVehicleContext
{
    PreparedVehicleReads reads;
    ObjectKey key{};
};
PreparedVehicleContext prepareVehicleContext(ParseContext& ctx, ObjectKey key);
void prepareVehicleContextStorage(const PreparedVehicleContext& input, VehicleContextDesc& output);
bool fillVehicleContext(const PreparedVehicleContext& input, VehicleContextDesc& output, VehicleFillStatus& status) noexcept;
void finishVehicleContextStorage(VehicleContextDesc& output, const VehicleFillStatus& status);

struct PreparedNonlinearCmdResponse
{
    PreparedVehicleReads reads;
    ObjectKey ownerKey{};
    std::string instanceName{};
    NonlinearCmdResponseInfo info{};
    TokenId instanceToken;
};
PreparedNonlinearCmdResponse prepareNonlinearCmdResponse(ParseContext& ctx, ObjectKey ownerKey, std::string_view instanceName, const NonlinearCmdResponseInfo& info);
void prepareNonlinearCmdResponseStorage(const PreparedNonlinearCmdResponse& input, NonlinearCmdResponseDesc& output);
bool fillNonlinearCmdResponse(const PreparedNonlinearCmdResponse& input, NonlinearCmdResponseDesc& output, VehicleFillStatus& status) noexcept;
void finishNonlinearCmdResponseStorage(NonlinearCmdResponseDesc& output, const VehicleFillStatus& status);

struct PreparedWheel
{
    PreparedVehicleReads reads;
    ObjectKey key{};
};
PreparedWheel prepareWheel(ParseContext& ctx, ObjectKey key);
void prepareWheelStorage(const PreparedWheel& input, WheelDesc& output);
bool fillWheel(const PreparedWheel& input, WheelDesc& output, VehicleFillStatus& status) noexcept;
void finishWheelStorage(WheelDesc& output, const VehicleFillStatus& status);

struct PreparedClutch
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    float kgmsScale{};
};
PreparedClutch prepareClutch(ParseContext& ctx, ObjectKey key, float kgmsScale);
void prepareClutchStorage(const PreparedClutch& input, ClutchDesc& output);
bool fillClutch(const PreparedClutch& input, ClutchDesc& output, VehicleFillStatus& status) noexcept;
void finishClutchStorage(ClutchDesc& output, const VehicleFillStatus& status);

struct PreparedGears
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    GearsInfo info{};
};
PreparedGears prepareGears(ParseContext& ctx, ObjectKey key, const GearsInfo& info);
void prepareGearsStorage(const PreparedGears& input, GearsDesc& output);
bool fillGears(const PreparedGears& input, GearsDesc& output, VehicleFillStatus& status) noexcept;
void finishGearsStorage(GearsDesc& output, const VehicleFillStatus& status);

struct PreparedEngine
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    EngineInfo info{};
};
PreparedEngine prepareEngine(ParseContext& ctx, ObjectKey key, const EngineInfo& info);
void prepareEngineStorage(const PreparedEngine& input, EngineDesc& output);
bool fillEngine(const PreparedEngine& input, EngineDesc& output, VehicleFillStatus& status) noexcept;
void finishEngineStorage(EngineDesc& output, const VehicleFillStatus& status);

struct PreparedDriveBasic
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    float kgmsScale{};
};
PreparedDriveBasic prepareDriveBasic(ParseContext& ctx, ObjectKey key, float kgmsScale);
void prepareDriveBasicStorage(const PreparedDriveBasic& input, DriveBasicDesc& output);
bool fillDriveBasic(const PreparedDriveBasic& input, DriveBasicDesc& output, VehicleFillStatus& status) noexcept;
void finishDriveBasicStorage(DriveBasicDesc& output, const VehicleFillStatus& status);

struct PreparedSuspensionCompliance
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    SuspensionComplianceInfo info{};
};
PreparedSuspensionCompliance prepareSuspensionCompliance(ParseContext& ctx, ObjectKey key, const SuspensionComplianceInfo& info);
void prepareSuspensionComplianceStorage(const PreparedSuspensionCompliance& input, SuspensionComplianceDesc& output);
bool fillSuspensionCompliance(const PreparedSuspensionCompliance& input, SuspensionComplianceDesc& output, VehicleFillStatus& status) noexcept;
void finishSuspensionComplianceStorage(SuspensionComplianceDesc& output, const VehicleFillStatus& status);

struct PreparedTankDifferential
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    DifferentialInfo info{};
    TankDifferentialInfo tankInfo{};
};
PreparedTankDifferential prepareTankDifferential(ParseContext& ctx, ObjectKey key, const DifferentialInfo& info, const TankDifferentialInfo& tankInfo);
void prepareTankDifferentialStorage(const PreparedTankDifferential& input, TankDifferentialDesc& output);
bool fillTankDifferential(const PreparedTankDifferential& input, TankDifferentialDesc& output, VehicleFillStatus& status) noexcept;
void finishTankDifferentialStorage(TankDifferentialDesc& output, const VehicleFillStatus& status);

struct PreparedMultiWheelDifferential
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    DifferentialInfo info{};
};
PreparedMultiWheelDifferential prepareMultiWheelDifferential(ParseContext& ctx, ObjectKey key, const DifferentialInfo& info);
void prepareMultiWheelDifferentialStorage(const PreparedMultiWheelDifferential& input, MultiWheelDifferentialDesc& output);
bool fillMultiWheelDifferential(const PreparedMultiWheelDifferential& input, MultiWheelDifferentialDesc& output, VehicleFillStatus& status) noexcept;
void finishMultiWheelDifferentialStorage(MultiWheelDifferentialDesc& output, const VehicleFillStatus& status);

struct PreparedDriveStandard
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    DriveStandardInfo info{};
};
PreparedDriveStandard prepareDriveStandard(ParseContext& ctx, ObjectKey key, const DriveStandardInfo& info);
void prepareDriveStandardStorage(const PreparedDriveStandard& input, DriveStandardDesc& output);
bool fillDriveStandard(const PreparedDriveStandard& input, DriveStandardDesc& output, VehicleFillStatus& status) noexcept;
void finishDriveStandardStorage(DriveStandardDesc& output, const VehicleFillStatus& status);

struct PreparedVehicle
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    VehicleInfo info{};
};
PreparedVehicle prepareVehicle(ParseContext& ctx, ObjectKey key, const VehicleInfo& info);
void prepareVehicleStorage(const PreparedVehicle& input, VehicleDesc& output);
bool fillVehicle(const PreparedVehicle& input, VehicleDesc& output, VehicleFillStatus& status) noexcept;
void finishVehicleStorage(VehicleDesc& output, const VehicleFillStatus& status);

struct PreparedTireFrictionTable
{
    PreparedVehicleReads reads;
    ObjectKey key{};
    TireFrictionTableInfo info{};
};
PreparedTireFrictionTable prepareTireFrictionTable(ParseContext& ctx, ObjectKey key, const TireFrictionTableInfo& info);
void prepareTireFrictionTableStorage(const PreparedTireFrictionTable& input, TireFrictionTableDesc& output);
bool fillTireFrictionTable(const PreparedTireFrictionTable& input, TireFrictionTableDesc& output, VehicleFillStatus& status) noexcept;
void finishTireFrictionTableStorage(TireFrictionTableDesc& output, const VehicleFillStatus& status);

}

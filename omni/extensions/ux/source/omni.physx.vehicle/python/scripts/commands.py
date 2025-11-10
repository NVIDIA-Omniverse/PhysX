# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
import math
import copy

from omni.usd.commands.usd_commands import DeletePrimsCommand

from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf, UsdPhysics, PhysxSchema, Vt
from omni.physxcommands import (
    PhysicsCommand,
    ApplyAPISchemaCommand,
    UnapplyAPISchemaCommand,
    SetCustomMetadataCommand,
)
from omni.physx.scripts.pythonUtils import autoassign
from omni.physx.scripts.physicsUtils import remove_collision_from_collision_group
from omni.physx.scripts.utils import (
    has_custom_metadata,
    get_custom_metadata,
)

from . wizards import physxVehicleWizard as Wizard


#
# Some general info:
# - closing a stage is supposed to clear the undo stack
# - storing references to prims in commands will not work in most cases since
#   after the command, the referenced prim might get deleted. An undo will 
#   recreate the object but the prim will not be the same. Thus, only path
#   strings seem safe to store.
#


TIRE_FRICTION_TABLE_PATH_PREFIX = "/TireFrictionTable"


def getRelationshipAtPath(path):
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage:
        return stage.GetObjectAtPath(path)
    else:
        return None


def getAttributeAtPath(path):
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage:
        return stage.GetAttributeAtPath(path)
    else:
        return None


def getPrimAtPath(path):
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage:
        return stage.GetPrimAtPath(path)
    else:
        return None


class PhysXVehicleSetRelationshipCommand(PhysicsCommand):
    @autoassign
    def __init__(self, relationshipPath, targetPath):
        pass

    def do(self):
        relationship = getRelationshipAtPath(self._relationshipPath)
        if relationship:
            targets = relationship.GetTargets()
            if (len(targets) > 0):
                self._oldTargetPath = targets[0].pathString
            else:
                self._oldTargetPath = None

            relationship.SetTargets([self._targetPath])

    def undo(self):
        relationship = getRelationshipAtPath(self._relationshipPath)
        if (relationship and (self._oldTargetPath is not None)):
            relationship.SetTargets([self._oldTargetPath])


class PhysXVehicleSetArrayEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, arrayAttributePath, entries):
        pass

    def do(self):
        self._oldArray = None
        attr = getAttributeAtPath(self._arrayAttributePath)
        if (attr):
            self._oldArray = []
            if (attr.HasAuthoredValue()):
                self._hadAuthoredValue = True
                values = attr.Get()
                for v in values:
                    self._oldArray.append(v)
            else:
                self._hadAuthoredValue = False

            attr.Set(self._entries)

    def undo(self):
        if (self._oldArray is not None):
            attr = getAttributeAtPath(self._arrayAttributePath)
            if (attr):
                if (self._hadAuthoredValue):
                    attr.Set(self._oldArray)
                else:
                    attr.Clear()


class PhysXVehicleChangeArrayEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, arrayAttributePath, entryIndex, value, oldValue = None,
        fillValue = None, minFillSize = None):

        # arrayAttributePath: string of the USD attribute path of the array attribute
        # entryIndex: index of the array entry to change
        # value: new value for the entry
        # oldValue: if not None, then this value will be used for the undo operation instead
        #           of the current value at the defined index
        # fillValue: if not None and if entryIndex points beyond the last array entry, then
        #            the specified value will be used to fill in the undefined entries
        # minFillSize: if not None and if larger than the array size, then fillValue will be
        #              used to fill in the undefined entries that go beyond entryIndex up to
        #              an array size of minFillSize

        pass

    def do(self):
        attr = getAttributeAtPath(self._arrayAttributePath)
        if (attr):
            self._oldArray = None
            self._hadAuthoredValue = False  # only used together with self._oldArray

            if (attr.HasValue()):
                values = attr.Get()
                valueCount = len(values)
            else:
                values = []
                valueCount = 0

            if (self._entryIndex < valueCount):
                if (self._oldValue is None):
                    self._oldValue = values[self._entryIndex]
                values[self._entryIndex] = self._value
            
                attr.Set(values)
            elif (self._fillValue is not None):
                self._oldArray = []
                self._hadAuthoredValue = attr.HasAuthoredValue()
                newValues = []
                for v in values:
                    self._oldArray.append(v)
                    newValues.append(v)

                i = valueCount
                while (i < self._entryIndex):
                    newValues.append(self._fillValue)

                    i = i + 1

                newValues.append(self._value)

                if (self._minFillSize is not None):
                    i = i + 1
                    while (i < self._minFillSize):
                        newValues.append(self._fillValue)

                        i = i + 1

                attr.Set(newValues)

    def undo(self):
        attr = getAttributeAtPath(self._arrayAttributePath)
        if (attr and attr.HasValue()):
            if (self._oldArray is None):
                values = attr.Get()
                valueCount = len(values)

                if (self._entryIndex < valueCount):
                    values[self._entryIndex] = self._oldValue
            
                    attr.Set(values)
            else:
                if (self._hadAuthoredValue):
                    attr.Set(self._oldArray)
                else:
                    attr.Clear()


class PhysXVehicleAddArrayEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, arrayAttributePath, entry):
        pass

    def do(self):
        self._oldArray = None
        attr = getAttributeAtPath(self._arrayAttributePath)
        if (attr):
            self._oldArray = []
            self._hadAuthoredValue = attr.HasAuthoredValue()

            # this is gaga but could not find an append/push_back in Vt.FloatArray
            valuesWithEntryAdded = []

            if (attr.HasValue()):
                values = attr.Get()
                for v in values:
                    valuesWithEntryAdded.append(v)
                    self._oldArray.append(v)

            valuesWithEntryAdded.append(self._entry)

            attr.Set(valuesWithEntryAdded)

    def undo(self):
        if (self._oldArray is not None):
            attr = getAttributeAtPath(self._arrayAttributePath)
            if (attr):
                if (self._hadAuthoredValue):
                    attr.Set(self._oldArray)
                else:
                    attr.Clear()


class PhysXVehicleRemoveArrayEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, arrayAttributePath, entryIndex, oldValue = None):
        pass

    def do(self):
        self._oldArray = None
        attr = getAttributeAtPath(self._arrayAttributePath)
        if (attr and attr.HasValue()):
            values = attr.Get()
            valueCount = len(values)
            if (self._entryIndex < valueCount):
                self._oldArray = []

                # this is gaga but could not find a remove/del in Vt.FloatArray
                # and concatenation with slicing does not seem to work either
                valuesWithEntryRemoved = []
                i = 0
                while (i < self._entryIndex):
                    self._oldArray.append(values[i])
                    valuesWithEntryRemoved.append(values[i])
                    i = i + 1

                if (self._oldValue is None):
                    self._oldArray.append(values[i])
                else:
                    self._oldArray.append(self._oldValue)

                i = i + 1
                while (i < valueCount):
                    self._oldArray.append(values[i])
                    valuesWithEntryRemoved.append(values[i])
                    i = i + 1

                attr.Set(valuesWithEntryRemoved)

    def undo(self):
        if (self._oldArray is not None):
            attr = getAttributeAtPath(self._arrayAttributePath)
            if (attr):
                attr.Set(self._oldArray)


class PhysXVehicleTireFrictionTableAddCommand(PhysicsCommand):
    @autoassign
    def __init__(self, setPrimAsSelected: bool = True):
        pass

    def do(self):
        context = omni.usd.get_context()
        stage = context.get_stage()
        if stage:
            self._path = omni.usd.get_stage_next_free_path(stage, TIRE_FRICTION_TABLE_PATH_PREFIX, True)
            PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, self._path)
            if (self._setPrimAsSelected):
                context.get_selection().set_selected_prim_paths([self._path], True)
        else:
            self._path = None

    def undo(self):
        if (self._path is not None):
            DeletePrimsCommand([self._path]).do()


class PhysXVehicleTireFrictionTableAddEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, tireFrictionTablePath, materialPath, frictionValue):
        pass

    def do(self):
        prim = getPrimAtPath(self._tireFrictionTablePath)
        if (prim):
            tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(prim)
            tireFrictionTable.GetGroundMaterialsRel().AddTarget(self._materialPath)

            frictionValuesAttr = tireFrictionTable.GetFrictionValuesAttr()
            PhysXVehicleAddArrayEntryCommand.execute(frictionValuesAttr.GetPath().pathString,
                self._frictionValue)

    def undo(self):
        prim = getPrimAtPath(self._tireFrictionTablePath)
        if (prim):
            tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(prim)
            tireFrictionTable.GetGroundMaterialsRel().RemoveTarget(self._materialPath)


class PhysXVehicleTireFrictionTableRemoveEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, tireFrictionTablePath, entryIndex):
        pass

    def do(self):
        prim = getPrimAtPath(self._tireFrictionTablePath)
        if (prim):
            tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(prim)
            relationships = tireFrictionTable.GetGroundMaterialsRel()
            self._removedMaterialPath = relationships.GetTargets()[self._entryIndex].pathString
            relationships.RemoveTarget(self._removedMaterialPath)

            frictionValuesAttr = tireFrictionTable.GetFrictionValuesAttr()
            PhysXVehicleRemoveArrayEntryCommand.execute(frictionValuesAttr.GetPath().pathString, 
                self._entryIndex)

    def undo(self):
        prim = getPrimAtPath(self._tireFrictionTablePath)
        if (prim):
            tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(prim)
            relationships = tireFrictionTable.GetGroundMaterialsRel()
            targets = relationships.GetTargets()
            targets.insert(self._entryIndex, self._removedMaterialPath)
            relationships.SetTargets(targets)

            # the friction value part will be covered by the undo of PhysXVehicleRemoveArrayEntryCommand


class PhysXVehicleTireFrictionTableChangeEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, tireFrictionTablePath, entryIndex, frictionValue, oldFrictionValue = None):
        pass

    def do(self):
        prim = getPrimAtPath(self._tireFrictionTablePath)
        if (prim):
            tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(prim)
            PhysXVehicleChangeArrayEntryCommand.execute(tireFrictionTable.GetFrictionValuesAttr().GetPath().pathString,
                self._entryIndex, self._frictionValue, self._oldFrictionValue)

    def undo(self):
        pass


class PhysXVehicleCommandResponseChangeEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, valueArrayPath, indexArrayPath, entryIndex,
        valueEntry, oldValueEntry, indexEntry, fillValue, maxIndexCount):
        pass

    def do(self):
        newEntryIndex = self._entryIndex

        indexAttr = getAttributeAtPath(self._indexArrayPath)
        if (indexAttr and indexAttr.HasValue()):
            indices = indexAttr.Get()
            indexCount = len(indices)
        else:
            indexCount = 0

        if (indexCount > 0):
            if (self._entryIndex >= 0):
                # the index is in the index array
                # - matching entry in the value array => only that entry will get changed using oldValueEntry
                # or
                # - value array is empty => set valueEntry for the entry at entryIndex and also set values
                #   for all the other defined index entries using fillValue
                PhysXVehicleChangeArrayEntryCommand.execute(self._valueArrayPath, self._entryIndex,
                    self._valueEntry, self._oldValueEntry, self._fillValue, indexCount)
            else:
                # the index is not in the index array but there are other indices defined => add index to
                # the array and add the corresponding value to the value array, potentially setting values
                # for the other indices using fillValue
                PhysXVehicleAddArrayEntryCommand.execute(self._indexArrayPath, self._indexEntry)

                newEntryIndex = indexCount
                PhysXVehicleChangeArrayEntryCommand.execute(self._valueArrayPath, newEntryIndex,
                    self._valueEntry, None, self._fillValue, indexCount)
        else:
            # the index array is empty which is treated as all using the fillValue => fill the index array
            # with the full range of indices. Fill the value array with fillValue but use valueEntry for
            # the entry at indexEntry
            indexArray = range(self._maxIndexCount)
            valueArray = [self._fillValue] * self._maxIndexCount
            newEntryIndex = self._indexEntry
            valueArray[newEntryIndex] = self._valueEntry

            PhysXVehicleSetArrayEntryCommand.execute(self._indexArrayPath, indexArray)
            PhysXVehicleSetArrayEntryCommand.execute(self._valueArrayPath, valueArray)

        return newEntryIndex

    def undo(self):
        pass


class PhysXVehicleDifferentialChangeEntryCommand(PhysicsCommand):
    @autoassign
    def __init__(self, valueArrayPaths, valueArrayPathIndex,
        indexArrayPath, entryIndex,
        valueEntry, oldValueEntry, indexEntry):
        pass

    def do(self):
        newEntryIndex = self._entryIndex

        indexAttr = getAttributeAtPath(self._indexArrayPath)
        if (indexAttr and indexAttr.HasValue()):
            indices = indexAttr.Get()
            indexCount = len(indices)
        else:
            indexCount = 0

        if (self._entryIndex >= 0):
            # the index is in the index array

            if (self._valueEntry != 0):
                # - matching entry in the value array => only that entry will get changed using oldValueEntry
                # or
                # - value array is empty => set valueEntry for the entry at entryIndex and also set values
                #   for all the other defined index entries using fillValue
                if (indexCount > 1):
                    fillValue = (1.0 - math.fabs(self._valueEntry)) / (indexCount - 1)
                else:
                    fillValue = 0.0

                PhysXVehicleChangeArrayEntryCommand.execute(self._valueArrayPaths[self._valueArrayPathIndex], self._entryIndex,
                    self._valueEntry, self._oldValueEntry, fillValue, indexCount)
            else:
                # remove entry from index array and all value arrays if it turns 0 in all
                # value arrays (empty value array is also considered fine for removal).
                # The logic relies on the remove command to handle the case where the
                # index points beyond the array size.
                remove = True
                for i in range(len(self._valueArrayPaths)):
                    if (i != self._valueArrayPathIndex):
                        valueArrayAttr = getAttributeAtPath(self._valueArrayPaths[i])
                        if (valueArrayAttr and valueArrayAttr.HasValue()):
                            values = valueArrayAttr.Get()
                            valueCount = len(values)
                            if ((valueCount > self._entryIndex) and (values[self._entryIndex] != 0.0)):
                                remove = False
                                break

                if (remove):
                    PhysXVehicleRemoveArrayEntryCommand.execute(self._indexArrayPath, self._entryIndex)
                    for i in range(len(self._valueArrayPaths)):
                        attrPath = self._valueArrayPaths[i]
                        if (i != self._valueArrayPathIndex):
                            oldVal = None
                        else:
                            # make sure to use the old value because certain code paths might have changed
                            # the entry in the value array to 0 already
                            oldVal = self._oldValueEntry
                        PhysXVehicleRemoveArrayEntryCommand.execute(attrPath, self._entryIndex, oldVal)

                    newEntryIndex = -1
                else:
                    if (indexCount > 1):
                        fillValue = 1.0 / (indexCount - 1)
                    else:
                        fillValue = 0.0

                    PhysXVehicleChangeArrayEntryCommand.execute(self._valueArrayPaths[self._valueArrayPathIndex], self._entryIndex,
                        self._valueEntry, self._oldValueEntry, fillValue, indexCount)
        elif (self._valueEntry != 0):
            # the index is not in the index array => add index to the array and add the corresponding
            # value to the value array, potentially setting values for the other indices using fillValue.
            # Add 0 to other value arrays if they are defined.
            PhysXVehicleAddArrayEntryCommand.execute(self._indexArrayPath, self._indexEntry)

            if (indexCount > 0):
                fillValue = (1.0 - math.fabs(self._valueEntry)) / indexCount
            else:
                fillValue = 0.0

            newEntryIndex = indexCount
            PhysXVehicleChangeArrayEntryCommand.execute(self._valueArrayPaths[self._valueArrayPathIndex], newEntryIndex,
                self._valueEntry, None, fillValue, indexCount)

            for i in range(len(self._valueArrayPaths)):
                if (i != self._valueArrayPathIndex):
                    valueArrayPath = self._valueArrayPaths[i]
                    valueArrayAttr = getAttributeAtPath(valueArrayPath)
                    if (valueArrayAttr and valueArrayAttr.HasValue()):
                        values = valueArrayAttr.Get()
                        valueCount = len(values)
                        if (valueCount == indexCount):
                            PhysXVehicleAddArrayEntryCommand.execute(valueArrayPath, 0)

        return newEntryIndex

    def undo(self):
        pass


class SuspensionFrameTransformsAutocomputeBackup:
    def __init__(self, prim):
        self._wheelAttPrimPath = prim.GetPath().pathString
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(prim)

        suspFramePosAttr = wheelAttAPI.GetSuspensionFramePositionAttr()
        if (suspFramePosAttr.HasAuthoredValue()):
            self._suspFramePos = suspFramePosAttr.Get()
        else:
            self._suspFramePos = None

        suspFrameOrientAttr = wheelAttAPI.GetSuspensionFrameOrientationAttr()
        if (suspFrameOrientAttr.HasAuthoredValue()):
            self._suspFrameOrient = suspFrameOrientAttr.Get()
        else:
            self._suspFrameOrient = None

    def set_back(self):
        prim = getPrimAtPath(self._wheelAttPrimPath)
        if (prim):
            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(prim)

            suspFramePosAttr = wheelAttAPI.GetSuspensionFramePositionAttr()
            if (self._suspFramePos is not None):
                suspFramePosAttr.Set(self._suspFramePos)
            else:
                suspFramePosAttr.Clear()

            suspFrameOrientAttr = wheelAttAPI.GetSuspensionFrameOrientationAttr()
            if (self._suspFrameOrient is not None):
                suspFrameOrientAttr.Set(self._suspFrameOrient)
            else:
                suspFrameOrientAttr.Clear()


class PhysXVehicleSuspensionFrameTransformsAutocomputeCommand(PhysicsCommand):
    @autoassign
    def __init__(self, primPath):
        pass

    def do(self):
        self._backupList = []
        prim = getPrimAtPath(self._primPath)
        if (prim):
            if prim.HasAPI(PhysxSchema.PhysxVehicleAPI):
                for p in Usd.PrimRange(prim):  # there is no GetDescendants()
                    if p.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI):
                        self._backupList.append(SuspensionFrameTransformsAutocomputeBackup(p))
            else:
                self._backupList.append(SuspensionFrameTransformsAutocomputeBackup(prim))

            physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
            physxVehicleInterface.compute_suspension_frame_transforms(self._primPath)

    def undo(self):
        for backup in self._backupList:
            backup.set_back()


class PhysXVehicleControllerEnableInputCommand(PhysicsCommand):
    @autoassign
    def __init__(self, primPath, enabled):
        pass

    def do(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_input_enabled(self._primPath, self._enabled)

    def undo(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_input_enabled(self._primPath, not self._enabled)


class PhysXVehicleControllerEnableMouseCommand(PhysicsCommand):
    @autoassign
    def __init__(self, primPath, enabled):
        pass

    def do(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_mouse_enabled(self._primPath, self._enabled)

    def undo(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_mouse_enabled(self._primPath, not self._enabled)


class PhysXVehicleControllerEnableAutoReverseCommand(PhysicsCommand):
    @autoassign
    def __init__(self, primPath, enabled):
        pass

    def do(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_auto_reverse_enabled(self._primPath, self._enabled)

    def undo(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_auto_reverse_enabled(self._primPath, not self._enabled)


class PhysXVehicleControllerSetSteeringSensitivityCommand(PhysicsCommand):
    @autoassign
    def __init__(self, primPath, sensitivity):
        self._previousSensitivity = 0.0

    def do(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        self._previousSensitivity = physxVehicleInterface.get_steering_sensitivity(self._primPath)
        physxVehicleInterface.set_steering_sensitivity(self._primPath, self._sensitivity)

    def undo(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_steering_sensitivity(self._primPath, self._previousSensitivity)


class PhysXVehicleControllerSetSteeringFilterTimeCommand(PhysicsCommand):
    @autoassign
    def __init__(self, primPath, timeConstant):
        self._previousFilterTime = 0.0

    def do(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        self._previousFilterTime = physxVehicleInterface.get_steering_filter_time(self._primPath)
        physxVehicleInterface.set_steering_filter_time(self._primPath, self._timeConstant)

    def undo(self):
        physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        physxVehicleInterface.set_steering_filter_time(self._primPath, self._previousFilterTime)


class PhysXVehicleWizardCreateCommand(PhysicsCommand):
    def __init__(self, vehicleData):
        self._vehicleData = copy.deepcopy(vehicleData)
        self._operationTracker = Wizard.OperationTracker()
        self._operationTracker.applyAPISingleCallback = self._on_apply_api_schema_single
        self._operationTracker.applyAPIMultipleCallback = self._on_apply_api_schema_multiple

    def _on_apply_api_schema_single(self, prim, apiSchema):
        if (prim.HasAPI(apiSchema)):
            # remove existing APIs first. The idea is that this should avoid tracking all the
            # changed attributes for the undo operation (UnapplyAPISchemaCommand takes
            # care of it).
            UnapplyAPISchemaCommand.execute(apiSchema, prim)

        ApplyAPISchemaCommand.execute(apiSchema, prim)

    def _on_apply_api_schema_multiple(self, prim, apiSchema, api_prefix, multiple_api_token):
        if (prim.HasAPI(apiSchema, multiple_api_token)):
            UnapplyAPISchemaCommand.execute(apiSchema, prim, api_prefix, multiple_api_token)

        ApplyAPISchemaCommand.execute(apiSchema, prim, api_prefix, multiple_api_token)

    def do(self):
        self._operationTracker.reset()

        context = omni.usd.get_context()
        stage = context.get_stage()

        if (self._vehicleData.vehiclePath):
            prim = stage.GetPrimAtPath(self._vehicleData.vehiclePath)
            if (prim):
                # hacked in here instead of the operation tracker since the meta data is deprecated and going away
                # in the future
                if (has_custom_metadata(prim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass)):
                    value = get_custom_metadata(prim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass)
                    do_dict = { self._vehicleData.vehiclePath: None }  # None means clear metadata
                    undo_dict = { self._vehicleData.vehiclePath: value }
                    SetCustomMetadataCommand.execute(stage, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, do_dict, undo_dict)
                else:
                    do_dict = { self._vehicleData.vehiclePath: None }
                    undo_dict = { self._vehicleData.vehiclePath: None }
                    SetCustomMetadataCommand.execute(stage, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, do_dict, undo_dict)

        return Wizard.create_vehicle(stage, self._vehicleData, self._operationTracker)

    def undo(self):
        context = omni.usd.get_context()
        stage = context.get_stage()

        operationTracker = self._operationTracker

        for collision in operationTracker.collisionGroupIncludeList:
            remove_collision_from_collision_group(stage, collision.path, collision.groupPath)

        DeletePrimsCommand(operationTracker.createdPrimPaths).do()


omni.kit.commands.register_all_commands_in_module(__name__)

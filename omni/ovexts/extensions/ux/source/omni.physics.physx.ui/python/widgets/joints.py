# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics import PhysicsWidget
from omni.kit.property.physics.widgets import ChildJointWidget
from omni.kit.property.physics import database
from pxr import UsdPhysics, PhysxSchema
from omni.kit.property.physics.utils import enable_widget, add_disabled_styles
from omni.physx.scripts.ifaces import get_physx_interface
from omni.physx.bindings._physx import SimulationEvent


class GearJointWidget(ChildJointWidget):
    def __init__(self, title, schema):
        super().__init__("Gear Joint", schema)


class RackAndPinionJoint(ChildJointWidget):
    def __init__(self, title, schema):
        super().__init__("Rack And Pinion Joint", schema)


class ExtendedMimicJointWidget(PhysicsWidget):

    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)

        if (prop.base_name == "referenceJoint"):
            def changed(model):
                self.request_rebuild()

            # changing the reference joint can change the decision whether the
            # reference joint axis attribute widget should be enabled/disabled.
            # Hence, a rebuild should get triggered by any change.
            model.value_model.add_value_changed_fn(changed)
            
        elif (prop.base_name == "referenceJointAxis"):
            # The reference joint axis attribute widget should be disabled if the reference
            # joint is a single degree of freedom joint (prismatic/revolute) because the
            # axis is implicitly defined in that case

            hasSingleDofRefJoints = False
            for primPath in prim_paths:
                prim = stage.GetPrimAtPath(primPath)
                mimicJoint = PhysxSchema.PhysxMimicJointAPI(prim, prop.instance_name)
                refJointRelTargets = mimicJoint.GetReferenceJointRel().GetTargets()
                if ((refJointRelTargets is not None) and (len(refJointRelTargets) > 0)):
                    refJointPath = refJointRelTargets[0]
                    refJointPrim = stage.GetPrimAtPath(refJointPath)
                    if (refJointPrim.IsValid() and (refJointPrim.IsA(UsdPhysics.PrismaticJoint) or refJointPrim.IsA(UsdPhysics.RevoluteJoint))):
                        hasSingleDofRefJoints = True
                        break

            if hasSingleDofRefJoints:
                add_disabled_styles(model.value_widget)
                enable_widget(model.value_widget, False)

        return model


class JointStateWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "JointState", "Joint State", None)

    def __init__(self, title, schema):
        super().__init__(title, schema)
        self._disabled = True
        self._pos_widget = None
        self._vel_widget = None
        physxInterface = get_physx_interface()
        self._simulationEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
            self._on_simulation_event
        )

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.STOPPED):
            if not self._disabled:
                self._disabled = True
                if self._pos_widget:
                    self._pos_widget.enabled = False
                    self._vel_widget.enabled = False
        elif event.type == int(SimulationEvent.RESUMED):
            if self._disabled:
                self._disabled = False
                if self._pos_widget:
                    self._pos_widget.enabled = True
                    self._vel_widget.enabled = True

    def build_impl(self):
        # fake a joint state component so we can produce an all-instance removing button
        self._build_impl_with_remove_button(JointStateWidget.fake_component)

    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physics:position":
            add_disabled_styles(model.value_widget)
            self._pos_widget = model.value_widget
            enable_widget(self._pos_widget, not self._disabled)
        elif prop.base_name == "physics:velocity":
            add_disabled_styles(model.value_widget)
            self._vel_widget = model.value_widget
            enable_widget(self._vel_widget, not self._disabled)
        return model


class JointAxisWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "PhysxJointAxis", "Properties", None)

    def build_impl(self):
        self._build_impl_with_remove_button(JointAxisWidget.fake_component)

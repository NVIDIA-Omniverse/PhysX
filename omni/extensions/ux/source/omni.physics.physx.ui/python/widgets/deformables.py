# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics import PhysicsWidget
from omni.kit.property.physics.widgets import UiProp, REFRESH_GLYPH
from omni.kit.property.physics.utils import enable_widget, add_disabled_styles
from omni.kit.property.physics.builders import UsdPropertiesWidgetBuilder
from pxr import Usd, Sdf, UsdPhysics, PhysxSchema
import omni.ui as ui
import enum
import omni.usd
from omni.physx.scripts import utils
from functools import partial
from omni.physx.scripts.ifaces import get_physx_attachment_private_interface
from omni.physxui import get_physxui_interface


class ExtendedDeformableBodyWidgetDeprecated(PhysicsWidget):
    is_kinematic_title = "Kinematic Enabled"
    sim_mesh_res_title = "Simulation Mesh Resolution"
    coll_simp_title = "Collision Mesh Simplification"
    coll_simp_remeshing_title = "Enable Remeshing"
    coll_simp_remeshing_res_title = "Remeshing Resolution"
    coll_simp_target_title = "Target Triangle Count"
    coll_simp_force_conforming_title = "Force Conforming"

    def __init__(self, title, schema):
        super().__init__(title, schema)
        self._kinematic_enabled = False

    def _set_widget_state(self):
        if self._sim_res_widget is not None:
            enable_widget(self._sim_res_widget, self._coll_simp_enabled or not self._kinematic_enabled)
        if self._coll_simp_remeshing_widget is not None:
            enable_widget(self._coll_simp_remeshing_widget, self._coll_simp_enabled and not self._kinematic_enabled)
        if self._coll_simp_remeshing_res_widget is not None:
            enable_widget(self._coll_simp_remeshing_res_widget, self._coll_simp_enabled and not self._kinematic_enabled)
            enable_widget(self._coll_simp_remeshing_res_sent_widget, self._coll_simp_enabled and not self._kinematic_enabled)
        if self._coll_simp_target_widget is not None:
            enable_widget(self._coll_simp_target_widget, self._coll_simp_enabled)
            enable_widget(self._coll_simp_target_sent_widget, self._coll_simp_enabled)
        if self._coll_simp_force_conforming_widget is not None:
            enable_widget(self._coll_simp_force_conforming_widget, self._coll_simp_enabled and not self._kinematic_enabled)

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        if prim.HasAttribute("physxDeformable:kinematicEnabled"):
            kinematicEnabled = prim.GetAttribute("physxDeformable:kinematicEnabled").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:kinematicEnabled",
                                  ExtendedDeformableBodyWidgetDeprecated.is_kinematic_title, "", 'bool', kinematicEnabled))
        if prim.HasAttribute("physxDeformable:simulationHexahedralResolution"):
            simulationHexahedralResolution = prim.GetAttribute("physxDeformable:simulationHexahedralResolution").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:simulationHexahedralResolution",
                                  ExtendedDeformableBodyWidgetDeprecated.sim_mesh_res_title, "", 'int', simulationHexahedralResolution))
        if prim.HasAttribute("physxDeformable:collisionSimplification"):
            collisionSimplification = prim.GetAttribute("physxDeformable:collisionSimplification").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplification",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_title, "", 'bool', collisionSimplification))
        if prim.HasAttribute("physxDeformable:collisionSimplificationRemeshing"):
            collisionSimplificationRemeshing = prim.GetAttribute("physxDeformable:collisionSimplificationRemeshing").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationRemeshing",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_remeshing_title, "", 'bool', collisionSimplificationRemeshing))
        if prim.HasAttribute("physxDeformable:collisionSimplificationRemeshingResolution"):
            collisionSimplificationRemeshingResolution = prim.GetAttribute("physxDeformable:collisionSimplificationRemeshingResolution").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationRemeshingResolution",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_remeshing_res_title, "", 'int', collisionSimplificationRemeshingResolution))
        if prim.HasAttribute("physxDeformable:collisionSimplificationTargetTriangleCount"):
            collisionSimplificationTargetTriangleCount = prim.GetAttribute("physxDeformable:collisionSimplificationTargetTriangleCount").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationTargetTriangleCount",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_target_title, "", 'int', collisionSimplificationTargetTriangleCount))
        if prim.HasAttribute("physxDeformable:collisionSimplificationForceConforming"):
            collisionSimplificationForceConforming = prim.GetAttribute("physxDeformable:collisionSimplificationForceConforming").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationForceConforming",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_force_conforming_title, "", 'bool', collisionSimplificationForceConforming))

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["physxDeformable:collisionSimplificationRemeshing",
                             "physxDeformable:collisionSimplificationRemeshingResolution",
                             "physxDeformable:collisionSimplificationTargetTriangleCount",
                             "physxDeformable:collisionSimplificationForceConforming"],
                            "Collision Mesh Simplification")

        add_props_to_groups(["physxDeformable:selfCollisionFilterDistance",
                             "physxDeformable:sleepThreshold",
                             "physxDeformable:settlingThreshold",
                             "physxDeformable:sleepDamping",
                             "physxDeformable:solverPositionIterationCount"],
                            "Advanced")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["physxDeformable:restPoints",
                      "physxDeformable:collisionPoints", "physxDeformable:collisionRestPoints", "physxDeformable:collisionIndices",
                      "physxDeformable:simulationPoints", "physxDeformable:simulationRestPoints", "physxDeformable:simulationIndices",
                      "physxDeformable:simulationVelocities",
                      "physxDeformable:disableGravity",
                      "physxCollision:torsionalPatchRadius", "physxCollision:minTorsionalPatchRadius"])

        return filtered_props

    def build_items(self):
        self._sim_res_widget = None
        self._coll_simp_remeshing_widget = None
        self._coll_simp_remeshing_res_widget = None
        self._coll_simp_remeshing_res_sent_widget = None
        self._coll_simp_target_widget = None
        self._coll_simp_target_sent_widget = None
        self._coll_simp_force_conforming_widget = None
        super().build_items()
        self._set_widget_state()

    def _build_property_item(self, stage, prop, prim_paths):

        def changed_coll_simp(model):
            if self._coll_simp_enabled != model.as_bool:
                self._coll_simp_enabled = model.as_bool
                self._set_widget_state()

        def changed_kinematic(model):
            if self._kinematic_enabled != model.as_bool:
                self._kinematic_enabled = model.as_bool
                self._set_widget_state()

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physxDeformable:kinematicEnabled":
            model.add_value_changed_fn(changed_kinematic)
            self._kinematic_enabled = model.as_bool
        if prop.base_name == "physxDeformable:simulationHexahedralResolution":
            if not model._ambiguous and (model.as_int < prop.info.min or model.as_int > prop.info.max):
                with ui.HStack():
                    UsdPropertiesWidgetBuilder._create_label(ExtendedDeformableBodyWidgetDeprecated.sim_mesh_res_title)
                    ui.Spacer(width=8)
                    text = "Not available" if model.as_int <= 0 else f"{model.as_int}"
                    ui.Label(text, name="label")
            add_disabled_styles(model.value_widget)
            self._sim_res_widget = model.value_widget
        if prop.base_name == "physxDeformable:collisionSimplification":
            model.add_value_changed_fn(changed_coll_simp)
            self._coll_simp_enabled = model.as_bool
        if prop.base_name == "physxDeformable:collisionSimplificationRemeshing":
            add_disabled_styles(model.value_widget)
            self._coll_simp_remeshing_widget = model.value_widget
        if prop.base_name == "physxDeformable:collisionSimplificationRemeshingResolution":
            add_disabled_styles(model.value_widget)
            self._coll_simp_remeshing_res_widget = model.value_widget
            self._coll_simp_remeshing_res_sent_widget = model.sentinel_widget
        if prop.base_name == "physxDeformable:collisionSimplificationTargetTriangleCount":
            add_disabled_styles(model.value_widget)
            self._coll_simp_target_widget = model.value_widget
            self._coll_simp_target_sent_widget = model.sentinel_widget
        if prop.base_name == "physxDeformable:collisionSimplificationForceConforming":
            add_disabled_styles(model.value_widget)
            self._coll_simp_force_conforming_widget = model.value_widget

        return model


class ExtendedDeformableSurfaceWidgetDeprecated(PhysicsWidget):
    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["physxDeformable:sleepThreshold",
                             "physxDeformable:settlingThreshold",
                             "physxDeformable:sleepDamping",
                             "physxDeformable:solverPositionIterationCount"],
                             "Advanced")

        add_props_to_groups(["physxCollision:contactOffset",
                             "physxCollision:restOffset",
                             "physxDeformable:selfCollisionFilterDistance",
                             "physxDeformableSurface:collisionPairUpdateFrequency",
                             "physxDeformableSurface:collisionIterationMultiplier",
                             "physxDeformable:maxDepenetrationVelocity"],
                             "Collision")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["physxDeformable:enableCCD", "physxCollision:torsionalPatchRadius",
                      "physxCollision:minTorsionalPatchRadius", "physxCollision:maxTorsionalPatchRadius",
                      "physxDeformableSurface:bendingStiffnessScale",
                      "physxDeformable:restPoints", "physxDeformable:simulationIndices",
                      "physxDeformable:simulationVelocities"])

        return filtered_props


class ExtendedDeformalbeSurfaceMaterialWidgetDeprecated(PhysicsWidget):

    prop_data_custom = {
        "physxDeformableSurfaceMaterial:bendDamping": ("Bend Damping", "", 'float', 0.0, "The amount of damping that gets applied to bending motions."),
        "physxDeformableSurfaceMaterial:elasticityDamping": ("Elasticity Damping", "", 'float', 0.0, "The amount of damping that gets applied to stretching motions."),
        "physxDeformableSurfaceMaterial:bendStiffness": ("Bend Stiffness", "", 'float', 0.0, "The bending stiffness defines the surface's resistance to bending."),
    }

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        for prop_name, (disp_name, disp_group, prop_type, default_val, docs) in self.prop_data_custom.items():
            if prim.HasAttribute(prop_name):
                prop = UiProp().from_custom(prop_name, disp_name, disp_group, prop_type, default_val, docs)
                filtered_props.append(prop)

        return filtered_props

    def build_items(self):
        super().build_items()

    def _build_property_item(self, stage, prop, prim_paths):
        model = super()._build_property_item(stage, prop, prim_paths)
        return model


class ExtendedDeformableBodyWidget(PhysicsWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)
        self._surface_sim_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsSurfaceDeformableSimAPI")
        self._volume_sim_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsVolumeDeformableSimAPI")
        self._ext_surface_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("PhysxSurfaceDeformableBodyAPI")
        self._ext_volume_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("PhysxBaseDeformableBodyAPI")

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #figure out whether this is for surface or volume deformable
        is_surface = False
        is_volume = False

        if prim.HasAPI(self._surface_sim_type):
            is_surface = True
        if prim.HasAPI(self._volume_sim_type):
            is_volume = True

        children = prim.GetChildren()
        for child in children:
            if child.HasAPI(self._surface_sim_type):
                is_surface = True
            if child.HasAPI(self._volume_sim_type):
                is_volume = True

        if is_surface == is_volume:
            return []

        ext_type = self._ext_surface_type if is_surface else self._ext_volume_type
        adjusted_filtered_props = []
        for prop in filtered_props:
            if not prop.apply_schema or (prop.apply_schema == ext_type):
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props

class ExtendedSurfaceDeformableMaterialWidget(PhysicsWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)
        self.base_mat_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsDeformableMaterialAPI")
        self.ext_base_mat_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("PhysxDeformableMaterialAPI")

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        # remove properties from UsdPhysics.DeformableMaterialAPI and PhysxSchema.PhysxDeformableMaterialAPI
        deformable_base_mat_props = utils.getSchemaPropertyNames(self.base_mat_type)
        deformable_base_mat_props.extend(utils.getSchemaPropertyNames(self.ext_base_mat_type))
        adjusted_filtered_props = []
        for prop in filtered_props:
            if prop.base_name in deformable_base_mat_props:
                pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedTetrahedralMeshWidgetDeprecated(PhysicsWidget):
    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        return filtered_props


class ExtendedAttachmentWidgetDeprecated(PhysicsWidget):
    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["collisionFilterIndices0",
                      "collisionFilterIndices1",
                      "filterType0",
                      "filterType1",
                      "pointsActor0",
                      "pointsActor1"])

        return filtered_props


class ExtendedComputeAutoAttachmentWidgetDeprecated(PhysicsWidget):

    prop_data_schema = {
        "physxAutoAttachment:enableDeformableVertexAttachments": ("Attach Overlapping Vertices", "", 'bool', "Enables vertex based attachments"),
        "physxAutoAttachment:deformableVertexOverlapOffset": ("Overlap Offset", "", 'float', "Defines at which distance attachments are created"),
        "physxAutoAttachment:enableRigidSurfaceAttachments": ("Attach Rigid Surface", "", 'bool', "Enables creating attachments on rigid surface"),
        "physxAutoAttachment:rigidSurfaceSamplingDistance": ("Surface Sampling Distance", "", 'float', "Defines distance at which attachments are created on rigid surface"),
        "physxAutoAttachment:enableCollisionFiltering": ("Collision Filtering", "", 'bool', "Enables collision filtering"),
        "physxAutoAttachment:collisionFilteringOffset": ("Filtering Offset", "", 'float', "Defines at which distance elements are being filtered"),
    }

    prop_data_custom = {
        "physxAutoAttachment:maskShapes": ("Mask Shapes", "", '', None, "The union of the shapes defines a mask volume within which attachment points are generated"),
    }

    class AType(enum.Enum):
        WORLD = 0
        RIGID = 1
        DEFORMABLE_BODY = 2
        DEFORMABLE_SURFACE = 3
        PARTICLES = 4

    def is_deformableVertexOverlapOffset_enabled(self):
        return self._bool_controllers["physxAutoAttachment:enableDeformableVertexAttachments"]

    def is_enableRigidSurfaceAttachments_enabled(self):
        return self._all_have_type[self.AType.RIGID]

    def is_rigidSurfaceSamplingDistance_enabled(self):
        return self._bool_controllers["physxAutoAttachment:enableRigidSurfaceAttachments"] and self._all_have_type[self.AType.RIGID]

    def is_collisionFilteringOffset_enabled(self):
        return self._bool_controllers["physxAutoAttachment:enableCollisionFiltering"] and self._none_have_type[self.AType.WORLD]

    def _update_types(self):
        self._all_have_type = {type_key: True for type_key in self.AType}
        self._none_have_type = {type_key: True for type_key in self.AType}

        stage = omni.usd.get_context().get_stage()
        for prim_path in self._payload:
            has_type = {type_key: False for type_key in self.AType}
            attachment = PhysxSchema.PhysxPhysicsAttachment(self._get_prim(prim_path))
            if attachment:
                for t in range(2):
                    targets = attachment.GetActorRel(t).GetTargets()
                    if len(targets) > 0:
                        target_prim = stage.GetPrimAtPath(targets[0])
                        if target_prim:
                            has_type[self.AType.RIGID] = has_type[self.AType.RIGID] or target_prim.HasAPI(UsdPhysics.CollisionAPI) or target_prim.HasAPI(UsdPhysics.RigidBodyAPI)
                            has_type[self.AType.DEFORMABLE_BODY] = has_type[self.AType.DEFORMABLE_BODY] or target_prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI)
                            has_type[self.AType.DEFORMABLE_SURFACE] = has_type[self.AType.DEFORMABLE_SURFACE] or target_prim.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI)
                            has_type[self.AType.PARTICLES] = has_type[self.AType.PARTICLES] or target_prim.HasAPI(PhysxSchema.PhysxParticleAPI)
                    else:
                        has_type[self.AType.WORLD] = True

            for type_key in self.AType:
                self._all_have_type[type_key] = self._all_have_type[type_key] and has_type[type_key]
                self._none_have_type[type_key] = self._none_have_type[type_key] and not has_type[type_key]


    #only called once for all instances
    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        self._payload = payload
        self._update_types()

        return True

    def _set_widget_state(self):

        for prop_name, (widget, is_enabled_fn_name) in self._state_controlled.items():
            if widget is not None and is_enabled_fn_name is not None:
                is_enabled_fn = getattr(self, is_enabled_fn_name)
                enable_widget(widget, is_enabled_fn())

    def _filter_props_to_build(self, prim):

        #create custom attributes in case they are missing
        maskShapes_rel = prim.GetRelationship("physxAutoAttachment:maskShapes")
        if not maskShapes_rel:
            prim.CreateRelationship("physxAutoAttachment:maskShapes", True)

        filtered_props = super()._filter_props_to_build(prim)

        for prop_name, (disp_name, disp_group, prop_type, docs) in self.prop_data_schema.items():
            if prim.HasAttribute(prop_name) or prim.HasRelationship(prop_name):
                prim_def = utils.getSchemaPrimDef(self._main_schema)
                prop_spec = prim_def.GetSchemaPropertySpec(prop_name)
                prop = UiProp().from_custom(prop_name, disp_name, disp_group, prop_type, prop_spec.default, docs)
                filtered_props.append(prop)

        for prop_name, (disp_name, disp_group, prop_type, default_val, docs) in self.prop_data_custom.items():
            if prim.HasAttribute(prop_name) or prim.HasRelationship(prop_name):
                prop = UiProp().from_custom(prop_name, disp_name, disp_group, prop_type, default_val, docs)
                filtered_props.append(prop)

        return filtered_props

    def build_items(self):

        self._bool_controllers = {}
        self._state_controlled = {}

        super().build_items()

        def on_click():
            stage = omni.usd.get_context().get_stage()
            physxui_interface = get_physxui_interface()
            if stage is None or physxui_interface is None:
                return

            for prim_path in self._payload:
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid() and prim.IsA(PhysxSchema.PhysxPhysicsAttachment):
                    physxui_interface.refresh_attachment(prim_path.pathString)

        ui.Button(f"{REFRESH_GLYPH}", clicked_fn=on_click)

        # 'luckily' changing the attachment actor rels does triggers build_items,
        # so we don't need a callback
        self._update_types()
        self._set_widget_state()

    def _make_bool_controller(self, prop_name, model):

        def changed_bool(prop_name, model):
            if self._bool_controllers[prop_name] != model.as_bool:
                self._bool_controllers[prop_name] = model.as_bool
                self._set_widget_state()

        self._bool_controllers[prop_name] = model.as_bool and not model._ambiguous
        model.add_value_changed_fn(partial(changed_bool, prop_name))

    def _make_state_controlled(self, prop_name, model, is_enabled_fn_name):
            add_disabled_styles(model.value_widget)
            self._state_controlled[prop_name] = (model.value_widget, is_enabled_fn_name)

    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)

        if prop.base_name == "physxAutoAttachment:enableDeformableVertexAttachments":
            self._make_bool_controller(prop.base_name, model)

        if prop.base_name == "physxAutoAttachment:deformableVertexOverlapOffset":
            self._make_state_controlled(prop.base_name, model, "is_deformableVertexOverlapOffset_enabled")

        if prop.base_name == "physxAutoAttachment:enableRigidSurfaceAttachments":
            self._make_bool_controller(prop.base_name, model)
            self._make_state_controlled(prop.base_name, model, "is_enableRigidSurfaceAttachments_enabled")

        if prop.base_name == "physxAutoAttachment:rigidSurfaceSamplingDistance":
            self._make_state_controlled(prop.base_name, model, "is_rigidSurfaceSamplingDistance_enabled")

        if prop.base_name == "physxAutoAttachment:enableCollisionFiltering":
            self._make_bool_controller(prop.base_name, model)

        if prop.base_name == "physxAutoAttachment:collisionFilteringOffset":
            self._make_state_controlled(prop.base_name, model, "is_collisionFilteringOffset_enabled")

        return model


class ExtendedAutoDeformableAttachmentWidget(PhysicsWidget):

    prop_data_schema = {
        "physxAutoDeformableAttachment:enableDeformableVertexAttachments": ("Attach Overlapping Vertices", "", 'bool', "Enables vertex based attachments"),
        "physxAutoDeformableAttachment:deformableVertexOverlapOffset": ("Overlap Offset", "", 'float', "Defines at which distance attachments are created"),
        "physxAutoDeformableAttachment:enableRigidSurfaceAttachments": ("Attach Rigid Surface", "", 'bool', "Enables creating attachments on rigid surface"),
        "physxAutoDeformableAttachment:rigidSurfaceSamplingDistance": ("Surface Sampling Distance", "", 'float', "Defines distance at which attachments are created on rigid surface"),
        "physxAutoDeformableAttachment:enableCollisionFiltering": ("Collision Filtering", "", 'bool', "Enables collision filtering"),
        "physxAutoDeformableAttachment:collisionFilteringOffset": ("Filtering Offset", "", 'float', "Defines at which distance elements are being filtered"),
    }

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        filtered_props_mod = []

        for prop in filtered_props:
            if prop.base_name == "physxAutoDeformableAttachment:enableRigidSurfaceAttachments":
                pass
            elif prop.base_name == "physxAutoDeformableAttachment:rigidSurfaceSamplingDistance":
                pass
            elif prop.base_name in self.prop_data_schema:
                (disp_name, disp_group, prop_type, docs) = self.prop_data_schema[prop.base_name]
                prim_def = utils.getSchemaPrimDef(self._main_schema)
                prop_spec = prim_def.GetSchemaPropertySpec(prop.base_name)
                prop_mod = UiProp().from_custom(prop.base_name, disp_name, disp_group, prop_type, prop_spec.default, docs)
                filtered_props_mod.append(prop_mod)
            else:
                filtered_props_mod.append(prop)

        return filtered_props_mod

    def build_items(self):

        super().build_items()

        def on_click():
            stage = omni.usd.get_context().get_stage()
            physxui_interface = get_physxui_interface()
            physx_attachment_private_interface = get_physx_attachment_private_interface()
            if stage is None or physxui_interface is None:
                return

            for prim_path in self._payload:
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid():
                    physx_attachment_private_interface.setup_auto_deformable_attachment(prim_path.pathString)
                    physxui_interface.refresh_attachment(prim_path.pathString)

        ui.Button(f"{REFRESH_GLYPH}", clicked_fn=on_click)


    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)
        return model


class ExtendedAttachmentElementFilterWidget(PhysicsWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)

    def _filter_props_to_build(self, prim):
        filtered_props = [p for p in super()._filter_props_to_build(prim) if p.base_name.startswith("physics:")]
        return filtered_props

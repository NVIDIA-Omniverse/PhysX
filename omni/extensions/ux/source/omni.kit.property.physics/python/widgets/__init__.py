# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .mainframe import MainFrameWidget
from .invisible import InvisibleWidget, InvisibleMenuWidgetBase
from .base import PhysicsWidget, ExtensionSchemaWidget
from .custom_properties import CustomPropertiesWidget
from .uiprop import UiProp
from .utils import *

from pxr import UsdPhysics, Usd, Sdf, UsdGeom, PhysxSchema
import omni.ui as ui
import omni.kit.undo
import omni.physx.bindings._physx as pxb
from omni.kit.window.property.templates import SimplePropertyWidget, build_frame_header, HORIZONTAL_SPACING
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
from omni.kit.property.material.scripts.usd_binding_widget import UsdBindingAttributeWidget
from omni.kit.material.library import MaterialUtils
from omni.physx.scripts import utils
from omni.physx.scripts.utils import get_schema_instances
from omni.physxui import get_physxui_interface
from . import database
from ..utils import get_align_property_util, get_schema_name



class MassAPIWidget(PhysicsWidget):
    def _show_remove_button(self):
        return self._schema_present

    def build_items(self):
        if self._schema_present:
            super().build_items()
        else:
            ui.Label("Mass component is not present, computing mass properties from collision volumes and density.", name="label")

    def on_new_payload(self, payload):
        self._payload = payload

        if not payload or len(payload) == 0:
            return False

        allHaveRigidBody = True

        for prim_path in payload:
            prim = self._get_prim(prim_path)

            if not prim:
                continue

            # DEPRECATED
            hasDeformableBody = prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI)
            hasDeformableSurface = prim.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI)
            #~DEPRECATED
            hasParticle = prim.HasAPI(PhysxSchema.PhysxParticleAPI)
            hasRigidBody = prim.HasAPI(UsdPhysics.RigidBodyAPI)
            allHaveRigidBody = allHaveRigidBody and hasRigidBody

            # don't show if there's at least one prim that doesn't have either rigidbody or a collision
            if not hasRigidBody and not prim.HasAPI(UsdPhysics.CollisionAPI) and not hasDeformableBody and not hasDeformableSurface and not hasParticle:
                return False

        self._schema_present = super().on_new_payload(payload)

        # we want to show the autocompute message only when there's no schema and all prims have a rigidbody
        if not self._schema_present and not allHaveRigidBody:
            return False

        return True


class JointWidget(PhysicsWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)

        self._utils = dict()

        # fixed joint util
        fixed_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.FixedJoint)
        u0 = get_align_property_util("Transform", False, self, True, True)
        u1 = get_align_property_util("Transform", True, self, True, True)
        self._utils[fixed_joint_type_name] = {
            "physics:localPos0": u0,
            "physics:localPos1": u1,
            "physics:localRot0": u0,
            "physics:localRot1": u1,
        }

        # revolute joint util
        revolute_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.RevoluteJoint)
        self._utils[revolute_joint_type_name] = {
            "physics:localPos0": get_align_property_util("Position", False, self, True, False),
            "physics:localPos1": get_align_property_util("Position", True, self, True, False),
        }

        # prismatic joint util
        prismatic_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.PrismaticJoint)
        u0 = get_align_property_util("Transform", False, self, True, True)
        u1 = get_align_property_util("Transform", True, self, True, True)
        self._utils[prismatic_joint_type_name] = {
            "physics:localPos0": u0,
            "physics:localPos1": u1,
            "physics:localRot0": u0,
            "physics:localRot1": u1,
        }

        # spherical joint util
        spherical_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.SphericalJoint)
        self._utils[spherical_joint_type_name] = {
            "physics:localPos0": get_align_property_util("Position", False, self, True, False),
            "physics:localPos1": get_align_property_util("Position", True, self, True, False),
        }

    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        last_type = None
        all_types_same = True

        for prim_path in payload:
            prim = self._get_prim(prim_path)

            if not prim:
                return False

            # all types the same check so that we can show the utility
            if all_types_same:
                curr_type = prim.GetTypeName()
                if last_type is not None and curr_type != last_type:
                    all_types_same = False
                last_type = curr_type

        self._property_utils = dict()

        if all_types_same:
            self._property_utils = self._utils.get(last_type, dict())
        return True


class ChildJointWidget(PhysicsWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)
        self._joint_names = set(utils.getSchemaPrimDef(UsdPhysics.Joint).GetPropertyNames())
        self._joint_names = self._joint_names.union(set(utils.getSchemaPrimDef(PhysxSchema.PhysxJointAPI).GetPropertyNames()))

    def _filter_props_to_build(self, prim):
        filtered_props = [p for p in super()._filter_props_to_build(prim) if p.base_name not in self._joint_names]
        return filtered_props


class LimitWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "PhysicsLimit", "Limit", None)

    def build_impl(self):
        # fake a limit component so we can produce an all-instance removing button
        self._build_impl_with_remove_button(LimitWidget.fake_component)


class DriveWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "PhysicsDrive", "Drive", None)
    advanced_properties = set(database.get_property_order('PhysxDrivePerformanceEnvelopeAPI')) 

    def build_impl(self):
        # fake a drive component so we can produce an all-instance removing button
        self._build_impl_with_remove_button(DriveWidget.fake_component)

    def _build_group_frame(self, name, props, stage):

        if len(props) == 0:
            return None
        
        is_instance = props[0].instance_name is not None

        def get_title():
            if not is_instance:
                return display_name_from_name(name)
            if self._main_schema in database.display_raw_instance_name_apis:
                return name
            return database.instance_display_names.get(name, display_name_from_name(name))
        
        def build_instance_frame(component=None):

            collapsed = not is_instance
            parent_frame = ui.CollapsableFrame(
                title=get_title(), 
                build_header_fn=build_frame_header,
                name="parentFrame"
            )
            
            basic_props = [prop for prop in props if prop.base_name not in self.advanced_properties]
            advanced_props = [prop for prop in props if prop.base_name in self.advanced_properties]
            with parent_frame:
                with ui.HStack():
                    with ui.VStack(height=0, spacing=5, name="props_stack"):
                        for prop in basic_props:
                                self.build_property_item(stage, prop, self._payload)
                                collapsed |= prop.display_group_collapsed
                        if component is not None:
                            ui.Button(
                                style=REMOVE_BUTTON_STYLE,
                                clicked_fn=lambda: self._remove_prompt_check(
                                    component.title,
                                    lambda: remove_component(component, self._payload, props[0].instance_name)
                                ),
                                width=16,
                                tooltip="Remove Component",
                                identifier=f"remove_component_{component.title}"
                            )
                
                        child_frame = ui.CollapsableFrame(
                            title="Advanced",  
                            build_header_fn=build_frame_header,
                            name="childFrame",
                            collapsed=True
                        )
                        
                        with child_frame:
                            with ui.HStack():
                                with ui.VStack(height=0, spacing=5, name="props_stack"):
                                        for prop in advanced_props:
                                                self.build_property_item(stage, prop, self._payload)

            parent_frame.collapsed = collapsed
            return parent_frame


        if self._is_main_multi:
            instance_name = f"{get_schema_name(self._main_schema)}:{props[0].instance_name}"
            component = database.components.get(instance_name)
            if component is not None:
                return build_instance_frame(component)
            else:
                component = database.components.get(get_schema_name(self._main_schema))
                return build_instance_frame(component)
        else:
            return build_instance_frame()
  

class PhysxLimitExtJointWidget(ChildJointWidget):
    def __init__(self, title, schema, limit_instance):
        super().__init__(title, schema)

        self._limit_api = PhysxSchema.PhysxLimitAPI
        self._limit_instance = limit_instance
        self._limit_type_name = Usd.SchemaRegistry().GetSchemaTypeName(self._limit_api)
        self._limit_prefix = database.ext_multi_api_prefixes[self._limit_api]
        self._limit_prop_names = [(name, f"{self._limit_prefix}:{limit_instance}:{name}") for name in utils.getSchemaPropertyNames(self._limit_api)]
        prim_def = utils.getSchemaPrimDef(self._limit_api)
        self._limit_prop_specs = [prim_def.GetSchemaPropertySpec(name) for name in prim_def.GetPropertyNames()]

    def _filter_props_to_build(self, prim):
        filtered_props = []
        instances = get_schema_instances(prim, self._limit_type_name)

        if self._limit_instance in instances:
            for base_name, full_name in self._limit_prop_names:
                prop = prim.GetProperty(full_name)
                filtered_props.append(UiProp().from_property(prop, base_name, self._limit_instance))
        else:
            for prop_spec in self._limit_prop_specs:
                filtered_props.append(UiProp().from_property_spec(prop_spec, prim, self._limit_api, self._limit_instance, self._limit_prefix))

        for prop in filtered_props:
            prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = "Limit Configuration"
            prop.display_group_collapsed = True

        return super()._filter_props_to_build(prim) + filtered_props


class FixedJointWidget(ChildJointWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)

    def on_new_payload(self, payload):
        # Fixed joint now only inherits from Joint and has no properties itself
        return False


class RevoluteJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema, "angular")


class PrismaticJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema, "linear")


class DistanceJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema, "distance")


class SphericalJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema, "cone")


class LocalSpaceVelocitiesWidget(PhysicsWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)
        self.localSpaceVelocityProp = UiProp().from_custom(
            pxb.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES,
            "Velocities in Local Space",
            "",
            'bool',
            False,
            """If checked, both Linear and Angular Velocities
        are stored in Local Space, otherwise in Global Space.
        This value is by default inferred from global Physics Settings,
        but you have the option to override the value here."""
        )

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        filtered_props.append(self.localSpaceVelocityProp)
        return filtered_props


class ExtendedColliderWidget(PhysicsWidget):
    margin_title = "Margin"

    def __init__(self, title, schema):
        super().__init__(title, schema)
        self._min_model = None
        self._max_model = None
        self._deformable_b_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsDeformableBodyAPI")
        self._deformable_b_name = "omniphysics:deformableBodyEnabled"
        self._deformable_ignore_props = {
            "physics:approximation",
            "physics:simulationOwner",
            "physxCollision:torsionalPatchRadius",
            "physxCollision:minTorsionalPatchRadius",
        }

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def is_part_of_deformable(prim):
            if not prim.IsA(UsdGeom.Mesh) and not prim.IsA(UsdGeom.TetMesh):
                return False
            while prim:
                rigid_body = UsdPhysics.RigidBodyAPI(prim)
                if rigid_body and rigid_body.GetRigidBodyEnabledAttr().Get():
                    return False
                if prim.HasAPI(self._deformable_b_type) and prim.GetAttribute(self._deformable_b_name).Get():
                    return True
                xformable = UsdGeom.Xformable(prim)
                if xformable and xformable.GetResetXformStack():
                    return False
                prim = prim.GetParent()

        def try_inactivate(approx_api, approx_name, curr_approx_name):
            for prop_spec in self._int_ext_prop_specs[approx_api]:
                prop = filtered_props_dict.get(prop_spec.name)
                if prop is not None:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = "Inactive"
                    prop.display_group_collapsed = True

        if is_part_of_deformable(prim):
            filtered_props_mod = []
            for prop in filtered_props:
                if not prop.base_name in self._deformable_ignore_props:
                    filtered_props_mod.append(prop)
            return filtered_props_mod

        filtered_props_dict = {prop.base_name: prop for prop in filtered_props}
        mesh_api = UsdPhysics.MeshCollisionAPI(prim)
        if mesh_api:
            curr_approx_name = mesh_api.GetApproximationAttr().Get()
            curr_approx_api = utils.MESH_APPROXIMATIONS.get(curr_approx_name)
            for approx_name, approx_api in utils.MESH_APPROXIMATIONS.items():
                if approx_api is not None and curr_approx_api != approx_api:
                    try_inactivate(approx_api, approx_name, curr_approx_name)

        if prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Cone):
            prop = UiProp().from_custom("physxConvexGeometry:margin", ExtendedColliderWidget.margin_title, "", 'float', 0)
            prop.display_group = "Advanced"
            prop.display_group_collapsed = True
            filtered_props.append(prop)

        return filtered_props

    def _build_property_item(self, stage, prop, prim_paths):
        def on_approximation_changed(model, _):
            new_api_str = str(model._allowed_tokens[model._current_index.as_int].token)
            new_api = utils.MESH_APPROXIMATIONS.get(new_api_str, None)
            if new_api is not None:
                for prim in generate_prims(prim_paths.get_paths()):
                    new_api.Apply(prim)
            self.request_rebuild()

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physics:approximation":
            model.add_item_changed_fn(on_approximation_changed)
        return model


class JointVisualizationWidget(SimplePropertyWidget):
    name = "physx_joint_visualization"

    def __init__(self):
        super().__init__("Joint Edit Mode", False)

    def on_new_payload(self, payload):
        # disabling this panel for now
        # we don't have enough data to support local pose editing for bodies
        # and world pose editing for the joint itself isn't meaningful
        return False

        if not super().on_new_payload(payload):
            return False

        if not payload or len(payload) != 1:
            return False

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False

        path = str(payload[0])
        return get_physxui_interface().show_joint_edit_mode_ui(path)

    def is_visible(self):
        return True

    def refresh_apis(self):
        pass

    def build_items(self):
        edit_local_pose = False
        for path in self._payload:
            edit_local_pose = get_physxui_interface().get_joint_edit_local_pose_mode(str(path))

        def joint_edit_local_pose_changed(model):
            for path in self._payload:
                get_physxui_interface().set_joint_edit_local_pose_mode(str(path), model.as_bool)

        with ui.VStack():
            with ui.HStack(height=HORIZONTAL_SPACING):
                UsdPropertiesWidgetBuilder._create_label(
                    "Edit Local Pose Transform", {},
                    {
                        "tooltip": (
                            "Enabled: changes in transform will update 'local position/orientation' "
                            "data for the joint, which is used to cacluate the limits of the joint\n"
                            "Disabled: changes in transform will update the starting position of the body "
                            "without changing the limits of the joint (ex: open a drawer slightly)"
                        )
                    }
                )
                self._joint_model = ui.SimpleBoolModel(edit_local_pose)
                ui.CheckBox(self._joint_model, height=25)
                self._joint_model.add_value_changed_fn(joint_edit_local_pose_changed)


class PhysicsMaterialBindingWidget(UsdBindingAttributeWidget):
    name = "physx_material_binding"
    title = "Physics materials on selected models"

    def __init__(self):
        self._material_utils = MaterialUtils()

        super().__init__(
            title=PhysicsMaterialBindingWidget.title,
            material_purpose="physics",
            filter_fn=lambda p: database.has_any_material_api(p),
            get_materials_async_fn=self._material_utils.get_materials_from_stage_async
        )

    def is_visible(self):
        return True

    def refresh_apis(self):
        pass

    def on_new_payload(self, payload):
        ret = super().on_new_payload(payload)

        # handle particle system before because PS is not an UsdGeom.Gprim, Xform or Subset thus the call to super() will return False
        for path in payload:
            stage = payload.get_stage()
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsA(PhysxSchema.PhysxParticleSystem):
                return True

        if not ret:
            return False

        for path in payload:
            prim = self._get_prim(path)
            if (not prim or (
                not prim.HasAPI(UsdPhysics.CollisionAPI) and
                # DEPRECATED
                not prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI) and 
                not prim.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI) and
                #~DEPRECATED
                not prim.HasAPI("OmniPhysicsDeformableBodyAPI") and
                not prim.HasAPI("OmniPhysicsVolumeDeformableSimAPI") and
                not prim.HasAPI("OmniPhysicsSurfaceDeformableSimAPI")
                )):
                return False

        return True


class PhysicsDefaultMaterialBindingWidget(UsdBindingAttributeWidget):
    name = "physx_default_material_binding"
    title = "Physics default material"

    def __init__(self):
        self._material_utils = MaterialUtils()

        super().__init__(
            title=PhysicsDefaultMaterialBindingWidget.title,
            material_purpose="physics",
            filter_fn=lambda p: database.has_any_material_api(p),
            get_materials_async_fn=self._material_utils.get_materials_from_stage_async
        )

    def refresh_apis(self):
        pass

    def is_visible(self):
        return True

    def on_new_payload(self, payload):
        if not SimplePropertyWidget.on_new_payload(self, payload):
            return False

        if len(payload) == 0:
            return False

        for path in payload:
            prim = self._get_prim(path)
            if not prim or (not prim.IsA(UsdPhysics.Scene)):
                return False

        return True

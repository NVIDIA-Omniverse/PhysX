# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics import PhysicsWidget
from omni.kit.property.physics.widgets import UiProp, REFRESH_GLYPH
from omni.kit.property.physics.utils import enable_widget, add_disabled_styles
from omni.kit.property.physics.builders import UsdPropertiesWidgetBuilder
from pxr import Usd, Sdf, UsdPhysics, PhysxSchema
import omni.ui as ui
import omni.usd
from omni.physx.scripts import utils
from omni.physx.scripts.ifaces import get_physx_attachment_private_interface
from omni.physxui import get_physxui_interface


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

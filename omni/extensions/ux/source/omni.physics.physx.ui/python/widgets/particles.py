# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics import PhysicsWidget
from pxr import UsdGeom, Sdf
from omni.kit.property.physics.widgets import UiProp


class ExtendedParticleClothWidgetDeprecated(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["physxParticle:selfCollisionFilter",
                             "physxParticle:particleGroup",
                             "physxDeformable:settlingThreshold",
                             "physxDeformable:sleepDamping",
                             "physxDeformable:solverPositionIterationCount"],
                             "Advanced")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["physxParticle:restPoints",
                      "physxParticle:springIndices",
                      "physxParticle:springStiffnesses",
                      "physxParticle:springDampings",
                      "physxParticle:springRestLengths"])

        return filtered_props


class ExtendedAutoParticleClothWidgetDeprecated(PhysicsWidget):

    springStretchStiffness_title = "Stretch Stiffness"
    springBendStiffness_title = "Bend Stiffness"
    springShearStiffness_title = "Shear Stiffness"
    springDamping_title = "Spring Damping"

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        if prim.HasAttribute("physxParticle:springStretchStiffness"):
            filtered_props.append(UiProp().from_custom("physxParticle:springStretchStiffness",
                                  ExtendedAutoParticleClothWidgetDeprecated.springStretchStiffness_title, "", 'float', True))

        if prim.HasAttribute("physxParticle:springBendStiffness"):
            filtered_props.append(UiProp().from_custom("physxParticle:springBendStiffness",
                                  ExtendedAutoParticleClothWidgetDeprecated.springBendStiffness_title, "", 'float', True))

        if prim.HasAttribute("physxParticle:springShearStiffness"):
            filtered_props.append(UiProp().from_custom("physxParticle:springShearStiffness",
                                  ExtendedAutoParticleClothWidgetDeprecated.springShearStiffness_title, "", 'float', True))

        if prim.HasAttribute("physxParticle:springDamping"):
            filtered_props.append(UiProp().from_custom("physxParticle:springDamping",
                                  ExtendedAutoParticleClothWidgetDeprecated.springDamping_title, "", 'float', True))

        return filtered_props

    def build_items(self):
        super().build_items()

    def _build_property_item(self, stage, prop, prim_paths):
        model = super()._build_property_item(stage, prop, prim_paths)
        return model


class ExtendedParticleSystemWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["contactOffset",
                             "restOffset",
                             "solidRestOffset",
                             "fluidRestOffset"],
                             "Advanced")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        gprim_props = UsdGeom.Gprim.GetSchemaAttributeNames()
        gprim_props.append('proxyPrim')
        gprim_props.append('globalSelfCollisionEnabled')
        gprim_props.append('nonParticleCollisionEnabled')
        remove_props(gprim_props)

        return filtered_props

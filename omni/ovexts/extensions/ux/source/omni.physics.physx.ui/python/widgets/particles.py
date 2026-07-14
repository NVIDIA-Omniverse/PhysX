# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics import PhysicsWidget
from pxr import UsdGeom, Sdf


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

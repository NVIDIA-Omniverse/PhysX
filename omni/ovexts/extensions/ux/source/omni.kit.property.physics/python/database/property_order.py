# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import UsdPhysics, PhysxSchema, Usd
from collections import defaultdict

_property_orders = defaultdict(dict)

def get_property_order(schema):
    for parent_schema, orders in _property_orders.items():
        if schema in orders:
            return orders[schema]
    return []
    
def add_property_orders(parent_schema, orders):
    _property_orders[parent_schema] = orders

def remove_property_orders(parent_schema):
    _property_orders.pop(parent_schema, None)

def add_property_order(schema, order):
    _property_orders["default"][schema] = order

def remove_property_order(schema):
    _property_orders["default"].pop(schema, None)

limit_drive_order = ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]
properties_order= limit_drive_order = ["rotX", "rotY", "rotZ"]

custom_instance_order_map = {
    UsdPhysics.LimitAPI: limit_drive_order,
    PhysxSchema.PhysxLimitAPI: limit_drive_order,
    UsdPhysics.DriveAPI: limit_drive_order,
    Usd.SchemaRegistry.GetTypeFromSchemaTypeName("PhysxJointAxisAPI"): properties_order
}

# call this from e.g. Manager.__init__ to get string to update default_order_map below in a case of a schema change
# relative path of the schemas to ${kit} is valid only for the physics repo!
def generate_order():
    import carb
    from pxr import Sdf
    from collections import defaultdict

    names = ["UsdPhysics", "PhysxSchema"]
    order_dict = defaultdict(list)
    for name in names:
        path = f"{name}/resources/schema.usda"
        path = carb.tokens.get_tokens_interface().resolve("${kit}/../schema/" + path)
        sdfLayer = Sdf.Layer.FindOrOpen(path)

        for sdfPrim in sdfLayer.rootPrims:
            if sdfPrim.name == "Typed" or sdfPrim.specifier != Sdf.SpecifierClass:
                continue

            sdf_name = sdfPrim.customData.get("className", sdfPrim.name)
            sdf_name = f"{name}.{sdf_name}"

            for sdfProp in sdfPrim.properties:
                order_dict[sdf_name].append(sdfProp.name)

    for name, order in order_dict.items():
        print(f"{name}: {order},")

# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import functools
import math

import omni.stageupdate


def register_stage_update_node(display_name, priority=8, **kwargs):  # priority defaults to 8 = pre-physics
    """
    Register a stage update node with the stage update interface.

    Args:
        display_name: The name to display for this update node
        priority: The priority/order of the node (default: 8 for pre-physics)
        **kwargs: Additional arguments to pass to create_stage_update_node

    Returns:
        The created stage update node
    """
    stage_update = omni.stageupdate.get_stage_update_interface()
    stage_update_node = stage_update.create_stage_update_node(display_name, **kwargs)
    nodes = stage_update.get_stage_update_nodes()
    stage_update.set_stage_update_node_order(len(nodes) - 1, priority)
    return stage_update_node


@functools.lru_cache(maxsize=1000, typed=True)
def float_to_string(value, max_length=12, precision=2) -> str:
    if value is None or not isinstance(value, float):
        return "-"
    elif math.isnan(value):
        return "NaN"
    elif value == 0.0:
        # Will often be the case so return early. Also prevents -0.0 output.
        return "0.0"
    elif math.isinf(value):
        if value > 0.0:
            return "Infinity"
        else:
            return "-Infinity"

    max_decimals = (
        max_length - 2
    )  # assume one is used for the decimal point, and there always being at least one number on each side.
    if value < 0.0:
        max_decimals -= 1  # negative sign

    highest_decimal = math.floor(math.log10(math.fabs(value)))
    if highest_decimal < 0:
        # Precision is relative to the first decimal if within -1 to 1.
        precision -= highest_decimal

    value = round(value, min(precision, max_decimals))

    if value == 0.0:
        # Prevents -0.0 output.
        return "0.0"

    if abs(highest_decimal) + 1 > max_decimals:
        return ("{value:>{max_length}.{decimals}e}").format(
            value=value, decimals=max(max_decimals - (4 if highest_decimal < 0 else 4), 1), max_length=max_length
        )

    if highest_decimal > 3:
        # Reserve space for 1k separators.
        max_decimals -= (highest_decimal - 1) // 3

    value_string = ("{value:,.{decimals}f}").format(
        value=value, decimals=min(precision, max_decimals if max_decimals > 0 else 1)
    )

    value_string = value_string.rstrip("0")
    if value_string[-1] == ".":
        value_string += "0"

    return value_string

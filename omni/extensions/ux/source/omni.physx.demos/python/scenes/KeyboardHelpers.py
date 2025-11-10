# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb, omni.appwindow

def init_keyboard(keyboard_callback):
    # register input callback (this should be refactord into actions in the future)
    input = carb.input.acquire_input_interface()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    return input.subscribe_to_keyboard_events(keyboard, keyboard_callback)

def close_keyboard(keyboard_sub):
    input = carb.input.acquire_input_interface()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input.unsubscribe_to_keyboard_events(keyboard, keyboard_sub)

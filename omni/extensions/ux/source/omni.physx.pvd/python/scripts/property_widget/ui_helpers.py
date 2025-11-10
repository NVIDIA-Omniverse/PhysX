# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni import ui

LABEL_WIDTH = 200
LABEL_HEIGHT = 18
INDENT = '    '

def make_label(name, indent_level=0):
    with ui.HStack():
        ui.Label(f'{indent_level * INDENT}{name}', name="label", word_wrap=True, width=LABEL_WIDTH, height=LABEL_HEIGHT)

def make_label_value(name, value, indent_level=0):
    with ui.HStack():
        ui.Label(f'{indent_level * INDENT}{name}', name="label", word_wrap=True, width=LABEL_WIDTH, height=LABEL_HEIGHT)
        ui.StringField(name="models").model.set_value(str(value) if value != 3.4028234663852885981170418348452e+38 else 'PX_MAX_F32')

def make_label_button(name, value, callback, indent_level=0, enabled=True):
    with ui.HStack():
        ui.Label(f'{indent_level * INDENT}{name}', name="label", word_wrap=True, width=LABEL_WIDTH, height=LABEL_HEIGHT)
        ui.Button(
            text=str(value), 
            width=0,  # Let the button size to its content
            height=LABEL_HEIGHT, 
            clicked_fn=callback if enabled else None, 
            alignment=ui.Alignment.LEFT,
            enabled=enabled,
            style={"Button": {"alignment": ui.Alignment.LEFT}}
        )

def make_label_int_field(name, initial_value, min_value, max_value, callback=None, indent_level=0):
    """Create a label with an integer input field that has min/max constraints."""
    with ui.HStack():
        ui.Label(f'{indent_level * INDENT}{name}', name="label", word_wrap=True, width=LABEL_WIDTH, height=LABEL_HEIGHT)
        int_field = ui.IntDrag(min=min_value, max=max_value, width=LABEL_WIDTH, height=LABEL_HEIGHT)
        int_field.model.set_value(initial_value)
        if callback:
            int_field.model.add_value_changed_fn(callback)
        return int_field

def make_inline_pagination_controls(display_name, total_size, start_index, page_size, on_prev_page, on_next_page, on_page_index_change, on_page_size_change, indent_level=0):
    """Create pagination controls inline with the attribute label."""
    # Use same height as regular labels for consistent font size
    PAGINATION_HEIGHT = LABEL_HEIGHT
    
    # Calculate current page index (0-based)
    current_page = start_index // page_size if page_size > 0 else 0
    total_pages = (total_size + page_size - 1) // page_size if page_size > 0 else 1
    
    # Validate input values to prevent crashes
    max_page_size = min(100, total_size) if total_size > 0 else 1
    valid_page_size = max(1, min(max_page_size, page_size))
    
    with ui.HStack():
        # Attribute label
        ui.Label(f'{indent_level * INDENT}{display_name}', name=f"label_{display_name}_attr", word_wrap=True, width=LABEL_WIDTH, height=PAGINATION_HEIGHT, style={"font_size": 14})
        
        # Previous page button
        prev_button = ui.Button(
            text="<", 
            width=25, 
            height=LABEL_HEIGHT,
            clicked_fn=on_prev_page if on_prev_page else lambda: None,
            enabled=current_page > 0,
            style={"Button": {"alignment": ui.Alignment.CENTER}}
        )
        
        # Next page button
        next_button = ui.Button(
            text=">", 
            width=25, 
            height=LABEL_HEIGHT,
            clicked_fn=on_next_page if on_next_page else lambda: None,
            enabled=current_page < total_pages - 1,
            style={"Button": {"alignment": ui.Alignment.CENTER}}
        )
        
        # Page number field (editable)
        ui.Label("PageIndex:", name=f"label_{display_name}_page_label", word_wrap=True, width=70, height=LABEL_HEIGHT, style={"font_size": 14}, alignment=ui.Alignment.CENTER_BOTTOM)
        page_index_field = ui.StringField(width=40, height=LABEL_HEIGHT)
        page_index_field.model.set_value(str(current_page))  # Display as 0-based
        if on_page_index_change:
            page_index_field.model.add_end_edit_fn(on_page_index_change)
        
        # Page size field (editable)
        ui.Label("PageSize:", name=f"label_{display_name}_pagesize_label", word_wrap=True, width=60, height=LABEL_HEIGHT, style={"font_size": 14}, alignment=ui.Alignment.CENTER_BOTTOM)
        page_size_field = ui.StringField(width=40, height=LABEL_HEIGHT)
        page_size_field.model.set_value(str(valid_page_size))
        if on_page_size_change:
            page_size_field.model.add_end_edit_fn(on_page_size_change)
        
        # Total pages (read-only)
        ui.Label("NbrPages:", name=f"label_{display_name}_nbrpages_label", word_wrap=True, width=65, height=LABEL_HEIGHT, style={"font_size": 14}, alignment=ui.Alignment.CENTER_BOTTOM)
        ui.Label(f"{total_pages}", name=f"label_{display_name}_total_pages", word_wrap=True, width=40, height=LABEL_HEIGHT, style={"font_size": 14}, alignment=ui.Alignment.CENTER_BOTTOM)
        
        # Total elements (read-only)
        ui.Label("NbrElements:", name=f"label_{display_name}_nbrelements_label", word_wrap=True, width=80, height=LABEL_HEIGHT, style={"font_size": 14}, alignment=ui.Alignment.CENTER_BOTTOM)
        ui.Label(f"{total_size}", name=f"label_{display_name}_total_elements", word_wrap=True, width=50, height=LABEL_HEIGHT, style={"font_size": 14}, alignment=ui.Alignment.CENTER_BOTTOM)
        
        return prev_button, next_button, page_index_field, page_size_field

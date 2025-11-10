# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from collections.abc import Iterable
from pxr import Usd
from functools import partial
import asyncio
import omni.kit.app

from omni.physxpvd.scripts.property_widget.ui_helpers import (
    make_label,
    make_label_value,
    make_label_button,
    make_inline_pagination_controls,
)

from omni.physxpvd.scripts.property_widget.usd_helpers import (
    get_class_name,
    get_time,
    convert_from_camel_case,
    select_by_object_handle,
    get_object_handle,
    OMNI_PVD_ATTRIBUTE_DATA_PREFIX,
    OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX,
)

# Global state for pagination controls
_pagination_state = {}

# Flag to track if we're currently in a rebuild operation
_is_rebuilding = False

# Debug flag to enable logging
_debug_pagination = False

def _get_pagination_key(prim_path, attribute_name):
    """Generate a unique key for pagination state."""
    return f"{prim_path}_{attribute_name}"

def reset_pagination_state():
    """Reset all pagination state. Useful for debugging."""
    global _pagination_state, _is_rebuilding
    _pagination_state.clear()
    _is_rebuilding = False
    if _debug_pagination:
        print("Pagination state reset")



def _preserve_scroll_position_and_rebuild():
    """Preserve scroll position and rebuild the property window."""
    global _is_rebuilding
    
    if _debug_pagination:
        print(f"Rebuild function called, _is_rebuilding flag: {_is_rebuilding}")
    
    # Force reset the flag if it's been stuck for too long
    # This is a safety mechanism to prevent the flag from getting stuck
    if _is_rebuilding:
        if _debug_pagination:
            print("Rebuild flag is True, forcing reset and continuing")
        _is_rebuilding = False
    
    try:
        import omni.kit.window.property as property_window
        import omni.kit.property.physx.utils as physx_utils
        
        # Get the current property window
        window = property_window.get_window()
        
        if window and hasattr(window, '_window_frame'):
            # Check if there's a target scroll position set from reference navigation
            try:
                from omni.physxpvd.scripts.property_widget.usd_helpers import get_target_scroll_position
                target_scroll_y = get_target_scroll_position()
                if target_scroll_y is not None:
                    # Use target scroll position from reference navigation
                    current_scroll_y = target_scroll_y
                else:
                    # Capture current scroll position
                    current_scroll_y = window._window_frame.scroll_y
            except Exception:
                # Fallback to current scroll position
                current_scroll_y = window._window_frame.scroll_y
            
            # Set rebuilding flag
            _is_rebuilding = True
            
            if _debug_pagination:
                print(f"Starting rebuild with scroll position: {current_scroll_y}")
                print(f"Window frame scroll_y before rebuild: {window._window_frame.scroll_y}")
            
            # Rebuild the property window
            physx_utils.rebuild_property_window()
            
            # Restore scroll position after rebuild
            async def restore_scroll_position():
                try:
                    # Wait for rebuild to complete - use more frames to ensure rebuild is done
                    for _ in range(10):
                        await omni.kit.app.get_app().next_update_async()
                    
                    # Restore scroll position
                    try:
                        window_after_rebuild = property_window.get_window()
                        if _debug_pagination:
                            print(f"Window after rebuild: {window_after_rebuild}")
                        
                        if window_after_rebuild and hasattr(window_after_rebuild, '_window_frame'):
                            if _debug_pagination:
                                print(f"Window frame scroll_y before restore: {window_after_rebuild._window_frame.scroll_y}")
                            
                            window_after_rebuild._window_frame.scroll_y = current_scroll_y
                            
                            if _debug_pagination:
                                print(f"Restored scroll position to: {current_scroll_y}")
                                print(f"Window frame scroll_y after restore: {window_after_rebuild._window_frame.scroll_y}")
                            
                            # Try a second restoration after a short delay in case the first one didn't stick
                            await omni.kit.app.get_app().next_update_async()
                            if window_after_rebuild._window_frame.scroll_y != current_scroll_y:
                                if _debug_pagination:
                                    print(f"Scroll position changed, restoring again: {window_after_rebuild._window_frame.scroll_y} -> {current_scroll_y}")
                                window_after_rebuild._window_frame.scroll_y = current_scroll_y
                            

                        else:
                            if _debug_pagination:
                                print(f"Could not find window after rebuild")
                    except Exception as e:
                        if _debug_pagination:
                            print(f"Failed to restore scroll position: {e}")
                except Exception as e:
                    if _debug_pagination:
                        print(f"Error in restore_scroll_position: {e}")
                finally:
                    _is_rebuilding = False
                    if _debug_pagination:
                        print("Rebuild completed, flag reset")
            
            # Start the async task and ensure it completes
            task = asyncio.ensure_future(restore_scroll_position())
            
            # Add a timeout to ensure the flag gets reset even if the task fails
            async def timeout_handler():
                try:
                    await asyncio.wait_for(task, timeout=2.0)  # 2 second timeout
                except asyncio.TimeoutError:
                    if _debug_pagination:
                        print("Rebuild timeout, forcing flag reset")
                    _is_rebuilding = False
                except Exception as e:
                    if _debug_pagination:
                        print(f"Rebuild task failed: {e}")
                    _is_rebuilding = False
            
            asyncio.ensure_future(timeout_handler())
        else:
            # Simple rebuild if no window frame
            physx_utils.rebuild_property_window()
            _is_rebuilding = False
    except Exception as e:
        _is_rebuilding = False
        if _debug_pagination:
            print(f"Rebuild failed: {e}")

def _on_start_index_change(prim_path, attribute_name):
    """Callback for start index changes."""
    def callback(model):
        try:
            key = _get_pagination_key(prim_path, attribute_name)
            input_value = model.get_value_as_string()
            
            if _debug_pagination:
                print(f"Start index callback: key={key}, input_value={input_value}")
            
            # Try to parse the input as an integer
            try:
                new_value = int(input_value)
            except ValueError:
                if _debug_pagination:
                    print(f"Invalid input: {input_value}, setting to minimum value")
                new_value = 0
            
            if key in _pagination_state:
                # Validate start index: must be >= 0 and < total_size
                total_size = _pagination_state[key].get('total_size', 0)
                max_start_index = max(0, total_size - 1) if total_size > 0 else 0
                valid_start_index = max(0, min(max_start_index, new_value))
                
                # Update the model if the value was clamped
                if valid_start_index != new_value:
                    model.set_value(str(valid_start_index))
                    if _debug_pagination:
                        print(f"Clamped start_index from {new_value} to {valid_start_index}")
                
                _pagination_state[key]['start_index'] = valid_start_index
                if _debug_pagination:
                    print(f"Updated start_index to {valid_start_index}")
                _preserve_scroll_position_and_rebuild()
            else:
                if _debug_pagination:
                    print(f"Key {key} not found, trying to find matching key")
                # Try to find matching key
                for existing_key in _pagination_state.keys():
                    if attribute_name in existing_key:
                        total_size = _pagination_state[existing_key].get('total_size', 0)
                        max_start_index = max(0, total_size - 1) if total_size > 0 else 0
                        valid_start_index = max(0, min(max_start_index, new_value))
                        
                        # Update the model if the value was clamped
                        if valid_start_index != new_value:
                            model.set_value(str(valid_start_index))
                            if _debug_pagination:
                                print(f"Clamped start_index from {new_value} to {valid_start_index}")
                        
                        _pagination_state[existing_key]['start_index'] = valid_start_index
                        if _debug_pagination:
                            print(f"Updated using matching key: {existing_key}")
                        _preserve_scroll_position_and_rebuild()
                        return
                if _debug_pagination:
                    print("No matching key found")
        except Exception as e:
            if _debug_pagination:
                print(f"Start index callback error: {e}")
    return callback

def _on_page_size_change(prim_path, attribute_name):
    """Callback for page size changes."""
    def callback(model):
        try:
            key = _get_pagination_key(prim_path, attribute_name)
            input_value = model.get_value_as_string()
            
            if _debug_pagination:
                print(f"Page size callback: key={key}, input_value={input_value}")
            
            # Try to parse the input as an integer
            try:
                new_value = int(input_value)
            except ValueError:
                if _debug_pagination:
                    print(f"Invalid input: {input_value}, setting to minimum value")
                new_value = 1
            
            if key in _pagination_state:
                total_size = _pagination_state[key].get('total_size', 100)
                # Validate page size: must be >= 1 and <= min(100, total_size)
                max_page_size = min(100, total_size) if total_size > 0 else 1
                valid_page_size = max(1, min(max_page_size, new_value))
                
                # Update the model if the value was clamped
                if valid_page_size != new_value:
                    model.set_value(str(valid_page_size))
                    if _debug_pagination:
                        print(f"Clamped page_size from {new_value} to {valid_page_size}")
                
                # Adjust start_index to maintain current page when page size changes
                old_page_size = _pagination_state[key].get('page_size', valid_page_size)
                old_start_index = _pagination_state[key].get('start_index', 0)
                current_page = old_start_index // old_page_size if old_page_size > 0 else 0
                new_start_index = current_page * valid_page_size
                
                # Ensure new start index doesn't exceed bounds
                max_start_index = max(0, total_size - valid_page_size) if total_size > 0 else 0
                new_start_index = min(new_start_index, max_start_index)
                
                _pagination_state[key]['page_size'] = valid_page_size
                _pagination_state[key]['start_index'] = new_start_index
                
                if _debug_pagination:
                    print(f"Updated page_size to {valid_page_size}, adjusted start_index to {new_start_index}")
                
                _preserve_scroll_position_and_rebuild()
            else:
                if _debug_pagination:
                    print(f"Key {key} not found, trying to find matching key")
                # Try to find matching key
                for existing_key in _pagination_state.keys():
                    if attribute_name in existing_key:
                        total_size = _pagination_state[existing_key].get('total_size', 100)
                        max_page_size = min(100, total_size) if total_size > 0 else 1
                        valid_page_size = max(1, min(max_page_size, new_value))
                        
                        # Update the model if the value was clamped
                        if valid_page_size != new_value:
                            model.set_value(str(valid_page_size))
                            if _debug_pagination:
                                print(f"Clamped page_size from {new_value} to {valid_page_size}")
                        
                        _pagination_state[existing_key]['page_size'] = valid_page_size
                        if _debug_pagination:
                            print(f"Updated using matching key: {existing_key}")
                        
                        _preserve_scroll_position_and_rebuild()
                        return
                if _debug_pagination:
                    print("No matching key found")
        except Exception as e:
            if _debug_pagination:
                print(f"Page size callback error: {e}")
    return callback

def _on_prev_page(prim_path, attribute_name):
    """Callback for previous page button."""
    def callback():
        try:
            key = _get_pagination_key(prim_path, attribute_name)
            if _debug_pagination:
                print(f"Previous page callback: key={key}")
            
            if key in _pagination_state:
                current_state = _pagination_state[key]
                page_size = current_state.get('page_size', 10)
                start_index = current_state.get('start_index', 0)
                
                # Calculate new start index (previous page)
                new_start_index = max(0, start_index - page_size)
                
                current_state['start_index'] = new_start_index
                if _debug_pagination:
                    print(f"Updated start_index to {new_start_index} (previous page)")
                
                _preserve_scroll_position_and_rebuild()
        except Exception as e:
            if _debug_pagination:
                print(f"Previous page callback error: {e}")
    return callback

def _on_next_page(prim_path, attribute_name):
    """Callback for next page button."""
    def callback():
        try:
            key = _get_pagination_key(prim_path, attribute_name)
            if _debug_pagination:
                print(f"Next page callback: key={key}")
            
            if key in _pagination_state:
                current_state = _pagination_state[key]
                page_size = current_state.get('page_size', 10)
                start_index = current_state.get('start_index', 0)
                total_size = current_state.get('total_size', 0)
                
                # Calculate new start index (next page)
                max_start_index = max(0, total_size - page_size) if total_size > 0 else 0
                new_start_index = min(start_index + page_size, max_start_index)
                
                current_state['start_index'] = new_start_index
                if _debug_pagination:
                    print(f"Updated start_index to {new_start_index} (next page)")
                
                _preserve_scroll_position_and_rebuild()
        except Exception as e:
            if _debug_pagination:
                print(f"Next page callback error: {e}")
    return callback

def _on_page_index_change(prim_path, attribute_name):
    """Callback for page index changes."""
    def callback(model):
        try:
            key = _get_pagination_key(prim_path, attribute_name)
            input_value = model.get_value_as_string()
            
            if _debug_pagination:
                print(f"Page index callback: key={key}, input_value={input_value}")
            
            # Try to parse the input as an integer (0-based)
            try:
                page_index = int(input_value)
            except ValueError:
                if _debug_pagination:
                    print(f"Invalid input: {input_value}, setting to page 0")
                page_index = 0
            
            if key in _pagination_state:
                current_state = _pagination_state[key]
                page_size = current_state.get('page_size', 10)
                total_size = current_state.get('total_size', 0)
                
                # Calculate total pages and validate page index (0-based)
                total_pages = (total_size + page_size - 1) // page_size if page_size > 0 else 1
                valid_page_index = max(0, min(total_pages - 1, page_index))
                
                # Update the model if the value was clamped
                if valid_page_index != page_index:
                    model.set_value(str(valid_page_index))
                    if _debug_pagination:
                        print(f"Clamped page_index from {page_index} to {valid_page_index}")
                
                # Convert to start index
                new_start_index = valid_page_index * page_size
                
                current_state['start_index'] = new_start_index
                if _debug_pagination:
                    print(f"Updated start_index to {new_start_index} (page index {valid_page_index})")
                
                _preserve_scroll_position_and_rebuild()
        except Exception as e:
            if _debug_pagination:
                print(f"Page index callback error: {e}")
    return callback

def _get_object_names_for_handles(handles, stage):
    """Get object names and actionability for a list of handles.
    
    Args:
        handles: List of object handles
        stage: USD stage to search in
        
    Returns:
        Tuple of (object_names, is_actionable_list)
        object_names: List of object names
        - in case no prim is found for the handle, the value is "INVALID"
        - if the handle is 0, the value is "NULL"
        - if the handle belongs to a prim with a viz attribute, the prim that is currently active or the prim that was active most recently will be used, otherwise the prim that was first encountered will be used.
        is_actionable_list: List of booleans indicating if each handle is actionable (has active viz or no viz requirement)
        - False for handles with value 0 ("NULL")
        - False for handles that point to prims with viz attribute but are not currently active
        - True for handles that point to prims without viz attribute or with active viz attribute
        - False for "INVALID" handles
    """
    # Initialize result lists - set "NULL" for handles with value 0, "INVALID" for others
    object_names = []
    is_actionable = []
    for handle in handles:
        if int(handle) == 0:
            object_names.append("NULL")
            is_actionable.append(False)  # NULL handles are not actionable
        else:
            object_names.append("INVALID")
            is_actionable.append(False)  # Invalid handles are not actionable
    curr_time = get_time()
    
    # Convert handles to set for O(1) lookup
    handle_set = {int(handle) for handle in handles}
    
    # Track the best prim for each handle (either currently active or last active)
    handle_to_best_prim = {}
    
    # Iterate over prims to find the best match for each handle
    for prim in stage.TraverseAll():
        traversing_handle = get_object_handle(prim)
        if not traversing_handle:
            continue
            
        handle_int = int(traversing_handle)
        if handle_int not in handle_set:
            continue
            
        # Check if this prim has viz attribute and determine its state
        has_viz_attr = prim.HasAttribute("omni:pvdi:viz")
        is_currently_active = False
        last_active_time = None
        
        if has_viz_attr:
            viz_attr = prim.GetAttribute("omni:pvdi:viz")
            is_currently_active = viz_attr.Get(curr_time)
            
            # Calculate the last active time for this prim
            if not is_currently_active:
                try:
                    # Get all time samples for this attribute
                    time_samples = viz_attr.GetTimeSamples()
                    # Find the latest time where this prim was active and <= curr_time
                    for sample_time in reversed(time_samples):
                        if sample_time <= curr_time and viz_attr.Get(sample_time):
                            last_active_time = sample_time
                            break
                except:
                    last_active_time = None
        
        # Determine if this prim is better than the current best for this handle
        should_use_this_prim = False
        
        if handle_int not in handle_to_best_prim:
            # First prim we've seen with this handle
            should_use_this_prim = True
        else:
            existing_prim, existing_active, existing_last_time, existing_has_viz = handle_to_best_prim[handle_int]
            
            # Priority: has_viz_attr > currently active > last active time > first encountered
            if has_viz_attr and not existing_has_viz:
                # Current prim has viz attribute, existing doesn't
                should_use_this_prim = True
            elif not has_viz_attr and existing_has_viz:
                # Current prim doesn't have viz attribute, existing does - keep existing
                should_use_this_prim = False
            elif has_viz_attr and existing_has_viz:
                # Both have viz attributes - use activity-based priority
                if is_currently_active and not existing_active:
                    # Current prim is active, existing isn't
                    should_use_this_prim = True
                elif not is_currently_active and existing_active:
                    # Current prim isn't active, existing is - keep existing
                    should_use_this_prim = False
                elif is_currently_active and existing_active:
                    # Both are active - keep the first one (existing)
                    should_use_this_prim = False
                else:
                    # Both are inactive - use the one that was active more recently
                    if last_active_time is not None and (existing_last_time is None or last_active_time > existing_last_time):
                        should_use_this_prim = True
            else:
                # Neither has viz attribute - keep the first one (existing)
                should_use_this_prim = False
        
        if should_use_this_prim:
            handle_to_best_prim[handle_int] = (prim.GetName(), is_currently_active, last_active_time, has_viz_attr)
    
    # Update object names and actionability based on the best prim found for each handle
    for i, target_handle in enumerate(handles):
        handle_int = int(target_handle)
        if handle_int in handle_to_best_prim:
            prim_name, is_currently_active, last_active_time, has_viz_attr = handle_to_best_prim[handle_int]
            object_names[i] = prim_name
            
            # Determine if this handle is actionable:
            # - Prims without viz attribute are always actionable
            # - Prims with viz attribute are only actionable if currently active
            if not has_viz_attr:
                is_actionable[i] = True  # No viz attribute = always actionable
            else:
                is_actionable[i] = is_currently_active  # Has viz = actionable only if active
    
    return object_names, is_actionable

def prim_handler(prim: Usd.Prim) -> bool:
    if not prim.HasAttribute("omni:pvdi:class"):
        return False
    make_label(get_class_name(prim))

    for attribute in prim.GetAttributes():
        name = attribute.GetName()
        if name.startswith(OMNI_PVD_ATTRIBUTE_DATA_PREFIX):
            value = attribute.Get(get_time())
            display_name = convert_from_camel_case(name, OMNI_PVD_ATTRIBUTE_DATA_PREFIX)
            
            # Check if the value is iterable (list/array of data)
            if isinstance(value, Iterable) and not isinstance(value, str):
                data_values = list(value)
                total_size = len(data_values)
                
                # Handle empty iterable case
                if total_size == 0:
                    make_label(f'{display_name} (Empty Data List)', 1)
                    continue
                
                # Handle small iterables (< 10 elements) - display directly without pagination
                if total_size < 10:
                    if total_size == 1:
                        # Single element - display as before
                        make_label_value(display_name, data_values[0])
                    else:
                        # Multiple elements but < 10 - display all with indices
                        make_label(display_name, 1)
                        for i, data_value in enumerate(data_values):
                            make_label_value(f'[{i}]', data_value, 2)
                    continue
                
                # Initialize pagination state if not exists for data attributes
                key = _get_pagination_key(prim.GetPath(), name)
                if _debug_pagination:
                    print(f"Processing data pagination for prim_path: {prim.GetPath()}, attribute: {name}, key: {key}")
                
                if key not in _pagination_state:
                    _pagination_state[key] = {
                        'start_index': 0,
                        'page_size': min(10, total_size),
                        'total_size': total_size
                    }
                    if _debug_pagination:
                        print(f"Created data pagination state for key: {key}")
                else:
                    # Update total size in case it changed, but preserve existing pagination values
                    old_start_index = _pagination_state[key].get('start_index', 0)
                    old_page_size = _pagination_state[key].get('page_size', min(10, total_size))
                    
                    _pagination_state[key]['total_size'] = total_size
                    
                    if _debug_pagination:
                        print(f"Updated data pagination state for key: {key}, total_size: {total_size}")
                        print(f"Preserved start_index: {old_start_index}, page_size: {old_page_size}")
                
                current_state = _pagination_state[key]
                start_index = current_state['start_index']
                page_size = current_state['page_size']
                
                # Ensure start_index doesn't exceed bounds after total_size update
                max_start_index = max(0, total_size - 1) if total_size > 0 else 0
                if start_index > max_start_index:
                    start_index = max_start_index
                    current_state['start_index'] = start_index
                    if _debug_pagination:
                        print(f"Adjusted data start_index to {start_index} due to total_size change")
                
                # Ensure page_size doesn't exceed total_size
                max_page_size = min(100, total_size) if total_size > 0 else 1
                if page_size > max_page_size:
                    page_size = max_page_size
                    current_state['page_size'] = page_size
                    if _debug_pagination:
                        print(f"Adjusted data page_size to {page_size} due to total_size change")
                
                # Ensure start_index doesn't exceed bounds
                start_index = min(start_index, max(0, total_size - 1))
                current_state['start_index'] = start_index
                
                # Create pagination controls for data
                if _debug_pagination:
                    print(f"Creating data pagination controls with start_index={start_index}, page_size={page_size}, total_size={total_size}")
                
                # Create callbacks with current prim path and attribute name
                prev_page_callback = _on_prev_page(prim.GetPath(), name)
                next_page_callback = _on_next_page(prim.GetPath(), name)
                page_index_callback = _on_page_index_change(prim.GetPath(), name)
                page_size_callback = _on_page_size_change(prim.GetPath(), name)
                
                if _debug_pagination:
                    print(f"Created data callbacks for prim_path: {prim.GetPath()}, attribute: {name}")
                
                prev_button, next_button, page_index_field, page_size_field = make_inline_pagination_controls(
                    display_name,
                    total_size,
                    start_index,
                    page_size,
                    prev_page_callback,
                    next_page_callback,
                    page_index_callback,
                    page_size_callback
                )
                
                # Display the paginated data elements
                end_index = min(start_index + page_size, total_size)
                data_in_page = data_values[start_index:end_index]
                
                for i, data_value in enumerate(data_in_page):
                    make_label_value(f'[{start_index + i}]', data_value, 1)
            else:
                # Single value or non-iterable - display as before
                make_label_value(display_name, value)
        if name.startswith(OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX):
            value = attribute.Get(get_time())

            display_name = convert_from_camel_case(name, OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX)
            
            # If the value is an iterable, we need to handle it as a list of handles
            if isinstance(value, Iterable):
                target_handles = value
                total_size = len(target_handles)
                
                # Handle empty iterable case
                if total_size == 0:
                    make_label(f'{display_name} (Empty Ref List)', 1)
                    continue
                
                # Handle small iterables (< 10 elements) - display directly without pagination
                if total_size < 10:
                    # Get object names and actionability for all handles
                    object_names, is_actionable = _get_object_names_for_handles(target_handles, prim.GetStage())
                    
                    if total_size == 1:
                        # Single element - display as before
                        target_handle = target_handles[0]
                        object_name = object_names[0]
                        button_text = f"{int(target_handle)} : {object_name}"
                        make_label_button(f'{display_name} (Ref)', button_text, partial(select_by_object_handle, target_handle), enabled=is_actionable[0])
                    else:
                        # Multiple elements but < 10 - display all with indices
                        make_label(f'{display_name} (Ref List)', 1)
                        for i, (target_handle, object_name, actionable) in enumerate(zip(target_handles, object_names, is_actionable)):
                            button_text = f"{int(target_handle)} : {object_name}"
                            make_label_button(f'[{i}]', button_text, partial(select_by_object_handle, target_handle), 2, enabled=actionable)
                    continue
                
                # Initialize pagination state if not exists
                key = _get_pagination_key(prim.GetPath(), name)
                if _debug_pagination:
                    print(f"Processing pagination for prim_path: {prim.GetPath()}, attribute: {name}, key: {key}")
                
                if key not in _pagination_state:
                    _pagination_state[key] = {
                        'start_index': 0,
                        'page_size': min(10, total_size),
                        'total_size': total_size
                    }
                    if _debug_pagination:
                        print(f"Created pagination state for key: {key}")
                else:
                    # Update total size in case it changed, but preserve existing pagination values
                    old_start_index = _pagination_state[key].get('start_index', 0)
                    old_page_size = _pagination_state[key].get('page_size', min(10, total_size))
                    
                    _pagination_state[key]['total_size'] = total_size
                    
                    if _debug_pagination:
                        print(f"Updated pagination state for key: {key}, total_size: {total_size}")
                        print(f"Preserved start_index: {old_start_index}, page_size: {old_page_size}")
                
                current_state = _pagination_state[key]
                start_index = current_state['start_index']
                page_size = current_state['page_size']
                
                # Ensure start_index doesn't exceed bounds after total_size update
                max_start_index = max(0, total_size - 1) if total_size > 0 else 0
                if start_index > max_start_index:
                    start_index = max_start_index
                    current_state['start_index'] = start_index
                    if _debug_pagination:
                        print(f"Adjusted start_index to {start_index} due to total_size change")
                
                # Ensure page_size doesn't exceed total_size
                max_page_size = min(100, total_size) if total_size > 0 else 1
                if page_size > max_page_size:
                    page_size = max_page_size
                    current_state['page_size'] = page_size
                    if _debug_pagination:
                        print(f"Adjusted page_size to {page_size} due to total_size change")
                
                # Ensure start_index doesn't exceed bounds
                start_index = min(start_index, max(0, total_size - 1))
                current_state['start_index'] = start_index
                
                # Create pagination controls
                if _debug_pagination:
                    print(f"Creating pagination controls with start_index={start_index}, page_size={page_size}, total_size={total_size}")
                
                # Create callbacks with current prim path and attribute name
                prev_page_callback = _on_prev_page(prim.GetPath(), name)
                next_page_callback = _on_next_page(prim.GetPath(), name)
                page_index_callback = _on_page_index_change(prim.GetPath(), name)
                page_size_callback = _on_page_size_change(prim.GetPath(), name)
                
                if _debug_pagination:
                    print(f"Created callbacks for prim_path: {prim.GetPath()}, attribute: {name}")
                
                prev_button, next_button, page_index_field, page_size_field = make_inline_pagination_controls(
                    display_name,
                    total_size,
                    start_index,
                    page_size,
                    prev_page_callback,
                    next_page_callback,
                    page_index_callback,
                    page_size_callback
                )
                
                # Display the paginated elements
                end_index = min(start_index + page_size, total_size)
                handles_in_page = target_handles[start_index:end_index]
                
                # Get object names and actionability for all handles in the current page
                object_names, is_actionable = _get_object_names_for_handles(handles_in_page, prim.GetStage())
                
                for i, (target_handle, object_name, actionable) in enumerate(zip(handles_in_page, object_names, is_actionable)):
                    button_text = f"{int(target_handle)} : {object_name}"
                    make_label_button(f'[{start_index + i}]', button_text, partial(select_by_object_handle, target_handle), 1, enabled=actionable)
            else:
                target_handle = value
                
                # Get object name and actionability using the helper function
                object_names, is_actionable = _get_object_names_for_handles([target_handle], prim.GetStage())
                object_name = object_names[0]
                actionable = is_actionable[0]
                
                button_text = f"{int(target_handle)} : {object_name}"
                make_label_button(f'{display_name} (Ref)', button_text, partial(select_by_object_handle, target_handle), enabled=actionable)

    return True

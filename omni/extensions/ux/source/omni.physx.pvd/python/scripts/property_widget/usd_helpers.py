# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from re import sub
from omni.usd import get_context
from omni.timeline import get_timeline_interface

OMNI_PVD_ATTRIBUTE_DATA_PREFIX = 'omni:pvd:'
OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX = 'omni:pvdh:'
OMNI_PVD_METADATA_PREFIX = 'omni:pvdi:'

# Global registry for reference navigation
_reference_navigation_delegate = None
_global_reference_history = []  # List of context dictionaries
_global_reference_history_index = -1
_max_history_size = 50
_target_scroll_position = None  # For storing target scroll position during navigation

################################################################################
# PERFORMANCE OPTIMIZATION: Handle-to-prim cache
# Caches handle -> prim path mapping to avoid expensive full stage traversal
# on every handle lookup. The cache is invalidated on stage changes.
################################################################################
class HandlePrimCache:
    def __init__(self):
        self._handle_to_paths = {}  # handle -> list of prim paths
        self._is_valid = False
        self._stage_id = None  # Track which stage this cache is for
    
    def invalidate(self):
        """Invalidate the cache."""
        self._handle_to_paths = {}
        self._is_valid = False
        self._stage_id = None
    
    def _build_cache(self, stage):
        """Build the handle-to-path cache by traversing the stage once."""
        self._handle_to_paths = {}
        self._stage_id = id(stage) if stage else None
        
        if not stage:
            self._is_valid = False
            return
        
        # Traverse once and cache all handle -> path mappings
        for prim in stage.TraverseAll():
            if prim.HasAttribute("omni:pvdi:handle"):
                handle_attr = prim.GetAttribute("omni:pvdi:handle")
                handle = handle_attr.Get()
                if handle is not None:
                    handle_int = int(handle)
                    if handle_int not in self._handle_to_paths:
                        self._handle_to_paths[handle_int] = []
                    self._handle_to_paths[handle_int].append(prim.GetPath())
        
        self._is_valid = True
    
    def ensure_valid(self, stage):
        """Ensure the cache is valid for the current stage."""
        if stage is None:
            self.invalidate()
            return False
        
        current_stage_id = id(stage)
        if not self._is_valid or self._stage_id != current_stage_id:
            self._build_cache(stage)
        
        return self._is_valid
    
    def get_paths_for_handle(self, handle):
        """Get cached prim paths for a handle. Returns empty list if not found."""
        handle_int = int(handle)
        return self._handle_to_paths.get(handle_int, [])
    
    def get_all_handles(self):
        """Get all cached handles."""
        return set(self._handle_to_paths.keys())

# Global cache instance
_handle_prim_cache = HandlePrimCache()

def invalidate_handle_cache():
    """Invalidate the handle-to-prim cache. Call this on stage open/close."""
    global _handle_prim_cache
    _handle_prim_cache.invalidate()

def get_handle_prim_cache():
    """Get the global handle-to-prim cache."""
    return _handle_prim_cache

def register_reference_navigation_delegate(delegate):
    """Register a delegate for reference navigation."""
    global _reference_navigation_delegate
    
    # If we already have a delegate, transfer its history to the new one
    if _reference_navigation_delegate and _reference_navigation_delegate != delegate:
        if hasattr(_reference_navigation_delegate, '_reference_history'):
            old_history = _reference_navigation_delegate._reference_history
            old_index = _reference_navigation_delegate._reference_history_index
            
            # Transfer history to new delegate
            delegate._reference_history = old_history
            delegate._reference_history_index = old_index
            delegate._max_history_size = getattr(_reference_navigation_delegate, '_max_history_size', 50)
            
            # Update button states for new delegate
            if hasattr(delegate, '_update_reference_navigation_buttons'):
                delegate._update_reference_navigation_buttons()
    
    _reference_navigation_delegate = delegate

def unregister_reference_navigation_delegate():
    """Unregister the reference navigation delegate."""
    global _reference_navigation_delegate
    _reference_navigation_delegate = None

def add_to_global_reference_history(prim_path, ui_context=None):
    """Add a prim path with UI context to the global reference navigation history."""
    global _global_reference_history, _global_reference_history_index, _max_history_size
    
    # Create context entry
    context_entry = {
        'prim_path': prim_path,
        'ui_context': ui_context or {},
        'timestamp': __import__('time').time()
    }
    
    # If we're not at the end of history, remove everything after current position
    if _global_reference_history_index < len(_global_reference_history) - 1:
        _global_reference_history = _global_reference_history[:_global_reference_history_index + 1]
    
    # Add new entry or update existing if same prim but with new UI context
    if not _global_reference_history or _global_reference_history[-1]['prim_path'] != prim_path:
        # Different prim - add new entry
        _global_reference_history.append(context_entry)
        _global_reference_history_index = len(_global_reference_history) - 1
        
        # Limit history size
        if len(_global_reference_history) > _max_history_size:
            _global_reference_history.pop(0)
            _global_reference_history_index = len(_global_reference_history) - 1
    elif ui_context:
        # Same prim but with new UI context - update the context
        _global_reference_history[-1]['ui_context'] = ui_context
        _global_reference_history[-1]['timestamp'] = context_entry['timestamp']
    
    # Update button states on current delegate
    if _reference_navigation_delegate and hasattr(_reference_navigation_delegate, '_update_reference_navigation_buttons'):
        _reference_navigation_delegate._update_reference_navigation_buttons()

def navigate_reference_history_prev():
    """Navigate to previous reference in global history."""
    global _global_reference_history, _global_reference_history_index
    
    if _global_reference_history_index > 0:
        _global_reference_history_index -= 1
        entry = _global_reference_history[_global_reference_history_index]
        prim_path = entry['prim_path']
        ui_context = entry.get('ui_context', {})
        
        # Set selection without adding to history
        from omni.usd import get_context
        get_context().get_selection().set_selected_prim_paths([prim_path], True)
        
        # Force property window rebuild after selection change
        import asyncio
        import omni.kit.app as kit_app
        async def delayed_rebuild_and_restore():
            # Wait for selection change to propagate
            for _ in range(3):
                await kit_app.get_app().next_update_async()
            
            # Force rebuild the property window
            try:
                import omni.kit.window.property as property_window
                window = property_window.get_window()
                if window and hasattr(window, '_window') and hasattr(window._window, 'frame'):
                    window._window.frame.rebuild()
            except Exception:
                pass
            
            # Wait for rebuild to complete
            for _ in range(3):
                await kit_app.get_app().next_update_async()
            
            # Then restore UI context
            _restore_ui_context(ui_context)
        asyncio.ensure_future(delayed_rebuild_and_restore())
        
        # Update button states
        if _reference_navigation_delegate and hasattr(_reference_navigation_delegate, '_update_reference_navigation_buttons'):
            _reference_navigation_delegate._update_reference_navigation_buttons()
        return True
    else:
        return False

def navigate_reference_history_next():
    """Navigate to next reference in global history."""
    global _global_reference_history, _global_reference_history_index
    
    if _global_reference_history_index < len(_global_reference_history) - 1:
        _global_reference_history_index += 1
        entry = _global_reference_history[_global_reference_history_index]
        prim_path = entry['prim_path']
        ui_context = entry.get('ui_context', {})
        
        # Set selection without adding to history
        from omni.usd import get_context
        get_context().get_selection().set_selected_prim_paths([prim_path], True)
        
        # Force property window rebuild after selection change
        import asyncio
        import omni.kit.app as kit_app
        async def delayed_rebuild_and_restore():
            # Wait for selection change to propagate
            for _ in range(3):
                await kit_app.get_app().next_update_async()
            
            # Force rebuild the property window
            try:
                import omni.kit.window.property as property_window
                window = property_window.get_window()
                if window and hasattr(window, '_window') and hasattr(window._window, 'frame'):
                    window._window.frame.rebuild()
            except Exception:
                pass
            
            # Wait for rebuild to complete
            for _ in range(3):
                await kit_app.get_app().next_update_async()
            
            # Then restore UI context
            _restore_ui_context(ui_context)
        asyncio.ensure_future(delayed_rebuild_and_restore())
        
        # Update button states
        if _reference_navigation_delegate and hasattr(_reference_navigation_delegate, '_update_reference_navigation_buttons'):
            _reference_navigation_delegate._update_reference_navigation_buttons()
        return True
    else:
        return False

def get_global_navigation_button_states():
    """Get the enabled states for navigation buttons."""
    global _global_reference_history, _global_reference_history_index
    prev_enabled = _global_reference_history_index > 0
    next_enabled = _global_reference_history_index < len(_global_reference_history) - 1
    return prev_enabled, next_enabled

def set_target_scroll_position(scroll_position):
    """Set the target scroll position for the next rebuild."""
    global _target_scroll_position
    _target_scroll_position = scroll_position

def get_target_scroll_position():
    """Get and clear the target scroll position."""
    global _target_scroll_position
    position = _target_scroll_position
    _target_scroll_position = None
    return position

def _capture_current_ui_context():
    """Capture current UI context including pagination and scroll state."""
    try:
        ui_context = {}
        
        # Try to capture current pagination state from the global pagination state
        from omni.physxpvd.scripts.property_widget.prim_handlers.default import _pagination_state
        if _pagination_state:
            ui_context['pagination_state'] = dict(_pagination_state)
        
        # Try to capture scroll position from the property window
        try:
            import omni.kit.window.property as property_window
            window = property_window.get_window()
            if window and hasattr(window, '_window_frame'):
                ui_context['scroll_position'] = window._window_frame.scroll_y
        except Exception:
            pass
        
        return ui_context
    except Exception:
        return {}

def _restore_ui_context(ui_context):
    """Restore UI context including pagination and scroll state."""
    try:
        # Restore pagination state
        if 'pagination_state' in ui_context:
            try:
                from omni.physxpvd.scripts.property_widget.prim_handlers import default
                # Update the global pagination state
                default._pagination_state.clear()
                default._pagination_state.update(ui_context['pagination_state'])
            except Exception:
                pass
        
        # Set target scroll position and trigger UI rebuild
        if 'scroll_position' in ui_context:
            # Set the target scroll position for the rebuild function to use
            set_target_scroll_position(ui_context['scroll_position'])
        
        # Trigger UI rebuild to apply pagination state
        try:
            from omni.physxpvd.scripts.property_widget.prim_handlers.default import _preserve_scroll_position_and_rebuild
            _preserve_scroll_position_and_rebuild()
        except Exception:
            pass
            
    except Exception:
        pass

def get_time():
    timeline = get_timeline_interface()
    return timeline.get_current_time() * timeline.get_time_codes_per_seconds()

def select_by_object_handle(target_handle):
    """
    Select a prim by its object handle.
    OPTIMIZED: Uses cached handle-to-path mapping instead of full stage traversal.
    """
    global _handle_prim_cache
    
    stage = get_context().get_stage()
    if not stage:
        return
    
    curr_time = get_time()
    
    # Ensure cache is valid and use it for lookup
    _handle_prim_cache.ensure_valid(stage)
    
    # Get cached paths for this handle (much faster than full traversal)
    cached_paths = _handle_prim_cache.get_paths_for_handle(target_handle)
    
    for prim_path in cached_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            continue
        
        # Check visibility at current time
        if prim.HasAttribute("omni:pvdi:viz"):
            if not prim.GetAttribute("omni:pvdi:viz").Get(curr_time):
                continue
        else:
            continue
        
        prim_path_str = str(prim_path)
        
        # Add to reference navigation history
        try:
            # Capture current UI context (pagination, scroll position)
            ui_context = _capture_current_ui_context()
            
            # First, add the current selection (source object) to history with UI context
            current_selection = get_context().get_selection().get_selected_prim_paths()
            if current_selection:
                current_prim_path = current_selection[0]
                add_to_global_reference_history(current_prim_path, ui_context)
            
            # Then set the new selection (target object)
            get_context().get_selection().set_selected_prim_paths([prim_path_str], True)
            
            # And add the target to history as well (without UI context since it's the destination)
            add_to_global_reference_history(prim_path_str)
                
        except Exception:
            pass
        
        return

def get_value_with_prefix(prim, name, prefix):
    name_with_prefix = prefix + name
    if not prim.HasAttribute(name_with_prefix):
        return None
    return prim.GetAttribute(name_with_prefix).Get(get_time())

def get_value(prim, name):
    return get_value_with_prefix(prim, name, OMNI_PVD_ATTRIBUTE_DATA_PREFIX)

def get_class_name(prim):
    return get_value_with_prefix(prim, 'class', OMNI_PVD_METADATA_PREFIX)

def get_object_handle(prim):
    return get_value_with_prefix(prim, 'handle', OMNI_PVD_METADATA_PREFIX)

def convert_from_camel_case(name, prefix):
    name = name[len(prefix):]
    words = sub('([A-Z][a-z]+)', r' \1', sub('([A-Z]+)', r' \1', name)).split()
    return ' '.join([word[0].upper() + word[1:] for word in words])

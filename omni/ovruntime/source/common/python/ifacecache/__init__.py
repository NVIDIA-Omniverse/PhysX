# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Cached Carbonite interface accessors for the physics extensions.

Every extension caches the interfaces it acquires so the accessors stay cheap. The
cache lives on the getter function object, which outlives the extension, so it has
to be dropped when the extension releases the interfaces - otherwise the accessor
hands back a pointer into a freed interface struct and dispatching through it
crashes with an access violation (NVBugs 6418297).

Each extension links its own copy of this module into its own Python namespace, so
the registry below only ever holds that extension's accessors and its on_shutdown
clears exactly those.

Usage:

    from .ifacecache import cached_interface, release_interface_caches

    def get_foo_interface() -> IFoo:
        return cached_interface(get_foo_interface, acquire_foo_interface)

    class FooExtension(omni.ext.IExt):
        def on_shutdown(self):
            release_foo_interface(self._foo_interface)
            self._foo_interface = None
            release_interface_caches()
"""

__all__ = ["cached_interface", "release_interface_caches"]


# Accessors currently holding a cached interface.
_cached_getters = set()


def cached_interface(func, acq):
    """Return func's cached interface, acquiring it on first use.

    Pass the accessor itself as func, so the cache is stored on it.
    """
    if not hasattr(func, "iface"):
        # carb raises rather than handing back null when the plugin is unavailable, so a
        # failed acquire leaves the cache empty instead of poisoning it with a bad value.
        func.iface = acq()
        _cached_getters.add(func)
    return func.iface


def release_interface_caches():
    """Drop the cached interface pointers so a later call re-acquires them.

    The interfaces are freed once the extension releases them. Anything reaching an
    accessor after that - a reload, an extension disable, or a consumer that simply
    shuts down later - would otherwise dispatch through a dangling pointer.
    """
    for func in _cached_getters:
        if hasattr(func, "iface"):
            del func.iface
    _cached_getters.clear()

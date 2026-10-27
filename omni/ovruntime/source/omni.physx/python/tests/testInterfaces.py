# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.test


class TestPhysxInterfaces(omni.kit.test.AsyncTestCaseFailOnLogError):
    async def test_get_physx_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_simulation_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_simulation_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_cooking_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_cooking_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_scene_query_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_scene_query_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_visualization_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_visualization_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_stage_update_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_stage_update_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_statistics_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_statistics_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_replicator_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_replicator_interface()
        self.assertIsNotNone(iface)

    async def test_get_physx_benchmarks_interface(self):
        import omni.physx
        iface = omni.physx.get_physx_benchmarks_interface()
        self.assertIsNotNone(iface)

    async def test_interface_caches_released(self):
        # Regression for NVBugs 6418297 (and 6224575 / 6224632). The accessors cache the
        # acquired interface on the getter function object. on_shutdown releases the
        # interfaces, which frees the carb function-pointer struct, so the cache must be
        # dropped too - otherwise the next call hands back a dangling pointer and
        # dispatching through it crashes with an exec access violation.
        import types
        from omni.physx import ifacecache
        from omni.physx.scripts import ifaces
        from omni.physx.scripts.extension import PhysxExtension

        # Collect every accessor, so a getter added later cannot silently skip the registry.
        getters = [
            getattr(ifaces, name)
            for name in dir(ifaces)
            if name.startswith("get_") and name.endswith("_interface")
        ]
        self.assertTrue(getters)

        for getter in getters:
            self.assertIsNotNone(getter(), f"{getter.__name__} returned no interface")
            self.assertTrue(hasattr(getter, "iface"), f"{getter.__name__} did not cache its interface")

        release_interface_caches = getattr(ifaces, "release_interface_caches", None)
        self.assertIsNotNone(release_interface_caches, "ifaces must expose release_interface_caches()")

        # Dropping a cache does not release the interface - the extension's own
        # release_physx_*() call balances the acquire - so put the caches back afterwards.
        # Re-acquiring here instead would leave this process with more acquires than
        # releases and keep a half-torn-down plugin alive.
        saved = {getter: getter.iface for getter in getters}
        try:
            release_interface_caches()

            for getter in getters:
                self.assertFalse(hasattr(getter, "iface"), f"{getter.__name__} kept a stale cached interface")
        finally:
            for getter, iface in saved.items():
                getter.iface = iface
                ifacecache._cached_getters.add(getter)

        # Exposing the function is not enough, the extension has to actually call it.
        # Scan every method of the class so extracting a helper does not break this.
        def referenced_names(code):
            names = set(code.co_names)
            for const in code.co_consts:
                if isinstance(const, types.CodeType):
                    names |= referenced_names(const)
            return names

        called = set()
        for attr in vars(PhysxExtension).values():
            code = getattr(attr, "__code__", None)
            if code is not None:
                called |= referenced_names(code)
        self.assertIn("release_interface_caches", called, "PhysxExtension must drop the caches")

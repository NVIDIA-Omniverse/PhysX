# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.test


class TestConvexDecompositionInterfaces(omni.kit.test.AsyncTestCaseFailOnLogError):
    async def test_acquire_convexdecomposition_interface(self):
        from omni.convexdecomposition.bindings import _convexdecomposition
        iface = _convexdecomposition.acquire_convexdecomposition_interface()
        self.assertIsNotNone(iface)

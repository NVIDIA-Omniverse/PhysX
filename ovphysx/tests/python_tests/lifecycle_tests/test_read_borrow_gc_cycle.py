# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-5 AC-9

"""An unreachable SDK/read-array cycle must still destroy its native instance."""

import gc
import warnings
import weakref

from ovphysx import PhysX
import ovphysx.api as api
from ovphysx.types import ApiStatus, SimObjectType
from test_utils import attach_usd_with_ovstage, data_path


def test_unreachable_read_array_cycle_destroys_native_instance(monkeypatch):
    PhysX.set_cpu_mode(True)
    sdk = PhysX()
    stage = result = None
    destroyed_results = []
    native_destroy = api._lib.ovphysx_destroy_instance

    def record_destroy(handle):
        status = native_destroy(handle)
        destroyed_results.append((handle, status.status))
        return status

    monkeypatch.setattr(api._lib, "ovphysx_destroy_instance", record_destroy)
    try:
        stage = attach_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
        result = sdk.read_sync(SimObjectType.RIGID_BODY, ["position"])
        sdk.retained_read_array = result.groups[0].tensors[0]
        result.close()
        result = None
        handle = sdk.handle
        instance_ref = weakref.ref(sdk)
        assert any(holder.has_live_borrows for holder in sdk._live_read_holders)

        with warnings.catch_warnings(record=True) as emitted:
            warnings.simplefilter("always")
            sdk = None
            gc.collect()
        assert instance_ref() is None
        assert destroyed_results == [(handle, ApiStatus.SUCCESS)]
        assert not [warning for warning in emitted if issubclass(warning.category, RuntimeWarning)]
        # Warp deleters hand their lifecycle token to the owning-thread drain.
        api._drain_lifecycle_releases_at_exit()
        assert api._PROCESS_LIFECYCLE_REFCOUNT == 0
    finally:
        result = None
        if sdk is not None:
            sdk.retained_read_array = None
            gc.collect()
            sdk.destroy()
        if stage is not None:
            stage.destroy()

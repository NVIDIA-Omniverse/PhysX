# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-API-001
# @covers AC-1 AC-2 AC-4
# @implements REQ-COMP-PYTHON-001
# @covers AC-3
"""Package compatibility declarations; importing these initializes no provider.

Actual availability is negotiated from the loaded service, not inferred from a
package version. Every listed C table requires exact version 1 and sizeof(table).
Native C++ consumers additionally require the installed SDK build/compiler ABI.
See docs/compatibility.md for per-operation modes and customer USD qualification.
"""
from types import MappingProxyType

from . import __version__

# Qualified with the core from the same tree; both versions come from ovphysx/VERSION.
QUALIFIED_CORE_VERSION=__version__
QUALIFIED_CUSTOMER_USD=(0,26,8)
# name -> (binding module, ctypes layout, ABI). Python names are package-relative.
SERVICE_ABIS=MappingProxyType({
    '':('native','_Api',1),
    'preview':('preview','_PreviewApi',1),
    'picking':('interaction','_PickingApi',1),
    'mesh':('mesh','_MeshApi',1),
    'mesh_motion':('mesh','_MotionApi',1),
    'mesh_filter':('mesh','_FilterApi',1),
    'native':('plugins','_Api',1),
    'native_capabilities':('plugins','_CapabilitiesApi',1),
    **{name:(name,'_Api',1) for name in ('cct','mass','inspection','vehicle','queries','interaction',
                                      'events','particles','debug','ovd','configuration','cooking')},
})

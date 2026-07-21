# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from ovphysx.types import (
    ApiStatus,
    SceneQueryGeometryType,
    SceneQueryMode,
    TensorType,
)


def test_module_all_exports():
    """Test module __all__ contains expected exports."""
    import ovphysx

    assert hasattr(ovphysx, "__all__"), "Module should define __all__"
    all_exports = ovphysx.__all__

    expected_exports = [
        "PhysX",
        "TensorType",
        "ApiStatus",
        "LogLevel",
        "BindingPrimMode",
        "SceneQueryMode",
        "SceneQueryGeometryType",
        "ContactEventHeader",
        "ContactPoint",
        "TensorBindingSpec",
        "DLDevice",
        "DLDataType",
        "DLTensor",
        "codeless_schema_paths",
        "codeless_schema_root",
    ]

    for export in expected_exports:
        assert export in all_exports, f"{export} should be in __all__"


def test_intenum_types_importable_without_native():
    """TensorType and friends must be importable without triggering native loading."""
    from ovphysx.types import (
        ApiStatus,
        BindingPrimMode,
        LogLevel,
        SceneQueryGeometryType,
        SceneQueryMode,
        TensorType,
    )

    assert TensorType.ARTICULATION_ROOT_POSE == 10
    assert ApiStatus.SUCCESS == 0
    assert LogLevel.WARNING == 2
    assert BindingPrimMode.EXISTING_ONLY == 0
    assert SceneQueryMode.CLOSEST == 0
    assert SceneQueryGeometryType.SPHERE == 0


def test_tensor_type_values():
    """Spot-check that key TensorType values match the C header."""
    assert TensorType.RIGID_BODY_POSE == 1
    assert TensorType.ARTICULATION_ROOT_POSE == 10
    assert TensorType.ARTICULATION_LINK_POSE == 20
    assert TensorType.ARTICULATION_DOF_POSITION == 30
    assert TensorType.ARTICULATION_DOF_STIFFNESS == 35
    assert TensorType.ARTICULATION_LINK_WRENCH == 52
    assert TensorType.ARTICULATION_BODY_MASS == 60
    assert TensorType.ARTICULATION_JACOBIAN == 70
    assert TensorType.ARTICULATION_FIXED_TENDON_STIFFNESS == 80
    assert TensorType.ARTICULATION_SPATIAL_TENDON_STIFFNESS == 90


def test_api_status_codes():
    """Test API status code IntEnum members."""
    assert isinstance(ApiStatus.SUCCESS, int)
    assert isinstance(ApiStatus.ERROR, int)
    assert ApiStatus.SUCCESS != ApiStatus.ERROR


def test_tensor_type_is_int():
    """TensorType members must work as plain ints for the C API."""
    assert isinstance(TensorType.ARTICULATION_ROOT_POSE, int)
    assert TensorType.ARTICULATION_ROOT_POSE + 1 == 11
    d = {TensorType.ARTICULATION_ROOT_POSE: "pose"}
    assert d[10] == "pose"


def test_tensor_type_repr():
    """IntEnum repr should show the name, not just the int."""
    assert "ROOT_POSE" in repr(TensorType.ARTICULATION_ROOT_POSE)


def test_tensor_type_iteration():
    """All TensorType members should be iterable."""
    members = list(TensorType)
    assert len(members) > 40


def test_operation_index_constants():
    """Test OP_INDEX_ALL is accessible via lazy loading."""
    import ovphysx

    assert hasattr(ovphysx, "OP_INDEX_ALL")
    assert isinstance(ovphysx.OP_INDEX_ALL, int)


def test_module_has_version_attribute():
    """Test module __version__ attribute is available without native loading."""
    import ovphysx

    assert hasattr(ovphysx, "__version__")
    version = ovphysx.__version__
    assert isinstance(version, str)
    assert len(version) > 0


def test_physx_class_importable():
    """Test PhysX class can be imported (triggers native loading)."""
    from ovphysx import PhysX

    assert PhysX is not None
    assert isinstance(PhysX, type)


def test_tensor_binding_spec_importable():
    """Test TensorBindingSpec can be imported (triggers native loading)."""
    from ovphysx import TensorBindingSpec

    assert TensorBindingSpec is not None


def test_dlpack_structures_importable():
    """Test DLPack structures can be imported (triggers native loading)."""
    from ovphysx import (
        DLDataType,
        DLDevice,
        DLManagedTensor,
        DLTensor,
        ManagedDLTensor,
    )

    assert DLDevice is not None
    assert DLDataType is not None
    assert DLTensor is not None
    assert DLManagedTensor is not None
    assert ManagedDLTensor is not None

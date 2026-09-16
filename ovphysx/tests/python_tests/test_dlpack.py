# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import ctypes
import gc

from ovphysx import (
    DLPACK_VERSION,
    DLDataType,
    DLDataTypeCode,
    DLDevice,
    DLDeviceType,
    DLManagedTensor,
    DLTensor,
)


def test_dlpack_dtype_code_constants():
    """Test DLPack data type code constants are accessible via DLDataTypeCode."""
    assert hasattr(DLDataTypeCode, "kDLInt"), "DLDataTypeCode should have kDLInt"
    assert hasattr(DLDataTypeCode, "kDLUInt"), "DLDataTypeCode should have kDLUInt"
    assert hasattr(DLDataTypeCode, "kDLFloat"), "DLDataTypeCode should have kDLFloat"
    assert isinstance(DLDataTypeCode.kDLInt, int)
    assert isinstance(DLDataTypeCode.kDLUInt, int)
    assert isinstance(DLDataTypeCode.kDLFloat, int)


def test_dlpack_device_type_constants():
    """Test DLPack device type constants are accessible via DLDeviceType."""
    assert hasattr(DLDeviceType, "kDLCPU"), "DLDeviceType should have kDLCPU"
    assert hasattr(DLDeviceType, "kDLCUDA"), "DLDeviceType should have kDLCUDA"
    assert isinstance(DLDeviceType.kDLCPU, int)
    assert isinstance(DLDeviceType.kDLCUDA, int)


def test_dlpack_structures_available():
    """Test DLPack structure classes are accessible.

    Covered APIs:
        DLDevice, DLDataType, DLTensor, DLManagedTensor

    Args:
        None

    Returns:
        None: Ensures DLPack structures are properly exported.
    """
    assert DLDevice is not None, "DLDevice should be accessible"
    assert DLDataType is not None, "DLDataType should be accessible"
    assert DLTensor is not None, "DLTensor should be accessible"
    assert DLManagedTensor is not None, "DLManagedTensor should be accessible"

    assert isinstance(DLPACK_VERSION, int), "DLPACK_VERSION should be an integer"
    assert DLPACK_VERSION > 0, "DLPACK_VERSION should be positive"


def test_dldevice_creation():
    """Test DLDevice structure creation and field access.

    Covered APIs:
        DLDevice.__init__
        DLDevice.device_type
        DLDevice.device_id

    Args:
        None

    Returns:
        None: Ensures DLDevice can be created and accessed.
    """
    device = DLDevice()
    device.device_type = DLDeviceType.kDLCPU
    device.device_id = 0

    # device_type returns a ctypes enum, so .value gives the integer.
    assert device.device_type.value == DLDeviceType.kDLCPU, "Device type should be CPU"
    assert device.device_id == 0, "Device ID should be 0"


def test_dldatatype_creation():
    """Test DLDataType structure creation and field access.

    Covered APIs:
        DLDataType.__init__
        DLDataType.code
        DLDataType.bits
        DLDataType.lanes

    Args:
        None

    Returns:
        None: Ensures DLDataType can be created and accessed.
    """
    dtype = DLDataType()
    dtype.code = DLDataTypeCode.kDLFloat
    dtype.bits = 32
    dtype.lanes = 1

    # code returns a ctypes enum, so .value gives the integer.
    assert dtype.code.value == DLDataTypeCode.kDLFloat, "Data type code should be float"
    assert dtype.bits == 32, "Bits should be 32"
    assert dtype.lanes == 1, "Lanes should be 1"


# =============================================================================
# String Representation Tests
# These __str__ implementations are used in production error messages and debugging.
# Testing them ensures readable output when users encounter DLPack-related issues.
# =============================================================================


def test_dldevice_type_string_representation():
    """Test DLDeviceType __str__ method for all device types.

    Covered APIs:
        DLDeviceType.__str__
        String conversion for all device type constants

    Args:
        None

    Returns:
        None: Ensures DLDeviceType objects stringify correctly.
    """
    device_tests = [
        (DLDeviceType.kDLCPU, "CPU"),
        (DLDeviceType.kDLCUDA, "CUDA"),
        (DLDeviceType.kDLCUDAHost, "CUDAHost"),
        (DLDeviceType.kDLOpenCL, "OpenCL"),
        (DLDeviceType.kDLVulkan, "Vulkan"),
        (DLDeviceType.kDLMetal, "Metal"),
        (DLDeviceType.kDLROCM, "ROCM"),
        (DLDeviceType.kDLCUDAManaged, "CUDAManaged"),
    ]

    for device_value, expected_str in device_tests:
        device = DLDeviceType(device_value)
        assert str(device) == expected_str, f"Device {device_value} should stringify to {expected_str}"


def test_dldatatype_code_string_representation():
    """Test DLDataTypeCode __str__ method for all data type codes.

    Covered APIs:
        DLDataTypeCode.__str__
        String conversion for all data type codes

    Args:
        None

    Returns:
        None: Ensures DLDataTypeCode objects stringify correctly.
    """
    datatype_tests = [
        (DLDataTypeCode.kDLInt, "int"),
        (DLDataTypeCode.kDLUInt, "uint"),
        (DLDataTypeCode.kDLFloat, "float"),
        (DLDataTypeCode.kDLBfloat, "bfloat"),
        (DLDataTypeCode.kDLComplex, "complex"),
        (DLDataTypeCode.kDLOpaqueHandle, "void_p"),
    ]

    for code_value, expected_str in datatype_tests:
        dtype_code = DLDataTypeCode(code_value)
        assert str(dtype_code) == expected_str, f"DataType {code_value} should stringify to {expected_str}"


def test_dldevice_string_representation():
    """Test DLDevice __str__ method.

    Covered APIs:
        DLDevice.__str__
        DLDevice string formatting

    Args:
        None

    Returns:
        None: Ensures DLDevice objects have readable string representation.
    """
    # device_id 0 stringifies to just the device type name.
    device = DLDevice()
    device.device_type = DLDeviceType.kDLCPU
    device.device_id = 0

    device_str = str(device)
    assert device_str == "CPU", f"Device with id 0 should stringify to 'CPU', got '{device_str}'"

    # A non-zero device_id stringifies as "TYPE:<id>".
    device.device_id = 3
    device_str = str(device)
    assert device_str == "CPU:3", f"Device with id 3 should stringify to 'CPU:3', got '{device_str}'"


def test_dldatatype_string_representation():
    """Test DLDataType __str__ method.

    Covered APIs:
        DLDataType.__str__
        DLDataType string formatting

    Args:
        None

    Returns:
        None: Ensures DLDataType objects have readable string representation.
    """
    dtype = DLDataType()
    dtype.code = DLDataTypeCode.kDLFloat
    dtype.bits = 32
    dtype.lanes = 1

    dtype_str = str(dtype)
    assert "float" in dtype_str.lower(), "DataType string should contain type name"
    assert "32" in dtype_str, "DataType string should contain bit width"


def test_dldatatype_code_compares_equal_to_constants():
    """DLDataType.code should compare directly with DLDataTypeCode constants."""
    dtype = DLDataType()
    dtype.code = DLDataTypeCode.kDLFloat
    dtype.bits = 32
    dtype.lanes = 1

    assert dtype.code == DLDataTypeCode.kDLFloat
    assert dtype.code != DLDataTypeCode.kDLUInt
    assert dtype.code != "2"
    assert dtype.code != 2.7


def test_dldatatype_code_converts_to_integer():
    """DLDataType.code should convert to its integer DLPack type code."""
    dtype = DLDataType()
    dtype.code = DLDataTypeCode.kDLFloat

    assert int(dtype.code) == DLDataTypeCode.kDLFloat


def test_dltensor_structure_fields():
    """Test DLTensor structure field access and initialization.

    Covered APIs:
        DLTensor structure fields
        DLTensor.data, .device, .ndim, .dtype, .shape, .strides, .byte_offset

    Args:
        None

    Returns:
        None: Ensures DLTensor structure is properly defined.
    """
    tensor = DLTensor()

    tensor.device.device_type = DLDeviceType.kDLCPU
    tensor.device.device_id = 0

    tensor.dtype.code = DLDataTypeCode.kDLFloat
    tensor.dtype.bits = 32
    tensor.dtype.lanes = 1

    tensor.ndim = 2
    shape_array = (ctypes.c_int64 * 2)(3, 4)
    tensor.shape = ctypes.cast(shape_array, ctypes.POINTER(ctypes.c_int64))

    # Enum fields return a ctypes enum, so .value gives the integer.
    assert tensor.device.device_type.value == DLDeviceType.kDLCPU, "Device type should match"
    assert tensor.dtype.code.value == DLDataTypeCode.kDLFloat, "Data type code should match"
    assert tensor.ndim == 2, "Dimensions should be 2"


def test_managed_dltensor_structure():
    """Test the low-level DLManagedTensor Structure.

    Covered APIs:
        DLManagedTensor Structure (low-level)

    Args:
        None

    Returns:
        None: Ensures the low-level Structure exposes the DLPack fields.
    """
    dl_managed = DLManagedTensor()

    assert hasattr(dl_managed, "dl_tensor"), "DLManagedTensor Structure should have dl_tensor field"
    assert hasattr(dl_managed, "manager_ctx"), "DLManagedTensor Structure should have manager_ctx field"
    assert hasattr(dl_managed, "deleter"), "DLManagedTensor Structure should have deleter field"


def test_dlpack_version_tuple():
    """Test DLPACK_VERSION is an integer representing packed version.

    Covered APIs:
        DLPACK_VERSION integer format

    Args:
        None

    Returns:
        None: Ensures DLPACK_VERSION is accessible and valid.

    Note:
        The API returns DLPACK_VERSION as a packed integer (e.g., 128 for version 1.0),
        not as a tuple. This is the actual implementation behavior.
    """
    assert isinstance(DLPACK_VERSION, int), "DLPACK_VERSION should be an integer"
    assert DLPACK_VERSION > 0, "DLPACK_VERSION should be positive"

    assert DLPACK_VERSION < 10000, "DLPACK_VERSION should be a reasonable packed version number"


def test_device_type_constants_values():
    """Test DLDeviceType constants have expected values per DLPack spec."""
    assert DLDeviceType.kDLCPU == 1, "kDLCPU should be 1 per DLPack spec"
    assert DLDeviceType.kDLCUDA == 2, "kDLCUDA should be 2 per DLPack spec"


def test_datatype_constants_values():
    """Test DLDataTypeCode constants have expected values per DLPack spec."""
    assert DLDataTypeCode.kDLInt == 0, "kDLInt should be 0 per DLPack spec"
    assert DLDataTypeCode.kDLUInt == 1, "kDLUInt should be 1 per DLPack spec"
    assert DLDataTypeCode.kDLFloat == 2, "kDLFloat should be 2 per DLPack spec"


# ============================================================================
# DLPack Utilities Tests (_dlpack_utils.py)
# ============================================================================


def test_numpy_to_dltensor_float32():
    """Test numpy_to_dltensor with float32 array.

    Covered APIs:
        ovphysx._dlpack_utils.numpy_to_dltensor
        DLTensor structure creation from NumPy

    Args:
        None

    Returns:
        None: Ensures float32 NumPy arrays convert correctly.
    """
    import numpy as np
    from ovphysx._dlpack_utils import numpy_to_dltensor

    arr = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    dl_tensor = numpy_to_dltensor(arr)

    assert dl_tensor.ndim == 2, "Should have 2 dimensions"
    assert dl_tensor.dtype.code.value == DLDataTypeCode.kDLFloat, "Should be float type"
    assert dl_tensor.dtype.bits == 32, "Should be 32 bits"
    assert dl_tensor.device.device_type.value == DLDeviceType.kDLCPU, "Should be CPU device"

    shape = [dl_tensor.shape[i] for i in range(dl_tensor.ndim)]
    assert shape == [2, 2], "Shape should be [2, 2]"


def test_numpy_to_dltensor_int32():
    """Test numpy_to_dltensor with int32 array.

    Covered APIs:
        ovphysx._dlpack_utils.numpy_to_dltensor
        DLTensor int32 dtype handling

    Args:
        None

    Returns:
        None: Ensures int32 NumPy arrays convert correctly.
    """
    import numpy as np
    from ovphysx._dlpack_utils import numpy_to_dltensor

    arr = np.array([1, 2, 3, 4], dtype=np.int32)
    dl_tensor = numpy_to_dltensor(arr)

    assert dl_tensor.ndim == 1, "Should have 1 dimension"
    assert dl_tensor.dtype.code.value == DLDataTypeCode.kDLInt, "Should be int type"
    assert dl_tensor.dtype.bits == 32, "Should be 32 bits"


def test_numpy_to_dltensor_float64():
    """Test numpy_to_dltensor with float64 array.

    Covered APIs:
        ovphysx._dlpack_utils.numpy_to_dltensor
        DLTensor float64 dtype handling

    Args:
        None

    Returns:
        None: Ensures float64 NumPy arrays convert correctly.
    """
    import numpy as np
    from ovphysx._dlpack_utils import numpy_to_dltensor

    arr = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    dl_tensor = numpy_to_dltensor(arr)

    assert dl_tensor.ndim == 1, "Should have 1 dimension"
    assert dl_tensor.dtype.code.value == DLDataTypeCode.kDLFloat, "Should be float type"
    assert dl_tensor.dtype.bits == 64, "Should be 64 bits"


def test_numpy_to_dltensor_non_contiguous():
    """Test numpy_to_dltensor rejects non-contiguous arrays.

    Covered APIs:
        ovphysx._dlpack_utils.numpy_to_dltensor
        Non-contiguous array error handling

    Args:
        None

    Returns:
        None: Ensures ValueError is raised for non-C-contiguous arrays.
    """
    import numpy as np
    import pytest
    from ovphysx._dlpack_utils import numpy_to_dltensor

    arr = np.array([[1, 2], [3, 4]], dtype=np.float32).T
    assert not arr.flags["C_CONTIGUOUS"], "Array should not be C-contiguous"

    with pytest.raises(ValueError, match="Array must be C-contiguous"):
        numpy_to_dltensor(arr)


def test_numpy_to_dltensor_unsupported_dtype():
    """Test numpy_to_dltensor rejects unsupported dtypes.

    Covered APIs:
        ovphysx._dlpack_utils.numpy_to_dltensor
        Unsupported dtype error handling

    Args:
        None

    Returns:
        None: Ensures ValueError is raised for unsupported dtypes like complex64.
    """
    import numpy as np
    import pytest
    from ovphysx._dlpack_utils import numpy_to_dltensor

    arr = np.array([1 + 2j, 3 + 4j], dtype=np.complex64)

    with pytest.raises(ValueError, match=r"Unsupported dtype"):
        numpy_to_dltensor(arr)


def test_numpy_to_dltensor_keepalive_attribute():
    """Test numpy_to_dltensor sets _keepalive attribute.

    Covered APIs:
        ovphysx._dlpack_utils.numpy_to_dltensor
        _keepalive attribute for GC protection

    Args:
        None

    Returns:
        None: Ensures _keepalive attribute is set to prevent GC.
    """
    import numpy as np
    from ovphysx._dlpack_utils import numpy_to_dltensor

    arr = np.array([1.0, 2.0], dtype=np.float32)
    dl_tensor = numpy_to_dltensor(arr)

    assert hasattr(dl_tensor, "_keepalive"), "Should have _keepalive attribute"
    assert isinstance(dl_tensor._keepalive, tuple), "_keepalive should be a tuple"
    assert len(dl_tensor._keepalive) == 3, "_keepalive should have 3 elements"


def test_acquire_dltensor_direct():
    """Test acquire_dltensor with direct DLTensor input.

    Covered APIs:
        ovphysx._dlpack_utils.acquire_dltensor
        DLTensor passthrough path

    Args:
        None

    Returns:
        None: Ensures DLTensor objects are returned as-is.
    """
    from ovphysx._dlpack_utils import acquire_dltensor

    dl_tensor = DLTensor()
    dl_tensor.ndim = 1

    result_tensor, keepalive = acquire_dltensor(dl_tensor)

    assert result_tensor is dl_tensor, "Should return the same DLTensor object"
    assert keepalive is None, "Keepalive should be None for direct DLTensor"


def test_acquire_dltensor_numpy_array():
    """Test acquire_dltensor with NumPy array via __dlpack__ protocol.

    Covered APIs:
        ovphysx._dlpack_utils.acquire_dltensor
        NumPy __dlpack__() protocol path

    Args:
        None

    Returns:
        None: Ensures NumPy arrays are converted via __dlpack__() protocol.

    Note:
        Modern NumPy (>= 1.22) supports __dlpack__(), so that path is used
        instead of __array_interface__. The capsule must be kept alive.
    """
    import numpy as np
    from ovphysx._dlpack_utils import acquire_dltensor

    arr = np.array([1.0, 2.0, 3.0], dtype=np.float32)

    result_tensor, keepalive = acquire_dltensor(arr)

    assert isinstance(result_tensor, DLTensor), "Should return a DLTensor"
    assert result_tensor.ndim == 1, "Should have 1 dimension"
    # NumPy uses __dlpack__() which returns a capsule that must be kept alive
    assert keepalive is not None, "Keepalive capsule should be returned for __dlpack__() path"


def test_copy_dltensor_owns_shape_and_strides():
    """Copied descriptors must not alias producer-owned metadata."""
    from ovphysx._dlpack_utils import copy_dltensor

    storage = (ctypes.c_float * 6)(*range(6))
    shape = (ctypes.c_int64 * 2)(2, 3)
    strides = (ctypes.c_int64 * 2)(3, 1)
    source = DLTensor()
    source.data = ctypes.addressof(storage)
    source.device.device_type = DLDeviceType.kDLCPU
    source.device.device_id = 0
    source.ndim = 2
    source.dtype.code = DLDataTypeCode.kDLFloat
    source.dtype.bits = 32
    source.dtype.lanes = 1
    source.shape = shape
    source.strides = strides
    source.byte_offset = ctypes.sizeof(ctypes.c_float)

    source_address = ctypes.addressof(source)
    copied = copy_dltensor(source)
    source.strides = None
    copied_without_strides = copy_dltensor(source)

    assert ctypes.addressof(copied) != source_address
    assert copied.data == ctypes.addressof(storage)
    assert copied.device.device_type.value == DLDeviceType.kDLCPU
    assert copied.device.device_id == 0
    assert copied.ndim == 2
    assert copied.dtype.code.value == source.dtype.code.value
    assert copied.dtype.bits == source.dtype.bits
    assert copied.dtype.lanes == source.dtype.lanes
    assert copied.byte_offset == ctypes.sizeof(ctypes.c_float)
    assert ctypes.cast(copied.shape, ctypes.c_void_p).value != ctypes.addressof(shape)

    del source
    del shape
    del strides
    gc.collect()

    assert tuple(copied.shape[i] for i in range(copied.ndim)) == (2, 3)
    assert tuple(copied.strides[i] for i in range(copied.ndim)) == (3, 1)
    assert tuple(copied_without_strides.shape[i] for i in range(copied_without_strides.ndim)) == (2, 3)
    assert not copied_without_strides.strides


def test_detect_data_ptr_uses_warp_public_ptr():
    """Warp cache validation must re-read the array's public ptr value."""
    from ovphysx.api import _detect_data_ptr

    warp_like_type = type("WarpLike", (), {"__module__": "warp.fake"})
    tensor = warp_like_type()
    tensor.ptr = 1234

    data_ptr, ptr_getter = _detect_data_ptr(tensor)

    assert data_ptr == 1234
    tensor.ptr = 5678
    assert ptr_getter(tensor) == 5678


def test_acquire_dltensor_non_contiguous_dlpack():
    """Test acquire_dltensor rejects non-contiguous __dlpack__ producers.

    Covered APIs:
        ovphysx._dlpack_utils.acquire_dltensor
        __dlpack__ contiguity validation

    Args:
        None

    Returns:
        None: Ensures ValueError is raised for non-C-contiguous DLPack tensors.
    """
    import numpy as np
    import pytest
    from ovphysx._dlpack_utils import acquire_dltensor

    arr = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32).T
    assert hasattr(arr, "__dlpack__"), "NumPy array should provide __dlpack__"
    assert not arr.flags["C_CONTIGUOUS"], "Array should be non-C-contiguous"

    with pytest.raises(Exception, match=r"(?i)C-contiguous|contiguous\(\)|contiguous input"):
        acquire_dltensor(arr)


def test_acquire_dltensor_invalid_object():
    """Test acquire_dltensor rejects incompatible objects.

    Covered APIs:
        ovphysx._dlpack_utils.acquire_dltensor
        TypeError for incompatible objects

    Args:
        None

    Returns:
        None: Ensures TypeError is raised for non-DLPack-compatible objects.
    """
    import pytest
    from ovphysx._dlpack_utils import acquire_dltensor

    with pytest.raises(TypeError, match=r"Object of type .* is not DLPack-compatible"):
        acquire_dltensor("not a tensor")

    with pytest.raises(TypeError, match=r"Object of type .* is not DLPack-compatible"):
        acquire_dltensor({"not": "a tensor"})


# ============================================================================
# Additional DLPack structure tests
# ============================================================================


def test_dl_device_gpu_types():
    """Test DLDevice with GPU device types.

    Covered APIs:
        DLDevice structure with CUDA/ROCM device types
        DLDeviceType GPU enum values

    Args:
        None

    Returns:
        None: Ensures GPU device types are properly defined.
    """
    from ovphysx import DLDevice, DLDeviceType

    cuda_device = DLDevice()
    cuda_device.device_type = DLDeviceType.kDLCUDA
    cuda_device.device_id = 0

    assert cuda_device.device_type.value == 2, "kDLCUDA should be 2"
    assert str(cuda_device) == "CUDA", "String representation should be 'CUDA'"

    cuda_device.device_id = 3
    assert str(cuda_device) == "CUDA:3", "Should include device ID in string"

    rocm_device = DLDevice()
    rocm_device.device_type = DLDeviceType.kDLROCM
    rocm_device.device_id = 0
    assert rocm_device.device_type.value == 10, "kDLROCM should be 10"

    assert hasattr(DLDeviceType, "kDLCUDAHost"), "Should have kDLCUDAHost"
    assert hasattr(DLDeviceType, "kDLVulkan"), "Should have kDLVulkan"
    assert hasattr(DLDeviceType, "kDLMetal"), "Should have kDLMetal"
    assert hasattr(DLDeviceType, "kDLWebGPU"), "Should have kDLWebGPU"


def test_dl_datatype_all_codes():
    """Test DLDataType with all data type codes.

    Covered APIs:
        DLDataType structure with various type codes
        DLDataTypeCode enum values (kDLInt, kDLUInt, kDLBfloat, kDLComplex)

    Args:
        None

    Returns:
        None: Ensures all data type codes are properly defined.
    """
    from ovphysx import DLDataType, DLDataTypeCode

    int_dtype = DLDataType()
    int_dtype.code = DLDataTypeCode.kDLInt
    int_dtype.bits = 32
    int_dtype.lanes = 1
    assert int_dtype.code.value == 0, "kDLInt should be 0"
    assert "int" in str(int_dtype).lower(), "String should contain 'int'"

    uint_dtype = DLDataType()
    uint_dtype.code = DLDataTypeCode.kDLUInt
    uint_dtype.bits = 8
    uint_dtype.lanes = 1
    assert uint_dtype.code.value == 1, "kDLUInt should be 1"

    bfloat_dtype = DLDataType()
    bfloat_dtype.code = DLDataTypeCode.kDLBfloat
    bfloat_dtype.bits = 16
    bfloat_dtype.lanes = 1
    assert bfloat_dtype.code.value == 4, "kDLBfloat should be 4"

    complex_dtype = DLDataType()
    complex_dtype.code = DLDataTypeCode.kDLComplex
    complex_dtype.bits = 64
    complex_dtype.lanes = 1
    assert complex_dtype.code.value == 5, "kDLComplex should be 5"

    vec_dtype = DLDataType()
    vec_dtype.code = DLDataTypeCode.kDLFloat
    vec_dtype.bits = 32
    vec_dtype.lanes = 4
    dtype_str = str(vec_dtype)
    assert "32" in dtype_str and "4" in dtype_str, "Should indicate 32-bit with 4 lanes"


def test_dldevice_string_representation_all_types():
    """Test DLDevice __str__ for various device types.

    Covered APIs:
        DLDevice.__str__ method
        DLDeviceType string mapping

    Args:
        None

    Returns:
        None: Ensures string representation works for all device types.
    """
    from ovphysx import DLDevice, DLDeviceType

    test_cases = [
        (DLDeviceType.kDLCPU, 0, "CPU"),
        (DLDeviceType.kDLCUDA, 0, "CUDA"),
        (DLDeviceType.kDLCUDA, 1, "CUDA:1"),
        (DLDeviceType.kDLOpenCL, 0, "OpenCL"),
        (DLDeviceType.kDLVulkan, 0, "Vulkan"),
        (DLDeviceType.kDLMetal, 2, "Metal:2"),
        (DLDeviceType.kDLROCM, 0, "ROCM"),
    ]

    for device_type, device_id, expected in test_cases:
        device = DLDevice()
        device.device_type = device_type
        device.device_id = device_id
        result = str(device)
        assert result == expected, f"Device {device_type} with ID {device_id} should be '{expected}', got '{result}'"


def test_dldatatype_string_representation_all_types():
    """Test DLDataType __str__ for various data types.

    Covered APIs:
        DLDataType.__str__ method
        DLDataType.TYPE_MAP

    Args:
        None

    Returns:
        None: Ensures string representation works for all data types.
    """
    from ovphysx import DLDataType, DLDataTypeCode

    type_specs = [
        (DLDataTypeCode.kDLInt, 8, 1, "int8"),
        (DLDataTypeCode.kDLInt, 32, 1, "int32"),
        (DLDataTypeCode.kDLInt, 64, 1, "int64"),
        (DLDataTypeCode.kDLUInt, 8, 1, "uint8"),
        (DLDataTypeCode.kDLUInt, 16, 1, "uint16"),
        (DLDataTypeCode.kDLFloat, 16, 1, "float16"),
        (DLDataTypeCode.kDLFloat, 32, 1, "float32"),
        (DLDataTypeCode.kDLFloat, 64, 1, "float64"),
        (DLDataTypeCode.kDLBfloat, 16, 1, "bfloat16"),
    ]

    for code, bits, lanes, expected in type_specs:
        dtype = DLDataType()
        dtype.code = code
        dtype.bits = bits
        dtype.lanes = lanes
        result = str(dtype)
        assert result == expected, f"Type {code}/{bits}bit/{lanes}lanes should be '{expected}', got '{result}'"


def test_dlpack_version_constant():
    """Test DLPACK_VERSION constant value and type.

    Covered APIs:
        DLPACK_VERSION constant

    Args:
        None

    Returns:
        None: Ensures DLPACK_VERSION is correctly defined.
    """
    from ovphysx import DLPACK_VERSION

    # DLPack version is packed as (major << 8) | minor.
    # Current bindings track dlpack.py constants (1.3 => 259).
    assert DLPACK_VERSION == 259, f"DLPACK_VERSION should be 259 ((1 << 8) | 3), got {DLPACK_VERSION}"
    assert isinstance(DLPACK_VERSION, int), "DLPACK_VERSION should be an integer"


def test_dldatatype_type_map():
    """Test DLDataType.TYPE_MAP contains expected mappings.

    Covered APIs:
        DLDataType.TYPE_MAP dictionary

    Args:
        None

    Returns:
        None: Ensures TYPE_MAP has all standard type mappings.
    """
    from ovphysx import DLDataType

    assert hasattr(DLDataType, "TYPE_MAP"), "DLDataType should have TYPE_MAP"
    type_map = DLDataType.TYPE_MAP

    expected_types = [
        "int8",
        "int16",
        "int32",
        "int64",
        "uint8",
        "uint16",
        "uint32",
        "uint64",
        "float16",
        "float32",
        "float64",
        "bfloat16",
    ]

    for type_name in expected_types:
        assert type_name in type_map, f"TYPE_MAP should contain '{type_name}'"
        code, bits, lanes = type_map[type_name]
        assert isinstance(bits, int), f"Bits for {type_name} should be int"
        assert isinstance(lanes, int), f"Lanes for {type_name} should be int"

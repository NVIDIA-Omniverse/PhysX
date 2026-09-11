// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-BINDING-DEVICE-001
// @covers AC-5

#ifndef OVPHYSX_EXPERIMENTAL_TENSOR_BINDING_HPP
#define OVPHYSX_EXPERIMENTAL_TENSOR_BINDING_HPP

//
// C++17 RAII wrapper for ovphysx_tensor_binding_handle_t.
// Provides automatic cleanup and thin C++ wrappers for the tensor binding C API functions.
//

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_export.h"

namespace ovphysx {

/**
 * @brief RAII wrapper for ovphysx_tensor_binding_handle_t
 *
 * Automatically calls ovphysx_destroy_tensor_binding on destruction.
 * Move-only (non-copyable) to ensure unique ownership.
 *
 * Created via PhysX::createTensorBinding(). Do not construct directly.
 *
 * Example:
 *   TensorBinding binding;
 *   physx.createTensorBinding(binding, "/World/robot", OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);
 *   binding.read(myTensor);
 *
 * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
 *   (reads) and @ref ovphysx_write (writes) instead. The action members below
 *   (spec/metadata/read/write/writeMasked/destroy) and the
 *   PhysX::createTensorBinding() factory carry the deprecation marker, so both
 *   creating and using a binding warn. The class type itself is left unmarked on
 *   purpose: a deprecated type warns wherever it is named, including this header's
 *   own factory declaration, which would warn every consumer merely for including
 *   the header. Member-function markers warn only on call.
 */
class OVPHYSX_API TensorBinding {
public:
    TensorBinding();
    ~TensorBinding();

    TensorBinding(TensorBinding&& other) noexcept;
    TensorBinding& operator=(TensorBinding&& other) noexcept;

    TensorBinding(const TensorBinding&) = delete;
    TensorBinding& operator=(const TensorBinding&) = delete;

    /// Get the raw binding handle (for use with C API)
    ovphysx_tensor_binding_handle_t handle() const { return m_bindingHandle; }

    /// Check whether this wrapper currently owns a non-null binding handle.
    /// This does not query whether the underlying TensorAPI view survived a
    /// stage reset or bound-object removal. read/write report that error.
    explicit operator bool() const { return m_bindingHandle != 0; }

    /// Query the tensor spec (dtype, ndim, shape)
    OVPHYSX_DEPRECATED_MSG("ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_api_status_t spec(ovphysx_tensor_spec_t& out_spec) const;

    /// Query the native DLPack device used by this binding
    OVPHYSX_DEPRECATED_MSG("ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_api_status_t nativeDevice(DLDevice& out_device) const;

    /// Query articulation topology metadata (dof_count, body_count, joint_count,
    /// fixed_tendon_count, spatial_tendon_count, is_fixed_base).
    /// Only valid for articulation bindings. Returns OVPHYSX_API_ERROR otherwise.
    OVPHYSX_DEPRECATED_MSG("ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_api_status_t metadata(ovphysx_articulation_metadata_t& out_metadata) const;

    /// Read simulation data into a DLTensor
    OVPHYSX_DEPRECATED_MSG("ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_api_status_t read(DLTensor& dst) const;

    /// Write data from a DLTensor, optionally with an index tensor for sparse updates
    OVPHYSX_DEPRECATED_MSG("ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_api_status_t write(const DLTensor& src, const DLTensor* indices = nullptr) const;

    /// Write data from a DLTensor using a boolean mask for selective updates
    OVPHYSX_DEPRECATED_MSG("ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_api_status_t writeMasked(const DLTensor& src, const DLTensor& mask) const;

    /// Explicitly destroy the binding (called automatically by destructor)
    OVPHYSX_DEPRECATED_MSG("ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    void destroy();

private:
    friend class PhysX;
    TensorBinding(ovphysx_handle_t instanceHandle, ovphysx_tensor_binding_handle_t bindingHandle);

    ovphysx_handle_t m_instanceHandle;
    ovphysx_tensor_binding_handle_t m_bindingHandle;
};

} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_TENSOR_BINDING_HPP

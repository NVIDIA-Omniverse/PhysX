// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Implementation of C++ RAII wrapper for ovphysx tensor binding
// See: ovphysx/experimental/TensorBinding.hpp

#include "ovphysx/experimental/TensorBinding.hpp"
#include "carb/logging/Log.h"

namespace ovphysx {

namespace {
inline bool ensureBinding(ovphysx_tensor_binding_handle_t h, const char* fn) {
    if (!h) {
        CARB_LOG_ERROR("%s: invalid tensor binding handle (null). Was createTensorBinding() successful?", fn);
        return false;
    }
    return true;
}
}

//------------------------------------------------------------------------------------------------------------
// Constructors / Destructor / Move
//------------------------------------------------------------------------------------------------------------

TensorBinding::TensorBinding()
    : m_instanceHandle(0), m_bindingHandle(0) {}

TensorBinding::TensorBinding(ovphysx_handle_t instanceHandle, ovphysx_tensor_binding_handle_t bindingHandle)
    : m_instanceHandle(instanceHandle), m_bindingHandle(bindingHandle) {}

TensorBinding::~TensorBinding() {
    destroy();
}

TensorBinding::TensorBinding(TensorBinding&& other) noexcept
    : m_instanceHandle(other.m_instanceHandle), m_bindingHandle(other.m_bindingHandle) {
    other.m_instanceHandle = 0;
    other.m_bindingHandle = 0;
}

TensorBinding& TensorBinding::operator=(TensorBinding&& other) noexcept {
    if (this != &other) {
        destroy();
        m_instanceHandle = other.m_instanceHandle;
        m_bindingHandle = other.m_bindingHandle;
        other.m_instanceHandle = 0;
        other.m_bindingHandle = 0;
    }
    return *this;
}

//------------------------------------------------------------------------------------------------------------
// Operations
//------------------------------------------------------------------------------------------------------------

ovphysx_api_status_t TensorBinding::spec(ovphysx_tensor_spec_t& out_spec) const {
    if (!ensureBinding(m_bindingHandle, "TensorBinding::spec")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_get_tensor_binding_spec(m_instanceHandle, m_bindingHandle, &out_spec);
    return r.status;
}

ovphysx_api_status_t TensorBinding::metadata(ovphysx_articulation_metadata_t& out_metadata) const {
    if (!ensureBinding(m_bindingHandle, "TensorBinding::metadata")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_get_articulation_metadata(m_instanceHandle, m_bindingHandle, &out_metadata);
    return r.status;
}

ovphysx_api_status_t TensorBinding::read(DLTensor& dst) const {
    if (!ensureBinding(m_bindingHandle, "TensorBinding::read")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_read_tensor_binding(m_instanceHandle, m_bindingHandle, &dst);
    return r.status;
}

ovphysx_api_status_t TensorBinding::write(const DLTensor& src, const DLTensor* indices) const {
    if (!ensureBinding(m_bindingHandle, "TensorBinding::write")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_write_tensor_binding(m_instanceHandle, m_bindingHandle, &src, indices);
    return r.status;
}

ovphysx_api_status_t TensorBinding::writeMasked(const DLTensor& src, const DLTensor& mask) const {
    if (!ensureBinding(m_bindingHandle, "TensorBinding::writeMasked")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_write_tensor_binding_masked(m_instanceHandle, m_bindingHandle, &src, &mask);
    return r.status;
}

void TensorBinding::destroy() {
    if (m_bindingHandle && m_instanceHandle) {
        ovphysx_destroy_tensor_binding(m_instanceHandle, m_bindingHandle);
    }
    m_bindingHandle = 0;
    m_instanceHandle = 0;
}

} // namespace ovphysx

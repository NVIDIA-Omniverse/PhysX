// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-HANDLE-001
 * @covers AC-4
 */

#pragma once

#include <carb/Defines.h>

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/stage.h>

// Session-layer edit guard (relocated from common/utilities/Utilities.h, ADR-0027): scopes an
// edit target onto `layer` and temporarily forces it editable.
class ScopedLayerEdit
{
public:
    ScopedLayerEdit(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfLayerHandle& layer)
        : m_usdEditCtx(stage, layer), m_layer(layer)
    {
        m_wasLayerEditable = m_layer->PermissionToEdit();
        m_layer->SetPermissionToEdit(true);
        CARB_ASSERT(m_layer->PermissionToEdit());
    }

    ~ScopedLayerEdit()
    {
        m_layer->SetPermissionToEdit(m_wasLayerEditable);
    }

private:
    PXR_NS::UsdEditContext m_usdEditCtx;
    PXR_NS::SdfLayerHandle m_layer;
    bool m_wasLayerEditable;
};

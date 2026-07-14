// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

class OmniPvdObject;

void addCleanName(std::string& primPath, char* objectName);
void createSDFPrimSpec(PXR_NS::SdfLayerRefPtr& layer, OmniPvdObject* omniPvdObject, bool isSharedLayer);
void createSDFPrimSpecPass(PXR_NS::SdfLayerRefPtr& layer, std::list<OmniPvdObject*>& objectCreations, bool isSharedLayer);
void createAndClearLayer(PXR_NS::SdfLayerRefPtr& sublayer, std::string& sublayerName);
void insertAsSublayer(PXR_NS::UsdStageRefPtr& stage, PXR_NS::SdfLayerRefPtr& sublayer, std::string& sublayerName);

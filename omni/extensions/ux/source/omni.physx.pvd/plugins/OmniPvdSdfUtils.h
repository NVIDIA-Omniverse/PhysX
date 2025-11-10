// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

class OmniPvdObject;

void addCleanName(std::string& primPath, char* objectName);
void createSDFPrimSpec(pxr::SdfLayerRefPtr& layer, OmniPvdObject* omniPvdObject, bool isSharedLayer);
void createSDFPrimSpecPass(pxr::SdfLayerRefPtr& layer, std::list<OmniPvdObject*>& objectCreations, bool isSharedLayer);
void createAndClearLayer(PXR_NS::SdfLayerRefPtr& sublayer, std::string& sublayerName);
void insertAsSublayer(pxr::UsdStageRefPtr& stage, PXR_NS::SdfLayerRefPtr& sublayer, std::string& sublayerName);

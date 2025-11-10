// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdDomUtils.h"
#include "OmniPvdUsdOverWriter.h"
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/dictionary.h>

void processTranslationOver(pxr::UsdGeomXformCache& xformCache,
                            pxr::UsdPrim* ancestorPrim,
                            pxr::UsdPrim* overPrim,
                            OmniPvdAttributeSample* attrib,
                            OmniPvdObject* omniPvdObject);

void processRotationOver(pxr::UsdGeomXformCache& xformCache,
                         pxr::UsdPrim* ancestorPrim,
                         pxr::UsdPrim* overPrim,
                         OmniPvdAttributeSample* attrib,
                         OmniPvdObject* omniPvdObject);

void processScaleOver(pxr::UsdGeomXformCache& xformCache,
                      pxr::UsdPrim* ancestorPrim,
                      pxr::UsdPrim* overPrim,
                      OmniPvdAttributeSample* attrib,
                      OmniPvdObject* omniPvdObject);

pxr::UsdPrim getTformAncestor(pxr::UsdPrim& prim);

void createPrimPassOver(pxr::UsdStageRefPtr* usdStage,
                        std::list<OmniPvdObject*>& objectCreations,
                        std::unordered_map<std::string, pxr::TfToken*>& tokenMap,
                        int isUSDA);

void createAttribPassOver(pxr::UsdStageRefPtr* usdStage,
                          std::list<OmniPvdObject*>& objectCreations,
                          std::unordered_map<std::string, pxr::TfToken*>& tokenMap,
                          int isUSDA);

void writeUSDFileOver(
    OmniPvdDOMState &domState
);

bool writeUSDFileOverWithLayerCreation(
    OmniPvdDOMState &domState,
    const std::string& inputStageFileAbsolutePath,
    const std::string& outputDir,
    const std::string& outputStageFile,
    float startTime,
    float stopTime,
    bool newLayersAreASCII,
    bool verifyOverLayer
);

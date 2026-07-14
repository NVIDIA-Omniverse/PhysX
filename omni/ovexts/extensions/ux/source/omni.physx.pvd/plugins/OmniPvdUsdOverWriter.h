// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "PvdDomUtils.h"
#include "OmniPvdUsdAdapter.h"
#include "OmniPvdUsdOverWriter.h"
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/dictionary.h>

void processTranslationOver(PXR_NS::UsdGeomXformCache& xformCache,
                            PXR_NS::UsdPrim* ancestorPrim,
                            PXR_NS::UsdPrim* overPrim,
                            OmniPvdAttributeSample* attrib,
                            OmniPvdObject* omniPvdObject);

void processRotationOver(PXR_NS::UsdGeomXformCache& xformCache,
                         PXR_NS::UsdPrim* ancestorPrim,
                         PXR_NS::UsdPrim* overPrim,
                         OmniPvdAttributeSample* attrib,
                         OmniPvdObject* omniPvdObject);

void processScaleOver(PXR_NS::UsdGeomXformCache& xformCache,
                      PXR_NS::UsdPrim* ancestorPrim,
                      PXR_NS::UsdPrim* overPrim,
                      OmniPvdAttributeSample* attrib,
                      OmniPvdObject* omniPvdObject);

PXR_NS::UsdPrim getTformAncestor(PXR_NS::UsdPrim& prim);

void createPrimPassOver(PXR_NS::UsdStageRefPtr* usdStage,
                        std::list<OmniPvdObject*>& objectCreations,
                        int isUSDA);

void createAttribPassOver(PXR_NS::UsdStageRefPtr* usdStage,
                          std::list<OmniPvdObject*>& objectCreations,
                          int isUSDA);

void createParticleAttribPassOver(PXR_NS::UsdStageRefPtr* usdStage,
                                  std::list<OmniPvdObject*>& objectCreations,
                                  int isUSDA);

void createDeformableAttribPassOver(PXR_NS::UsdStageRefPtr* usdStage,
                                    std::list<OmniPvdObject*>& objectCreations,
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

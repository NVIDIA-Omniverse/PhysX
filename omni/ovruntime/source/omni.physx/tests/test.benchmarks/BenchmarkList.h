// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#ifndef BENCHMARK_LIST_H
#define BENCHMARK_LIST_H

#include "framework/BmGlobals.h"

void bmInitialize(bool sanityCheck, const char* dataFolder, int32_t numThreads, bool forceGpu, bool profile,
    bool enableTracy, const char** kitArguments = NULL, uint32_t kitArgumentCount = 0);

void bmTerminate();
void bmGetRegister(std::vector<BmRegistrable*>&, const char* filterString = NULL, bool includeHidden = false);

#endif

// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_LIBRARY_FUNCTIONS_H
#define OMNI_PVD_LIBRARY_FUNCTIONS_H

#include "OmniPvdDefines.h"

class OmniPvdReader;
class OmniPvdWriter;

typedef OmniPvdReader* (OMNI_PVD_CALL *createOmniPvdReaderFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdReaderFp)(OmniPvdReader *reader);

typedef OmniPvdWriter* (OMNI_PVD_CALL *createOmniPvdWriterFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdWriterFp)(OmniPvdWriter *writer);

#endif
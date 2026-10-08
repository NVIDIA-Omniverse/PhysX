// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_LOG_H
#define OMNI_PVD_LOG_H

#include "OmniPvdDefines.h"

class OmniPvdLog {
public:
	OmniPvdLog();
	~OmniPvdLog();
	void setLogFunction(OmniPvdLogFunction logFunction);
	void outputLine(const char* fmt, ...);
	OmniPvdLogFunction mLogFunction;
};

#endif

// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_ERROR_CODES_H
#define PX_PVD_ERROR_CODES_H



#if !PX_DOXYGEN
namespace physx
{
namespace pvdsdk
{
#endif

struct PvdErrorType
{
	enum Enum
	{
		Success = 0,
		NetworkError,
		ArgumentError,
		Disconnect,
		InternalProblem
	};
};

typedef PvdErrorType::Enum PvdError;

#if !PX_DOXYGEN
}
}
#endif
#endif


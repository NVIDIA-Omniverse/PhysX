// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_CLIENT_H
#define PX_PVD_CLIENT_H

#include "foundation/PxFlags.h"
#include "foundation/PxVec3.h"

#if !PX_DOXYGEN
namespace physx
{
namespace pvdsdk
{
#endif

class PvdDataStream;
class PvdUserRenderer;

/**
\brief PvdClient is the per-client connection to PVD.
It provides callback when PVD is connected/disconnted.
It provides access to the internal object so that advanced users can create extension client.
*/
class PvdClient
{
  public:
	virtual PvdDataStream* getDataStream() = 0;

	virtual bool isConnected() const = 0;
	virtual void onPvdConnected() = 0;
	virtual void onPvdDisconnected() = 0;
	virtual void flush() = 0;

  protected:
	virtual ~PvdClient()
	{
	}
};

#if !PX_DOXYGEN
} // namespace pvdsdk
} // namespace physx
#endif

#endif


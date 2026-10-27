// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_COMM_STREAM_EVENT_SINK_H
#define PX_PVD_COMM_STREAM_EVENT_SINK_H

#include "PxPvdObjectModelBaseTypes.h"
#include "PxPvdCommStreamEvents.h"
#include "PxPvdCommStreamTypes.h"

namespace physx
{
namespace pvdsdk
{

class PvdCommStreamEventSink
{
  public:
	template <typename TStreamType>
	static void writeStreamEvent(const EventSerializeable& evt, PvdCommStreamEventTypes::Enum evtType, TStreamType& stream)
	{
		EventStreamifier<TStreamType> streamifier_concrete(stream);
		PvdEventSerializer& streamifier(streamifier_concrete);
		streamifier.streamify(evtType);
		const_cast<EventSerializeable&>(evt).serialize(streamifier);
	}
};

} // pvd
} // physx
#endif


// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxArticulationReducedCoordinate.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxMemory.h"
#include "ScArticulationSensor.h"
#include "NpBase.h"

#ifndef NP_ARTICULATION_SENSOR_H
#define NP_ARTICULATION_SENSOR_H

namespace physx
{

typedef PxU32	ArticulationSensorHandle;

class NpArticulationSensor : public PxArticulationSensor, public NpBase
{
public:

	NpArticulationSensor(PxArticulationLink* link, const PxTransform& relativePose);
	virtual ~NpArticulationSensor() {}

	virtual void release();
	virtual PxSpatialForce getForces() const;
	virtual PxTransform getRelativePose() const;
	virtual void setRelativePose(const PxTransform&);
	virtual PxArticulationLink* getLink() const;
	virtual PxU32 getIndex() const;
	virtual PxArticulationReducedCoordinate* getArticulation() const;
	virtual PxArticulationSensorFlags getFlags() const;
	virtual void setFlag(PxArticulationSensorFlag::Enum flag, bool enabled);

	PX_FORCE_INLINE Sc::ArticulationSensorCore& getSensorCore() { return mCore; }
	PX_FORCE_INLINE const Sc::ArticulationSensorCore& getSensorCore() const { return mCore; }
	PX_FORCE_INLINE ArticulationSensorHandle	getHandle() { return mHandle; }
	PX_FORCE_INLINE	void						setHandle(ArticulationSensorHandle handle) { mHandle = handle; }

	static		void								getBinaryMetaData(PxOutputStream& stream);

private:
	PxArticulationLink* mLink;
	Sc::ArticulationSensorCore mCore;
	ArticulationSensorHandle   mHandle;
};
}

#endif //NP_ARTICULATION_SENSOR_H

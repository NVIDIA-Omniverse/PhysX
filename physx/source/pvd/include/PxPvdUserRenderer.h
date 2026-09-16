// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_USER_RENDERER_H
#define PX_PVD_USER_RENDERER_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "common/PxRenderBuffer.h"
#include "pvd/PxPvd.h"

#include "PxPvdDataStream.h"
#include "foundation/PxUserAllocated.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPvd;

#if !PX_DOXYGEN
namespace pvdsdk
{
#endif

class RendererEventClient;

class PvdUserRenderer : public PxUserAllocated
{
  protected:
	virtual ~PvdUserRenderer()
	{
	}

  public:
	virtual void release() = 0;
	virtual void setClient(RendererEventClient* client) = 0;

	// Instance to associate the further rendering with.
	virtual void setInstanceId(const void* instanceId) = 0;
	// Draw these points associated with this instance
	virtual void drawPoints(const PxDebugPoint* points, uint32_t count) = 0;
	// Draw these lines associated with this instance
	virtual void drawLines(const PxDebugLine* lines, uint32_t count) = 0;
	// Draw these triangles associated with this instance
	virtual void drawTriangles(const PxDebugTriangle* triangles, uint32_t count) = 0;
	// Draw this text associated with this instance
	virtual void drawText(const PxDebugText& text) = 0;

	// Draw SDK debug render
	virtual void drawRenderbuffer(const PxDebugPoint* pointData, uint32_t pointCount, const PxDebugLine* lineData,
	                              uint32_t lineCount, const PxDebugTriangle* triangleData, uint32_t triangleCount) = 0;

	// Constraint visualization routines
	virtual void visualizeJointFrames(const PxTransform& parent, const PxTransform& child) = 0;
	virtual void visualizeLinearLimit(const PxTransform& t0, const PxTransform& t1, float value) = 0;
	virtual void visualizeAngularLimit(const PxTransform& t0, float lower, float upper) = 0;
	virtual void visualizeLimitCone(const PxTransform& t, float tanQSwingY, float tanQSwingZ) = 0;
	virtual void visualizeDoubleCone(const PxTransform& t, float angle) = 0;

	// Clear the immedate buffer.
	virtual void flushRenderEvents() = 0;

	static PvdUserRenderer* create(uint32_t bufferSize = 0x2000);
};

class RendererEventClient 
{
 public:
	virtual ~RendererEventClient(){}

	virtual void handleBufferFlush(const uint8_t* inData, uint32_t inLength) = 0;
};

#if !PX_DOXYGEN
}
}
#endif
#endif


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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.


#ifndef DY_DEFORMABLE_SURFACE_H
#define DY_DEFORMABLE_SURFACE_H

#include "foundation/PxSimpleTypes.h"
#include "DyDeformableSurfaceCore.h"
#include "PxvGeometry.h"

namespace physx
{
namespace Sc
{
	class DeformableSurfaceSim;
}

namespace Dy
{

typedef size_t DeformableSurfaceHandle;
struct DeformableSurfaceCore;

class DeformableSurface
{
	PX_NOCOPY(DeformableSurface)

public:
	DeformableSurface(Sc::DeformableSurfaceSim* sim, Dy::DeformableSurfaceCore& core) :
		mSim(sim), mCore(core), mElementId(0xffffffff), mGpuRemapId(0xffffffff)
	{}

	~DeformableSurface() {}

	PX_FORCE_INLINE void							setGpuRemapId(const PxU32 remapId)
	{
		mGpuRemapId = remapId;
		PxTriangleMeshGeometryLL& geom = mShapeCore->mGeometry.get<PxTriangleMeshGeometryLL>();
		geom.materialsLL.gpuRemapId = remapId;
	}

	PX_FORCE_INLINE PxTriangleMesh*					getTriangleMesh()
	{
		PxTriangleMeshGeometryLL& geom = mShapeCore->mGeometry.get<PxTriangleMeshGeometryLL>();
		return geom.triangleMesh;
	}

	PX_FORCE_INLINE PxU32							getGpuRemapId() { return mGpuRemapId; }

	PX_FORCE_INLINE void							setElementId(const PxU32 elementId) { mElementId = elementId; }
	PX_FORCE_INLINE PxU32							getElementId()						{ return mElementId; }

	PX_FORCE_INLINE PxsShapeCore&					getShapeCore() { return *mShapeCore; }
	PX_FORCE_INLINE void							setShapeCore(PxsShapeCore* shapeCore) { mShapeCore = shapeCore; }

	PX_FORCE_INLINE Sc::DeformableSurfaceSim*		getSim() const { return mSim; }
	PX_FORCE_INLINE const DeformableSurfaceCore&	getCore() const { return mCore; }
	PX_FORCE_INLINE DeformableSurfaceCore&			getCore() { return mCore; }

	PX_FORCE_INLINE PxU16							getIterationCounts() const { return mCore.solverIterationCounts; }

	void											addAttachmentHandle(PxU32 handle);
	void											removeAttachmentHandle(PxU32 handle);

	PxArray<PxU32>									mAttachmentHandles;
	PxArray<PxU32>									mSurfaceSurfaceAttachments;
private:
			
	Sc::DeformableSurfaceSim*						mSim;
	DeformableSurfaceCore&							mCore;
	PxsShapeCore*									mShapeCore;

	PxU32											mElementId; //this is used for the bound array, contactDist
	PxU32											mGpuRemapId;
};

PX_FORCE_INLINE DeformableSurface* getDeformableSurface(DeformableSurfaceHandle handle)
{
	return reinterpret_cast<DeformableSurface*>(handle);
}

PX_FORCE_INLINE void DeformableSurface::addAttachmentHandle(PxU32 handle)
{
	mAttachmentHandles.pushBack(handle);
}

PX_FORCE_INLINE void DeformableSurface::removeAttachmentHandle(PxU32 handle)
{
	for (PxU32 i = 0; i < mAttachmentHandles.size(); ++i)
	{
		if (mAttachmentHandles[i] == handle)
		{
			mAttachmentHandles.replaceWithLast(i);
		}
	}
}

} // namespace Dy
} // namespace physx

#endif // DY_DEFORMABLE_SURFACE_H
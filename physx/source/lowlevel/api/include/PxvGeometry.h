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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXV_GEOMETRY_H
#define PXV_GEOMETRY_H

#include "foundation/PxTransform.h"
#include "PxvConfig.h"

/*!
\file
Geometry interface
*/

/************************************************************************/
/* Shapes                                                               */
/************************************************************************/

#include "GuGeometryChecks.h"
#include "CmUtils.h"

namespace physx
{

//
// Summary of our material approach:
//
// On the API level, materials are accessed via pointer. Internally we store indices into the material table.
// The material table is stored in the SDK and the materials are shared among scenes. To make this threadsafe,
// we have the following approach:
//
// - Every scene has a copy of the SDK master material table
// - At the beginning of a simulation step, the scene material table gets synced to the master material table.
// - While the simulation is running, the scene table does not get touched.
// - Each shape stores the indices of its material(s). When the simulation is not running and a user requests the
//   materials of the shape, the indices are used to fetch the material from the master material table. When the
//   the simulation is running then the same indices are used internally to fetch the materials from the scene
//   material table. 
// - This whole scheme only works as long as the position of a material in the material table does not change
//   when other materials get deleted/inserted. The data structure of the material table makes sure that is the case.
//

struct MaterialIndicesStruct
{
// PX_SERIALIZATION
	MaterialIndicesStruct(const PxEMPTY)	{}
	static void getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

	MaterialIndicesStruct()
	:	indices(NULL)
	,	numIndices(0)
	,	pad(PX_PADDING_16)
	{
	}

	~MaterialIndicesStruct()
	{
	}

	void allocate(PxU16 size)
	{
		indices = PX_ALLOCATE(PxU16, size, "MaterialIndicesStruct::allocate");
		numIndices = size;
	}

	void deallocate()
	{
		PX_FREE(indices);
		numIndices = 0;
	}
	PxU16*	indices;	// the remap table for material index
	PxU16	numIndices; // the size of the remap table
	PxU16	pad;		// pad for serialization
	PxU32	gpuRemapId;	// PT: using padding bytes on x64
};

struct PxConvexMeshGeometryLL: public PxConvexMeshGeometry
{
	bool					gpuCompatible;	// PT: TODO: remove?
};

struct PxTriangleMeshGeometryLL: public PxTriangleMeshGeometry
{
	MaterialIndicesStruct	materialsLL;
};

struct PxParticleSystemGeometryLL : public PxParticleSystemGeometry
{
	MaterialIndicesStruct	materialsLL;
};

struct PxTetrahedronMeshGeometryLL : public PxTetrahedronMeshGeometry
{
	MaterialIndicesStruct	materialsLL;
};

struct PxHeightFieldGeometryLL : public PxHeightFieldGeometry
{
	MaterialIndicesStruct	materialsLL;
};

template <> struct PxcGeometryTraits<PxParticleSystemGeometryLL>	{ enum { TypeID = PxGeometryType::ePARTICLESYSTEM}; };
template <> struct PxcGeometryTraits<PxConvexMeshGeometryLL>		{ enum { TypeID = PxGeometryType::eCONVEXMESH }; };
template <> struct PxcGeometryTraits<PxTriangleMeshGeometryLL>		{ enum { TypeID = PxGeometryType::eTRIANGLEMESH }; };
template <> struct PxcGeometryTraits<PxTetrahedronMeshGeometryLL>	{ enum { TypeID = PxGeometryType::eTETRAHEDRONMESH }; };
template <> struct PxcGeometryTraits<PxHeightFieldGeometryLL>		{ enum { TypeID = PxGeometryType::eHEIGHTFIELD }; };

class InvalidGeometry : public PxGeometry
{
public:
	PX_CUDA_CALLABLE PX_FORCE_INLINE InvalidGeometry() : PxGeometry(PxGeometryType::eINVALID) {}
};

class GeometryUnion
{
public:
// PX_SERIALIZATION
	GeometryUnion(const PxEMPTY)	{}
	static	void	getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

	PX_CUDA_CALLABLE PX_FORCE_INLINE						GeometryUnion()						{ reinterpret_cast<InvalidGeometry&>(mGeometry) = InvalidGeometry(); }
	PX_CUDA_CALLABLE PX_FORCE_INLINE						GeometryUnion(const PxGeometry& g)	{ set(g);	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxGeometry&		getGeometry()				const	{ return reinterpret_cast<const PxGeometry&>(mGeometry);			}
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxGeometryType::Enum	getType()					const	{ return reinterpret_cast<const PxGeometry&>(mGeometry).getType();	}

	PX_CUDA_CALLABLE void	set(const PxGeometry& g);

	template<class Geom> PX_CUDA_CALLABLE PX_FORCE_INLINE Geom& get()
	{
		checkType<Geom>(getGeometry());
		return reinterpret_cast<Geom&>(mGeometry);
	}

	template<class Geom> PX_CUDA_CALLABLE PX_FORCE_INLINE const Geom& get() const
	{
		checkType<Geom>(getGeometry());
		return reinterpret_cast<const Geom&>(mGeometry);
	}

private:

	union {
		void*	alignment;	// PT: Makes sure the class is at least aligned to pointer size. See DE6803. 
		PxU8	box[sizeof(PxBoxGeometry)];
		PxU8	sphere[sizeof(PxSphereGeometry)];
		PxU8	capsule[sizeof(PxCapsuleGeometry)];
		PxU8	plane[sizeof(PxPlaneGeometry)];
		PxU8	convexCore[sizeof(PxConvexCoreGeometry)];
		PxU8	convex[sizeof(PxConvexMeshGeometryLL)];
		PxU8	particleSystem[sizeof(PxParticleSystemGeometryLL)];
		PxU8	mesh[sizeof(PxTriangleMeshGeometryLL)];
		PxU8	tetMesh[sizeof(PxTetrahedronMeshGeometryLL)];
		PxU8	heightfield[sizeof(PxHeightFieldGeometryLL)];
		PxU8	custom[sizeof(PxCustomGeometry)];
		PxU8	invalid[sizeof(InvalidGeometry)];
	} mGeometry;
};

	struct PxShapeCoreFlag
	{
		enum Enum
		{
			eOWNS_MATERIAL_IDX_MEMORY	= (1<<0),	// PT: for de-serialization to avoid deallocating material index list. Moved there from Sc::ShapeCore (since one byte was free).
			eIS_EXCLUSIVE				= (1<<1),	// PT: shape's exclusive flag
			eIDT_TRANSFORM				= (1<<2),	// PT: true if PxsShapeCore::transform is identity
			eDEFORMABLE_SURFACE_SHAPE	= (1<<3),	// True if this shape is a deformable surface shape
			eDEFORMABLE_VOLUME_SHAPE	= (1<<4)	// True if this shape is a deformable volume shape
		};
	};

	typedef PxFlags<PxShapeCoreFlag::Enum,PxU8> PxShapeCoreFlags;
	PX_FLAGS_OPERATORS(PxShapeCoreFlag::Enum,PxU8)

struct PxsShapeCore
{
	PxsShapeCore()
	{
		setDensityForFluid(800.0f);
	}

// PX_SERIALIZATION
	PxsShapeCore(const PxEMPTY) : mShapeCoreFlags(PxEmpty), mGeometry(PxEmpty)	{}
//~PX_SERIALIZATION

#if PX_WINDOWS_FAMILY	// PT: to avoid "error: offset of on non-standard-layout type" on Linux
	protected:
#endif
	PX_ALIGN_PREFIX(16)
	PxTransform			mTransform PX_ALIGN_SUFFIX(16);			// PT: Offset 0
#if PX_WINDOWS_FAMILY	// PT: to avoid "error: offset of on non-standard-layout type" on Linux
	public:
#endif
	PX_FORCE_INLINE	const PxTransform&	getTransform()	const
	{
		return mTransform;
	}

	PX_FORCE_INLINE	void	setTransform(const PxTransform& t)
	{
		mTransform = t;
		if(t.p.isZero() && t.q.isIdentity())
			mShapeCoreFlags.raise(PxShapeCoreFlag::eIDT_TRANSFORM);
		else
			mShapeCoreFlags.clear(PxShapeCoreFlag::eIDT_TRANSFORM);
	}

	PxReal				mContactOffset;				// PT: Offset 28
	PxU8				mShapeFlags;				// PT: Offset 32	!< API shape flags	// PT: TODO: use PxShapeFlags here. Needs to move flags to separate file.
	PxShapeCoreFlags	mShapeCoreFlags;			// PT: Offset 33
	PxU16				mMaterialIndex;				// PT: Offset 34
	PxReal				mRestOffset;				// PT: Offset 36 - same as the API property of the same name - PT: moved from Sc::ShapeCore to fill padding bytes
	GeometryUnion		mGeometry;					// PT: Offset 40
	PxReal				mTorsionalRadius;			// PT: Offset 104 - PT: moved from Sc::ShapeCore to fill padding bytes
	PxReal				mMinTorsionalPatchRadius;	// PT: Offset 108 - PT: moved from Sc::ShapeCore to fill padding bytes

	PX_FORCE_INLINE	float	getDensityForFluid()	const
	{
		return mGeometry.getGeometry().mTypePadding;
	}

	PX_FORCE_INLINE	void	setDensityForFluid(float density)
	{
		const_cast<PxGeometry&>(mGeometry.getGeometry()).mTypePadding = density;
	}
};

PX_COMPILE_TIME_ASSERT( sizeof(GeometryUnion) <= 64);	// PT: if you break this one I will not be happy

PX_COMPILE_TIME_ASSERT( (sizeof(PxsShapeCore)&0xf) == 0);

}

#endif

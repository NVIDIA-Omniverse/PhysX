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

#ifndef GU_TRIANGLEMESH_H
#define GU_TRIANGLEMESH_H

#include "foundation/PxIO.h"
#include "geometry/PxTriangle.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "geometry/PxTriangleMesh.h"

#include "GuMeshData.h"
#include "GuCenterExtents.h"
#include "CmScaling.h"
#include "CmRefCountable.h"
#include "common/PxRenderOutput.h"

namespace physx
{
class PxMeshScale;
struct PxTriangleMeshInternalData;

namespace Gu
{
	PX_FORCE_INLINE	void	getVertexRefs(PxU32 triangleIndex, PxU32& vref0, PxU32& vref1, PxU32& vref2, const void* indices, bool has16BitIndices)
	{
		if(has16BitIndices)
		{
			const PxU16* inds = reinterpret_cast<const PxU16*>(indices) + triangleIndex*3;
			vref0 = inds[0];
			vref1 = inds[1];
			vref2 = inds[2];
		} 
		else 
		{ 
			const PxU32* inds = reinterpret_cast<const PxU32*>(indices) + triangleIndex*3;
			vref0 = inds[0];
			vref1 = inds[1];
			vref2 = inds[2];
		} 
	}

class MeshFactory;
#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

class EdgeList;

class TriangleMesh : public PxTriangleMesh, public PxUserAllocated
{
public:

// PX_SERIALIZATION
														TriangleMesh(PxBaseFlags baseFlags)	: PxTriangleMesh(baseFlags), mSdfData(PxEmpty)	{}

								void					preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
	virtual						void					exportExtraData(PxSerializationContext& context);
								void					importExtraData(PxDeserializationContext& context);
	PX_PHYSX_COMMON_API	static	void					getBinaryMetaData(PxOutputStream& stream);
	virtual						void					release();
	virtual						void					requiresObjects(PxProcessPxBaseCallback&){}
//~PX_SERIALIZATION
														TriangleMesh(MeshFactory* factory, TriangleMeshData& data);
														TriangleMesh(const PxTriangleMeshInternalData& data);
	virtual												~TriangleMesh();
	
	// PxBase
	virtual						void					onRefCountZero();
	//~PxBase

	// PxRefCounted
	virtual						void					acquireReference()					{ Cm::RefCountable_incRefCount(*this);			}
	virtual						PxU32					getReferenceCount()			const	{ return Cm::RefCountable_getRefCount(*this);	}
	//~PxRefCounted
	
	// PxTriangleMesh
	virtual						PxU32					getNbVertices()				const   { return mNbVertices;}
	virtual						const PxVec3*			getVertices()				const   { return mVertices; }

	virtual						PxVec3*					getVerticesForModification();
	virtual						PxBounds3				refitBVH();
	virtual						PxU32					getNbTriangles()			const	{ return mNbTriangles;					}
	virtual						const void*				getTriangles()				const	{ return mTriangles;					}
	virtual						PxTriangleMeshFlags		getTriangleMeshFlags()		const	{ return PxTriangleMeshFlags(mFlags);	}
	virtual						const PxU32*			getTrianglesRemap()			const	{ return mFaceRemap;					}
	virtual						void					setPreferSDFProjection(bool preferProjection)
	{
		if (preferProjection)
			mFlags &= PxU8(~PxTriangleMeshFlag::ePREFER_NO_SDF_PROJ);
		else
			mFlags |= PxTriangleMeshFlag::ePREFER_NO_SDF_PROJ;
	}

	virtual						bool					getPreferSDFProjection()	const { return !(mFlags & PxTriangleMeshFlag::ePREFER_NO_SDF_PROJ); }

	virtual						PxMaterialTableIndex	getTriangleMaterialIndex(PxTriangleID triangleIndex)	const
														{
															return hasPerTriangleMaterials() ? getMaterials()[triangleIndex] : PxMaterialTableIndex(0xffff);
														}

	virtual						PxBounds3				getLocalBounds()			const
														{
															PX_ASSERT(mAABB.isValid());
															return PxBounds3::centerExtents(mAABB.mCenter, mAABB.mExtents);
														}

	virtual						const PxReal*			getSDF()					const
														{
															return mSdfData.mSdf;
														}

	virtual						void					getSDFDimensions(PxU32& numX, PxU32& numY, PxU32& numZ) const
														{
															if(mSdfData.mSdf)
															{
																numX = mSdfData.mDims.x;
																numY = mSdfData.mDims.y;
																numZ = mSdfData.mDims.z;
															}
															else
																numX = numY = numZ = 0;
														}

	virtual						void					getMassInformation(PxReal& mass, PxMat33& localInertia, PxVec3& localCenterOfMass)	const
														{
															mass = mMass; localInertia = mInertia; localCenterOfMass = mLocalCenterOfMass;
														}
	
	//~PxTriangleMesh

	virtual						bool					getInternalData(PxTriangleMeshInternalData&, bool)	const	{ return false; }

	// PT: this one is just to prevent instancing Gu::TriangleMesh.
	// But you should use PxBase::getConcreteType() instead to avoid the virtual call.
	virtual						PxMeshMidPhase::Enum	getMidphaseID()				const	= 0;

	PX_FORCE_INLINE				const PxU32*			getFaceRemap()				const	{ return mFaceRemap;											}
	PX_FORCE_INLINE				bool					has16BitIndices()			const	{ return (mFlags & PxMeshFlag::e16_BIT_INDICES) ? true : false;	}
	PX_FORCE_INLINE				bool					hasPerTriangleMaterials()	const	{ return mMaterialIndices != NULL;								}
	PX_FORCE_INLINE				PxU32					getNbVerticesFast()			const	{ return mNbVertices;		}
	PX_FORCE_INLINE				PxU32					getNbTrianglesFast()		const	{ return mNbTriangles;		}
	PX_FORCE_INLINE				const void*				getTrianglesFast()			const	{ return mTriangles;		}
	PX_FORCE_INLINE				const PxVec3*			getVerticesFast()			const	{ return mVertices;			}
	PX_FORCE_INLINE				const PxU32*			getAdjacencies()			const	{ return mAdjacencies;		}
	PX_FORCE_INLINE				PxReal					getGeomEpsilon()			const	{ return mGeomEpsilon;		}
	PX_FORCE_INLINE				const CenterExtents&	getLocalBoundsFast()		const	{ return mAABB;				}
	PX_FORCE_INLINE				const PxU16*			getMaterials()				const	{ return mMaterialIndices;	}
	PX_FORCE_INLINE				const PxU8*				getExtraTrigData()			const	{ return mExtraTrigData;	}
	
	PX_FORCE_INLINE				const PxU32*			getAccumulatedTriangleRef()	const	{ return mAccumulatedTrianglesRef;	}
	PX_FORCE_INLINE				const PxU32*			getTriangleReferences()		const	{ return mTrianglesReferences;		}
	PX_FORCE_INLINE				PxU32					getNbTriangleReferences()	const	{ return mNbTrianglesReferences;	}

	PX_FORCE_INLINE			const CenterExtentsPadded&	getPaddedBounds()			const
														{
															// PT: see compile-time assert in cpp
															return static_cast<const CenterExtentsPadded&>(mAABB);
														}

	PX_FORCE_INLINE				void					computeWorldTriangle(
															PxTriangle& worldTri, PxTriangleID triangleIndex, const PxMat34& worldMatrix, bool flipNormal = false,
															PxU32* PX_RESTRICT vertexIndices=NULL, PxU32* PX_RESTRICT adjacencyIndices=NULL) const;
	PX_FORCE_INLINE				void					getLocalTriangle(PxTriangle& localTri, PxTriangleID triangleIndex, bool flipNormal = false) const;

								void					setMeshFactory(MeshFactory* factory) { mMeshFactory = factory; }

								// SDF methods
	PX_FORCE_INLINE				const SDF&				getSdfDataFast()			const { return mSdfData; }
								//~SDF methods

	PX_FORCE_INLINE				PxReal					getMass()			const { return mMass; }

								// PT: for debug viz
	PX_PHYSX_COMMON_API			const Gu::EdgeList*		requestEdgeList()			const;

protected:
								PxU32					mNbVertices;
								PxU32					mNbTriangles;
								PxVec3*					mVertices;
								void*					mTriangles;				//!< 16 (<= 0xffff #vertices) or 32 bit trig indices (mNbTriangles * 3)
								// 16 bytes block

		// PT: WARNING: bounds must be followed by at least 32bits of data for safe SIMD loading
								CenterExtents			mAABB;
								PxU8*					mExtraTrigData;			//one per trig
								PxReal					mGeomEpsilon;			//!< see comments in cooking code referencing this variable
								// 16 bytes block
		/*
		low 3 bits (mask: 7) are the edge flags:
		b001 = 1 = ignore edge 0 = edge v0-->v1
		b010 = 2 = ignore edge 1 = edge v0-->v2
		b100 = 4 = ignore edge 2 = edge v1-->v2
		*/
								PxU8					mFlags;					//!< Flag whether indices are 16 or 32 bits wide
																				//!< Flag whether triangle adajacencies are build
								PxU16*					mMaterialIndices;		//!< the size of the array is numTriangles.
								PxU32*					mFaceRemap;				//!< new faces to old faces mapping (after cleaning, etc). Usage: old = faceRemap[new]
								PxU32*					mAdjacencies;			//!< Adjacency information for each face - 3 adjacent faces
																				//!< Set to 0xFFFFffff if no adjacent face
	
								MeshFactory*			mMeshFactory;			// PT: changed to pointer for serialization
				mutable			Gu::EdgeList*			mEdgeList;				// PT: for debug viz


								PxReal					mMass;					//this is mass assuming a unit density that can be scaled by instances!
								PxMat33					mInertia;				//in local space of mesh!
								PxVec3					mLocalCenterOfMass;		//local space com
public:
								
								// GRB data -------------------------
								void*					mGRB_triIndices;		//!< GRB: GPU-friendly tri indices

								// TODO avoroshilov: cooking - adjacency info - duplicated, remove it and use 'mAdjacencies' and 'mExtraTrigData' see GuTriangleMesh.cpp:325
								void*					mGRB_triAdjacencies;	//!< GRB: adjacency data, with BOUNDARY and NONCONVEX flags (flags replace adj indices where applicable)

								PxU32*					mGRB_faceRemap;			//!< GRB : gpu to cpu triangle indice remap
								PxU32*					mGRB_faceRemapInverse;
								Gu::BV32Tree*			mGRB_BV32Tree;			//!< GRB: BV32 tree
								// End of GRB data ------------------

								// SDF data -------------------------
								SDF						mSdfData;
								// End of SDF data ------------------

								void					setAllEdgesActive();
								//Vertex mapping data
								PxU32*					mAccumulatedTrianglesRef;//runsum
								PxU32*					mTrianglesReferences;
								PxU32					mNbTrianglesReferences;
								//End of vertex mapping data
};

#if PX_VC
#pragma warning(pop)
#endif

} // namespace Gu

PX_FORCE_INLINE void Gu::TriangleMesh::computeWorldTriangle(PxTriangle& worldTri, PxTriangleID triangleIndex, const PxMat34& worldMatrix, bool flipNormal,
	PxU32* PX_RESTRICT vertexIndices, PxU32* PX_RESTRICT adjacencyIndices) const
{
	PxU32 vref0, vref1, vref2;
	getVertexRefs(triangleIndex, vref0, vref1, vref2, mTriangles, has16BitIndices());

	if(flipNormal)
		PxSwap<PxU32>(vref1, vref2);

	const PxVec3* PX_RESTRICT vertices = getVerticesFast();
	worldTri.verts[0] = worldMatrix.transform(vertices[vref0]);
	worldTri.verts[1] = worldMatrix.transform(vertices[vref1]);
	worldTri.verts[2] = worldMatrix.transform(vertices[vref2]);

	if(vertexIndices)
	{
		vertexIndices[0] = vref0;
		vertexIndices[1] = vref1;
		vertexIndices[2] = vref2;
	}

	if(adjacencyIndices)
	{
		if(mAdjacencies)
		{
			// PT: TODO: is this correct?
			adjacencyIndices[0] = flipNormal ? mAdjacencies[triangleIndex*3 + 2] : mAdjacencies[triangleIndex*3 + 0];
			adjacencyIndices[1] = mAdjacencies[triangleIndex*3 + 1];
			adjacencyIndices[2] = flipNormal ? mAdjacencies[triangleIndex*3 + 0] : mAdjacencies[triangleIndex*3 + 2];
		}
		else
		{
			adjacencyIndices[0] = 0xffffffff;
			adjacencyIndices[1] = 0xffffffff;
			adjacencyIndices[2] = 0xffffffff;
		}
	}
}

PX_FORCE_INLINE void Gu::TriangleMesh::getLocalTriangle(PxTriangle& localTri, PxTriangleID triangleIndex, bool flipNormal) const
{
	PxU32 vref0, vref1, vref2;
	getVertexRefs(triangleIndex, vref0, vref1, vref2, mTriangles, has16BitIndices());

	if(flipNormal)
		PxSwap<PxU32>(vref1, vref2);

	const PxVec3* PX_RESTRICT vertices = getVerticesFast();
	localTri.verts[0] = vertices[vref0];
	localTri.verts[1] = vertices[vref1];
	localTri.verts[2] = vertices[vref2];
}

PX_INLINE float computeSweepData(const PxTriangleMeshGeometry& triMeshGeom, /*const Cm::FastVertex2ShapeScaling& scaling,*/ PxVec3& sweepOrigin, PxVec3& sweepExtents, PxVec3& sweepDir, float distance)
{
	PX_ASSERT(!Cm::isEmpty(sweepOrigin, sweepExtents));

	const PxVec3 endPt = sweepOrigin + sweepDir*distance;
	PX_ASSERT(!Cm::isEmpty(endPt, sweepExtents));

	const Cm::FastVertex2ShapeScaling meshScaling(triMeshGeom.scale.getInverse()); // shape to vertex transform

	const PxMat33& vertex2ShapeSkew = meshScaling.getVertex2ShapeSkew();

	const PxVec3 originBoundsCenter = vertex2ShapeSkew * sweepOrigin;
	const PxVec3 originBoundsExtents = Cm::basisExtent(vertex2ShapeSkew.column0, vertex2ShapeSkew.column1, vertex2ShapeSkew.column2, sweepExtents);

	sweepOrigin = originBoundsCenter;
	sweepExtents = originBoundsExtents;
	sweepDir = (vertex2ShapeSkew * endPt) - originBoundsCenter;
	return sweepDir.normalizeSafe();
}

PX_FORCE_INLINE const Gu::TriangleMesh* _getMeshData(const PxTriangleMeshGeometry& meshGeom)
{
	return static_cast<const Gu::TriangleMesh*>(meshGeom.triangleMesh);
}

}

#endif

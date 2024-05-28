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

#ifndef GU_BV4_H
#define GU_BV4_H

#include "foundation/PxBounds3.h"
#include "GuBV4Settings.h"
#include "GuCenterExtents.h"
#include "GuTriangle.h"
#include "foundation/PxVecMath.h"
#include "common/PxPhysXCommonConfig.h"

#define V4LoadU_Safe	physx::aos::V4LoadU	// PT: prefix needed on Linux. Sigh.
#define V4LoadA_Safe	V4LoadA
#define V4StoreA_Safe	V4StoreA
#define V4StoreU_Safe	V4StoreU

namespace physx
{
	class PxSerializationContext;
	class PxDeserializationContext;

namespace Gu
{
	struct VertexPointers
	{
		const PxVec3*	Vertex[3];
	};

	struct TetrahedronPointers
	{
		const PxVec3* Vertex[4];
	};

	// PT: TODO: make this more generic, rename to IndQuad32, refactor with GRB's int4
	class IndTetrahedron32 : public physx::PxUserAllocated
	{
	public:
	public:
		PX_FORCE_INLINE			IndTetrahedron32() {}
		PX_FORCE_INLINE			IndTetrahedron32(PxU32 r0, PxU32 r1, PxU32 r2, PxU32 r3) { mRef[0] = r0; mRef[1] = r1; mRef[2] = r2; mRef[3] = r3; }
		PX_FORCE_INLINE			IndTetrahedron32(const IndTetrahedron32& tetrahedron)
								{
									mRef[0] = tetrahedron.mRef[0];
									mRef[1] = tetrahedron.mRef[1];
									mRef[2] = tetrahedron.mRef[2];
									mRef[3] = tetrahedron.mRef[3];
								}
		PX_FORCE_INLINE			~IndTetrahedron32() {}
						PxU32	mRef[4];
	};
	PX_COMPILE_TIME_ASSERT(sizeof(IndTetrahedron32) == 16);

	// PT: TODO: make this more generic, rename to IndQuad16
	class IndTetrahedron16 : public physx::PxUserAllocated
	{
	public:
	public:
		PX_FORCE_INLINE			IndTetrahedron16() {}
		PX_FORCE_INLINE			IndTetrahedron16(PxU16 r0, PxU16 r1, PxU16 r2, PxU16 r3) { mRef[0] = r0; mRef[1] = r1; mRef[2] = r2; mRef[3] = r3; }
		PX_FORCE_INLINE			IndTetrahedron16(const IndTetrahedron16& tetrahedron)
								{
									mRef[0] = tetrahedron.mRef[0];
									mRef[1] = tetrahedron.mRef[1];
									mRef[2] = tetrahedron.mRef[2];
									mRef[3] = tetrahedron.mRef[3];
								}
		PX_FORCE_INLINE			~IndTetrahedron16() {}
						PxU16	mRef[4];
	};
	PX_COMPILE_TIME_ASSERT(sizeof(IndTetrahedron16) == 8);

	typedef IndexedTriangle32 IndTri32;
	typedef IndexedTriangle16 IndTri16;

	PX_FORCE_INLINE void getVertexReferences(PxU32& vref0, PxU32& vref1, PxU32& vref2, PxU32 index, const IndTri32* T32, const IndTri16* T16)
	{
		if(T32)
		{
			const IndTri32* PX_RESTRICT tri = T32 + index;
			vref0 = tri->mRef[0];
			vref1 = tri->mRef[1];
			vref2 = tri->mRef[2];
		}
		else
		{
			const IndTri16* PX_RESTRICT tri = T16 + index;
			vref0 = tri->mRef[0];
			vref1 = tri->mRef[1];
			vref2 = tri->mRef[2];
		}
	}

	PX_FORCE_INLINE void getVertexReferences(PxU32& vref0, PxU32& vref1, PxU32& vref2, PxU32& vref3, PxU32 index, const IndTetrahedron32* T32, const IndTetrahedron16* T16)
	{
		if(T32)
		{
			const IndTetrahedron32* PX_RESTRICT tet = T32 + index;
			vref0 = tet->mRef[0];
			vref1 = tet->mRef[1];
			vref2 = tet->mRef[2];
			vref3 = tet->mRef[3];
		}
		else
		{
			const IndTetrahedron16* PX_RESTRICT tet = T16 + index;
			vref0 = tet->mRef[0];
			vref1 = tet->mRef[1];
			vref2 = tet->mRef[2];
			vref3 = tet->mRef[3];
		}
	}

	class SourceMeshBase : public physx::PxUserAllocated
	{
		public:
		enum MeshType
		{
			TRI_MESH,
			TET_MESH,
			FORCE_DWORD	= 0x7fffffff
		};
										SourceMeshBase(MeshType meshType);
		 virtual						~SourceMeshBase();
	
										SourceMeshBase(const PxEMPTY) {}
		static			void			getBinaryMetaData(PxOutputStream& stream);

						PxU32			mNbVerts;
						const PxVec3*	mVerts;

		PX_FORCE_INLINE	PxU32			getNbVertices()		const	{ return mNbVerts;	}
		PX_FORCE_INLINE	const PxVec3*	getVerts()			const	{ return mVerts;	}

		PX_FORCE_INLINE	void			setNbVertices(PxU32 nb)		{ mNbVerts = nb;	}

		PX_FORCE_INLINE	void			initRemap()					{ mRemap = NULL;	}
		PX_FORCE_INLINE	const PxU32*	getRemap()			const	{ return mRemap;	}
		PX_FORCE_INLINE	void			releaseRemap()				{ PX_FREE(mRemap);	}

		PX_FORCE_INLINE	MeshType		getMeshType()		const	{ return mType;		}

		// PT: TODO: check whether adding these vcalls affected build & runtime performance
		virtual			PxU32			getNbPrimitives()	const = 0;
		virtual			void			remapTopology(const PxU32* order) = 0;
		virtual			void			getPrimitiveBox(const PxU32 primitiveInd, physx::aos::Vec4V& minV, physx::aos::Vec4V& maxV) = 0;
		virtual			void			refit(const PxU32 primitiveInd, PxBounds3& refitBox) = 0;
		
		protected:
						MeshType		mType;
						PxU32*			mRemap;
	};

	class SourceMesh : public SourceMeshBase
	{
	public:
										SourceMesh();
		 virtual						~SourceMesh();
		// PX_SERIALIZATION
										SourceMesh(const PxEMPTY) : SourceMeshBase(PxEmpty) {}
		static			void			getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION

						void			reset();
						void			operator = (SourceMesh& v);

						PxU32			mNbTris;
						IndTri32*		mTriangles32;
						IndTri16*		mTriangles16;

		PX_FORCE_INLINE	PxU32			getNbTriangles()	const	{ return mNbTris;		}
		PX_FORCE_INLINE	const IndTri32*	getTris32()			const	{ return mTriangles32;	}
		PX_FORCE_INLINE	const IndTri16*	getTris16()			const	{ return mTriangles16;	}

		PX_FORCE_INLINE	void			setNbTriangles(PxU32 nb)	{ mNbTris = nb;			}

		// SourceMeshBase
		virtual			PxU32			getNbPrimitives()	const	{ return  getNbTriangles(); }
		virtual			void			remapTopology(const PxU32* order);
		virtual			void			getPrimitiveBox(const PxU32 primitiveInd, physx::aos::Vec4V& minV, physx::aos::Vec4V& maxV);
		virtual			void			refit(const PxU32 primitiveInd, PxBounds3& refitBox);
		//~SourceMeshBase

		PX_FORCE_INLINE	void			setPointers(IndTri32* tris32, IndTri16* tris16, const PxVec3* verts)
										{
											mTriangles32	= tris32;
											mTriangles16	= tris16;
											mVerts			= verts;
										}

						bool			isValid()		const;

		PX_FORCE_INLINE	void			getTriangle(VertexPointers& vp, PxU32 index)	const
										{
											PxU32 VRef0, VRef1, VRef2;
											getVertexReferences(VRef0, VRef1, VRef2, index, mTriangles32, mTriangles16);
											vp.Vertex[0] = mVerts + VRef0;
											vp.Vertex[1] = mVerts + VRef1;
											vp.Vertex[2] = mVerts + VRef2;
										}
	};

	class TetrahedronSourceMesh : public SourceMeshBase
	{
	public:
												TetrahedronSourceMesh();
		virtual									~TetrahedronSourceMesh();
		// PX_SERIALIZATION
												TetrahedronSourceMesh(const PxEMPTY) : SourceMeshBase(TET_MESH) {}
		static			void					getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION

						void					reset();
						void					operator = (TetrahedronSourceMesh& v);

						PxU32					mNbTetrahedrons;
						IndTetrahedron32*		mTetrahedrons32;
						IndTetrahedron16*		mTetrahedrons16;

		PX_FORCE_INLINE	PxU32					getNbTetrahedrons()			const	{ return mNbTetrahedrons;	}
		PX_FORCE_INLINE	const IndTetrahedron32*	getTetrahedrons32()			const	{ return mTetrahedrons32;	}
		PX_FORCE_INLINE	const IndTetrahedron16*	getTetrahedrons16()			const	{ return mTetrahedrons16;	}

		PX_FORCE_INLINE	void					setNbTetrahedrons(PxU32 nb)			{ mNbTetrahedrons = nb;		}

		// SourceMeshBase
		virtual			PxU32					getNbPrimitives()			const	{ return  getNbTetrahedrons(); }
		virtual			void					remapTopology(const PxU32* order);
		virtual			void					getPrimitiveBox(const PxU32 primitiveInd, physx::aos::Vec4V& minV, physx::aos::Vec4V& maxV);
		virtual			void					refit(const PxU32 primitiveInd, PxBounds3& refitBox);
		//~SourceMeshBase

		PX_FORCE_INLINE	void					setPointers(IndTetrahedron32* tets32, IndTetrahedron16* tets16, const PxVec3* verts)
												{
													mTetrahedrons32 = tets32;
													mTetrahedrons16 = tets16;
													mVerts = verts;
												}

						bool					isValid()		const;

		PX_FORCE_INLINE	void					getTetrahedron(TetrahedronPointers& vp, PxU32 index)	const
												{
													PxU32 VRef0, VRef1, VRef2, VRef3;
													getVertexReferences(VRef0, VRef1, VRef2, VRef3, index, mTetrahedrons32, mTetrahedrons16);
													vp.Vertex[0] = mVerts + VRef0;
													vp.Vertex[1] = mVerts + VRef1;
													vp.Vertex[2] = mVerts + VRef2;
													vp.Vertex[3] = mVerts + VRef3;
												}
	};

	struct LocalBounds
	{
		// PX_SERIALIZATION
								LocalBounds(const PxEMPTY)										{}
		//~PX_SERIALIZATION
								LocalBounds() : mCenter(PxVec3(0.0f)), mExtentsMagnitude(0.0f)	{}

						PxVec3	mCenter;
						float	mExtentsMagnitude;

		PX_FORCE_INLINE	void	init(const PxBounds3& bounds)
								{
									mCenter = bounds.getCenter();
									// PT: TODO: compute mag first, then multiplies by 0.5f (TA34704)
									mExtentsMagnitude = bounds.getExtents().magnitude();
								}
	};

	class QuantizedAABB
	{
		public:

		struct Data
		{
			PxU16	mExtents;	//!< Quantized extents
			PxI16	mCenter;	//!< Quantized center
		};
			Data	mData[3];
	};
	PX_COMPILE_TIME_ASSERT(sizeof(QuantizedAABB)==12);

	/////

	#define GU_BV4_CHILD_OFFSET_SHIFT_COUNT	11
	static	PX_FORCE_INLINE	PxU32	getChildOffset(PxU32 data)	{ return data>>GU_BV4_CHILD_OFFSET_SHIFT_COUNT;	}
	static	PX_FORCE_INLINE	PxU32	getChildType(PxU32 data)	{ return (data>>1)&3;							}

	template<class BoxType>
	struct BVDataPackedT
	{
						BoxType	mAABB;
						PxU32	mData;

		PX_FORCE_INLINE	PxU32	isLeaf()			const	{ return mData&1;								}
		PX_FORCE_INLINE	PxU32	getPrimitive()		const	{ return mData>>1;								}
		PX_FORCE_INLINE	PxU32	getChildOffset()	const	{ return mData>>GU_BV4_CHILD_OFFSET_SHIFT_COUNT;}
		PX_FORCE_INLINE	PxU32	getChildType()		const	{ return (mData>>1)&3;							}
		PX_FORCE_INLINE	PxU32	getChildData()		const	{ return mData;									}

		PX_FORCE_INLINE	void	encodePNS(PxU32 code)
								{
									PX_ASSERT(code<256);
									mData |= code<<3;
								}
		PX_FORCE_INLINE	PxU32	decodePNSNoShift()	const	{ return mData;									}
	};

	typedef BVDataPackedT<QuantizedAABB>	BVDataPackedQ;
	typedef BVDataPackedT<CenterExtents>	BVDataPackedNQ;

	// PT: TODO: align class to 16? (TA34704)
	class BV4Tree : public physx::PxUserAllocated
	{
		public:
		// PX_SERIALIZATION
								BV4Tree(const PxEMPTY);
				void			exportExtraData(PxSerializationContext&);
				void			importExtraData(PxDeserializationContext& context);
		static	void			getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION
								BV4Tree();
								BV4Tree(SourceMesh* meshInterface, const PxBounds3& localBounds);
								~BV4Tree();

				bool			refit(PxBounds3& globalBounds, float epsilon);

				bool			load(PxInputStream& stream, bool mismatch);

				void			reset();
				void			operator = (BV4Tree& v);

				bool			init(SourceMeshBase* meshInterface, const PxBounds3& localBounds);
				void			release();
						
				SourceMeshBase*	mMeshInterface;
						
				LocalBounds		mLocalBounds;

				PxU32			mNbNodes;
				void*			mNodes;				// PT: BVDataPacked / BVDataSwizzled
				PxU32			mInitData;
				// PT: the dequantization coeffs are only used for quantized trees
				PxVec3			mCenterOrMinCoeff;	// PT: dequantization coeff, either for Center or Min (depending on AABB format)
				PxVec3			mExtentsOrMaxCoeff;	// PT: dequantization coeff, either for Extents or Max (depending on AABB format)
				bool			mUserAllocated;		// PT: please keep these 4 bytes right after mCenterOrMinCoeff/mExtentsOrMaxCoeff for safe V4 loading
				bool			mQuantized;			// PT: true for quantized trees
				bool			mIsEdgeSet;			// PT: equivalent to RTree::IS_EDGE_SET
				bool			mPadding;
	};

} // namespace Gu
}

#endif // GU_BV4_H

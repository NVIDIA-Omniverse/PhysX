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

#ifndef GU_CONVEX_MESH_H
#define GU_CONVEX_MESH_H

#include "foundation/PxBitAndData.h"
#include "common/PxMetaData.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "foundation/PxUserAllocated.h"
#include "CmRefCountable.h"
#include "common/PxRenderOutput.h"
#include "GuConvexMeshData.h"

namespace physx
{
class BigConvexData;

namespace Gu
{
	class MeshFactory;
	struct HullPolygonData;

	PX_INLINE PxU32 computeBufferSize(const Gu::ConvexHullData& data, PxU32 nb)
	{
		PxU32 bytesNeeded = sizeof(Gu::HullPolygonData) * data.mNbPolygons;
		bytesNeeded += sizeof(PxVec3) * data.mNbHullVertices;
		bytesNeeded += sizeof(PxU8) * data.mNbEdges * 2;		// mFacesByEdges8
		bytesNeeded += sizeof(PxU8) * data.mNbHullVertices * 3; // mFacesByVertices8;
		bytesNeeded += data.mNbEdges.isBitSet() ? (sizeof(PxU16) * data.mNbEdges * 2) : 0;		 // mEdges;
		bytesNeeded += sizeof(PxU8) * nb;						// mVertexData8
		
		//4 align the whole thing!
		const PxU32 mod = bytesNeeded % sizeof(PxReal);
		if (mod)
			bytesNeeded += sizeof(PxReal) - mod;
		return bytesNeeded;
	}

	struct ConvexHullInitData
	{
		ConvexHullData	mHullData;
		PxU32			mNb;
		PxReal			mMass;		
		PxMat33			mInertia;
		BigConvexData*	mBigConvexData;
		SDF*			mSdfData;
	};

	// 0: includes raycast map
	// 1: discarded raycast map
	// 2: support map not always there
	// 3: support stackless trees for non-recursive collision queries
	// 4: no more opcode model
	// 5: valencies table and gauss map combined, only exported over a vertex count treshold that depends on the platform cooked for.
	// 6: removed support for edgeData16.
	// 7: removed support for edge8Data.
	// 8: removed support for triangles.
	// 9: removed local sphere.
	//10: removed geometric center.
	//11: removed mFlags, and mERef16 from Poly; nbVerts is just a byte.
	//12: removed explicit minimum, maximum from Poly
	//13: internal objects
	//14: SDF
    #define  PX_CONVEX_VERSION 14
  
	class ConvexMesh : public PxConvexMesh, public PxUserAllocated
	{
	public:
	// PX_SERIALIZATION
							 					ConvexMesh(PxBaseFlags baseFlags) : PxConvexMesh(baseFlags), mHullData(PxEmpty), mNb(PxEmpty) 
												{
													mNb.setBit();
												}									

						void					preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
		 virtual		void					exportExtraData(PxSerializationContext& stream);
						void					importExtraData(PxDeserializationContext& context);
		PX_PHYSX_COMMON_API static	ConvexMesh*	createObject(PxU8*& address, PxDeserializationContext& context);
		PX_PHYSX_COMMON_API static	void		getBinaryMetaData(PxOutputStream& stream);
						void					resolveReferences(PxDeserializationContext&)				{}
		virtual			void					requiresObjects(PxProcessPxBaseCallback&){}
	//~PX_SERIALIZATION
		 										ConvexMesh(MeshFactory* factory);

												ConvexMesh(MeshFactory* factory, ConvexHullInitData& data);

						bool					load(PxInputStream& stream);

		// PxBase
		virtual			void					onRefCountZero();
		//~PxBase

		// PxRefCounted
		virtual			PxU32					getReferenceCount()								const;
		virtual			void					acquireReference();
		//~PxRefCounted

		// PxConvexMesh										
		virtual			void					release();
		virtual			PxU32					getNbVertices()									const	{ return mHullData.mNbHullVertices;		}
		virtual			const PxVec3*			getVertices()									const	{ return mHullData.getHullVertices();	}
		virtual			const PxU8*				getIndexBuffer()								const	{ return mHullData.getVertexData8();	}
		virtual			PxU32					getNbPolygons()									const	{ return mHullData.mNbPolygons;			}
		virtual			bool					getPolygonData(PxU32 i, PxHullPolygon& data)	const;
		virtual			bool					isGpuCompatible()								const;						

		virtual			void					getMassInformation(PxReal& mass, PxMat33& localInertia, PxVec3& localCenterOfMass)	const;
		virtual			PxBounds3				getLocalBounds()								const;
		virtual			const PxReal*			getSDF() const;
		
		//~PxConvexMesh

		PX_FORCE_INLINE	PxU32					getNbVerts()									const	{ return mHullData.mNbHullVertices;		}
		PX_FORCE_INLINE	const PxVec3*			getVerts()										const	{ return mHullData.getHullVertices();	}
		PX_FORCE_INLINE	PxU32					getNbPolygonsFast()								const	{ return mHullData.mNbPolygons;			}
		PX_FORCE_INLINE	const HullPolygonData&	getPolygon(PxU32 i)								const	{ return mHullData.mPolygons[i];		}
		PX_FORCE_INLINE	const HullPolygonData*	getPolygons()									const	{ return mHullData.mPolygons;			}
		PX_FORCE_INLINE	PxU32					getNbEdges()									const	{ return mHullData.mNbEdges;			}

		PX_FORCE_INLINE	const ConvexHullData&	getHull()										const	{ return mHullData;						}
		PX_FORCE_INLINE	ConvexHullData&			getHull()												{ return mHullData;						}
		PX_FORCE_INLINE	const CenterExtents&	getLocalBoundsFast()							const	{ return mHullData.mAABB;				}
		PX_FORCE_INLINE	PxReal					getMass()										const	{ return mMass;							}
		PX_FORCE_INLINE void					setMass(PxReal mass)									{ mMass = mass;							}		
		PX_FORCE_INLINE	const PxMat33&			getInertia()									const	{ return mInertia;						}
		PX_FORCE_INLINE void					setInertia(const PxMat33& inertia)						{ mInertia = inertia;					}

		PX_FORCE_INLINE BigConvexData*			getBigConvexData()								const	{ return mBigConvexData;				}
		PX_FORCE_INLINE void					setBigConvexData(BigConvexData* bcd)					{ mBigConvexData = bcd;					}

		PX_FORCE_INLINE	PxU32					getBufferSize()									const	{ return computeBufferSize(mHullData, getNb());	}

		virtual									~ConvexMesh();

		PX_FORCE_INLINE	void					setMeshFactory(MeshFactory* f)							{ mMeshFactory = f;						}
		PX_FORCE_INLINE void					setNb(PxU32 nb)											{ mNb = nb;								}

	protected:
						ConvexHullData			mHullData;
						PxBitAndDword			mNb;	// ### PT: added for serialization. Try to remove later?

						SDF*					mSdfData;
						BigConvexData*			mBigConvexData;		//!< optional, only for large meshes! PT: redundant with ptr in chull data? Could also be end of other buffer
						PxReal					mMass;				//this is mass assuming a unit density that can be scaled by instances!
						PxMat33					mInertia;			//in local space of mesh!
	private:
						MeshFactory*			mMeshFactory;	// PT: changed to pointer for serialization

		PX_FORCE_INLINE	PxU32					getNb()												const	{ return mNb;						}
		PX_FORCE_INLINE	PxU32					ownsMemory()										const	{ return PxU32(!mNb.isBitSet());	}
	};

	PX_FORCE_INLINE const Gu::ConvexHullData* _getHullData(const PxConvexMeshGeometry& convexGeom)
	{
		return &static_cast<const Gu::ConvexMesh*>(convexGeom.convexMesh)->getHull();
	}

} // namespace Gu

}

#endif

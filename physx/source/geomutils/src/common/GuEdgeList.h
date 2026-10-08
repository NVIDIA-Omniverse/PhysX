// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_EDGE_LIST_H
#define GU_EDGE_LIST_H

#include "foundation/PxSimpleTypes.h"
#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxIO.h"
#include "foundation/PxVec3.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
namespace Gu
{
	enum EdgeType
	{
		PX_EDGE_UNDEFINED,

		PX_EDGE_BOUNDARY,	//!< Edge belongs to a single triangle
		PX_EDGE_INTERNAL,	//!< Edge belongs to exactly two triangles
		PX_EDGE_SINGULAR,	//!< Edge belongs to three or more triangles

		PX_EDGE_FORCE_DWORD	= 0x7fffffff
	};

	enum EdgeFlag
	{
		PX_EDGE_ACTIVE	= (1<<0)
	};

	//! Basic edge-data
	struct EdgeData
	{
		PxU32	Ref0;	//!< First vertex reference	
		PxU32	Ref1;	//!< Second vertex reference
	};
	PX_COMPILE_TIME_ASSERT(sizeof(EdgeData) == 8);

	//! Basic edge-data using 8-bit references
	struct Edge8Data
	{
		PxU8	Ref0;	//!< First vertex reference
		PxU8	Ref1;	//!< Second vertex reference
	};
	PX_COMPILE_TIME_ASSERT(sizeof(Edge8Data) == 2);

	//! A count/offset pair = an edge descriptor
	struct EdgeDescData
	{
		PxU16	Flags;
		PxU16	Count;
		PxU32	Offset;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(EdgeDescData) == 8);

	//! Edge<->triangle mapping
	struct EdgeTriangleData
	{
		PxU32	mLink[3];
	};
	PX_COMPILE_TIME_ASSERT(sizeof(EdgeTriangleData) == 12);

	enum
	{
		MSH_EDGE_LINK_MASK		= 0x0fffffff,
		MSH_ACTIVE_EDGE_MASK	= 0x80000000,
		MSH_ACTIVE_VERTEX_MASK	= 0x40000000
	};

	class EdgeTriangleAC
	{
	public:
		PX_INLINE static PxU32		GetEdge01(const EdgeTriangleData& data)					{ return data.mLink[0] & MSH_EDGE_LINK_MASK;	}
		PX_INLINE static PxU32		GetEdge12(const EdgeTriangleData& data)					{ return data.mLink[1] & MSH_EDGE_LINK_MASK;	}
		PX_INLINE static PxU32		GetEdge20(const EdgeTriangleData& data)					{ return data.mLink[2] & MSH_EDGE_LINK_MASK;	}
		PX_INLINE static PxU32		GetEdge(const EdgeTriangleData& data, PxU32 i)			{ return data.mLink[i] & MSH_EDGE_LINK_MASK;	}

		PX_INLINE static PxIntBool	HasActiveEdge01(const EdgeTriangleData& data)			{ return PxIntBool(data.mLink[0] & MSH_ACTIVE_EDGE_MASK);	}
		PX_INLINE static PxIntBool	HasActiveEdge12(const EdgeTriangleData& data)			{ return PxIntBool(data.mLink[1] & MSH_ACTIVE_EDGE_MASK);	}
		PX_INLINE static PxIntBool	HasActiveEdge20(const EdgeTriangleData& data)			{ return PxIntBool(data.mLink[2] & MSH_ACTIVE_EDGE_MASK);	}
		PX_INLINE static PxIntBool	HasActiveEdge(const EdgeTriangleData& data, PxU32 i)	{ return PxIntBool(data.mLink[i] & MSH_ACTIVE_EDGE_MASK);	}
	};

	//! The edge-list creation structure.
	struct EDGELISTCREATE
	{
						EDGELISTCREATE() :
						NbFaces			(0),
						DFaces			(NULL),
						WFaces			(NULL),
						FacesToEdges	(false),
						EdgesToFaces	(false),
						Verts			(NULL),
						Epsilon			(0.1f)
						{}
				
		PxU32			NbFaces;	//!< Number of faces in source topo
		const PxU32*	DFaces;		//!< List of faces (dwords) or NULL
		const PxU16*	WFaces;		//!< List of faces (words) or NULL

		bool			FacesToEdges;
		bool			EdgesToFaces;
		const PxVec3*	Verts;
		float			Epsilon;
	};

	class PX_PHYSX_COMMON_API EdgeList : public PxUserAllocated
	{
		public:
												EdgeList();
												~EdgeList();

						bool					init(const EDGELISTCREATE& create);

						bool					load(PxInputStream& stream);

		PX_FORCE_INLINE	PxU32					getNbEdges()							const	{ return mNbEdges;						}
		PX_FORCE_INLINE	const EdgeData*			getEdges()								const	{ return mEdges;						}
		PX_FORCE_INLINE	const EdgeData&			getEdge(PxU32 edge_index)				const	{ return mEdges[edge_index];			}

		PX_FORCE_INLINE	PxU32					getNbFaces()							const	{ return mNbFaces;						}
		PX_FORCE_INLINE	const EdgeTriangleData* getEdgeTriangles()						const	{ return mEdgeFaces;					}
		PX_FORCE_INLINE	const EdgeTriangleData& getEdgeTriangle(PxU32 face_index)		const	{ return mEdgeFaces[face_index];		}

		PX_FORCE_INLINE	const EdgeDescData*		getEdgeToTriangles()					const	{ return mEdgeToTriangles;				}
		PX_FORCE_INLINE	const EdgeDescData&		getEdgeToTriangles(PxU32 edge_index)	const	{ return mEdgeToTriangles[edge_index];	}
		PX_FORCE_INLINE	const PxU32*			getFacesByEdges()						const	{ return mFacesByEdges;					}
		PX_FORCE_INLINE	PxU32					getFacesByEdges(PxU32 face_index)		const	{ return mFacesByEdges[face_index];		}

		private:
						// The edge list
						PxU32					mNbEdges;			//!< Number of edges in the list
						EdgeData*				mEdges;				//!< List of edges
						// Faces to edges
						PxU32					mNbFaces;			//!< Number of faces for which we have data
						EdgeTriangleData*		mEdgeFaces;			//!< Array of edge-triangles referencing mEdges
						// Edges to faces
						EdgeDescData*			mEdgeToTriangles;	//!< An EdgeDesc structure for each edge
						PxU32*					mFacesByEdges;		//!< A pool of face indices

						bool					createFacesToEdges(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces);
						bool					createEdgesToFaces(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces);
						bool					computeActiveEdges(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces, const PxVec3* verts, float epsilon);
	};

} // namespace Gu

}

#endif

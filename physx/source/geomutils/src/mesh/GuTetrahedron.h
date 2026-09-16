// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_TETRAHEDRON_H
#define GU_TETRAHEDRON_H

#include "foundation/PxVec3.h"
#include "foundation/PxUtilities.h"
#include "GuTriangle.h"

namespace physx
{
	namespace Gu
	{
		/**
		\brief Structure used to store indices for a triangles points. T is either PxU32 or PxU16

		*/

		template <class T>
		struct TetrahedronT
		{
			PX_INLINE	TetrahedronT() {}
			PX_INLINE	TetrahedronT(T a, T b, T c, T d) { v[0] = a; v[1] = b; v[2] = c; v[3] = d; }
			template <class TX>
			PX_INLINE	TetrahedronT(const TetrahedronT<TX>& other) { v[0] = other[0]; v[1] = other[1]; v[2] = other[2]; }
			PX_INLINE	T& operator[](T i) { return v[i]; }
			template<class TX>//any type of TriangleT<>, possibly with different T
			PX_INLINE	TetrahedronT<T>& operator=(const TetrahedronT<TX>& i) { v[0] = i[0]; v[1] = i[1]; v[2] = i[2]; v[3] = i[3]; return *this; }
			PX_INLINE	const T& operator[](T i) const { return v[i]; }

			PX_INLINE PxI32 indexOf(T i) const
			{
				if (v[0] == i) return 0;
				if (v[1] == i) return 1;
				if (v[2] == i) return 2;
				if (v[3] == i) return 3;
				return -1;
			}

			PX_INLINE bool contains(T id) const
			{
				return v[0] == id || v[1] == id || v[2] == id || v[3] == id;
			}

			PX_INLINE void replace(T oldId, T newId)
			{
				if (v[0] == oldId) v[0] = newId;
				if (v[1] == oldId) v[1] = newId;
				if (v[2] == oldId) v[2] = newId;
				if (v[3] == oldId) v[3] = newId;
			}

			PX_INLINE void sort()
			{
				if (v[0] > v[1]) PxSwap(v[0], v[1]);
				if (v[2] > v[3]) PxSwap(v[2], v[3]);
				if (v[0] > v[2]) PxSwap(v[0], v[2]);
				if (v[1] > v[3]) PxSwap(v[1], v[3]);
				if (v[1] > v[2]) PxSwap(v[1], v[2]);
			}

			PX_INLINE bool containsFace(const Gu::IndexedTriangleT<T>& triangle) const
			{
				return contains(triangle[0]) && contains(triangle[1]) && contains(triangle[2]);
			}

			PX_INLINE static bool identical(Gu::TetrahedronT<T> x, Gu::TetrahedronT<T> y)
			{
				x.sort();
				y.sort();
				return x.v[0] == y.v[0] && x.v[1] == y.v[1] && x.v[2] == y.v[2] && x.v[3] == y.v[3];
			}

			T v[4];	//vertex indices
		};
	}

}

#endif

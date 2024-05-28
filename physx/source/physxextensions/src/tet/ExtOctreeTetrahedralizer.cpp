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


#include "ExtOctreeTetrahedralizer.h"
#include "foundation/PxSort.h"
#include "foundation/PxQuat.h"
#include "CmRandom.h"

namespace physx
{
	namespace Ext
	{
		// -------------------------------------------------------------------------------------
		static const PxI32 childRelPos[8][3] = { {0,0,0}, {1,0,0},{0,1,0},{1,1,0}, {0,0,1}, {1,0,1},{0,1,1},{1,1,1} };

		static const PxI32 cubeCorners[8][3] = { {0,0,0}, {1,0,0},{1,1,0},{0,1,0}, {0,0,1}, {1,0,1},{1,1,1},{0,1,1} };
		//static const PxI32 cubeEdges[12][2] = { {0,1}, {1,2},{2,3},{3,0}, {0,4},{1,5},{2,6},{3,7},{4,5},{5,6},{6,7},{7,4} };

		static const PxI32 tetFaces[4][3] = { {2,1,0}, {0,1,3}, {1,2,3}, {2,0,3} };
		static const PxI32 cubeTets[6][4] = { {0,1,2,5}, {0,4,5,2}, {2,4,5,6}, {4,7,6,3}, {2,6,3,4}, {0,2,3,4} };

		static const PxI32 cubeTetNeighbors[6][4] = { {-1,-1,-1,1}, {-1,5,2,0}, {1,4,-1,-1},
		{-1,-1,-1,4}, {-1,2,3,5}, {-1,1,4,-1} };

		// -------------------------------------------------------------------------------------
		OctreeTetrahedralizer::OctreeTetrahedralizer()
		{
			clear();
		}

		// -------------------------------------------------------------------------------------
		void OctreeTetrahedralizer::clearTets()
		{
			tetVerts.clear();
			tetIds.clear();
			firstFreeTet = -1;
			currentTetMark = 0;
			tetMarks.clear();
			tetNeighbors.clear();
			renderVerts.clear();
			renderTriIds.clear();
			firstABBVert = 0;
		}

		// -------------------------------------------------------------------------------------
		void OctreeTetrahedralizer::clear()
		{
			surfaceVerts.clear();
			surfaceTriIds.clear();
			tetIds.clear();
			tetNeighbors.clear();
			cells.clear();
			clearTets();

			prevClip = -1.0f;
			prevScale = -1.0f;
		}

		// -----------------------------------------------------------------------------------
		void OctreeTetrahedralizer::createTree()
		{
			Bounds3 bounds;
			bounds.setEmpty();

			for (PxI32 i = 0; i < PxI32(surfaceVerts.size()); i++) 
			{
				const PxVec3& v = surfaceVerts[i];
				bounds.include(PxVec3d(PxF64(v.x), PxF64(v.y), PxF64(v.z)));
			}

			bounds.expand(0.01);
			PxVec3d dims = bounds.getDimensions();
			PxF64 size = PxMax(dims.x, PxMax(dims.y, dims.z));

			// create root
			cells.resize(1);
			cells.front().init();
			cells.front().orig = bounds.minimum;
			cells.front().size = size;

			cells.front().depth = 0;

			// insert vertices

			PxI32 numVerts = PxI32(surfaceVerts.size());

			for (PxI32 i = 0; i < numVerts; i++) 
			{
				treeInsertVert(0, i);
			}
		}

		// -------------------------------------------------------------------------------------

		PxI32 OctreeTetrahedralizer::Cell::getChildNr(const PxVec3d& p)
		{
			if (firstChild < 0)
				return -1;
			PxI32 nr = 0;
			if (p.x > orig.x + 0.5 * size) nr |= 1;
			if (p.y > orig.y + 0.5 * size) nr |= 2;
			if (p.z > orig.z + 0.5 * size) nr |= 4;
			return firstChild + nr;
		}

		// -------------------------------------------------------------------------------------

		void OctreeTetrahedralizer::treeInsertVert(PxI32 cellNr, PxI32 vertNr)
		{
			// inner node

			if (cells[cellNr].firstChild >= 0)
			{
				treeInsertVert(cells[cellNr].getChildNr(surfaceVerts[vertNr]), vertNr);
				return;
			}

			// add

			vertsOfCell.add(cellNr, vertNr);
			cells[cellNr].numVerts++;

			if (cells[cellNr].numVerts <= maxVertsPerCell ||
				cells[cellNr].depth >= maxTreeDepth)
				return;

			// split

			PxI32 firstChild = cells.size();
			cells[cellNr].firstChild = firstChild;
			cells.resize(cells.size() + 8);

			for (PxI32 i = 0; i < 8; i++) 
			{
				Cell& child = cells[firstChild + i];
				child.init();
				child.depth = cells[cellNr].depth + 1;
				child.size = cells[cellNr].size * 0.5;
				child.orig = cells[cellNr].orig + PxVec3d(
					childRelPos[i][0] * child.size,
					childRelPos[i][1] * child.size,
					childRelPos[i][2] * child.size);
			}

			PxI32 iterator, id;
			vertsOfCell.initIteration(cellNr, iterator);
			while (vertsOfCell.iterate(id, iterator))
				treeInsertVert(cells[cellNr].getChildNr(surfaceVerts[id]), id);

			vertsOfCell.removeAll(cellNr);
			cells[cellNr].numVerts = 0;
		}

		// -----------------------------------------------------------------------------------

		static PxVec3d jitter(const PxVec3d& p, Cm::RandomR250& random)
		{
			PxF64 eps = 0.001;
			return PxVec3d(
				p.x - eps + 2.0 * eps * PxF64(random.rand(0.0f, 1.0f)),
				p.y - eps + 2.0 * eps * PxF64(random.rand(0.0f, 1.0f)),
				p.z - eps + 2.0 * eps * PxF64(random.rand(0.0f, 1.0f))); 
		}

		// -----------------------------------------------------------------------------------

		void OctreeTetrahedralizer::createTetVerts(bool includeOctreeNodes)
		{
			tetVerts.clear();

			insideTester.init(surfaceVerts.begin(), PxI32(surfaceVerts.size()),
				surfaceTriIds.begin(), PxI32(surfaceTriIds.size()) / 3);

			for (PxI32 i = 0; i < PxI32(surfaceVerts.size()); i++)
			{
				const PxVec3& v = surfaceVerts[i];
				tetVerts.pushBack(PxVec3d(PxF64(v.x), PxF64(v.y), PxF64(v.z)));
			}

			if (includeOctreeNodes) 
			{

				PxArray<PxVec3d> treeVerts;

				for (PxI32 i = 0; i < PxI32(cells.size()); i++)
				{
					PxF64 s = cells[i].size;

					for (PxI32 j = 0; j < 8; j++) {
						PxVec3d p = cells[i].orig + PxVec3d(
							s * cubeCorners[j][0],
							s * cubeCorners[j][1],
							s * cubeCorners[j][2]);
						treeVerts.pushBack(p);
					}
				}

				// remove duplicates

				PxF64 eps = 1e-8;

				struct Ref
				{
					PxF64 d;
					PxI32 vertNr;
					bool operator < (const Ref& r) const 
					{
						return d < r.d;
					}
				};

				PxI32 numTreeVerts = PxI32(treeVerts.size());

				PxArray<Ref> refs(numTreeVerts);
				for (PxI32 i = 0; i < numTreeVerts; i++)
				{
					PxVec3d& p = treeVerts[i];
					refs[i].d = p.x + 0.3 * p.y + 0.1 * p.z;
					refs[i].vertNr = i;
				}

				PxSort(refs.begin(), refs.size());
				PxArray<bool> duplicate(numTreeVerts, false);
				
				PxI32 nr = 0;
				Cm::RandomR250 random(0);
				while (nr < numTreeVerts) 
				{
					Ref& r = refs[nr];
					nr++;
					if (duplicate[r.vertNr])
						continue;
					PxVec3d& p = treeVerts[r.vertNr];
					PxVec3d v = jitter(p, random);

					if (insideTester.isInside(PxVec3(PxReal(v.x), PxReal(v.y), PxReal(v.z))))
						tetVerts.pushBack(jitter(p, random));

					PxI32 i = nr;
					while (i < numTreeVerts && fabs(refs[i].d - r.d) < eps)
					{
						PxVec3d& q = treeVerts[refs[i].vertNr];
						if ((p - q).magnitude() < eps)
							duplicate[refs[i].vertNr] = true;
						i++;
					}
				}
			}			
		}

		// -----------------------------------------------------------------------------------
		PxVec3d OctreeTetrahedralizer::getTetCenter(PxI32 tetNr) const
		{
			return (tetVerts[tetIds[4 * tetNr]] +
				tetVerts[tetIds[4 * tetNr + 1]] +
				tetVerts[tetIds[4 * tetNr + 2]] +
				tetVerts[tetIds[4 * tetNr + 3]]) * 0.25;
		}

		// -----------------------------------------------------------------------------------
		void OctreeTetrahedralizer::treeInsertTet(PxI32 tetNr)
		{
			PxVec3d center = getTetCenter(tetNr);

			PxI32 cellNr = 0;
			while (cellNr >= 0) 
			{
				Cell& c = cells[cellNr];
				if (c.closestTetNr < 0)
					c.closestTetNr = tetNr;
				else 
				{
					PxVec3d cellCenter = c.orig + PxVec3d(c.size, c.size, c.size) * 0.5;
					PxVec3d closest = getTetCenter(c.closestTetNr);
					if ((cellCenter - center).magnitudeSquared() < (cellCenter - closest).magnitudeSquared())
						c.closestTetNr = tetNr;
				}
				cellNr = cells[cellNr].getChildNr(center);
			}
		}

		// -----------------------------------------------------------------------------------
		void OctreeTetrahedralizer::treeRemoveTet(PxI32 tetNr)
		{
			PxVec3d center = getTetCenter(tetNr);

			PxI32 cellNr = 0;
			while (cellNr >= 0)
			{
				Cell& c = cells[cellNr];
				if (c.closestTetNr == tetNr)
					c.closestTetNr = -1;
				cellNr = cells[cellNr].getChildNr(center);
			}
		}

		void resizeFast(PxArray<PxI32>& arr, PxU32 newSize, PxI32 value = 0)
		{
			if (newSize < arr.size())
				arr.removeRange(newSize, arr.size() - newSize);
			else
			{
				while (arr.size() < newSize)
					arr.pushBack(value);
			}
		}

		// -----------------------------------------------------------------------------------
		PxI32 OctreeTetrahedralizer::getNewTetNr()
		{
			PxI32 newTetNr;
			if (firstFreeTet >= 0) 
			{ 		
				// take from free list
				newTetNr = firstFreeTet;
				firstFreeTet = tetIds[4 * firstFreeTet];
			}
			else 
			{		
				// append
				newTetNr = PxI32(tetIds.size()) / 4;
				resizeFast(tetIds, tetIds.size() + 4);
				resizeFast(tetMarks, newTetNr + 1, 0);
				resizeFast(tetNeighbors, tetIds.size(), -1);
			}
			return newTetNr;
		}

		// -----------------------------------------------------------------------------------
		void OctreeTetrahedralizer::removeTetNr(PxI32 tetNr)
		{
			// add to free list
			tetIds[4 * tetNr] = firstFreeTet;
			tetIds[4 * tetNr + 1] = -1;
			tetIds[4 * tetNr + 2] = -1;
			tetIds[4 * tetNr + 3] = -1;
			firstFreeTet = tetNr;
		}

		// -----------------------------------------------------------------------------------
		bool OctreeTetrahedralizer::findSurroundingTet(const PxVec3d& p, PxI32 startTetNr, PxI32& tetNr)
		{
			currentTetMark++;
			tetNr = startTetNr;

			bool found = false;

			while (!found) 
			{
				if (tetNr < 0 || tetMarks[tetNr] == currentTetMark)    // circular, something went wrong
					break;
				tetMarks[tetNr] = currentTetMark;

				PxVec3d c = getTetCenter(tetNr);

				PxI32* ids = &tetIds[4 * tetNr];
				PxF64 minT = DBL_MAX;
				PxI32 minFaceNr = -1;

				for (PxI32 i = 0; i < 4; i++) 
				{
					const PxVec3d& p0 = tetVerts[ids[tetFaces[i][0]]];
					const PxVec3d& p1 = tetVerts[ids[tetFaces[i][1]]];
					const PxVec3d& p2 = tetVerts[ids[tetFaces[i][2]]];

					PxVec3d n = (p1 - p0).cross(p2 - p0);
					n = n.getNormalized();
					PxF64 hp = (p - p0).dot(n);
					PxF64 hc = (c - p0).dot(n);

					PxF64 t = hp - hc;
					if (t == 0.0)
						continue;
					t = -hc / t;     // time when c -> p hits the face
					if (t >= 0.0 && t < minT) {     // in front and new min
						minT = t;
						minFaceNr = i;
					}
				}
				if (minT >= 1.0)
					found = true;
				else
					tetNr = tetNeighbors[4 * tetNr + minFaceNr];
			}

			return found;
		}

		// -----------------------------------------------------------------------------------
		bool OctreeTetrahedralizer::findSurroundingTet(const PxVec3d& p, PxI32& tetNr)
		{
			PxI32 startTet = 0;
			PxI32 cellNr = 0;
			while (cellNr >= 0) 
			{
				if (cells[cellNr].closestTetNr >= 0)
					startTet = cells[cellNr].closestTetNr;
				cellNr = cells[cellNr].getChildNr(p);
			}

			return findSurroundingTet(p, startTet, tetNr);
		}

		// -----------------------------------------------------------------------------------
		static PxVec3d getCircumCenter(PxVec3d& p0, PxVec3d& p1, PxVec3d& p2, PxVec3d& p3)
		{
			PxVec3d b = p1 - p0;
			PxVec3d c = p2 - p0;
			PxVec3d d = p3 - p0;
			PxF64 det = 2.0 * (b.x*(c.y*d.z - c.z*d.y) - b.y*(c.x*d.z - c.z*d.x) + b.z*(c.x*d.y - c.y*d.x));
			if (det == 0.0)
				return p0;
			else 
			{
				PxVec3d v = c.cross(d)*b.dot(b) + d.cross(b)*c.dot(c) + b.cross(c)*d.dot(d);
				v /= det;
				return p0 + v;
			}
		}


		// -----------------------------------------------------------------------------------
		bool OctreeTetrahedralizer::meshInsertTetVert(PxI32 vertNr)
		{
			const PxVec3d& p = tetVerts[vertNr];
			PxI32 surroundingTetNr;
			if (!findSurroundingTet(p, surroundingTetNr))
				return false;

			// find violating tets

			violatingTets.clear();
			stack.clear();
			currentTetMark++;
			stack.pushBack(surroundingTetNr);

			while (!stack.empty()) 
			{
				PxI32 tetNr = stack.back();
				stack.popBack();
				if (tetMarks[tetNr] == currentTetMark)
					continue;
				tetMarks[tetNr] = currentTetMark;
				violatingTets.pushBack(tetNr);

				for (PxI32 i = 0; i < 4; i++) 
				{
					PxI32 n = tetNeighbors[4 * tetNr + i];
					if (n < 0 || tetMarks[n] == currentTetMark)
						continue;

					// Delaunay condition test
					PxI32* ids = &tetIds[4 * n];
					PxVec3d c = getCircumCenter(tetVerts[ids[0]], tetVerts[ids[1]], tetVerts[ids[2]], tetVerts[ids[3]]);

					PxF64 r2 = (tetVerts[ids[0]] - c).magnitudeSquared();
					if ((p - c).magnitudeSquared() < r2)
						stack.pushBack(n);
				}
			}

			// remove old tets, create new ones

			edges.clear();
			Edge e;

			for (PxI32 i = 0; i < PxI32(violatingTets.size()); i++) 
			{
				PxI32 tetNr = violatingTets[i];

				// copy information before we delete it
				PxI32 ids[4], ns[4];
				for (PxI32 j = 0; j < 4; j++) {
					ids[j] = tetIds[4 * tetNr + j];
					ns[j] = tetNeighbors[4 * tetNr + j];
				}

				// delete the tetrahedron
				treeRemoveTet(tetNr);
				removeTetNr(tetNr);

				// visit neighbors
				for (PxI32 j = 0; j < 4; j++) {
					PxI32 n = ns[j];
					if (n < 0 || tetMarks[n] != currentTetMark) 
					{
						// no neighbor or neighbor is not-violating -> we are facing the border

						// create new tetrahedron

						PxI32 newTetNr = getNewTetNr();
						PxI32 id0 = ids[tetFaces[j][2]];
						PxI32 id1 = ids[tetFaces[j][1]];
						PxI32 id2 = ids[tetFaces[j][0]];

						tetIds[4 * newTetNr] = id0;
						tetIds[4 * newTetNr + 1] = id1;
						tetIds[4 * newTetNr + 2] = id2;
						tetIds[4 * newTetNr + 3] = vertNr;

						treeInsertTet(newTetNr);

						tetNeighbors[4 * newTetNr] = n;

						if (n >= 0) 
						{
							for (PxI32 k = 0; k < 4; k++) 
							{
								if (tetNeighbors[4 * n + k] == tetNr)
									tetNeighbors[4 * n + k] = newTetNr;
							}
						}

						// will set the neighbors among the new tetrahedra later

						tetNeighbors[4 * newTetNr + 1] = -1;
						tetNeighbors[4 * newTetNr + 2] = -1;
						tetNeighbors[4 * newTetNr + 3] = -1;

						e.init(id0, id1, newTetNr, 1); edges.pushBack(e);
						e.init(id1, id2, newTetNr, 2); edges.pushBack(e);
						e.init(id2, id0, newTetNr, 3); edges.pushBack(e);
					}
				}     // next neighbor
			}  // next violating tetrahedron

			// fix neighbors

			PxSort(edges.begin(), edges.size());

			PxI32 nr = 0;
			while (nr < PxI32(edges.size())) 
			{
				Edge& e0 = edges[nr];
				nr++;
				if (nr < PxI32(edges.size()) && edges[nr] == e0) 
				{
					Edge& e1 = edges[nr];

					tetNeighbors[4 * e0.tetNr + e0.faceNr] = e1.tetNr;
					tetNeighbors[4 * e1.tetNr + e1.faceNr] = e0.tetNr;
					nr++;
				}
			}

			return true;
		}

		// -----------------------------------------------------------------------------------
		static PxF64 tetQuality(const PxVec3d& p0, const PxVec3d& p1, const PxVec3d& p2, const PxVec3d& p3)
		{
			PxVec3d d0 = p1 - p0;
			PxVec3d d1 = p2 - p0;
			PxVec3d d2 = p3 - p0;
			PxVec3d d3 = p2 - p1;
			PxVec3d d4 = p3 - p2;
			PxVec3d d5 = p1 - p3;

			PxF64 s0 = d0.magnitudeSquared();
			PxF64 s1 = d1.magnitudeSquared();
			PxF64 s2 = d2.magnitudeSquared();
			PxF64 s3 = d3.magnitudeSquared();
			PxF64 s4 = d4.magnitudeSquared();
			PxF64 s5 = d5.magnitudeSquared();

			PxF64 ms = (s0 + s1 + s2 + s3 + s4 + s5) / 6.0;
			PxF64 rms = sqrt(ms);

			static const PxF64 s = 12.0 / sqrt(2.0);

			PxF64 vol = d0.dot(d1.cross(d2)) / 6.0;
			return s * vol / (rms * rms * rms);		// 1.0 for regular tetrahedron
		}

		// -----------------------------------------------------------------------------------
		void OctreeTetrahedralizer::pruneTets()
		{
			insideTester.init(surfaceVerts.begin(), PxI32(surfaceVerts.size()),
				surfaceTriIds.begin(), PxI32(surfaceTriIds.size()) / 3);

			static PxF64 minQuality = 0.01;

			PxI32 numTets = tetIds.size() / 4;

			PxI32 num = 0;

			for (PxI32 i = 0; i < numTets; i++) 
			{
				bool remove = false;

				PxI32* ids = &tetIds[4 * i];
				for (PxI32 j = 0; j < 4; j++) 
				{
					if (ids[j] >= firstABBVert)
						remove = true;
				}

				if (ids[0] < 0 || ids[1] < 0 || ids[2] < 0 || ids[3] < 0)
					remove = true;

				if (!remove)
				{
					PxVec3d c = getTetCenter(i);
					if (!insideTester.isInside(PxVec3(PxReal(c.x), PxReal(c.y), PxReal(c.z))))
						remove = true;

					if (tetQuality(tetVerts[ids[0]], tetVerts[ids[1]],
						tetVerts[ids[2]], tetVerts[ids[3]]) < minQuality)
						continue;
				}

				if (remove)
					continue;

				for (PxI32 j = 0; j < 4; j++)
					tetIds[4 * num + j] = ids[j];
				num++;
			}

			tetIds.resize(4 * num);
		}

		// -----------------------------------------------------------------------------------
		void OctreeTetrahedralizer::createTetMesh(const PxArray<PxVec3>& verts, const PxArray<PxU32>& triIds,
			bool includeOctreeNodes, PxI32 _maxVertsPerCell, PxI32 _maxTreeDepth)
		{

			this->surfaceVerts = verts;
			surfaceTriIds.resize(triIds.size());
			for (PxU32 i = 0; i < triIds.size(); i++)
				this->surfaceTriIds[i] = triIds[i];
			this->maxVertsPerCell = _maxVertsPerCell;
			this->maxTreeDepth = _maxTreeDepth;
			createTree();

			clearTets();

			if (cells.empty())
				return;

			createTetVerts(includeOctreeNodes);
			if (tetVerts.empty())
				return;

			for (PxI32 i = 0; i < PxI32(cells.size()); i++)
				cells[i].closestTetNr = -1;

			// create aabb tets

			Bounds3 bounds;
			bounds.setEmpty();
			for (PxI32 i = 0; i < PxI32(tetVerts.size()); i++)
				bounds.include(tetVerts[i]);
			bounds.expand(bounds.getDimensions().magnitude() * 0.1);

			firstABBVert = PxI32(tetVerts.size());
			PxVec3d dims = bounds.getDimensions();

			for (PxI32 i = 0; i < 8; i++) 
			{
				tetVerts.pushBack(bounds.minimum + PxVec3d(
					cubeCorners[i][0] * dims.x,
					cubeCorners[i][1] * dims.y,
					cubeCorners[i][2] * dims.z));
			}

			for (PxI32 i = 0; i < 6; i++) 
			{
				for (PxI32 j = 0; j < 4; j++) 
				{
					tetIds.pushBack(firstABBVert + cubeTets[i][j]);
					tetNeighbors.pushBack(cubeTetNeighbors[i][j]);
				}
				treeInsertTet(i);
			}

			tetMarks.resize(6, 0);

			for (PxI32 i = 0; i < firstABBVert; i++)
			{
				meshInsertTetVert(i);
			}

			pruneTets();

			renderTriIds.clear();
			renderVerts.clear();
		}

		// -----------------------------------------------------------------------------------
		void OctreeTetrahedralizer::readBack(PxArray<PxVec3> &outputTetVerts, PxArray<PxU32> &outputTetIds)
		{
			outputTetVerts.resize(tetVerts.size());

			for (PxU32 i = 0; i < tetVerts.size(); i++) 
			{
				PxVec3d &v = tetVerts[i];
				outputTetVerts[i] = PxVec3(PxReal(v.x), PxReal(v.y), PxReal(v.z));
			}

			outputTetIds.resize(tetIds.size());

			for (PxU32 i = 0; i < tetIds.size(); i++)
				outputTetIds[i] = PxU32(tetIds[i]);
		}
	}
}


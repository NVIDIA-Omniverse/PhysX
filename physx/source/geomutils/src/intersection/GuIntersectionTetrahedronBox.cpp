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

#include "GuIntersectionTetrahedronBox.h"
#include "foundation/PxBasicTemplates.h"
#include "GuIntersectionTriangleBox.h"
#include "GuBox.h"

using namespace physx;

namespace physx
{
namespace Gu
{
	bool intersectTetrahedronBox(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxBounds3& box)
	{
		if (box.contains(a) || box.contains(b) || box.contains(c) || box.contains(d))
			return true;

		PxBounds3 tetBox = PxBounds3::empty();
		tetBox.include(a);
		tetBox.include(b);
		tetBox.include(c);
		tetBox.include(d);
		tetBox.fattenFast(1e-6f);

		if (!box.intersects(tetBox))
			return false;

		Gu::BoxPadded boxP;
		boxP.center = box.getCenter();
		boxP.extents = box.getExtents();
		boxP.rot = PxMat33(PxIdentity);
		return intersectTriangleBox(boxP, a, b, c) || intersectTriangleBox(boxP, a, b, d) || intersectTriangleBox(boxP, a, c, d) || intersectTriangleBox(boxP, b, c, d);
	}
}
}

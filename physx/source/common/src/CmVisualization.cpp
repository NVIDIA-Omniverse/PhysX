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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "CmVisualization.h"

using namespace physx;
using namespace Cm;

static const PxU32 gLimitColor = PxU32(PxDebugColor::eARGB_YELLOW);

void Cm::visualizeJointFrames(PxRenderOutput& out, PxReal scale, const PxTransform& parent, const PxTransform& child)
{
	if(scale==0.0f)
		return;

	out << parent << PxDebugBasis(PxVec3(scale, scale, scale) * 1.5f,
		PxU32(PxDebugColor::eARGB_DARKRED), PxU32(PxDebugColor::eARGB_DARKGREEN), PxU32(PxDebugColor::eARGB_DARKBLUE));
	out << child << PxDebugBasis(PxVec3(scale, scale, scale));	
}

void Cm::visualizeLinearLimit(PxRenderOutput& out, PxReal scale, const PxTransform& t0, const PxTransform& /*t1*/, PxReal value)
{
	if(scale==0.0f)
		return;

	// debug circle is around z-axis, and we want it around x-axis
	PxTransform r(t0.p+value*t0.q.getBasisVector0(), t0.q*PxQuat(PxPi/2,PxVec3(0,1.f,0)));
	out << gLimitColor;
	out << PxTransform(PxIdentity);
	out << PxDebugArrow(t0.p,r.p-t0.p);

	out << r << PxDebugCircle(20, scale*0.3f);
}

void Cm::visualizeAngularLimit(PxRenderOutput& out, PxReal scale, const PxTransform& t, PxReal lower, PxReal upper)
{
	if(scale==0.0f)
		return;

	out << t << gLimitColor;
	
	out << PxRenderOutput::LINES 
		<< PxVec3(0) << PxVec3(0, PxCos(lower), PxSin(lower)) * scale
		<< PxVec3(0) << PxVec3(0, PxCos(upper), PxSin(upper)) * scale;

	out << PxRenderOutput::LINESTRIP;
	PxReal angle = lower, step = (upper-lower)/20;

	for(PxU32 i=0; i<=20; i++, angle += step)
		out << PxVec3(0, PxCos(angle), PxSin(angle)) * scale;
}

void Cm::visualizeLimitCone(PxRenderOutput& out, PxReal scale, const PxTransform& t, PxReal tanQSwingY, PxReal tanQSwingZ)
{
	if(scale==0.0f)
		return;

	out << t << gLimitColor;
	out << PxRenderOutput::LINES;

	PxVec3 prev(0,0,0);
	
	const PxU32 LINES = 32;

	for(PxU32 i=0;i<=LINES;i++)
	{
		PxReal angle = 2*PxPi/LINES*i;
		PxReal c = PxCos(angle), s = PxSin(angle);
		PxVec3 rv(0,-tanQSwingZ*s, tanQSwingY*c);
		PxReal rv2 = rv.magnitudeSquared();
		PxQuat q = PxQuat(0,2*rv.y,2*rv.z,1-rv2) * (1/(1+rv2));
		PxVec3 a = q.rotate(PxVec3(1.0f,0,0)) * scale;

		out << prev << a << PxVec3(0) << a;
		prev = a;
	}
}

void Cm::visualizeDoubleCone(PxRenderOutput& out, PxReal scale, const PxTransform& t, PxReal angle)
{
	if(scale==0.0f)
		return;

	out << t << gLimitColor;

	const PxReal height = PxTan(angle);

	const PxU32 LINES = 32;

	out << PxRenderOutput::LINESTRIP;

	const PxReal step = PxPi*2/LINES;

	for(PxU32 i=0; i<=LINES; i++)
		out << PxVec3(height, PxCos(step * i), PxSin(step * i)) * scale;

	angle = 0;
	out << PxRenderOutput::LINESTRIP;
	for(PxU32 i=0; i<=LINES; i++, angle += PxPi*2/LINES)
		out << PxVec3(-height, PxCos(step * i), PxSin(step * i)) * scale;

	angle = 0;
	out << PxRenderOutput::LINES;
	for(PxU32 i=0;i<LINES;i++, angle += PxPi*2/LINES)
	{
		out << PxVec3(0) << PxVec3(-height, PxCos(step * i), PxSin(step * i)) * scale;
		out << PxVec3(0) << PxVec3(height, PxCos(step * i), PxSin(step * i)) * scale;
	}
}

void Cm::renderOutputDebugBox(PxRenderOutput& out, const PxBounds3& box)
{
	out << PxDebugBox(box, true);
}

void Cm::renderOutputDebugCircle(PxRenderOutput& out, PxU32 s, PxReal r)
{
	out << PxDebugCircle(s, r);
}

void Cm::renderOutputDebugBasis(PxRenderOutput& out, const PxDebugBasis& basis)
{
	out << basis;
}

void Cm::renderOutputDebugArrow(PxRenderOutput& out, const PxDebugArrow& arrow)
{
	out << arrow;
}

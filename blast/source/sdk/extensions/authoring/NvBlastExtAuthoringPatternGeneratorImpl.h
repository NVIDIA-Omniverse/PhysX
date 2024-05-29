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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTEXTAUTHORINGPATTERNGENERATORIMPL_H
#define NVBLASTEXTAUTHORINGPATTERNGENERATORIMPL_H

#include "NvBlastExtAuthoringTypes.h"
#include "NvBlastExtAuthoringPatternGenerator.h"


namespace Nv
{
namespace Blast
{

    class PatternGeneratorImpl : public PatternGenerator
    {
    public:
        virtual DamagePattern* generateUniformPattern(const UniformPatternDesc* desc) override;
        virtual DamagePattern* generateBeamPattern(const BeamPatternDesc* desc) override;
        virtual DamagePattern* generateRegularRadialPattern(const RegularRadialPatternDesc* desc) override;
        virtual void release() override;

        virtual DamagePattern* generateVoronoiPattern(uint32_t pointCount, const NvcVec3* points, int32_t interiorMaterialId) override;
    private:
        DamagePattern* generateVoronoiPatternInternal(uint32_t pointCount, const NvcVec3* points, int32_t interiorMaterialId, float angle = 0.0f);
    };


} // namespace Blast
} // namespace Nv

#endif // ifndef NVBLASTEXTAUTHORINGMESHCLEANER_H
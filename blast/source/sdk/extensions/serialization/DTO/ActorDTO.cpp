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
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.


#include "ActorDTO.h"
#include "NvBlastGlobals.h"
#include "NvBlastIDDTO.h"
#include "NvBlastChunkDTO.h"
#include "NvBlastBondDTO.h"


namespace Nv
{
namespace Blast
{

bool ActorDTO::serialize(Nv::Blast::Serialization::Actor::Builder builder, const Nv::Blast::Actor* poco)
{
    builder.setFamilyOffset(poco->getFamilyOffset());
    builder.setFirstVisibleChunkIndex(poco->getFirstVisibleChunkIndex());
    builder.setVisibleChunkCount(poco->getVisibleChunkCount());
    builder.setFirstGraphNodeIndex(poco->getFirstGraphNodeIndex());
    builder.setGraphNodeCount(poco->getGraphNodeCount());
    builder.setLeafChunkCount(poco->getLeafChunkCount());

    return true;
}


Nv::Blast::Actor* ActorDTO::deserialize(Nv::Blast::Serialization::Actor::Reader reader)
{
    NV_UNUSED(reader);
    return nullptr;
}


bool ActorDTO::deserializeInto(Nv::Blast::Serialization::Actor::Reader reader, Nv::Blast::Actor* poco)
{
    poco->setFamilyOffset(reader.getFamilyOffset());
    poco->setFirstVisibleChunkIndex(reader.getFirstVisibleChunkIndex());
    poco->setVisibleChunkCount(reader.getVisibleChunkCount());
    poco->setFirstGraphNodeIndex(reader.getFirstGraphNodeIndex());
    poco->setGraphNodeCount(reader.getGraphNodeCount());
    poco->setLeafChunkCount(reader.getLeafChunkCount());

    return true;
}

}   // namespace Blast
}   // namespace Nv

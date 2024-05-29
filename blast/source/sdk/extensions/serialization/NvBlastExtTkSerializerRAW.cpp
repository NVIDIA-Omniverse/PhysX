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


#include "NvBlastExtSerializationInternal.h"
#include "NvBlastTkFramework.h"
#include "NvBlastTkAsset.h"
#include "NvBlast.h"


namespace Nv
{
namespace Blast
{

// Legacy IDs
struct ExtTkSerializationLegacyID
{
    enum Enum
    {
        Framework = NVBLAST_FOURCC('T', 'K', 'F', 'W'), //!< TkFramework identifier token, used in serialization
        Asset =     NVBLAST_FOURCC('A', 'S', 'S', 'T'), //!< TkAsset identifier token, used in serialization
        Family =    NVBLAST_FOURCC('A', 'C', 'T', 'F'), //!< TkFamily identifier token, used in serialization
    };
};


// Legacy object format versions
struct ExtTkSerializationLegacyAssetVersion
{
    enum Enum
    {
        /** Initial version */
        Initial,

        //  New formats must come before Count.  They should be given descriptive names with more information in comments.

        /** The number of serialized formats. */
        Count,

        /** The current version.  This should always be Count-1 */
        Current = Count - 1
    };
};

struct ExtTkSerializationLegacyFamilyVersion
{
    enum Enum
    {
        /** Initial version */
        Initial,

        //  New formats must come before Count.  They should be given descriptive names with more information in comments.

        /** The number of serialized formats. */
        Count,

        /** The current version.  This should always be Count-1 */
        Current = Count - 1
    };
};


static bool deserializeTkObjectHeader(uint32_t& legacyTypeID, uint32_t& legacyVersion, NvBlastID& objID, uint64_t& userIntData, ExtIStream& stream)
{
    // Read framework ID
    uint32_t fwkID = 0; // Initialize to silence some compilers
    stream >> fwkID;
    if (fwkID != ExtTkSerializationLegacyID::Framework)
    {
        NVBLAST_LOG_ERROR("deserializeTkObjectHeader: stream does not contain a BlastTk legacy object.");
        return false;
    }

    // Read object class ID
    stream >> legacyTypeID;

    // Read object class version and ensure it's current
    stream >> legacyVersion;

    // Object ID
    stream.read(objID.data, sizeof(NvBlastID));

    // Serializable user data
    uint32_t lsd, msd;
    stream >> lsd >> msd;
    userIntData = static_cast<uint64_t>(msd) << 32 | static_cast<uint64_t>(lsd);

    return !stream.fail();
}


TkAsset* deserializeTkAsset(ExtIStream& stream, TkFramework& framework)
{
    // Deserializer header
    uint32_t legacyTypeID;
    uint32_t legacyVersion;
    NvBlastID objID;
    uint64_t userIntData;
    if (!deserializeTkObjectHeader(legacyTypeID, legacyVersion, objID, userIntData, stream))
    {
        return nullptr;
    }

    if (legacyTypeID != ExtTkSerializationLegacyID::Asset)
    {
        NVBLAST_LOG_ERROR("deserializeTkAsset: stream does not contain a BlastTk legacy asset.");
        return nullptr;
    }

    if (legacyVersion > ExtTkSerializationLegacyAssetVersion::Current)
    {
        NVBLAST_LOG_ERROR("deserializeTkAsset: stream contains a BlastTk legacy asset which is in an unknown version.");
        return nullptr;
    }

    // LL asset
    uint32_t assetSize;
    stream >> assetSize;
    NvBlastAsset* llAsset = static_cast<NvBlastAsset*>(NVBLAST_ALLOC_NAMED(assetSize, "deserializeTkAsset"));
    stream.read(reinterpret_cast<char*>(llAsset), assetSize);

    // Joint descs
    uint32_t jointDescCount;
    stream >> jointDescCount;
    std::vector<TkAssetJointDesc> jointDescs(jointDescCount);
    for (uint32_t i = 0; i < jointDescs.size(); ++i)
    {
        TkAssetJointDesc& jointDesc = jointDescs[i];
        stream >> jointDesc.nodeIndices[0];
        stream >> jointDesc.nodeIndices[1];
        stream >> jointDesc.attachPositions[0].x;
        stream >> jointDesc.attachPositions[0].y;
        stream >> jointDesc.attachPositions[0].z;
        stream >> jointDesc.attachPositions[1].x;
        stream >> jointDesc.attachPositions[1].y;
        stream >> jointDesc.attachPositions[1].z;
    }

    if (stream.fail())
    {
        NVBLAST_FREE(llAsset);
        return nullptr;
    }

    TkAsset* asset = framework.createAsset(llAsset, jointDescs.data(), (uint32_t)jointDescs.size(), true);

    NvBlastID zeroID;
    memset(zeroID.data, 0, sizeof(zeroID));
    if (!memcmp(zeroID.data, objID.data, sizeof(NvBlastID)))
    {
        asset->setID(objID);
    }
    asset->userIntData = userIntData;

    return asset;
}

}   // namespace Blast
}   // namespace Nv

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
// Copyright (c) 2016-2022 NVIDIA Corporation. All rights reserved.


#ifndef BLASTBASETEST_H
#define BLASTBASETEST_H


#include "NvBlastTkFramework.h"

#include "gtest/gtest.h"

#include "NvBlast.h"

#include "TestAssets.h"

#include "NvBlastGlobals.h"

#include <ostream>


template<int FailLevel, int Verbosity>
class BlastBaseTest : public testing::Test, public Nv::Blast::ErrorCallback
{
public:
    BlastBaseTest()
    {
        NvBlastGlobalSetErrorCallback(this);
    }

    // A zeroing alloc with the same signature as malloc
    static void* alignedZeroedAlloc(size_t size)
    {
        return memset(NVBLAST_ALLOC(size), 0, size);
    }

    static void alignedFree(void* mem)
    {
        NVBLAST_FREE(mem);
    }

    // Message log for blast functions
    static void messageLog(int type, const char* msg, const char* file, int line)
    {
        if (FailLevel >= type)
        {
            switch (type)
            {
            case NvBlastMessage::Error:     EXPECT_TRUE(false) << "NvBlast Error message in " << file << "(" << line << "): " << msg << "\n";   break;
            case NvBlastMessage::Warning:   EXPECT_TRUE(false) << "NvBlast Warning message in " << file << "(" << line << "): " << msg << "\n"; break;
            case NvBlastMessage::Info:      EXPECT_TRUE(false) << "NvBlast Info message in " << file << "(" << line << "): " << msg << "\n";    break;
            case NvBlastMessage::Debug:     EXPECT_TRUE(false) << "NvBlast Debug message in " << file << "(" << line << "): " << msg << "\n";   break;
            }
        }
        else
        if (Verbosity > 0)
        {
            switch (type)
            {
            case NvBlastMessage::Error:     std::cout << "NvBlast Error message in " << file << "(" << line << "): " << msg << "\n";    break;
            case NvBlastMessage::Warning:   std::cout << "NvBlast Warning message in " << file << "(" << line << "): " << msg << "\n";  break;
            case NvBlastMessage::Info:      std::cout << "NvBlast Info message in " << file << "(" << line << "): " << msg << "\n";     break;
            case NvBlastMessage::Debug:     std::cout << "NvBlast Debug message in " << file << "(" << line << "): " << msg << "\n";    break;
            }
        }
    }

    // ErrorCallback interface
    virtual void reportError(Nv::Blast::ErrorCode::Enum code, const char* message, const char* file, int line) override
    {
        uint32_t failMask = 0;
        switch (FailLevel)
        {
        case NvBlastMessage::Debug:
        case NvBlastMessage::Info:      failMask |= Nv::Blast::ErrorCode::eDEBUG_INFO;
        case NvBlastMessage::Warning:   failMask |= Nv::Blast::ErrorCode::eDEBUG_WARNING;
        case NvBlastMessage::Error:     failMask |= Nv::Blast::ErrorCode::eABORT | Nv::Blast::ErrorCode::eABORT | Nv::Blast::ErrorCode::eINTERNAL_ERROR | Nv::Blast::ErrorCode::eOUT_OF_MEMORY | Nv::Blast::ErrorCode::eINVALID_OPERATION | Nv::Blast::ErrorCode::eINVALID_PARAMETER;
        default: break;
        }

        if (!(failMask & code) && Verbosity <= 0)
        {
            return;
        }

        std::string output = "NvBlast Test ";
        switch (code)
        {
        case Nv::Blast::ErrorCode::eNO_ERROR:                                           break;
        case Nv::Blast::ErrorCode::eDEBUG_INFO:         output += "Debug Info";         break;
        case Nv::Blast::ErrorCode::eDEBUG_WARNING:      output += "Debug Warning";      break;
        case Nv::Blast::ErrorCode::eINVALID_PARAMETER:  output += "Invalid Parameter";  break;
        case Nv::Blast::ErrorCode::eINVALID_OPERATION:  output += "Invalid Operation";  break;
        case Nv::Blast::ErrorCode::eOUT_OF_MEMORY:      output += "Out of Memory";      break;
        case Nv::Blast::ErrorCode::eINTERNAL_ERROR:     output += "Internal Error";     break;
        case Nv::Blast::ErrorCode::eABORT:              output += "Abort";              break;
        case Nv::Blast::ErrorCode::ePERF_WARNING:       output += "Perf Warning";   break;
        default:                                        FAIL();
        }
        output += std::string(" message in ") + file + "(" + std::to_string(line) + "): " + message + "\n";

        if (failMask & code)
        {
            EXPECT_TRUE(false) << output;
        }
        else
        {
            std::cout << output;
        }
    }
};


#endif // #ifndef BLASTBASETEST_H

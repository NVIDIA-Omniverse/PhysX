// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BENCHMARK_OUTPUT_H
#define BENCHMARK_OUTPUT_H

#include <map>
#include <vector>

struct BmResult;
class BmRecord;

typedef void (*PrintfCbFunc)(const char*, ...);

namespace carb
{
    namespace filesystem
    {
    inline namespace v1
    {
        struct File;
    }
    }
}

class BmOutput
{
public:
    BmOutput(bool regenerate, uint32_t slop, bool console, const char* reportFile, PrintfCbFunc printfCb);
    ~BmOutput();
    void printHeaders() const;
    bool emit(const char* name, const BmResult& result);
    void dump(const char* name, const BmRecord& recorder) const;

private:
    struct Record
    {
        char name[256];
        uint32_t avgTime, stdDevTime, maxMem;
        uint32_t timeNegativeTolerance;
        uint32_t timePositiveTolerance;
        uint32_t memNegativeTolerance;
        uint32_t memPositiveTolerance;        
    };    

    static float performanceDelta(uint32_t current, uint32_t ref);

    bool mRegenerate;
    uint32_t mSlop;
    bool mConsole;

    std::vector<Record*> mBaselineRecords;
    std::vector<Record*> mBaseline;
    PrintfCbFunc mPrintfCb;
    carb::filesystem::File* mReportFile;
};

#endif

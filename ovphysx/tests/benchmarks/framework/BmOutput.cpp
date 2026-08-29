// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "UsdPCH.h"


#include "BmOutput.h"

#include "BmBenchmark.h"
#include "BmGlobals.h"
#include "BmUtils.h"

#include <carb/filesystem/IFileSystem.h>

#include <stdlib.h>
#include <string.h>

using namespace carb;

#if CARB_PLATFORM_WINDOWS
#    pragma warning(push)
#    pragma warning(disable : 4996) // unsafe string functions
#define SPRINTF sprintf_s
#else
#define SPRINTF snprintf
#endif

size_t strlcpy(char* dst, size_t dstSize, const char* src)
{
    size_t i = 0;
    if (dst && dstSize)
    {
        for (; i + 1 < dstSize && src[i]; i++) // copy up to dstSize-1 bytes
            dst[i] = src[i];
        dst[i] = 0; // always null-terminate
    }

    while (src[i]) // read any remaining characters in the src string to get the length
        i++;

    return i;
}


BmOutput::BmOutput(bool regenerate, uint32_t slop, bool console, const char* reportFile, PrintfCbFunc printfCb)
    : mRegenerate(regenerate), mSlop(slop), mConsole(console), mPrintfCb(printfCb), mReportFile(nullptr)
{
    filesystem::IFileSystem* fs = BmGlobals::getInstance().getFileSystem();

    if (reportFile)
    {
        filesystem::File* file = fs->openFileToWrite(reportFile);
        if (file)
        {
            mReportFile = file;
        }

        if (!mReportFile)
        {
            CARB_LOG_ERROR("Could not open requested report file! (%s)", reportFile);
        }
    }

    if (!regenerate)
    {
        const char* appDir = fs->getAppDirectoryPath();

        char filepath[512];
        char fileline[1024];

        SPRINTF(filepath, 512, "%s/../data/benchmarkData/_baseline.txt", appDir);

        filesystem::File* file = fs->openFileToRead(filepath);
        if (file)
        {
            Record* r = new Record;
            //read two header lines
            fs->readFileLine(file, fileline, 1024);
            fs->readFileLine(file, fileline, 1024);

            while (fs->readFileLine(file, fileline, 1024))
            {
                sscanf(fileline, "%s %d %d %d %d %d %d", r->name,
                    &r->avgTime, &r->timePositiveTolerance, &r->timeNegativeTolerance,
                    &r->maxMem, &r->memPositiveTolerance, &r->memNegativeTolerance);
                mBaselineRecords.push_back(r);
                r = new Record;
            }
            delete r;
            fs->closeFile(file);
        }
        else
        {
            CARB_LOG_WARN("Could not open baseline file for read\n");
        }
    }
}

BmOutput::~BmOutput()
{
    filesystem::IFileSystem* fs = BmGlobals::getInstance().getFileSystem();

    if (mRegenerate)
    {
        const char* appDir = fs->getAppDirectoryPath();

        char filepath[512];
        char fileline[1024];
        SPRINTF(filepath, 512, "%s/../data/benchmarkData", appDir);
        fs->makeDirectories(filepath);
        SPRINTF(filepath, 512, "%s/../data/benchmarkData/_baseline.txt", appDir);

        filesystem::File* file = fs->openFileToWrite(filepath);

        if (file)
        {
            // print header lines
            SPRINTF(fileline, 1024, "%-65s %12s %6s %6s %3s %12s %6s %6s\n",
                "benchmark name",
                "time avg[us]", "+tol%", "-tol%",
                "",
                "mem max", "+tol%", "-tol%");
            fs->writeFileLine(file, fileline);

            for (size_t i = 0; i < mBaseline.size(); i++)
            {
                const Record* r = mBaseline[i];
                SPRINTF(fileline, 1024, "%-65s %12d %6d %6d %3s %12d %6d %6d",
                    r->name,
                    r->avgTime, r->timePositiveTolerance, r->timeNegativeTolerance,
                    "",
                    r->maxMem, r->memPositiveTolerance, r->memNegativeTolerance);
                fs->writeFileLine(file, fileline);
                delete r;
            }
            fs->closeFile(file);
        }
        else
        {
            CARB_LOG_ERROR("Could not open baseline file for write");
        }

        for (size_t i = 0; i < mBaselineRecords.size(); i++)
            delete mBaselineRecords[i];
    }
    if (mReportFile)
    {
        fs->closeFile(mReportFile);
        mReportFile = nullptr;
    }
}

void BmOutput::printHeaders() const
{
    const char* h1 = "Name                                      Avg (us)           Memory";
    const char* h2 = "----                                      --------           ------";

    if (mConsole)
    {
        printFormatted("%s\n", h1);
        printFormatted("%s\n", h2);
    }

    if (mReportFile)
    {
        filesystem::IFileSystem* fs = BmGlobals::getInstance().getFileSystem();
        fs->writeFileLine(mReportFile, h1);
        fs->writeFileLine(mReportFile, h2);
    }
}

float BmOutput::performanceDelta(uint32_t current, uint32_t ref)
{
    // if current > ref it means current is slower than ref.
    // if current / ref = 2 it means current is 2x slower than ref, we want to print -100%
    // if current / ref = 3 it means current is 3x slower than ref, we want to print -200%
    // otherwise if current < ref it means current is faster than ref.
    // if ref / current = 2 it means current is 2x faster than ref, we want to print +100%
    // if ref / current = 3 it means current is 3x faster than ref, we want to print +200%
    return BmMin(ref, current) == 0 ? 0 : 100.0f * float(int32_t(ref) - int32_t(current)) / BmMin(ref, current);
}

bool BmOutput::emit(const char* name, const BmResult& result)
{
    uint32_t avgTime = uint32_t(result.avgTime);
    uint32_t stdDevTime = uint32_t(result.stdDevTime);
    uint32_t maxMem = uint32_t(result.maxMemory);

    bool bRet = true;
    char buffer[1024];
    if (mRegenerate)
    {
        Record* r = new Record;
        strlcpy(r->name, 256, name);
        r->avgTime = avgTime;
        r->stdDevTime = stdDevTime;
        r->maxMem = maxMem;
        r->memPositiveTolerance = mSlop;
        r->memNegativeTolerance = mSlop;
        r->timePositiveTolerance = mSlop;
        r->timeNegativeTolerance = mSlop;

        mBaseline.push_back(r);
        snprintf(buffer, sizeof(buffer), "%-40s %9d (+/-%d) %9d\n", name, avgTime, stdDevTime, maxMem);
        if (mConsole)
        {
            printFormatted(buffer);
            if (mPrintfCb)
            {
                (mPrintfCb)(buffer);
            }
        }
        if (mReportFile)
        {
            filesystem::IFileSystem* fs = BmGlobals::getInstance().getFileSystem();
            fs->writeFileLine(mReportFile, buffer);
        }
    }
    else
    {
        size_t index = 0xFFFFFFFF;
        for (size_t i = 0; i < mBaselineRecords.size(); i++)
        {
            if (strstr(mBaselineRecords[i]->name,name))
            {
                index = i;
                break;
            }
        }
        float dat = 0, dmm = 0;
        uint32_t rat = 0, rmm = 0;

        bool skipped = false;
        bool fail = false;

        if(result.avgTime == 0)
        {
            skipped = true;
        }
        else if (index != 0xFFFFFFFF)
        {
            int32_t timeMinPerformance = 0, memMinPerformance = 0;
            int32_t timeMaxPerformance = 0, memMaxPerformance = 0;

            const Record& r = *mBaselineRecords[index];
            rat = r.avgTime;
            rmm = r.maxMem;

            timeMinPerformance = -int32_t(r.timeNegativeTolerance);
            timeMaxPerformance = int32_t(r.timePositiveTolerance);
            memMinPerformance = -int32_t(r.memNegativeTolerance);
            memMaxPerformance = int32_t(r.memPositiveTolerance);

            dat = performanceDelta(avgTime, rat);
            dmm = performanceDelta(maxMem, rmm);
            // dat and dmm are -100% for a doubling of runtime or memory and +100% for a halfing of runtime or memory.
            fail = dat <= timeMinPerformance || dat >= timeMaxPerformance || dmm <= memMinPerformance ||
                        dmm >= memMaxPerformance;
        }
        
        snprintf(buffer, sizeof(buffer), "%-40s %9d (%+5.1f%%) %9d (%+5.1f%%)  %s", name, avgTime, dat, maxMem, dmm,
            skipped ? "Skipped" : index == 0xFFFFFFFF ? "No baseline" : fail ? "FAIL" : "");

        if (mConsole)
        {
            printString(buffer);
            if (mPrintfCb)
            {
                (mPrintfCb)(buffer);
                (mPrintfCb)("\n");
            }
        }
        if (mReportFile)
        {
            filesystem::IFileSystem* fs = BmGlobals::getInstance().getFileSystem();
            fs->writeFileLine(mReportFile, buffer);
        }
        bRet = !fail;
    }
    return bRet;
}


void BmOutput::dump(const char* name, const BmRecord& record) const
{
    char path[260];

    carb::filesystem::IFileSystem* fs = BmGlobals::getInstance().getFileSystem();

    const char* execDir = fs->getExecutableDirectoryPath();
    strlcpy(path, sizeof(path), execDir);
    snprintf(path + strlen(path), sizeof(path) - strlen(path), "/%s.csv", name);

    carb::filesystem::File* file = fs->openFileToWrite(path);
    if (file)
    {
        fs->writeFileLine(file, "Time (us)");

        char chLine[512];
        for (uint32_t i = 0; i < record.time.size(); i++)
        {
            snprintf(chLine, sizeof(chLine), "%d", (uint32_t)record.time[i]);
            fs->writeFileLine(file, chLine);
        }
        fs->closeFile(file);
    }
    else
        printFormatted("Could not open benchmark record file for write");
}


#if CARB_PLATFORM_WINDOWS
#    pragma warning(pop)
#endif

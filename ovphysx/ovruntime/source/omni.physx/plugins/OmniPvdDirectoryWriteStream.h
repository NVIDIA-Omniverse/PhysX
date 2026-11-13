// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-OMNIPVD-TRANSPORT-001
// @covers AC-3 AC-4

#pragma once

#include "OmniPvdWriteStream.h"

#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>

namespace omni
{
namespace physx
{

// The runtime owns this stream; OmniPvdWriter only borrows it. Each open reserves
// a new directory filename atomically, or replaces an explicit exact filename.
// Lifecycle and writes are serialized by recording admission.
class OmniPvdDirectoryWriteStream final : public OmniPvdWriteStream
{
public:
    OmniPvdDirectoryWriteStream(const std::filesystem::path& directory, const std::string& timestampPrefix)
        : mDirectory(directory), mTimestampPrefix(timestampPrefix)
    {
    }

    explicit OmniPvdDirectoryWriteStream(const std::filesystem::path& exactFile)
        : mFilePath(exactFile), mExactFile(true)
    {
    }

    OmniPvdDirectoryWriteStream(const OmniPvdDirectoryWriteStream&) = delete;
    OmniPvdDirectoryWriteStream& operator=(const OmniPvdDirectoryWriteStream&) = delete;

    ~OmniPvdDirectoryWriteStream() override
    {
        closeStream();
    }

    uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* bytes, uint64_t count) override
    {
        return mFile ? std::fwrite(bytes, 1, static_cast<size_t>(count), mFile) : 0;
    }

    bool OMNI_PVD_CALL flush() override
    {
        return mFile && std::fflush(mFile) == 0;
    }

    static bool isNameCollision(int error, const std::filesystem::path& candidate)
    {
        if (error == EEXIST)
            return true;
#if defined(_WIN32)
        // The Windows CRT reports EACCES for an existing directory too.
        std::error_code existsError;
        return error == EACCES && std::filesystem::exists(candidate, existsError) && !existsError;
#else
        (void)candidate;
        return false;
#endif
    }

    bool OMNI_PVD_CALL openStream() override
    {
        if (mFile)
            return true;

        mOwnsReservedFile = false;
        if (mExactFile)
            return openFile(false);

        for (uint64_t counter = 1; counter <= std::numeric_limits<uint32_t>::max(); ++counter)
        {
            char suffix[32];
            std::snprintf(suffix, sizeof(suffix), "_%02u_rec.ovd", static_cast<uint32_t>(counter));
            mFilePath = mDirectory / (mTimestampPrefix + suffix);
            if (openFile(true))
            {
                mOwnsReservedFile = true;
                return true;
            }
            if (isNameCollision(mOpenError, mFilePath))
                continue;
            return false;
        }
        mOpenError = EOVERFLOW;
        return false;
    }

    bool OMNI_PVD_CALL closeStream() override
    {
        if (!mFile)
            return true;
        FILE* file = mFile;
        mFile = nullptr;
        return std::fclose(file) == 0;
    }

    const std::filesystem::path& getFilePath() const
    {
        return mFilePath;
    }

    int getOpenError() const
    {
        return mOpenError;
    }

    // Close before removing only the directory filename this stream reserved.
    // Exact files can replace caller data and must survive failed starts.
    bool discardIncompleteFile(std::error_code& removeError)
    {
        const bool closed = closeStream();
        removeError.clear();
        if (!mOwnsReservedFile)
            return closed;
        mOwnsReservedFile = false;
        std::filesystem::remove(mFilePath, removeError);
        return closed && !removeError;
    }

private:
    bool openFile(bool exclusive)
    {
#if defined(_WIN32)
        mOpenError = _wfopen_s(&mFile, mFilePath.c_str(), exclusive ? L"wbx" : L"wb");
#else
        errno = 0;
        mFile = std::fopen(mFilePath.c_str(), exclusive ? "wbx" : "wb");
        mOpenError = mFile ? 0 : errno;
#endif
        return mFile != nullptr;
    }

    std::filesystem::path mDirectory;
    std::string mTimestampPrefix;
    std::filesystem::path mFilePath;
    FILE* mFile{ nullptr };
    int mOpenError{ 0 };
    bool mExactFile{ false };
    bool mOwnsReservedFile{ false };
};

} // namespace physx
} // namespace omni

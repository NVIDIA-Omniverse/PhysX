// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../tools/OvdAnnotationFile.h"
#include "../tools/OvdCheckedOutput.h"
#include <chrono>
#include <iostream>

class FailedBuffer : public std::streambuf
{
protected:
    std::streamsize xsputn(const char*, std::streamsize) override { return 0; }
    int overflow(int) override { return traits_type::eof(); }
};

int main(int argc, char** argv)
{
    if (argc != 2) return 1;
    FailedBuffer failedBuffer;
    std::ostream failedOutput(&failedBuffer);
    if (writeOvdJson(failedOutput, "{}")) { std::cerr << "Failed stdout was accepted\n"; return 1; }
    const std::string base = std::string(argv[1]) + "/annotations-" +
        std::to_string((std::chrono::steady_clock::now() - std::chrono::steady_clock::time_point()).count());
    const std::string text = "comparison evidence";
    const auto write = [](FILE* file, const std::string& data)
    { return fwrite(data.data(), 1, data.size(), file) == data.size(); };
    const auto close = [](FILE* file) { return fclose(file) == 0; };
    const auto failsWith = [&](const std::string& path, const std::string& expected, auto failingWrite, auto failingClose)
    {
        try { saveNewOvdAnnotations(path, text, failingWrite, failingClose); }
        catch (const std::runtime_error& error)
        {
            FILE* remains = fopen(path.c_str(), "rb");
            if (remains) { fclose(remains); std::cerr << "Partial annotation remains\n"; return false; }
            if (std::string(error.what()).find(expected) == std::string::npos)
            { std::cerr << "Wrong saved error: " << error.what() << '\n'; return false; }
            return true;
        }
        std::cerr << "Injected annotation failure was accepted\n";
        return false;
    };

    const std::string shortPath = base + "-short.json";
    const auto shortWrite = [](FILE* file, const std::string&)
    { fwrite("partial", 1, 7, file); errno = ENOSPC; return false; };
    if (!failsWith(shortPath, std::strerror(ENOSPC), shortWrite, close)) return 1;
    saveNewOvdAnnotations(shortPath, text, write, close);
    FILE* retry = fopen(shortPath.c_str(), "rb");
    if (!retry) return 1;
    char saved[32] = {};
    const size_t size = fread(saved, 1, sizeof(saved), retry);
    fclose(retry);
    if (std::string(saved, size) != text) return 1;
    std::remove(shortPath.c_str());

    const std::string closePath = base + "-close.json";
    const auto failedClose = [](FILE* file) { fclose(file); errno = EIO; return false; };
    if (!failsWith(closePath, std::strerror(EIO), write, failedClose)) return 1;
    const std::string removePath = base + "-remove.json";
    const auto failedRemove = [](const char*) { errno = EACCES; return -1; };
    try { saveNewOvdAnnotations(removePath, text, shortWrite, close, failedRemove); }
    catch (const std::runtime_error& error)
    {
        const int failureErrno = errno;
        const std::string diagnostic = error.what();
        if (failureErrno != ENOSPC)
        { std::cerr << "Cleanup replaced the original write errno\n"; return 1; }
        if (diagnostic.find(std::strerror(ENOSPC)) == std::string::npos ||
            diagnostic.find(std::strerror(EACCES)) == std::string::npos ||
            diagnostic.find("remove") == std::string::npos)
        { std::cerr << "Missing original or cleanup error: " << diagnostic << '\n'; return 1; }
        FILE* partial = fopen(removePath.c_str(), "rb");
        if (!partial) { std::cerr << "Injected failed removal did not leave the owned partial file\n"; return 1; }
        fclose(partial);
        std::remove(removePath.c_str());
        return 0;
    }
    std::cerr << "Failed partial removal was accepted\n";
    return 1;
}

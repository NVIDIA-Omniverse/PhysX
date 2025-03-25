// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

#include "NvFlowProcess.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowString.h"
#include "NvFlowTypes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#    include <Windows.h>
#else
#    include <sys/stat.h>
#    include <sys/wait.h>

#    include <unistd.h>
#endif

struct NvFlowProcess
{
#if defined(_WIN32)
    HANDLE childStdinRead;
    HANDLE childStdinWrite;
    HANDLE childStdoutRead;
    HANDLE childStdoutWrite;

    PROCESS_INFORMATION procInfo;
    DWORD returnValue;
#else
    int pipefd[2];
    int childid;
#endif
    NvFlowArray<char> cmdline;
    NvFlowArray<char> stdoutData;

    int state;
};

void NvFlowProcessResizeStdoutData(NvFlowProcess* ptr, NvFlowUint64 minCap)
{
    NvFlowUint64 capacity = 4096u;
    while (capacity < minCap)
    {
        capacity *= 2u;
    }
    ptr->stdoutData.reserve(capacity);
}

void NvFlowProcessPushStdoutData(NvFlowProcess* ptr, char* buf, NvFlowUint64 count)
{
    NvFlowProcessResizeStdoutData(ptr, ptr->stdoutData.size + count + 1);
    memcpy(&ptr->stdoutData[ptr->stdoutData.size], buf, count);
    ptr->stdoutData.size += count;
    ptr->stdoutData[ptr->stdoutData.size] = '\0';
}

NvFlowProcess* NvFlowProcessCreate()
{
    NvFlowProcess* ptr = new NvFlowProcess();

    return ptr;
}

void NvFlowProcessDestroy(NvFlowProcess* ptr)
{
    delete ptr;
}

int NvFlowProcessLaunch(NvFlowProcess* ptr, const char** argv)
{
    // free existing cmdline
    ptr->cmdline.size = 0u;

    NvFlowUint64 cmdline_length = 0;
    for (int argIdx = 0; argv[argIdx]; argIdx++)
    {
        cmdline_length += strlen(argv[argIdx]) + 1 + 2; // one space, two quotes
    }

    ptr->cmdline.reserve(cmdline_length);
    ptr->cmdline.size = cmdline_length - 1u; // do not count null

    NvFlowUint64 cmdline_offset = 0;
    for (int argIdx = 0; argv[argIdx]; argIdx++)
    {
        NvFlowUint64 arg_length = strlen(argv[argIdx]) + 1 + 2; // one space, two quotes

        ptr->cmdline[cmdline_offset] = '\"';

        memcpy(ptr->cmdline.data + cmdline_offset + 1, argv[argIdx], arg_length - 3);

        ptr->cmdline[cmdline_offset + arg_length - 2] = '\"';
        ptr->cmdline[cmdline_offset + arg_length - 1] = ' ';

        cmdline_offset += arg_length;
    }
    ptr->cmdline[cmdline_length - 1u] = '\0';

    // reset text
    ptr->stdoutData.size = 0;

    ptr->state = 0;

#if defined(_WIN32)
    SECURITY_ATTRIBUTES saAttr = {};
    saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
    saAttr.bInheritHandle = TRUE;
    saAttr.lpSecurityDescriptor = NULL;

    CreatePipe(&ptr->childStdoutRead, &ptr->childStdoutWrite, &saAttr, 0);
    SetHandleInformation(ptr->childStdoutRead, HANDLE_FLAG_INHERIT, 0);
    CreatePipe(&ptr->childStdinRead, &ptr->childStdinWrite, &saAttr, 0);
    SetHandleInformation(ptr->childStdinWrite, HANDLE_FLAG_INHERIT, 0);

    PROCESS_INFORMATION procInfo = {};
    STARTUPINFOA startInfo = {};
    startInfo.cb = sizeof(STARTUPINFOA);
    startInfo.hStdError = ptr->childStdoutWrite;
    startInfo.hStdOutput = ptr->childStdoutWrite;
    startInfo.hStdInput = ptr->childStdinRead;
    startInfo.dwFlags |= STARTF_USESTDHANDLES;

    BOOL success = CreateProcessA(NULL, ptr->cmdline.data, NULL, NULL, TRUE,
                                  CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW, NULL, NULL, &startInfo, &procInfo);

    if (!success)
    {
        CloseHandle(ptr->childStdinRead);
        CloseHandle(ptr->childStdoutWrite);

        CloseHandle(ptr->childStdoutRead);
        CloseHandle(ptr->childStdinWrite);

        return 1;
    }

    ptr->procInfo = procInfo;

    CloseHandle(ptr->childStdinRead);
    CloseHandle(ptr->childStdoutWrite);
#else

    int err = pipe(ptr->pipefd);
    if (err != 0)
    {
        return 1;
    }

    ptr->childid = fork();

    if (ptr->childid == 0)
    {
        dup2(ptr->pipefd[1], STDOUT_FILENO);
        close(ptr->pipefd[1]);

        int execv_ret = execvp(argv[0], (char* const*)argv);

        exit(0);
    }

    close(ptr->pipefd[1]);
#endif
    ptr->state = 1;
    return 0;
}

int NvFlowProcessIsComplete(NvFlowProcess* ptr)
{
#if defined(_WIN32)
    // collect output
    if (ptr->state == 1)
    {
        CHAR buf[4096u];
        DWORD bytesRead = 0;

        // peek in order to offer early out
        DWORD bytesAvailable = 0u;
        BOOL peekSuccess = PeekNamedPipe(ptr->childStdoutRead, nullptr, 0u, nullptr, &bytesAvailable, nullptr);
        if (peekSuccess && bytesAvailable == 0u)
        {
            return 0;
        }

        while (1)
        {
            BOOL success = ReadFile(ptr->childStdoutRead, buf, 4096u, &bytesRead, NULL);
            if (!success || bytesRead == 0)
            {
                break;
            }

            NvFlowProcessPushStdoutData(ptr, buf, bytesRead);
        }
        ptr->state = 2;
    }
#else
    // collect output
    if (ptr->state == 1)
    {
        char buf[4096u];
        ssize_t bytesRead = 0;
        while (1)
        {
            bytesRead = read(ptr->pipefd[0], buf, 4096u);
            if (bytesRead == 0)
            {
                break;
            }

            NvFlowProcessPushStdoutData(ptr, buf, bytesRead);
        }
        ptr->state = 2;
    }
#endif
    return 1;
}

NvFlowProcessReturn NvFlowProcessGetReturn(NvFlowProcess* ptr)
{
    NvFlowProcessReturn ret = {};

    ret.cmdlineData = ptr->cmdline.data;
    ret.cmdlineSize = (int)ptr->cmdline.size;

    if (ptr->state != 2)
    {
        ret.stdoutData = "Process launch failed.\n\n";
        ret.stdoutSize = sizeof("Process launch failed.\n\n");
        ret.returnLaunch = 1;
        ret.returnProcess = 1;
        return ret;
    }

#if defined(_WIN32)
    GetExitCodeProcess(ptr->procInfo.hProcess, &ptr->returnValue);

    CloseHandle(ptr->procInfo.hProcess);
    CloseHandle(ptr->procInfo.hThread);

    CloseHandle(ptr->childStdoutRead);
    CloseHandle(ptr->childStdinWrite);

    ret.returnLaunch = 0;
    ret.returnProcess = ptr->returnValue;
    ret.stdoutData = ptr->stdoutData.data;
    ret.stdoutSize = (int)ptr->stdoutData.size;
#else
    int retStatus = 0;

    waitpid(ptr->childid, &retStatus, 0);

    int returnValue = WIFEXITED(retStatus) ? WEXITSTATUS(retStatus) : 1;

    close(ptr->pipefd[0]);

    ret.returnLaunch = 0;
    ret.returnProcess = returnValue;
    ret.stdoutData = ptr->stdoutData.data;
    ret.stdoutSize = (int)ptr->stdoutData.size;
#endif
    ptr->state = 3;
    return ret;
}

void NvFlowProcessSendCtrlBreak(NvFlowProcess* ptr)
{
#if defined(_WIN32)
    GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, ptr->procInfo.dwProcessId);
#else
    // Linux
#endif
}

void NvFlowProcessOutputDebugMessage(const char* message)
{
#if defined(_WIN32)
    OutputDebugStringA(message);
    fprintf(stdout, "%s", message);
    fflush(stdout);
#else
    fprintf(stdout, "%s", message);
    fflush(stdout);
#endif
}

int NvFlowProcessMkdir(const char* name)
{
#if defined(_WIN32)
    BOOL success = CreateDirectoryA(name, NULL);
    if (!success)
    {
        return GetLastError();
    }
    return 0;
#else
    int ret = mkdir(name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    return ret;
#endif
}

int NvFlowProcessMkdirRecursive(const char* name)
{
    int ret = 0;

    NvFlowStringPool* pool = NvFlowStringPoolCreate();

    const char* strView = name;

    NvFlowArray<const char*, 16u> stringStack;

    strView = NvFlowStringTrimEnd(pool, strView, '/');

    char* a = {};
    char* b = {};
    NvFlowStringSplitDelimLast(pool, &a, &b, strView, '/');

    while (NvFlowStringLength(b) > 0)
    {
        stringStack.pushBack(b);

        a = NvFlowStringTrimEnd(pool, a, '/');

        NvFlowStringSplitDelimLast(pool, &a, &b, a, '/');
    }

    bool isRelativePath = strView && strView[0] != '/';

    // compose in reverse order, calling mkdir
    const char* str = NvFlowStringDup(pool, a);

    for (NvFlowUint64 idx = 0u; idx < stringStack.size; idx++)
    {
        NvFlowUint64 idxInv = stringStack.size - 1u - idx;

        const char* baseStr = str;
        const char* addStr = stringStack[idxInv];

        if (isRelativePath && NvFlowStringLength(baseStr) == 0)
        {
            str = NvFlowStringDup(pool, addStr);
        }
        else
        {
            str = NvFlowStringConcat3(pool, baseStr, "/", addStr);
        }

        if (NvFlowStringLength(addStr) > 0 && NvFlowCharIsAlphaNum(addStr[0]))
        {
            ret = NvFlowProcessMkdir(str);
#if 0
            NvFlowProcessOutputDebugMessage("NvFlowShaderTool: mkdir ");
            NvFlowProcessOutputDebugMessage(str.data);
            if (ret == 0)
            {
                NvFlowProcessOutputDebugMessage(" : success\n");
            }
            else
            {
                NvFlowProcessOutputDebugMessage(" : fail\n");
            }
#endif
        }
    }

    NvFlowStringPoolDestroy(pool);

    return ret;
}

void NvFlowProcessSleep(int milliseconds)
{
#if defined(_WIN32)
    Sleep(milliseconds);
#else
    usleep(1000 * milliseconds);
#endif
}

/// *************************** NvFlowProcessList ***************************

struct NvFlowProcessItem
{
    NvFlowArray<char*, 16u> argv;
};

struct NvFlowProcessList
{
    NvFlowArray<NvFlowProcessItem, 16u> items;

    static const int numProcesses = 16u;
    NvFlowProcess* process[numProcesses];
    NvFlowUint64 processState[numProcesses];

    NvFlowStringPool* stringPool;
};

NvFlowProcessList* NvFlowProcessListCreate()
{
    auto ptr = new NvFlowProcessList();

    for (int idx = 0; idx < ptr->numProcesses; idx++)
    {
        ptr->process[idx] = NvFlowProcessCreate();
    }
    ptr->stringPool = NvFlowStringPoolCreate();

    return ptr;
}

void NvFlowProcessListDestroy(NvFlowProcessList* ptr)
{
    for (int idx = 0; idx < ptr->numProcesses; idx++)
    {
        NvFlowProcessDestroy(ptr->process[idx]);
    }
    NvFlowStringPoolDestroy(ptr->stringPool);

    delete ptr;
}

void NvFlowProcessListAdd(NvFlowProcessList* ptr, int argc, const char** args)
{
    auto item = &ptr->items[ptr->items.allocateBack()];

    item->argv.reserve(argc + 1u);
    item->argv.size = argc + 1u;

    for (int idx = 0u; idx < argc; idx++)
    {
        item->argv[idx] = NvFlowStringDup(ptr->stringPool, args[idx]);
    }
    item->argv[argc] = nullptr;
}

void NvFlowProcessListAdd(NvFlowProcessList* ptr, const char** argv)
{
    int argc = 0;
    while (argv[argc])
    {
        argc++;
    }

    NvFlowProcessListAdd(ptr, argc, argv);
}

void NvFlowProcessListFlush(NvFlowProcessList* ptr)
{
    // execute state machine
    NvFlowUint64 scheduleItemIdx = 0u;

    for (NvFlowUint64 processIdx = 0u; processIdx < ptr->numProcesses; processIdx++)
    {
        ptr->processState[processIdx] = 0;
    }

    bool anyNotState2 = true;
    while (anyNotState2)
    {
        for (NvFlowUint64 processIdx = 0u; processIdx < ptr->numProcesses; processIdx++)
        {
            if (ptr->processState[processIdx] == 0)
            {
                NvFlowUint64 itemIdx = scheduleItemIdx;
                scheduleItemIdx++;

                if (itemIdx < ptr->items.size)
                {
                    auto item = &ptr->items[itemIdx];

                    int launch = NvFlowProcessLaunch(ptr->process[processIdx], (const char**)item->argv.data);

                    ptr->processState[processIdx] = 1;
                }
                else
                {
                    ptr->processState[processIdx] = 2;
                }
            }
            else if (ptr->processState[processIdx] == 1)
            {
                if (NvFlowProcessIsComplete(ptr->process[processIdx]))
                {
                    NvFlowProcessReturn processRet = NvFlowProcessGetReturn(ptr->process[processIdx]);

                    const char* cmdlineData =
                        NvFlowStringFromView(ptr->stringPool, processRet.cmdlineData, processRet.cmdlineSize);
                    const char* stdoutData =
                        NvFlowStringFromView(ptr->stringPool, processRet.stdoutData, processRet.stdoutSize);

                    const char* debugs[4] = { "NvFlowProcessList: ", processRet.cmdlineData ? cmdlineData : "", "\n\t ",
                                              processRet.stdoutData ? stdoutData : "\n" };
                    char* debugString = NvFlowStringConcatN(ptr->stringPool, debugs, 4);
                    NvFlowProcessOutputDebugMessage(debugString);

                    ptr->processState[processIdx] = 0;
                }
            }
        }

        anyNotState2 = false;
        for (NvFlowUint64 processIdx = 0u; processIdx < ptr->numProcesses; processIdx++)
        {
            if (ptr->processState[processIdx] != 2)
            {
                anyNotState2 = true;
            }
        }
    }
}

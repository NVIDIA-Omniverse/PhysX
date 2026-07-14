// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// Coverage helper: exports a function that flushes gcov .gcda data for this
// shared library.  Compiled only when OVRUNTIME_ENABLE_COVERAGE is ON.
//
// Background: GCC's gcov writes .gcda files from a __attribute__((destructor))
// that runs at process exit, AFTER all atexit() handlers.  Carbonite's atexit
// handlers call into Python, which triggers a Fatal Python error (GIL released)
// and an abort/segfault that kills the process before gcov's destructor runs.
//
// Fix: the coverage conftest.py calls this function on each loaded ovruntime
// .so BEFORE exit(), ensuring coverage data is written while the process is
// still healthy.

extern "C" void __gcov_dump(void);

// __attribute__((used)) prevents --gc-sections from stripping this function
// even though nothing within the .so references it.
extern "C" __attribute__((visibility("default"), used))
void ovruntime_gcov_flush(void)
{
    __gcov_dump();
}

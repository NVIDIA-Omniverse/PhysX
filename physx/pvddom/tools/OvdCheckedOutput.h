// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <ostream>
#include <string>

inline bool writeOvdJson(std::ostream& output, const std::string& json)
{
    output << json << '\n';
    output.flush();
    return bool(output);
}

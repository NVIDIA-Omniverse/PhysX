// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <string>

namespace omni {
namespace sdk {
namespace usd_schema_paths {

static constexpr const char* kNamespacedUsdPluginPathEnvVar = "OV_PXR_PLUGINPATH_2511";

std::string getLibraryDirectory();
std::string getPluginsDirectory();
std::string getUsdPluginsDirectory();

bool registerSchemaPaths(std::string* out_error = nullptr);
bool registerSchemaPathsOnce(std::string* out_error = nullptr, bool* out_registered = nullptr);

void resetSchemaPathRegistrationForTests();

} // namespace usd_schema_paths
} // namespace sdk
} // namespace omni

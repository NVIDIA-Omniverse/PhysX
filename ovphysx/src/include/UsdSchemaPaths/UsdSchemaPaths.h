// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

// Locate Carbonite plugins relative to the ovphysx library or OVPHYSX_LIB.
// These helpers do not load or configure USD.

namespace omni {
namespace sdk {
namespace usd_schema_paths {

// Directory containing the ovphysx shared library. OVPHYSX_LIB (a library
// path or a directory) wins over the loaded module location. Empty on failure.
std::string getLibraryDirectory();

// ovphysx Carbonite plugin directory: <libdir>/../plugins for the SDK and
// wheel layouts, or <libdir>/plugins for a copied runtime layout.
std::string getPluginsDirectory();

#ifdef _WIN32
// Directory of an already-loaded module, looked up by file name. Empty when the
// module is not loaded.
std::string getLoadedLibraryDirectory(const std::string& libraryName);
#endif

} // namespace usd_schema_paths
} // namespace sdk
} // namespace omni

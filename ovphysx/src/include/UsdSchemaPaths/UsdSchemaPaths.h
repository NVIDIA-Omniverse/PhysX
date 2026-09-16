// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2
 */
#pragma once

#include <string>

// Filesystem discovery for the ovphysx runtime layout. ovphysx does not load,
// link, or configure OpenUSD: these helpers only compute directories from the
// location of the ovphysx shared library (or the OVPHYSX_LIB override) so the
// loader can find its Carbonite plugins and the application can find the
// codeless PhysX USD schemas that ovphysx ships as data.

namespace omni {
namespace sdk {
namespace usd_schema_paths {

// Directory containing the ovphysx shared library. OVPHYSX_LIB (a library
// path or a directory) wins over the loaded module location. Empty on failure.
std::string getLibraryDirectory();

// ovphysx Carbonite plugin directory: <libdir>/../plugins for the SDK and
// wheel layouts, or <libdir>/plugins for a copied runtime layout.
std::string getPluginsDirectory();

// Root of the shipped codeless PhysX USD schemas (the directory holding the
// root plugInfo.json and one <Module>/resources/ directory per schema module):
// <libdir>/../schemas/physx for the SDK and wheel layouts, or
// <libdir>/schemas/physx for a copied runtime layout. Returns an empty string
// and fills *out_error when no such root with a plugInfo.json exists.
std::string getCodelessSchemaRoot(std::string* out_error = nullptr);

#ifdef _WIN32
// Directory of an already-loaded module, looked up by file name. Empty when the
// module is not loaded.
std::string getLoadedLibraryDirectory(const std::string& libraryName);
#endif

} // namespace usd_schema_paths
} // namespace sdk
} // namespace omni

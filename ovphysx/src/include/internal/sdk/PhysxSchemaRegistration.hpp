// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-3 AC-8
 */

#pragma once

#include "internal/sdk/PlatformIncludes.hpp"
#include <ovstage/ovstage_population.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <sddl.h>
#endif

namespace ovphysx
{
namespace schema_registration
{

constexpr const char* kRequireRegistration = "/ovphysx/schemas/requireRegistration";
constexpr const char* kMissingSchemasMessage =
    "attach_ovstage: the PhysX USD schema families were not both registered with ovstage before USD schema "
    "definitions were built in this process (by population or export). Missing PhysX schemas can drop "
    "authored Physx* APIs and attributes, "
    "including self-collision and joint-limit settings. Restart the process and register both "
    "PhysxSchema/resources and OmniUsdPhysicsDeformableSchema/resources from the separately installed "
    "physx-usd-schemas provider before the first USD schema read (population or export): "
    "use ovstage_population_register_usd_schemas() "
    "(Python: ovstage.population.register_usd_schemas()). See docs/physics_schemas.md.";

inline void* resolveOvstageSymbol(const char* symbol)
{
#ifdef _WIN32
    static const char* const candidates[] = { "ovstage.dll", "libovstage.dll" };
    for (const char* candidate : candidates)
    {
        HMODULE module = GetModuleHandleA(candidate);
        if (module)
        {
            FARPROC proc = GetProcAddress(module, symbol);
            if (proc)
                return reinterpret_cast<void*>(proc);
        }
    }
    return nullptr;
#else
    static const char* const candidates[] = { "libovstage.so", "libovstage.so.1", "libovstage.so.0" };
    for (const char* candidate : candidates)
    {
        void* module = dlopen(candidate, RTLD_NOLOAD | RTLD_NOW);
        if (module)
        {
            void* proc = dlsym(module, symbol);
            // RTLD_NOLOAD acquires a reference; the application's owner keeps the library alive.
            dlclose(module);
            if (proc)
                return proc;
        }
    }
    return dlsym(RTLD_DEFAULT, symbol);
#endif
}

#ifdef _WIN32
inline std::unique_ptr<void, decltype(&::LocalFree)> privateDirectorySecurity()
{
    HANDLE token = nullptr;
    if (!::OpenThreadToken(::GetCurrentThread(), TOKEN_QUERY, TRUE, &token))
    {
        const DWORD error = ::GetLastError();
        if (error != ERROR_NO_TOKEN)
            throw std::system_error(static_cast<int>(error), std::system_category(), "cannot query thread identity");
        if (!::OpenProcessToken(::GetCurrentProcess(), TOKEN_QUERY, &token))
            throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(),
                                    "cannot query process identity");
    }
    const std::unique_ptr<void, decltype(&::CloseHandle)> ownedToken(token, &::CloseHandle);
    DWORD bytes = 0;
    ::GetTokenInformation(token, TokenUser, nullptr, 0, &bytes);
    if (::GetLastError() != ERROR_INSUFFICIENT_BUFFER || bytes == 0)
        throw std::runtime_error("cannot size the effective user identity");
    std::vector<unsigned char> userBuffer(bytes);
    if (!::GetTokenInformation(token, TokenUser, userBuffer.data(), bytes, &bytes))
        throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(),
                                "cannot read the effective user identity");
    const TOKEN_USER* user = reinterpret_cast<const TOKEN_USER*>(userBuffer.data());
    LPWSTR sid = nullptr;
    if (!::ConvertSidToStringSidW(user->User.Sid, &sid))
        throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(),
                                "cannot format the effective user identity");
    const std::unique_ptr<void, decltype(&::LocalFree)> ownedSid(sid, &::LocalFree);
    // Protect the DACL from the temp parent's permissions. Only the effective user and
    // SYSTEM can access this directory; files and subdirectories inherit the same grants.
    const std::wstring sddl = L"D:P(A;OICI;FA;;;" + std::wstring(sid) + L")(A;OICI;FA;;;SY)";
    PSECURITY_DESCRIPTOR descriptor = nullptr;
    if (!::ConvertStringSecurityDescriptorToSecurityDescriptorW(sddl.c_str(), SDDL_REVISION_1,
                                                               &descriptor, nullptr))
        throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(),
                                "cannot create private directory permissions");
    return { descriptor, &::LocalFree };
}
#endif

// These descriptors declare no schema types or resources. They probe plugin identities only;
// the family descriptors must never be registered while USD can still build its definitions.
class ProbeDescriptors
{
public:
    explicit ProbeDescriptors(uint64_t sequence)
    {
        std::error_code temporaryError;
        // Files are created only inside a fresh private child: mkdtemp mode 0700 on
        // Linux, or an explicit protected effective-user/SYSTEM DACL on Windows.
        // The application's configured temporary root and its ancestors are trusted.
        const std::filesystem::path temporaryRoot = std::filesystem::temp_directory_path(temporaryError);

#ifndef _WIN32
        // A missing, relative, or unwritable TMPDIR must not prevent schema verification.
        // Fall back to the system temp root, never the application's working directory.
        const std::filesystem::path roots[] = { temporaryRoot, "/tmp" };
        for (const std::filesystem::path& root : roots)
        {
            if (!root.is_absolute())
                continue;
            // Atomic creation with mode 0700; never adopt an existing directory.
            std::string pattern =
                (root / ("ovphysx-schema-probe-" + std::to_string(sequence) + "-XXXXXX")).string();
            if (::mkdtemp(pattern.data()) != nullptr)
            {
                m_root = std::filesystem::path(pattern);
                return;
            }
        }
        throw std::runtime_error("cannot create a private schema probe directory in the temporary roots");
#else
        if (temporaryError || !temporaryRoot.is_absolute())
            throw std::runtime_error("cannot find an absolute OS temporary directory");
        std::vector<wchar_t> volumeRoot(temporaryRoot.native().size() + 2);
        if (!::GetVolumePathNameW(temporaryRoot.c_str(), volumeRoot.data(), static_cast<DWORD>(volumeRoot.size())))
            throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(),
                                    "cannot locate the temporary directory volume");
        DWORD volumeFlags = 0;
        if (!::GetVolumeInformationW(volumeRoot.data(), nullptr, 0, nullptr, nullptr, &volumeFlags, nullptr, 0))
            throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(),
                                    "cannot query temporary directory volume permissions");
        if ((volumeFlags & FILE_PERSISTENT_ACLS) == 0)
            throw std::runtime_error("the temporary directory volume cannot enforce private permissions");
        const std::unique_ptr<void, decltype(&::LocalFree)> security = privateDirectorySecurity();
        SECURITY_ATTRIBUTES attributes{ sizeof(SECURITY_ATTRIBUTES), security.get(), FALSE };
        std::random_device random;
        for (unsigned attempt = 0; attempt < 64; ++attempt)
        {
            char token[17];
            std::snprintf(token, sizeof(token), "%08x%08x",
                          static_cast<unsigned>(random()), static_cast<unsigned>(random()));
            std::filesystem::path candidate = temporaryRoot /
                ("ovphysx-schema-probe-" + std::to_string(sequence) + "-" + token);
            if (::CreateDirectoryW(candidate.c_str(), &attributes))
            {
                m_root = std::move(candidate);
                break;
            }
            const DWORD error = ::GetLastError();
            if (error != ERROR_ALREADY_EXISTS)
                throw std::filesystem::filesystem_error("cannot create a schema probe directory", candidate,
                                                       std::error_code(static_cast<int>(error), std::system_category()));
        }
        if (m_root.empty())
            throw std::runtime_error("cannot create a unique schema probe directory");
#endif
    }

    ~ProbeDescriptors()
    {
        // USD has already read these typeless descriptors synchronously. Keep no resources
        // or generated schema files behind; cached plugin identities require neither.
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
    }

    ProbeDescriptors(const ProbeDescriptors&) = delete;
    ProbeDescriptors& operator=(const ProbeDescriptors&) = delete;

    std::string registryMarker() const
    {
        return "ovphysx_registry_probe_" + m_root.filename().string();
    }

    std::string write(const std::string& name) const
    {
        // All names are internal ASCII constants or the private directory's generated name.
        const std::filesystem::path directory = m_root / name;
        if (!std::filesystem::create_directory(directory))
            throw std::runtime_error("cannot create a schema probe descriptor directory");
        std::ofstream descriptor;
        descriptor.exceptions(std::ios::failbit | std::ios::badbit);
        descriptor.open(directory / "plugInfo.json", std::ios::out | std::ios::binary);
        descriptor << "{\"Plugins\":[{\"Name\":\"" << name
                   << "\",\"Type\":\"resource\",\"Root\":\".\",\"ResourcePath\":\".\",\"Info\":{}}]}";
        descriptor.close();
        return directory.u8string();
    }

private:
    std::filesystem::path m_root;
};

inline bool verifyPhysxSchemasRegistered(std::string& errorOut)
{
    struct State
    {
        std::mutex mutex;
        uint64_t nextProbe = 0;
        bool registered = false;
        bool missing = false;
        std::string missingDetail;
    };
    static State state;
    // A late probe inserts plugin identities. Serialize it with the sticky result so a
    // second attachment cannot mistake those newly inserted identities for timely registration.
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.registered)
        return true;
    if (state.missing)
    {
        errorOut = kMissingSchemasMessage;
        errorOut += state.missingDetail;
        return false;
    }

    using RegisterSchemas = ovstage_api_status_t (*)(const ovx_string_t*, size_t);
    using LastError = ovx_string_t (*)();
    const RegisterSchemas registerSchemas = reinterpret_cast<RegisterSchemas>(
        resolveOvstageSymbol("ovstage_population_register_usd_schemas"));
    const LastError lastError =
        reinterpret_cast<LastError>(resolveOvstageSymbol("ovstage_population_get_last_error"));
    if (!registerSchemas || !lastError)
        return true; // No population bridge to verify; native/procedural consumers remain supported.

    const auto detail = [lastError]() -> std::string {
        const ovx_string_t message = lastError();
        return message.ptr && message.length ? std::string(message.ptr, message.length) : std::string();
    };

    try
    {
        const ProbeDescriptors probes(++state.nextProbe);
        const std::string marker = probes.write(probes.registryMarker());
        const ovx_string_t markerPath{ marker.c_str(), marker.size() };
        const ovstage_api_status_t markerStatus = registerSchemas(&markerPath, 1);
        if (markerStatus == OVSTAGE_OK || markerStatus == OVSTAGE_ERROR_NOT_SUPPORTED)
            return true; // Registry still open (or bridge unavailable); do not select schema families.
        const std::string markerDetail = detail();
        if (markerStatus != OVSTAGE_ERROR_OP_FAILED || markerDetail.find("already built") == std::string::npos)
        {
            errorOut = "attach_ovstage: cannot verify whether USD schema definitions are frozen: " + markerDetail;
            return false;
        }

        // These are the provider's plugin names, not its module directory names. Distinct
        // directories prevent USD from coalescing both descriptors under one resource path.
        const std::string physx = probes.write("physxSchema");
        const std::string deformable = probes.write("omniUsdPhysicsDeformableSchema");
        const ovx_string_t familyPaths[] = {
            { physx.c_str(), physx.size() }, { deformable.c_str(), deformable.size() }
        };
        const ovstage_api_status_t familyStatus = registerSchemas(familyPaths, 2);
        if (familyStatus == OVSTAGE_OK)
        {
            // This observes plugin bookkeeping, not frozen schema definitions.
            // An application that ignored its own earlier late-registration
            // error has already inserted these identities and is undetectable.
            state.registered = true;
            return true;
        }
        const std::string familyDetail = detail();
        if (familyStatus == OVSTAGE_ERROR_OP_FAILED && familyDetail.find("already built") != std::string::npos)
        {
            state.missing = true;
            state.missingDetail = " ovstage: " + familyDetail;
            errorOut = kMissingSchemasMessage;
            errorOut += state.missingDetail;
            return false;
        }
        errorOut = "attach_ovstage: verifying PhysX USD schema registration failed: " + familyDetail;
        return false;
    }
    catch (const std::exception& error)
    {
        errorOut = "attach_ovstage: cannot verify PhysX USD schema registration: ";
        errorOut += error.what();
        errorOut += ". Check that the OS temporary directory is writable; see docs/physics_schemas.md.";
        return false;
    }
}

} // namespace schema_registration
} // namespace ovphysx

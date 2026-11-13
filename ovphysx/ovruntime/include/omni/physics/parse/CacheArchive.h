// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-ARCHIVE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace omni::physics::parse::cache
{
using Digest = std::array<uint8_t, 32>;
using Bytes = std::vector<uint8_t>;

// The deepest nesting of a stable native-instancing prototype path,
// `/__Prototype(<archived path>)/...`, counting the outermost as depth 0. The
// identity and the archive validation share it: a name one admits, the other accepts.
constexpr int kMaxPrototypeNesting = 16;
// An insertion unit names its own paths relative to its root, "." for the root
// and "./<rest>" below it, so that it restores under another root. No absolute
// prim path starts with '.', so a local name never equals an authored path. The
// identity and the archive share it.
constexpr std::string_view kUnitRoot = ".";

// SHA-256 over bytes, with no process-dependent seed or native object layout.
Digest digest(const void* data, size_t size);
std::string hex(const Digest& value);

// Fixed-width, little-endian wire encoding. Lengths are always uint64_t.
class Writer
{
public:
    void clear()
    {
        mData.clear();
    }
    void reserve(size_t size)
    {
        mData.reserve(size);
    }
    void u8(uint8_t value);
    void u32(uint32_t value);
    void u64(uint64_t value);
    void f32(float value);
    void bytes(const void* data, size_t size);
    void string(std::string_view value);
    const Bytes& data() const
    {
        return mData;
    }
    Bytes take()
    {
        return std::move(mData);
    }

private:
    Bytes mData;
};

// A failed read is sticky. Never allocate from an unchecked archive count.
class Reader
{
public:
    Reader(const void* data, size_t size);
    uint8_t u8();
    uint32_t u32();
    uint64_t u64();
    float f32();
    bool bytes(void* destination, size_t size);
    std::string string();
    size_t remaining() const
    {
        return mSize - mPosition;
    }
    bool good() const
    {
        return mGood;
    }
    bool done() const
    {
        return mGood && remaining() == 0;
    }

private:
    const uint8_t* mData;
    size_t mSize;
    size_t mPosition = 0;
    bool mGood = true;
};

// One immutable, checksummed artifact. Compatibility is an explicit digest of
// the caller's exact build/SDK/platform/options profile, separate from content.
struct Bundle
{
    Digest input{};
    Digest compatibility{};
    Bytes runtime;
    Bytes native;
};
struct ByteView
{
    const uint8_t* data = nullptr;
    size_t size = 0;
};
struct BundleView
{
    Digest input{}, compatibility{};
    ByteView runtime, native;
};
// Transactional validation without section copies. The caller keeps the input
// immutable and alive until every consumer of these borrowed views has joined.
bool decodeBundleView(const uint8_t* bytes,
                      size_t size,
                      const Digest& input,
                      const Digest& compatibility,
                      BundleView& output,
                      size_t maxBytes = 512 * 1024 * 1024);
Bytes encodeBundle(const Bundle& bundle);
// Transactional: output is unchanged on any failure. No PhysX code is called
// until this envelope has been validated. maxBytes bounds total allocations.
// A successful output owns its decoded sections independently of bytes.
bool decodeBundle(const Bytes& bytes,
                  const Digest& input,
                  const Digest& compatibility,
                  Bundle& output,
                  size_t maxBytes = 512 * 1024 * 1024);
} // namespace omni::physics::parse::cache

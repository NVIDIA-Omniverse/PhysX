// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-ARCHIVE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */
#include <omni/physics/parse/CacheArchive.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <new>
#include <atomic>
#include <future>
#include <thread>
#if defined(__x86_64__) || defined(_M_X64)
#    include <immintrin.h>
#    if defined(_MSC_VER)
#        include <intrin.h>
#    else
#        include <cpuid.h>
#    endif
#endif

namespace omni::physics::parse::cache
{
namespace
{
std::atomic_size_t digestCalls{ 0 };
constexpr uint32_t kRound[64] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
                                  0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
                                  0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
                                  0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
                                  0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
                                  0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
                                  0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
                                  0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
                                  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
                                  0xc67178f2 };
uint32_t rotate(uint32_t x, uint32_t n)
{
    return (x >> n) | (x << (32 - n));
}
void compress(uint32_t* state, const uint8_t* block)
{
    uint32_t w[64];
    for (size_t i = 0; i < 16; ++i)
        w[i] = (uint32_t(block[i * 4]) << 24) | (uint32_t(block[i * 4 + 1]) << 16) | (uint32_t(block[i * 4 + 2]) << 8) |
               block[i * 4 + 3];
    for (size_t i = 16; i < 64; ++i)
    {
        const uint32_t a = w[i - 15], b = w[i - 2];
        w[i] = w[i - 16] + (rotate(a, 7) ^ rotate(a, 18) ^ (a >> 3)) + w[i - 7] +
               (rotate(b, 17) ^ rotate(b, 19) ^ (b >> 10));
    }
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
    for (size_t i = 0; i < 64; ++i)
    {
        const uint32_t t1 = h + (rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25)) + ((e & f) ^ (~e & g)) + kRound[i] + w[i];
        const uint32_t t2 = (rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
}
// Keep the portable implementation for unsupported CPUs and architectures.
void compressBlocksPortable(uint32_t* state, const uint8_t* bytes, size_t blocks)
{
    for (size_t i = 0; i < blocks; ++i)
        compress(state, bytes + 64 * i);
}
#if defined(__x86_64__) || defined(_M_X64)
// Function-local ISA selection: the rest of the binary retains its baseline ISA.
#    if !defined(_MSC_VER)
__attribute__((target("sha,ssse3,sse4.1")))
#    endif
void compressBlocksSha(uint32_t* state, const uint8_t* bytes, size_t blocks)
{
    __m128i tmp = _mm_shuffle_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(state)), 0xb1);
    __m128i second = _mm_shuffle_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(state + 4)), 0x1b);
    __m128i first = _mm_alignr_epi8(tmp, second, 8);
    second = _mm_blend_epi16(second, tmp, 0xf0);
    const __m128i endian = _mm_set_epi64x(0x0c0d0e0f08090a0bLL, 0x0405060700010203LL);
    for (size_t block = 0; block < blocks; ++block)
    {
        const __m128i savedFirst = first, savedSecond = second;
        __m128i words[4];
        for (size_t group = 0; group < 16; ++group)
        {
            const size_t slot = group % 4;
            if (group < 4)
                words[slot] = _mm_shuffle_epi8(
                    _mm_loadu_si128(reinterpret_cast<const __m128i*>(bytes + block * 64 + group * 16)), endian);
            else
            {
                tmp = _mm_sha256msg1_epu32(words[slot], words[(slot + 1) % 4]);
                tmp = _mm_add_epi32(tmp, _mm_alignr_epi8(words[(slot + 3) % 4], words[(slot + 2) % 4], 4));
                words[slot] = _mm_sha256msg2_epu32(tmp, words[(slot + 3) % 4]);
            }
            tmp = _mm_add_epi32(words[slot], _mm_loadu_si128(reinterpret_cast<const __m128i*>(kRound + group * 4)));
            second = _mm_sha256rnds2_epu32(second, first, tmp);
            tmp = _mm_shuffle_epi32(tmp, 0x0e);
            first = _mm_sha256rnds2_epu32(first, second, tmp);
        }
        first = _mm_add_epi32(first, savedFirst);
        second = _mm_add_epi32(second, savedSecond);
    }
    tmp = _mm_shuffle_epi32(first, 0x1b);
    second = _mm_shuffle_epi32(second, 0xb1);
    first = _mm_blend_epi16(tmp, second, 0xf0);
    second = _mm_alignr_epi8(second, tmp, 8);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(state), first);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(state + 4), second);
}
bool hasShaInstructions()
{
#    if defined(_MSC_VER)
    int cpu[4];
    __cpuid(cpu, 0);
    if (cpu[0] < 7)
        return false;
    __cpuidex(cpu, 7, 0);
    if (!(uint32_t(cpu[1]) & (1u << 29)))
        return false;
    __cpuid(cpu, 1);
    const uint32_t features = uint32_t(cpu[2]);
#    else
    unsigned int a, b, c, d;
    if (!__get_cpuid_count(7, 0, &a, &b, &c, &d) || !(b & (1u << 29)))
        return false;
    if (!__get_cpuid(1, &a, &b, &c, &d))
        return false;
    const uint32_t features = c;
#    endif
    return (features & ((1u << 9) | (1u << 19))) == ((1u << 9) | (1u << 19));
}
#endif
using CompressBlocks = void (*)(uint32_t*, const uint8_t*, size_t);
CompressBlocks selectCompressBlocks()
{
#if defined(__x86_64__) || defined(_M_X64)
    if (hasShaInstructions())
        return compressBlocksSha;
#endif
    return compressBlocksPortable;
}
constexpr uint64_t kMagic = 0x314548434143564fULL; // "OVCACHE1"
constexpr uint32_t kVersion = 2;
constexpr size_t kHeaderSize = 8 + 4 + 32 + 32 + 8 + 8;
} // namespace

static Digest digestWith(const void* data, size_t size, CompressBlocks compressBlocks)
{
    uint32_t state[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
    const uint8_t* cursor = static_cast<const uint8_t*>(data);
    size_t remaining = size;
    if (remaining >= 64)
    {
        const size_t blocks = remaining / 64;
        compressBlocks(state, cursor, blocks);
        cursor += blocks * 64;
        remaining -= blocks * 64;
    }
    uint8_t tail[128]{};
    if (remaining)
        std::memcpy(tail, cursor, remaining);
    tail[remaining] = 0x80;
    const size_t length = remaining < 56 ? 64 : 128;
    const uint64_t bits = uint64_t(size) * 8;
    for (size_t i = 0; i < 8; ++i)
        tail[length - 1 - i] = uint8_t(bits >> (i * 8));
    compressBlocks(state, tail, length / 64);
    Digest result{};
    for (size_t i = 0; i < 32; ++i)
        result[i] = uint8_t(state[i / 4] >> ((3 - i % 4) * 8));
    return result;
}
Digest digest(const void* data, size_t size)
{
    digestCalls.fetch_add(1, std::memory_order_relaxed);
    static const CompressBlocks compressBlocks = selectCompressBlocks();
    return digestWith(data, size, compressBlocks);
}
namespace detail
{
void resetDigestCallsForTest()
{
    digestCalls.store(0, std::memory_order_relaxed);
}
size_t getDigestCallsForTest()
{
    return digestCalls.load(std::memory_order_relaxed);
}
// Internal reference implementation for accelerator parity tests.
Digest digestPortable(const void* data, size_t size)
{
    return digestWith(data, size, compressBlocksPortable);
}
bool digestAcceleratedForTest()
{
    return selectCompressBlocks() != compressBlocksPortable;
}
} // namespace detail
std::string hex(const Digest& value)
{
    constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(64);
    for (uint8_t byte : value)
    {
        result.push_back(digits[byte >> 4]);
        result.push_back(digits[byte & 15]);
    }
    return result;
}
void Writer::u8(uint8_t value)
{
    mData.push_back(value);
}
void Writer::u32(uint32_t value)
{
    const uint8_t encoded[] = { uint8_t(value), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24) };
    bytes(encoded, sizeof(encoded));
}
void Writer::u64(uint64_t value)
{
    uint8_t encoded[8];
    for (size_t i = 0; i < 8; ++i)
        encoded[i] = uint8_t(value >> (8 * i));
    bytes(encoded, sizeof(encoded));
}
void Writer::f32(float value)
{
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    uint32_t bits;
    std::memcpy(&bits, &value, 4);
    u32(bits);
}
void Writer::bytes(const void* data, size_t size)
{
    if (size)
    {
        const uint8_t* begin = static_cast<const uint8_t*>(data);
        mData.insert(mData.end(), begin, begin + size);
    }
}
void Writer::string(std::string_view value)
{
    u64(value.size());
    bytes(value.data(), value.size());
}
Reader::Reader(const void* data, size_t size)
    : mData(static_cast<const uint8_t*>(data)), mSize(size), mGood(data || !size)
{
}
bool Reader::bytes(void* destination, size_t size)
{
    if (!mGood || size > remaining() || (!destination && size))
        return mGood = false;
    if (size)
        std::memcpy(destination, mData + mPosition, size);
    mPosition += size;
    return true;
}
uint8_t Reader::u8()
{
    uint8_t value = 0;
    bytes(&value, 1);
    return value;
}
uint32_t Reader::u32()
{
    uint8_t value[4]{};
    if (!bytes(value, sizeof(value)))
        return 0;
    return uint32_t(value[0]) | (uint32_t(value[1]) << 8) | (uint32_t(value[2]) << 16) | (uint32_t(value[3]) << 24);
}
uint64_t Reader::u64()
{
    uint8_t value[8]{};
    if (!bytes(value, sizeof(value)))
        return 0;
    uint64_t result = 0;
    for (size_t i = 0; i < 8; ++i)
        result |= uint64_t(value[i]) << (8 * i);
    return result;
}
float Reader::f32()
{
    const uint32_t bits = u32();
    float value;
    std::memcpy(&value, &bits, 4);
    return value;
}
std::string Reader::string()
{
    const uint64_t size = u64();
    if (!mGood || size > remaining())
    {
        mGood = false;
        return {};
    }
    std::string result;
    if (size)
        result.assign(reinterpret_cast<const char*>(mData + mPosition), size_t(size));
    mPosition += size_t(size);
    return result;
}
namespace
{
// Version 2 binds a large envelope through ordered SHA-256 chunk digests. The
// root includes its domain, byte length, chunk size and count; every body byte
// and its position remain protected. Small envelopes retain a single SHA-256.
Digest envelopeDigest(const void* data, size_t size)
{
    constexpr size_t chunkSize = 1024 * 1024;
    if (size <= chunkSize)
        return digest(data, size);
    const size_t count = size / chunkSize + (size % chunkSize != 0);
    std::vector<Digest> chunks(count);
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    std::atomic<size_t> next{ 0 };
    auto worker = [&]() {
        for (;;)
        {
            const size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= count)
                break;
            const size_t offset = i * chunkSize;
            chunks[i] = digest(bytes + offset, std::min(chunkSize, size - offset));
        }
    };
    const size_t workers = std::min<size_t>({ count, 8, std::max(1u, std::thread::hardware_concurrency()) });
    std::vector<std::future<void>> jobs;
    for (size_t i = 1; i < workers; ++i)
        jobs.push_back(std::async(std::launch::async, worker));
    worker();
    for (std::future<void>& job : jobs)
        job.get();
    Writer root;
    root.string("ovruntime.bundle.v2.chunk-sha256");
    root.u64(size);
    root.u64(chunkSize);
    root.u64(count);
    for (const Digest& chunk : chunks)
        root.bytes(chunk.data(), chunk.size());
    return digest(root.data().data(), root.data().size());
}
} // namespace
Bytes encodeBundle(const Bundle& bundle)
{
    Writer w;
    if (bundle.runtime.size() > SIZE_MAX - kHeaderSize - 32 ||
        bundle.native.size() > SIZE_MAX - kHeaderSize - 32 - bundle.runtime.size())
        throw std::bad_alloc();
    w.reserve(kHeaderSize + 32 + bundle.runtime.size() + bundle.native.size());
    w.u64(kMagic);
    w.u32(kVersion);
    w.bytes(bundle.input.data(), bundle.input.size());
    w.bytes(bundle.compatibility.data(), bundle.compatibility.size());
    w.u64(bundle.runtime.size());
    w.u64(bundle.native.size());
    w.bytes(bundle.runtime.data(), bundle.runtime.size());
    w.bytes(bundle.native.data(), bundle.native.size());
    const Digest checksum = envelopeDigest(w.data().data(), w.data().size());
    w.bytes(checksum.data(), checksum.size());
    return w.take();
}
bool decodeBundle(const Bytes& bytes, const Digest& input, const Digest& compatibility, Bundle& output, size_t maxBytes)
{
    BundleView view;
    if (!decodeBundleView(bytes.data(), bytes.size(), input, compatibility, view, maxBytes))
        return false;
    Bundle candidate;
    candidate.input = view.input;
    candidate.compatibility = view.compatibility;
    candidate.runtime.assign(view.runtime.data, view.runtime.data + view.runtime.size);
    candidate.native.assign(view.native.data, view.native.data + view.native.size);
    output = std::move(candidate);
    return true;
}
bool decodeBundleView(const uint8_t* bytes,
                      size_t size,
                      const Digest& input,
                      const Digest& compatibility,
                      BundleView& output,
                      size_t maxBytes)
{
    if (!bytes || size < kHeaderSize + 32 || size > maxBytes)
        return false;
    const size_t bodySize = size - 32;
    const Digest checksum = envelopeDigest(bytes, bodySize);
    if (std::memcmp(checksum.data(), bytes + bodySize, 32))
        return false;
    Reader r(bytes, bodySize);
    if (r.u64() != kMagic || r.u32() != kVersion)
        return false;
    BundleView candidate;
    r.bytes(candidate.input.data(), 32);
    r.bytes(candidate.compatibility.data(), 32);
    const uint64_t runtimeSize = r.u64(), nativeSize = r.u64();
    if (!r.good() || candidate.input != input || candidate.compatibility != compatibility ||
        runtimeSize > r.remaining() || nativeSize != r.remaining() - runtimeSize)
        return false;
    const uint8_t* runtime = bytes + bodySize - r.remaining();
    candidate.runtime = { runtime, size_t(runtimeSize) };
    candidate.native = { runtime + size_t(runtimeSize), size_t(nativeSize) };
    output = std::move(candidate);
    return true;
}
} // namespace omni::physics::parse::cache

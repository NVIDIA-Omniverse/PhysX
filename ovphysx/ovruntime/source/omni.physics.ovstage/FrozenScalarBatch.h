// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3 AC-5
 */
#pragma once
#include <omni/physics/parse/IPhysicsSource.h>
#include <ovstage/ovstage.h>
#include <limits>
#include <type_traits>
#include <memory>
#include <vector>

namespace omni::physics::ovstage
{
using namespace omni::physics::parse;
class OvstageSource;

// Numeric scalar/vector rows and coordinator-resolved token values. This object owns references to retained read
// groups, not a live source. Read-group release and owner copies happen outside
// workers. The ovstage instance must outlive the retained groups.
class FrozenScalarBatch
{
public:
    class Row
    {
    public:
        // Match OvstageSource: producer-side population resolves authored defaults.
        bool hasAuthoredAttribute(ObjectKey, TokenId) const noexcept { return true; }
        template <typename T> bool getAttribute(ObjectKey key, TokenId attr, T& out) const noexcept
        {
            const AttrValue v = mBatch->read(mIndex, key, attr, *mComplete);
            using K = AttrValue::Kind;
            if constexpr (std::is_same_v<T, bool>) { if (v.kind == K::eBool) { out = v.b; return true; } }
            else if constexpr (std::is_same_v<T, int64_t>)
            {
                if (v.kind == K::eInt) { out = v.i; return true; }
                if (v.kind == K::eBool) { out = v.b ? 1 : 0; return true; }
            }
            else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>)
            {
                if (v.kind == K::eFloat || v.kind == K::eHalf) { out = static_cast<T>(v.f); return true; }
                if (v.kind == K::eDouble) { out = static_cast<T>(v.d); return true; }
            }
            else if constexpr (std::is_same_v<T, carb::Float2>) { if (v.kind == K::eFloat2) { out = v.f2; return true; } }
            else if constexpr (std::is_same_v<T, carb::Float3>) { if (v.kind == K::eFloat3) { out = v.f3; return true; } }
            else if constexpr (std::is_same_v<T, carb::Float4>) { if (v.kind == K::eFloat4) { out = v.f4; return true; } }
            else if constexpr (std::is_same_v<T, TokenId>) { if (v.kind == K::eToken) { out = v.tok; return true; } }
            else static_assert(!sizeof(T), "unsupported frozen scalar type");
            return false;
        }
    private:
        friend class FrozenScalarBatch;
        const FrozenScalarBatch* mBatch;
        size_t mIndex;
        bool* mComplete;
        Row(const FrozenScalarBatch* batch, size_t index, bool& complete) : mBatch(batch), mIndex(index), mComplete(&complete) {}
    };
    Row row(size_t index, bool& complete) const noexcept { return Row(this, index, complete); }
    size_t capacityBytes() const noexcept
    {
        return mKeys.capacity() * sizeof(ObjectKey) + mValues.capacity() * sizeof(Reference) + mColumns.capacity() * sizeof(uint32_t) +
               mFallbacks.capacity() * sizeof(AttrValue);
    }
    size_t fallbackValues() const noexcept { return mFallbacks.size(); }
private:
    friend class OvstageSource;
    friend struct OvstageSourceTestAccess;
    struct Reference { const DLTensor* tensor = nullptr; int64_t comps = 0; uint32_t row = 0; };
    AttrValue read(size_t index, ObjectKey key, TokenId attr, bool& complete) const noexcept;
    std::shared_ptr<const void> mOwner;
    std::vector<ObjectKey> mKeys;
    std::vector<Reference> mValues;
    std::vector<uint32_t> mColumns;
    std::vector<AttrValue> mFallbacks;
    size_t mRows = 0, mStride = 0;
};
} // namespace omni::physics::ovstage

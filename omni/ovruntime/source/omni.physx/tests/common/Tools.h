// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
    namespace physx
    {

        inline uint64_t asInt(const PXR_NS::SdfPath& path)
        {
            static_assert(sizeof(PXR_NS::SdfPath) == sizeof(uint64_t), "Change to make the same size as PXR_NS::SdfPath");
            uint64_t ret;
            std::memcpy(&ret, &path, sizeof(PXR_NS::SdfPath));
            return ret;
        }

        inline const PXR_NS::SdfPath& intToPath(const uint64_t& path)
        {
            static_assert(sizeof(PXR_NS::SdfPath) == sizeof(uint64_t), "Change to make the same size as PXR_NS::SdfPath");

            return reinterpret_cast<const PXR_NS::SdfPath&>(path);
        }

        class BasicRandom
        {
        public:
            BasicRandom(uint32_t seed = 0) : mRnd(seed) {}
            ~BasicRandom() {}

            inline	void		setSeed(uint32_t seed) { mRnd = seed; }
            inline	uint32_t		getCurrentValue()	const { return mRnd; }
            uint32_t		randomize() { mRnd = mRnd * 2147001325 + 715136305; return mRnd; }

            inline	uint32_t		rand() { return randomize() & 0xffff; }
            inline	uint32_t		rand32() { return randomize() & 0xffffffff; }

            float		rand(float a, float b)
            {
                const float r = rand32() / (static_cast<float>(0xffffffff));
                return r * (b - a) + a;
            }

            int32_t		rand(int32_t a, int32_t b)
            {
                return a + static_cast<int32_t>(rand32() % (b - a));
            }

            float		randomFloat()
            {
                return rand() / (static_cast<float>(0xffff)) - 0.5f;
            }
            float		randomFloat32()
            {
                return rand32() / (static_cast<float>(0xffffffff)) - 0.5f;
            }

            float		randomFloat32(float a, float b) { return rand32() / float(0xffffffff) * (b - a) + a; }

            void		unitRandomPt(PXR_NS::GfVec3f& v)
            {
                v = unitRandomPt();
            }

            void		unitRandomQuat(PXR_NS::GfQuatf& v)
            {
                v = unitRandomQuat();
            }

            PXR_NS::GfVec3f		unitRandomPt()
            {
                PXR_NS::GfVec3f v;
                do
                {
                    v[0] = randomFloat();
                    v[1] = randomFloat();
                    v[2] = randomFloat();
                } while (v.Normalize() < 1e-6f);
                return v;
            }

            PXR_NS::GfQuatf		unitRandomQuat()
            {
                PXR_NS::GfQuatf v;
                do
                {
                    PXR_NS::GfVec3f i;
                    i[0] = randomFloat();
                    i[1] = randomFloat();
                    i[2] = randomFloat();
                    v.SetImaginary(i);
                    v.SetReal(randomFloat());
                } while (v.Normalize() < 1e-6f);

                return v;
            }

            PXR_NS::GfQuath		unitRandomQuath()
            {
                PXR_NS::GfQuath v;
                do
                {
                    PXR_NS::GfVec3h i;
                    i[0] = randomFloat();
                    i[1] = randomFloat();
                    i[2] = randomFloat();
                    v.SetImaginary(i);
                    v.SetReal(randomFloat());
                } while (v.Normalize() < 1e-6f);

                return v;
            }


        private:
            uint32_t		mRnd;
        };
    }
}

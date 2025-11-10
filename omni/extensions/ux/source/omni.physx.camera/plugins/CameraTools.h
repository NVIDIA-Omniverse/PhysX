// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <math.h>

#include <carb/Defines.h>

#include <PxPhysicsAPI.h>


template <typename V>
inline V Lerp(const V& start, const V& end, const float& t)
{
    float tClamp = CARB_CLAMP(t, 0.0f, 1.0f);
    return start + (end - start) * tClamp;
}

template <typename V>
inline V Lerp(const float& x0, const V& y0, const float& x1, const V& y1, const float& in)
{
    float t = (in - x0) / (x1 - x0);
    t = CARB_CLAMP(t, 0.0f, 1.0f);

    return y0 + t * (y1 - y0);
}

inline float gaussian(float a, float b, float c, float x)
{
    float numerator = -(x - b) * (x - b);
    float denominator = 2.0f * c * c;

    return a * exp(numerator / denominator);
}

inline ::physx::PxQuat Slerp(const ::physx::PxQuat& a, const ::physx::PxQuat& b, const float t)
{
    ::physx::PxQuat q;

    float cosTheta = a.dot(b);

    float beta;
    float theta;

    if (cosTheta > 0.0f)
    {
        beta = 1.0f;
        theta = acosf(cosTheta);
    }
    else
    {
        beta = -1.0f;
        theta = (float)M_PI - acosf(cosTheta);
    }

    if (fabsf(cosTheta) < 1.0f)
    {
        float inverseSinTheta = 1.0f / sinf(theta);

        float tTheta = t * theta;
        float alpha = sin(theta - tTheta) * inverseSinTheta;
        beta *= sin(tTheta) * inverseSinTheta;

        q.w = alpha * a.w + beta * b.w;
        q.x = alpha * a.x + beta * b.x;
        q.y = alpha * a.y + beta * b.y;
        q.z = alpha * a.z + beta * b.z;
    }
    else
    {
        q = a;
    }

    return q;
}

class LowPassFilter
{
private:
    bool mInitialized;

    float mTimeConstant;
    float mOneOverTimeConstant;

    float mValue;

public:
    LowPassFilter()
    {
        mInitialized = false;

        mValue = 0.0f;
        mTimeConstant = 0.0f;
        mOneOverTimeConstant = 0.0f;
    }

    void setTimeConstant(float timeConstant)
    {
        if (timeConstant > 0.0f)
        {
            mTimeConstant = timeConstant;
            mOneOverTimeConstant = 1.0f / timeConstant;
        }

        mInitialized = true;
    }

    void reset(float initialValue)
    {
        mValue = initialValue;
    }

    float getValue()
    {
        return mValue;
    }

    float filter(float value, float timeStep)
    {
        if (mInitialized && timeStep < mTimeConstant)
        {
            float k = timeStep * mOneOverTimeConstant;

            mValue = k * value + (1.0f - k) * mValue;
        }
        else
        {
            mValue = value;
        }

        return mValue;
    }
};

class VectorLowPassFilter
{
private:
    bool mInitialized;

    ::physx::PxVec3 mTimeConstant;
    ::physx::PxVec3 mOneOverTimeConstant;

    ::physx::PxVec3 mValue;

public:
    VectorLowPassFilter()
    {
        mInitialized = false;

        mValue = ::physx::PxVec3(0.0f);
        mTimeConstant = ::physx::PxVec3(0.0f);
        mOneOverTimeConstant = ::physx::PxVec3(0.0f);
    }

    void setTimeConstant(::physx::PxVec3 timeConstant)
    {
        if (timeConstant.x > 0.0f)
        {
            mTimeConstant.x = timeConstant.x;
            mOneOverTimeConstant.x = 1.0f / timeConstant.x;
        }

        if (timeConstant.y > 0.0f)
        {
            mTimeConstant.y = timeConstant.y;
            mOneOverTimeConstant.y = 1.0f / timeConstant.y;
        }

        if (timeConstant.z > 0.0f)
        {
            mTimeConstant.z = timeConstant.z;
            mOneOverTimeConstant.z = 1.0f / timeConstant.z;
        }

        mInitialized = true;
    }

    void reset(::physx::PxVec3 initialValue)
    {
        mValue = initialValue;
    }

    ::physx::PxVec3 getValue()
    {
        return mValue;
    }

    ::physx::PxVec3 filter(::physx::PxVec3 value, float timeStep)
    {
        float k;
        ::physx::PxVec3 nextValue = value;

        if (mInitialized && timeStep < mTimeConstant.x)
        {
            k = timeStep * mOneOverTimeConstant.x;
            nextValue.x = k * value.x + (1.0f - k) * mValue.x;
        }

        if (mInitialized && timeStep < mTimeConstant.y)
        {
            k = timeStep * mOneOverTimeConstant.y;
            nextValue.y = k * value.y + (1.0f - k) * mValue.y;
        }

        if (mInitialized && timeStep < mTimeConstant.z)
        {
            k = timeStep * mOneOverTimeConstant.z;
            nextValue.z = k * value.z + (1.0f - k) * mValue.z;
        }

        mValue = nextValue;

        return mValue;
    }
};

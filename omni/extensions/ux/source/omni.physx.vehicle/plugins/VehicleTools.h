// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <math.h>

#include <carb/Defines.h>

#include <PxPhysicsAPI.h>


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
        else
        {
            mTimeConstant = 0.0f;
            mOneOverTimeConstant = 0.0f;
        }
    }

    float getTimeConstant()
    {
        return mTimeConstant;
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
        if (timeStep < mTimeConstant)
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
    ::physx::PxVec3 mTimeConstant;
    ::physx::PxVec3 mOneOverTimeConstant;

    ::physx::PxVec3 mValue;

public:
    VectorLowPassFilter()
    {
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
        else
        {
            mTimeConstant.x = 0.0f;
            mOneOverTimeConstant.x = 0.0f;
        }

        if (timeConstant.y > 0.0f)
        {
            mTimeConstant.y = timeConstant.y;
            mOneOverTimeConstant.y = 1.0f / timeConstant.y;
        }
        else
        {
            mTimeConstant.y = 0.0f;
            mOneOverTimeConstant.y = 0.0f;
        }

        if (timeConstant.z > 0.0f)
        {
            mTimeConstant.z = timeConstant.z;
            mOneOverTimeConstant.z = 1.0f / timeConstant.z;
        }
        else
        {
            mTimeConstant.z = 0.0f;
            mOneOverTimeConstant.z = 0.0f;
        }
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

        if (timeStep < mTimeConstant.x)
        {
            k = timeStep * mOneOverTimeConstant.x;
            nextValue.x = k * value.x + (1.0f - k) * mValue.x;
        }

        if (timeStep < mTimeConstant.y)
        {
            k = timeStep * mOneOverTimeConstant.y;
            nextValue.y = k * value.y + (1.0f - k) * mValue.y;
        }

        if (timeStep < mTimeConstant.z)
        {
            k = timeStep * mOneOverTimeConstant.z;
            nextValue.z = k * value.z + (1.0f - k) * mValue.z;
        }

        mValue = nextValue;

        return mValue;
    }
};

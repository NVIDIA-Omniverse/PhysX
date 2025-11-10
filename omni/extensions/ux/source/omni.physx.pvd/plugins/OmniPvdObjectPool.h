// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

template <typename T>
class OmniPvdObjectPoolSlice
{
public:
    OmniPvdObjectPoolSlice()
    {
        mNbrObjects = 0;
        mNbrUsedObjects = 0;
        mObjectArray = 0;
        mNextSlice = 0;
    }

    ~OmniPvdObjectPoolSlice()
    {
        delete[] mObjectArray;
        mObjectArray = 0;
        mNbrObjects = 0;
    }

    void allocateSlice(int nbrObjects)
    {
        if (mObjectArray)
            return;
        mNbrObjects = nbrObjects;
        mObjectArray = new T[mNbrObjects];
        mNbrUsedObjects = 0;
    }

    bool hasFreeObject()
    {
        return (mNbrUsedObjects < mNbrObjects) ? true : false;
    }

    T* getFreeObject()
    {
        if (!hasFreeObject())
            return 0;
        mNbrUsedObjects++;
        return &mObjectArray[mNbrUsedObjects - 1];
    }

    int mNbrObjects;
    int mNbrUsedObjects;
    T* mObjectArray;
    OmniPvdObjectPoolSlice* mNextSlice;
};

template <typename T>
class OmniPvdObjectPool
{
public:
    OmniPvdObjectPool()
    {
        mFirstSlice = 0;
        mNbrObjectPerSlice = 64;
    }

    ~OmniPvdObjectPool()
    {
        OmniPvdObjectPoolSlice<T>* slice = mFirstSlice;
        while (slice)
        {
            OmniPvdObjectPoolSlice<T>* nextSlice = slice->mNextSlice;
            delete slice;
            slice = nextSlice;
        }
    }

    void setObjectsPerSlice(int nbrObjectPerSlice)
    {
        mNbrObjectPerSlice = nbrObjectPerSlice;
    }

    T* allocObject()
    {
        if (mFirstSlice)
        {
            if (!mFirstSlice->hasFreeObject())
            {
                addSlice(mNbrObjectPerSlice);
            }
        }
        else
        {
            addSlice(mNbrObjectPerSlice);
        }
        return mFirstSlice->getFreeObject();
    }

    void addSlice(int nbrObjectInSlice)
    {
        // Prepend the slice to the front of the list
        OmniPvdObjectPoolSlice<T>* slice = new OmniPvdObjectPoolSlice<T>;
        slice->allocateSlice(nbrObjectInSlice);
        if (mFirstSlice)
        {
            slice->mNextSlice = mFirstSlice;
        }
        mFirstSlice = slice;
    }

    int mNbrObjectPerSlice;
    OmniPvdObjectPoolSlice<T>* mFirstSlice;
};

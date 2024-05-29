// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.

#pragma once

#include "NvPreprocessor.h"
#include <assert.h>
#include <stdio.h>
#include <vector>


#if NV_WINDOWS_FAMILY
#define POD_Buffer std::vector
#else
template<typename T, int Alignment = sizeof(T)>
class POD_Buffer
{
public:
    POD_Buffer() : _size(0), _capacity(0), _data(nullptr) {}
    ~POD_Buffer() { deallocate(); }

    size_t  size() const { return _size; }

    void
    resize(size_t new_size)
    {
        if (new_size > _capacity)
        {
            reserve(new_size);
        }
        _size = new_size;
    }

    void
    reserve(size_t min_capacity)
    {
        if (min_capacity > _capacity)
        {
            void* new_data = allocate(min_capacity);
            if (!!_size)
            {
                memcpy(new_data, _data, _size*sizeof(T));
            }
            deallocate();
            _capacity = min_capacity;
            _data = reinterpret_cast<T*>(new_data);
        }
    }

    void
    push_back(const T& e)
    {
        if (_size >= _capacity)
        {
            reserve(!!_size ? 2*_size : (size_t)16);
        }
        _data[_size++] = e;
    }

    void
    pop_back()
    {
        if (!!_size) --_size;
    }

    T* data() { return _data; }
    const T* data() const { return _data; }

    T& operator [] (size_t index) { assert(_size > index); return _data[index]; }
    const T& operator [] (size_t index) const { assert(_size > index); return _data[index]; }

    T& back() { return (*this)[_size-1]; }
    const T& back() const { return (*this)[_size-1]; }

private:
    void*
    allocate(size_t buffer_size)
    {
        const size_t mem_size = sizeof(T)*buffer_size;
        unsigned char* mem = (unsigned char*)malloc(mem_size + Alignment);
        const unsigned char offset = (unsigned char)((uintptr_t)Alignment - (uintptr_t)mem % Alignment - 1);
        mem += offset;
        *mem++ = offset;
        return mem;
    }

    void
    deallocate()
    {
        if (!!_data)
        {
            unsigned char* cmem = (unsigned char*)_data;
            const unsigned char offset = *--cmem;
            ::free(cmem - offset);
        }
        _size = 0;
        _capacity = 0;
        _data = nullptr;
    }

    size_t  _size;
    size_t  _capacity;
    T*      _data;
};
#endif

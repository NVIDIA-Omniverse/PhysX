/*
 * Copyright 2009-2011 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO USER:
 *
 * This source code is subject to NVIDIA ownership rights under U.S. and
 * international Copyright laws.  Users and possessors of this source code
 * are hereby granted a nonexclusive, royalty-free license to use this code
 * in individual and commercial software.
 *
 * NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE
 * CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR
 * IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
 * IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS,  WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION,  ARISING OUT OF OR IN CONNECTION WITH THE USE
 * OR PERFORMANCE OF THIS SOURCE CODE.
 *
 * U.S. Government End Users.   This source code is a "commercial item" as
 * that term is defined at  48 C.F.R. 2.101 (OCT 1995), consisting  of
 * "commercial computer  software"  and "commercial computer software
 * documentation" as such terms are  used in 48 C.F.R. 12.212 (SEPT 1995)
 * and is provided to the U.S. Government only as a commercial end item.
 * Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through
 * 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the
 * source code with only those rights set forth herein.
 *
 * Any use of this source code in individual and commercial software must
 * include, in the user documentation and internal comments to the code,
 * the above Disclaimer and U.S. Government End Users Notice.
 */

#ifndef NS_MEMORY_BUFFER_H
#define NS_MEMORY_BUFFER_H

#include "Ns.h"
#include "NsUserAllocated.h"
#include "NsAlignedMalloc.h"
#include "NvFileBuf.h"
#include "NvAssert.h"

namespace nvidia
{
namespace general_NvIOStream2
{
    using namespace shdfnd;

    const uint32_t BUFFER_SIZE_DEFAULT = 4096;

//Use this class if you want to use your own allocator
template<class Allocator>
class NvMemoryBufferBase : public NvFileBuf, public Allocator
{
    NV_NOCOPY(NvMemoryBufferBase)
    void init(const void *readMem, uint32_t readLen)
    {
        mAllocator = this;

        mReadBuffer = mReadLoc = static_cast<const uint8_t *>(readMem);
        mReadStop   = &mReadLoc[readLen];

        mWriteBuffer = mWriteLoc = mWriteStop = NULL;
        mWriteBufferSize = 0;
        mDefaultWriteBufferSize = BUFFER_SIZE_DEFAULT;

        mOpenMode = OPEN_READ_ONLY;
        mSeekType = SEEKABLE_READ;
    }

    void init(uint32_t defaultWriteBufferSize)
    {
        mAllocator = this;

        mReadBuffer = mReadLoc = mReadStop = NULL;

        mWriteBuffer = mWriteLoc = mWriteStop = NULL;
        mWriteBufferSize = 0;
        mDefaultWriteBufferSize = defaultWriteBufferSize;

        mOpenMode = OPEN_READ_WRITE_NEW;
        mSeekType = SEEKABLE_READWRITE;
    }

public:
    NvMemoryBufferBase(const void *readMem,uint32_t readLen)
    {
        init(readMem, readLen);
    }

    NvMemoryBufferBase(const void *readMem,uint32_t readLen, const Allocator &alloc): Allocator(alloc)
    {
        init(readMem, readLen);
    }

    NvMemoryBufferBase(uint32_t defaultWriteBufferSize = BUFFER_SIZE_DEFAULT)
    {
        init(defaultWriteBufferSize);
    }

    NvMemoryBufferBase(uint32_t defaultWriteBufferSize, const Allocator &alloc): Allocator(alloc)
    {
        init(defaultWriteBufferSize);
    }

    virtual ~NvMemoryBufferBase(void)
    {
        reset();
    }

    void setAllocator(Allocator *allocator)
    {
        mAllocator = allocator;
    }

    void initWriteBuffer(uint32_t size)
    {
        if ( mWriteBuffer == NULL )
        {
            if ( size < mDefaultWriteBufferSize ) size = mDefaultWriteBufferSize;
            mWriteBuffer = static_cast<uint8_t *>(mAllocator->allocate(size));
            NV_ASSERT( mWriteBuffer );
            mWriteLoc    = mWriteBuffer;
            mWriteStop  = &mWriteBuffer[size];
            mWriteBufferSize = size;
            mReadBuffer = mWriteBuffer;
            mReadStop   = &mWriteBuffer[size];
            mReadLoc    = mWriteBuffer;
        }
    }

    void reset(void)
    {
        mAllocator->deallocate(mWriteBuffer);
        mWriteBuffer = NULL;
        mWriteBufferSize = 0;
        mWriteLoc = NULL;
        mWriteStop = NULL;
        mReadBuffer = NULL;
        mReadStop = NULL;
        mReadLoc = NULL;
    }

    virtual OpenMode    getOpenMode(void) const
    {
        return mOpenMode;
    }


    SeekType isSeekable(void) const
    {
        return mSeekType;
    }

    virtual     uint32_t            read(void* buffer, uint32_t size)
    {
        if ( (mReadLoc+size) > mReadStop )
        {
            size = uint32_t(mReadStop - mReadLoc);
        }
        if ( size != 0 )
        {
            memmove(buffer,mReadLoc,size);
            mReadLoc+=size;
        }
        return size;
    }

    virtual     uint32_t            peek(void* buffer, uint32_t size)
    {
        if ( (mReadLoc+size) > mReadStop )
        {
            size = uint32_t(mReadStop - mReadLoc);
        }
        if ( size != 0 )
        {
            memmove(buffer,mReadLoc,size);
        }
        return size;
    }

    virtual     uint32_t        write(const void* buffer, uint32_t size)
    {
        NV_ASSERT( mOpenMode == OPEN_READ_WRITE_NEW );
        if ( mOpenMode == OPEN_READ_WRITE_NEW )
        {
            if ( (mWriteLoc+size) > mWriteStop )
                growWriteBuffer(size);
            memmove(mWriteLoc,buffer,size);
            mWriteLoc+=size;
            mReadStop = mWriteLoc;
        }
        else
        {
            size = 0;
        }
        return size;
    }

    NV_INLINE const uint8_t * getReadLoc(void) const { return mReadLoc; }
    NV_INLINE void advanceReadLoc(uint32_t len)
    {
        NV_ASSERT(mReadBuffer);
        if ( mReadBuffer )
        {
            mReadLoc+=len;
            if ( mReadLoc >= mReadStop )
            {
                mReadLoc = mReadStop;
            }
        }
    }

    virtual uint32_t tellRead(void) const
    {
        uint32_t ret=0;

        if ( mReadBuffer )
        {
            ret = uint32_t(mReadLoc-mReadBuffer);
        }
        return ret;
    }

    virtual uint32_t tellWrite(void) const
    {
        return uint32_t(mWriteLoc-mWriteBuffer);
    }

    virtual uint32_t seekRead(uint32_t loc)
    {
        uint32_t ret = 0;
        NV_ASSERT(mReadBuffer);
        if ( mReadBuffer )
        {
            mReadLoc = &mReadBuffer[loc];
            if ( mReadLoc >= mReadStop )
            {
                mReadLoc = mReadStop;
            }
            ret = uint32_t(mReadLoc-mReadBuffer);
        }
        return ret;
    }

    virtual uint32_t seekWrite(uint32_t loc)
    {
        uint32_t ret = 0;
        NV_ASSERT( mOpenMode == OPEN_READ_WRITE_NEW );
        if ( mWriteBuffer )
        {
            if ( loc > mWriteBufferSize )
            {
                mWriteLoc = mWriteStop;
                growWriteBuffer(loc - mWriteBufferSize);
            }
            mWriteLoc = &mWriteBuffer[loc];
            ret = uint32_t(mWriteLoc-mWriteBuffer);
        }
        return ret;
    }

    virtual void flush(void)
    {

    }

    virtual uint32_t getFileLength(void) const
    {
        uint32_t ret = 0;
        if ( mReadBuffer )
        {
            ret = uint32_t(mReadStop-mReadBuffer);
        }
        else if ( mWriteBuffer )
        {
            ret = uint32_t(mWriteLoc-mWriteBuffer);
        }
        return ret;
    }

    uint32_t    getWriteBufferSize(void) const
    {
        return uint32_t(mWriteLoc-mWriteBuffer);
    }

    void setWriteLoc(uint8_t *writeLoc)
    {
        NV_ASSERT(writeLoc >= mWriteBuffer && writeLoc < mWriteStop );
        mWriteLoc = writeLoc;
        mReadStop = mWriteLoc;
    }

    const uint8_t * getWriteBuffer(void) const
    {
        return mWriteBuffer;
    }

    /**
     * Attention: if you use aligned allocator you cannot free memory with NV_FREE macros instead use deallocate method from base
     */
    uint8_t * getWriteBufferOwnership(uint32_t &dataLen) // return the write buffer, and zero it out, the caller is taking ownership of the memory
    {
        uint8_t *ret = mWriteBuffer;
        dataLen = uint32_t(mWriteLoc-mWriteBuffer);
        mWriteBuffer = NULL;
        mWriteLoc = NULL;
        mWriteStop = NULL;
        mWriteBufferSize = 0;
        return ret;
    }


    void alignRead(uint32_t a)
    {
        uint32_t loc = tellRead();
        uint32_t aloc = ((loc+(a-1))/a)*a;
        if ( aloc != loc )
        {
            seekRead(aloc);
        }
    }

    void alignWrite(uint32_t a)
    {
        uint32_t loc = tellWrite();
        uint32_t aloc = ((loc+(a-1))/a)*a;
        if ( aloc != loc )
        {
            seekWrite(aloc);
        }
    }

private:


    // double the size of the write buffer or at least as large as the 'size' value passed in.
    void growWriteBuffer(uint32_t size)
    {
        if ( mWriteBuffer == NULL )
        {
            if ( size < mDefaultWriteBufferSize ) size = mDefaultWriteBufferSize;
            initWriteBuffer(size);
        }
        else
        {
            uint32_t oldWriteIndex = uint32_t(mWriteLoc - mWriteBuffer);
            uint32_t newSize =  mWriteBufferSize*2;
            uint32_t avail = newSize-oldWriteIndex;
            if ( size >= avail ) newSize = newSize+size;
            uint8_t *writeBuffer = static_cast<uint8_t *>(mAllocator->allocate(newSize));
            NV_ASSERT( writeBuffer );
            memmove(writeBuffer,mWriteBuffer,mWriteBufferSize);
            mAllocator->deallocate(mWriteBuffer);
            mWriteBuffer = writeBuffer;
            mWriteBufferSize = newSize;
            mWriteLoc = &mWriteBuffer[oldWriteIndex];
            mWriteStop = &mWriteBuffer[mWriteBufferSize];
            uint32_t oldReadLoc = uint32_t(mReadLoc-mReadBuffer);
            mReadBuffer = mWriteBuffer;
            mReadStop   = mWriteLoc;
            mReadLoc = &mReadBuffer[oldReadLoc];
        }
    }

    const   uint8_t *mReadBuffer;
    const   uint8_t *mReadLoc;
    const   uint8_t *mReadStop;

            uint8_t *mWriteBuffer;
            uint8_t *mWriteLoc;
            uint8_t *mWriteStop;

            uint32_t    mWriteBufferSize;
            uint32_t    mDefaultWriteBufferSize;
            Allocator   *mAllocator;
            OpenMode    mOpenMode;
            SeekType    mSeekType;

};

class NvMemoryBufferAllocator
{
public:
    NvMemoryBufferAllocator(uint32_t a = 0) : alignment(a) {}

    virtual void * allocate(uint32_t size)
    {
        switch(alignment)
        {
        case 0:
            return NV_ALLOC(size, NV_DEBUG_EXP("NvMemoryBufferAllocator"));         
        case 16 :
            return nvidia::AlignedAllocator<16>().allocate(size, __FILE__, __LINE__);           
        case 32 :
            return nvidia::AlignedAllocator<32>().allocate(size, __FILE__, __LINE__);           
        case 64 :
            return nvidia::AlignedAllocator<64>().allocate(size, __FILE__, __LINE__);           
        case 128 :
            return nvidia::AlignedAllocator<128>().allocate(size, __FILE__, __LINE__);          
        default :
            NV_ASSERT(0);
        }
        return NULL;
    }
    virtual void deallocate(void *mem)
    {
        switch(alignment)
        {
        case 0:
            NV_FREE(mem);
            break;
        case 16 :
            nvidia::AlignedAllocator<16>().deallocate(mem);         
            break;
        case 32 :
            nvidia::AlignedAllocator<32>().deallocate(mem);
            break;
        case 64 :
            nvidia::AlignedAllocator<64>().deallocate(mem);
            break;
        case 128 :
            nvidia::AlignedAllocator<128>().deallocate(mem);
            break;
        default :
            NV_ASSERT(0);
        }
    }
    virtual ~NvMemoryBufferAllocator(void) {}
private:
    NvMemoryBufferAllocator& operator=(const NvMemoryBufferAllocator&);

    const uint32_t alignment;
};

//Use this class if you want to use PhysX memory allocator
class NsMemoryBuffer: public NvMemoryBufferBase<NvMemoryBufferAllocator>, public UserAllocated
{
    NV_NOCOPY(NsMemoryBuffer)
    typedef NvMemoryBufferBase<NvMemoryBufferAllocator> BaseClass;

public:
    NsMemoryBuffer(const void *readMem,uint32_t readLen): BaseClass(readMem, readLen) {}    
    NsMemoryBuffer(const void *readMem,uint32_t readLen, uint32_t alignment): BaseClass(readMem, readLen, NvMemoryBufferAllocator(alignment)) {}

    NsMemoryBuffer(uint32_t defaultWriteBufferSize=BUFFER_SIZE_DEFAULT): BaseClass(defaultWriteBufferSize) {}
    NsMemoryBuffer(uint32_t defaultWriteBufferSize,uint32_t alignment): BaseClass(defaultWriteBufferSize, NvMemoryBufferAllocator(alignment)) {}
};

}
using namespace general_NvIOStream2;
}

#endif // NV_MEMORY_BUFFER_H


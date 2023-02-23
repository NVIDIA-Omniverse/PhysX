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

#ifndef NV_FILE_BUF_H
#define NV_FILE_BUF_H

#include "NvCTypes.h"

/** \addtogroup foundation
  @{
*/

namespace nvidia
{

namespace general_NvIOStream2
{

NV_PUSH_PACK_DEFAULT

/**
\brief Callback class for data serialization.

The user needs to supply an NvFileBuf implementation to a number of methods to allow the SDK to read or write
chunks of binary data. This allows flexibility for the source/destination of the data. For example the NvFileBuf
could store data in a file, memory buffer or custom file format.

\note It is the users responsibility to ensure that the data is written to the appropriate offset.

*/
class NvFileBuf
{
public:

    enum EndianMode
    {
        ENDIAN_NONE     = 0, // do no conversion for endian mode
        ENDIAN_BIG      = 1, // always read/write data as natively big endian (Power PC, etc.)
        ENDIAN_LITTLE   = 2 // always read/write data as natively little endian (Intel, etc.) Default Behavior!
    };

    NvFileBuf(EndianMode mode=ENDIAN_LITTLE)
    {
        setEndianMode(mode);
    }

    virtual ~NvFileBuf(void)
    {

    }

    /**
    \brief Declares a constant to seek to the end of the stream.
    *
    * Does not support streams longer than 32 bits
    */
    static const uint32_t STREAM_SEEK_END=0xFFFFFFFF;

    enum OpenMode
    {
        OPEN_FILE_NOT_FOUND,
        OPEN_READ_ONLY,                 // open file buffer stream for read only access
        OPEN_WRITE_ONLY,                // open file buffer stream for write only access
        OPEN_READ_WRITE_NEW,            // open a new file for both read/write access
        OPEN_READ_WRITE_EXISTING        // open an existing file for both read/write access
    };

    virtual OpenMode    getOpenMode(void) const  = 0;

    bool isOpen(void) const
    {
        return getOpenMode()!=OPEN_FILE_NOT_FOUND;
    }

    enum SeekType
    {
        SEEKABLE_NO         = 0,
        SEEKABLE_READ       = 0x1,
        SEEKABLE_WRITE      = 0x2,
        SEEKABLE_READWRITE  = 0x3
    };

    virtual SeekType isSeekable(void) const = 0;

    void    setEndianMode(EndianMode e)
    {
        mEndianMode = e;
        if ( (e==ENDIAN_BIG && !isBigEndian() ) ||
             (e==ENDIAN_LITTLE && isBigEndian() ) )
        {
            mEndianSwap = true;
        }
        else
        {
            mEndianSwap = false;
        }
    }

    EndianMode  getEndianMode(void) const
    {
        return mEndianMode;
    }

    virtual uint32_t getFileLength(void) const = 0;

    /**
    \brief Seeks the stream to a particular location for reading
    *
    * If the location passed exceeds the length of the stream, then it will seek to the end.
    * Returns the location it ended up at (useful if you seek to the end) to get the file position
    */
    virtual uint32_t    seekRead(uint32_t loc) = 0;

    /**
    \brief Seeks the stream to a particular location for writing
    *
    * If the location passed exceeds the length of the stream, then it will seek to the end.
    * Returns the location it ended up at (useful if you seek to the end) to get the file position
    */
    virtual uint32_t    seekWrite(uint32_t loc) = 0;

    /**
    \brief Reads from the stream into a buffer.

    \param[out] mem  The buffer to read the stream into.
    \param[in]  len  The number of bytes to stream into the buffer

    \return Returns the actual number of bytes read.  If not equal to the length requested, then reached end of stream.
    */
    virtual uint32_t    read(void *mem,uint32_t len) = 0;


    /**
    \brief Reads from the stream into a buffer but does not advance the read location.

    \param[out] mem  The buffer to read the stream into.
    \param[in]  len  The number of bytes to stream into the buffer

    \return Returns the actual number of bytes read.  If not equal to the length requested, then reached end of stream.
    */
    virtual uint32_t    peek(void *mem,uint32_t len) = 0;

    /**
    \brief Writes a buffer of memory to the stream

    \param[in] mem The address of a buffer of memory to send to the stream.
    \param[in] len  The number of bytes to send to the stream.

    \return Returns the actual number of bytes sent to the stream.  If not equal to the length specific, then the stream is full or unable to write for some reason.
    */
    virtual uint32_t    write(const void *mem,uint32_t len) = 0;

    /**
    \brief Reports the current stream location read aqccess.

    \return Returns the current stream read location.
    */
    virtual uint32_t    tellRead(void) const = 0;

    /**
    \brief Reports the current stream location for write access.

    \return Returns the current stream write location.
    */
    virtual uint32_t    tellWrite(void) const = 0;

    /**
    \brief  Causes any temporarily cached data to be flushed to the stream.
    */
    virtual void    flush(void) = 0;

    /**
    \brief  Close the stream.
    */
    virtual void close(void) {}

    void release(void)
    {
        delete this;
    }

    static NV_INLINE bool isBigEndian()
     {
       int32_t i = 1;
        return *(reinterpret_cast<char*>(&i))==0;
    }

    NV_INLINE void swap2Bytes(void* _data) const
    {
        char *data = static_cast<char *>(_data);
        char one_byte;
        one_byte = data[0]; data[0] = data[1]; data[1] = one_byte;
    }

    NV_INLINE void swap4Bytes(void* _data) const
    {
        char *data = static_cast<char *>(_data);
        char one_byte;
        one_byte = data[0]; data[0] = data[3]; data[3] = one_byte;
        one_byte = data[1]; data[1] = data[2]; data[2] = one_byte;
    }

    NV_INLINE void swap8Bytes(void *_data) const
    {
        char *data = static_cast<char *>(_data);
        char one_byte;
        one_byte = data[0]; data[0] = data[7]; data[7] = one_byte;
        one_byte = data[1]; data[1] = data[6]; data[6] = one_byte;
        one_byte = data[2]; data[2] = data[5]; data[5] = one_byte;
        one_byte = data[3]; data[3] = data[4]; data[4] = one_byte;
    }


    NV_INLINE void storeDword(uint32_t v)
    {
        if ( mEndianSwap )
            swap4Bytes(&v);

        write(&v,sizeof(v));
    }

    NV_INLINE void storeFloat(float v)
    {
        if ( mEndianSwap )
            swap4Bytes(&v);
        write(&v,sizeof(v));
    }

    NV_INLINE void storeDouble(double v)
    {
        if ( mEndianSwap )
            swap8Bytes(&v);
        write(&v,sizeof(v));
    }

    NV_INLINE  void storeByte(uint8_t b)
    {
        write(&b,sizeof(b));
    }

    NV_INLINE void storeWord(uint16_t w)
    {
        if ( mEndianSwap )
            swap2Bytes(&w);
        write(&w,sizeof(w));
    }

    uint8_t readByte(void) 
    {
        uint8_t v=0;
        read(&v,sizeof(v));
        return v;
    }

    uint16_t readWord(void) 
    {
        uint16_t v=0;
        read(&v,sizeof(v));
        if ( mEndianSwap )
            swap2Bytes(&v);
        return v;
    }

    uint32_t readDword(void) 
    {
        uint32_t v=0;
        read(&v,sizeof(v));
        if ( mEndianSwap )
            swap4Bytes(&v);
        return v;
    }

    float readFloat(void) 
    {
        float v=0;
        read(&v,sizeof(v));
        if ( mEndianSwap )
            swap4Bytes(&v);
        return v;
    }

    double readDouble(void) 
    {
        double v=0;
        read(&v,sizeof(v));
        if ( mEndianSwap )
            swap8Bytes(&v);
        return v;
    }

private:
    bool        mEndianSwap;    // whether or not the endian should be swapped on the current platform
    EndianMode  mEndianMode;    // the current endian mode behavior for the stream
};

NV_POP_PACK

} // end of namespace

using namespace general_NvIOStream2;

namespace general_NvIOStream = general_NvIOStream2;

} // end of namespace

#endif // NV_FILE_BUF_H

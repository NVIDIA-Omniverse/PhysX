#[unsafe(no_mangle)]
pub extern "C" fn __wasi_proc_exit(_code: u32) -> ! {
    std::process::abort()
}

#[unsafe(no_mangle)]
pub extern "C" fn __wasi_fd_close(_fd: u32) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn __wasi_fd_write(
    _fd: u32,
    _iovs: *const core::ffi::c_void,
    _iovs_len: usize,
    nwritten: *mut usize,
) -> u16 {
    if !nwritten.is_null() {
        // The packaged wasm backend does not rely on stdio output.
        unsafe { *nwritten = 0 };
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn __wasi_fd_read(
    _fd: u32,
    _iovs: *const core::ffi::c_void,
    _iovs_len: usize,
    nread: *mut usize,
) -> u16 {
    if !nread.is_null() {
        unsafe { *nread = 0 };
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn __wasi_environ_sizes_get(count: *mut usize, bytes: *mut usize) -> u16 {
    if !count.is_null() {
        unsafe { *count = 0 };
    }
    if !bytes.is_null() {
        unsafe { *bytes = 0 };
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn __wasi_environ_get(_environ: *mut *mut u8, _environ_buf: *mut u8) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn __wasi_fd_seek(_fd: u32, _offset: i64, _whence: u8, new_offset: *mut u64) -> u16 {
    if !new_offset.is_null() {
        unsafe { *new_offset = 0 };
    }
    0
}

//! Pure-Rust stubs for every libc symbol the Blast C++ backend (plus
//! its transitive libc++ dependency) references on
//! `wasm32-unknown-unknown`.
//!
//! ## Why
//!
//! `wasm32-unknown-unknown` ships no libc.  If we linked wasi-libc to
//! resolve C++ calls like `malloc` / `fwrite` / `abort`, the final
//! wasm would import from `wasi_snapshot_preview1` — and that in turn
//! causes wasm-bindgen to wrap every export in a `command_export`
//! shim that calls `__wasm_call_dtors → __funcs_on_exit →
//! __stdio_exit`.  Those walk wasi-libc's atexit table, which is
//! never initialised in library mode, so every
//! `__wbindgen_malloc` / `__wbindgen_free` call traps.
//!
//! Instead we provide our own `#[no_mangle] extern "C"` stubs here.
//! `malloc` / `free` / `realloc` forward to Rust's global allocator
//! (via a 16-byte size header).  Stdio, locale, wide-char, and
//! `strto*` routines are no-op stubs — the stress solver doesn't
//! actually use them at runtime, but libc++'s STL error paths still
//! reference them for fallback output.  Character-class helpers
//! implement the ASCII subset directly.
//!
//! The result: the final cdylib has **zero** `env.*` libc imports and
//! **zero** `wasi_snapshot_preview1.*` imports.  wasm-bindgen emits a
//! normal library module with no command_export wrappers.

use core::ffi::{c_char, c_int, c_void};
use std::alloc::{alloc, dealloc, realloc as rust_realloc, Layout};
use std::sync::Once;

// -----------------------------------------------------------------------------
// Allocator
// -----------------------------------------------------------------------------
//
// libc-style `malloc`/`free` don't carry a size parameter on `free`,
// but Rust's `GlobalAlloc` requires one.  We prepend a 16-byte header
// storing the user-requested size so `free`/`realloc` can reconstruct
// the original `Layout`.  16 bytes also keeps the returned pointer
// 16-aligned, which is stricter than what libc promises and matches
// what libc++ assumes for SSE vector types.

const HEADER_SIZE: usize = 16;
const ALIGN: usize = 16;

#[inline]
fn layout_for(user_size: usize) -> Layout {
    // +HEADER_SIZE so the pointer the user sees still has `user_size`
    // bytes available after the header.
    Layout::from_size_align(user_size + HEADER_SIZE, ALIGN).expect("layout")
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn malloc(size: usize) -> *mut c_void {
    if size == 0 {
        return core::ptr::null_mut();
    }
    let base = alloc(layout_for(size));
    if base.is_null() {
        return core::ptr::null_mut();
    }
    (base as *mut usize).write(size);
    base.add(HEADER_SIZE) as *mut c_void
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn free(ptr: *mut c_void) {
    if ptr.is_null() {
        return;
    }
    let base = (ptr as *mut u8).sub(HEADER_SIZE);
    let size = (base as *mut usize).read();
    dealloc(base, layout_for(size));
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn realloc(ptr: *mut c_void, new_size: usize) -> *mut c_void {
    if ptr.is_null() {
        return malloc(new_size);
    }
    if new_size == 0 {
        free(ptr);
        return core::ptr::null_mut();
    }
    let base = (ptr as *mut u8).sub(HEADER_SIZE);
    let old_size = (base as *mut usize).read();
    let new_base = rust_realloc(base, layout_for(old_size), new_size + HEADER_SIZE);
    if new_base.is_null() {
        return core::ptr::null_mut();
    }
    (new_base as *mut usize).write(new_size);
    new_base.add(HEADER_SIZE) as *mut c_void
}

/// POSIX `aligned_alloc(alignment, size)`.  libc++'s
/// `__libcpp_aligned_alloc` routes here from `operator new(size,
/// align_val_t)`.  Our `malloc` already returns 16-byte-aligned
/// pointers (via the header trick), which covers every alignment
/// the Blast backend actually requests on wasm — `NvcVec3`,
/// `NvAlignedAllocator`, and every `std::vector` allocation — as
/// long as the build passes `STRESS_SOLVER_FORCE_SCALAR` (no SSE
/// vector types).  A stricter request would land on the `unreachable`
/// branch and show up as a `wasm_fixture_instantiates_and_runs`
/// failure, which is the signal to extend this shim.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn aligned_alloc(alignment: usize, size: usize) -> *mut c_void {
    if alignment > ALIGN {
        core::arch::wasm32::unreachable();
    }
    malloc(size)
}

// -----------------------------------------------------------------------------
// Fatal exit
// -----------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub unsafe extern "C" fn abort() -> ! {
    // Emit a wasm `unreachable`, which traps the module.  Unlike
    // `std::process::abort`, this does not call any exit handlers.
    core::arch::wasm32::unreachable()
}

// -----------------------------------------------------------------------------
// C++ static dtors
// -----------------------------------------------------------------------------

// libc++ registers every `thread_local` and function-local `static`
// destructor via `__cxa_atexit`.  In library mode we never run them,
// so just report success and drop the registration on the floor.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __cxa_atexit(
    _func: *mut c_void,
    _arg: *mut c_void,
    _dso_handle: *mut c_void,
) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn __cxa_uncaught_exceptions() -> c_int {
    0
}

// -----------------------------------------------------------------------------
// String / memory helpers
// -----------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub unsafe extern "C" fn strcmp(a: *const c_char, b: *const c_char) -> c_int {
    let mut i = 0isize;
    loop {
        let ca = *a.offset(i) as u8;
        let cb = *b.offset(i) as u8;
        if ca != cb {
            return ca as c_int - cb as c_int;
        }
        if ca == 0 {
            return 0;
        }
        i += 1;
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn memchr(s: *const c_void, c: c_int, n: usize) -> *mut c_void {
    let p = s as *const u8;
    let target = c as u8;
    let mut i = 0usize;
    while i < n {
        if *p.add(i) == target {
            return p.add(i) as *mut c_void;
        }
        i += 1;
    }
    core::ptr::null_mut()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn strerror(_errnum: c_int) -> *const c_char {
    // Return a static empty C string.
    static EMPTY: [u8; 1] = [0];
    EMPTY.as_ptr() as *const c_char
}

// wchar_t on wasm32 is 32-bit.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wcslen(s: *const u32) -> usize {
    let mut i = 0usize;
    while *s.add(i) != 0 {
        i += 1;
    }
    i
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn wmemchr(s: *const u32, c: u32, n: usize) -> *mut u32 {
    let mut i = 0usize;
    while i < n {
        if *s.add(i) == c {
            return s.add(i) as *mut u32;
        }
        i += 1;
    }
    core::ptr::null_mut()
}

static INIT_CTYPE_TABLES: Once = Once::new();
static mut CTYPE_TOUPPER_TABLE: [c_int; 384] = [0; 384];
static mut CTYPE_TOLOWER_TABLE: [c_int; 384] = [0; 384];
static mut CTYPE_TOUPPER_PTR: *const c_int = core::ptr::null();
static mut CTYPE_TOLOWER_PTR: *const c_int = core::ptr::null();

#[inline]
fn ascii_toupper(byte: u8) -> c_int {
    if byte.is_ascii_lowercase() {
        (byte - b'a' + b'A') as c_int
    } else {
        byte as c_int
    }
}

#[inline]
fn ascii_tolower(byte: u8) -> c_int {
    if byte.is_ascii_uppercase() {
        (byte - b'A' + b'a') as c_int
    } else {
        byte as c_int
    }
}

fn init_ctype_tables() {
    INIT_CTYPE_TABLES.call_once(|| unsafe {
        for value in -128i32..=255 {
            let index = (value + 128) as usize;
            let mapped = if value == -1 {
                -1
            } else {
                let byte = value as u8;
                ascii_toupper(byte)
            };
            CTYPE_TOUPPER_TABLE[index] = mapped;
            CTYPE_TOLOWER_TABLE[index] = if value == -1 {
                -1
            } else {
                let byte = value as u8;
                ascii_tolower(byte)
            };
        }

        CTYPE_TOUPPER_PTR = core::ptr::addr_of!(CTYPE_TOUPPER_TABLE)
            .cast::<c_int>()
            .add(128);
        CTYPE_TOLOWER_PTR = core::ptr::addr_of!(CTYPE_TOLOWER_TABLE)
            .cast::<c_int>()
            .add(128);
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn __ctype_toupper_loc() -> *const *const c_int {
    init_ctype_tables();
    &raw const CTYPE_TOUPPER_PTR
}

#[unsafe(no_mangle)]
pub extern "C" fn __ctype_tolower_loc() -> *const *const c_int {
    init_ctype_tables();
    &raw const CTYPE_TOLOWER_PTR
}

// -----------------------------------------------------------------------------
// Stdio (no-op)
// -----------------------------------------------------------------------------
//
// The stress solver only uses stdio for error / profiling output,
// which we don't need.  Each stub succeeds silently.

#[unsafe(no_mangle)]
pub unsafe extern "C" fn fwrite(
    _ptr: *const c_void,
    _size: usize,
    n: usize,
    _stream: *mut c_void,
) -> usize {
    // Pretend the whole block was written.
    n
}

/// libc++'s `std::__put_character_sequence` (and by extension
/// `operator<<(basic_ostream&, const char*)`) reaches for `fputc`
/// on its slow path when the stream's `sputn` isn't available.
/// We're not interested in the output — just pretend we wrote
/// the character.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fputc(c: c_int, _stream: *mut c_void) -> c_int {
    c
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn fflush(_stream: *mut c_void) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn fprintf(
    _stream: *mut c_void,
    _fmt: *const c_char,
    _arg: *mut c_void,
) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn fiprintf(
    _stream: *mut c_void,
    _fmt: *const c_char,
    _arg: *mut c_void,
) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vfprintf(
    _stream: *mut c_void,
    _fmt: *const c_char,
    _ap: *mut c_void,
) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn snprintf(
    buf: *mut c_char,
    n: usize,
    _fmt: *const c_char,
    _arg: *mut c_void,
) -> c_int {
    if !buf.is_null() && n > 0 {
        *buf = 0;
    }
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vsnprintf(
    buf: *mut c_char,
    n: usize,
    _fmt: *const c_char,
    _ap: *mut c_void,
) -> c_int {
    if !buf.is_null() && n > 0 {
        *buf = 0;
    }
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn sscanf(
    _s: *const c_char,
    _fmt: *const c_char,
    _arg: *mut c_void,
) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vsscanf(
    _s: *const c_char,
    _fmt: *const c_char,
    _ap: *mut c_void,
) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vasprintf(
    _strp: *mut *mut c_char,
    _fmt: *const c_char,
    _ap: *mut c_void,
) -> c_int {
    -1
}

static mut ERRNO_VALUE: c_int = 0;

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __errno_location() -> *mut c_int {
    &raw mut ERRNO_VALUE
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn getc(_stream: *mut c_void) -> c_int {
    -1
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ungetc(_c: c_int, _stream: *mut c_void) -> c_int {
    -1
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn getwc(_stream: *mut c_void) -> c_int {
    -1
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ungetwc(_c: c_int, _stream: *mut c_void) -> c_int {
    -1
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn pthread_mutex_lock(_mutex: *mut c_void) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn pthread_mutex_unlock(_mutex: *mut c_void) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn pthread_cond_broadcast(_cond: *mut c_void) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn pthread_cond_wait(_cond: *mut c_void, _mutex: *mut c_void) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn fputwc(_c: c_int, _stream: *mut c_void) -> c_int {
    -1
}

// -----------------------------------------------------------------------------
// Numeric parsing (strto*)
// -----------------------------------------------------------------------------
//
// The Blast backend never invokes these at runtime — they're reachable
// only through libc++'s locale / iostream fallback paths.  Returning
// zero is safe: any code path that actually depended on the parsed
// value would have to go through `operator<<` first, which is
// similarly stubbed.

#[unsafe(no_mangle)]
pub unsafe extern "C" fn strtoll(
    _nptr: *const c_char,
    endptr: *mut *mut c_char,
    _base: c_int,
) -> i64 {
    if !endptr.is_null() {
        *endptr = core::ptr::null_mut();
    }
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn strtoull(
    _nptr: *const c_char,
    endptr: *mut *mut c_char,
    _base: c_int,
) -> u64 {
    if !endptr.is_null() {
        *endptr = core::ptr::null_mut();
    }
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn strtof_l(
    _nptr: *const c_char,
    endptr: *mut *mut c_char,
    _loc: *mut c_void,
) -> f32 {
    if !endptr.is_null() {
        *endptr = core::ptr::null_mut();
    }
    0.0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn strtod_l(
    _nptr: *const c_char,
    endptr: *mut *mut c_char,
    _loc: *mut c_void,
) -> f64 {
    if !endptr.is_null() {
        *endptr = core::ptr::null_mut();
    }
    0.0
}

/// `long double strtold_l(...)` returns via an sret pointer on wasm32
/// because `long double` is larger than a scalar result can carry.
/// The caller has already reserved 16 bytes of stack; zero them out.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn strtold_l(
    result: *mut u8,
    _nptr: *const c_char,
    endptr: *mut *mut c_char,
    _loc: *mut c_void,
) {
    if !result.is_null() {
        core::ptr::write_bytes(result, 0, 16);
    }
    if !endptr.is_null() {
        *endptr = core::ptr::null_mut();
    }
}

// -----------------------------------------------------------------------------
// Locale (no-op)
// -----------------------------------------------------------------------------
//
// libc++ creates one `locale_t` for the "C" locale at startup.  Any
// non-null pointer works as a sentinel — we never dereference it.

const FAKE_LOCALE: *mut c_void = 1 as *mut c_void;

#[unsafe(no_mangle)]
pub unsafe extern "C" fn newlocale(
    _category_mask: c_int,
    _locale: *const c_char,
    _base: *mut c_void,
) -> *mut c_void {
    FAKE_LOCALE
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn uselocale(_loc: *mut c_void) -> *mut c_void {
    FAKE_LOCALE
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn freelocale(_loc: *mut c_void) {}

// -----------------------------------------------------------------------------
// Character classes
// -----------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub unsafe extern "C" fn toupper(c: c_int) -> c_int {
    if (b'a' as c_int..=b'z' as c_int).contains(&c) {
        c - 32
    } else {
        c
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn tolower(c: c_int) -> c_int {
    if (b'A' as c_int..=b'Z' as c_int).contains(&c) {
        c + 32
    } else {
        c
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn isdigit_l(c: c_int, _loc: *mut c_void) -> c_int {
    (b'0' as c_int..=b'9' as c_int).contains(&c) as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn isxdigit_l(c: c_int, _loc: *mut c_void) -> c_int {
    let ok = (b'0' as c_int..=b'9' as c_int).contains(&c)
        || (b'a' as c_int..=b'f' as c_int).contains(&c)
        || (b'A' as c_int..=b'F' as c_int).contains(&c);
    ok as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn islower_l(c: c_int, _loc: *mut c_void) -> c_int {
    (b'a' as c_int..=b'z' as c_int).contains(&c) as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn isupper_l(c: c_int, _loc: *mut c_void) -> c_int {
    (b'A' as c_int..=b'Z' as c_int).contains(&c) as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn iswlower_l(c: c_int, _loc: *mut c_void) -> c_int {
    (b'a' as c_int..=b'z' as c_int).contains(&c) as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __ctype_get_mb_cur_max() -> usize {
    // ASCII / UTF-8: one byte per character in the "C" locale.
    1
}

// -----------------------------------------------------------------------------
// Multibyte / wide-char (no-op)
// -----------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mbsrtowcs(
    _dst: *mut u32,
    _src: *mut *const c_char,
    _len: usize,
    _ps: *mut c_void,
) -> usize {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mbsnrtowcs(
    _dst: *mut u32,
    _src: *mut *const c_char,
    _nms: usize,
    _len: usize,
    _ps: *mut c_void,
) -> usize {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn wcsnrtombs(
    _dst: *mut c_char,
    _src: *mut *const u32,
    _nwc: usize,
    _len: usize,
    _ps: *mut c_void,
) -> usize {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn wcrtomb(_s: *mut c_char, _wc: u32, _ps: *mut c_void) -> usize {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mbrtowc(
    _pwc: *mut u32,
    _s: *const c_char,
    _n: usize,
    _ps: *mut c_void,
) -> usize {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mbtowc(_pwc: *mut u32, _s: *const c_char, _n: usize) -> c_int {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mbrlen(_s: *const c_char, _n: usize, _ps: *mut c_void) -> usize {
    0
}

// -----------------------------------------------------------------------------
// Time formatting (no-op)
// -----------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub unsafe extern "C" fn strftime_l(
    buf: *mut c_char,
    n: usize,
    _fmt: *const c_char,
    _tm: *mut c_void,
    _loc: *mut c_void,
) -> usize {
    if !buf.is_null() && n > 0 {
        *buf = 0;
    }
    0
}

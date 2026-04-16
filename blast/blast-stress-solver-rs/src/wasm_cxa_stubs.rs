//! Trap-only stubs for the tiny surface of the Itanium C++ ABI that
//! libc++ still references even when compiled with `-fno-exceptions`.
//!
//! Any call to these functions indicates the C++ backend hit an
//! unexpected error path (e.g. out-of-memory inside `std::vector`).
//! We trap immediately so the browser surfaces a WebAssembly
//! RuntimeError with a clear stack frame rather than returning
//! nonsense to the caller.

/// libc++ calls `__cxa_allocate_exception` before constructing the
/// exception object.  With `-fno-exceptions` the call is only emitted
/// for unwind-friendly STL helpers; none of them are on the steady-
/// state hot path, so aborting is fine.
#[unsafe(no_mangle)]
pub extern "C" fn __cxa_allocate_exception(_size: usize) -> *mut u8 {
    core::arch::wasm32::unreachable()
}

/// Paired with `__cxa_allocate_exception`; libc++ calls this after
/// filling in the exception object to start unwinding.  We never reach
/// here in practice because `__cxa_allocate_exception` traps first,
/// but linkers still want the symbol resolved.
#[unsafe(no_mangle)]
pub extern "C" fn __cxa_throw(
    _thrown_object: *mut u8,
    _tinfo: *mut u8,
    _dest: *mut u8,
) -> ! {
    core::arch::wasm32::unreachable()
}

// `__funcs_on_exit` / `__stdio_exit` do not appear in the final wasm
// at all: because we never link wasi-libc and our
// `wasm_runtime_shims` module covers every libc reference, the
// cdylib has zero `wasi_snapshot_preview1` imports, so wasm-bindgen
// emits a normal library module rather than one wrapped in
// `command_export` shims.  Nothing ever calls the exit hooks, so we
// don't need to shadow them.

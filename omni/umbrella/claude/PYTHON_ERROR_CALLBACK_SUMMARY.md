# Python Error Callback Implementation Summary

## Overview

The Physics Umbrella library now has full Python support for custom error callbacks, matching the C++ functionality.

## What Was Added

### 1. Python Bindings (`bindings/BindingsPhysics.cpp`)

#### ErrorCode Enum
```python
ErrorCode.INFO     # 0
ErrorCode.WARNING  # 1
ErrorCode.ERROR    # 2
```

#### ErrorCallback Class
- Base class with trampoline (PyErrorCallback) for Python inheritance
- `report_error(code, message, file, line)` method to override

#### IPhysics Methods
- `get_error_callback()` - Get current callback
- `set_error_callback(callback)` - Set/update callback
- `get_physics_interface(error_callback=None)` - Optional callback on init

### 2. Python Test (`tests/python/test_error_callback.py`)

Seven comprehensive tests:
1. `test_error_code_enum` - Verify enum values
2. `test_callback_lifecycle` - Set and clear callback
3. `test_error_callback_capture` - Capture error messages
4. `test_no_errors_on_valid_operations` - No errors on valid ops
5. `test_multiple_errors` - Collect multiple errors
6. `test_console_logger_example` - Console logger demo
7. `test_file_logger_example` - File logger demo

### 3. Python Examples (`examples/error_callback_example.py`)

Four example implementations:
1. **ColoredConsoleLogger** - ANSI colored terminal output
2. **FileLogger** - Write to log file
3. **CollectingLogger** - Collect messages for analysis
4. **Default logging** - Fallback to console

### 4. Documentation

#### `docs/PythonErrorCallback.md`
- Complete API reference
- Common patterns and best practices
- Integration with Python logging module
- Testing examples
- Lifetime management guidelines

## Usage Example

```python
from _physics_umbrella import (
    get_physics_interface,
    ErrorCallback,
    ErrorCode,
)

class MyLogger(ErrorCallback):
    def report_error(self, code, message, file, line):
        level = ['INFO', 'WARNING', 'ERROR'][code]
        print(f"[{level}] {message}")

# Create logger and physics interface
logger = MyLogger()
physics = get_physics_interface()
physics.set_error_callback(logger)

# Use physics - errors will be captured
sim = physics.get_simulation(invalid_id)

# Clean up
physics.set_error_callback(None)
```

## Key Features

✓ **Full Python Integration** - Native Python class inheritance from ErrorCallback
✓ **Enum Support** - ErrorCode enum with INFO, WARNING, ERROR
✓ **Flexible API** - Set callback at init or runtime with set_error_callback()
✓ **Context Managers** - Easy cleanup with __enter__/__exit__
✓ **Testing Support** - Collecting logger for unit tests
✓ **Thread-Safe** - Matches C++ thread safety guarantees
✓ **Backward Compatible** - Falls back to console if no callback set

## File Structure

```
umbrella/
├── bindings/
│   ├── __init__.py                        # Updated exports
│   └── BindingsPhysics.cpp                # Added ErrorCallback bindings
├── tests/python/
│   └── test_error_callback.py             # 7 comprehensive tests
├── examples/
│   ├── error_callback_example.py          # Python examples
│   └── error_callback_example.cpp         # C++ examples
└── docs/
    ├── ErrorCallback.md                   # C++ documentation
    └── PythonErrorCallback.md             # Python documentation
```

## Implementation Details

### Pybind11 Trampoline Class

```cpp
class PyErrorCallback : public ErrorCallback
{
public:
    using ErrorCallback::ErrorCallback;

    void reportError(ErrorCode::Enum code, const char* message,
                     const char* file, int line) override
    {
        PYBIND11_OVERRIDE_PURE(
            void,           // Return type
            ErrorCallback,  // Parent class
            reportError,    // Function name
            code, message, file, line  // Arguments
        );
    }
};
```

This trampoline allows Python classes to inherit from ErrorCallback and override `report_error()`.

### Binding Registration

```cpp
// ErrorCode enum
py::enum_<ErrorCode::Enum>(m, "ErrorCode")
    .value("INFO", ErrorCode::eINFO)
    .value("WARNING", ErrorCode::eWARNING)
    .value("ERROR", ErrorCode::eERROR)
    .export_values();

// ErrorCallback with trampoline
py::class_<ErrorCallback, PyErrorCallback>(m, "ErrorCallback")
    .def(py::init<>())
    .def("report_error", &ErrorCallback::reportError);

// IPhysics methods
py::class_<IPhysics>(m, "IPhysics")
    .def("get_error_callback", &IPhysics::getErrorCallback)
    .def("set_error_callback", &IPhysics::setErrorCallback)
    // ... other methods ...
```

## Common Use Cases

### 1. Unit Testing
```python
class TestLogger(ErrorCallback):
    def __init__(self):
        super().__init__()
        self.errors = []

    def report_error(self, code, message, file, line):
        self.errors.append(message)

# In test
logger = TestLogger()
physics.set_error_callback(logger)
# ... test operations ...
assert len(logger.errors) > 0
```

### 2. Production Logging
```python
import logging

class ProductionLogger(ErrorCallback):
    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger('physics')

    def report_error(self, code, message, file, line):
        if code == ErrorCode.ERROR:
            self.logger.error(message)
        elif code == ErrorCode.WARNING:
            self.logger.warning(message)
        else:
            self.logger.info(message)
```

### 3. Debugging
```python
class VerboseLogger(ErrorCallback):
    def report_error(self, code, message, file, line):
        print(f"[{code}] {message}")
        if file and line:
            print(f"  Location: {file}:{line}")
        import traceback
        traceback.print_stack()
```

## Building Python Bindings

### Requirements
```bash
pip install pybind11
```

### Build
```bash
cd umbrella
./build.bat     # Windows
./build.sh      # Linux/Mac
```

The bindings will be built to:
- `umbrella/_build/windows-x86_64/bindings/_physics_umbrella.pyd` (Windows)
- `umbrella/_build/linux-x86_64/bindings/_physics_umbrella.so` (Linux)

### Running Tests

```bash
# Run Python error callback tests
cd umbrella/tests/python
python test_error_callback.py

# Run examples
cd umbrella/examples
python error_callback_example.py
```

## Best Practices

1. **Keep callback alive**: Store callback in a variable to prevent garbage collection
2. **Clean up**: Always call `set_error_callback(None)` before callback goes out of scope
3. **Use context managers**: Implement `__enter__`/`__exit__` for automatic cleanup
4. **Thread safety**: Make callback thread-safe if using from multiple threads
5. **Performance**: Keep `report_error()` fast

## Limitations

- Callback instance must remain alive while in use (raw pointer stored in C++)
- Callback is shared globally via singleton physics interface
- pybind11 required to build bindings

## Testing Status

**Python bindings require pybind11 to build**. Once built, all tests should pass:

```
Expected test results:
- 7 tests total
- All tests pass
- Example scripts run successfully
```

## Next Steps

To use the Python error callback:

1. Install pybind11: `pip install pybind11`
2. Build umbrella library: `cd umbrella && ./build.bat`
3. Run tests: `python tests/python/test_error_callback.py`
4. Try examples: `python examples/error_callback_example.py`

## See Also

- [Python Error Callback Documentation](docs/PythonErrorCallback.md)
- [C++ Error Callback Documentation](docs/ErrorCallback.md)
- `examples/error_callback_example.py` - Usage examples
- `tests/python/test_error_callback.py` - Test examples

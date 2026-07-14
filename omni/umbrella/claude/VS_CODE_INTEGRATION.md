# VS Code Integration - Summary

The Physics Umbrella project is now fully integrated with VS Code. This document summarizes what was added and how to get started.

## What Was Added

### 1. VS Code Configuration Files

Located in `r:\src\physics\.vscode\`:

- **tasks.json** - Build and test tasks
- **launch.json** - Debug configurations
- **settings.json** - Python environment and workspace settings
- **extensions.json** - Recommended VS Code extensions

### 2. Documentation

- **omni/umbrella/VSCODE.md** - Comprehensive guide (10+ pages)
- **omni/umbrella/.vscode/README.md** - Quick reference card

### 3. Workspace Integration

The umbrella project is already included in `physics.code-workspace` as a folder, so it's ready to use.

## Quick Start

### Step 1: Open the Workspace

```bash
code r:\src\physics\physics.code-workspace
```

### Step 2: Reload VS Code Window

After opening, reload the window to ensure configurations are loaded:
- Press `Ctrl+Shift+P`
- Type "Developer: Reload Window"
- Press Enter

### Step 3: Install Recommended Extensions

When prompted, click "Install All" to install:
- C++ tools
- Python tools
- Debugpy
- CMake tools

### Step 3: Setup Python Environment

1. Press `Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Select "Umbrella: Setup Python Environment"

This runs:
```bash
cd omni/umbrella/tests/python
../../repo.bat uv sync
../../repo.bat uv run python setup_paths.py
```

### Step 4: Build

Press `Ctrl+Shift+B` and select "Umbrella: Build (Debug)"

### Step 5: Run Tests

1. Press `Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Select "Umbrella: Run All Python Tests"

## Available Tasks

### Build Tasks
- **Umbrella: Build (Debug)** - Build debug configuration
- **Umbrella: Build (Release)** - Build release configuration
- **Umbrella: Build (All)** - Build both configurations
- **Umbrella: Clean** - Clean build artifacts

### Test Tasks
- **Umbrella: Setup Python Environment** - Initialize uv environment
- **Umbrella: Run All Python Tests** - Run all 23 Python tests
- **Umbrella: Run Current Python Test** - Run the open test file
- **Umbrella: Run C++ Tests (Debug)** - Run C++ unit tests (debug)
- **Umbrella: Run C++ Tests (Release)** - Run C++ unit tests (release)

### Compound Tasks
- **Umbrella: Build and Test (Debug)** - Full debug build + all tests
- **Umbrella: Build and Test (Release)** - Full release build + all tests

## Available Debug Configurations

Press `F5` or use the Debug view to select:

### Python Debugging
1. **Umbrella: Debug Current Python Test** - Debug the open test file
2. **Umbrella: Debug All Python Tests** - Debug entire test suite
3. **Umbrella: Debug Python Test with uv** - Debug using uv environment

### C++ Debugging
4. **Umbrella: Debug C++ Tests (Debug)** - Debug C++ unit tests
5. **Umbrella: Debug C++ Tests (Release)** - Debug release C++ tests
6. **Umbrella: Debug Python Bindings (C++)** - Debug C++ code called from Python

### Process Debugging
7. **Umbrella: Attach to Process** - Attach debugger to running process

## Key Features

### ✓ Build Integration
- One-click builds with `Ctrl+Shift+B`
- Automatic problem detection and navigation
- Support for debug and release configurations

### ✓ Test Integration
- Run tests from Command Palette
- Run individual or all tests
- Test results in Terminal panel

### ✓ Python Test Discovery
- Tests auto-discovered in Testing view
- Click to run individual tests
- Green/red indicators for pass/fail

### ✓ Debugging Support
- **Python tests**: Set breakpoints, step through code
- **C++ tests**: Full C++ debugging support
- **Mixed debugging**: Debug Python → C++ bindings

### ✓ IntelliSense
- Auto-completion for `_physics_umbrella` imports
- Hover documentation
- Go to definition (F12)

### ✓ Terminal Integration
- PYTHONPATH automatically set
- Right-click folder → "Open in Integrated Terminal"

## File Structure

```
r:\src\physics\
├── physics.code-workspace      # Multi-root workspace
└── omni\umbrella\
    ├── VSCODE.md              # Full documentation (comprehensive guide)
    ├── VS_CODE_INTEGRATION.md # This summary file
    ├── .vscode\               # ← VS Code configurations are here!
    │   ├── tasks.json         # Build and test tasks
    │   ├── launch.json        # Debug configurations
    │   ├── settings.json      # Umbrella-specific settings
    │   ├── extensions.json    # Recommended extensions
    │   └── README.md          # Quick reference card
    ├── bindings\
    │   ├── pyproject.toml     # Python package config
    │   └── ...
    └── tests\python\
        ├── pyproject.toml     # Test environment config
        ├── setup_paths.py     # Path configuration script
        └── test_*.py          # Test files
```

**Important**: The VS Code configurations are in `omni/umbrella/.vscode/`, not at the workspace root. This is the correct location for multi-root workspaces.

## Common Workflows

### Development Workflow

1. **Edit code** in VS Code
2. **Build**: Press `Ctrl+Shift+B`
3. **Run tests**: Task → "Umbrella: Run All Python Tests"
4. **Debug failures**: Open test file → `F5`

### Testing Workflow

1. **Open test file** (e.g., `test_simulation_interface.py`)
2. **Set breakpoints** (click in gutter)
3. **Press F5** → Select "Umbrella: Debug Current Python Test"
4. **Step through code** with `F10` (step over) and `F11` (step into)

### C++ Debugging Workflow

1. **Open C++ source** (e.g., `BindingsPhysics.cpp`)
2. **Set breakpoints**
3. **Select**: "Umbrella: Debug Python Bindings (C++)"
4. **Press F5** → Runs Python but debugs C++ code

### Build and Test Workflow

1. **Run task**: "Umbrella: Build and Test (Debug)"
2. Automatically:
   - Builds debug configuration
   - Runs all Python tests
   - Runs C++ tests
3. **Review results** in Terminal

## Keyboard Shortcuts Reference

| Shortcut | Action |
|----------|--------|
| `Ctrl+Shift+B` | Build (default task) |
| `Ctrl+Shift+P` | Command Palette |
| `F5` | Start debugging |
| `Ctrl+F5` | Run without debugging |
| `F9` | Toggle breakpoint |
| `F10` | Step over |
| `F11` | Step into |
| `Shift+F11` | Step out |
| `Ctrl+Shift+M` | Show Problems panel |
| `Ctrl+`` | Toggle terminal |

## Testing the Integration

To verify everything works:

```bash
# 1. Open workspace
code r:\src\physics\physics.code-workspace

# 2. Install extensions when prompted

# 3. Run setup task
# Ctrl+Shift+P → "Tasks: Run Task" → "Umbrella: Setup Python Environment"

# 4. Build
# Ctrl+Shift+B → "Umbrella: Build (Debug)"

# 5. Run tests
# Ctrl+Shift+P → "Tasks: Run Task" → "Umbrella: Run All Python Tests"
```

Expected output: All 23 tests pass ✓

## Troubleshooting

### Issue: "Tasks or Debug Configurations not showing"
**Solution**:
1. Ensure you opened `physics.code-workspace` (not just the umbrella folder)
2. **Reload VS Code**: Press `Ctrl+Shift+P` → "Developer: Reload Window"
3. Check that `omni/umbrella/.vscode/` contains `tasks.json` and `launch.json`

### Issue: "ModuleNotFoundError: _physics_umbrella"
**Solution**: Run "Umbrella: Setup Python Environment" task.

### Issue: "Build fails - command not found"
**Solution**: Check that build.bat/build.sh exists in umbrella folder.

### Issue: "Debugger doesn't stop at breakpoints"
**Solution**:
- Ensure debug build was used
- Check that `justMyCode: false` is set (for Python)
- Verify program path in launch.json

### Issue: "IntelliSense not working"
**Solution**:
1. Select Python interpreter: `Ctrl+Shift+P` → "Python: Select Interpreter"
2. Choose: `omni/umbrella/tests/python/.venv/Scripts/python.exe`
3. Reload window: `Ctrl+Shift+P` → "Developer: Reload Window"

## Next Steps

1. **Read the full guide**: [VSCODE.md](VSCODE.md) - Comprehensive 10+ page documentation
2. **Quick reference**: [.vscode/README.md](.vscode/README.md) - One-page cheat sheet
3. **Customize**: Edit `.vscode/tasks.json` and `.vscode/launch.json` for custom configurations

## Related Documentation

- [tests/python/README.md](tests/python/README.md) - Python testing guide
- [VSCODE.md](VSCODE.md) - Full VS Code guide (main documentation)
- [VS Code Python Debugging](https://code.visualstudio.com/docs/python/debugging)
- [VS Code C++ Debugging](https://code.visualstudio.com/docs/cpp/cpp-debug)

## Summary

The umbrella project now has:

✅ **14 tasks** for building and testing
✅ **7 debug configurations** for Python, C++, and mixed debugging
✅ **Python environment integration** with uv
✅ **Test discovery** and execution
✅ **IntelliSense** for Python bindings
✅ **Problem matching** for build errors
✅ **Comprehensive documentation** (3 guides)

Everything is ready to use - just open the workspace and start coding! 🚀

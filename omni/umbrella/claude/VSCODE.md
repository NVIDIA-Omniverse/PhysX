# VS Code Integration for Umbrella

This guide explains how to use VS Code for building, testing, and debugging the Physics Umbrella framework.

## Setup

### 1. Open the Workspace

Open the physics workspace in VS Code:
```bash
code r:\src\physics\physics.code-workspace
```

The workspace includes the umbrella folder along with other physics projects.

### 2. Reload Window (Important!)

After opening the workspace for the first time, reload the window to ensure all configurations are loaded:

1. Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (Mac)
2. Type "Developer: Reload Window"
3. Press Enter

This ensures VS Code picks up the umbrella-specific tasks and debug configurations from `omni/umbrella/.vscode/`.

### 3. Install Recommended Extensions

When you open the workspace, VS Code will prompt you to install recommended extensions. Click "Install All" or install them manually:

- **ms-vscode.cpptools** - C++ IntelliSense and debugging
- **ms-python.python** - Python language support
- **ms-python.debugpy** - Python debugging
- **ms-vscode.cmake-tools** - CMake integration

### 3. Setup Python Environment (One Time)

Before running Python tests, set up the uv environment:

1. Open Command Palette (Ctrl+Shift+P / Cmd+Shift+P)
2. Run: "Tasks: Run Task"
3. Select: "Umbrella: Setup Python Environment"

Or run from terminal:
```bash
cd omni/umbrella/tests/python
../../repo.bat uv sync
../../repo.bat uv run python setup_paths.py
```

## Building

### Build Tasks

Access build tasks via Command Palette → "Tasks: Run Task" or use keyboard shortcuts:

| Task | Description | Shortcut |
|------|-------------|----------|
| **Umbrella: Build (Debug)** | Build debug configuration | Ctrl+Shift+B |
| **Umbrella: Build (Release)** | Build release configuration | - |
| **Umbrella: Build (All)** | Build both debug and release | - |
| **Umbrella: Clean** | Clean build artifacts | - |

### Quick Build

1. Press `Ctrl+Shift+B` (Windows/Linux) or `Cmd+Shift+B` (Mac)
2. Select a build task from the list
3. Build output appears in the Terminal panel

## Testing

### Python Tests

#### Run Tests via Tasks

| Task | Description |
|------|-------------|
| **Umbrella: Run All Python Tests** | Run all test modules |
| **Umbrella: Run Current Python Test** | Run the currently open test file |

**Steps:**
1. Open a test file (e.g., `test_umbrella_basic.py`)
2. Command Palette → "Tasks: Run Task"
3. Select "Umbrella: Run Current Python Test"

#### Run Tests via Test Explorer

1. Open the Testing view (beaker icon in Activity Bar)
2. Tests are auto-discovered in `omni/umbrella/tests/python/`
3. Click the play button next to any test to run it
4. Green checkmarks indicate passing tests

#### Run Tests from Terminal

```bash
cd omni/umbrella/tests/python
../../repo.bat uv run python test_umbrella_basic.py
../../repo.bat uv run python run_all_tests.py
```

### C++ Tests

| Task | Description |
|------|-------------|
| **Umbrella: Run C++ Tests (Debug)** | Run C++ unit tests (debug build) |
| **Umbrella: Run C++ Tests (Release)** | Run C++ unit tests (release build) |

### Combined Build + Test

| Task | Description |
|------|-------------|
| **Umbrella: Build and Test (Debug)** | Build debug + run all tests |
| **Umbrella: Build and Test (Release)** | Build release + run all tests |

## Debugging

### Debug Python Tests

#### Option 1: Debug Current Test File

1. Open a test file (e.g., `test_simulation_interface.py`)
2. Set breakpoints by clicking in the gutter
3. Press `F5` or open Debug view and select "Umbrella: Debug Current Python Test"
4. Debugger stops at breakpoints

#### Option 2: Debug All Tests

1. Open Debug view (bug icon in Activity Bar)
2. Select "Umbrella: Debug All Python Tests"
3. Press `F5`

#### Option 3: Debug with uv Environment

1. Select "Umbrella: Debug Python Test with uv"
2. Uses the exact uv-managed Python environment
3. Press `F5`

### Debug C++ Tests

1. Open Debug view
2. Select "Umbrella: Debug C++ Tests (Debug)"
3. Set breakpoints in C++ code
4. Press `F5`

The task will automatically build before debugging.

### Debug Python Bindings (C++ Code)

To debug the C++ code behind the Python bindings:

1. Open Debug view
2. Select "Umbrella: Debug Python Bindings (C++)"
3. Set breakpoints in C++ binding code (e.g., `BindingsPhysics.cpp`)
4. Press `F5`

This launches Python but attaches the C++ debugger, allowing you to step through both Python and C++ code.

### Debug Running Process

1. Open Debug view
2. Select "Umbrella: Attach to Process"
3. Select the process to attach to (e.g., `python.exe`, `umbrella_tests.exe`)

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+Shift+B` | Run default build task |
| `F5` | Start debugging (current configuration) |
| `Ctrl+F5` | Run without debugging |
| `F9` | Toggle breakpoint |
| `F10` | Step over |
| `F11` | Step into |
| `Shift+F11` | Step out |
| `Ctrl+Shift+P` | Open Command Palette |

## Workspace Structure

The physics workspace includes multiple folders:

```
physics.code-workspace
├── physx/          - PhysX SDK
├── ovphysx/        - Omniverse PhysX runtime
├── umbrella/       - Physics Umbrella (this project)
├── ovexts/         - Omniverse extensions
└── ...
```

Each folder appears as a separate root in the Explorer, allowing you to work on multiple projects simultaneously.

## Tips and Tricks

### Quick Test Execution

- Right-click on a test file → "Run Python File in Terminal"
- Uses the uv environment automatically if configured

### Integrated Terminal

Open a terminal in the umbrella folder:
1. Right-click on `umbrella` in Explorer
2. Select "Open in Integrated Terminal"
3. Terminal automatically sets PYTHONPATH

### Python IntelliSense

The workspace is configured to recognize the bindings module:
- Auto-completion works for `_physics_umbrella` imports
- Hover over functions to see docstrings
- Go to definition (F12) works for Python code

### Problem Matching

Build errors and warnings appear in the Problems panel (Ctrl+Shift+M):
- Click on an error to jump to the file/line
- Filter by severity (errors, warnings, info)

### Test Discovery

Python tests are auto-discovered when you:
1. Open the Testing view
2. Tests matching `test_*.py` in `tests/python/` are found
3. Click refresh icon to re-scan

### Debugging Tips

**Python Debugging:**
- Use `justMyCode: false` to step into library code
- Check the Call Stack panel to see execution path
- Use Debug Console to evaluate expressions

**C++ Debugging:**
- Enable "Just My Code" in settings for faster debugging
- Use Watch expressions to monitor variables
- Breakpoints support conditions and hit counts

**Mixed Python/C++ Debugging:**
- Set breakpoints in both Python and C++ files
- Debugger automatically switches between languages
- View both Python and C++ stack frames

## Troubleshooting

### Tests Not Running

**Problem:** "ModuleNotFoundError: No module named '_physics_umbrella'"

**Solution:** Run the setup task:
```bash
cd omni/umbrella/tests/python
../../repo.bat uv sync
../../repo.bat uv run python setup_paths.py
```

### Build Errors

**Problem:** Build task fails with "command not found"

**Solution:** Ensure you're running from the correct folder:
- Tasks are configured with `"cwd": "${workspaceFolder:umbrella}"`
- If you opened a single folder instead of the workspace, paths may be wrong

### Debugger Not Stopping

**Problem:** Breakpoints show as gray/unverified

**Solution:**
1. Ensure you built with debug symbols (use Debug configuration)
2. Check that the program path in launch.json matches the built executable
3. For Python: Ensure `justMyCode: false` is set

### IntelliSense Not Working

**Problem:** Auto-completion doesn't work for `_physics_umbrella`

**Solution:**
1. Ensure Python extension is installed
2. Select the correct Python interpreter (from `.venv`)
3. Reload window: Command Palette → "Developer: Reload Window"

### DLL Load Failed (Windows)

**Problem:** "ImportError: DLL load failed while importing _physics_umbrella"

**Solution:** Run `setup_paths.py` to configure DLL search paths:
```bash
cd omni/umbrella/tests/python
../../repo.bat uv run python setup_paths.py
```

### PowerShell Script Execution Error

**Problem:** "Activate.ps1 cannot be loaded because running scripts is disabled on this system"

This is a Windows PowerShell security restriction. Things work fine, but you see this error.

**Solutions:**

**Option 1 - Use Command Prompt (Default):**
The `.vscode/settings.json` is already configured to use Command Prompt instead of PowerShell.

**Option 2 - Allow PowerShell Scripts:**
Open PowerShell (as regular user) and run:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

**Option 3 - Use PowerShell with Bypass:**
Edit `.vscode/settings.json` and uncomment this line:
```json
"terminal.integrated.shellArgs.windows": ["-ExecutionPolicy", "Bypass"]
```

## Advanced Configuration

### Custom Build Configurations

Edit `.vscode/tasks.json` to add custom build tasks:

```json
{
    "label": "Umbrella: Build Custom",
    "type": "shell",
    "command": ".\\build.bat --custom-flags",
    "options": {
        "cwd": "${workspaceFolder:umbrella}"
    }
}
```

### Custom Launch Configurations

Edit `.vscode/launch.json` to add debug configurations:

```json
{
    "name": "Umbrella: Debug Custom Test",
    "type": "debugpy",
    "request": "launch",
    "program": "${workspaceFolder:umbrella}/tests/python/my_test.py",
    "cwd": "${workspaceFolder:umbrella}/tests/python"
}
```

### Environment Variables

Add environment variables in `launch.json`:

```json
"env": {
    "MY_VAR": "value",
    "PYTHONPATH": "${workspaceFolder:umbrella}/bindings"
}
```

## See Also

- [Umbrella Python Tests README](tests/python/README.md)
- [VS Code Python Debugging](https://code.visualstudio.com/docs/python/debugging)
- [VS Code C++ Debugging](https://code.visualstudio.com/docs/cpp/cpp-debug)
- [VS Code Tasks](https://code.visualstudio.com/docs/editor/tasks)

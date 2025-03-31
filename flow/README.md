# NVIDIA Flow SDK

## Getting Started

Platforms supported: Windows, Linux, Linux ARM64


## Building the SDK

### Build Requirements

Windows: Visual Studio 2017 and higher. VsWhere will select the latest version.

Linux: GMake

After cloning this repository onto your local drive:

### Windows
```bat
cd flow
.\build.bat
```
To run Flow editor: `_build\windows-x86_64\release\nvfloweditor.exe` (release), `_build\windows-x86_64\debug\nvfloweditor.exe` (debug) 

### Linux
```sh
cd flow
./build.sh
```
To run Flow editor: `_build/linux-x86_64/release/nvfloweditor` (release), `_build/linux-x86_64/debug/nvfloweditor` (debug) 

Note: If build fails, might need to install xrandr-dev. On Ubuntu:
```sh
sudo apt install libxrandr-dev
```

## Documentation

The documentation is available on [GitHub Pages](https://nvidia-omniverse.github.io/PhysX/flow/index.html).

## Acknowledgements

This depot references packages of third party open source software copyright their respective owners.
For copyright details, please refer to the license files included in the packages.


| Software              | Copyright Holder                                  | Package |
| --------------------- | ------------------------------------------------- | ------- |
| DXC                   | Microsoft Corporation                             | dxc     |
| GLFW                  | Marcus Geelnard and Camilla Löwy                  | glfw    |
| Dear ImGui            | Omar Cornut                                       | imgui   |
| Premake               | Jason Perkins and individual contributors         | premake |
| Slang                 | Carnegie Mellon University and NVIDIA Corporation | slang   |
| Visual Studio Locator | Microsoft Corporation                             | VsWhere |

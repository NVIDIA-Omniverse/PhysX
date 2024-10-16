# NVIDIA PhysX

Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this list of conditions, and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of NVIDIA CORPORATION nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Table of Contents
- [Introduction](#introduction)
- [Documentation](#documentation)
- [Instructions](#instructions)
- [Community-Maintained Build Configuration Fork](#community-maintained-build-configuration-fork)
- [Support](#support)
- [Installation](#installation)
- [Getting Started](#getting-started)
- [Contributing](#contributing)
- [License](#license)
- [FAQ](#faq)
- [Acknowledgments](#acknowledgments)

## Introduction

Welcome to the NVIDIA PhysX source code repository.

This repository contains source releases of the PhysX, Flow, and Blast SDKs used in NVIDIA Omniverse.

## Documentation

The user guide and API documentation are available on [GitHub Pages](https://nvidia-omniverse.github.io/PhysX). Please create an [Issue](https://github.com/NVIDIA-Omniverse/PhysX/issues/) if you find a documentation issue.

## Instructions

Please see instructions specific to each of the libraries in the respective subfolder.

## Community-Maintained Build Configuration Fork

Please see [the O3DE Fork](https://github.com/o3de/PhysX) for community-maintained additional build configurations.

## Support

* Please use GitHub [Discussions](https://github.com/NVIDIA-Omniverse/PhysX/discussions/) for questions and comments.
* GitHub [Issues](https://github.com/NVIDIA-Omniverse/PhysX/issues) should only be used for bug reports or documentation issues.
* You can also ask questions in the NVIDIA Omniverse #physics [Discord Channel](https://discord.com/invite/XWQNJDNuaC).

## Installation

### Prerequisites
- Supported operating systems: Windows, Linux, macOS
- Required tools: CMake, Visual Studio, GCC, etc.
- Optional: NVIDIA GPU for accelerated simulations

### Steps
1. Clone the repository:
   ```bash
   git clone https://github.com/NVIDIA-Omniverse/PhysX.git
   ```
2. Navigate to the PhysX directory:
   ```bash
   cd PhysX
   ```
3. Build the project using CMake:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

## Getting Started

Here's a simple example to get you started with PhysX:

```cpp
#include <PxPhysicsAPI.h>

using namespace physx;

int main() {
    // Initialize PhysX
    PxDefaultAllocator allocator;
    PxDefaultErrorCallback errorCallback;
    PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
    PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());

    // Create a simple dynamic actor
    PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.6f);
    PxTransform transform(PxVec3(0, 10, 0));
    PxRigidDynamic* actor = physics->createRigidDynamic(transform);
    PxShape* shape = physics->createShape(PxSphereGeometry(1.0f), *material);
    actor->attachShape(*shape);
    shape->release();

    // Clean up
    actor->release();
    physics->release();
    foundation->release();

    return 0;
}
```

## Contributing

We welcome contributions from the community. Please follow these steps to contribute:
1. Fork the repository and create a new branch for your changes.
2. Make your changes in the branch.
3. Ensure your code follows the [coding standards](https://docs.omniverse.nvidia.com/kit/docs/carbonite/latest/CODING.html).
4. Write tests for your changes and ensure all existing tests pass.
5. Submit a Pull Request with a clear description of the changes.

Please refer to our detailed [contributing guidelines](link-to-contributing-doc) for more information.

## License

NVIDIA PhysX is released under the [NVIDIA license](LICENSE.txt). Please review the license before using the software.

## FAQ

**Q: How do I report a bug?**
A: Please check the [Issues](https://github.com/NVIDIA-Omniverse/PhysX/issues) section to see if the bug has already been reported. If not, open a new Issue with a detailed description.

**Q: Can I use PhysX in commercial products?**
A: Yes, PhysX can be used in commercial products. Please refer to the [license](LICENSE.txt) for more details.

**Q: Where can I find more examples and tutorials?**
A: More examples and tutorials can be found in the [documentation](https://nvidia-omniverse.github.io/PhysX).

## Acknowledgments

We would like to thank all the contributors who have made this project possible. Special thanks to the developers of related libraries and tools.

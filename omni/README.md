# NVIDIA Omniverse PhysX Extensions and USD Schemas

## Introduction

Welcome to the NVIDIA Omniverse PhysX extensions and USD schemas source code repository. These extensions provide user and programmatic interfaces to author USD properties and objects based on PhysX schemas, and integrate the PhysX SDK into the [NVIDIA Omniverse platform](https://www.nvidia.com/en-us/omniverse/) as a simulation engine.

A key feature of this repository is the ability to directly modify the PhysX SDK source code, as well as the Omniverse PhysX integration and USD schemas, and immediately test your changes in [IsaacSim](https://github.com/isaac-sim/IsaacSim), [IsaacLab](https://github.com/isaac-sim/IsaacLab), or applications built with the [Kit App Template](https://github.com/NVIDIA-Omniverse/kit-app-template).

## Requirements

The following software is required for building the code in this repository. The versions listed are those with which it was tested:

* Windows: Microsoft Visual Studio 2019, WinSDK 10.0.1904.0. Tested on Windows 10/11.
* Linux: gcc 11.4, GNU Make 4.3. Tested on Ubuntu 24.04 LTS.

* Git with Git LFS.

* Optional: Python 3 for running the IsaacSim setup and build script.

Note: gcc versions later than 11 are not supported by the internally used nvcc compiler. You can use an override flag; however, using an unsupported host compiler may cause compilation failure or incorrect runtime execution. Use at your own risk. Switching to an older gcc version on Ubuntu can be achieved by using update-alternatives e.g. `sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 10`.

Note: For downloading Visual Studio 2019 see [Visual Studio Dev Essentials](https://visualstudio.microsoft.com/dev-essentials/) and [Visual Studio Older Downloads](http://visualstudio.microsoft.com/vs/older-downloads/).

## User Guide and API Documentation

The developer guide and API documentation are available on the [Omniverse Physics Developer Guide pages](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/). Switch to a version of the documentation appropriate to your checked-out release version.

## Quick Start

Clone this repository to your local drive, make sure you have Git LFS installed. Change directory to `/omni` and run ``./build.[bat|sh] -r``. This will build the release configuration of the extensions. After a successful build, run ``./_build/platform/release/omni.bloky.physx.kit.[bat|sh]``.

Note: During the pre-build process, dependencies will be automatically downloaded using the packman package manager. The first build can thus take a significant amount of time depending on the speed of your internet connection.

## Building

Note: If your intention is to build just Omniverse PhysX with default dependencies (you are not following e.g. [Usage with IsaacSim](#usage-with-isaacsim)), then first run `./repo.[bat|sh] source clear` to clear any previous kit-kernel package dependency overrides (.user files) from previous attempts.

Run `./build.[bat|sh]` with the following optional switches:
* `-x`: Cleans the repository before building. Important: If you have just set up or cleaned up a kit-kernel dependency override with `./repo.[bat|sh] source`, you must always build with `-x` to do a clean rebuild!
* `-g`: Generates project files and stops.
* `--devphysx`: Builds with local PhysX SDK binaries present in the `/physx` directory instead of using a prebuilt package. On `Linux`, you must first build the PhysX SDK separately; see the [Building with PhysX SDK](#building-with-physx-sdk) section.
* `--devschema`: Generates and builds with PhysX USD schema code in the `/omni/schema` directory instead of using a prebuilt package. Make sure to use ``./build.[bat|sh]`` instead of ``./repo.[bat|sh] build`` when using ``--devschema``, otherwise the schema generation process will not trigger!

⚠️Windows Path Length Limitation⚠️: Windows has a path length limitation of 260 characters. If you encounter errors related to missing files or other build errors, please move the repository to a shorter path.

See ``./build.[bat|sh] --help`` for more information.

### Building with PhysX SDK

* `Linux`: Run `/physx/generate_projects.sh` and choose the linux-carbonite or linux-aarch64-carbonite preset, depending on your platform, to generate Makefiles. Build PhysX SDK with the `checked` configuration. See also `/physx/README.md`. After the PhysX SDK is built, build Omniverse PhysX with the `--devphysx` switch; see the [Building](#building) section.
* `Windows`: Run `/physx/generate_projects.bat` and choose the vc16win64-carbonite preset to generate a Visual Studio solution. Build PhysX SDK with the `checked` configuration. See also `/physx/README.md`. After the PhysX SDK is built, build Omniverse PhysX with the `--devphysx` switch; see the [Building](#building) section. We also include PhysX SDK Visual Studio project files and build them as part of the Omniverse PhysX build process when the `--devphysx` switch is used.

## Usage with IsaacSim

* Check out a tag of the IsaacSim repository according to the following mapping of IsaacSim repository tags to PhysX repository tags:
    * v5.1.0 - 107.3-omni-and-physx-5.6.1
    * Fallback to the default branch if the tag is not yet available.
* Run `python ./tools/isaacsim/setup_and_build.py` with the following arguments:
    * `--isaacsim PATH_TO_REPO`: Specifies the path to your local IsaacSim repository.
    * (optional) `--devphysx`: Build with local PhysX SDK binaries. You must first build the PhysX SDK separately; see the [Building with PhysX SDK](#building-with-physx-sdk) section.
    * (optional) `--devschema`: Build with local PhysX USD schema code.
    * See [IsaacSim Setup and Build Script - Details](#isaacsim-setup-and-build-script---details) section for explanation of steps performed by the script.
* Once setup_and_build.py completes successfully, it will print out the appropriate command line for you to launch `isaac-sim.[bat|sh]` with the `--devFolder` argument to specify the location of your locally built PhysX extensions.

### IsaacSim Setup and Build Script - Details

* The following is the list of steps performed by the `./tools/isaacsim/setup_and_build.py` helper script:
    * In the IsaacSim directory:
        * Run `./repo.[bat|sh] source clear` to clear any previous package dependency overrides (.user files).
        * Run `./build.[bat|sh] -g -x` to download all IsaacSim dependencies and clean any previously built files.
    * In the `omni` folder of the Omniverse PhysX repository, run `./repo.[bat|sh] source link kit-kernel '/path/to/isaacsim/_build/${platform_target}/${config}/kit'` (replace */path/to/isaacsim* with the path to the directory where you cloned the IsaacSim repository). This will ensure that Omniverse PhysX is built with the same KitSDK version as IsaacSim.
    * Now build Omniverse PhysX extensions according to the [Building](#building) section above.
        * Make sure to add `--devphysx` if you want to build with your locally built PhysX SDK binaries in the `/physx` folder. You must first build the PhysX SDK separately; see the [Building with PhysX SDK](#building-with-physx-sdk) section!
        * Make sure to add `--devschema` if you want to build with your local changes to the PhysX schemas in the `/omni/schema` folder.
    * Now switch back to the IsaacSim directory.
        * If you built Omniverse PhysX with `--devphysx`, run `./repo.[bat|sh] source link physx /path/to/physx`. Replace */path/to/physx* with the path to the `/physx` subdirectory in the cloned Omniverse PhysX repository. This will use local PhysX SDK headers when building IsaacSim.
        * If you built Omniverse PhysX with `--devschema`, run `./repo.[bat|sh] source link 'usd_ext_physics_${config}' '/path/to/omni/_build/${platform_target}/release/schema/'`. Replace */path/to/omni* with the path to the `/omni` subdirectory in this repository. This will use local PhysX schema headers when building IsaacSim.
        * Run ``./repo.[bat|sh] source link omni_physics /path/to/omni``. Replace */path/to/omni* with the path to the `/omni` subdirectory in this repository. This will use local Omniverse PhysX headers when building IsaacSim.
        * Compile IsaacSim by running `./build.[bat|sh]`.
* Run `isaac-sim.[bat|sh]` with a `--devFolder` argument pointing to where the locally built PhysX extensions are located:
``/path/to/isaacsim/_build/your_platform/release/isaac-sim.[bat|sh] --/app/exts/devFolders=[/path/to/omni/_build/your_platform/release/extsPhysics/]``
    * Note: Replace */path/to/isaacsim* with the path to the IsaacSim repository's directory, *your_platform* with `windows-x86_64`, `linux-x86_64` or `linux-aarch64` amnd */path/to/omni* with the path to the `/omni` subdirectory in this repository.

## Usage with IsaacLab

* Follow the [Usage with IsaacSim](#usage-with-isaacsim) section.
* Install IsaacLab according to the instructions in the IsaacLab repository - use the installation method when using IsaacSim source code.
* Run an IsaacLab script with a `--devFolder` path pointing to where the locally built PhysX extensions are located through a `--kit-args` argument, for example:
``isaaclab.[bat|sh] -p script.py --kit_args="--/app/exts/devFolders=[/path/to/omni/_build/your_platform/release/extsPhysics/] "``
Replace */path/to/omni* with the path to the `/omni` subdirectory in this repository and *your_platform* with `windows-x86_64`, `linux-x86_64` or `linux-aarch64`. The space at the end of `--kit_args` above is currently necessary for the call not to fail with only one parameter!

## Usage with Kit App Template

* Run `./repo.[bat|sh] source clear` to clear any previous package dependency overrides (.user files).
* Build Omniverse PhysX extensions according to the [Building](#building) section above.
* Check out a version branch of the Kit App Template repository with the same major.minor version as the kit-kernel package in `/omni/deps/kit-sdk-target-deps.packman.xml` in this repository.
* Create and configure a new application from a template according to the instructions in the Kit App Template repository.
* Open the generated `.kit` file for your application in the `/source/apps/` directory in the Kit App Template repository.
    * Find the `[settings.app.exts]` section and add the following line into that section: `devFolders = ["/path/to/omni/_build/your_platform/release/extsPhysics"]`, with the path pointing to where the locally built PhysX extensions are located. Replace */path/to/omni* with the path to the `/omni` subdirectory in this repository and *your_platform* with `windows-x86_64`, `linux-x86_64` or `linux-aarch64`.
    * Optionally, you can add `"omni.physx.bundle" = {}` into the `[dependencies]` section to enable all Omniverse PhysX extensions by default. If `"omni.physx.stageupdate" = {}` is already present in that section, replace it with `"omni.physx.bundle" = {}` instead.
* Run `./repo.[bat|sh] launch` from the Kit App Template repository.

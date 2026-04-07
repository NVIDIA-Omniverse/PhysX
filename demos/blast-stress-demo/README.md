# Blast Stress Demo

This sample shows a minimal end-to-end integration between the Blast Toolkit API and the stress solver extension driven by PhysX.

## Summary
- creates a TkAsset for a Voronoi-fractured unit cube in memory
- instantiates the asset, registers it with `ExtStressSolver`
- simulates a PhysX scene, applying the solver and fracturing when bonds exceed their limits
- spawns a rigid sphere that hits the cube so it shatters

## Prerequisites
- Blast SDK and PhysX libraries built in this workspace (run `./blast/build.sh` and `./physx/generate_projects.sh && ./physx/buildtools/<platform>/build.sh` or the equivalent for your platform)
- CMake 3.16+

## Configure & Build
```sh
mkdir -p /workspaces/PhysX/demos/blast-stress-demo/build
cd /workspaces/PhysX/demos/blast-stress-demo/build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

> Adjust `CMAKE_PREFIX_PATH` or `CMAKE_LIBRARY_PATH` if your Blast/PhysX libs sit outside default search paths.

## Run
```sh
./blast_stress_demo
```

The executable runs a short simulation (roughly six seconds @60 Hz) and prints `Blast stress demo finished` after the solver has fractured the cube.




















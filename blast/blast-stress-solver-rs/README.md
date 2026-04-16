# blast-stress-solver

Rust bindings for the Blast stress solver, packaged for straightforward native
and WebAssembly consumption from Cargo.

This crate gives you two main layers:

- `ExtStressSolver` for Blast-only stress, bond failure, and split handling
- `blast_stress_solver::rapier::DestructibleSet` for keeping Blast actors and
  Rapier rigid bodies in sync

## Installation

Core solver only:

```toml
[dependencies]
blast-stress-solver = "0.1.0"
```

With built-in scenario builders:

```toml
[dependencies]
blast-stress-solver = { version = "0.1.0", features = ["scenarios"] }
```

With Rapier integration and scenario builders:

```toml
[dependencies]
blast-stress-solver = { version = "0.1.0", features = ["rapier", "scenarios"] }
rapier3d = { version = "0.30", default-features = false, features = ["dim3", "f32"] }
```

## Target support

The published crate currently ships packaged backends for:

- `aarch64-apple-darwin`
- `wasm32-unknown-unknown`

For other Apple/Linux native targets, the published crate falls back to
compiling the bundled Blast C++ sources on the consumer machine with a normal
C++17 toolchain. That removes the need to vendor the PhysX monorepo just to
support x86_64 macOS or Linux consumers.

`wasm32-unknown-unknown` intentionally stays prepackaged: downstream web builds
do not need Emscripten, wasi-sdk, or a second Blast-side wasm/JS loader.

Advanced overrides:

- `BLAST_STRESS_SOLVER_STATIC_LIB_PATH=/abs/path/to/libblast_stress_solver_ffi.a`
- `BLAST_STRESS_SOLVER_LIB_DIR=/abs/path/to/lib/dir`
- `BLAST_STRESS_SOLVER_FORCE_SOURCE_BUILD=1` for the bundled Apple/Linux native
  fallback

## Quick Start

The easiest way to understand the API is in two steps:

1. Build a destructible wall and drive the Blast solver directly.
2. Take the same wall and let `DestructibleSet` keep it synchronized with
   Rapier rigid bodies.

The examples below are intentionally explicit about which code belongs to your
application and which code is specific to `blast-stress-solver`.

### 1. Blast-only example: build and collapse a simple wall

This version does not use Rapier yet. It shows the minimum Blast-side workflow:

1. Build a wall scenario.
2. Create `ExtStressSolver` from its nodes and bonds.
3. Apply gravity and an impact force.
4. Ask Blast for fracture commands and apply them.

```rust
use blast_stress_solver::scenarios::{build_wall_scenario, WallOptions};
use blast_stress_solver::{ExtStressSolver, ForceMode, SolverSettings, Vec3};

fn main() {
    // Step 1: consumer app setup.
    // Choose a small wall so the example stays readable.
    let wall_options = WallOptions {
        span: 4.0,
        height: 2.0,
        thickness: 0.30,
        span_segments: 8,
        height_segments: 4,
        layers: 1,
        deck_mass: 200.0,
        ..WallOptions::default()
    };
    let wall = build_wall_scenario(&wall_options);

    // Step 2: Blast-specific setup.
    // Convert the scenario into the node/bond descriptors expected by the solver.
    let (nodes, bonds) = wall.to_solver_descs();

    // Use intentionally weak limits so the wall can visibly fail under load.
    let settings = SolverSettings {
        max_solver_iterations_per_frame: 64,
        compression_elastic_limit: 0.01,
        compression_fatal_limit: 0.05,
        tension_elastic_limit: 0.01,
        tension_fatal_limit: 0.05,
        shear_elastic_limit: 0.01,
        shear_fatal_limit: 0.05,
        ..SolverSettings::default()
    };

    let mut solver =
        ExtStressSolver::new(&nodes, &bonds, &settings).expect("failed to create solver");

    // Pick the top-center block as the impact target.
    let impact_node =
        (wall_options.height_segments - 1) * wall_options.span_segments + wall_options.span_segments / 2;
    let impact_position = wall.nodes[impact_node as usize].centroid;

    // Step 3: consumer app simulation loop.
    // In a real game or app, this would run once per frame.
    for frame in 0..30 {
        // Blast-specific: tell the solver about persistent gravity.
        solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));

        // Consumer app event: inject a one-time impulse-like load.
        if frame == 0 {
            solver.add_force(
                impact_node,
                impact_position,
                Vec3::new(5_000.0, 0.0, 0.0),
                ForceMode::Force,
            );
        }

        // Blast-specific: solve stress for the accumulated loads.
        solver.update();

        // Blast-specific: convert overstressed bonds into fracture commands.
        let commands = solver.generate_fracture_commands();
        let broken_bonds: usize = commands.iter().map(|cmd| cmd.bond_fractures.len()).sum();

        // Blast-specific: apply those fractures and let Blast split actors.
        let split_events = solver.apply_fracture_commands(&commands);

        println!(
            "frame={frame:02} actors={} overstressed={} broken_bonds={} split_events={}",
            solver.actor_count(),
            solver.overstressed_bond_count(),
            broken_bonds,
            split_events.len(),
        );

        if !split_events.is_empty() {
            println!("wall split after frame {frame}");
            break;
        }
    }
}
```

What to notice:

- `build_wall_scenario(...)` is optional convenience. If you already have your
  own nodes and bonds, you can skip the `scenarios` feature and build
  `NodeDesc` / `BondDesc` directly.
- The bottom row of the built-in wall is automatically marked as support
  (`mass == 0.0`), so the wall has something fixed to break away from.
- `ExtStressSolver` owns the Blast family/actor state. After fractures are
  applied, `actor_count()` grows as disconnected pieces split apart.

### 2. Rapier example: the same wall, now backed by rigid bodies

This version shows where Rapier is actually integrated.

`DestructibleSet` does **not** replace Rapier. Your app still owns the Rapier
world, the physics pipeline, and the frame loop. `blast-stress-solver` handles
the destruction-specific part:

1. keep a Blast stress graph for the structure
2. detect failed bonds
3. split Blast actors
4. create/update/destroy the corresponding Rapier bodies and colliders

```rust
use blast_stress_solver::rapier::{DestructibleSet, FracturePolicy};
use blast_stress_solver::scenarios::{build_wall_scenario, WallOptions};
use blast_stress_solver::{SolverSettings, Vec3};
use rapier3d::prelude::*;

fn main() {
    // Step 1: consumer app setup.
    // Build the same simple wall shape.
    let wall_options = WallOptions {
        span: 4.0,
        height: 2.0,
        thickness: 0.30,
        span_segments: 8,
        height_segments: 4,
        layers: 1,
        deck_mass: 200.0,
        ..WallOptions::default()
    };
    let wall = build_wall_scenario(&wall_options);

    // Step 2: Blast-specific setup.
    // Create the destruction controller that bridges Blast and Rapier.
    let settings = SolverSettings {
        max_solver_iterations_per_frame: 64,
        compression_elastic_limit: 0.01,
        compression_fatal_limit: 0.05,
        tension_elastic_limit: 0.01,
        tension_fatal_limit: 0.05,
        shear_elastic_limit: 0.01,
        shear_fatal_limit: 0.05,
        ..SolverSettings::default()
    };
    let fracture_policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut destructible = DestructibleSet::from_scenario(
        &wall,
        settings,
        Vec3::new(0.0, -9.81, 0.0),
        fracture_policy,
    )
    .expect("failed to create destructible set");

    // Step 3: consumer app setup.
    // You still own the Rapier world and physics pipeline.
    let mut rigid_bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut island_manager = IslandManager::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let mut physics_pipeline = PhysicsPipeline::new();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd_solver = CCDSolver::new();
    let integration_parameters = IntegrationParameters::default();
    let gravity = Vector::new(0.0, -9.81, 0.0);

    // Blast-specific: create the initial Rapier bodies/colliders that match
    // the current Blast actor table.
    destructible.initialize(&mut rigid_bodies, &mut colliders);

    let impact_node =
        (wall_options.height_segments - 1) * wall_options.span_segments + wall_options.span_segments / 2;
    let impact_position = wall.nodes[impact_node as usize].centroid;

    // Step 4: consumer app frame loop.
    for frame in 0..120 {
        // Consumer app event: inject a hit into the wall.
        if frame == 5 {
            destructible.add_force(impact_node, impact_position, Vec3::new(5_000.0, 0.0, 0.0));
        }

        // Blast-specific:
        // - apply gravity to the stress graph
        // - solve bond stresses
        // - fracture failed bonds
        // - split actors
        // - rebuild Rapier bodies/colliders as needed
        let fracture_step = destructible.step(
            &mut rigid_bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );

        // Consumer app:
        // Advance the rest of Rapier as usual.
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
            &(),
            &(),
        );

        println!(
            "frame={frame:03} fractures={} split_events={} new_bodies={} actors={} bodies={}",
            fracture_step.fractures,
            fracture_step.split_events,
            fracture_step.new_bodies,
            destructible.actor_count(),
            destructible.body_count(),
        );

        if fracture_step.split_events > 0 {
            println!("wall has started splitting into separate Rapier bodies");
            break;
        }
    }
}
```

What to notice:

- Your app still owns `RigidBodySet`, `ColliderSet`, the Rapier pipeline, and
  the main frame loop.
- `DestructibleSet::initialize(...)` is the point where Blast’s initial actor
  graph becomes actual Rapier bodies and colliders.
- `DestructibleSet::step(...)` is the integration point. That is where this
  crate applies gravity to the stress graph, breaks bonds, and rewrites the
  Rapier body/collider layout when actors split.
- `physics_pipeline.step(...)` is still your responsibility. This crate does not
  hide Rapier from you.

## WebAssembly

This crate supports downstream Rust applications that build for
`wasm32-unknown-unknown`.

The intended model is:

- your application depends on `blast-stress-solver` as a normal Rust dependency
- your application builds one final wasm output
- no Blast-specific sidecar wasm or JS loader is required

### Downstream `wasm-bindgen` example

```toml
[lib]
crate-type = ["cdylib"]

[dependencies]
blast-stress-solver = "0.1.0"
wasm-bindgen = "0.2"
```

```rust
use blast_stress_solver::{BondDesc, ExtStressSolver, NodeDesc, SolverSettings, Vec3};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn blast_tick() -> u32 {
    let nodes = [
        NodeDesc { centroid: Vec3::new(0.0, 0.0, 0.0), mass: 0.0, volume: 1.0 },
        NodeDesc { centroid: Vec3::new(0.0, 1.0, 0.0), mass: 10.0, volume: 1.0 },
    ];
    let bonds = [BondDesc {
        centroid: Vec3::new(0.0, 0.5, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
        area: 1.0,
        node0: 0,
        node1: 1,
    }];

    let Some(mut solver) = ExtStressSolver::new(&nodes, &bonds, &SolverSettings::default()) else {
        return u32::MAX;
    };

    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();
    solver.node_count()
}
```

That build still emits one final application wasm file. There is no extra Blast
runtime bundle to host or load manually.

## Features

- `rapier`: enables Rapier3D integration helpers
- `scenarios`: enables built-in wall, tower, and bridge scenario builders

## Notes

- Local publish-style proof:
  `scripts/assemble-blast-stress-solver-package.sh --verify-demo-consumer`
  stages the crate, verifies the packaged native and wasm smoke consumers, and
  runs the real `blast/blast-stress-demo-rs` headless fracture test against the
  staged package.
- The published crate is distributed as packaged Rust source plus:
  - prebuilt backend artifacts for `aarch64-apple-darwin` and `wasm32-unknown-unknown`
  - bundled native C++ sources for the Apple/Linux source-build fallback
- The monorepo development setup can still build the backend from source, but
  consumers no longer need to vendor the monorepo to get that native fallback.

#!/usr/bin/env node
/**
 * Generate reference data for Rust cross-validation tests.
 *
 * Runs key scenarios through the JS stress solver and outputs JSON
 * with exact numerical results that the Rust tests compare against.
 *
 * Usage: node --experimental-vm-modules generate-reference-data.mjs
 * (must be run from blast/blast-stress-solver/ after `npm run build`)
 */

import { loadStressSolver } from '../dist/index.js';
import { buildWallScenario, buildTowerScenario, buildBeamBridgeScenario } from '../dist/scenarios.js';

import { writeFileSync } from 'fs';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';

const __dirname = dirname(fileURLToPath(import.meta.url));

async function main() {
  const rt = await loadStressSolver();
  const results = {};

  // ----- Scenario shape tests -----
  {
    const wall = buildWallScenario(); // default options
    results.wall_default = {
      nodeCount: wall.nodes.length,
      bondCount: wall.bonds.length,
      supportCount: wall.nodes.filter((n) => n.mass === 0).length,
      dynamicCount: wall.nodes.filter((n) => n.mass > 0).length,
      firstNodeCentroid: wall.nodes[0].centroid,
      lastNodeCentroid: wall.nodes[wall.nodes.length - 1].centroid,
      firstBondArea: wall.bonds[0].area,
      lastBondArea: wall.bonds[wall.bonds.length - 1].area,
    };
  }
  {
    const tower = buildTowerScenario(); // default options
    results.tower_default = {
      nodeCount: tower.nodes.length,
      bondCount: tower.bonds.length,
      supportCount: tower.nodes.filter((n) => n.mass === 0).length,
      dynamicCount: tower.nodes.filter((n) => n.mass > 0).length,
    };
  }
  {
    const bridge = buildBeamBridgeScenario(); // default options
    results.bridge_default = {
      nodeCount: bridge.nodes.length,
      bondCount: bridge.bonds.length,
      supportCount: bridge.nodes.filter((n) => n.mass === 0).length,
      dynamicCount: bridge.nodes.filter((n) => n.mass > 0).length,
    };
  }

  // ----- Solver simulation tests -----
  // Helper: simulate N frames, return fracture/actor counts
  function simulate(solver, gravity, frames) {
    let totalFractures = 0;
    for (let i = 0; i < frames; i++) {
      solver.addGravity(gravity);
      solver.update();
      const overstressed = solver.overstressedBondCount();
      if (overstressed > 0) {
        const cmds = solver.generateFractureCommandsPerActor();
        let frameFractures = 0;
        for (const cmd of cmds) {
          frameFractures += cmd.fractures.length;
        }
        totalFractures += frameFractures;
        if (cmds.length > 0) {
          solver.applyFractureCommands(cmds);
        }
      }
    }
    return {
      totalFractures,
      actorCount: solver.actors().length,
      converged: solver.converged(),
    };
  }

  // Strong settings (should survive gravity)
  const strongSettings = {
    maxSolverIterationsPerFrame: 24,
    compressionElasticLimit: 90_000,
    compressionFatalLimit: 270_000,
    tensionElasticLimit: 90_000,
    tensionFatalLimit: 270_000,
    shearElasticLimit: 120_000,
    shearFatalLimit: 360_000,
  };

  // Weak settings (should fracture under gravity)
  const weakSettings = {
    maxSolverIterationsPerFrame: 24,
    compressionElasticLimit: 0.001,
    compressionFatalLimit: 0.002,
    tensionElasticLimit: 0.001,
    tensionFatalLimit: 0.002,
    shearElasticLimit: 0.001,
    shearFatalLimit: 0.002,
  };

  const gravity = { x: 0, y: -9.81, z: 0 };
  const frames = 60;

  // --- Wall with strong bonds ---
  {
    const wall = buildWallScenario();
    const solver = rt.createExtSolver({
      nodes: wall.nodes,
      bonds: wall.bonds,
      settings: strongSettings,
    });
    const result = simulate(solver, gravity, frames);
    results.wall_strong_gravity = {
      frames,
      ...result,
    };
    solver.destroy();
  }

  // --- Wall with weak bonds ---
  {
    const wall = buildWallScenario();
    const solver = rt.createExtSolver({
      nodes: wall.nodes,
      bonds: wall.bonds,
      settings: weakSettings,
    });
    const result = simulate(solver, gravity, frames);
    results.wall_weak_gravity = {
      frames,
      ...result,
    };
    solver.destroy();
  }

  // --- Tower with strong bonds ---
  {
    const tower = buildTowerScenario();
    const solver = rt.createExtSolver({
      nodes: tower.nodes,
      bonds: tower.bonds,
      settings: strongSettings,
    });
    const result = simulate(solver, gravity, frames);
    results.tower_strong_gravity = {
      frames,
      ...result,
    };
    solver.destroy();
  }

  // --- Tower with weak bonds ---
  {
    const tower = buildTowerScenario();
    const solver = rt.createExtSolver({
      nodes: tower.nodes,
      bonds: tower.bonds,
      settings: weakSettings,
    });
    const result = simulate(solver, gravity, frames);
    results.tower_weak_gravity = {
      frames,
      ...result,
    };
    solver.destroy();
  }

  // --- Simple 2-node column (gravity fixture) ---
  {
    const nodes = [
      { centroid: { x: 0, y: 0, z: 0 }, mass: 0, volume: 1 },
      { centroid: { x: 0, y: 1, z: 0 }, mass: 1, volume: 1 },
    ];
    const bonds = [
      {
        centroid: { x: 0, y: 0.5, z: 0 },
        normal: { x: 0, y: 1, z: 0 },
        area: 1,
        node0: 0,
        node1: 1,
      },
    ];
    const settings = {
      maxSolverIterationsPerFrame: 16,
      compressionElasticLimit: 0.5,
      compressionFatalLimit: 1.0,
      tensionElasticLimit: 0.5,
      tensionFatalLimit: 1.0,
      shearElasticLimit: 1e6,
      shearFatalLimit: 1e6,
    };
    const solver = rt.createExtSolver({ nodes, bonds, settings });

    // Frame 1: gravity down (compression)
    solver.addGravity({ x: 0, y: -5.0, z: 0 });
    solver.update();
    const downOverstressed = solver.overstressedBondCount();
    const downConverged = solver.converged();
    const err = solver.stressError();

    results.column_gravity_down = {
      overstressedBondCount: downOverstressed,
      converged: downConverged,
      linearError: err.lin,
      angularError: err.ang,
    };

    solver.destroy();
  }

  // --- Determinism: two identical runs produce same results ---
  {
    function runDeterminismTest() {
      const wall = buildWallScenario();
      const settings = {
        maxSolverIterationsPerFrame: 24,
        compressionElasticLimit: 5.0,
        compressionFatalLimit: 10.0,
        tensionElasticLimit: 5.0,
        tensionFatalLimit: 10.0,
        shearElasticLimit: 5.0,
        shearFatalLimit: 10.0,
      };
      const solver = rt.createExtSolver({
        nodes: wall.nodes,
        bonds: wall.bonds,
        settings,
      });
      const result = simulate(solver, gravity, 30);
      solver.destroy();
      return result;
    }
    const run1 = runDeterminismTest();
    const run2 = runDeterminismTest();
    results.determinism = {
      run1_actorCount: run1.actorCount,
      run2_actorCount: run2.actorCount,
      run1_totalFractures: run1.totalFractures,
      run2_totalFractures: run2.totalFractures,
      match: run1.actorCount === run2.actorCount && run1.totalFractures === run2.totalFractures,
    };
  }

  // --- Force impact ---
  {
    const wall = buildWallScenario();
    const settings = {
      maxSolverIterationsPerFrame: 24,
      compressionElasticLimit: 50.0,
      compressionFatalLimit: 100.0,
      tensionElasticLimit: 50.0,
      tensionFatalLimit: 100.0,
      shearElasticLimit: 50.0,
      shearFatalLimit: 100.0,
    };
    const solver = rt.createExtSolver({
      nodes: wall.nodes,
      bonds: wall.bonds,
      settings,
    });

    // Apply large force to center node
    const centerNode = Math.floor(wall.nodes.length / 2);
    const centerPos = wall.nodes[centerNode].centroid;
    solver.addForce(centerNode, centerPos, { x: 50000, y: 0, z: 0 }, 0 /* Force */);
    solver.update();

    const overstressed = solver.overstressedBondCount();
    let fractures = 0;
    if (overstressed > 0) {
      const cmds = solver.generateFractureCommandsPerActor();
      for (const cmd of cmds) {
        fractures += cmd.fractures.length;
      }
      if (cmds.length > 0) {
        solver.applyFractureCommands(cmds);
      }
    }

    results.wall_force_impact = {
      centerNode,
      overstressedAfterForce: overstressed,
      fracturesFromForce: fractures,
      actorCountAfterForce: solver.actors().length,
    };
    solver.destroy();
  }

  // --- Material strength comparison ---
  {
    function runWithMaterialScale(scale) {
      const wall = buildWallScenario();
      const settings = {
        maxSolverIterationsPerFrame: 24,
        compressionElasticLimit: 0.0009 * scale,
        compressionFatalLimit: 0.0027 * scale,
        tensionElasticLimit: 0.0009 * scale,
        tensionFatalLimit: 0.0027 * scale,
        shearElasticLimit: 0.0012 * scale,
        shearFatalLimit: 0.0036 * scale,
      };
      const solver = rt.createExtSolver({
        nodes: wall.nodes,
        bonds: wall.bonds,
        settings,
      });
      const result = simulate(solver, gravity, 30);
      solver.destroy();
      return result;
    }
    const weak = runWithMaterialScale(1e4);
    const strong = runWithMaterialScale(1e6);

    results.material_strength_comparison = {
      weak_fractures: weak.totalFractures,
      weak_actors: weak.actorCount,
      strong_fractures: strong.totalFractures,
      strong_actors: strong.actorCount,
      stronger_has_fewer_fractures: strong.totalFractures <= weak.totalFractures,
    };
  }

  // Write output
  const outPath = join(__dirname, '..', '..', 'blast-stress-solver-rs', 'tests', 'fixtures', 'js_reference_data.json');
  writeFileSync(outPath, JSON.stringify(results, null, 2));
  console.log(`Reference data written to: ${outPath}`);
  console.log(JSON.stringify(results, null, 2));
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});

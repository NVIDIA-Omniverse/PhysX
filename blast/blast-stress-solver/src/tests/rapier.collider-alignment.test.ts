/**
 * Regression tests for collider / mesh position alignment after bond fracture.
 *
 * Bug: flushColliderMigrations() computed an inverse-rotated body-local offset
 * for colliders, but the Three.js adapter always uses baseLocalOffset for mesh
 * positioning. This caused Rapier debug wireframes to diverge from rendered
 * meshes after second-level fractures (when a fragment that has already moved
 * from origin fractures again).
 *
 * The bug is invisible for first-level fractures because the root body is at
 * (0,0,0) with identity rotation, so the inverse rotation is a no-op.
 *
 * Fix: use baseLocalOffset directly as the collider translation during
 * migration, matching the rendering code.
 *
 * Requires full WASM + TS build. Skips gracefully if dist is unavailable.
 * Run: npm run build && npx vitest run src/tests/rapier.collider-alignment.test.ts
 */
import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = resolve(here, '../../dist/stress_solver.wasm');
const runtimeAvailable = existsSync(wasmPath);

type Vec3 = { x: number; y: number; z: number };

/**
 * Build a grid scenario designed to trigger second-level fractures:
 *   Row 0 (y=0.5): 3 support nodes (mass=0)
 *   Row 1 (y=1.5): 3 dynamic nodes
 *   Row 2 (y=2.5): 3 dynamic nodes
 *
 * Vertical bonds (between rows) are WEAK (area=0.01) → break first.
 * Horizontal bonds (within rows) are STRONG (area=2.0) → break second.
 *
 * This creates multi-node fragments that fall and then re-fracture,
 * triggering the second-level fracture where the bug manifests.
 */
function createGridScenario() {
  const nodes: Array<{ centroid: Vec3; mass: number; volume: number }> = [];
  for (let row = 0; row < 3; row++) {
    for (let col = 0; col < 3; col++) {
      nodes.push({
        centroid: { x: col, y: row * 1.0 + 0.5, z: 0 },
        mass: row === 0 ? 0 : 5.0,
        volume: 1.0,
      });
    }
  }

  const bonds: Array<{ node0: number; node1: number; centroid: Vec3; normal: Vec3; area: number }> = [];
  // Horizontal bonds (within rows) - STRONG
  for (let row = 0; row < 3; row++) {
    for (let col = 0; col < 2; col++) {
      const n0 = row * 3 + col;
      const n1 = row * 3 + col + 1;
      bonds.push({
        node0: n0, node1: n1,
        centroid: { x: col + 0.5, y: row * 1.0 + 0.5, z: 0 },
        normal: { x: 1, y: 0, z: 0 },
        area: 2.0,
      });
    }
  }
  // Vertical bonds (between rows) - WEAK
  for (let row = 0; row < 2; row++) {
    for (let col = 0; col < 3; col++) {
      const n0 = row * 3 + col;
      const n1 = (row + 1) * 3 + col;
      bonds.push({
        node0: n0, node1: n1,
        centroid: { x: col, y: row * 1.0 + 1.0, z: 0 },
        normal: { x: 0, y: 1, z: 0 },
        area: 0.01,
      });
    }
  }

  return { nodes, bonds };
}

/**
 * Compute the expected mesh world position for a chunk, matching the formula
 * used by the Three.js adapter (destructible-adapter.ts):
 *   worldPos = bodyPos + bodyRot * baseLocalOffset
 */
function computeMeshWorldPosition(
  bodyTranslation: Vec3,
  bodyRotation: { x: number; y: number; z: number; w: number },
  baseLocalOffset: Vec3,
): Vec3 {
  const lx = baseLocalOffset.x, ly = baseLocalOffset.y, lz = baseLocalOffset.z;
  const qx = bodyRotation.x, qy = bodyRotation.y, qz = bodyRotation.z, qw = bodyRotation.w;
  const ix = qw * lx + qy * lz - qz * ly;
  const iy = qw * ly + qz * lx - qx * lz;
  const iz = qw * lz + qx * ly - qy * lx;
  const iw = -(qx * lx + qy * ly + qz * lz);
  return {
    x: bodyTranslation.x + iw * (-qx) + ix * qw + iy * (-qz) - iz * (-qy),
    y: bodyTranslation.y + iw * (-qy) + iy * qw + iz * (-qx) - ix * (-qz),
    z: bodyTranslation.z + iw * (-qz) + iz * qw + ix * (-qy) - iy * (-qx),
  };
}

function dist(a: Vec3, b: Vec3): number {
  return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2);
}

describe.skipIf(!runtimeAvailable)('Collider/mesh alignment after fracture (requires WASM build)', () => {
  let buildDestructibleCore: (opts: any) => Promise<any>;

  async function loadModules() {
    if (buildDestructibleCore) return;
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;
  }

  async function buildGridCore() {
    return buildDestructibleCore({
      scenario: createGridScenario(),
      gravity: -9.81,
      materialScale: 0.001,
      resimulateOnFracture: true,
      maxResimulationPasses: 5,
      skipSingleBodies: false,
    });
  }

  it('localOffset equals baseLocalOffset after second-level fracture', async () => {
    await loadModules();
    const core = await buildGridCore();

    // Step until second-level fractures occur (fragments fall and re-fracture)
    for (let i = 0; i < 120; i++) core.step(1 / 60);

    // Find chunks on non-root bodies (these have been through migration)
    const migratedChunks = core.chunks.filter(
      (c: any) => c.active && c.bodyHandle != null && c.bodyHandle !== core.rootBodyHandle
    );
    // Verify some chunks actually migrated (fracture happened)
    expect(migratedChunks.length).toBeGreaterThan(0);

    // After migration, localOffset MUST equal baseLocalOffset for all active chunks.
    // With the buggy code, localOffset = invRot(bodyRot) * (baseLocalOffset - bodyPos),
    // which diverges from baseLocalOffset when bodyPos ≠ (0,0,0).
    for (const chunk of core.chunks) {
      if (!chunk.active || chunk.bodyHandle == null) continue;
      expect(chunk.localOffset.x).toBeCloseTo(chunk.baseLocalOffset.x, 4);
      expect(chunk.localOffset.y).toBeCloseTo(chunk.baseLocalOffset.y, 4);
      expect(chunk.localOffset.z).toBeCloseTo(chunk.baseLocalOffset.z, 4);
    }

    core.dispose();
  });

  it('collider world position matches mesh world position after fracture', async () => {
    await loadModules();
    const core = await buildGridCore();

    for (let i = 0; i < 120; i++) core.step(1 / 60);

    let checkedCount = 0;
    for (const chunk of core.chunks) {
      if (!chunk.active || chunk.bodyHandle == null || chunk.colliderHandle == null) continue;

      const body = core.world.getRigidBody(chunk.bodyHandle);
      if (!body) continue;

      const bodyPos = body.translation();
      const bodyRot = body.rotation();

      // Mesh position as the Three.js adapter computes it
      const meshPos = computeMeshWorldPosition(bodyPos, bodyRot, chunk.baseLocalOffset);

      // Collider position from Rapier (what the debug renderer draws)
      const collider = core.world.getCollider(chunk.colliderHandle);
      if (!collider) continue;
      const colliderPos = collider.translation();

      const d = dist(meshPos, { x: colliderPos.x, y: colliderPos.y, z: colliderPos.z });
      expect(d).toBeLessThan(0.01);
      checkedCount++;
    }

    expect(checkedCount).toBeGreaterThan(0);
    core.dispose();
  });

  it('worldPosition matches mesh formula after cascading fracture', async () => {
    await loadModules();
    const core = await buildGridCore();

    // Run longer to allow cascading fractures
    for (let i = 0; i < 180; i++) core.step(1 / 60);

    let checkedCount = 0;
    for (const chunk of core.chunks) {
      if (!chunk.active || chunk.bodyHandle == null) continue;
      if (!chunk.worldPosition) continue;

      const body = core.world.getRigidBody(chunk.bodyHandle);
      if (!body) continue;

      const bodyPos = body.translation();
      const bodyRot = body.rotation();
      const meshPos = computeMeshWorldPosition(bodyPos, bodyRot, chunk.baseLocalOffset);

      // worldPosition (computed by core via localOffset) must match
      // meshPos (computed via baseLocalOffset)
      const d = dist(meshPos, chunk.worldPosition);
      expect(d).toBeLessThan(0.01);
      checkedCount++;
    }

    expect(checkedCount).toBeGreaterThan(0);
    core.dispose();
  });
});

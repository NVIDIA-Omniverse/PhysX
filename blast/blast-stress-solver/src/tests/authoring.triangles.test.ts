import { describe, it, expect } from 'vitest';
import type * as Runtime from '..';
import {
  cubeTriangles,
  touchingCubeChunks,
  stackedCubeChunks,
  separatedCubeChunks,
  almostEqual
} from './bondFixtures';

async function importRuntime(): Promise<typeof Runtime> {
  return (await import('../../dist/index.js')) as typeof Runtime;
}

describe('createBondsFromTriangles', () => {
  it('produces a single bond for two cubes sharing one face', async () => {
    const { loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = touchingCubeChunks();
    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });

    expect(bonds).toHaveLength(1);
    const bond = bonds[0];
    expect(bond.node0).toBe(0);
    expect(bond.node1).toBe(1);
    expect(almostEqual(bond.area ?? 0, 1.0)).toBe(true);
    expect(almostEqual(bond.centroid?.x ?? 1e9, 0.0, 1e-6)).toBe(true);
    expect(Math.abs(bond.normal?.x ?? 0)).toBe(1);
  });

  it('returns zero bonds for a single isolated chunk', async () => {
    const { loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const singleChunk = [
      {
        triangles: cubeTriangles(0),
        isSupport: true
      }
    ];
    const bonds = rt.createBondsFromTriangles(singleChunk, { mode: 'exact' });
    expect(bonds.length).toBe(0);
  });

  it('is re-entrant across consecutive calls', async () => {
    const { loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const first = rt.createBondsFromTriangles(touchingCubeChunks(), { mode: 'exact' });
    const second = rt.createBondsFromTriangles(touchingCubeChunks(), { mode: 'exact' });

    expect(first).toHaveLength(1);
    expect(second).toHaveLength(1);
  });

  it('produces bonds for a 2x2 grid of touching cubes (supports everywhere)', async () => {
    const { loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = stackedCubeChunks(2, 2, 1.0);
    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });

    // We only require that some bonds exist between chunks; the exact number
    // is implementation-dependent inside the Blast bond generator.
    expect(bonds.length).toBeGreaterThan(0);

    // Sanity check that all referenced nodes exist and are distinct
    for (const bond of bonds) {
      expect(bond.node0).toBeGreaterThanOrEqual(0);
      expect(bond.node1).toBeGreaterThanOrEqual(0);
      expect(bond.node0).not.toBe(bond.node1);
      expect(bond.node0).toBeLessThan(chunks.length);
      expect(bond.node1).toBeLessThan(chunks.length);
    }
  });

  it('respects support flags when generating bonds', async () => {
    const { loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = stackedCubeChunks(2, 2, 1.0);

    // Mark only the first row as support; second row is non-support
    chunks[0].isSupport = true;
    chunks[1].isSupport = true;
    chunks[2].isSupport = false;
    chunks[3].isSupport = false;

    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });

    // All bonds should connect only nodes 0 and/or 1
    for (const bond of bonds) {
      expect([0, 1]).toContain(bond.node0);
      expect([0, 1]).toContain(bond.node1);
    }
  });

  // NOTE: Average-mode bonding from raw triangles requires a ConvexMeshBuilder
  // implementation, which the current WASM bridge does not provide. For now we
  // rely on exact-mode bonding only and leave average-mode coverage to the
  // native C++ test suite.
});


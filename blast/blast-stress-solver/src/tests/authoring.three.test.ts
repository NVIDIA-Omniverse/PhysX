import { describe, it, expect } from 'vitest';
import type * as Runtime from '..';
import { touchingBoxGeometries, gridBoxGeometries } from './bondFixtures';

async function importRuntime(): Promise<typeof Runtime> {
  return (await import('../../dist/index.js')) as typeof Runtime;
}

describe('Three.js helpers (end-to-end)', () => {
  it('chunksFromBufferGeometries + bonding', async () => {
    const [geomA, geomB] = touchingBoxGeometries();
    const { chunksFromBufferGeometries, loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
      isSupport: true,
      nonIndexed: true,
      cloneGeometry: true
    }));

    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
    expect(bonds).toHaveLength(1);
  });

  it('chunkFromBufferGeometry generates a usable chunk', async () => {
    const geom = touchingBoxGeometries()[0].clone();
    const { chunkFromBufferGeometry, loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunk = chunkFromBufferGeometry(geom, { isSupport: true });
    expect(chunk.triangles.length).toBeGreaterThan(0);

    const bonds = rt.createBondsFromTriangles([chunk], { mode: 'exact' });
    expect(bonds.length).toBe(0);
  });

  it('chunksFromBufferGeometries handles a small grid and honors isSupport flags', async () => {
    const geoms = gridBoxGeometries(2, 2, 1.0);
    const { chunksFromBufferGeometries, loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();

    // Mark only the top row as support
    const chunks = chunksFromBufferGeometries(geoms, (_geom, index) => {
      const row = Math.floor(index / 2);
      return {
        isSupport: row === 1,
        nonIndexed: true,
        cloneGeometry: true
      };
    });

    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });

    // All bonds should connect only nodes from the top row (indices 2 and 3).
    for (const bond of bonds) {
      expect([2, 3]).toContain(bond.node0);
      expect([2, 3]).toContain(bond.node1);
    }
  });
});


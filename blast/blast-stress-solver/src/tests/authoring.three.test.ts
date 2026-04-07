import { describe, it, expect } from 'vitest';
import type * as Runtime from '..';
import {
  touchingBoxGeometries,
  gridBoxGeometries,
  separatedBoxGeometries,
  preciseFaceBond,
  preciseFaceSharingGeometry
} from './bondFixtures';

async function importRuntime(): Promise<typeof Runtime> {
  return (await import('../../dist/index.js')) as typeof Runtime;
}

function expectBondMatches(
  bond: Runtime.ExtStressBondDesc,
  expectedArea = preciseFaceBond.area,
  areaTolerance?: number
) {
  expect([preciseFaceBond.nodes[0], preciseFaceBond.nodes[1]]).toContain(bond.node0);
  expect([preciseFaceBond.nodes[0], preciseFaceBond.nodes[1]]).toContain(bond.node1);
  expect(Math.abs(bond.normal?.x ?? 0)).toBe(preciseFaceBond.normalX);
  expect(bond.centroid).toBeTruthy();
  expect(Math.abs((bond.centroid?.x ?? 0) - preciseFaceBond.centroid.x)).toBeLessThan(1e-6);
  expect(Math.abs((bond.centroid?.y ?? 0) - preciseFaceBond.centroid.y)).toBeLessThan(1e-6);
  expect(Math.abs((bond.centroid?.z ?? 0) - preciseFaceBond.centroid.z)).toBeLessThan(1e-6);
  if (areaTolerance !== undefined) {
    expect(Math.abs((bond.area ?? 0) - expectedArea)).toBeLessThan(areaTolerance);
  }
}

describe('Three.js helpers (end-to-end)', () => {
  it('chunksFromBufferGeometries + bonding', async () => {
    const [geomA, geomB] = preciseFaceSharingGeometry();
    const { chunksFromBufferGeometries, loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
      isSupport: true,
      nonIndexed: true,
      cloneGeometry: true
    }));

    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
    expect(bonds).toHaveLength(1);
    expectBondMatches(bonds[0]);
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

  it('supports average bonding for BufferGeometry chunks with small gaps', async () => {
    const geoms = separatedBoxGeometries(0.1);
    const { chunksFromBufferGeometries, loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = chunksFromBufferGeometries(geoms, () => ({
      isSupport: true,
      nonIndexed: true,
      cloneGeometry: true
    }));

    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0.5 });
    expect(bonds).toHaveLength(1);
    expectBondMatches(bonds[0]);
  });
});


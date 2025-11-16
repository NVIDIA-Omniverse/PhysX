import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import type * as Runtime from '..';

async function importRuntime(): Promise<typeof Runtime> {
  return (await import('../../dist/index.js')) as typeof Runtime;
}

function createGeometries() {
  const boxA = new THREE.BoxGeometry(1, 1, 1);
  const boxB = new THREE.BoxGeometry(1, 1, 1);
  boxA.translate(-0.25, 0, 0);
  boxB.translate(0.0, 0, 0);
  return [boxA, boxB];
}

describe('Three.js helpers (end-to-end)', () => {
  it('chunksFromBufferGeometries + bonding', async () => {
    const [geomA, geomB] = createGeometries();
    const { chunksFromBufferGeometries, loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
      isSupport: true,
      nonIndexed: true,
      cloneGeometry: true
    }));

    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
    expect(Array.isArray(bonds)).toBe(true);
  });

  it('chunkFromBufferGeometry generates a usable chunk', async () => {
    const geom = new THREE.BoxGeometry(1, 1, 1);
    const { chunkFromBufferGeometry, loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunk = chunkFromBufferGeometry(geom, { isSupport: true });
    expect(chunk.triangles.length).toBeGreaterThan(0);

    const bonds = rt.createBondsFromTriangles([chunk], { mode: 'exact' });
    expect(bonds.length).toBe(0);
  });
});


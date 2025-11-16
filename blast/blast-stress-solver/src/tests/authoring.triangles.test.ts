import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import type * as Runtime from '..';

async function importRuntime(): Promise<typeof Runtime> {
  return (await import('../../dist/index.js')) as typeof Runtime;
}

function cubeTriangles(offset: number): Float32Array {
  const base = new THREE.BoxGeometry(1, 1, 1);
  const geom = base.toNonIndexed() ?? base;
  geom.translate(offset, 0, 0);
  const position = geom.getAttribute('position');
  return Float32Array.from(position.array as ArrayLike<number>);
}

function touchingCubes(): { triangles: Float32Array; isSupport?: boolean }[] {
  const cubeA = cubeTriangles(-0.4);
  const cubeB = cubeTriangles(-0.1);
  return [
    { triangles: cubeA, isSupport: true },
    { triangles: cubeB, isSupport: true }
  ];
}

describe('createBondsFromTriangles', () => {
  it('returns an array for overlapping chunks', async () => {
    const { loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    const chunks = touchingCubes();
    const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });

    expect(Array.isArray(bonds)).toBe(true);
    expect(typeof bonds.length).toBe('number');
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
    const first = rt.createBondsFromTriangles(touchingCubes(), { mode: 'exact' });
    const second = rt.createBondsFromTriangles(touchingCubes(), { mode: 'exact' });

    expect(Array.isArray(first)).toBe(true);
    expect(Array.isArray(second)).toBe(true);
  });
});


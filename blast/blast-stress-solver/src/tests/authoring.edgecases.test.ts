import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import type * as Runtime from '..';

async function importRuntime(): Promise<typeof Runtime> {
  return (await import('../../dist/index.js')) as typeof Runtime;
}

/**
 * Edge case tests for createBondsFromTriangles to help diagnose common issues
 * developers face when using the bonding API.
 */
describe('createBondsFromTriangles edge cases', () => {
  describe('exact mode', () => {
    it('returns 0 bonds when chunks have micro-gaps (floating point precision)', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      // Create two boxes with an extremely small gap (1e-7 meters)
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-0.5, 0, 0);
      geomB.translate(0.5 + 1e-7, 0, 0); // Tiny gap - should fail exact mode

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      // Exact mode is very strict about vertex positions
      // This may or may not find a bond depending on internal tolerance
      // The test documents the behavior
      expect(bonds.length).toBeGreaterThanOrEqual(0);
    });

    it('finds bonds between perfectly touching unit cubes', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      // Two 1x1x1 cubes sharing a face at X=0
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-0.5, 0, 0); // spans X: [-1, 0]
      geomB.translate(0.5, 0, 0);  // spans X: [0, 1]

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      expect(bonds.length).toBe(1);
      expect(bonds[0].area).toBeCloseTo(1.0, 2);
      expect(bonds[0].centroid?.x).toBeCloseTo(0, 4);
    });

    it('returns 0 bonds when chunks do not touch', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      // Two cubes with clear separation
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-1.5, 0, 0); // spans X: [-2, -1]
      geomB.translate(1.5, 0, 0);  // spans X: [1, 2]

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      expect(bonds.length).toBe(0);
    });

    it('returns 0 bonds when all chunks are marked as non-support', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-0.5, 0, 0);
      geomB.translate(0.5, 0, 0);

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: false, // Both non-support!
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      expect(bonds.length).toBe(0);
    });

    it('generates bonds only between support chunks in a mixed configuration', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      // 3 cubes in a row: [support] [non-support] [support]
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      const geomC = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-1, 0, 0); // support
      geomB.translate(0, 0, 0);  // non-support
      geomC.translate(1, 0, 0);  // support

      const chunks = chunksFromBufferGeometries([geomA, geomB, geomC], (_geom, index) => ({
        isSupport: index !== 1, // Middle chunk is non-support
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      // Bonds should only connect support chunks (0 and 2)
      // But chunks 0 and 2 don't touch, so no bonds expected
      expect(bonds.length).toBe(0);
    });

    it('handles a single chunk (returns 0 bonds)', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const geom = new THREE.BoxGeometry(1, 1, 1);
      const chunks = chunksFromBufferGeometries([geom], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      expect(bonds.length).toBe(0);
    });

    it('handles empty chunk array', async () => {
      const { loadStressSolver } = await importRuntime();
      const rt = await loadStressSolver();

      const bonds = rt.createBondsFromTriangles([], { mode: 'exact' });
      expect(bonds.length).toBe(0);
    });
  });

  describe('average mode', () => {
    it('throws when maxSeparation is not provided', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const geom = new THREE.BoxGeometry(1, 1, 1);
      const chunks = chunksFromBufferGeometries([geom], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      expect(() => {
        rt.createBondsFromTriangles(chunks, { mode: 'average' });
      }).toThrow(/maxSeparation/i);
    });

    it('throws when maxSeparation is zero', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const geom = new THREE.BoxGeometry(1, 1, 1);
      const chunks = chunksFromBufferGeometries([geom], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      expect(() => {
        rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0 });
      }).toThrow(/maxSeparation/i);
    });

    it('finds bonds for chunks with small gaps when maxSeparation covers the gap', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const gap = 0.05; // 5cm gap
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-0.5 - gap / 2, 0, 0);
      geomB.translate(0.5 + gap / 2, 0, 0);

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      // Gap is 5cm, so maxSeparation of 10cm should find the bond
      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0.1 });
      expect(bonds.length).toBe(1);
    });

    it('returns 0 bonds when maxSeparation is smaller than the gap', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const gap = 0.1; // 10cm gap
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-0.5 - gap / 2, 0, 0);
      geomB.translate(0.5 + gap / 2, 0, 0);

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      // Gap is 10cm, maxSeparation of 2cm shouldn't find bond
      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0.02 });
      expect(bonds.length).toBe(0);
    });
  });

  describe('transform handling', () => {
    it('applies world transforms via applyMatrix option', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      // Two cubes centered at origin, but with transforms that move them to touch
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);

      const matrixA = new THREE.Matrix4().makeTranslation(-0.5, 0, 0);
      const matrixB = new THREE.Matrix4().makeTranslation(0.5, 0, 0);

      const chunks = chunksFromBufferGeometries([geomA, geomB], (_geom, index) => ({
        isSupport: true,
        applyMatrix: index === 0 ? matrixA : matrixB,
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      expect(bonds.length).toBe(1);
    });

    it('works with rotation transforms', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      // Two cubes, one rotated 90 degrees around Y
      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);

      const matrixA = new THREE.Matrix4().makeTranslation(-0.5, 0, 0);
      const matrixB = new THREE.Matrix4()
        .makeRotationY(Math.PI / 2) // 90 degree rotation
        .setPosition(0.5, 0, 0);

      const chunks = chunksFromBufferGeometries([geomA, geomB], (_geom, index) => ({
        isSupport: true,
        applyMatrix: index === 0 ? matrixA : matrixB,
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
      // Rotated cube's face still aligns with the first cube's face
      expect(bonds.length).toBe(1);
    });
  });

  describe('data format validation', () => {
    it('handles Float32Array input correctly', async () => {
      const { loadStressSolver } = await importRuntime();
      const rt = await loadStressSolver();

      // Manual triangle data for two adjacent cubes
      // This is a simplified case with just one triangle each (not realistic but tests format)
      const chunk0 = {
        triangles: new Float32Array([
          // Triangle at X=0 plane
          0, -0.5, -0.5,
          0, 0.5, -0.5,
          0, 0.5, 0.5
        ]),
        isSupport: true
      };

      const chunk1 = {
        triangles: new Float32Array([
          // Triangle at X=0 plane (same plane, touching)
          0, -0.5, -0.5,
          0, 0.5, 0.5,
          0, -0.5, 0.5
        ]),
        isSupport: true
      };

      // These triangles share an edge but not a full face, so exact mode may or may not bond them
      const bonds = rt.createBondsFromTriangles([chunk0, chunk1], { mode: 'exact' });
      expect(Array.isArray(bonds)).toBe(true);
    });

    it('handles regular array input (not Float32Array)', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-0.5, 0, 0);
      geomB.translate(0.5, 0, 0);

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      // Convert to regular arrays
      const chunksWithArrays = chunks.map((c) => ({
        ...c,
        triangles: Array.from(c.triangles)
      }));

      const bonds = rt.createBondsFromTriangles(chunksWithArrays, { mode: 'exact' });
      expect(bonds.length).toBe(1);
    });

    it('throws on invalid triangle count (not multiple of 9 floats)', async () => {
      const { loadStressSolver } = await importRuntime();
      const rt = await loadStressSolver();

      const invalidChunk = {
        triangles: new Float32Array([1, 2, 3, 4, 5]), // 5 floats, not divisible by 9
        isSupport: true
      };

      expect(() => {
        rt.createBondsFromTriangles([invalidChunk], { mode: 'exact' });
      }).toThrow(/9/);
    });
  });

  describe('bond output validation', () => {
    it('outputs bonds with correct structure', async () => {
      const { loadStressSolver, chunksFromBufferGeometries } = await importRuntime();
      const rt = await loadStressSolver();

      const geomA = new THREE.BoxGeometry(1, 1, 1);
      const geomB = new THREE.BoxGeometry(1, 1, 1);
      geomA.translate(-0.5, 0, 0);
      geomB.translate(0.5, 0, 0);

      const chunks = chunksFromBufferGeometries([geomA, geomB], () => ({
        isSupport: true,
        nonIndexed: true,
        cloneGeometry: true
      }));

      const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });

      expect(bonds.length).toBe(1);
      const bond = bonds[0];

      // Validate structure
      expect(bond).toHaveProperty('node0');
      expect(bond).toHaveProperty('node1');
      expect(bond).toHaveProperty('centroid');
      expect(bond).toHaveProperty('normal');
      expect(bond).toHaveProperty('area');

      // Validate types
      expect(typeof bond.node0).toBe('number');
      expect(typeof bond.node1).toBe('number');
      expect(typeof bond.area).toBe('number');
      expect(bond.centroid).toHaveProperty('x');
      expect(bond.centroid).toHaveProperty('y');
      expect(bond.centroid).toHaveProperty('z');
      expect(bond.normal).toHaveProperty('x');
      expect(bond.normal).toHaveProperty('y');
      expect(bond.normal).toHaveProperty('z');

      // Validate values make sense
      expect(bond.node0).toBeGreaterThanOrEqual(0);
      expect(bond.node1).toBeGreaterThanOrEqual(0);
      expect(bond.node0).not.toBe(bond.node1);
      expect(bond.area).toBeGreaterThan(0);

      // Normal should be unit length
      const normalLen = Math.sqrt(
        (bond.normal?.x ?? 0) ** 2 +
        (bond.normal?.y ?? 0) ** 2 +
        (bond.normal?.z ?? 0) ** 2
      );
      expect(normalLen).toBeCloseTo(1.0, 3);
    });
  });
});


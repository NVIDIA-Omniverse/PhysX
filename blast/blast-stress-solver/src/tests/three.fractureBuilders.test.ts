/**
 * Tests for fracture builder functions (wall, floor, column, foundation)
 * and bond strength multipliers.
 *
 * Fragment builder tests require @dgreenheck/three-pinata.
 * Bond strength multiplier tests are always available (no deps).
 */
import { describe, it, expect, beforeAll } from 'vitest';
import * as THREE from 'three';

// Check synchronously at module load time so skipIf works
let pinataAvailable = false;
try {
  require.resolve('@dgreenheck/three-pinata');
  pinataAvailable = true;
} catch {
  pinataAvailable = false;
}

describe('fractureBuilders', () => {
  beforeAll(async () => {
    if (pinataAvailable) {
      const { ensurePinataLoaded } = await import('../three/pinataFracture');
      await ensurePinataLoaded();
    }
  });

  // ── Wall fragments ──────────────────────────────────────────

  describe('buildWallFragments (requires three-pinata)', () => {
    it.skipIf(!pinataAvailable)('produces fragments with wall type', async () => {
      const { buildWallFragments } = await import('../three/fractureBuilders');

      const fragments = buildWallFragments({
        span: 4, height: 2, thickness: 0.3,
        fragmentCount: 6,
      });

      expect(fragments.length).toBeGreaterThan(0);
      for (const f of fragments) {
        expect(f.fragmentType).toBe('wall');
        expect(f.isSupport).toBe(false);
        expect(f.geometry).toBeInstanceOf(THREE.BufferGeometry);
        expect(f.halfExtents.x).toBeGreaterThan(0);
        expect(f.halfExtents.y).toBeGreaterThan(0);
        expect(f.halfExtents.z).toBeGreaterThan(0);
      }
    });

    it.skipIf(!pinataAvailable)('positions fragments at specified baseY', async () => {
      const { buildWallFragments } = await import('../three/fractureBuilders');

      const baseY = 5.0;
      const height = 3.0;
      const fragments = buildWallFragments({
        span: 4, height, thickness: 0.3,
        fragmentCount: 4,
        baseY,
      });

      // All fragments should be above baseY and below baseY + height
      for (const f of fragments) {
        expect(f.worldPosition.y).toBeGreaterThanOrEqual(baseY - 0.01);
        expect(f.worldPosition.y).toBeLessThanOrEqual(baseY + height + 0.01);
      }
    });

    it.skipIf(!pinataAvailable)('applies rotation for side walls', async () => {
      const { buildWallFragments } = await import('../three/fractureBuilders');

      const unrotated = buildWallFragments({
        span: 6, height: 2, thickness: 0.3,
        fragmentCount: 4,
        rotationY: 0,
      });

      const rotated = buildWallFragments({
        span: 6, height: 2, thickness: 0.3,
        fragmentCount: 4,
        rotationY: Math.PI * 0.5,
      });

      // Unrotated wall spans X; rotated wall spans Z
      // Check that rotated fragments have different X/Z distribution
      const unrotMaxX = Math.max(...unrotated.map((f) => Math.abs(f.worldPosition.x)));
      const rotMaxZ = Math.max(...rotated.map((f) => Math.abs(f.worldPosition.z)));

      // Both should have significant extent in their primary direction
      expect(unrotMaxX).toBeGreaterThan(0.5);
      expect(rotMaxZ).toBeGreaterThan(0.5);
    });
  });

  // ── Floor fragments ─────────────────────────────────────────

  describe('buildFloorFragments (requires three-pinata)', () => {
    it.skipIf(!pinataAvailable)('produces fragments with floor type', async () => {
      const { buildFloorFragments } = await import('../three/fractureBuilders');

      const centerY = 10.0;
      const fragments = buildFloorFragments({
        spanX: 6, spanZ: 6, thickness: 0.35,
        fragmentCount: 6,
        centerY,
      });

      expect(fragments.length).toBeGreaterThan(0);
      for (const f of fragments) {
        expect(f.fragmentType).toBe('floor');
        expect(f.isSupport).toBe(false);
        // Floor fragments should be near centerY
        expect(Math.abs(f.worldPosition.y - centerY)).toBeLessThan(1.0);
      }
    });
  });

  // ── Column fragments ────────────────────────────────────────

  describe('buildColumnFragments (requires three-pinata)', () => {
    it.skipIf(!pinataAvailable)('produces fragments with column type', async () => {
      const { buildColumnFragments } = await import('../three/fractureBuilders');

      const fragments = buildColumnFragments({
        sizeX: 0.8, sizeZ: 0.8, height: 3.0,
        fragmentCount: 4,
        baseY: 1.0,
      });

      expect(fragments.length).toBeGreaterThan(0);
      for (const f of fragments) {
        expect(f.fragmentType).toBe('column');
        expect(f.isSupport).toBe(false);
        // Column fragments should be between baseY and baseY + height
        expect(f.worldPosition.y).toBeGreaterThanOrEqual(0.9);
        expect(f.worldPosition.y).toBeLessThanOrEqual(4.1);
      }
    });
  });

  // ── Grid foundation ─────────────────────────────────────────

  describe('buildGridFoundationFragments', () => {
    it('creates a grid of support fragments', async () => {
      const { buildGridFoundationFragments } = await import('../three/fractureBuilders');

      const { fragments, foundationTopY } = buildGridFoundationFragments({
        width: 10, depth: 10, height: 0.5,
      });

      // Should create multiple tiles
      expect(fragments.length).toBeGreaterThan(1);
      expect(foundationTopY).toBeCloseTo(0.501, 2); // groundClearance + height

      for (const f of fragments) {
        expect(f.isSupport).toBe(true);
        expect(f.fragmentType).toBe('foundation');
        expect(f.geometry).toBeInstanceOf(THREE.BufferGeometry);
        // Should be within footprint
        expect(Math.abs(f.worldPosition.x)).toBeLessThanOrEqual(5.01);
        expect(Math.abs(f.worldPosition.z)).toBeLessThanOrEqual(5.01);
      }
    });

    it('scales tile count with structure size', async () => {
      const { buildGridFoundationFragments } = await import('../three/fractureBuilders');

      const small = buildGridFoundationFragments({ width: 5, depth: 5, height: 0.2 });
      const large = buildGridFoundationFragments({ width: 40, depth: 40, height: 0.5 });

      expect(large.fragments.length).toBeGreaterThan(small.fragments.length);
    });
  });

  // ── Bond strength multipliers ───────────────────────────────

  describe('getBondStrengthMultiplier', () => {
    it('returns correct multipliers for type pairs', async () => {
      const { getBondStrengthMultiplier } = await import('../three/fractureBuilders');

      expect(getBondStrengthMultiplier('column', 'column')).toBe(4.0);
      expect(getBondStrengthMultiplier('column', 'wall')).toBe(2.5);
      expect(getBondStrengthMultiplier('wall', 'column')).toBe(2.5);
      expect(getBondStrengthMultiplier('column', 'floor')).toBe(2.5);
      expect(getBondStrengthMultiplier('floor', 'floor')).toBe(2.0);
      expect(getBondStrengthMultiplier('floor', 'wall')).toBe(1.5);
      expect(getBondStrengthMultiplier('wall', 'floor')).toBe(1.5);
      expect(getBondStrengthMultiplier('wall', 'wall')).toBe(1.0);
      expect(getBondStrengthMultiplier(undefined, undefined)).toBe(1.0);
      expect(getBondStrengthMultiplier('wall', undefined)).toBe(1.0);
    });
  });

  describe('applyBondStrengthMultipliers', () => {
    it('scales bond areas by fragment type', async () => {
      const { applyBondStrengthMultipliers } = await import('../three/fractureBuilders');

      const bonds = [
        { node0: 0, node1: 1, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 1.0 },
        { node0: 2, node1: 3, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 0, y: 1, z: 0 }, area: 1.0 },
        { node0: 0, node1: 2, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 0, y: 0, z: 1 }, area: 1.0 },
      ];
      const types = ['column', 'column', 'wall', 'wall'] as const;

      const result = applyBondStrengthMultipliers(bonds, [...types]);

      // column-column: 4x
      expect(result[0].area).toBe(4.0);
      // wall-wall: 1x
      expect(result[1].area).toBe(1.0);
      // column-wall: 2.5x
      expect(result[2].area).toBe(2.5);
    });

    it('preserves bonds with no type change', async () => {
      const { applyBondStrengthMultipliers } = await import('../three/fractureBuilders');

      const bond = { node0: 0, node1: 1, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 2.5 };
      const result = applyBondStrengthMultipliers([bond], ['wall', 'wall']);

      // wall-wall = 1.0x, so area unchanged; bond object identity preserved
      expect(result[0]).toBe(bond);
      expect(result[0].area).toBe(2.5);
    });
  });
});

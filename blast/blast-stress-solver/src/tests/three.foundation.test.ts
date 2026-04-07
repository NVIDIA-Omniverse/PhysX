/**
 * Tests for foundation/support plate generation.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { buildFoundationFragments, buildSingleFoundationPlate } from '../three/foundation';

describe('buildFoundationFragments', () => {
  it('generates the expected number of segments', () => {
    const { fragments } = buildFoundationFragments({
      span: { x: 6, y: 3, z: 0.32 },
      segments: 6,
    });
    expect(fragments.length).toBe(6);
  });

  it('auto-computes segment count when not specified', () => {
    const { fragments } = buildFoundationFragments({
      span: { x: 6, y: 3, z: 0.32 },
    });
    // max(6, round(6 / 0.5)) = 12
    expect(fragments.length).toBe(12);
  });

  it('all fragments are marked as support', () => {
    const { fragments } = buildFoundationFragments({
      span: { x: 6, y: 3, z: 0.32 },
      segments: 4,
    });
    for (const f of fragments) {
      expect(f.isSupport).toBe(true);
    }
  });

  it('fragments have valid geometry', () => {
    const { fragments } = buildFoundationFragments({
      span: { x: 6, y: 3, z: 0.32 },
      segments: 4,
    });
    for (const f of fragments) {
      expect(f.geometry).toBeInstanceOf(THREE.BufferGeometry);
      expect(f.halfExtents.x).toBeGreaterThan(0);
      expect(f.halfExtents.y).toBeGreaterThan(0);
      expect(f.halfExtents.z).toBeGreaterThan(0);
    }
  });

  it('foundation top is above ground clearance', () => {
    const { foundationTopY } = buildFoundationFragments({
      span: { x: 6, y: 3, z: 0.32 },
      groundClearance: 0.01,
    });
    expect(foundationTopY).toBeGreaterThan(0.01);
  });

  it('foundation height respects maxHeight', () => {
    const { fragments } = buildFoundationFragments({
      span: { x: 6, y: 100, z: 0.32 }, // very tall -> heightRatio would give large height
      maxHeight: 0.1,
      segments: 1,
    });
    expect(fragments[0].halfExtents.y * 2).toBeLessThanOrEqual(0.1 + 1e-6);
  });

  it('segments span the full width', () => {
    const { fragments } = buildFoundationFragments({
      span: { x: 4, y: 2, z: 1 },
      segments: 4,
    });
    // Each segment should be 1.0 wide
    for (const f of fragments) {
      expect(f.halfExtents.x * 2).toBeCloseTo(1.0, 5);
    }
    // First and last should be at the edges
    const xs = fragments.map((f) => f.worldPosition.x).sort((a, b) => a - b);
    expect(xs[0]).toBeCloseTo(-1.5, 5);
    expect(xs[xs.length - 1]).toBeCloseTo(1.5, 5);
  });
});

describe('buildSingleFoundationPlate', () => {
  it('creates a single support fragment', () => {
    const { fragment } = buildSingleFoundationPlate({
      span: { x: 4, y: 3, z: 2 },
    });
    expect(fragment.isSupport).toBe(true);
    expect(fragment.geometry).toBeInstanceOf(THREE.BufferGeometry);
  });

  it('plate covers the full span', () => {
    const { fragment } = buildSingleFoundationPlate({
      span: { x: 4, y: 3, z: 2 },
    });
    expect(fragment.halfExtents.x * 2).toBeCloseTo(4, 5);
    expect(fragment.halfExtents.z * 2).toBeCloseTo(2, 5);
  });
});

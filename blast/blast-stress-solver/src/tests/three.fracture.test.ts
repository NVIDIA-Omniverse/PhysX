/**
 * Tests for proximity-based bond detection and area normalization.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import {
  computeBondsFromFragments,
  normalizeBondAreasByAxis,
  uniformizeBondAreasByAxis,
  projectExtentsOnAxisWorld,
  type FragmentInfo,
} from '../three/fracture';

function makeBoxFragment(
  position: { x: number; y: number; z: number },
  halfExtents: { x: number; y: number; z: number },
  isSupport = false,
): FragmentInfo {
  const geometry = new THREE.BoxGeometry(
    halfExtents.x * 2,
    halfExtents.y * 2,
    halfExtents.z * 2,
  );
  return { worldPosition: position, halfExtents, geometry, isSupport };
}

describe('computeBondsFromFragments', () => {
  it('detects bonds between two adjacent boxes', () => {
    // Two unit cubes side by side along X
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const bonds = computeBondsFromFragments(fragments);
    expect(bonds.length).toBe(1);
    expect(bonds[0].node0).toBe(0);
    expect(bonds[0].node1).toBe(1);
    expect(bonds[0].area).toBeGreaterThan(0);
  });

  it('detects no bonds between separated boxes', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 5, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const bonds = computeBondsFromFragments(fragments);
    expect(bonds.length).toBe(0);
  });

  it('detects bonds in a 2x2 grid of boxes', () => {
    // Four boxes in a 2x2 grid: (0,0), (1,0), (0,1), (1,1)
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 0, y: 1, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 1, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const bonds = computeBondsFromFragments(fragments);
    // 4 axis-aligned bonds (H at y=0, H at y=1, two vertical) plus
    // 2 diagonal bonds (corner-to-corner overlap detected by proximity)
    expect(bonds.length).toBeGreaterThanOrEqual(4);
    expect(bonds.length).toBeLessThanOrEqual(6);
  });

  it('skips support-to-support bonds by default', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, true),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, true),
    ];
    const bonds = computeBondsFromFragments(fragments);
    expect(bonds.length).toBe(0);
  });

  it('allows support-to-support bonds when disabled', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, true),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, true),
    ];
    const bonds = computeBondsFromFragments(fragments, { skipSupportToSupport: false });
    expect(bonds.length).toBe(1);
  });

  it('detects support-to-fragment bonds', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, true),
      makeBoxFragment({ x: 0, y: 1, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, false),
    ];
    const bonds = computeBondsFromFragments(fragments);
    expect(bonds.length).toBe(1);
  });

  it('bond area is approximately correct for unit cube contact', () => {
    // Two unit cubes touching: contact area should be ~1.0 (1x1 face)
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const bonds = computeBondsFromFragments(fragments);
    expect(bonds[0].area).toBeCloseTo(1.0, 1);
  });

  it('bond centroid is between the two fragments', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const bonds = computeBondsFromFragments(fragments);
    // Centroid should be approximately at x=0.5 (between the two)
    expect(bonds[0].centroid.x).toBeCloseTo(0.5, 0);
    expect(bonds[0].centroid.y).toBeCloseTo(0, 0);
    expect(bonds[0].centroid.z).toBeCloseTo(0, 0);
  });

  it('bond normal points from first to second fragment', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const bonds = computeBondsFromFragments(fragments);
    // Normal should point along +X
    expect(bonds[0].normal.x).toBeCloseTo(1.0, 1);
    expect(Math.abs(bonds[0].normal.y)).toBeLessThan(0.1);
    expect(Math.abs(bonds[0].normal.z)).toBeLessThan(0.1);
  });

  it('handles empty fragment array', () => {
    expect(computeBondsFromFragments([])).toEqual([]);
  });

  it('handles single fragment', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    expect(computeBondsFromFragments(fragments)).toEqual([]);
  });
});

describe('projectExtentsOnAxisWorld', () => {
  it('projects a unit cube correctly along X', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const result = projectExtentsOnAxisWorld(
      geometry,
      { x: 2, y: 0, z: 0 },
      { x: 1, y: 0, z: 0 },
    );
    expect(result.min).toBeCloseTo(1.5, 5);
    expect(result.max).toBeCloseTo(2.5, 5);
  });
});

describe('normalizeBondAreasByAxis', () => {
  it('preserves bond count', () => {
    const bonds = [
      { node0: 0, node1: 1, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 0.5 },
      { node0: 1, node1: 2, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 0, y: 1, z: 0 }, area: 0.3 },
    ];
    const result = normalizeBondAreasByAxis(bonds, { x: 2, y: 3, z: 1 });
    expect(result.length).toBe(2);
  });

  it('scales X-normal bonds to match YZ cross-section', () => {
    const bonds = [
      { node0: 0, node1: 1, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 1.0 },
    ];
    const result = normalizeBondAreasByAxis(bonds, { x: 2, y: 3, z: 4 });
    // Target for X-normal = y*z = 3*4 = 12, single bond should get all of it
    expect(result[0].area).toBeCloseTo(12, 5);
  });

  it('distributes area proportionally among bonds on same axis', () => {
    const bonds = [
      { node0: 0, node1: 1, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 1.0 },
      { node0: 2, node1: 3, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 3.0 },
    ];
    const result = normalizeBondAreasByAxis(bonds, { x: 2, y: 3, z: 4 });
    // Total target = 12. Original ratio 1:3 preserved.
    expect(result[0].area).toBeCloseTo(3, 5);
    expect(result[1].area).toBeCloseTo(9, 5);
  });
});

describe('uniformizeBondAreasByAxis', () => {
  it('gives equal area to all bonds on same axis', () => {
    const bonds = [
      { node0: 0, node1: 1, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 0.1 },
      { node0: 2, node1: 3, centroid: { x: 0, y: 0, z: 0 }, normal: { x: 1, y: 0, z: 0 }, area: 5.0 },
    ];
    const result = uniformizeBondAreasByAxis(bonds, { x: 2, y: 3, z: 4 });
    // Both X-normal bonds get target/count = 12/2 = 6
    expect(result[0].area).toBeCloseTo(6, 5);
    expect(result[1].area).toBeCloseTo(6, 5);
  });
});

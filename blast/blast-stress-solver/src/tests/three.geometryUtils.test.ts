/**
 * Tests for geometry preparation utilities.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import {
  ensurePlainAttributes,
  ensureNonIndexed,
  prepareGeometryForFracture,
  recenterGeometry,
} from '../three/geometryUtils';

describe('ensurePlainAttributes', () => {
  it('preserves already-plain Float32Array attributes', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const before = geometry.getAttribute('position') as THREE.BufferAttribute;
    expect(before.array).toBeInstanceOf(Float32Array);
    ensurePlainAttributes(geometry);
    const after = geometry.getAttribute('position') as THREE.BufferAttribute;
    expect(after.array).toBeInstanceOf(Float32Array);
    expect(after.count).toBe(before.count);
  });

  it('returns the geometry for chaining', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const result = ensurePlainAttributes(geometry);
    expect(result).toBe(geometry);
  });
});

describe('ensureNonIndexed', () => {
  it('converts indexed geometry to non-indexed', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    // BoxGeometry is indexed by default
    expect(geometry.index).not.toBeNull();
    const result = ensureNonIndexed(geometry);
    expect(result.index).toBeNull();
    // Non-indexed has more vertices (each face has its own)
    const pos = result.getAttribute('position') as THREE.BufferAttribute;
    expect(pos.count).toBeGreaterThan(0);
  });

  it('returns same geometry if already non-indexed', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1).toNonIndexed();
    const result = ensureNonIndexed(geometry);
    expect(result).toBe(geometry);
  });
});

describe('prepareGeometryForFracture', () => {
  it('produces non-indexed geometry with plain attributes and normals', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const result = prepareGeometryForFracture(geometry);

    // Non-indexed
    expect(result.index).toBeNull();

    // Has position with Float32Array
    const pos = result.getAttribute('position') as THREE.BufferAttribute;
    expect(pos).toBeTruthy();
    expect(pos.array).toBeInstanceOf(Float32Array);

    // Has normals
    const normal = result.getAttribute('normal') as THREE.BufferAttribute;
    expect(normal).toBeTruthy();
    expect(normal.count).toBe(pos.count);
  });

  it('does not mutate the original geometry', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const originalIndex = geometry.index;
    prepareGeometryForFracture(geometry);
    expect(geometry.index).toBe(originalIndex);
  });
});

describe('recenterGeometry', () => {
  it('centers geometry at the origin', () => {
    const geometry = new THREE.BoxGeometry(2, 2, 2);
    // Translate so it's off-center
    geometry.translate(5, 3, 1);
    const { offset } = recenterGeometry(geometry);

    // The offset should be the old center
    expect(offset.x).toBeCloseTo(5, 5);
    expect(offset.y).toBeCloseTo(3, 5);
    expect(offset.z).toBeCloseTo(1, 5);

    // After recentering, bounding box should be centered at origin
    geometry.computeBoundingBox();
    const bbox = geometry.boundingBox as THREE.Box3;
    const center = new THREE.Vector3();
    bbox.getCenter(center);
    expect(center.x).toBeCloseTo(0, 5);
    expect(center.y).toBeCloseTo(0, 5);
    expect(center.z).toBeCloseTo(0, 5);
  });

  it('returns the same geometry object', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const { geometry: result } = recenterGeometry(geometry);
    expect(result).toBe(geometry);
  });
});

/**
 * Geometry preparation utilities for fracturing pipelines.
 *
 * Handles common geometry issues: interleaved attributes, indexed buffers,
 * missing normals, and off-center geometry.
 */
import * as THREE from 'three';

/**
 * Ensure all standard attributes (position, normal, uv) are plain Float32Array
 * BufferAttributes rather than interleaved. Mutates the geometry in-place and
 * returns it for chaining.
 */
export function ensurePlainAttributes(geometry: THREE.BufferGeometry): THREE.BufferGeometry {
  const names = ['position', 'normal', 'uv'] as const;
  for (const name of names) {
    const attr = geometry.getAttribute(name) as THREE.BufferAttribute | undefined;
    if (!attr) continue;
    // Skip if already a plain Float32Array
    if (attr.array instanceof Float32Array) continue;

    const count = attr.count;
    const itemSize = attr.itemSize;
    const data = new Float32Array(count * itemSize);
    for (let i = 0; i < count; i++) {
      if (itemSize >= 1) data[i * itemSize] = attr.getX(i);
      if (itemSize >= 2) data[i * itemSize + 1] = attr.getY(i);
      if (itemSize >= 3) data[i * itemSize + 2] = attr.getZ(i);
    }
    geometry.setAttribute(name, new THREE.BufferAttribute(data, itemSize));
  }
  return geometry;
}

/**
 * Convert indexed geometry to non-indexed. Returns the geometry (mutated or replaced).
 */
export function ensureNonIndexed(geometry: THREE.BufferGeometry): THREE.BufferGeometry {
  return geometry.index ? geometry.toNonIndexed() : geometry;
}

/**
 * Full preparation for fracturing: ensure non-indexed, plain attributes, and normals.
 * Returns a new geometry (the input is not mutated).
 */
export function prepareGeometryForFracture(geometry: THREE.BufferGeometry): THREE.BufferGeometry {
  let prepared = geometry.clone();
  prepared = ensureNonIndexed(prepared);
  ensurePlainAttributes(prepared);
  if (!prepared.getAttribute('normal')) {
    prepared.computeVertexNormals();
  }
  return prepared;
}

/**
 * Re-center geometry around its bounding-box center. Mutates the geometry
 * in-place. Returns the offset that was applied (the original center).
 */
export function recenterGeometry(
  geometry: THREE.BufferGeometry,
): { geometry: THREE.BufferGeometry; offset: THREE.Vector3 } {
  geometry.computeBoundingBox();
  const bbox = geometry.boundingBox as THREE.Box3;
  const center = new THREE.Vector3();
  bbox.getCenter(center);
  geometry.translate(-center.x, -center.y, -center.z);
  return { geometry, offset: center };
}

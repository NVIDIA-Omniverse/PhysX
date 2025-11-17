import * as THREE from 'three';

export type ChunkInput = { triangles: Float32Array; isSupport?: boolean };

export function cubeTriangles(offset: number): Float32Array {
  const geom = new THREE.BoxGeometry(1, 1, 1);
  geom.translate(offset, 0, 0);
  const nonIndexed = geom.toNonIndexed() ?? geom;
  const position = nonIndexed.getAttribute('position');
  if (!position) {
    throw new Error('geometry missing position attribute');
  }
  return Float32Array.from(position.array as ArrayLike<number>);
}

export function touchingCubeChunks(): ChunkInput[] {
  return [
    { triangles: cubeTriangles(-0.5), isSupport: true },
    { triangles: cubeTriangles(0.5), isSupport: true }
  ];
}

export function separatedCubeChunks(gap: number): ChunkInput[] {
  const half = gap * 0.5;
  return [
    { triangles: cubeTriangles(-0.5 - half), isSupport: true },
    { triangles: cubeTriangles(0.5 + half), isSupport: true }
  ];
}

export function preciseFaceSharingChunks(): ChunkInput[] {
  return touchingCubeChunks();
}

export function preciseFaceSharingGeometry(overlap = 0.0): THREE.BufferGeometry[] {
  const geomA = new THREE.BoxGeometry(1, 1, 1);
  const geomB = new THREE.BoxGeometry(1, 1, 1);
  geomA.translate(-0.5 - overlap * 0.5, 0, 0);
  geomB.translate(0.5 + overlap * 0.5, 0, 0);
  return [geomA, geomB];
}

export const preciseFaceBond = {
  nodes: [0, 1],
  area: 1.0,
  centroid: { x: 0, y: 0, z: 0 },
  normalX: 1
};

export function stackedCubeChunks(columns: number, rows: number, spacing = 1.0): ChunkInput[] {
  const chunks: ChunkInput[] = [];
  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < columns; col++) {
      const offset = col * spacing;
      const chunk: ChunkInput = { triangles: cubeTriangles(offset), isSupport: row === rows - 1 };
      chunks.push(chunk);
    }
  }
  return chunks;
}

export function gridBoxGeometries(columns: number, rows: number, spacing = 1.0): THREE.BufferGeometry[] {
  const geoms: THREE.BufferGeometry[] = [];
  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < columns; col++) {
      const geom = new THREE.BoxGeometry(1, 1, 1);
      geom.translate(col * spacing, row * spacing, 0);
      geoms.push(geom);
    }
  }
  return geoms;
}

export function touchingBoxGeometries(): THREE.BufferGeometry[] {
  const geomA = new THREE.BoxGeometry(1, 1, 1);
  const geomB = new THREE.BoxGeometry(1, 1, 1);
  geomA.translate(-0.5, 0, 0);
  geomB.translate(0.5, 0, 0);
  return [geomA, geomB];
}

export function separatedBoxGeometries(gap: number): THREE.BufferGeometry[] {
  const geomA = new THREE.BoxGeometry(1, 1, 1);
  const geomB = new THREE.BoxGeometry(1, 1, 1);
  const offset = 0.5 + gap * 0.5;
  geomA.translate(-offset, 0, 0);
  geomB.translate(offset, 0, 0);
  return [geomA, geomB];
}

export function almostEqual(a: number, b: number, epsilon = 1e-3): boolean {
  return Math.abs(a - b) <= epsilon;
}



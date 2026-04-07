import * as THREE from 'three';
import type { DestructibleCore } from '../rapier/types';

/** Debug line from solver (local space) */
export type DebugLine = {
  p0: { x: number; y: number; z: number };
  p1: { x: number; y: number; z: number };
  color0: number;
  color1: number;
};

const GRID_CELL_SIZE = 2.0;

/**
 * Helper class for rendering stress solver debug lines.
 * Encapsulates all caching and GPU buffer management for optimal performance.
 *
 * Usage:
 *   const helper = new SolverDebugLinesHelper();
 *   scene.add(helper.object);
 *   // In render loop:
 *   helper.update(core, core.getSolverDebugLines(), visible);
 *   // On cleanup:
 *   helper.dispose();
 */
export class SolverDebugLinesHelper {
  // THREE.js objects
  private geometry: THREE.BufferGeometry;
  private material: THREE.LineBasicMaterial;
  public readonly object: THREE.LineSegments;

  // Pre-allocated GPU buffers
  private positions: Float32Array;
  private colors: Float32Array;
  private maxLines: number;

  // Spatial grid for chunk lookups (built once per core)
  private chunkGrid: Map<string, number[]> | null = null;
  private chunkGridCoreId = -1;

  // Line-to-chunk cache (computed once, permanent until line count changes)
  private lineToChunk: number[] = [];
  private lineToChunkCount = -1;

  // Node-to-actor cache (invalidated when actor count changes)
  private nodeToActorCache = new Map<number, number>();
  private nodeToActorVersion = -1;

  // Pooled pose objects for actor transforms
  private readonly actorPosePool: Array<{ t: THREE.Vector3; q: THREE.Quaternion }> = [];
  private readonly actorPoseMap = new Map<number, { t: THREE.Vector3; q: THREE.Quaternion }>();

  // Reusable transform objects
  private readonly v0 = new THREE.Vector3();
  private readonly v1 = new THREE.Vector3();
  private readonly rootPoseT = new THREE.Vector3();
  private readonly rootPoseQ = new THREE.Quaternion();

  constructor(initialMaxLines = 50000) {
    this.maxLines = initialMaxLines;
    this.positions = new Float32Array(initialMaxLines * 6);
    this.colors = new Float32Array(initialMaxLines * 6);

    this.geometry = new THREE.BufferGeometry();
    const posAttr = new THREE.BufferAttribute(this.positions, 3);
    const colAttr = new THREE.BufferAttribute(this.colors, 3);
    posAttr.setUsage(THREE.DynamicDrawUsage);
    colAttr.setUsage(THREE.DynamicDrawUsage);
    this.geometry.setAttribute('position', posAttr);
    this.geometry.setAttribute('color', colAttr);

    this.material = new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.95,
      depthTest: false,
    });

    this.object = new THREE.LineSegments(this.geometry, this.material);
    this.object.visible = false;
    this.object.frustumCulled = false;
  }

  /**
   * Update debug lines each frame.
   * @param core The destructible core instance
   * @param lines Debug lines from solver (local space)
   * @param visible Whether to show the lines
   */
  update(core: DestructibleCore, lines: DebugLine[], visible: boolean): void {
    if (!visible) {
      this.object.visible = false;
      return;
    }

    const lineCount = lines.length;
    if (lineCount === 0) {
      this.object.visible = false;
      return;
    }

    // Ensure buffers are large enough
    this.ensureBufferCapacity(lineCount);

    // Ensure caches are up to date
    this.ensureChunkGrid(core);
    this.ensureLineToChunkCache(core, lines);
    this.updateNodeToActorCache(core);
    this.updateActorPoses(core);

    // Transform and write lines to GPU buffers
    this.transformLines(core, lines, lineCount);

    // Mark buffers for upload
    const posAttr = this.geometry.getAttribute('position') as THREE.BufferAttribute;
    const colAttr = this.geometry.getAttribute('color') as THREE.BufferAttribute;
    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    this.geometry.setDrawRange(0, lineCount * 2);

    this.object.visible = true;
  }

  /**
   * Invalidate all caches. Call when core is disposed/reset.
   */
  invalidate(): void {
    this.chunkGrid = null;
    this.chunkGridCoreId = -1;
    this.lineToChunk = [];
    this.lineToChunkCount = -1;
    this.nodeToActorCache.clear();
    this.nodeToActorVersion = -1;
    this.actorPoseMap.clear();
  }

  /**
   * Dispose THREE.js resources.
   */
  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
    this.invalidate();
  }

  // =========================================================================
  // Private: Buffer management
  // =========================================================================

  private ensureBufferCapacity(lineCount: number): void {
    if (lineCount <= this.maxLines) return;

    // Double capacity to avoid frequent reallocations
    this.maxLines = Math.max(lineCount, this.maxLines * 2);
    this.positions = new Float32Array(this.maxLines * 6);
    this.colors = new Float32Array(this.maxLines * 6);

    const posAttr = new THREE.BufferAttribute(this.positions, 3);
    const colAttr = new THREE.BufferAttribute(this.colors, 3);
    posAttr.setUsage(THREE.DynamicDrawUsage);
    colAttr.setUsage(THREE.DynamicDrawUsage);
    this.geometry.setAttribute('position', posAttr);
    this.geometry.setAttribute('color', colAttr);
  }

  // =========================================================================
  // Private: Chunk grid (built once per core)
  // =========================================================================

  private ensureChunkGrid(core: DestructibleCore): void {
    const coreId = core.chunks.length;
    if (this.chunkGrid && this.chunkGridCoreId === coreId) return;

    this.chunkGrid = new Map();
    for (let i = 0; i < core.chunks.length; i++) {
      const c = core.chunks[i];
      const key = this.getGridKey(c.baseLocalOffset.x, c.baseLocalOffset.y, c.baseLocalOffset.z);
      let arr = this.chunkGrid.get(key);
      if (!arr) {
        arr = [];
        this.chunkGrid.set(key, arr);
      }
      arr.push(i);
    }
    this.chunkGridCoreId = coreId;
  }

  private getGridKey(x: number, y: number, z: number): string {
    const cx = Math.floor(x / GRID_CELL_SIZE);
    const cy = Math.floor(y / GRID_CELL_SIZE);
    const cz = Math.floor(z / GRID_CELL_SIZE);
    return `${cx},${cy},${cz}`;
  }

  // =========================================================================
  // Private: Line-to-chunk cache (permanent, only rebuilt when line count changes)
  // =========================================================================

  private ensureLineToChunkCache(core: DestructibleCore, lines: DebugLine[]): void {
    if (this.lineToChunkCount === lines.length) return;

    // Rebuild cache - this only happens when bonds break (line count changes)
    this.lineToChunk = new Array(lines.length);
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      const mx = (line.p0.x + line.p1.x) * 0.5;
      const my = (line.p0.y + line.p1.y) * 0.5;
      const mz = (line.p0.z + line.p1.z) * 0.5;
      this.lineToChunk[i] = this.findNearestChunkIndex(core, mx, my, mz);
    }
    this.lineToChunkCount = lines.length;
  }

  private findNearestChunkIndex(core: DestructibleCore, mx: number, my: number, mz: number): number {
    if (!this.chunkGrid) return -1;

    let bestNode = -1;
    let bestD2 = Infinity;

    const cx = Math.floor(mx / GRID_CELL_SIZE);
    const cy = Math.floor(my / GRID_CELL_SIZE);
    const cz = Math.floor(mz / GRID_CELL_SIZE);

    // Search 3x3x3 neighborhood
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        for (let dz = -1; dz <= 1; dz++) {
          const key = `${cx + dx},${cy + dy},${cz + dz}`;
          const indices = this.chunkGrid.get(key);
          if (!indices) continue;

          for (const i of indices) {
            const c = core.chunks[i];
            const ddx = c.baseLocalOffset.x - mx;
            const ddy = c.baseLocalOffset.y - my;
            const ddz = c.baseLocalOffset.z - mz;
            const d2 = ddx * ddx + ddy * ddy + ddz * ddz;
            if (d2 < bestD2) {
              bestD2 = d2;
              bestNode = i;
            }
          }
        }
      }
    }

    return bestNode;
  }

  // =========================================================================
  // Private: Node-to-actor cache (updated when actor structure changes)
  // =========================================================================

  private updateNodeToActorCache(core: DestructibleCore): void {
    const currentVersion = core.actorMap.size;
    if (this.nodeToActorVersion === currentVersion) return;

    this.nodeToActorCache.clear();
    try {
      const solver = core.solver as unknown as {
        actors?: () => Array<{ actorIndex: number; nodes: number[] }>;
      };
      const actors = solver.actors?.() ?? [];
      for (const a of actors) {
        const nodes = Array.isArray(a.nodes) ? a.nodes : [];
        for (const n of nodes) {
          this.nodeToActorCache.set(n, a.actorIndex);
        }
      }
    } catch {
      // Ignore errors from solver
    }
    this.nodeToActorVersion = currentVersion;
  }

  // =========================================================================
  // Private: Actor poses (updated every frame)
  // =========================================================================

  private updateActorPoses(core: DestructibleCore): void {
    this.actorPoseMap.clear();
    let poseIndex = 0;

    for (const [actorIndex, { bodyHandle }] of core.actorMap.entries()) {
      const body = core.world.getRigidBody(bodyHandle);
      if (!body) continue;

      const tr = body.translation();
      const rot = body.rotation();
      const pose = this.getPooledPose(poseIndex++);
      pose.t.set(tr.x, tr.y, tr.z);
      pose.q.set(rot.x, rot.y, rot.z, rot.w);
      this.actorPoseMap.set(actorIndex, pose);
    }

    // Update root pose
    const rootBody = core.world.getRigidBody(core.rootBodyHandle);
    if (rootBody) {
      const rt = rootBody.translation();
      const rr = rootBody.rotation();
      this.rootPoseT.set(rt.x, rt.y, rt.z);
      this.rootPoseQ.set(rr.x, rr.y, rr.z, rr.w);
    } else {
      this.rootPoseT.set(0, 0, 0);
      this.rootPoseQ.identity();
    }
  }

  private getPooledPose(index: number): { t: THREE.Vector3; q: THREE.Quaternion } {
    if (index < this.actorPosePool.length) {
      return this.actorPosePool[index];
    }
    const pose = { t: new THREE.Vector3(), q: new THREE.Quaternion() };
    this.actorPosePool.push(pose);
    return pose;
  }

  // =========================================================================
  // Private: Transform lines to world space and write to GPU buffers
  // =========================================================================

  private transformLines(_core: DestructibleCore, lines: DebugLine[], lineCount: number): void {
    for (let i = 0; i < lineCount; i++) {
      const line = lines[i];

      // Use cached line-to-chunk mapping (O(1) array access)
      const chunkIndex = this.lineToChunk[i];
      const actorIdx = chunkIndex >= 0 ? this.nodeToActorCache.get(chunkIndex) : undefined;

      // Get pose (actor or root fallback)
      const pose = actorIdx != null ? this.actorPoseMap.get(actorIdx) : null;
      const poseT = pose ? pose.t : this.rootPoseT;
      const poseQ = pose ? pose.q : this.rootPoseQ;

      // Transform endpoints
      this.v0.set(line.p0.x, line.p0.y, line.p0.z).applyQuaternion(poseQ).add(poseT);
      this.v1.set(line.p1.x, line.p1.y, line.p1.z).applyQuaternion(poseQ).add(poseT);

      // Write positions
      const base = i * 6;
      this.positions[base] = this.v0.x;
      this.positions[base + 1] = this.v0.y;
      this.positions[base + 2] = this.v0.z;
      this.positions[base + 3] = this.v1.x;
      this.positions[base + 4] = this.v1.y;
      this.positions[base + 5] = this.v1.z;

      // Write colors (inline extraction)
      const c0 = line.color0;
      const c1 = line.color1 ?? c0;
      this.colors[base] = ((c0 >> 16) & 0xff) / 255;
      this.colors[base + 1] = ((c0 >> 8) & 0xff) / 255;
      this.colors[base + 2] = (c0 & 0xff) / 255;
      this.colors[base + 3] = ((c1 >> 16) & 0xff) / 255;
      this.colors[base + 4] = ((c1 >> 8) & 0xff) / 255;
      this.colors[base + 5] = (c1 & 0xff) / 255;
    }
  }
}

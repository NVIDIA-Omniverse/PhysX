import * as THREE from 'three';

type RapierWorld = {
  debugRender?: (filterFlags?: number) => { vertices: Float32Array; colors: Float32Array; free?: () => void } | null;
};

export type RapierDebugRendererOptions = {
  enabled?: boolean;
  geometry?: THREE.BufferGeometry;
  material?: THREE.LineBasicMaterial;
};

/**
 * Lightweight helper for rendering Rapier's built-in physics debug wireframes
 * (collider outlines, joints, etc.).
 *
 * Usage:
 *   const debugRenderer = new RapierDebugRenderer(scene, world);
 *   // In render loop:
 *   debugRenderer.update();
 *   // Toggle:
 *   debugRenderer.toggle();
 *   // Cleanup:
 *   debugRenderer.dispose();
 */
export class RapierDebugRenderer {
  private scene: THREE.Scene | null;
  private world: RapierWorld | null;
  private mesh: THREE.LineSegments<THREE.BufferGeometry, THREE.LineBasicMaterial>;
  private enabled: boolean;

  constructor(scene: THREE.Scene | null, world: RapierWorld | null, options: RapierDebugRendererOptions = {}) {
    this.scene = scene ?? null;
    this.world = world ?? null;
    this.enabled = options.enabled ?? false;

    const geometry = options.geometry ?? new THREE.BufferGeometry();
    const material = options.material ?? new THREE.LineBasicMaterial({ color: 0xffffff, vertexColors: true });

    this.mesh = new THREE.LineSegments(geometry, material);
    this.mesh.frustumCulled = false;
    this.mesh.visible = this.enabled;

    if (this.scene) {
      this.scene.add(this.mesh);
    }
  }

  setEnabled(enabled: boolean) {
    this.enabled = enabled;
    if (!enabled) {
      this.mesh.visible = false;
    }
    return this.enabled;
  }

  toggle() {
    return this.setEnabled(!this.enabled);
  }

  update() {
    if (!this.enabled || !this.world || typeof this.world.debugRender !== 'function') {
      return;
    }

    const buffers = this.world.debugRender(0);
    if (!buffers) return;

    const { vertices, colors } = buffers;
    if (!vertices || vertices.length === 0) {
      this.mesh.visible = false;
      this.#free(buffers);
      return;
    }

    const geometry = this.mesh.geometry;
    geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 4));
    geometry.setDrawRange(0, vertices.length / 3);

    this.mesh.visible = true;
    this.#free(buffers);
  }

  dispose({ disposeGeometry = true, disposeMaterial = true }: { disposeGeometry?: boolean; disposeMaterial?: boolean } = {}) {
    if (this.mesh.parent) {
      this.mesh.parent.remove(this.mesh);
    }
    if (disposeGeometry) {
      this.mesh.geometry.dispose();
    }
    if (disposeMaterial) {
      this.mesh.material.dispose();
    }
  }

  #free(buffers: { free?: () => void } | null) {
    if (buffers && typeof buffers.free === 'function') {
      try { buffers.free(); } catch {}
    }
  }
}

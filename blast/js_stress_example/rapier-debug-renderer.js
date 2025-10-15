/**
 * Rapier debug overlay helper.
 *
 * Example:
 * ```js
 * import RapierDebugRenderer from './rapier-debug-renderer.js';
 *
 * const rapierWorld = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
 * const scene = new THREE.Scene();
 * const debug = new RapierDebugRenderer(scene, rapierWorld);
 *
 * debug.setEnabled(true); // show Rapier's wireframe
 *
 * function animate() {
 *   rapierWorld.step();
 *   debug.update();
 *   renderer.render(scene, camera);
 *   requestAnimationFrame(animate);
 * }
 * ```
 */
import * as THREE from 'three';

export class RapierDebugRenderer {
  constructor(scene, world, options = {}) {
    this.scene = scene;
    this.world = world;
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

  setEnabled(enabled) {
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

    // Pass 0 as filterFlags to avoid WASM error with undefined
    const buffers = this.world.debugRender(0);
    if (!buffers) {
      return;
    }

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

  dispose({ disposeGeometry = true, disposeMaterial = true } = {}) {
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

  #free(buffers) {
    if (buffers && typeof buffers.free === 'function') {
      buffers.free();
    }
  }
}

export default RapierDebugRenderer;


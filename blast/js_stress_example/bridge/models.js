import * as THREE from 'three';

export class BridgeChunk {
  constructor(desc) {
    Object.assign(this, desc);
    this.colliderHandle = null;
    this.bodyHandle = null;
    this.mesh = null;
    this.active = true;
    this.severity = 0;
    this.pendingForce = { x: 0, y: 0, z: 0 };
    this.localOffset = new THREE.Vector3(this.localPosition.x, this.localPosition.y, this.localPosition.z);
    this.localQuat = desc.localRotation instanceof THREE.Quaternion
      ? desc.localRotation.clone()
      : new THREE.Quaternion(
          this.localRotation.x ?? 0,
          this.localRotation.y ?? 0,
          this.localRotation.z ?? 0,
          this.localRotation.w ?? 1
        );
  }
}

export class BridgeBond {
  constructor(desc) {
    Object.assign(this, desc);
    this.active = true;
    this.severity = createSeverity();
    this.key = `${this.node0}-${this.node1}`;
  }
}

export function createSeverity() {
  return {
    compression: 0,
    tension: 0,
    shear: 0,
    max: 0
  };
}


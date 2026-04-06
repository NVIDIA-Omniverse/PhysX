import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
export function createColliderForChunk(chunk) {
    const rotation = chunk.localQuat ?? new THREE.Quaternion();
    switch (chunk.shape.type) {
        case 'box':
            return RAPIER.ColliderDesc.cuboid(chunk.shape.hx, chunk.shape.hy, chunk.shape.hz)
                .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
                .setRotation(new RAPIER.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
        case 'capsule':
            return RAPIER.ColliderDesc.capsule(chunk.shape.halfHeight, chunk.shape.radius)
                .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
                .setRotation(new RAPIER.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
        default:
            return RAPIER.ColliderDesc.ball(1).setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z);
    }
}
export function createMeshForChunk(chunk, baseMaterial) {
    const material = baseMaterial.clone();
    let geometry;
    switch (chunk.shape.type) {
        case 'box':
            geometry = new THREE.BoxGeometry(chunk.shape.hx * 2, chunk.shape.hy * 2, chunk.shape.hz * 2);
            break;
        case 'capsule':
            geometry = new THREE.CapsuleGeometry(chunk.shape.radius, chunk.shape.halfHeight * 2, 18, 36);
            break;
        default:
            geometry = new THREE.SphereGeometry(0.6, 20, 20);
    }
    return new THREE.Mesh(geometry, material);
}

import * as THREE from 'three';
const scratchVec = new THREE.Vector3();
const scratchQuat = new THREE.Quaternion();
export function resolveParentBody(world, chunk) {
    if (!chunk) {
        return null;
    }
    const bodyHandle = chunk.bodyHandle ?? chunk.parentBodyHandle;
    if (bodyHandle == null) {
        return null;
    }
    return world.getRigidBody(bodyHandle);
}
export function computeWorldPose(parentBody, chunk) {
    if (!parentBody || !chunk) {
        return null;
    }
    const translation = parentBody.translation();
    const rotation = parentBody.rotation();
    const parentPos = scratchVec.set(translation.x, translation.y, translation.z);
    const parentQuat = scratchQuat.set(rotation.x, rotation.y, rotation.z, rotation.w);
    const localOffset = chunk.localOffset ?? chunk.restOffset;
    const localQuat = chunk.localQuat ?? chunk.restQuat;
    const worldPos = parentPos.clone().add(localOffset.clone().applyQuaternion(parentQuat));
    const worldQuat = parentQuat.clone().multiply(localQuat ?? scratchQuat.identity());
    return { position: worldPos, rotation: worldQuat };
}
export function computeLinearVelocityAtPoint(parentBody, offsetWorld) {
    const linear = parentBody.linvel();
    const angular = parentBody.angvel();
    return {
        x: linear.x + angular.y * offsetWorld.z - angular.z * offsetWorld.y,
        y: linear.y + angular.z * offsetWorld.x - angular.x * offsetWorld.z,
        z: linear.z + angular.x * offsetWorld.y - angular.y * offsetWorld.x
    };
}
export function identityQuaternion() {
    return new THREE.Quaternion(0, 0, 0, 1);
}

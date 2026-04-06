import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { PROJECTILE_SPEED } from './constants.js';
import { pushEvent } from './ui.js';
export function spawnProjectile(world, bridge) {
    const origin = new THREE.Vector3(-20, 4, 0);
    const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(origin.x, origin.y, origin.z)
        .setCanSleep(false);
    const projectileBody = world.createRigidBody(bodyDesc);
    const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.6), projectileBody);
    projectileBody.setLinvel({ x: PROJECTILE_SPEED, y: -1, z: (Math.random() - 0.5) * 5 }, true);
    const projectileMaterial = new THREE.MeshStandardMaterial({ color: 0xff9147, emissive: 0x552211 });
    const projectileMesh = new THREE.Mesh(new THREE.SphereGeometry(0.6, 20, 20), projectileMaterial);
    projectileMesh.castShadow = true;
    projectileMesh.receiveShadow = true;
    projectileMesh.position.copy(origin);
    bridge.scene.add(projectileMesh);
    bridge.projectiles.push({
        bodyHandle: projectileBody.handle,
        colliderHandle: collider.handle,
        mesh: projectileMesh,
        ttl: 12
    });
    pushEvent('Projectile fired at bridge');
}
export function updateProjectiles(world, bridge, delta) {
    const remaining = [];
    bridge.projectiles.forEach((projectile) => {
        const body = world.getRigidBody(projectile.bodyHandle);
        if (!body) {
            projectile.mesh?.removeFromParent();
            return;
        }
        projectile.ttl -= delta;
        if (projectile.ttl <= 0 || body.translation().y < -10) {
            world.removeRigidBody(body);
            projectile.mesh?.removeFromParent();
            return;
        }
        const translation = body.translation();
        projectile.mesh.position.set(translation.x, translation.y, translation.z);
        const rotation = body.rotation();
        projectile.mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
        remaining.push(projectile);
    });
    bridge.projectiles = remaining;
}

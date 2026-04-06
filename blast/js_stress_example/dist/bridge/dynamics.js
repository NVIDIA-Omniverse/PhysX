// Physics dynamics helpers shared by the web demo and tests (no Three.js).
export function spawnLoadVehicle(world, options = {}) {
    const mass = options.mass ?? 5200;
    const length = options.length ?? 3.6;
    const width = options.width ?? 1.8;
    const height = options.height ?? 1.2;
    const speed = options.speed ?? 4.5;
    const idleDuration = options.idleDuration ?? 0.75;
    const minX = options.minX ?? -6.0;
    const maxX = options.maxX ?? +6.0;
    const margin = length * 0.2;
    const bodyDesc = options.bodyDescFactory
        ? options.bodyDescFactory()
        : world.RAPIER?.RigidBodyDesc?.dynamic?.() ?? null;
    const body = world.createRigidBody((bodyDesc ?? world.RAPIER.RigidBodyDesc.dynamic())
        .setTranslation(minX - margin, 2.0 + height * 0.5 + 0.1, 0)
        .setLinearDamping(0.4)
        .setAngularDamping(1.0));
    const collider = world.createCollider(world.RAPIER.ColliderDesc.cuboid(length * 0.5, height * 0.5, width * 0.5)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setMass(mass)
        .setActiveEvents(world.RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0), body);
    body.setLinvel({ x: speed, y: 0, z: 0 }, true);
    const car = {
        bodyHandle: body.handle,
        colliderHandle: collider.handle,
        mass,
        speed,
        baseSpeed: speed,
        direction: 1,
        waitTimer: idleDuration,
        active: true,
        minX,
        maxX,
        margin,
        height,
        mesh: null
    };
    // Optional visual mesh (for browser demo)
    if (options.scene && options.THREE) {
        const THREE = options.THREE;
        const mesh = new THREE.Mesh(new THREE.BoxGeometry(length, height, width), new THREE.MeshStandardMaterial({ color: 0xffd166, roughness: 0.4, metalness: 0.6 }));
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        options.scene.add(mesh);
        const t0 = body.translation();
        const q0 = body.rotation();
        mesh.position.set(t0.x, t0.y, t0.z);
        mesh.quaternion.set(q0.x, q0.y, q0.z, q0.w);
        car.mesh = mesh;
    }
    return car;
}
export function updateLoadVehicle(world, car, delta) {
    if (!car?.active)
        return;
    const body = world.getRigidBody(car.bodyHandle);
    if (!body)
        return;
    if (car.waitTimer > 0) {
        car.waitTimer = Math.max(0, car.waitTimer - delta);
        return;
    }
    const tr = body.translation();
    body.setLinvel({ x: car.speed * car.direction, y: body.linvel().y, z: body.linvel().z }, true);
    if (tr.x > car.maxX + car.margin) {
        body.setTranslation({ x: car.maxX + car.margin, y: tr.y, z: tr.z }, true);
        car.direction = -1;
        car.waitTimer = 0.75;
    }
    else if (tr.x < car.minX - car.margin) {
        body.setTranslation({ x: car.minX - car.margin, y: tr.y, z: tr.z }, true);
        car.direction = 1;
        car.waitTimer = 0.75;
    }
}
export function spawnProjectile(world, options = {}) {
    const scale = options.scale ?? 1.0;
    const origin = options.origin ?? { x: (Math.random() - 0.5) * 6, y: 5, z: (Math.random() - 0.5) * 6 };
    const kind = options.kind ?? 'box';
    const body = world.createRigidBody(world.RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(origin.x, origin.y, origin.z)
        .setCanSleep(false));
    const collider = kind === 'ball'
        ? world.createCollider(world.RAPIER.ColliderDesc.ball(0.6 * scale), body)
        : world.createCollider(world.RAPIER.ColliderDesc.cuboid(0.6 * scale, 0.6 * scale, 0.6 * scale)
            .setMass(100000000.0 * scale * scale)
            .setFriction(0.0)
            .setRestitution(0.0)
            .setActiveEvents(world.RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
            .setContactForceEventThreshold(0.0), body);
    body.setLinvel({ x: (Math.random() - 0.5) * 5, y: -5, z: (Math.random() - 0.5) * 5 }, true);
    return {
        bodyHandle: body.handle,
        colliderHandle: collider.handle,
        ttl: 12
    };
}
export function updateProjectiles(world, projectiles, delta, bridge) {
    const remaining = [];
    projectiles.forEach((proj) => {
        const body = world.getRigidBody(proj.bodyHandle);
        if (!body)
            return;
        proj.ttl -= delta;
        const t = body.translation();
        if (proj.ttl <= 0 || t.y < -20) {
            // Defer removal to safe-step if bridge provided; else remove immediately
            if (bridge) {
                if (!bridge.bodiesToRemove)
                    bridge.bodiesToRemove = new Set();
                bridge.bodiesToRemove.add(proj.bodyHandle);
            }
            else {
                world.removeRigidBody(body);
            }
            return;
        }
        remaining.push(proj);
    });
    return remaining;
}

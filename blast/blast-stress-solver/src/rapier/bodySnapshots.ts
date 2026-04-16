import type RAPIER from '@dimforge/rapier3d-compat';

export type BodySnapshot = {
  handle: number;
  translation: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number; w: number };
  linvel: { x: number; y: number; z: number };
  angvel: { x: number; y: number; z: number };
};

export type BodySnapshotCaptureResult = {
  snapshots: BodySnapshot[];
  size: number;
  bytes: number;
};

export function captureDynamicBodySnapshots(
  world: RAPIER.World,
  snapshotPool: BodySnapshot[],
  snapshotIndex: Map<number, BodySnapshot>,
): BodySnapshotCaptureResult {
  snapshotIndex.clear();
  let size = 0;

  world.forEachRigidBody((body) => {
    if (body.isFixed()) return;

    const t = body.translation();
    const r = body.rotation();
    const lv = body.linvel();
    const av = body.angvel();

    let snap: BodySnapshot;
    if (size < snapshotPool.length) {
      snap = snapshotPool[size];
      snap.handle = body.handle;
      snap.translation.x = t.x;
      snap.translation.y = t.y;
      snap.translation.z = t.z;
      snap.rotation.x = r.x;
      snap.rotation.y = r.y;
      snap.rotation.z = r.z;
      snap.rotation.w = r.w;
      snap.linvel.x = lv.x;
      snap.linvel.y = lv.y;
      snap.linvel.z = lv.z;
      snap.angvel.x = av.x;
      snap.angvel.y = av.y;
      snap.angvel.z = av.z;
    } else {
      snap = {
        handle: body.handle,
        translation: { x: t.x, y: t.y, z: t.z },
        rotation: { x: r.x, y: r.y, z: r.z, w: r.w },
        linvel: { x: lv.x, y: lv.y, z: lv.z },
        angvel: { x: av.x, y: av.y, z: av.z },
      };
      snapshotPool.push(snap);
    }

    snapshotIndex.set(body.handle, snap);
    size += 1;
  });

  return {
    snapshots: snapshotPool,
    size,
    bytes: size * 13 * 8,
  };
}

export function restoreDynamicBodySnapshots(
  world: RAPIER.World,
  snapshots: BodySnapshot[] | null,
  snapshotCount: number,
) {
  if (!snapshots) return;

  for (let index = 0; index < snapshotCount; index += 1) {
    const snap = snapshots[index];
    const body = world.getRigidBody(snap.handle);
    if (!body) continue;
    body.setTranslation(snap.translation, true);
    body.setRotation(snap.rotation, true);
    body.setLinvel(snap.linvel, true);
    body.setAngvel(snap.angvel, true);
  }
}

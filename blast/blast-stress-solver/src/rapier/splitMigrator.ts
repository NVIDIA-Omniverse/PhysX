import type { SplitChild } from "./types";

export type ExistingBodyState = {
  handle: number;
  nodeIndices: ReadonlySet<number>;
  isFixed: boolean;
};

export type PlannerChild = SplitChild & { index: number };

export type SplitMigrationPlan = {
  reuse: Array<{ childIndex: number; bodyHandle: number }>;
  create: Array<{ childIndex: number }>;
};

export function planSplitMigration(
  bodies: ExistingBodyState[],
  children: PlannerChild[],
  timings?: { onDuration?: (ms: number) => void },
): SplitMigrationPlan {
  const start = typeof performance !== "undefined" ? performance.now() : Date.now();

  if (bodies.length === 0 || children.length === 0) {
    const plan = {
      reuse: [],
      create: children.map((child) => ({ childIndex: child.index })),
    };
    timings?.onDuration?.(
      (typeof performance !== "undefined" ? performance.now() : Date.now()) - start,
    );
    return plan;
  }

  const bodyHashes = buildBodyHashIndex(bodies);
  const reuse: Array<{ childIndex: number; bodyHandle: number }> = [];
  const reusedChildren = new Set<number>();
  const assignedBodies = new Set<number>();
  const unmatchedChildren: PlannerChild[] = [];

  for (const child of children) {
    const hash = hashNodes(child.nodes);
    const candidates = bodyHashes.get(hash);
    if (!candidates) {
      unmatchedChildren.push(child);
      continue;
    }
    const body = candidates.find(
      (candidate) =>
        !assignedBodies.has(candidate.handle) &&
        (candidate.isFixed ? child.isSupport : true),
    );
    if (!body) {
      unmatchedChildren.push(child);
      continue;
    }
    reuse.push({ childIndex: child.index, bodyHandle: body.handle });
    reusedChildren.add(child.index);
    assignedBodies.add(body.handle);
  }

  const unmatchedBodies = bodies.filter((body) => !assignedBodies.has(body.handle));

  let remainingPlan: SplitMigrationPlan = { reuse: [], create: [] };
  if (unmatchedBodies.length > 0 && unmatchedChildren.length > 0) {
    remainingPlan = planViaHungarian(unmatchedBodies, unmatchedChildren);
  } else {
    remainingPlan.create = unmatchedChildren.map((child) => ({
      childIndex: child.index,
    }));
  }

  const plan = {
    reuse: [...reuse, ...remainingPlan.reuse],
    create: remainingPlan.create,
  };
  timings?.onDuration?.(
    (typeof performance !== "undefined" ? performance.now() : Date.now()) - start,
  );
  return plan;
}

function planViaHungarian(
  bodies: ExistingBodyState[],
  children: PlannerChild[],
): SplitMigrationPlan {
  const overlapMatrix = buildOverlapMatrix(bodies, children);
  const assignments = hungarianMax(overlapMatrix);
  const reuse: Array<{ childIndex: number; bodyHandle: number }> = [];
  const reusedChildren = new Set<number>();

  bodies.forEach((body, rowIdx) => {
    const assignedCol = assignments[rowIdx];
    if (
      assignedCol == null ||
      assignedCol < 0 ||
      assignedCol >= children.length
    ) {
      return;
    }
    const overlap = overlapMatrix[rowIdx][assignedCol] ?? 0;
    if (overlap > 0) {
      reuse.push({
        childIndex: children[assignedCol].index,
        bodyHandle: body.handle,
      });
      reusedChildren.add(children[assignedCol].index);
    }
  });

  const create = children
    .filter((child) => !reusedChildren.has(child.index))
    .map((child) => ({ childIndex: child.index }));
  return { reuse, create };
}

function buildBodyHashIndex(
  bodies: ExistingBodyState[],
): Map<string, ExistingBodyState[]> {
  const map = new Map<string, ExistingBodyState[]>();
  for (const body of bodies) {
    const hash = hashNodes(body.nodeIndices);
    const bucket = map.get(hash);
    if (bucket) {
      bucket.push(body);
    } else {
      map.set(hash, [body]);
    }
  }
  return map;
}

function hashNodes(nodes: Iterable<number>): string {
  const list = Array.isArray(nodes)
    ? nodes.slice()
    : Array.from(nodes);
  list.sort((a, b) => a - b);
  return list.join(",");
}

function buildOverlapMatrix(
  bodies: ExistingBodyState[],
  children: PlannerChild[],
): number[][] {
  return bodies.map((body) => {
    return children.map((child) => {
      if (body.isFixed && !child.isSupport) {
        return 0;
      }
      let overlap = 0;
      for (const nodeIndex of child.nodes) {
        if (body.nodeIndices.has(nodeIndex)) overlap += 1;
      }
      return overlap;
    });
  });
}

function hungarianMax(matrix: number[][]): number[] {
  const rows = matrix.length;
  const cols = matrix[0]?.length ?? 0;
  if (!rows || !cols) return new Array(rows).fill(-1);
  const size = Math.max(rows, cols);
  let maxVal = 0;
  for (const row of matrix) {
    for (const value of row) maxVal = Math.max(maxVal, value);
  }
  const cost = Array.from({ length: size }, (_, i) =>
    Array.from({ length: size }, (_, j) => {
      const value = matrix[i]?.[j] ?? 0;
      return maxVal - value;
    }),
  );
  const assignment = hungarian(cost);
  return assignment.slice(0, rows);
}

function hungarian(cost: number[][]): number[] {
  const n = cost.length;
  const m = cost[0]?.length ?? 0;
  const size = Math.max(n, m);
  const paddedCost = Array.from({ length: size }, (_, i) =>
    Array.from({ length: size }, (_, j) => cost[i]?.[j] ?? 0),
  );
  const u = new Array(size + 1).fill(0);
  const v = new Array(size + 1).fill(0);
  const p = new Array(size + 1).fill(0);
  const way = new Array(size + 1).fill(0);

  for (let i = 1; i <= size; i++) {
    p[0] = i;
    const minv = new Array(size + 1).fill(Infinity);
    const used = new Array(size + 1).fill(false);
    let j0 = 0;
    do {
      used[j0] = true;
      const i0 = p[j0];
      let delta = Infinity;
      let j1 = 0;
      for (let j = 1; j <= size; j++) {
        if (used[j]) continue;
        const cur = paddedCost[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (let j = 0; j <= size; j++) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do {
      const j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  const result = new Array(size).fill(-1);
  for (let j = 1; j <= size; j++) {
    if (p[j] > 0) {
      result[p[j] - 1] = j - 1;
    }
  }
  return result;
}

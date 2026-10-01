import type { Intrinsics } from "./types.js";

export interface ParamDelta {
  parameter: keyof Intrinsics;
  estimated: number;
  truth: number;
  absolute: number;
  /** 相对误差（百分比），真值为 0 时为 null */
  relativePercent: number | null;
}

/** 比较估计内参与真值。 */
export function compareIntrinsics(
  estimated: Intrinsics,
  truth: Intrinsics,
): ParamDelta[] {
  const keys: (keyof Intrinsics)[] = ["fx", "fy", "cx", "cy", "k1", "k2"];
  return keys.map((key) => {
    const e = estimated[key];
    const t = truth[key];
    const abs = e - t;
    return {
      parameter: key,
      estimated: e,
      truth: t,
      absolute: abs,
      relativePercent: t !== 0 ? (100 * abs) / Math.abs(t) : null,
    };
  });
}

/** 取观测子集（按索引或固定步长），用于“观测子集比较”。 */
export function subsetByIds<T extends { id: string }>(
  items: T[],
  ids: string[],
): T[] {
  const set = new Set(ids);
  return items.filter((it) => set.has(it.id));
}

/** 固定步长抽稀子集。 */
export function subsetEvery<T>(items: T[], every: number, offset = 0): T[] {
  return items.filter((_, i) => (i - offset) % every === 0);
}

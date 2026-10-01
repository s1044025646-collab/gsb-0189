import { calibrate } from "./calibration";
import { Intrinsics, PointObservation } from "./types";

export interface ParameterDelta {
  param: keyof Intrinsics;
  a: number;
  b: number;
  absDelta: number;
  relDelta: number;
}

/** 逐参数比较两组内参。fx/fy/cx/cy 用绝对差，k 参数附相对差。 */
export function compareIntrinsics(a: Intrinsics, b: Intrinsics): ParameterDelta[] {
  const keys: (keyof Intrinsics)[] = ["fx", "fy", "cx", "cy", "k1", "k2"];
  return keys.map((param) => {
    const av = a[param];
    const bv = b[param];
    const absDelta = av - bv;
    return {
      param,
      a: av,
      b: bv,
      absDelta,
      relDelta: bv !== 0 ? absDelta / Math.abs(bv) : absDelta,
    };
  });
}

/**
 * 异常残差定位：对全量与“逐点剔除”(leave-one-out) 子集分别标定，
 * 若移除某点后 RMSE 显著下降（超过阈值比例），该点标记为可疑异常值。
 */
export function locateOutliers(
  obs: PointObservation[],
  opts: { influenceRatio?: number; config?: Parameters<typeof calibrate>[1] } = {}
): {
  baselineRmse: number;
  candidates: { id: string; rmseWithout: number; dropRatio: number }[];
} {
  const influenceRatio = opts.influenceRatio ?? 0.2;
  const baseline = calibrate(obs, opts.config);
  const baselineRmse = baseline.rmse ?? NaN;
  const candidates: { id: string; rmseWithout: number; dropRatio: number }[] = [];
  obs.forEach((target, i) => {
    const subset = obs.filter((_, j) => j !== i);
    const r = calibrate(subset, opts.config);
    if (r.rmse == null || baselineRmse === 0) return;
    const dropRatio = (baselineRmse - r.rmse) / baselineRmse;
    if (dropRatio > influenceRatio)
      candidates.push({ id: target.id ?? `p${i}`, rmseWithout: r.rmse, dropRatio });
  });
  candidates.sort((p, q) => q.dropRatio - p.dropRatio);
  return { baselineRmse, candidates };
}

/** 同一真值在两个观测子集上分别标定，返回参数与 RMSE 差异。 */
export function compareSubsets(
  subsetA: PointObservation[],
  subsetB: PointObservation[],
  config?: Parameters<typeof calibrate>[1]
) {
  const ra = calibrate(subsetA, config);
  const rb = calibrate(subsetB, config);
  return {
    resultA: ra,
    resultB: rb,
    parameterDeltas:
      ra.intrinsics && rb.intrinsics
        ? compareIntrinsics(ra.intrinsics, rb.intrinsics)
        : null,
    rmseDelta: (ra.rmse ?? NaN) - (rb.rmse ?? NaN),
  };
}

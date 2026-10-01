import {
  projectCameraPoint,
  reprojectionError,
  undistortNormalized,
} from "./geometry.js";
import { leastSquares, solveLinear } from "./linalg.js";
import type {
  CalibDiagnostics,
  CalibResult,
  Intrinsics,
  Observation,
  PerPointError,
} from "./types.js";

const MIN_POINTS = 6;

export interface CalibOptions {
  maxIterations?: number;
  /** 参数收敛阈值（相对） */
  tol?: number;
  /** 初始 LM 阻尼 */
  lambdaInit?: number;
}

interface Prepared {
  id: string;
  xn: number;
  yn: number;
  u: number;
  v: number;
  z: number;
}

function evaluate(
  data: Prepared[],
  p: Intrinsics,
): { rms: number; perPoint: PerPointError[]; undistortFail: number } {
  let sumSq = 0;
  const perPoint: PerPointError[] = [];
  let undistortFail = 0;
  for (const d of data) {
    const r2 = d.xn * d.xn + d.yn * d.yn;
    const D = 1 + p.k1 * r2 + p.k2 * r2 * r2;
    const uProj = p.fx * d.xn * D + p.cx;
    const vProj = p.fy * d.yn * D + p.cy;
    const du = uProj - d.u;
    const dv = vProj - d.v;
    sumSq += du * du + dv * dv;
    perPoint.push({
      id: d.id,
      u: d.u,
      v: d.v,
      uResidual: du,
      vResidual: dv,
      distance: Math.hypot(du, dv),
    });
    const xd = (d.u - p.cx) / p.fx;
    const yd = (d.v - p.cy) / p.fy;
    if (!undistortNormalized(xd, yd, p).converged) undistortFail++;
  }
  return {
    rms: Math.sqrt(sumSq / data.length),
    perPoint,
    undistortFail,
  };
}

/** 联合高斯-牛顿（Levenberg-Marquardt 阻尼）精化 [fx,fy,cx,cy,k1,k2]。 */
function refine(
  data: Prepared[],
  start: Intrinsics,
  opts: Required<CalibOptions>,
): { params: Intrinsics; iterations: number; converged: boolean; reason: string; rms: number } {
  let params: Intrinsics = { ...start };
  let lambda = opts.lambdaInit;
  let cur = evaluate(data, params).rms;
  let converged = false;
  let reason = "达到最大迭代次数仍未收敛";
  let iter = 0;

  for (iter = 0; iter < opts.maxIterations; iter++) {
    const n = 6;
    const JtJ = new Array(n * n).fill(0);
    const Jtr = new Array(n).fill(0);
    for (const d of data) {
      const r2 = d.xn * d.xn + d.yn * d.yn;
      const D = 1 + params.k1 * r2 + params.k2 * r2 * r2;
      const uProj = params.fx * d.xn * D + params.cx;
      const vProj = params.fy * d.yn * D + params.cy;
      const ru = uProj - d.u;
      const rv = vProj - d.v;
      // 雅可比行（u 分量与 v 分量）
      const ju = [
        d.xn * D, // fx
        0, // fy
        1, // cx
        0, // cy
        params.fx * d.xn * r2, // k1
        params.fx * d.xn * r2 * r2, // k2
      ];
      const jv = [
        0,
        d.yn * D,
        0,
        1,
        params.fy * d.yn * r2,
        params.fy * d.yn * r2 * r2,
      ];
      for (let a = 0; a < n; a++) {
        Jtr[a]! += ju[a]! * ru + jv[a]! * rv;
        for (let b = 0; b < n; b++) {
          JtJ[a * n + b]! += ju[a]! * ju[b]! + jv[a]! * jv[b]!;
        }
      }
    }
    // 尝试含阻尼的步长，失败则增大 lambda 重试
    let accepted = false;
    let candidate = params;
    let candRms = cur;
    for (let attempt = 0; attempt < 20; attempt++) {
      const H = JtJ.slice();
      for (let a = 0; a < n; a++) {
        const diag = Math.max(H[a * n + a]!, 1e-12);
        H[a * n + a] = diag * (1 + lambda);
      }
      const neg = Jtr.map((v) => -v);
      const delta = solveLinear(H, neg, n);
      if (!delta) {
        lambda *= 10;
        continue;
      }
      candidate = {
        fx: params.fx + delta[0]!,
        fy: params.fy + delta[1]!,
        cx: params.cx + delta[2]!,
        cy: params.cy + delta[3]!,
        k1: params.k1 + delta[4]!,
        k2: params.k2 + delta[5]!,
      };
      if (
        !(candidate.fx > 0) ||
        !(candidate.fy > 0) ||
        candidate.k1 < -5 ||
        candidate.k1 > 5 ||
        candidate.k2 < -20 ||
        candidate.k2 > 20
      ) {
        lambda *= 10;
        continue;
      }
      candRms = evaluate(data, candidate).rms;
      if (Number.isFinite(candRms) && candRms < cur) {
        accepted = true;
        break;
      }
      lambda *= 10;
    }
    if (!accepted) {
      converged = true;
      reason = "无法找到进一步下降的步长，按当前最优解停止";
      break;
    }
    const relRms = Math.abs(cur - candRms) / Math.max(1e-12, cur);
    params = candidate;
    cur = candRms;
    lambda = Math.max(lambda * 0.3, 1e-12);
    if (relRms < opts.tol) {
      converged = true;
      reason = `RMS 相对变化 ${relRms.toExponential(2)} < 容差 ${opts.tol}`;
      break;
    }
  }
  return { params, iterations: iter + 1, converged, reason, rms: cur };
}

/**
 * 标定主流程。
 * 方法：
 *  1) 剔除镜头后方点 (Z<=0)。
 *  2) 忽略畸变，用归一化坐标 xn=X/Z, yn=Y/Z 做线性最小二乘，
 *     分别求 (fx,cx) 与 (fy,cy) 的初值。
 *  3) 以线性解为初值，对 fx,fy,cx,cy,k1,k2 做联合高斯-牛顿（LM 阻尼）非线性最小二乘。
 * 收敛判断：相邻迭代 RMS 相对变化小于 tol，或无可接受下降步长，或达到最大迭代次数。
 */
export function calibrate(
  observations: Observation[],
  options: CalibOptions = {},
): CalibResult {
  const opts: Required<CalibOptions> = {
    maxIterations: options.maxIterations ?? 100,
    tol: options.tol ?? 1e-10,
    lambdaInit: options.lambdaInit ?? 1e-3,
  };
  const warnings: string[] = [];

  const behind = observations.filter((o) => !(o.point.z > 0));
  const valid = observations.filter((o) => o.point.z > 0);
  const prepared: Prepared[] = valid.map((o) => ({
    id: o.id,
    xn: o.point.x / o.point.z,
    yn: o.point.y / o.point.z,
    u: o.pixel.u,
    v: o.pixel.v,
    z: o.point.z,
  }));

  // 设计矩阵秩诊断：列 [xn, yn, 1]
  const design = prepared.map((d) => [d.xn, d.yn, 1]);
  const lsGeom = leastSquares(design, prepared.map((d) => d.u), 1e-9);
  const lsX = leastSquares(
    prepared.map((d) => [d.xn, 1]),
    prepared.map((d) => d.u),
    1e-9,
  );
  const lsY = leastSquares(
    prepared.map((d) => [d.yn, 1]),
    prepared.map((d) => d.v),
    1e-9,
  );

  const zs = prepared.map((d) => d.z);
  const diagnostics: CalibDiagnostics = {
    pointCount: valid.length,
    rank: lsGeom.rank,
    expectedRank: 3,
    minSingular: Math.min(...lsGeom.singularValues),
    conditionNumber: lsGeom.conditionNumber,
    zMin: zs.length ? Math.min(...zs) : NaN,
    zMax: zs.length ? Math.max(...zs) : NaN,
    behindCameraCount: behind.length,
    identifiable: false,
    warnings,
  };

  const degenerate = (msg: string): CalibResult => {
    warnings.push(msg);
    const dummy: Intrinsics = {
      fx: NaN,
      fy: NaN,
      cx: NaN,
      cy: NaN,
      k1: NaN,
      k2: NaN,
    };
    return {
      intrinsics: dummy,
      linearIntrinsics: dummy,
      diagnostics,
      rms: NaN,
      perPoint: [],
      iterations: 0,
      converged: false,
      convergenceReason: msg,
      undistortFailedCount: 0,
    };
  };

  if (valid.length < MIN_POINTS) {
    return degenerate(
      `有效点数 ${valid.length} 少于最少要求 ${MIN_POINTS}，参数不可辨识`,
    );
  }
  if (behind.length > 0) {
    warnings.push(`已剔除 ${behind.length} 个镜头后方点 (Z<=0)`);
  }
  if (lsGeom.rank < 3) {
    warnings.push(
      `几何设计矩阵秩=${lsGeom.rank} < 3（条件数≈${fmt(lsGeom.conditionNumber)}），点布局近似共线/共轴，fx/fy/主点无法同时辨识`,
    );
  }
  if (lsX.rank < 2) warnings.push("归一化 x=X/Z 几乎不变，fx 与 cx 不可分离");
  if (lsY.rank < 2) warnings.push("归一化 y=Y/Z 几乎不变，fy 与 cy 不可分离");
  if (lsGeom.conditionNumber > 1e8) {
    warnings.push(
      `设计矩阵条件数≈${fmt(lsGeom.conditionNumber)} 过大，结果对噪声极敏感`,
    );
  }
  const zSpan = diagnostics.zMax - diagnostics.zMin;
  const xyScale = Math.max(
    ...prepared.map((d) => Math.hypot(d.xn, d.yn)),
    1e-12,
  );
  if (xyScale < 1e-6) warnings.push("所有点几乎位于光轴上（视场角趋近零），内参不可辨识");
  if (zSpan < 1e-9) {
    warnings.push(
      "所有点深度相同：不影响相机坐标点标定，但无法约束任何与深度相关的量",
    );
  }

  const xParams = lsX.x;
  const yParams = lsY.x;
  if (!xParams || !yParams || lsX.rank < 2 || lsY.rank < 2 || lsGeom.rank < 3) {
    return degenerate("几何退化，无法给出可靠标定结果（参数标记为 NaN）");
  }

  const linear: Intrinsics = {
    fx: xParams[0]!,
    fy: yParams[0]!,
    cx: xParams[1]!,
    cy: yParams[1]!,
    k1: 0,
    k2: 0,
  };
  if (!(linear.fx > 0) || !(linear.fy > 0)) {
    return degenerate(
      "线性解出现非正焦距，观测与点坐标不一致（可能光心/方向配置错误）",
    );
  }

  diagnostics.identifiable = true;
  const refined = refine(prepared, linear, opts);
  const ev = evaluate(prepared, refined.params);
  return {
    intrinsics: refined.params,
    linearIntrinsics: linear,
    diagnostics,
    rms: ev.rms,
    perPoint: ev.perPoint,
    iterations: refined.iterations,
    converged: refined.converged,
    convergenceReason: refined.reason,
    undistortFailedCount: ev.undistortFail,
  };
}

function fmt(x: number): string {
  if (!Number.isFinite(x)) return "∞";
  if (Math.abs(x) >= 1e4 || (x !== 0 && Math.abs(x) < 1e-3))
    return x.toExponential(2);
  return x.toFixed(3);
}

/** 残差异常定位：返回距离超过 threshold 倍稳健 MAD 的点。 */
export function locateOutliers(
  perPoint: PerPointError[],
  thresholdMad = 3.5,
): { id: string; distance: number; zScore: number }[] {
  const dists = perPoint.map((p) => p.distance).sort((a, b) => a - b);
  if (dists.length < 4) return [];
  const median = dists[Math.floor(dists.length / 2)]!;
  const dev = perPoint.map((p) => Math.abs(p.distance - median)).sort((a, b) => a - b);
  const mad = dev[Math.floor(dev.length / 2)]! || 1e-12;
  return perPoint
    .map((p) => ({ id: p.id, distance: p.distance, zScore: 0.6745 * (p.distance - median) / mad }))
    .filter((o) => Math.abs(o.zScore) > thresholdMad);
}

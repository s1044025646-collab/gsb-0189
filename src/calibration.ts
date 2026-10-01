import { leastSquares, rankDiagnostics } from "./linalg";
import type { Matrix } from "./linalg";
import { project } from "./geometry";
import { gaussNewtonRefine } from "./refine";
import {
  CalibrationConfig,
  CalibrationResult,
  DEFAULT_CONFIG,
  Intrinsics,
  PointObservation,
  ReprojectionError,
} from "./types";

function rmseOf(errors: ReprojectionError[]): { rmse: number; max: number } {
  let s = 0;
  let max = 0;
  for (const e of errors) {
    s += e.residualU ** 2 + e.residualV ** 2;
    max = Math.max(max, e.distance);
  }
  return { rmse: Math.sqrt(s / errors.length), max };
}

export function evaluateErrors(
  obs: PointObservation[],
  intr: Intrinsics
): ReprojectionError[] {
  return obs.map((o, i) => {
    const pr = project(o.point, intr);
    const du = pr.u - o.pixel.u;
    const dv = pr.v - o.pixel.v;
    return {
      id: o.id ?? `p${i}`,
      u: o.pixel.u,
      v: o.pixel.v,
      uReproj: pr.u,
      vReproj: pr.v,
      residualU: du,
      residualV: dv,
      distance: Math.hypot(du, dv),
      depth: o.point.z,
      behindCamera: pr.behindCamera,
    };
  });
}

/**
 * 标定针孔内参 fx,fy,cx,cy 与两个径向畸变系数 k1,k2。
 *
 * 方法（外参已知，点给在相机坐标系下）:
 * 1. 令归一化无畸变坐标 x=X/z, y=Y/z（镜头后方 z<=0 的点剔除并报告）。
 * 2. 初始化 k1=k2=0，解线性最小二乘（正规方程，自行实现求逆）:
 *      u = fx*xd + cx, v = fy*yd + cy,  xd=x*(1+k1 r^2+k2 r^4)（k 固定时为已知量）。
 * 3. 固定 fx..cy，再对 k1,k2 解线性最小二乘:
 *      (u-cx)/fx = x + k1*x*r^2 + k2*x*r^4
 *      (v-cy)/fy = y + k1*y*r^2 + k2*y*r^4
 *    第 2、3 步交替，直至参数相对变化与 RMSE 变化均小于 tol，或达到 maxIterations。
 * 4. 收敛判断: max|Δparam|/(1+|param|) < tol 且 |Δrmse| < tol（像素）。
 *    发散判断: RMSE 较上轮显著上升、参数非有限、|k1|/|k2| 爆增（>1e3）。
 */
export function calibrate(
  observations: PointObservation[],
  partial?: Partial<CalibrationConfig>
): CalibrationResult {
  const config: CalibrationConfig = { ...DEFAULT_CONFIG, ...partial };
  const warnings: string[] = [];
  const behind: string[] = [];

  const valid = observations.filter((o, i) => {
    const fin =
      Number.isFinite(o.point.x) &&
      Number.isFinite(o.point.y) &&
      Number.isFinite(o.point.z) &&
      Number.isFinite(o.pixel.u) &&
      Number.isFinite(o.pixel.v);
    if (!fin) {
      warnings.push(`点 ${o.id ?? `p${i}`} 含非有限值，已剔除。`);
      return false;
    }
    if (o.point.z <= 0) {
      behind.push(o.id ?? `p${i}`);
      return false;
    }
    return true;
  });

  const baseDiag: CalibrationResult["diagnostics"] = {
    totalPoints: observations.length,
    usedPoints: valid.length,
    behindCameraPoints: behind,
    matrixRank: 0,
    matrixCols: 4,
    singularValues: [],
    conditionNumber: null as number | null,
    warnings,
    normalizedXRange: null as [number, number] | null,
    normalizedYRange: null as [number, number] | null,
    depthRange: null as [number, number] | null,
  };
  const fail = (
    status: CalibrationResult["status"],
    reason: string
  ): CalibrationResult => ({
    status,
    intrinsics: null,
    distortionEstimated: false,
    iterations: 0,
    converged: false,
    convergenceReason: reason,
    rmseHistory: [],
    rmse: null,
    maxError: null,
    errors: [],
    diagnostics: baseDiag,
    config,
  });

  if (valid.length === 0)
    return fail(
      "all_points_behind_camera",
      "所有点都在镜头后方(z<=0)或为非有限值，无法标定。"
    );
  if (valid.length < config.minPoints)
    return fail(
      "insufficient_points",
      `有效点 ${valid.length} 个，少于要求的最小点数 ${config.minPoints}。`
    );
  if (behind.length)
    warnings.push(`已剔除 ${behind.length} 个镜头后方点: ${behind.join(", ")}。`);

  const data = valid.map((o) => {
    const x = o.point.x / o.point.z;
    const y = o.point.y / o.point.z;
    return { o, x, y, r2: x * x + y * y };
  });

  const xs = data.map((d) => d.x);
  const ys = data.map((d) => d.y);
  const zs = valid.map((o) => o.point.z);
  baseDiag.normalizedXRange = [Math.min(...xs), Math.max(...xs)];
  baseDiag.normalizedYRange = [Math.min(...ys), Math.max(...ys)];
  baseDiag.depthRange = [Math.min(...zs), Math.max(...zs)];
  const distinctCount = (vals: number[]) => {
    const uniq = new Set(vals.map((v) => Math.round(v * 1e9)));
    return uniq.size;
  };
  if (distinctCount(xs) < 2)
    warnings.push("归一化 x=X/z 几乎只有单一取值：fx 与 cx 不可辨识（同轴/共面退化）。");
  if (distinctCount(ys) < 2)
    warnings.push("归一化 y=Y/z 几乎只有单一取值：fy 与 cy 不可辨识（同轴/共面退化）。");

  // 初始线性针孔设计矩阵（k=0），用于秩诊断。
  const initA: number[][] = new Array<number[]>();
  const initB: number[] = [];
  for (const d of data) {
    initA.push([d.x, 1, 0, 0]);
    initB.push(d.o.pixel.u);
    initA.push([0, 0, d.y, 1]);
    initB.push(d.o.pixel.v);
  }
  const rd = rankDiagnostics(initA, config.rankTol);
  baseDiag.matrixRank = rd.rank;
  baseDiag.singularValues = rd.singularValues;
  baseDiag.conditionNumber = rd.conditionNumber;

  if (rd.rank < 4) {
    return fail(
      "degenerate",
      `线性设计矩阵秩=${rd.rank} < 4，fx/fy/cx/cy 不可同时辨识（条件数=${
        rd.conditionNumber ?? "无穷大"
      }）。典型原因: 所有点位于光轴上、归一化坐标缺少变化或点数不足。`
    );
  }

  // 第 1 阶段：线性针孔初值（k=0，最小二乘，正规方程自行实现）。
  const lin0 = leastSquares(initA, initB);
  if (!lin0) return fail("degenerate", "针孔线性初值求解失败。");
  const [fx0, cx0, fy0, cy0] = lin0.x;
  if (fx0 <= 0 || fy0 <= 0)
    return fail(
      "diverged",
      `线性初值出现非正焦距 fx=${fx0}, fy=${fy0}，数据几何或观测不一致。`
    );

  // 第 2 阶段：若估计畸变，做两轮「内参/k 线性交替」热启动以接近真值，
  // 避免高斯-牛顿从 k=0 一步跨太大；热启动仅用于提供初值，不做收敛判定。
  let warm: Intrinsics = { fx: fx0, fy: fy0, cx: cx0, cy: cy0, k1: 0, k2: 0 };
  if (config.estimateK) {
    let ck1 = 0;
    let ck2 = 0;
    for (let pass = 0; pass < 3; pass++) {
      const Ak: Matrix = [];
      const bk: number[] = [];
      for (const d of data) {
        Ak.push([d.x * d.r2, d.x * d.r2 * d.r2]);
        bk.push((d.o.pixel.u - warm.cx) / warm.fx - d.x);
        Ak.push([d.y * d.r2, d.y * d.r2 * d.r2]);
        bk.push((d.o.pixel.v - warm.cy) / warm.fy - d.y);
      }
      const sk = leastSquares(Ak, bk);
      if (sk) [ck1, ck2] = sk.x;
      if (Math.abs(ck1) > 1e3 || Math.abs(ck2) > 1e3)
        return fail(
          "diverged",
          `热启动畸变系数爆增 k1=${ck1}, k2=${ck2}，判定极端畸变，非线性优化不可辨识。`
        );
      const A: Matrix = [];
      const b: number[] = [];
      for (const d of data) {
        const f = 1 + ck1 * d.r2 + ck2 * d.r2 * d.r2;
        A.push([d.x * f, 1, 0, 0]);
        b.push(d.o.pixel.u);
        A.push([0, 0, d.y * f, 1]);
        b.push(d.o.pixel.v);
      }
      const si = leastSquares(A, b);
      if (!si || si.x[0] <= 0 || si.x[2] <= 0) break;
      warm = { fx: si.x[0], cx: si.x[1], fy: si.x[2], cy: si.x[3], k1: ck1, k2: ck2 };
    }
  }

  // 第 3 阶段：高斯-牛顿（LM 阻尼）联合精修并做严格收敛判断。
  const refined = gaussNewtonRefine(valid, warm, {
    maxIterations: config.maxIterations,
    tol: config.tol,
    includeK: config.estimateK,
  });
  const intr: Intrinsics = refined.intrinsics;
  const history: number[] = refined.rmseHistory;
  const converged = refined.converged;
  const reason = refined.reason;

  // 极端畸变的独立诊断：结果 k 超界或最终 RMSE 仍在上升/非有限。
  if (
    Math.abs(intr.k1) > 1e3 ||
    Math.abs(intr.k2) > 1e3 ||
    !Number.isFinite(intr.fx)
  ) {
    return fail(
      "diverged",
      `精修得到极端参数 k1=${intr.k1}, k2=${intr.k2}，判定极端畸变不收敛。`
    );
  }

  const finalErrors = evaluateErrors(valid, intr);
  const { rmse, max } = rmseOf(finalErrors);
  return {
    status: converged ? "ok" : "diverged",
    intrinsics: intr,
    distortionEstimated: config.estimateK,
    iterations: refined.iterations,
    converged,
    convergenceReason: reason,
    rmseHistory: history,
    rmse,
    maxError: max,
    errors: finalErrors,
    diagnostics: baseDiag,
    config,
  };
}

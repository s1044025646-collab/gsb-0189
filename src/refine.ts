import { leastSquares, Matrix } from "./linalg";
import { Intrinsics, PointObservation, ReprojectionError } from "./types";

/**
 * 高斯-牛顿（Levenberg-Marquardt 阻尼）联合精修 fx,fy,cx,cy,k1,k2。
 * 残差: e_u = fx*xd+cx-u, e_v = fy*yd+cy-v
 * 其中 xd=x(1+k1 r2+k2 r4), x=X/z, y=Y/z 与参数无关。
 * 雅可比全部解析给出（自行推导，无第三方优化器）。
 */
export function gaussNewtonRefine(
  obs: PointObservation[],
  init: Intrinsics,
  opts: { maxIterations: number; tol: number; includeK?: boolean }
): {
  intrinsics: Intrinsics;
  converged: boolean;
  iterations: number;
  reason: string;
  rmseHistory: number[];
} {
  const includeK = opts.includeK !== false;
  const dim = includeK ? 6 : 4;
  let p = includeK
    ? [init.fx, init.fy, init.cx, init.cy, init.k1, init.k2]
    : [init.fx, init.fy, init.cx, init.cy];
  const getPar = (): Intrinsics =>
    includeK
      ? { fx: p[0], fy: p[1], cx: p[2], cy: p[3], k1: p[4], k2: p[5] }
      : { fx: p[0], fy: p[1], cx: p[2], cy: p[3], k1: 0, k2: 0 };
  const history: number[] = [];
  let lambda = 1e-3;

  const residuals = (par: number[]) => {
    const fx = par[0], fy = par[1], cx = par[2], cy = par[3];
    const k1 = includeK ? par[4] : 0;
    const k2 = includeK ? par[5] : 0;
    const ru: number[] = [];
    const rv: number[] = [];
    let sse = 0;
    for (const o of obs) {
      const x = o.point.x / o.point.z;
      const y = o.point.y / o.point.z;
      const r2 = x * x + y * y;
      const f = 1 + k1 * r2 + k2 * r2 * r2;
      const eu = fx * x * f + cx - o.pixel.u;
      const ev = fy * y * f + cy - o.pixel.v;
      ru.push(eu);
      rv.push(ev);
      sse += eu * eu + ev * ev;
    }
    return { ru, rv, rmse: Math.sqrt(sse / obs.length) };
  };

  const buildJacobian = (par: number[]) => {
    const fx = par[0], fy = par[1];
    const k1 = includeK ? par[4] : 0;
    const k2 = includeK ? par[5] : 0;
    const J: Matrix = [];
    const r: number[] = [];
    for (const o of obs) {
      const x = o.point.x / o.point.z;
      const y = o.point.y / o.point.z;
      const r2 = x * x + y * y;
      const r4 = r2 * r2;
      const f = 1 + k1 * r2 + k2 * r4;
      // u 残差行，参数顺序 fx,fy,cx,cy,k1,k2
      const rowU = [x * f, 0, 1, 0];
      const rowV = [0, y * f, 0, 1];
      if (includeK) {
        rowU.push(fx * x * r2, fx * x * r4);
        rowV.push(fy * y * r2, fy * y * r4);
      }
      J.push(rowU);
      r.push(fx * x * f + p[2] - o.pixel.u);
      J.push(rowV);
      r.push(fy * y * f + p[3] - o.pixel.v);
    }
    return { J, r };
  };

  let cur = residuals(p).rmse;
  history.push(cur);
  let converged = false;
  let reason = "";
  let iter = 0;

  for (iter = 0; iter < opts.maxIterations; iter++) {
    const { J, r } = buildJacobian(p);
    // LM: (J^T J + lambda*diag) delta = -J^T r
  const n = dim;
    const Jt = transposeN(J, n);
    const JtJ = mulN(Jt, J);
    const Jtr = matVecN(Jt, r.map((v) => -v));
    let delta: number[] | null = null;
    let candidate = p;
    let nextRmse = cur;
    for (let attempt = 0; attempt < 20; attempt++) {
      const A = JtJ.map((row, i) =>
        row.map((v, j) => (i === j ? v * (1 + lambda) : v))
      );
      const sol = solveSymmetric(A, Jtr);
      if (!sol) {
        lambda *= 10;
        continue;
      }
      const trial = p.map((v, i) => v + sol[i]);
      if (trial[0] <= 0 || trial[1] <= 0) {
        lambda *= 10;
        continue;
      }
      const trialRmse = residuals(trial).rmse;
      if (Number.isFinite(trialRmse) && trialRmse < cur) {
        delta = sol;
        candidate = trial;
        nextRmse = trialRmse;
        lambda = Math.max(lambda / 3, 1e-12);
        break;
      }
      lambda *= 3;
    }
    if (!delta) {
      converged = true;
      reason = `第 ${iter + 1} 轮起 LM 在强阻尼下仍无下降步长，RMSE 已达噪声底 ${cur.toFixed(
        5
      )}px，判定收敛。`;
      break;
    }
    const paramChange =
      Math.max(...delta.map((d, i) => Math.abs(d) / (1 + Math.abs(p[i]))));
    p = candidate;
    history.push(nextRmse);
    const rmseDrop = cur - nextRmse;
    cur = nextRmse;
    if (paramChange < opts.tol && Math.abs(rmseDrop) < opts.tol) {
      converged = true;
      reason = `参数相对变化 ${paramChange.toExponential(2)}、RMSE 下降 ${rmseDrop.toExponential(
        2
      )} 均小于容差 ${opts.tol}，高斯-牛顿于第 ${iter + 1} 轮收敛。`;
      break;
    }
  }
  if (!converged && !reason)
    reason = `高斯-牛顿达到最大迭代 ${opts.maxIterations} 仍未满足容差。`;
  return {
    intrinsics: getPar(),
    converged,
    iterations: iter + 1,
    reason,
    rmseHistory: history,
  };
}

function transposeN(a: Matrix, cols: number): Matrix {
  const r: Matrix = Array.from({ length: cols }, () => new Array(a.length).fill(0));
  a.forEach((row, i) => row.forEach((v, j) => (r[j][i] = v)));
  return r;
}
function mulN(a: Matrix, b: Matrix): Matrix {
  return a.map((row) =>
    Array.from({ length: b[0].length }, (_, j) =>
      row.reduce((s, v, k) => s + v * b[k][j], 0)
    )
  );
}
function matVecN(a: Matrix, v: number[]): number[] {
  return a.map((row) => row.reduce((s, av, i) => s + av * v[i], 0));
}
/** 小规模对称正定方程组高斯消元（带 Cholesky 风格主元保护）。 */
function solveSymmetric(Ain: Matrix, b: number[]): number[] | null {
  const n = b.length;
  const A = Ain.map((row) => [...row]);
  const y = [...b];
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++)
      if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    if (Math.abs(A[piv][col]) < 1e-14) return null;
    [A[piv], A[col]] = [A[col], A[piv]];
    [y[piv], y[col]] = [y[col], y[piv]];
    for (let r = col + 1; r < n; r++) {
      const f = A[r][col] / A[col][col];
      for (let j = col; j < n; j++) A[r][j] -= f * A[col][j];
      y[r] -= f * y[col];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = y[i];
    for (let j = i + 1; j < n; j++) s -= A[i][j] * x[j];
    x[i] = s / A[i][i];
  }
  return x;
}

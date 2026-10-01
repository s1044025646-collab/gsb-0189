/**
 * 自实现的最小二乘与秩诊断，不依赖任何外部数学库。
 * 仅使用高斯消元（列主元）、正规方程与 3x3 对称矩阵 Jacobi 特征分解。
 */

export function isFiniteNum(x: number): boolean {
  return typeof x === "number" && Number.isFinite(x);
}

/** 解线性方程组 M x = b，M 为 n x n 行主序。奇异返回 null。 */
export function solveLinear(M: number[], b: number[], n: number): number[] | null {
  const A: number[][] = [];
  for (let i = 0; i < n; i++) {
    A.push([...M.slice(i * n, i * n + n), b[i]!]);
  }
  for (let col = 0; col < n; col++) {
    let piv = col;
    let max = Math.abs(A[col]![col]!);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(A[r]![col]!);
      if (v > max) {
        max = v;
        piv = r;
      }
    }
    if (!(max > 1e-14)) return null;
    if (piv !== col) {
      const tmp = A[col]!;
      A[col] = A[piv]!;
      A[piv] = tmp;
    }
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = A[r]![col]! / A[col]![col]!;
      for (let c = col; c <= n; c++) {
        A[r]![c] = A[r]![c]! - factor * A[col]![c]!;
      }
    }
  }
  const x = new Array<number>(n);
  for (let i = 0; i < n; i++) x[i] = A[i]![n]! / A[i]![i]!;
  return x;
}

/**
 * 超定最小二乘 min ||A x - b||，A 为 m x n（m>=n），正规方程求解。
 * 同时返回 A^T A 的特征值，供秩/条件数诊断。
 */
export interface LSResult {
  x: number[] | null;
  eigenvalues: number[];
  singularValues: number[];
  rank: number;
  conditionNumber: number;
}

export function leastSquares(
  A: number[][],
  b: number[],
  tol = 1e-10,
): LSResult {
  const m = A.length;
  const n = m > 0 ? A[0]!.length : 0;
  const AtA = new Array(n * n).fill(0);
  const Atb = new Array(n).fill(0);
  for (let i = 0; i < m; i++) {
    for (let p = 0; p < n; p++) {
      Atb[p]! += A[i]![p]! * b[i]!;
      for (let q = 0; q < n; q++) {
        AtA[p * n + q]! += A[i]![p]! * A[i]![q]!;
      }
    }
  }
  const eig = jacobiEigenvalues(AtA, n);
  const singular = eig.map((e) => Math.sqrt(Math.max(0, e)));
  const maxS = Math.max(...singular, 0);
  // 相对容秩阈值（尺度无关）
  const threshold = tol * Math.max(1, maxS);
  const rank = singular.filter((s) => s > threshold).length;
  const minS = Math.min(...singular);
  const condition = minS > 0 ? maxS / minS : Infinity;
  const x = rank === n ? solveLinear(AtA, Atb, n) : null;
  return {
    x,
    eigenvalues: eig,
    singularValues: singular,
    rank,
    conditionNumber: condition,
  };
}

/** 对称矩阵特征值（Jacobi 旋转），M 行主序 n x n，n<=4。 */
export function jacobiEigenvalues(M: number[], n: number): number[] {
  const A: number[][] = [];
  for (let i = 0; i < n; i++) A.push([...M.slice(i * n, i * n + n)]);
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++)
      for (let q = p + 1; q < n; q++) off += Math.abs(A[p]![q]!);
    if (off < 1e-14) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = A[p]![q]!;
        if (Math.abs(apq) < 1e-16) continue;
        const app = A[p]![p]!;
        const aqq = A[q]![q]!;
        const tau = (aqq - app) / (2 * apq);
        const t =
          tau >= 0
            ? 1 / (tau + Math.sqrt(1 + tau * tau))
            : 1 / (tau - Math.sqrt(1 + tau * tau));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = A[k]![p]!;
          const akq = A[k]![q]!;
          A[k]![p] = c * akp - s * akq;
          A[k]![q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = A[p]![k]!;
          const aqk = A[q]![k]!;
          A[p]![k] = c * apk - s * aqk;
          A[q]![k] = s * apk + c * aqk;
        }
      }
    }
  }
  return A.map((row, i) => row[i]!).sort((a, b) => b - a);
}

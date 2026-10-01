/**
 * 最小二乘所需的极简线性代数：全部自行实现，不依赖任何标定库。
 * 仅包含: 矩阵乘、转置、高斯-若当求逆(带奇异检测)、奇异值(Jacobi 迭代)。
 */

export type Matrix = number[][];

export function transpose(a: Matrix): Matrix {
  const m = a.length;
  const n = a[0].length;
  const r: Matrix = Array.from({ length: n }, () => new Array(m).fill(0));
  for (let i = 0; i < m; i++)
    for (let j = 0; j < n; j++) r[j][i] = a[i][j];
  return r;
}

export function multiply(a: Matrix, b: Matrix): Matrix {
  const m = a.length;
  const p = b[0].length;
  const n = b.length;
  const r: Matrix = Array.from({ length: m }, () => new Array(p).fill(0));
  for (let i = 0; i < m; i++)
    for (let k = 0; k < n; k++) {
      const aik = a[i][k];
      for (let j = 0; j < p; j++) r[i][j] += aik * b[k][j];
    }
  return r;
}

export function matVec(a: Matrix, v: number[]): number[] {
  return a.map((row) => row.reduce((s, aij, j) => s + aij * v[j], 0));
}

/** 高斯-若当求逆；奇异时返回 null。 */
export function inverse(a: Matrix): Matrix | null {
  const n = a.length;
  const m: Matrix = a.map((row, i) => {
    const id = new Array(n).fill(0);
    id[i] = 1;
    return [...row, ...id];
  });
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++)
      if (Math.abs(m[r][col]) > Math.abs(m[piv][col])) piv = r;
    if (Math.abs(m[piv][col]) < 1e-14) return null;
    [m[piv], m[col]] = [m[col], m[piv]];
    const d = m[col][col];
    for (let j = 0; j < 2 * n; j++) m[col][j] /= d;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = m[r][col];
      if (f === 0) continue;
      for (let j = 0; j < 2 * n; j++) m[r][j] -= f * m[col][j];
    }
  }
  return m.map((row) => row.slice(n));
}

/**
 * 求解线性最小二乘 min ||A x - b||^2，正规方程法。
 * 返回参数与 A^T A 矩阵（供秩/奇异值诊断）；奇异时返回 null。
 */
export function leastSquares(
  a: Matrix,
  b: number[]
): { x: number[]; normal: Matrix } | null {
  const at = transpose(a);
  const normal = multiply(at, a);
  const atb = matVec(at, b);
  const inv = inverse(normal);
  if (!inv) return null;
  return { x: matVec(inv, atb), normal };
}

/**
 * 对称矩阵的特征值（Jacobi 法），返回升序特征值与对应特征向量列。
 * 用于对 A^T A 做奇异值诊断（奇异值 = sqrt(特征值)）。
 */
export function symmetricEigen(a: Matrix): { values: number[]; vectors: Matrix } {
  const n = a.length;
  const v: Matrix = Array.from({ length: n }, (_, i) =>
    new Array(n).fill(0).map((__, j) => (i === j ? 1 : 0))
  );
  const m = a.map((r) => [...r]);
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++)
      for (let q = p + 1; q < n; q++) off += m[p][q] * m[p][q];
    if (off < 1e-24) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(m[p][q]) < 1e-15) continue;
        const tau = (m[q][q] - m[p][p]) / (2 * m[p][q]);
        const t =
          (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const mpk = m[p][k];
          const mqk = m[q][k];
          m[p][k] = c * mpk - s * mqk;
          m[q][k] = s * mpk + c * mqk;
        }
        for (let k = 0; k < n; k++) {
          const mkp = m[k][p];
          const mkq = m[k][q];
          m[k][p] = c * mkp - s * mkq;
          m[k][q] = s * mkp + c * mkq;
        }
        for (let k = 0; k < n; k++) {
          const vkp = v[k][p];
          const vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const values = m.map((row, i) => row[i]);
  const idx = values.map((_, i) => i).sort((i, j) => values[i] - values[j]);
  const sortedValues = idx.map((i) => Math.max(0, values[i]));
  const sortedVectors: Matrix = Array.from({ length: n }, (_, row) =>
    idx.map((col) => v[row][col])
  );
  return { values: sortedValues, vectors: sortedVectors };
}

/** 设计矩阵奇异值（降序）、秩、条件数。 */
export function rankDiagnostics(a: Matrix, tol: number): {
  singularValues: number[];
  rank: number;
  conditionNumber: number | null;
} {
  const at = transpose(a);
  const normal = multiply(at, a);
  const { values } = symmetricEigen(normal);
  const sv = values
    .map((e) => Math.sqrt(e))
    .sort((p, q) => q - p);
  const smax = sv[0] ?? 0;
  const rank = sv.filter((s) => s > tol * Math.max(1, smax)).length;
  const smin = sv[sv.length - 1] ?? 0;
  const conditionNumber = smin > 0 ? smax / smin : null;
  return { singularValues: sv, rank, conditionNumber };
}

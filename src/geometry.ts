import {
  EulerRotation,
  Extrinsics,
  Intrinsics,
  Vec3,
} from "./types";

/** R = Rz(rz) * Ry(ry) * Rx(rx)，ZYX 内旋。相机 +X 右, +Y 下, +Z 前。 */
export function eulerToMatrix(r: EulerRotation): number[][] {
  const cx = Math.cos(r.rx), sx = Math.sin(r.rx);
  const cy = Math.cos(r.ry), sy = Math.sin(r.ry);
  const cz = Math.cos(r.rz), sz = Math.sin(r.rz);
  // Rx
  const Rx = [
    [1, 0, 0],
    [0, cx, -sx],
    [0, sx, cx],
  ];
  const Ry = [
    [cy, 0, sy],
    [0, 1, 0],
    [-sy, 0, cy],
  ];
  const Rz = [
    [cz, -sz, 0],
    [sz, cz, 0],
    [0, 0, 1],
  ];
  const ryRx = mul3(Ry, Rx);
  return mul3(Rz, ryRx);
}

function mul3(a: number[][], b: number[][]): number[][] {
  const r: number[][] = Array.from({ length: 3 }, () => [0, 0, 0]);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++) r[i][j] += a[i][k] * b[k][j];
  return r;
}

export function transformWorldToCamera(
  p: Vec3,
  ext: Extrinsics
): Vec3 {
  const R = eulerToMatrix(ext.rotation);
  return {
    x: R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z + ext.translation.x,
    y: R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z + ext.translation.y,
    z: R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z + ext.translation.z,
  };
}

/** 径向畸变因子 1 + k1 r^2 + k2 r^4。 */
export function distortionFactor(
  x: number,
  y: number,
  k1: number,
  k2: number
): number {
  const r2 = x * x + y * y;
  return 1 + k1 * r2 + k2 * r2 * r2;
}

export interface ProjectResult {
  u: number;
  v: number;
  x: number;
  y: number;
  xd: number;
  yd: number;
  depth: number;
  behindCamera: boolean;
}

/**
 * 针孔 + 两径向系数投影。
 * z <= 0 的点视为镜头后方：仍返回数学结果但 behindCamera=true 并给出标记。
 */
export function project(
  p: Vec3,
  intr: Intrinsics
): ProjectResult {
  const behind = p.z <= 0;
  const zSafe = p.z === 0 ? NaN : p.z;
  const x = p.x / zSafe;
  const y = p.y / zSafe;
  const d = distortionFactor(x, y, intr.k1, intr.k2);
  const xd = x * d;
  const yd = y * d;
  return {
    u: intr.fx * xd + intr.cx,
    v: intr.fy * yd + intr.cy,
    x,
    y,
    xd,
    yd,
    depth: p.z,
    behindCamera: behind,
  };
}

export interface UnprojectResult {
  ok: boolean;
  reason?: string;
  iterations: number;
  /** 归一化无畸变坐标。 */
  x?: number;
  y?: number;
  /** 给定深度的相机坐标三维点。 */
  point?: Vec3;
}

/**
 * 给定像素与深度 z 的反投影:
 * 1) 由观测像素得到归一化畸变坐标 xd=(u-cx)/fx, yd=(v-cy)/fy。
 * 2) 定点迭代求无畸变坐标:
 *      x_{n+1} = xd / (1 + k1 r_n^2 + k2 r_n^4)
 *    收敛于 x 使 distort(x)=xd；这是自行实现的迭代去畸变。
 * 3) X = x*z, Y = y*z, Z = z。
 * 半径发散或超过最大迭代 -> ok=false（极端畸变不收敛诊断）。
 */
export function unproject(
  u: number,
  v: number,
  depth: number,
  intr: Intrinsics,
  opts: { maxIter: number; tol: number; maxRadius: number }
): UnprojectResult {
  if (depth <= 0) {
    return {
      ok: false,
      reason: `深度 z=${depth} 非正：点位于镜头后方或光心平面上，无法反投影到前方。`,
      iterations: 0,
    };
  }
  const xd = (u - intr.cx) / intr.fx;
  const yd = (v - intr.cy) / intr.fy;
  if (!(Number.isFinite(xd) && Number.isFinite(yd))) {
    return { ok: false, reason: "fx/fy 非法或像素非有限值。", iterations: 0 };
  }
  let x = xd;
  let y = yd;
  let iter = 0;
  for (; iter < opts.maxIter; iter++) {
    const r2 = x * x + y * y;
    const factor = 1 + intr.k1 * r2 + intr.k2 * r2 * r2;
    if (!Number.isFinite(factor) || Math.abs(factor) < 1e-12) {
      return {
        ok: false,
        reason: `迭代去畸变在第 ${iter} 次迭代遇到畸变因子 ${factor}（极端畸变，数值发散）。`,
        iterations: iter,
      };
    }
    const xn = xd / factor;
    const yn = yd / factor;
    if (Math.hypot(xn, yn) > opts.maxRadius || !Number.isFinite(xn)) {
      return {
        ok: false,
        reason: `迭代去畸变半径超过 ${opts.maxRadius}（归一化坐标发散），判定为极端畸变不收敛。`,
        iterations: iter,
      };
    }
    const dx = xn - x;
    const dy = yn - y;
    x = xn;
    y = yn;
    if (Math.hypot(dx, dy) < opts.tol) {
      return {
        ok: true,
        iterations: iter + 1,
        x,
        y,
        point: { x: x * depth, y: y * depth, z: depth },
      };
    }
  }
  return {
    ok: false,
    reason: `迭代去畸变在 ${opts.maxIter} 次后仍未达到容差 ${opts.tol}。`,
    iterations: iter,
  };
}

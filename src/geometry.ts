import type {
  CamPoint,
  Extrinsics,
  Intrinsics,
  Point2D,
  Vec3,
  WorldPoint,
} from "./types.js";

/** 加性可分离 PRNG（固定种子，mulberry32）。 */
export function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return function () {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Box-Muller 标准正态，基于给定均匀随机源。 */
export function gaussian(rand: () => number): number {
  let u = 0;
  let v = 0;
  while (u === 0) u = rand();
  while (v === 0) v = rand();
  return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
}

export function matVec(R: number[], p: Vec3): Vec3 {
  return {
    x: R[0]! * p.x + R[1]! * p.y + R[2]! * p.z,
    y: R[3]! * p.x + R[4]! * p.y + R[5]! * p.z,
    z: R[6]! * p.x + R[7]! * p.y + R[8]! * p.z,
  };
}

/** Xc = R Xw + t。 */
export function worldToCamera(p: WorldPoint, ext: Extrinsics): CamPoint {
  const r = matVec(ext.R, p);
  return {
    x: r.x + ext.t[0]!,
    y: r.y + ext.t[1]!,
    z: r.z + ext.t[2]!,
  };
}

/** 由轴角（弧度）构造旋转矩阵，罗德里格斯公式。axis 无需单位化。 */
export function rotationFromAxisAngle(
  axis: [number, number, number],
  angle: number,
): number[] {
  const n = Math.hypot(axis[0], axis[1], axis[2]) || 1;
  const ax = axis[0]! / n;
  const ay = axis[1]! / n;
  const az = axis[2]! / n;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const C = 1 - c;
  return [
    c + ax * ax * C,
    ax * ay * C - az * s,
    ax * az * C + ay * s,
    ay * ax * C + az * s,
    c + ay * ay * C,
    ay * az * C - ax * s,
    az * ax * C - ay * s,
    az * ay * C + ax * s,
    c + az * az * C,
  ];
}

/** 径向畸变因子：1 + k1 r^2 + k2 r^4，r 为归一化平面半径。 */
export function distortFactor(xn: number, yn: number, k: Intrinsics): number {
  const r2 = xn * xn + yn * yn;
  return 1 + k.k1 * r2 + k.k2 * r2 * r2;
}

/**
 * 针孔正向投影（含径向畸变），输入相机坐标点。
 * xn=X/Z, yn=Y/Z；xd=xn*d；u=fx*xd+cx。
 * Z<=0（镜头后方/在镜头平面）返回 null。
 */
export function projectCameraPoint(
  p: CamPoint,
  k: Intrinsics,
): Point2D | null {
  if (!(p.z > 0)) return null;
  const xn = p.x / p.z;
  const yn = p.y / p.z;
  const d = distortFactor(xn, yn, k);
  return {
    u: k.fx * xn * d + k.cx,
    v: k.fy * yn * d + k.cy,
  };
}

/** 世界坐标点投影：先变外参再投影。 */
export function projectWorldPoint(
  p: WorldPoint,
  ext: Extrinsics,
  k: Intrinsics,
): Point2D | null {
  return projectCameraPoint(worldToCamera(p, ext), k);
}

export interface UndistortResult {
  xn: number;
  yn: number;
  converged: boolean;
  iterations: number;
}

/**
 * 给定观测到的（已畸变）归一化坐标 (xd, yd)，迭代求解无畸变归一化坐标。
 * 采用带阻尼的牛顿/不动点迭代：
 *   解  f(xn) = xn * D(xn) - xd = 0，其中 D=1+k1 r^2+k2 r^4。
 * 使用标量径向结构：令 rho=|xn|，rd=|xd|，满足 rd = rho*(1+k1 rho^2+k2 rho^4)，
 * 对 rho 做牛顿迭代（一维，稳定且易判发散），方向保持 xd/rd。
 */
export function undistortNormalized(
  xd: number,
  yd: number,
  k: Intrinsics,
  maxIter = 50,
  tol = 1e-12,
): UndistortResult {
  const rd = Math.hypot(xd, yd);
  if (rd === 0) return { xn: 0, yn: 0, converged: true, iterations: 0 };
  // 初始猜测：忽略畸变
  let rho = rd;
  let iter = 0;
  let converged = false;
  for (; iter < maxIter; iter++) {
    const r2 = rho * rho;
    const D = 1 + k.k1 * r2 + k.k2 * r2 * r2;
    const f = rho * D - rd;
    // d(rho*D)/d rho
    const dD = 2 * k.k1 * rho + 4 * k.k2 * rho * r2;
    const fp = D + rho * dD;
    if (Math.abs(f) < tol) {
      converged = true;
      break;
    }
    if (!(Math.abs(fp) > 1e-12) || !isFinite(rho)) break;
    let step = f / fp;
    // 限制步长，避免极端畸变下越过非物理解/振荡
    const maxStep = Math.max(0.5 * rho, 1e-6);
    if (Math.abs(step) > maxStep) step = Math.sign(step) * maxStep;
    const next = rho - step;
    if (next < 0) {
      rho = rho * 0.5;
    } else {
      rho = next;
    }
    if (Math.abs(step) < tol) {
      converged = true;
      break;
    }
  }
  // 校验：正向重算应能回到 rd
  const r2 = rho * rho;
  const back = rho * (1 + k.k1 * r2 + k.k2 * r2 * r2);
  if (Math.abs(back - rd) > 1e-5 * Math.max(1, rd)) converged = false;
  const scale = rho / rd;
  return { xn: xd * scale, yn: yd * scale, converged, iterations: iter };
}

export interface BackprojectResult {
  /** 反投影射线在相机坐标下、深度为 depth（米）处的三维点 */
  point: CamPoint | null;
  ray: Vec3 | null;
  converged: boolean;
  iterations: number;
  reason: string;
}

/**
 * 给定像素与指定深度 Z（米）反投影三维点。
 * 先去畸变归一化坐标，再 X = xn*Z, Y = yn*Z。
 * depth<=0 直接拒绝。
 */
export function backproject(
  pixel: Point2D,
  depth: number,
  k: Intrinsics,
): BackprojectResult {
  if (!(depth > 0)) {
    return {
      point: null,
      ray: null,
      converged: false,
      iterations: 0,
      reason: "深度必须为正（点须在镜头前方）",
    };
  }
  if (!(k.fx > 0) || !(k.fy > 0)) {
    return {
      point: null,
      ray: null,
      converged: false,
      iterations: 0,
      reason: "fx/fy 必须为正",
    };
  }
  const xd = (pixel.u - k.cx) / k.fx;
  const yd = (pixel.v - k.cy) / k.fy;
  const u = undistortNormalized(xd, yd, k);
  if (!u.converged) {
    return {
      point: null,
      ray: { x: u.xn, y: u.yn, z: 1 },
      converged: false,
      iterations: u.iterations,
      reason: "去畸变迭代不收敛（畸变过强或像素超出有效范围）",
    };
  }
  return {
    point: { x: u.xn * depth, y: u.yn * depth, z: depth },
    ray: { x: u.xn, y: u.yn, z: 1 },
    converged: true,
    iterations: u.iterations,
    reason: "ok",
  };
}

/** 重投影残差（像素）。Z<=0 返回 null。 */
export function reprojectionError(
  p: CamPoint,
  observed: Point2D,
  k: Intrinsics,
): { uResidual: number; vResidual: number; distance: number } | null {
  const proj = projectCameraPoint(p, k);
  if (!proj) return null;
  const du = proj.u - observed.u;
  const dv = proj.v - observed.v;
  return {
    uResidual: du,
    vResidual: dv,
    distance: Math.hypot(du, dv),
  };
}

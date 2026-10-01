/**
 * 统一类型定义与坐标系约定。
 *
 * 坐标系（右手）：
 *   - 世界坐标 (Xw, Yw, Zw) 与相机坐标 (Xc, Yc, Zc) 均为右手直角坐标系，单位：米 (meter)。
 *   - 相机朝 +Zc 方向观察；有效点必须满足 Zc > 0（镜头前方）。
 *   - Xc 指向相机右侧，Yc 指向相机下方（OpenCV 风格）。
 * 像素坐标：
 *   - u 沿像素列方向向右增大，v 沿像素行方向向下增大；原点在图像左上角。
 *   - fx, fy 单位为像素；cx, cy 单位为像素。
 * 外参：Xc = R * Xw + t，R 为 3x3 旋转矩阵（行主序平铺为长度 9 的数组），t 为长度 3 的平移向量（米）。
 * 长度单位：所有三维坐标、平移、深度统一为米；像素与内参为像素。
 */

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface Point2D {
  u: number;
  v: number;
}

/** 内参：fx, fy, cx, cy（像素），两个径向畸变系数 k1, k2（无量纲）。 */
export interface Intrinsics {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  k1: number;
  k2: number;
}

/** 外参：3x3 旋转矩阵（行主序，长度 9）+ 平移向量（米）。 */
export interface Extrinsics {
  /** 长度 9，R[row*3+col] */
 R: number[];
  t: [number, number, number];
}

/** 相机坐标下的三维点（标定阶段输入，单位米）。 */
export interface CamPoint extends Vec3 {}

/** 世界坐标三维点。 */
export interface WorldPoint extends Vec3 {}

/** 一个标定观测：三维点（相机坐标）+ 对应像素。 */
export interface Observation {
  id: string;
  point: CamPoint;
  pixel: Point2D;
}

export interface PerPointError {
  id: string;
  u: number;
  v: number;
  uResidual: number;
  vResidual: number;
  /** 重投影欧氏距离 sqrt(du^2+dv^2)，像素 */
  distance: number;
}

export interface CalibDiagnostics {
  pointCount: number;
  /** 线性设计矩阵（列 [x/z, y/z, 1]）的有效秩 */
  rank: number;
  /** 期望秩（3） */
  expectedRank: number;
  /** 最小奇异值（尺度相关） */
  minSingular: number;
  /** 条件数估计（最大/最小奇异值） */
  conditionNumber: number;
  /** 景深范围诊断：相机坐标 Z 的最小/最大值 */
  zMin: number;
  zMax: number;
  /** 落在镜头后方（Z<=0）被剔除的点数 */
  behindCameraCount: number;
  /** 几何是否可辨识 */
  identifiable: boolean;
  /** 人类可读的退化/警告原因 */
  warnings: string[];
}

export interface CalibResult {
  intrinsics: Intrinsics;
  /** 初始无畸变线性内参，用于对照 */
  linearIntrinsics: Intrinsics;
  diagnostics: CalibDiagnostics;
  /** RMS 重投影误差（像素），sqrt(mean(du^2+dv^2)) */
  rms: number;
  /** 每点残差 */
  perPoint: PerPointError[];
  /** 交替/非线性优化迭代次数 */
  iterations: number;
  converged: boolean;
  convergenceReason: string;
  /** 是否出现因极端畸变导致的去畸变不收敛 */
  undistortFailedCount: number;
}

export interface SyntheticConfig {
  seed: number;
  width: number;
  height: number;
  intrinsics: Intrinsics;
  pointCount: number;
  /** 观测高斯噪声标准差（像素） */
  noiseSigmaPx: number;
  /** 三维点深度范围（米） */
  zRange: [number, number];
  /** 相机坐标 x/y 半幅（米），控制视场 */
  xyHalf: number;
}

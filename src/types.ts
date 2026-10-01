/**
 * 全局类型定义。
 *
 * 坐标约定（贯穿全项目）:
 * - 世界坐标与相机坐标均为右手坐标系，长度单位默认为毫米(mm)（可由数据集声明，
 *   但必须全套数据一致；尺度不影响 fx/fy 的可辨识性，只影响其数值单位）。
 * - 相机坐标系: +X 向右, +Y 向下, +Z 指向场景前方（光轴）。相机看向 +Z。
 * - 像素坐标系: u 向右增大, v 向下增大；主点 (cx, cy) 单位为像素。
 * - 畸变模型为两个径向系数 k1, k2（无切向畸变）:
 *     x_d = x * (1 + k1*r^2 + k2*r^4),  y_d 同理，其中 (x,y) 为归一化平面坐标。
 */

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/** 旋转，以 ZYX 内旋欧拉角给出（弧度），R = Rz(rz) * Ry(ry) * Rx(rx)。 */
export interface EulerRotation {
  rx: number;
  ry: number;
  rz: number;
}

export interface Extrinsics {
  /** 世界 -> 相机: P_cam = R * P_world + t。t 单位与点集一致。 */
  rotation: EulerRotation;
  translation: Vec3;
}

export interface Intrinsics {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  /** 径向畸变系数（可为 0）。 */
  k1: number;
  k2: number;
}

export interface PointObservation {
  /** 三维点（相机坐标系），标定入口要求点已在相机坐标系下（外参已知）。 */
  point: Vec3;
  /** 观测像素。 */
  pixel: { u: number; v: number };
  id?: string;
}

export interface WorldPointObservation {
  pointWorld: Vec3;
  pixel: { u: number; v: number };
  id?: string;
}

export interface ReprojectionError {
  id: string;
  u: number;
  v: number;
  uReproj: number;
  vReproj: number;
  residualU: number;
  residualV: number;
  /** 像素欧氏距离。 */
  distance: number;
  /** 相机坐标深度 z（<=0 的点在镜头后方）。 */
  depth: number;
  behindCamera: boolean;
}

export interface CalibrationConfig {
  /** 是否估计两个径向畸变系数。 */
  estimateK: boolean;
  /** 交替优化最大迭代次数。 */
  maxIterations: number;
  /** 参数与 RMSE 相对变化小于该阈值 -> 收敛。 */
  tol: number;
  undistortMaxIter: number;
  undistortTol: number;
  /** 迭代去畸变发散判定半径。 */
  undistortMaxRadius: number;
  /** 设计矩阵相对奇异值阈值，低于则判退化。 */
  rankTol: number;
  minPoints: number;
}

export const DEFAULT_CONFIG: CalibrationConfig = {
  estimateK: true,
  maxIterations: 50,
  tol: 1e-10,
  undistortMaxIter: 100,
  undistortTol: 1e-9,
  undistortMaxRadius: 10,
  rankTol: 1e-9,
  minPoints: 6,
};

export type CalibrationStatus =
  | "ok"
  | "degenerate"
  | "diverged"
  | "insufficient_points"
  | "all_points_behind_camera";

export interface CalibrationResult {
  status: CalibrationStatus;
  intrinsics: Intrinsics | null;
  distortionEstimated: boolean;
  iterations: number;
  converged: boolean;
  convergenceReason: string;
  rmseHistory: number[];
  rmse: number | null;
  maxError: number | null;
  errors: ReprojectionError[];
  diagnostics: {
    totalPoints: number;
    usedPoints: number;
    behindCameraPoints: string[];
    matrixRank: number;
    matrixCols: number;
    singularValues: number[];
    conditionNumber: number | null;
    warnings: string[];
    normalizedXRange: [number, number] | null;
    normalizedYRange: [number, number] | null;
    depthRange: [number, number] | null;
  };
  config: CalibrationConfig;
}

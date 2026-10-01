import { gaussian, mulberry32, projectCameraPoint, rotationFromAxisAngle } from "./geometry.js";
import type {
  CamPoint,
  Extrinsics,
  Intrinsics,
  Observation,
  Point2D,
  SyntheticConfig,
  WorldPoint,
} from "./types.js";

export const DEFAULT_CONFIG: SyntheticConfig = {
  seed: 42,
  width: 1280,
  height: 800,
  intrinsics: { fx: 900, fy: 900, cx: 640, cy: 400, k1: -0.28, k2: 0.08 },
  pointCount: 60,
  noiseSigmaPx: 0.4,
  zRange: [1.5, 4.0],
  xyHalf: 0.8,
};

export interface SyntheticBundle {
  config: SyntheticConfig;
  worldPoints: WorldPoint[];
  /** 生成时用的外参 */
  extrinsics: Extrinsics;
  cameraPoints: CamPoint[];
  /** 无噪声理想像素 */
  cleanPixels: Point2D[];
  observations: Observation[];
  behindCameraCount: number;
}

/**
 * 生成合成数据：在相机前方生成随机世界点，施加一次非单位外参，
 * 用真实内参（含畸变）投影，再加固定种子高斯像素噪声。
 */
export function generateSynthetic(
  partial: Partial<SyntheticConfig> = {},
  extrinsics?: Extrinsics,
): SyntheticBundle {
  const config: SyntheticConfig = {
    ...DEFAULT_CONFIG,
    ...partial,
    intrinsics: { ...DEFAULT_CONFIG.intrinsics, ...(partial.intrinsics ?? {}) },
    zRange: partial.zRange ?? DEFAULT_CONFIG.zRange,
  };
  const rand = mulberry32(config.seed);
  const ext: Extrinsics =
    extrinsics ??
    ({
      R: rotationFromAxisAngle([0, 1, 0], 0.12),
      t: [0.05, -0.02, 0.0],
    });

  const worldPoints: WorldPoint[] = [];
  const cameraPoints: CamPoint[] = [];
  const cleanPixels: Point2D[] = [];
  const observations: Observation[] = [];
  let behind = 0;

  let attempts = 0;
  while (
    observations.length < config.pointCount &&
    attempts < config.pointCount * 50
  ) {
    attempts++;
    // 在相机前方直接采样相机坐标，再反推世界坐标，保证点在前方且覆盖视场
    const z = config.zRange[0] + rand() * (config.zRange[1] - config.zRange[0]);
    const cam: CamPoint = {
      x: (rand() * 2 - 1) * config.xyHalf,
      y: (rand() * 2 - 1) * config.xyHalf * (config.height / config.width),
      z,
    };
    const clean = projectCameraPoint(cam, config.intrinsics);
    if (!clean) {
      behind++;
      continue;
    }
    // 剔除落在像面外的点（畸变过强可能把点甩出）
    if (
      clean.u < 0 ||
      clean.v < 0 ||
      clean.u > config.width ||
      clean.v > config.height
    ) {
      continue;
    }
    const world = inverseCamera(cam, ext);
    const idx = observations.length;
    worldPoints.push(world);
    cameraPoints.push(cam);
    cleanPixels.push(clean);
    observations.push({
      id: `p${idx.toString().padStart(3, "0")}`,
      point: cam,
      pixel: {
        u: clean.u + gaussian(rand) * config.noiseSigmaPx,
        v: clean.v + gaussian(rand) * config.noiseSigmaPx,
      },
    });
  }

  return {
    config,
    worldPoints,
    extrinsics: ext,
    cameraPoints,
    cleanPixels,
    observations,
    behindCameraCount: behind,
  };
}

/** 近似 Xw = R^T (Xc - t)，仅用于合成数据记录。 */
function inverseCamera(cam: CamPoint, ext: Extrinsics): WorldPoint {
  const Rt = transpose3(ext.R);
  const q: CamPoint = {
    x: cam.x - ext.t[0],
    y: cam.y - ext.t[1],
    z: cam.z - ext.t[2],
  };
  return {
    x: Rt[0]! * q.x + Rt[1]! * q.y + Rt[2]! * q.z,
    y: Rt[3]! * q.x + Rt[4]! * q.y + Rt[5]! * q.z,
    z: Rt[6]! * q.x + Rt[7]! * q.y + Rt[8]! * q.z,
  };
}

function transpose3(R: number[]): number[] {
  return [
    R[0], R[3], R[6],
    R[1], R[4], R[7],
    R[2], R[5], R[8],
  ] as number[];
}

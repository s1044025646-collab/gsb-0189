import { project } from "./geometry";
import { Extrinsics, Intrinsics, PointObservation, Vec3, WorldPointObservation } from "./types";

/** 可复现的 mulberry32 伪随机数（固定种子）。 */
export function seededRng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** 标准正态样本（Box-Muller），用于高斯像素噪声。 */
function gaussian(rng: () => number): number {
  let u = 0;
  let v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export interface GenerateOptions {
  seed: number;
  intrinsics: Intrinsics;
  /** 网格行列数（相机坐标点）。 */
  cols: number;
  rows: number;
  /** 点的 x/y 跨度（单位与长度一致，例如 mm）。 */
  width: number;
  height: number;
  /** 基准深度及深度变化幅度（使点不全共面）。 */
  zBase: number;
  zJitter: number;
  /** 像素高斯噪声标准差（像素）。 */
  noiseSigma: number;
  /** 可选外参：若给出则三维点按世界坐标生成并变换到相机坐标。 */
  extrinsics?: Extrinsics;
  /** 单位标签，仅元数据。 */
  unit?: string;
}

export interface GeneratedSet {
  observations: PointObservation[];
  world: WorldPointObservation[] | null;
  meta: {
    seed: number;
    noiseSigma: number;
    unit: string;
    generatedAt: string;
  };
}

import { transformWorldToCamera } from "./geometry";

/** 生成带固定种子噪声的合成三维点-像素对应。 */
export function generate(opts: GenerateOptions): GeneratedSet {
  const rng = seededRng(opts.seed);
  const observations: PointObservation[] = [];
  let world: WorldPointObservation[] | null = opts.extrinsics ? [] : null;
  for (let r = 0; r < opts.rows; r++) {
    for (let c = 0; c < opts.cols; c++) {
      const id = `g_${r}_${c}`;
      const x = (c / (opts.cols - 1) - 0.5) * opts.width;
      const y = (r / (opts.rows - 1) - 0.5) * opts.height;
      const z = opts.zBase + (rng() - 0.5) * 2 * opts.zJitter;
      const worldPoint: Vec3 = { x, y, z };
      const camPoint = opts.extrinsics
        ? transformWorldToCamera(worldPoint, opts.extrinsics)
        : worldPoint;
      const pr = project(camPoint, opts.intrinsics);
      const u = pr.u + gaussian(rng) * opts.noiseSigma;
      const v = pr.v + gaussian(rng) * opts.noiseSigma;
      observations.push({ id, point: camPoint, pixel: { u, v } });
      if (opts.extrinsics && world)
        world.push({ id, pointWorld: worldPoint, pixel: { u, v } });
    }
  }
  return {
    observations,
    world,
    meta: {
      seed: opts.seed,
      noiseSigma: opts.noiseSigma,
      unit: opts.unit ?? "mm",
      generatedAt: new Date(0).toISOString(),
    },
  };
}

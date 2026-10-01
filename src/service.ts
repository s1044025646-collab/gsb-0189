import { resolve } from "node:path";
import { calibrate, locateOutliers, type CalibOptions } from "./calibration.js";
import { compareIntrinsics, type ParamDelta } from "./compare.js";
import { Store } from "./db.js";
import {
  backproject,
  projectCameraPoint,
  projectWorldPoint,
  undistortNormalized,
} from "./geometry.js";
import { importObservations, parseExtrinsics, parseIntrinsics, toCsv } from "./io.js";
import { generateSynthetic } from "./synthetic.js";
import type {
  CalibResult,
  Extrinsics,
  Intrinsics,
  Observation,
  Point2D,
  WorldPoint,
} from "./types.js";

export interface ServiceConfig {
  dataDir: string;
}

/** 纯函数入口（不依赖 DB），便于测试。 */
export function runCalibration(
  observations: Observation[],
  options?: CalibOptions,
): CalibResult & { outliers: ReturnType<typeof locateOutliers> } {
  const result = calibrate(observations, options);
  const outliers = Number.isFinite(result.rms)
    ? locateOutliers(result.perPoint)
    : [];
  return { ...result, outliers };
}

export class CalibService {
  readonly store: Store;

  constructor(config: ServiceConfig) {
    const dir = resolve(config.dataDir);
    this.store = new Store(resolve(dir, "calib.db"));
  }

  saveCamera(name: string, intrinsics: Intrinsics, note?: string) {
    return this.store.insertCamera(name, intrinsics, note);
  }

  savePointSet(name: string, observations: Observation[], note?: string) {
    return this.store.insertPointSet(name, observations, note);
  }

  importAndSave(name: string, text: string, note?: string) {
    const obs = importObservations(text);
    return { pointSet: this.store.insertPointSet(name, obs, note), observations: obs };
  }

  project(points: WorldPoint[], extrinsics: Extrinsics, intrinsics: Intrinsics) {
    return points.map((p) => ({
      point: p,
      pixel: projectWorldPoint(p, extrinsics, intrinsics),
      behindCamera: projectWorldPoint(p, extrinsics, intrinsics) === null,
    }));
  }

  projectCamera(points: { x: number; y: number; z: number }[], intrinsics: Intrinsics) {
    return points.map((p) => ({
      point: p,
      pixel: projectCameraPoint(p, intrinsics),
      behindCamera: !(p.z > 0),
    }));
  }

  backproject(pixels: Point2D[], depths: number[], intrinsics: Intrinsics) {
    return pixels.map((px, i) => ({
      pixel: px,
      depth: depths[i]!,
      ...backproject(px, depths[i]!, intrinsics),
    }));
  }

  undistort(pixels: Point2D[], intrinsics: Intrinsics) {
    return pixels.map((px) => {
      const xd = (px.u - intrinsics.cx) / intrinsics.fx;
      const yd = (px.v - intrinsics.cy) / intrinsics.fy;
      return { pixel: px, distorted: { xd, yd }, ...undistortNormalized(xd, yd, intrinsics) };
    });
  }

  calibratePointSet(
    pointsetId: number,
    options?: CalibOptions,
    cameraId: number | null = null,
  ) {
    const row = this.store.getPointSet(pointsetId);
    if (!row) throw new Error(`点集 ${pointsetId} 不存在`);
    const observations = JSON.parse(row.observations_json) as Observation[];
    const result = runCalibration(observations, options);
    const solve = this.store.insertSolve(
      cameraId,
      pointsetId,
      { options: options ?? {}, algorithm: "linear-init + joint Gauss-Newton(LM)" },
      result,
      Number.isFinite(result.rms) ? result.rms : null,
      result.diagnostics.identifiable,
    );
    return { solveId: solve.id, result };
  }

  /** 用历史快照保存的原始观测与配置重新求解，验证可复现性。 */
  reproduce(solveId: number) {
    const solve = this.store.getSolve(solveId);
    if (!solve) throw new Error(`求解结果 ${solveId} 不存在`);
    const pointSet = this.store.getPointSet(solve.pointset_id);
    if (!pointSet) throw new Error("关联点集已丢失");
    const observations = JSON.parse(pointSet.observations_json) as Observation[];
    const config = JSON.parse(solve.config_json) as { options?: CalibOptions };
    const rerun = runCalibration(observations, config.options);
    const stored = JSON.parse(solve.result_json) as CalibResult;
    return {
      storedRms: stored.rms,
      reproducedRms: rerun.rms,
      rmsMatch: Math.abs(stored.rms - rerun.rms) < 1e-12,
      result: rerun,
    };
  }

  compare(estimated: Intrinsics, truth: Intrinsics): ParamDelta[] {
    return compareIntrinsics(estimated, truth);
  }

  generateSynthetic(partial: Parameters<typeof generateSynthetic>[0], extrinsics?: Extrinsics) {
    return generateSynthetic(partial, extrinsics);
  }

  parseIntrinsics(obj: unknown): Intrinsics {
    return parseIntrinsics(obj);
  }

  parseExtrinsics(obj: unknown): Extrinsics {
    return parseExtrinsics(obj);
  }

  residualsCsv(result: CalibResult): string {
    return toCsv(
      result.perPoint.map((p) => ({
        id: p.id,
        u: round(p.u),
        v: round(p.v),
        du: round(p.uResidual),
        dv: round(p.vResidual),
        distance_px: round(p.distance),
      })),
      ["id", "u", "v", "du", "dv", "distance_px"],
    );
  }
}

function round(x: number): number {
  return Math.round(x * 1e9) / 1e9;
}

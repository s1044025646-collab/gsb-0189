/**
 * SQLite 存储：相机版本、点集版本、求解配置、结果快照。
 * 结果快照保存输入点集快照与配置，保证旧结果可按原数据复现。
 * 使用 Node 内置 node:sqlite（DatabaseSync），无第三方原生依赖。
 */
import { DatabaseSync } from "node:sqlite";
import * as fs from "node:fs";
import * as path from "node:path";
import {
  CalibrationConfig,
  CalibrationResult,
  Extrinsics,
  Intrinsics,
  PointObservation,
} from "./types";

export interface CameraRow {
  id: number;
  name: string;
  version: number;
  intrinsics: Intrinsics;
  extrinsics: Extrinsics | null;
  unit: string;
  created_at: string;
}

export interface PointSetRow {
  id: number;
  name: string;
  version: number;
  unit: string;
  observations: PointObservation[];
  created_at: string;
}

export interface ResultRow {
  id: number;
  camera_id: number | null;
  pointset_id: number;
  config: CalibrationConfig;
  result: CalibrationResult;
  /** 结果生成时的点集快照（复现用）。 */
  pointset_snapshot: PointObservation[];
  created_at: string;
  note: string;
}

export class Store {
  db: DatabaseSync;

  constructor(dataDir: string) {
    fs.mkdirSync(dataDir, { recursive: true });
    const file = path.join(dataDir, "calib.db");
    this.db = new DatabaseSync(file);
    this.db.exec("PRAGMA journal_mode = WAL;");
    this.migrate();
  }

  close() {
    this.db.close();
  }

  private migrate() {
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS cameras (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        version INTEGER NOT NULL DEFAULT 1,
        unit TEXT NOT NULL DEFAULT 'mm',
        intrinsics_json TEXT NOT NULL,
        extrinsics_json TEXT,
        created_at TEXT NOT NULL DEFAULT (datetime('now'))
      );
      CREATE TABLE IF NOT EXISTS pointsets (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        version INTEGER NOT NULL DEFAULT 1,
        unit TEXT NOT NULL DEFAULT 'mm',
        observations_json TEXT NOT NULL,
        created_at TEXT NOT NULL DEFAULT (datetime('now'))
      );
      CREATE TABLE IF NOT EXISTS results (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        camera_id INTEGER,
        pointset_id INTEGER NOT NULL,
        note TEXT NOT NULL DEFAULT '',
        config_json TEXT NOT NULL,
        result_json TEXT NOT NULL,
        pointset_snapshot_json TEXT NOT NULL,
        created_at TEXT NOT NULL DEFAULT (datetime('now'))
      );
    `);
  }

  createCamera(
    name: string,
    intrinsics: Intrinsics,
    opts: { unit?: string; extrinsics?: Extrinsics | null } = {}
  ): CameraRow {
    const info = this.db
      .prepare(
        `INSERT INTO cameras (name, version, unit, intrinsics_json, extrinsics_json)
         VALUES (?, 1, ?, ?, ?)`
      )
      .run(
        name,
        opts.unit ?? "mm",
        JSON.stringify(intrinsics),
        opts.extrinsics ? JSON.stringify(opts.extrinsics) : null
      );
    return this.getCamera(Number(info.lastInsertRowid))!;
  }

  getCamera(id: number): CameraRow | null {
    const row = this.db.prepare("SELECT * FROM cameras WHERE id=?").get(id) as any;
    if (!row) return null;
    return {
      id: row.id,
      name: row.name,
      version: row.version,
      unit: row.unit,
      intrinsics: JSON.parse(row.intrinsics_json),
      extrinsics: row.extrinsics_json ? JSON.parse(row.extrinsics_json) : null,
      created_at: row.created_at,
    };
  }

  listCameras(): CameraRow[] {
    const rows = this.db.prepare("SELECT * FROM cameras ORDER BY id").all() as any[];
    return rows.map((row) => ({
      id: row.id,
      name: row.name,
      version: row.version,
      unit: row.unit,
      intrinsics: JSON.parse(row.intrinsics_json),
      extrinsics: row.extrinsics_json ? JSON.parse(row.extrinsics_json) : null,
      created_at: row.created_at,
    }));
  }

  /** 新版本：同名相机再次写入时 version+1，旧版本保留。 */
  createPointSet(
    name: string,
    observations: PointObservation[],
    unit = "mm"
  ): PointSetRow {
    const info = this.db
      .prepare(
        `INSERT INTO pointsets (name, version, unit, observations_json)
         VALUES (?,
           COALESCE((SELECT MAX(version)+1 FROM pointsets WHERE name=?),1),
           ?, ?)`
      )
      .run(name, name, unit, JSON.stringify(observations));
    return this.getPointSet(Number(info.lastInsertRowid))!;
  }

  getPointSet(id: number): PointSetRow | null {
    const row = this.db.prepare("SELECT * FROM pointsets WHERE id=?").get(id) as any;
    if (!row) return null;
    return {
      id: row.id,
      name: row.name,
      version: row.version,
      unit: row.unit,
      observations: JSON.parse(row.observations_json),
      created_at: row.created_at,
    };
  }

  listPointSets(): PointSetRow[] {
    const rows = this.db.prepare("SELECT * FROM pointsets ORDER BY id").all() as any[];
    return rows.map((row) => ({
      id: row.id,
      name: row.name,
      version: row.version,
      unit: row.unit,
      observations: JSON.parse(row.observations_json),
      created_at: row.created_at,
    }));
  }

  saveResult(
    pointsetId: number,
    config: CalibrationConfig,
    result: CalibrationResult,
    snapshot: PointObservation[],
    opts: { cameraId?: number | null; note?: string } = {}
  ): number {
    const info = this.db
      .prepare(
        `INSERT INTO results
          (camera_id, pointset_id, note, config_json, result_json, pointset_snapshot_json)
         VALUES (?, ?, ?, ?, ?, ?)`
      )
      .run(
        opts.cameraId ?? null,
        pointsetId,
        opts.note ?? "",
        JSON.stringify(config),
        JSON.stringify(result),
        JSON.stringify(snapshot)
      );
    return Number(info.lastInsertRowid);
  }

  getResult(id: number): ResultRow | null {
    const row = this.db.prepare("SELECT * FROM results WHERE id=?").get(id) as any;
    if (!row) return null;
    return {
      id: row.id,
      camera_id: row.camera_id,
      pointset_id: row.pointset_id,
      note: row.note,
      config: JSON.parse(row.config_json),
      result: JSON.parse(row.result_json),
      pointset_snapshot: JSON.parse(row.pointset_snapshot_json),
      created_at: row.created_at,
    };
  }

  listResults(): { id: number; pointset_id: number; camera_id: number | null; created_at: string; note: string }[] {
    return (this.db
      .prepare("SELECT id, pointset_id, camera_id, created_at, note FROM results ORDER BY id")
      .all() as any[]).map((r) => ({
      id: r.id,
      pointset_id: r.pointset_id,
      camera_id: r.camera_id,
      created_at: r.created_at,
      note: r.note,
    }));
  }
}

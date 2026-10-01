import { mkdirSync } from "node:fs";
import { dirname } from "node:path";
import { DatabaseSync } from "node:sqlite";

export interface CameraRow {
  id: number;
  name: string;
  version: number;
  intrinsics_json: string;
  created_at: string;
  note: string | null;
}

export interface PointSetRow {
  id: number;
  name: string;
  version: number;
  observations_json: string;
  created_at: string;
  note: string | null;
}

export interface SolveRow {
  id: number;
  camera_id: number | null;
  pointset_id: number;
  config_json: string;
  result_json: string;
  created_at: string;
  rms: number | null;
  identifiable: number;
}

/**
 * SQLite 存储层。
 * 相机与点集均带 version；每次保存为新版本，旧版本与求解结果快照永不覆盖，
 * 因此历史结果可用其保存的 observations + config 原样复现。
 */
export class Store {
  readonly db: DatabaseSync;

  constructor(dbPath: string) {
    mkdirSync(dirname(dbPath), { recursive: true });
    this.db = new DatabaseSync(dbPath);
    this.db.exec("PRAGMA journal_mode = WAL;");
    this.db.exec("PRAGMA foreign_keys = ON;");
    this.migrate();
  }

  private migrate(): void {
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS cameras (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        version INTEGER NOT NULL,
        intrinsics_json TEXT NOT NULL,
        note TEXT,
        created_at TEXT NOT NULL DEFAULT (datetime('now'))
      );
      CREATE TABLE IF NOT EXISTS pointsets (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        version INTEGER NOT NULL,
        observations_json TEXT NOT NULL,
        note TEXT,
        created_at TEXT NOT NULL DEFAULT (datetime('now'))
      );
      CREATE TABLE IF NOT EXISTS solves (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        camera_id INTEGER REFERENCES cameras(id),
        pointset_id INTEGER NOT NULL REFERENCES pointsets(id),
        config_json TEXT NOT NULL,
        result_json TEXT NOT NULL,
        rms REAL,
        identifiable INTEGER NOT NULL DEFAULT 0,
        created_at TEXT NOT NULL DEFAULT (datetime('now'))
      );
    `);
  }

  insertCamera(
    name: string,
    intrinsics: unknown,
    note?: string,
  ): CameraRow {
    const version =
      (this.db
        .prepare("SELECT COALESCE(MAX(version),0) AS v FROM cameras WHERE name=?")
        .get(name) as { v: number }).v + 1;
    const info = this.db
      .prepare(
        "INSERT INTO cameras(name,version,intrinsics_json,note) VALUES(?,?,?,?)",
      )
      .run(name, version, JSON.stringify(intrinsics), note ?? null);
    return this.getCamera(Number(info.lastInsertRowid))!;
  }

  getCamera(id: number): CameraRow | undefined {
    return this.db.prepare("SELECT * FROM cameras WHERE id=?").get(id) as
      | CameraRow
      | undefined;
  }

  listCameras(name?: string): CameraRow[] {
    return name
      ? (this.db
          .prepare("SELECT * FROM cameras WHERE name=? ORDER BY id")
          .all(name) as unknown as CameraRow[])
      : (this.db.prepare("SELECT * FROM cameras ORDER BY id").all() as unknown as CameraRow[]);
  }

  insertPointSet(
    name: string,
    observations: unknown,
    note?: string,
  ): PointSetRow {
    const version =
      (this.db
        .prepare("SELECT COALESCE(MAX(version),0) AS v FROM pointsets WHERE name=?")
        .get(name) as { v: number }).v + 1;
    const info = this.db
      .prepare(
        "INSERT INTO pointsets(name,version,observations_json,note) VALUES(?,?,?,?)",
      )
      .run(name, version, JSON.stringify(observations), note ?? null);
    return this.getPointSet(Number(info.lastInsertRowid))!;
  }

  getPointSet(id: number): PointSetRow | undefined {
    return this.db.prepare("SELECT * FROM pointsets WHERE id=?").get(id) as
      | PointSetRow
      | undefined;
  }

  listPointSets(name?: string): PointSetRow[] {
    return name
      ? (this.db
          .prepare("SELECT * FROM pointsets WHERE name=? ORDER BY id")
          .all(name) as unknown as PointSetRow[])
      : (this.db
          .prepare("SELECT * FROM pointsets ORDER BY id")
          .all() as unknown as PointSetRow[]);
  }

  insertSolve(
    cameraId: number | null,
    pointsetId: number,
    config: unknown,
    result: unknown,
    rms: number | null,
    identifiable: boolean,
  ): SolveRow {
    const info = this.db
      .prepare(
        `INSERT INTO solves(camera_id,pointset_id,config_json,result_json,rms,identifiable)
         VALUES(?,?,?,?,?,?)`,
      )
      .run(
        cameraId,
        pointsetId,
        JSON.stringify(config),
        JSON.stringify(result),
        rms,
        identifiable ? 1 : 0,
      );
    return this.getSolve(Number(info.lastInsertRowid))!;
  }

  getSolve(id: number): SolveRow | undefined {
    return this.db.prepare("SELECT * FROM solves WHERE id=?").get(id) as
      | SolveRow
      | undefined;
  }

  listSolves(): SolveRow[] {
    return this.db.prepare("SELECT * FROM solves ORDER BY id").all() as unknown as SolveRow[];
  }

  close(): void {
    this.db.close();
  }
}


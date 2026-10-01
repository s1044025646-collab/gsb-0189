import { readFileSync, writeFileSync } from "node:fs";
import type { CamPoint, Extrinsics, Intrinsics, Observation, Point2D } from "./types.js";

/** CSV 行解析（支持引号与转义双引号）。 */
function parseCsvLine(line: string): string[] {
  const out: string[] = [];
  let cur = "";
  let inQ = false;
  for (let i = 0; i < line.length; i++) {
    const ch = line[i]!;
    if (inQ) {
      if (ch === '"') {
        if (line[i + 1] === '"') {
          cur += '"';
          i++;
        } else {
          inQ = false;
        }
      } else cur += ch;
    } else if (ch === '"') {
      inQ = true;
    } else if (ch === ",") {
      out.push(cur);
      cur = "";
    } else cur += ch;
  }
  out.push(cur);
  return out.map((s) => s.trim());
}

/**
 * 导入观测。
 * JSON: [{id,x,y,z,u,v}, ...] 或 {observations:[...]}
 * CSV 表头: id,x,y,z,u,v （x,y,z 相机坐标，米；u,v 像素）
 */
export function importObservations(text: string, format?: "json" | "csv"): Observation[] {
  const fmt = format ?? detect(text);
  if (fmt === "json") {
    const data = JSON.parse(text);
    const rows: any[] = Array.isArray(data) ? data : data.observations;
    if (!Array.isArray(rows)) throw new Error("JSON 需为数组或含 observations 字段");
    return rows.map(rowToObs);
  }
  const lines = text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);
  if (lines.length < 2) throw new Error("CSV 至少需要表头与一行数据");
  const header = parseCsvLine(lines[0]!).map((h) => h.toLowerCase());
  const idx = (name: string) => header.indexOf(name);
  const need = ["x", "y", "z", "u", "v"];
  for (const n of need) {
    if (idx(n) < 0) throw new Error(`CSV 缺少列: ${n}`);
  }
  const idCol = idx("id");
  return lines.slice(1).map((line, i) => {
    const c = parseCsvLine(line);
    const num = (name: string) => {
      const val = Number(c[idx(name)]);
      if (!Number.isFinite(val)) throw new Error(`第 ${i + 2} 行 ${name} 不是数字`);
      return val;
    };
    return {
      id: idCol >= 0 && c[idCol] ? c[idCol]! : `row${i + 1}`,
      point: { x: num("x"), y: num("y"), z: num("z") } as CamPoint,
      pixel: { u: num("u"), v: num("v") } as Point2D,
    };
  });
}

function rowToObs(row: any): Observation {
  const num = (v: any, name: string) => {
    const n = typeof v === "number" ? v : Number(v);
    if (!Number.isFinite(n)) throw new Error(`观测 ${name} 不是数字`);
    return n;
  };
  return {
    id: String(row.id ?? row.name ?? ""),
    point: {
      x: num(row.x ?? row.X, "x"),
      y: num(row.y ?? row.Y, "y"),
      z: num(row.z ?? row.Z, "z"),
    },
    pixel: { u: num(row.u ?? row.U, "u"), v: num(row.v ?? row.V, "v") },
  };
}

function detect(text: string): "json" | "csv" {
  return text.trimStart().startsWith("{") || text.trimStart().startsWith("[")
    ? "json"
    : "csv";
}

/** 导出观测/残差为 CSV。 */
export function toCsv(rows: Record<string, number | string>[], columns?: string[]): string {
  if (rows.length === 0) return columns ? columns.join(",") + "\n" : "";
  const cols = columns ?? Object.keys(rows[0]!);
  const esc = (v: number | string) => {
    const s = String(v);
    return /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
  };
  const head = cols.join(",");
  const body = rows.map((r) => cols.map((c) => esc(r[c] ?? "")).join(",")).join("\n");
  return head + "\n" + body + "\n";
}

export function readJsonFile<T>(path: string): T {
  return JSON.parse(readFileSync(path, "utf-8"));
}

export function readObservationsFile(path: string): Observation[] {
  return importObservations(readFileSync(path, "utf-8"));
}

export function writeText(path: string, text: string): void {
  writeFileSync(path, text, "utf-8");
}

export function parseIntrinsics(obj: any): Intrinsics {
  const k: any = obj.intrinsics ?? obj;
  const out: Intrinsics = {
    fx: Number(k.fx),
    fy: Number(k.fy),
    cx: Number(k.cx),
    cy: Number(k.cy),
    k1: Number(k.k1 ?? 0),
    k2: Number(k.k2 ?? 0),
  };
  for (const [key, val] of Object.entries(out)) {
    if (!Number.isFinite(val as number)) throw new Error(`内参 ${key} 不是有限数字`);
  }
  return out;
}

export function parseExtrinsics(obj: any): Extrinsics {
  const e = obj.extrinsics ?? obj;
  let R: number[];
  if (Array.isArray(e.R)) {
    R = e.R.flat(Infinity).map(Number);
  } else if (Array.isArray(e.rvec)) {
    R = rvecToR(e.rvec.map(Number));
  } else {
    R = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  }
  if (R.length !== 9 || R.some((v) => !Number.isFinite(v))) {
    throw new Error("旋转矩阵 R 必须为 9 个有限数（3x3，行主序）");
  }
  const t: [number, number, number] = [
    Number(e.t?.[0] ?? 0),
    Number(e.t?.[1] ?? 0),
    Number(e.t?.[2] ?? 0),
  ];
  if (t.some((v) => !Number.isFinite(v))) throw new Error("平移 t 必须为有限数字");
  return { R, t };
}

function rvecToR(r: number[]): number[] {
  const theta = Math.hypot(r[0]!, r[1]!, r[2]!);
  if (theta === 0) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const [x, y, z] = [r[0]! / theta, r[1]! / theta, r[2]! / theta];
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  const C = 1 - c;
  return [
    c + x * x * C, x * y * C - z * s, x * z * C + y * s,
    y * x * C + z * s, c + y * y * C, y * z * C - x * s,
    z * x * C - y * s, z * y * C + x * s, c + z * z * C,
  ];
}

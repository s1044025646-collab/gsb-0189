/** JSON / CSV 导入导出。 */
import { PointObservation, ReprojectionError, Vec3 } from "./types";

export interface ObservationFile {
  unit?: string;
  points?: PointObservation[];
  observations?: PointObservation[];
}

export function parseObservationsJson(text: string): {
  observations: PointObservation[];
  unit?: string;
} {
  const obj = JSON.parse(text) as ObservationFile;
  const points = obj.points ?? obj.observations ?? [];
  return { observations: points.map(normalizeObs), unit: obj.unit };
}

function toVec3(o: any): Vec3 {
  return {
    x: Number(o.x ?? o.X),
    y: Number(o.y ?? o.Y),
    z: Number(o.z ?? o.Z),
  };
}

function normalizeObs(o: any): PointObservation {
  const pixel = o.pixel ?? { u: o.u, v: o.v };
  return {
    id: String(o.id ?? ""),
    point: toVec3(o.point ?? o),
    pixel: { u: Number(pixel.u), v: Number(pixel.v) },
  };
}

/** CSV 列: id,x,y,z,u,v */
export function parseObservationsCsv(text: string): PointObservation[] {
  const lines = text
    .split(/\r?\n/)
    .map((l) => l.trim())
    .filter(Boolean);
  if (!lines.length) return [];
  const header = lines[0].split(",").map((h) => h.trim().toLowerCase());
  const hasHeader = header.includes("x") && header.includes("u");
  const rows = hasHeader ? lines.slice(1) : lines;
  const idx = (name: string) =>
    hasHeader ? header.indexOf(name) : ["id", "x", "y", "z", "u", "v"].indexOf(name);
  return rows.map((line, i) => {
    const c = line.split(",").map((f) => f.trim());
    return {
      id: c[idx("id")] || `p${i}`,
      point: {
        x: Number(c[idx("x")]),
        y: Number(c[idx("y")]),
        z: Number(c[idx("z")]),
      },
      pixel: { u: Number(c[idx("u")]), v: Number(c[idx("v")]) },
    };
  });
}

export function observationsToCsv(obs: PointObservation[]): string {
  const head = "id,x,y,z,u,v";
  const rows = obs.map(
    (o) =>
      `${o.id ?? ""},${o.point.x},${o.point.y},${o.point.z},${o.pixel.u},${o.pixel.v}`
  );
  return [head, ...rows].join("\n");
}

export function errorsToCsv(errors: ReprojectionError[]): string {
  const head =
    "id,u,v,u_reproj,v_reproj,residual_u,residual_v,distance_px,depth,behind_camera";
  const rows = errors.map(
    (e) =>
      `${e.id},${e.u},${e.v},${e.uReproj},${e.vReproj},${e.residualU},${e.residualV},${e.distance},${e.depth},${e.behindCamera}`
  );
  return [head, ...rows].join("\n");
}

export function parseFile(text: string, filename: string) {
  if (filename.toLowerCase().endsWith(".csv"))
    return { observations: parseObservationsCsv(text) };
  return parseObservationsJson(text);
}

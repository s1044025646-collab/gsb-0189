import { createServer, type IncomingMessage, type ServerResponse } from "node:http";
import { CalibService, runCalibration } from "./service.js";

export interface ServerOptions {
  port: number;
  dataDir: string;
}

function send(res: ServerResponse, code: number, body: unknown): void {
  const text =
    typeof body === "string" ? body : JSON.stringify(body, null, 2);
  const isObj = typeof body !== "string";
  res.writeHead(code, {
    "content-type": isObj ? "application/json; charset=utf-8" : "text/plain; charset=utf-8",
  });
  res.end(text);
}

async function readBody(req: IncomingMessage): Promise<any> {
  const chunks: Buffer[] = [];
  for await (const c of req) chunks.push(c as Buffer);
  const raw = Buffer.concat(chunks).toString("utf-8");
  if (!raw) return {};
  return JSON.parse(raw);
}

export function createApp(service: CalibService) {
  return createServer(async (req, res) => {
    const url = new URL(req.url ?? "/", "http://localhost");
    const path = url.pathname;
    const method = req.method ?? "GET";
    try {
      if (method === "GET" && path === "/api/health") {
        return send(res, 200, { ok: true });
      }

      if (method === "GET" && path === "/api/cameras") {
        return send(res, 200, service.store.listCameras(url.searchParams.get("name") ?? undefined));
      }
      if (method === "GET" && path === "/api/pointsets") {
        return send(res, 200, service.store.listPointSets(url.searchParams.get("name") ?? undefined));
      }
      if (method === "GET" && path === "/api/solves") {
        return send(res, 200, service.store.listSolves().map(lightSolve));
      }

      const m = path.match(/^\/api\/solve\/(\d+)$/);
      if (method === "GET" && m) {
        const row = service.store.getSolve(Number(m[1]));
        if (!row) return send(res, 404, { error: "solve 不存在" });
        return send(res, 200, { ...lightSolve(row), result: JSON.parse(row.result_json) });
      }
      const mr = path.match(/^\/api\/solve\/(\d+)\/reproduce$/);
      if (method === "POST" && mr) {
        return send(res, 200, service.reproduce(Number(mr[1])));
      }
      const mc = path.match(/^\/api\/solve\/(\d+)\/residuals\.csv$/);
      if (method === "GET" && mc) {
        const row = service.store.getSolve(Number(mc[1]));
        if (!row) return send(res, 404, { error: "solve 不存在" });
        res.writeHead(200, { "content-type": "text/csv; charset=utf-8" });
        return res.end(service.residualsCsv(JSON.parse(row.result_json)));
      }

      if (method === "POST" && path === "/api/synthetic/generate") {
        const body = await readBody(req);
        const bundle = service.generateSynthetic(body.config ?? {}, body.extrinsics ? service.parseExtrinsics(body.extrinsics) : undefined);
        let pointSet = null;
        if (body.saveAs) {
          pointSet = service.savePointSet(String(body.saveAs), bundle.observations, "synthetic");
        }
        return send(res, 200, { ...bundle, savedPointSet: pointSet });
      }

      if (method === "POST" && path === "/api/observations/import") {
        const body = await readBody(req);
        const text =
          typeof body.text === "string"
            ? body.text
            : JSON.stringify(body.observations ?? body);
        const { pointSet, observations } = service.importAndSave(
          String(body.name ?? "imported"),
          text,
          body.note,
        );
        return send(res, 200, { pointSet, count: observations.length });
      }

      if (method === "POST" && path === "/api/project") {
        const b = await readBody(req);
        return send(res, 200, {
          projections: service.project(
            b.points,
            service.parseExtrinsics(b.extrinsics ?? {}),
            service.parseIntrinsics(b.intrinsics),
          ),
        });
      }
      if (method === "POST" && path === "/api/project/camera") {
        const b = await readBody(req);
        return send(res, 200, {
          projections: service.projectCamera(b.points, service.parseIntrinsics(b.intrinsics)),
        });
      }
      if (method === "POST" && path === "/api/backproject") {
        const b = await readBody(req);
        const pixels = b.pixels as { u: number; v: number }[];
        const depths = (b.depths as number[]) ?? pixels.map(() => b.depth ?? 1);
        return send(res, 200, {
          results: service.backproject(pixels, depths, service.parseIntrinsics(b.intrinsics)),
        });
      }
      if (method === "POST" && path === "/api/calibrate/direct") {
        const b = await readBody(req);
        return send(res, 200, { result: runCalibration(b.observations, b.options) });
      }
      if (method === "POST" && path === "/api/calibrate") {
        const b = await readBody(req);
        const out = service.calibratePointSet(
          Number(b.pointsetId),
          b.options,
          b.cameraId ?? null,
        );
        return send(res, 200, { solveId: out.solveId, result: out.result });
      }
      if (method === "POST" && path === "/api/compare") {
        const b = await readBody(req);
        return send(res, 200, {
          deltas: service.compare(
            service.parseIntrinsics(b.estimated),
            service.parseIntrinsics(b.truth),
          ),
        });
      }

      if (method === "GET" && path === "/") {
        return send(res, 200, {
          name: "pinhole-calib",
          endpoints: [
            "GET /api/health",
            "POST /api/synthetic/generate",
            "POST /api/observations/import",
            "GET /api/cameras|pointsets|solves",
            "POST /api/project",
            "POST /api/project/camera",
            "POST /api/backproject",
            "POST /api/calibrate",
            "POST /api/calibrate/direct",
            "GET /api/solve/:id",
            "POST /api/solve/:id/reproduce",
            "GET /api/solve/:id/residuals.csv",
            "POST /api/compare",
          ],
        });
      }
      return send(res, 404, { error: "未找到路由", path });
    } catch (err) {
      return send(res, 400, { error: (err as Error).message });
    }

  });
}

function lightSolve(row: {
  id: number;
  camera_id: number | null;
  pointset_id: number;
  rms: number | null;
  identifiable: number;
  created_at: string;
  config_json: string;
}) {
  return {
    id: row.id,
    cameraId: row.camera_id,
    pointsetId: row.pointset_id,
    rms: row.rms,
    identifiable: !!row.identifiable,
    createdAt: row.created_at,
    config: JSON.parse(row.config_json),
  };
}

const isMain = process.argv[1] && process.argv[1].endsWith("server.js");
if (isMain) {
  const port = Number(process.env.PORT ?? 8080);
  const dataDir = process.env.DATA_DIR ?? "./data";
  const service = new CalibService({ dataDir });
  createApp(service).listen(port, () => {
    console.log(`针孔标定服务已启动: http://localhost:${port}`);
    console.log(`数据目录: ${dataDir}`);
  });
}

/**
 * 纯 JSON HTTP API（无第三方框架，使用 node:http）。
 * 端口可用环境变量 PORT 配置，数据目录用 DATA_DIR 配置（均在项目内 data/）。
 */
import * as http from "node:http";
import { Store } from "./store";
import { calibrate } from "./calibration";
import { project, transformWorldToCamera, unproject } from "./geometry";
import {
  compareIntrinsics,
  compareSubsets,
  locateOutliers,
} from "./analyze";
import { generate } from "./synthetic";
import { DEFAULT_CONFIG, Intrinsics, PointObservation } from "./types";
import * as path from "node:path";

const PORT = Number(process.env.PORT ?? 8787);
const DATA_DIR = process.env.DATA_DIR ?? path.join(process.cwd(), "data");

const store = new Store(DATA_DIR);

function json(res: http.ServerResponse, code: number, body: unknown) {
  res.writeHead(code, { "Content-Type": "application/json; charset=utf-8" });
  res.end(JSON.stringify(body, null, 2));
}

async function readBody(req: http.IncomingMessage): Promise<any> {
  const chunks: Buffer[] = [];
  for await (const c of req) chunks.push(c as Buffer);
  const text = Buffer.concat(chunks).toString("utf-8");
  return text ? JSON.parse(text) : {};
}

const server = http.createServer(async (req, res) => {
  const url = new URL(req.url ?? "/", `http://localhost:${PORT}`);
  const p = url.pathname;
  try {
    // 健康检查
    if (req.method === "GET" && p === "/api/health")
      return json(res, 200, { ok: true, dataDir: DATA_DIR });

    // 相机版本
    if (req.method === "GET" && p === "/api/cameras")
      return json(res, 200, store.listCameras());
    if (req.method === "POST" && p === "/api/cameras") {
      const b = await readBody(req);
      const row = store.createCamera(b.name, b.intrinsics as Intrinsics, {
        unit: b.unit,
        extrinsics: b.extrinsics ?? null,
      });
      return json(res, 201, row);
    }
    const camMatch = p.match(/^\/api\/cameras\/(\d+)$/);
    if (req.method === "GET" && camMatch)
      return json(res, 200, store.getCamera(Number(camMatch[1])));

    // 点集版本
    if (req.method === "GET" && p === "/api/pointsets")
      return json(res, 200, store.listPointSets());
    if (req.method === "POST" && p === "/api/pointsets") {
      const b = await readBody(req);
      const row = store.createPointSet(
        b.name,
        b.observations as PointObservation[],
        b.unit ?? "mm"
      );
      return json(res, 201, row);
    }
    const psMatch = p.match(/^\/api\/pointsets\/(\d+)$/);
    if (req.method === "GET" && psMatch)
      return json(res, 200, store.getPointSet(Number(psMatch[1])));

    // 合成数据生成
    if (req.method === "POST" && p === "/api/synthetic/generate") {
      const b = await readBody(req);
      return json(res, 200, generate(b));
    }

    // 投影（相机坐标点）
    if (req.method === "POST" && p === "/api/project") {
      const b = await readBody(req);
      return json(res, 200, project(b.point, b.intrinsics));
    }
    // 给定世界坐标 + 外参的投影接口
    if (req.method === "POST" && p === "/api/project-world") {
      const b = await readBody(req);
      const cam = transformWorldToCamera(b.pointWorld, b.extrinsics);
      return json(res, 200, { cameraPoint: cam, projection: project(cam, b.intrinsics) });
    }
    // 给定深度反投影
    if (req.method === "POST" && p === "/api/unproject") {
      const b = await readBody(req);
      return json(
        res,
        200,
        unproject(b.u, b.v, b.depth, b.intrinsics, {
          maxIter: b.config?.undistortMaxIter ?? DEFAULT_CONFIG.undistortMaxIter,
          tol: b.config?.undistortTol ?? DEFAULT_CONFIG.undistortTol,
          maxRadius: b.config?.undistortMaxRadius ?? DEFAULT_CONFIG.undistortMaxRadius,
        })
      );
    }

    // 标定（不落库）
    if (req.method === "POST" && p === "/api/calibrate") {
      const b = await readBody(req);
      return json(res, 200, calibrate(b.observations, b.config ?? {}));
    }
    // 标定并保存快照（可复现）
    if (req.method === "POST" && p === "/api/calibrate-save") {
      const b = await readBody(req);
      const config = { ...DEFAULT_CONFIG, ...(b.config ?? {}) };
      const result = calibrate(b.observations, config);
      const ps = store.createPointSet(
        b.name ?? "api_set",
        b.observations,
        b.unit ?? "mm"
      );
      const id = store.saveResult(ps.id, config, result, b.observations, {
        note: b.note ?? "",
      });
      return json(res, 201, { resultId: id, pointsetId: ps.id, result });
    }
    // 结果列表 / 读取（含原始快照）/ 按原数据复现
    if (req.method === "GET" && p === "/api/results")
      return json(res, 200, store.listResults());
    const resMatch = p.match(/^\/api\/results\/(\d+)$/);
    if (req.method === "GET" && resMatch)
      return json(res, 200, store.getResult(Number(resMatch[1])));
    if (req.method === "POST" && p.match(/^\/api\/results\/\d+\/reproduce$/)) {
      const id = Number(p.split("/")[3]);
      const saved = store.getResult(id);
      if (!saved) return json(res, 404, { error: "结果不存在" });
      const rerun = calibrate(saved.pointset_snapshot, saved.config);
      return json(res, 200, {
        saved: { rmse: saved.result.rmse, intrinsics: saved.result.intrinsics, status: saved.result.status },
        reproduced: { rmse: rerun.rmse, intrinsics: rerun.intrinsics, status: rerun.status },
      });
    }

    // 参数比较 / 子集比较 / 异常定位
    if (req.method === "POST" && p === "/api/compare") {
      const b = await readBody(req);
      return json(res, 200, compareIntrinsics(b.a, b.b));
    }
    if (req.method === "POST" && p === "/api/compare-subsets") {
      const b = await readBody(req);
      return json(res, 200, compareSubsets(b.subsetA, b.subsetB, b.config ?? {}));
    }
    if (req.method === "POST" && p === "/api/outliers") {
      const b = await readBody(req);
      return json(res, 200, locateOutliers(b.observations, { influenceRatio: b.influenceRatio, config: b.config }));
    }

    return json(res, 404, { error: `未找到路由 ${req.method} ${p}` });
  } catch (err) {
    return json(res, 400, { error: (err as Error).message });
  }
});

server.listen(PORT, () => {
  console.log(`针孔标定后端已启动: http://localhost:${PORT}  数据目录: ${DATA_DIR}`);
});

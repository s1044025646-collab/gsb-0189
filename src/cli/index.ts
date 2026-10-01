#!/usr/bin/env node
import { readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import { CalibService, runCalibration } from "../service.js";
import {
  backproject,
  projectCameraPoint,
  projectWorldPoint,
} from "../geometry.js";
import { importObservations, parseExtrinsics, parseIntrinsics } from "../io.js";
import { generateSynthetic } from "../synthetic.js";
import { compareIntrinsics } from "../compare.js";

function help(): string {
  return [
    "针孔相机标定 CLI",
    "",
    "用法: pinhole <命令> [参数]   (全局: --data-dir <dir>, 默认 ./data)",
    "",
    "generate [--seed 42] [--points 60] [--noise 0.4] [--k1 -0.28] [--k2 0.08]",
    "         [--fx 900] [--fy 900] [--cx 640] [--cy 400] [--save-as <名称>] [--out <文件.json>]",
    "import --file <json|csv> --name <点集名>",
    "project --intrinsics <json> [--extrinsics <json>] --points <json>",
    "backproject --intrinsics <json> --pixels <json [{u,v,depth}]>",
    "calibrate --pointset-id <id> [--max-iter 100] [--tol 1e-10]",
    "calibrate-direct --file <json|csv> [--max-iter 100]",
    "list [cameras|pointsets|solves]",
    "get-solve --id <id> | reproduce --id <id>",
    "export-residuals --id <id> --out <residuals.csv>",
    "compare --estimated <json> --truth <json>",
  ].join("\n");
}

function parseArgs(argv: string[]): { cmd: string; opts: Record<string, string> } {
  const cmd = argv[0] ?? "help";
  const opts: Record<string, string> = {};
  for (let i = 1; i < argv.length; i++) {
    const a = argv[i]!;
    if (a.startsWith("--")) {
      const key = a.slice(2);
      const next = argv[i + 1];
      if (next === undefined || next.startsWith("--")) {
        opts[key] = "true";
      } else {
        opts[key] = next;
        i++;
      }
    }
  }
  return { cmd, opts };
}

function num(opts: Record<string, string>, key: string, def: number): number {
  return opts[key] !== undefined ? Number(opts[key]) : def;
}

function readJson(path: string): any {
  return JSON.parse(readFileSync(resolve(path), "utf-8"));
}

function summary(r: any) {
  return {
    intrinsics: r.intrinsics,
    linearIntrinsics: r.linearIntrinsics,
    rms: r.rms,
    converged: r.converged,
    convergenceReason: r.convergenceReason,
    iterations: r.iterations,
    undistortFailedCount: r.undistortFailedCount,
    diagnostics: r.diagnostics,
    outliers: r.outliers,
  };
}

async function main(): Promise<void> {
  const { cmd, opts } = parseArgs(process.argv.slice(2));
  const dataDir = opts["data-dir"] ?? process.env.DATA_DIR ?? "./data";
  const service = new CalibService({ dataDir });
  const print = (x: unknown) => console.log(JSON.stringify(x, null, 2));

  switch (cmd) {
    case "help":
    case "--help":
    case "-h":
      console.log(help());
      break;

    case "generate": {
      const bundle = generateSynthetic({
        seed: num(opts, "seed", 42),
        pointCount: num(opts, "points", 60),
        noiseSigmaPx: num(opts, "noise", 0.4),
        intrinsics: {
          fx: num(opts, "fx", 900),
          fy: num(opts, "fy", 900),
          cx: num(opts, "cx", 640),
          cy: num(opts, "cy", 400),
          k1: num(opts, "k1", -0.28),
          k2: num(opts, "k2", 0.08),
        },
      });
      if (opts["save-as"]) {
        const ps = service.savePointSet(opts["save-as"], bundle.observations, "synthetic");
        console.log(`已保存点集 id=${ps.id} version=${ps.version}, ${bundle.observations.length} 点`);
      }
      if (opts.out) {
        writeFileSync(resolve(opts.out), JSON.stringify(bundle, null, 2), "utf-8");
        console.log(`已写出 ${opts.out}`);
      }
      if (!opts["save-as"] && !opts.out) print(bundle);
      break;
    }

    case "import": {
      const text = readFileSync(resolve(opts.file!), "utf-8");
      const { pointSet, observations } = service.importAndSave(opts.name!, text, opts.note);
      console.log(`已导入点集 id=${pointSet.id} version=${pointSet.version}, ${observations.length} 点`);
      break;
    }

    case "project": {
      const k = parseIntrinsics(readJson(opts.intrinsics!));
      const points = readJson(opts.points!);
      if (opts.extrinsics) {
        const ext = parseExtrinsics(readJson(opts.extrinsics));
        print(points.map((p: any) => ({ point: p, pixel: projectWorldPoint(p, ext, k) })));
      } else {
        print(points.map((p: any) => ({ point: p, pixel: projectCameraPoint(p, k) })));
      }
      break;
    }

    case "backproject": {
      const k = parseIntrinsics(readJson(opts.intrinsics!));
      const pixels = readJson(opts.pixels!);
      print(
        pixels.map((p: any) => ({
          pixel: { u: p.u, v: p.v },
          depth: p.depth,
          ...backproject({ u: p.u, v: p.v }, Number(p.depth ?? 1), k),
        })),
      );
      break;
    }

    case "calibrate": {
      const out = service.calibratePointSet(
        Number(opts["pointset-id"]),
        { maxIterations: num(opts, "max-iter", 100), tol: num(opts, "tol", 1e-10) },
        null,
      );
      console.log(`solveId=${out.solveId}  RMS=${out.result.rms}`);
      print(summary(out.result));
      break;
    }

    case "calibrate-direct": {
      const obs = importObservations(readFileSync(resolve(opts.file!), "utf-8"));
      print({ ...runCalibration(obs, { maxIterations: num(opts, "max-iter", 100) }) });
      break;
    }

    case "list": {
      const kind = process.argv.find((a) => ["cameras", "pointsets", "solves"].includes(a)) ?? "solves";
      if (kind === "cameras") print(service.store.listCameras());
      else if (kind === "pointsets") print(service.store.listPointSets());
      else
        print(
          service
            .store.listSolves()
            .map((s) => ({ id: s.id, pointset: s.pointset_id, rms: s.rms, identifiable: !!s.identifiable })),
        );
      break;
    }

    case "get-solve": {
      const row = service.store.getSolve(Number(opts.id));
      if (!row) throw new Error("不存在");
      print(summary(JSON.parse(row.result_json)));
      break;
    }

    case "reproduce": {
      print(service.reproduce(Number(opts.id)));
      break;
    }

    case "export-residuals": {
      const row = service.store.getSolve(Number(opts.id));
      if (!row) throw new Error("不存在");
      writeFileSync(resolve(opts.out!), service.residualsCsv(JSON.parse(row.result_json)), "utf-8");
      console.log(`已导出残差到 ${opts.out}`);
      break;
    }

    case "compare": {
      print(
        compareIntrinsics(
          parseIntrinsics(readJson(opts.estimated!)),
          parseIntrinsics(readJson(opts.truth!)),
        ),
      );
      break;
    }

    default:
      console.log(`未知命令: ${cmd}\n`);
      console.log(help());
      process.exitCode = 1;
  }
  service.store.close();
}

main().catch((err) => {
  console.error("错误:", err.message);
  process.exit(1);
});

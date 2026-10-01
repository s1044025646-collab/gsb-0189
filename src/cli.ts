#!/usr/bin/env node
/**
 * CLI: 导入、投影、标定、反投影、参数比较、导出。
 * 用法见 README，例如: npm run cli -- generate --seed 42 --out obs.json
 */
import * as fs from "node:fs";
import * as path from "node:path";
import { Store } from "./store";
import { calibrate } from "./calibration";
import { project, transformWorldToCamera, unproject } from "./geometry";
import {
  compareIntrinsics,
  compareSubsets,
  locateOutliers,
} from "./analyze";
import { generate } from "./synthetic";
import {
  errorsToCsv,
  observationsToCsv,
  parseFile,
  parseObservationsJson,
} from "./io";
import { DEFAULT_CONFIG, Intrinsics } from "./types";

const args = process.argv.slice(2);
const command = args[0];

function opt(name: string, def?: string): string | undefined {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : def;
}
function flag(name: string): boolean {
  return args.includes(`--${name}`);
}
function readJsonOrCsv(file: string) {
  return parseFile(fs.readFileSync(file, "utf-8"), path.basename(file));
}
function loadIntrinsics(file: string): Intrinsics {
  const obj = JSON.parse(fs.readFileSync(file, "utf-8"));
  return (obj.intrinsics ?? obj) as Intrinsics;
}
function out(text: string) {
  const file = opt("out");
  if (file) {
    fs.writeFileSync(file, text);
    console.log(`已写入 ${file}`);
  } else console.log(text);
}

const DATA_DIR = process.env.DATA_DIR ?? path.join(process.cwd(), "data");

async function main() {
  switch (command) {
    case "generate": {
      const intrFile = opt("intrinsics");
      const intr: Intrinsics = intrFile
        ? loadIntrinsics(intrFile)
        : { fx: 800, fy: 800, cx: 320, cy: 240, k1: -0.28, k2: 0.12 };
      const set = generate({
        seed: Number(opt("seed", "42")),
        intrinsics: intr,
        cols: Number(opt("cols", "8")),
        rows: Number(opt("rows", "6")),
        width: Number(opt("width", "400")),
        height: Number(opt("height", "300")),
        zBase: Number(opt("zbase", "1000")),
        zJitter: Number(opt("zjitter", "50")),
        noiseSigma: Number(opt("noise", "0.5")),
      });
      const body = JSON.stringify({ unit: "mm", observations: set.observations }, null, 2);
      out(flag("csv") ? observationsToCsv(set.observations) : body);
      break;
    }
    case "import": {
      const file = opt("file")!;
      const name = opt("name") ?? path.basename(file);
      const parsed = readJsonOrCsv(file);
      const store = new Store(DATA_DIR);
      const row = store.createPointSet(name, parsed.observations, parsed.unit ?? "mm");
      console.log(`已导入点集 id=${row.id} name=${row.name} version=${row.version} 点数=${row.observations.length}`);
      store.close();
      break;
    }
    case "calibrate": {
      const file = opt("file")!;
      const parsed = readJsonOrCsv(file);
      const result = calibrate(parsed.observations, { estimateK: !flag("no-k") });
      const summary = {
        status: result.status,
        converged: result.converged,
        iterations: result.iterations,
        convergenceReason: result.convergenceReason,
        rmse: result.rmse,
        maxError: result.maxError,
        intrinsics: result.intrinsics,
        diagnostics: {
          usedPoints: result.diagnostics.usedPoints,
          behindCameraPoints: result.diagnostics.behindCameraPoints,
          matrixRank: result.diagnostics.matrixRank,
          conditionNumber: result.diagnostics.conditionNumber,
          singularValues: result.diagnostics.singularValues,
          warnings: result.diagnostics.warnings,
        },
      };
      if (opt("errors-csv")) {
        fs.writeFileSync(opt("errors-csv")!, errorsToCsv(result.errors));
        console.log(`残差已导出 ${opt("errors-csv")}`);
      }
      if (opt("save")) {
        const store = new Store(DATA_DIR);
        const ps = store.createPointSet(opt("name") ?? "cli_set", parsed.observations, parsed.unit ?? "mm");
        const id = store.saveResult(ps.id, result.config, result, parsed.observations, { note: opt("note") ?? "" });
        console.log(`结果快照已保存 resultId=${id} pointsetId=${ps.id}`);
        store.close();
      }
      out(JSON.stringify(summary, null, 2));
      break;
    }
    case "project": {
      const intr = loadIntrinsics(opt("intrinsics")!);
      const point = JSON.parse(opt("point")!);
      if (opt("extrinsics")) {
        const ext = JSON.parse(fs.readFileSync(opt("extrinsics")!, "utf-8"));
        const cam = transformWorldToCamera(point, ext);
        out(JSON.stringify({ cameraPoint: cam, projection: project(cam, intr) }, null, 2));
      } else {
        out(JSON.stringify(project(point, intr), null, 2));
      }
      break;
    }
    case "unproject": {
      const intr = loadIntrinsics(opt("intrinsics")!);
      const r = unproject(
        Number(opt("u")!),
        Number(opt("v")!),
        Number(opt("depth")!),
        intr,
        {
          maxIter: DEFAULT_CONFIG.undistortMaxIter,
          tol: DEFAULT_CONFIG.undistortTol,
          maxRadius: DEFAULT_CONFIG.undistortMaxRadius,
        }
      );
      out(JSON.stringify(r, null, 2));
      break;
    }
    case "compare": {
      const a = loadIntrinsics(opt("a")!);
      const b = loadIntrinsics(opt("b")!);
      out(JSON.stringify(compareIntrinsics(a, b), null, 2));
      break;
    }
    case "compare-subsets": {
      const pa = readJsonOrCsv(opt("a")!);
      const pb = readJsonOrCsv(opt("b")!);
      out(JSON.stringify(
        compareSubsets(pa.observations, pb.observations, { estimateK: !flag("no-k") }),
        null,
        2
      ));
      break;
    }
    case "outliers": {
      const parsed = readJsonOrCsv(opt("file")!);
      out(JSON.stringify(
        locateOutliers(parsed.observations, { influenceRatio: Number(opt("ratio", "0.2")) }),
        null,
        2
      ));
      break;
    }
    case "results": {
      const store = new Store(DATA_DIR);
      if (opt("id")) {
        const row = store.getResult(Number(opt("id")));
        out(JSON.stringify(row, null, 2));
      } else {
        out(JSON.stringify(store.listResults(), null, 2));
      }
      store.close();
      break;
    }
    default:
      console.log(
        [
          "命令:",
          "  generate [--seed --noise --out --csv]            生成固定种子合成观测",
          "  import --file f.json --name n                     导入点集到 SQLite",
          "  calibrate --file f.json [--no-k] [--errors-csv e.csv] [--save]",
          "  project --intrinsics k.json --point '{\"x\":..}' [--extrinsics e.json]",
          "  unproject --intrinsics k.json --u --v --depth",
          "  compare --a k1.json --b k2.json",
          "  compare-subsets --a a.json --b b.json [--no-k]",
          "  outliers --file f.json [--ratio 0.2]",
          "  results [--id N]",
        ].join("\n")
      );
  }
}

main().catch((e) => {
  console.error("错误:", e.message);
  process.exit(1);
});

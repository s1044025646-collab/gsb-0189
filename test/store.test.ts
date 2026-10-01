import { test } from "node:test";
import assert from "node:assert/strict";
import * as fs from "node:fs";
import * as os from "node:os";
import * as path from "node:path";
import { Store } from "../src/store";
import { calibrate } from "../src/calibration";
import { generate } from "../src/synthetic";
import { parseObservationsCsv, errorsToCsv } from "../src/io";
import { Intrinsics } from "../src/types";

test("结果快照可按原数据复现，且点集同名产生新版本", () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "calib-test-"));
  const store = new Store(dir);
  const truth: Intrinsics = { fx: 700, fy: 700, cx: 300, cy: 200, k1: -0.2, k2: 0.05 };
  const obs = generate({
    seed: 5, intrinsics: truth, cols: 8, rows: 6,
    width: 400, height: 300, zBase: 1000, zJitter: 60, noiseSigma: 0.2,
  }).observations;
  const config = { estimateK: true, maxIterations: 30, tol: 1e-10,
    undistortMaxIter: 100, undistortTol: 1e-9, undistortMaxRadius: 10, rankTol: 1e-9, minPoints: 6 };
  const result = calibrate(obs, config);
  const ps1 = store.createPointSet("ds", obs);
  const id = store.saveResult(ps1.id, config, result, obs);
  const ps2 = store.createPointSet("ds", obs);
  assert.equal(ps1.version, 1);
  assert.equal(ps2.version, 2);
  const saved = store.getResult(id)!;
  const rerun = calibrate(saved.pointset_snapshot, saved.config);
  assert.equal(rerun.rmse, saved.result.rmse);
  assert.deepEqual(rerun.intrinsics, saved.result.intrinsics);
  store.close();
});

test("CSV 往返与残差导出", () => {
  const csv = "id,x,y,z,u,v\np0,1,2,10,80,60\np1,3,4,10,240,320";
  const obs = parseObservationsCsv(csv);
  assert.equal(obs.length, 2);
  assert.equal(obs[0].pixel.v, 60);
  const out = errorsToCsv([
    { id: "p0", u: 1, v: 2, uReproj: 1.1, vReproj: 2, residualU: 0.1, residualV: 0,
      distance: 0.1, depth: 10, behindCamera: false },
  ]);
  assert.ok(out.includes("p0,1,2,1.1,2,0.1,0,0.1,10,false"));
});

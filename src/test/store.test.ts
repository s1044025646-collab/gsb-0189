import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { CalibService } from "../service.js";
import { generateSynthetic } from "../synthetic.js";

function tempService(): { service: CalibService; dir: string } {
  const dir = mkdtempSync(join(tmpdir(), "pinhole-"));
  return { service: new CalibService({ dataDir: dir }), dir };
}

test("相机/点集版本递增且历史结果可复现", () => {
  const { service } = tempService();
  const bundle = generateSynthetic({ seed: 3, pointCount: 50, noiseSigmaPx: 0.3 });
  const ps1 = service.savePointSet("b", bundle.observations);
  const ps2 = service.savePointSet("b", bundle.observations.slice(0, 30));
  assert.equal(ps1.version, 1);
  assert.equal(ps2.version, 2);
  const cam1 = service.saveCamera("c", bundle.config.intrinsics);
  assert.equal(cam1.version, 1);

  const { solveId, result } = service.calibratePointSet(ps1.id, { tol: 1e-11 });
  const repro = service.reproduce(solveId);
  assert.equal(repro.rmsMatch, true);
  assert.ok(Math.abs(repro.reproducedRms - result.rms) < 1e-12);

  const rows = service.store.listSolves();
  assert.equal(rows.length, 1);
  assert.equal(rows[0]!.pointset_id, ps1.id);
  service.store.close();
});

test("残差 CSV 可导出且含每点距离", () => {
  const { service } = tempService();
  const bundle = generateSynthetic({ seed: 4, pointCount: 30, noiseSigmaPx: 0.2 });
  const ps = service.savePointSet("b", bundle.observations);
  const { solveId, result } = service.calibratePointSet(ps.id);
  const csv = service.residualsCsv(result);
  const lines = csv.trim().split("\n");
  assert.equal(lines.length, result.perPoint.length + 1);
  assert.match(lines[0]!, /distance_px/);
  assert.ok(service.store.getSolve(solveId));
  service.store.close();
});

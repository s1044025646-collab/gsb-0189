import { test } from "node:test";
import assert from "node:assert/strict";
import { calibrate } from "../src/calibration";
import { generate } from "../src/synthetic";
import { project, unproject } from "../src/geometry";
import { transformWorldToCamera } from "../src/geometry";
import { DEFAULT_CONFIG, Intrinsics, PointObservation, Extrinsics } from "../src/types";

const truth: Intrinsics = { fx: 800, fy: 802, cx: 320, cy: 240, k1: -0.28, k2: 0.12 };

test("无畸变噪声为零时精确恢复内参", () => {
  const set = generate({
    seed: 1, intrinsics: { ...truth, k1: 0, k2: 0 },
    cols: 10, rows: 8, width: 500, height: 380, zBase: 1000, zJitter: 80, noiseSigma: 0,
  });
  const r = calibrate(set.observations);
  assert.equal(r.status, "ok");
  assert.ok(r.converged);
  assert.ok(Math.abs(r.intrinsics!.fx - 800) < 1e-4);
  assert.ok(Math.abs(r.intrinsics!.fy - 802) < 1e-4);
  assert.ok(Math.abs(r.intrinsics!.cx - 320) < 1e-4);
  assert.ok(Math.abs(r.intrinsics!.cy - 240) < 1e-4);
  assert.ok(Math.abs(r.intrinsics!.k1) < 1e-6);
  assert.ok(Math.abs(r.intrinsics!.k2) < 1e-6);
  assert.ok(r.rmse! < 1e-6);
});

test("带畸变噪声为零时精确恢复 fx..cy,k1,k2", () => {
  const set = generate({
    seed: 7, intrinsics: truth,
    cols: 12, rows: 10, width: 600, height: 450, zBase: 1200, zJitter: 100, noiseSigma: 0,
  });
  const r = calibrate(set.observations);
  assert.equal(r.status, "ok");
  assert.ok(Math.abs(r.intrinsics!.fx - truth.fx) < 1e-2);
  assert.ok(Math.abs(r.intrinsics!.k1 - truth.k1) < 1e-5);
  assert.ok(Math.abs(r.intrinsics!.k2 - truth.k2) < 1e-5);
});

test("固定种子高斯噪声下参数与误差在受控范围内", () => {
  const sigma = 0.5;
  const set = generate({
    seed: 42, intrinsics: truth,
    cols: 20, rows: 16, width: 1200, height: 900, zBase: 1200, zJitter: 100, noiseSigma: sigma,
  });
  const r = calibrate(set.observations);
  assert.equal(r.status, "ok");
  // RMSE 应接近噪声水平（每坐标 sigma -> 距离约 sqrt(2)*sigma 的宽松上界内）。
  assert.ok(r.rmse! < 1.0, `RMSE=${r.rmse}`);
  assert.ok(r.rmse! > 0.3, `RMSE=${r.rmse}`);
  // 参数偏差处于合理的小范围。
  assert.ok(Math.abs(r.intrinsics!.fx - truth.fx) < 2);
  assert.ok(Math.abs(r.intrinsics!.cx - truth.cx) < 2);
  assert.ok(Math.abs(r.intrinsics!.k1 - truth.k1) < 0.02);
  assert.ok(Math.abs(r.intrinsics!.k2 - truth.k2) < 0.02);
});

test("投影与给定深度反投影往返一致（无畸变与有畸变）", () => {
  const p = { x: 120, y: -80, z: 900 };
  for (const intr of [truth, { ...truth, k1: 0, k2: 0 }] as Intrinsics[]) {
    const pr = project(p, intr);
    const up = unproject(pr.u, pr.v, p.z, intr, {
      maxIter: 100, tol: 1e-10, maxRadius: 10,
    });
    assert.ok(up.ok, up.reason);
    assert.ok(Math.abs(up.point!.x - p.x) < 1e-4);
    assert.ok(Math.abs(up.point!.y - p.y) < 1e-4);
    assert.equal(up.point!.z, p.z);
  }
});

test("非单位旋转平移的世界点投影与独立手算一致", () => {
  const ext: Extrinsics = {
    rotation: { rx: 0, ry: 0, rz: Math.PI / 2 },
    translation: { x: 0, y: 0, z: 0 },
  };
  // Rz(90°): (x,y,z) -> (-y,x,z)
  const cam = transformWorldToCamera({ x: 1, y: 2, z: 3 }, ext);
  assert.ok(Math.abs(cam.x - -2) < 1e-12);
  assert.ok(Math.abs(cam.y - 1) < 1e-12);
  assert.ok(Math.abs(cam.z - 3) < 1e-12);
  // 手算: k=0, x=-2/3,y=1/3
  const intr: Intrinsics = { fx: 600, fy: 600, cx: 0, cy: 0, k1: 0, k2: 0 };
  const pr = project(cam, intr);
  assert.ok(Math.abs(pr.u - (-400)) < 1e-9);
  assert.ok(Math.abs(pr.v - 200) < 1e-9);
});

test("同轴点（仅深度变化）判为退化", () => {
  const axial: PointObservation[] = Array.from({ length: 12 }, (_, i) => {
    const z = 800 + i * 40;
    const pr = project({ x: 0, y: 0, z }, truth);
    return { id: `ax${i}`, point: { x: 0, y: 0, z }, pixel: { u: pr.u, v: pr.v } };
  });
  const r = calibrate(axial);
  assert.equal(r.status, "degenerate");
  assert.ok(r.diagnostics.matrixRank < 4);
  assert.equal(r.intrinsics, null);
});

test("单轴变化布局（全部 y/z 恒定）也退化", () => {
  const obs: PointObservation[] = Array.from({ length: 10 }, (_, i) => {
    const x = -200 + i * 40;
    const p = { x, y: 0, z: 1000 };
    const pr = project(p, truth);
    return { id: `l${i}`, point: p, pixel: { u: pr.u, v: pr.v } };
  });
  const r = calibrate(obs);
  assert.equal(r.status, "degenerate");
});

test("镜头后方点被剔除并报告；全部后方直接失败", () => {
  const set = generate({
    seed: 3, intrinsics: truth,
    cols: 8, rows: 6, width: 400, height: 300, zBase: 1000, zJitter: 40, noiseSigma: 0,
  }).observations;
  const mixed = [...set, { id: "b", point: { x: 1, y: 1, z: -2 }, pixel: { u: 0, v: 0 } }];
  const r = calibrate(mixed);
  assert.deepEqual(r.diagnostics.behindCameraPoints, ["b"]);
  assert.equal(r.status, "ok");
  const allBehind = calibrate([
    { point: { x: 0, y: 0, z: -1 }, pixel: { u: 0, v: 0 } },
  ]);
  assert.equal(allBehind.status, "all_points_behind_camera");
});

test("极端畸变反投影不收敛并给出原因", () => {
  const extreme: Intrinsics = { fx: 800, fy: 800, cx: 320, cy: 240, k1: -3, k2: 2 };
  const up = unproject(10, 10, 1000, extreme, {
    maxIter: 100, tol: 1e-9, maxRadius: 10,
  });
  assert.equal(up.ok, false);
  assert.ok(up.reason!.length > 0);
});

test("非正深度反投影被拒绝", () => {
  const up = unproject(100, 100, 0, truth, {
    maxIter: 10, tol: 1e-9, maxRadius: 10,
  });
  assert.equal(up.ok, false);
});

test("固定种子噪声可复现：同种子两次生成完全一致", () => {
  const a = generate({
    seed: 99, intrinsics: truth, cols: 5, rows: 5,
    width: 300, height: 300, zBase: 1000, zJitter: 0, noiseSigma: 0.5,
  }).observations;
  const b = generate({
    seed: 99, intrinsics: truth, cols: 5, rows: 5,
    width: 300, height: 300, zBase: 1000, zJitter: 0, noiseSigma: 0.5,
  }).observations;
  assert.deepEqual(a, b);
});

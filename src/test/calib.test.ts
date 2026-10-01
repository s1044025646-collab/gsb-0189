import { test } from "node:test";
import assert from "node:assert/strict";
import {
  backproject,
  projectCameraPoint,
  projectWorldPoint,
  rotationFromAxisAngle,
  undistortNormalized,
} from "../geometry.js";
import { runCalibration } from "../service.js";
import { generateSynthetic } from "../synthetic.js";
import { compareIntrinsics } from "../compare.js";
import type { Intrinsics, Observation } from "../types.js";

const ideal: Intrinsics = { fx: 800, fy: 800, cx: 320, cy: 240, k1: 0, k2: 0 };

test("无畸变参数精确恢复", () => {
  const bundle = generateSynthetic({
    seed: 1,
    pointCount: 100,
    noiseSigmaPx: 0,
    intrinsics: ideal,
  });
  const result = runCalibration(bundle.observations);
  assert.ok(result.converged, result.convergenceReason);
  assert.ok(result.diagnostics.identifiable);
  for (const d of compareIntrinsics(result.intrinsics, ideal)) {
    assert.ok(
      Math.abs(d.absolute) < 1e-4,
      `${d.parameter} 偏差 ${d.absolute} 过大`,
    );
  }
  assert.ok(result.rms < 1e-5, `无噪 RMS 应≈0, 实际 ${result.rms}`);
});

test("固定噪声下误差在有界范围内", () => {
  const truth = { fx: 900, fy: 900, cx: 640, cy: 400, k1: -0.25, k2: 0.07 };
  const bundle = generateSynthetic({
    seed: 99,
    pointCount: 200,
    noiseSigmaPx: 0.3,
    xyHalf: 1.1,
    zRange: [1.0, 2.5],
    intrinsics: truth,
  });
  const result = runCalibration(bundle.observations);
  assert.ok(result.converged);
  // 二维各向 σ=0.3 的理论 RMS≈σ√2≈0.424，计入少量拟合误差取 0.5 上界
  assert.ok(result.rms < 0.5, `RMS ${result.rms} 超过噪声量级上界`);
  assert.ok(Math.abs(result.intrinsics.fx - 900) < 6, "fx 偏差超出蒙特卡洛容差");
  assert.ok(Math.abs(result.intrinsics.fy - 900) < 6, "fy 偏差超出蒙特卡洛容差");
  assert.ok(Math.abs(result.intrinsics.cx - 640) < 4, "cx 偏差超出容差");
  assert.ok(Math.abs(result.intrinsics.cy - 400) < 4, "cy 偏差超出容差");
  assert.ok(Math.abs(result.intrinsics.k1 - -0.25) < 0.01, "k1 偏差超出容差");
  assert.ok(Math.abs(result.intrinsics.k2 - 0.07) < 0.02, "k2 偏差超出容差");
});

test("手算投影结果（独立于实现的显式数值）", () => {
  const k: Intrinsics = { fx: 100, fy: 100, cx: 10, cy: 20, k1: 0.1, k2: 0 };
  // X/Z=0.1, Y/Z=0.2, r2=0.05, D=1+0.1*0.05=1.005
  const p = projectCameraPoint({ x: 0.1, y: 0.2, z: 1 }, k)!;
  assert.ok(Math.abs(p.u - (100 * 0.1 * 1.005 + 10)) < 1e-12);
  assert.ok(Math.abs(p.v - (100 * 0.2 * 1.005 + 20)) < 1e-12);
});

test("投影-反投影往返一致（含畸变）", () => {
  const k: Intrinsics = { fx: 900, fy: 900, cx: 640, cy: 400, k1: -0.28, k2: 0.08 };
  for (const orig of [
    { x: 0.3, y: 0.2, z: 2.0 },
    { x: -0.5, y: 0.4, z: 3.5 },
    { x: 0.1, y: -0.3, z: 1.2 },
  ]) {
    const px = projectCameraPoint(orig, k)!;
    const bp = backproject(px, orig.z, k);
    assert.ok(bp.converged, bp.reason);
    assert.ok(Math.abs(bp.point!.x - orig.x) < 1e-8);
    assert.ok(Math.abs(bp.point!.y - orig.y) < 1e-8);
    assert.ok(Math.abs(bp.point!.z - orig.z) < 1e-12);
  }
});

test("不同深度反投影落在同一条射线上", () => {
  const k: Intrinsics = { fx: 500, fy: 500, cx: 0, cy: 0, k1: 0, k2: 0 };
  const px = { u: 50, v: -25 };
  const a = backproject(px, 1, k).point!;
  const b = backproject(px, 3, k).point!;
  assert.ok(Math.abs(b.x / a.x - 3) < 1e-10);
  assert.ok(Math.abs(b.y / a.y - 3) < 1e-10);
});

test("非单位旋转平移后的世界点投影", () => {
  const ext = { R: rotationFromAxisAngle([0, 0, 1], Math.PI / 2), t: [1, 0, 0] as [number, number, number] };
  // 世界点 (0,0,2): 旋转90°(z) -> (0,0,2), +t -> (1,0,2)
  const px = projectWorldPoint({ x: 0, y: 0, z: 2 }, ext, ideal)!;
  // xn=0.5 -> u=800*0.5+320=720, yn=0 -> v=240
  assert.ok(Math.abs(px.u - 720) < 1e-9);
  assert.ok(Math.abs(px.v - 240) < 1e-9);
});

test("镜头后方点被拒绝", () => {
  assert.equal(projectCameraPoint({ x: 0, y: 0, z: -1 }, ideal), null);
  assert.equal(projectCameraPoint({ x: 0, y: 0, z: 0 }, ideal), null);
  const bp = backproject({ u: 1, v: 1 }, -2, ideal);
  assert.equal(bp.converged, false);
  assert.match(bp.reason, /深度/);
});

test("同轴（共轴）退化布局被标记为不可辨识", () => {
  const obs: Observation[] = Array.from({ length: 20 }, (_, i) => ({
    id: `a${i}`,
    point: { x: 0, y: 0, z: 1 + i * 0.1 },
    pixel: { u: 320, v: 240 },
  }));
  const r = runCalibration(obs);
  assert.equal(r.diagnostics.identifiable, false);
  assert.ok(r.diagnostics.warnings.length > 0);
  assert.ok(Number.isNaN(r.intrinsics.fx));
});

test("点数不足被拒绝", () => {
  const obs: Observation[] = Array.from({ length: 4 }, (_, i) => ({
    id: `s${i}`,
    point: { x: i * 0.1, y: i * 0.05, z: 2 },
    pixel: { u: 100 + i, v: 100 + i },
  }));
  const r = runCalibration(obs);
  assert.equal(r.diagnostics.identifiable, false);
  assert.match(r.convergenceReason, /点数/);
});

test("共线点（归一化坐标共线）秩不足", () => {
  // 所有 xn 与 yn 线性相关且覆盖有限：3 列设计 [xn,yn,1] 中 xn,yn 成比例 => 秩<=2
  const obs: Observation[] = Array.from({ length: 20 }, (_, i) => {
    const t = i / 19;
    const xn = t * 0.3;
    const yn = t * 0.6;
    const z = 2;
    return {
      id: `l${i}`,
      point: { x: xn * z, y: yn * z, z },
      pixel: { u: 800 * xn + 320, v: 800 * yn + 240 },
    };
  });
  const r = runCalibration(obs);
  assert.equal(r.diagnostics.rank < 3, true);
  assert.equal(r.diagnostics.identifiable, false);
});

test("极端畸变去畸变不收敛诊断", () => {
  // k1=-3,k2=2 时 rd=rho(1-3rho^2+2rho^4) 非单调，约 0.31~0.38 区间不存在物理逆解
  const k: Intrinsics = { fx: 900, fy: 900, cx: 640, cy: 400, k1: -3, k2: 2 };
  const u = undistortNormalized(0.35, 0, k);
  assert.equal(u.converged, false);
});

test("残差异常点被定位", () => {
  const bundle = generateSynthetic({
    seed: 5,
    pointCount: 60,
    noiseSigmaPx: 0.2,
    intrinsics: ideal,
  });
  // 注入一个 40 像素的粗差
  bundle.observations[10]!.pixel.u += 40;
  const r = runCalibration(bundle.observations);
  assert.ok(r.outliers.some((o) => o.id === bundle.observations[10]!.id));
});

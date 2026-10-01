/**
 * 端到端演示:
 * A) 生成带固定种子噪声的观测 -> 标定 -> 与真值比较 -> 导出每点残差。
 * B) 同轴退化数据：说明为何无法可靠标定（秩诊断）。
 * C) 投影/给定深度反投影往返验证（含非单位旋转平移）。
 * D) 极端畸变的迭代不收敛诊断。
 */
import * as fs from "node:fs";
import * as path from "node:path";
import { calibrate } from "./calibration";
import { project, transformWorldToCamera, unproject } from "./geometry";
import { generate } from "./synthetic";
import { compareIntrinsics } from "./analyze";
import { errorsToCsv } from "./io";
import {
  CalibrationConfig,
  DEFAULT_CONFIG,
  Extrinsics,
  Intrinsics,
  PointObservation,
} from "./types";

const outDir = path.join(process.cwd(), "data", "demo");
fs.mkdirSync(outDir, { recursive: true });

const truth: Intrinsics = { fx: 800, fy: 802, cx: 320, cy: 240, k1: -0.28, k2: 0.12 };

function hr(title: string) {
  console.log("\n" + "=".repeat(70));
  console.log(title);
  console.log("=".repeat(70));
}

hr("A. 生成观测 -> 标定 -> 与真值比较 -> 导出残差");
const set = generate({
  seed: 42,
  intrinsics: truth,
  cols: 20,
  rows: 16,
  width: 1200,
  height: 900,
  zBase: 1000,
  zJitter: 80,
  noiseSigma: 0.4,
});
console.log(`生成 ${set.observations.length} 个点, 噪声 sigma=0.4px, seed=42`);
const result = calibrate(set.observations, { estimateK: true });
console.log("状态:", result.status, "| 收敛:", result.converged, "| 迭代:", result.iterations);
console.log(result.convergenceReason);
console.log("RMSE:", result.rmse?.toFixed(4), "px  最大误差:", result.maxError?.toFixed(4), "px");
console.log("真值:", truth);
console.log("估计:", result.intrinsics);
console.log("参数差异:");
for (const d of compareIntrinsics(result.intrinsics!, truth))
  console.log(`  ${d.param}: 估计=${d.a.toFixed(5)} 真值=${d.b.toFixed(5)} 差=${d.absDelta.toFixed(5)}`);
const csvFile = path.join(outDir, "residuals.csv");
fs.writeFileSync(csvFile, errorsToCsv(result.errors));
console.log("每点残差已导出:", csvFile);

hr("B. 退化布局：所有点位于光轴上（x=y=0，仅深度变化）");
const axial: PointObservation[] = Array.from({ length: 12 }, (_, i) => {
  const z = 800 + i * 50;
  const p = { x: 0, y: 0, z };
  const pr = project(p, truth);
  return { id: `ax${i}`, point: p, pixel: { u: pr.u, v: pr.v } };
});
const bad = calibrate(axial);
console.log("状态:", bad.status);
console.log(bad.convergenceReason);
console.log("秩:", bad.diagnostics.matrixRank, "/ 4, 条件数:", bad.diagnostics.conditionNumber);
console.log("奇异值:", bad.diagnostics.singularValues.map((s) => s.toExponential(2)));
console.log("结论: 主点与焦距在该几何下不可辨识，拒绝给出看似合理的参数。");

hr("C. 非单位旋转平移下的投影 + 给定深度反投影往返");
const ext: Extrinsics = {
  rotation: { rx: 0.12, ry: -0.2, rz: 0.07 },
  translation: { x: 30, y: -15, z: 1000 },
};
const worldPoint = { x: 40, y: -25, z: 60 };
const camPoint = transformWorldToCamera(worldPoint, ext);
const proj = project(camPoint, truth);
const back = unproject(proj.u, proj.v, camPoint.z, truth, {
  maxIter: DEFAULT_CONFIG.undistortMaxIter,
  tol: DEFAULT_CONFIG.undistortTol,
  maxRadius: DEFAULT_CONFIG.undistortMaxRadius,
});
console.log("世界点:", worldPoint);
console.log("相机点:", camPoint);
console.log("投影像素:", { u: proj.u, v: proj.v });
console.log("给定 z 反投影点:", back.point, " 迭代次数:", back.iterations);
const roundErr = Math.hypot(
  back.point!.x - camPoint.x,
  back.point!.y - camPoint.y
);
console.log("往返平面误差(mm):", roundErr.toExponential(2));

hr("D. 极端畸变 -> 迭代去畸变不收敛诊断");
const extreme: Intrinsics = { fx: 800, fy: 800, cx: 320, cy: 240, k1: -3.0, k2: 2.0 };
const farPixel = { u: 10, v: 10 };
const up = unproject(farPixel.u, farPixel.v, 1000, extreme, {
  maxIter: 100,
  tol: 1e-9,
  maxRadius: 10,
});
console.log("ok:", up.ok, "| 迭代:", up.iterations);
console.log("原因:", up.reason);

hr("E. 镜头后方点被剔除");
const withBehind: PointObservation[] = [
  ...set.observations.slice(0, 10),
  { id: "behind", point: { x: 10, y: 10, z: -5 }, pixel: { u: 1, v: 1 } },
];
const rb = calibrate(withBehind);
console.log("剔除点:", rb.diagnostics.behindCameraPoints, " 有效点:", rb.diagnostics.usedPoints);
console.log("状态:", rb.status, "RMSE:", rb.rmse?.toFixed(4));

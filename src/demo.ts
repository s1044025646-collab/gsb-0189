import { mkdirSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import { CalibService, runCalibration } from "./service.js";
import { generateSynthetic } from "./synthetic.js";
import { compareIntrinsics } from "./compare.js";
import { backproject, projectCameraPoint } from "./geometry.js";
import type { Observation } from "./types.js";

function line(s: string): void {
  console.log(s);
}

function main(): void {
  mkdirSync("data/demo", { recursive: true });
  const service = new CalibService({ dataDir: "./data/demo" });

  line("================ 演示 1：从合成观测到标定与真值比较 ================");
  const truth = { fx: 900, fy: 900, cx: 640, cy: 400, k1: -0.28, k2: 0.08 };
  const bundle = generateSynthetic({
    seed: 2026,
    pointCount: 200,
    noiseSigmaPx: 0.3,
    xyHalf: 1.1,
    zRange: [1.0, 2.5],
    intrinsics: truth,
  });
  line(`生成 ${bundle.observations.length} 个带噪声观测（固定种子 2026, σ=0.3px, 宽视场）`);
  line(`外参: 绕 Y 轴旋转 0.12 rad, 平移 [0.05,-0.02,0] 米（非单位旋转平移）`);

  const ps = service.savePointSet("demo-board", bundle.observations, "演示点集");
  const cam = service.saveCamera("demo-camera-真值", truth, "生成时真值");
  line(`点集已入库 id=${ps.id}, 相机真值 id=${cam.id}`);

  const { solveId, result } = service.calibratePointSet(ps.id, undefined, cam.id);
  line(`\n标定完成 solveId=${solveId}, 收敛=${result.converged}, 迭代=${result.iterations}`);
  line(`收敛判据: ${result.convergenceReason}`);
  line(`RMS 重投影误差 = ${result.rms.toFixed(4)} 像素`);
  line(`几何可辨识 = ${result.diagnostics.identifiable}, 设计矩阵秩 = ${result.diagnostics.rank}, 条件数≈${result.diagnostics.conditionNumber.toExponential(2)}`);

  line("\n估计值 vs 真值：");
  const table = compareIntrinsics(result.intrinsics, truth);
  for (const d of table) {
    line(
      `  ${d.parameter.padEnd(4)} 估计=${d.estimated.toFixed(5).padStart(12)}  真值=${d.truth
        .toFixed(5)
        .padStart(12)}  绝对差=${d.absolute.toFixed(5).padStart(10)}  相对=${
        d.relativePercent === null ? "n/a" : d.relativePercent.toFixed(3) + "%"
      }`,
    );
  }
  if (result.outliers.length) line(`\n异常残差点: ${result.outliers.map((o) => `${o.id}(z=${o.zScore.toFixed(1)})`).join(", ")}`);
  else line("\n未检出异常残差点（合成数据无离群）。");

  const csv = service.residualsCsv(result);
  writeFileSync("data/demo/residuals.csv", csv, "utf-8");
  line("每点残差已导出: data/demo/residuals.csv");

  const repro = service.reproduce(solveId);
  line(`\n复现校验: 存储RMS=${repro.storedRms} 重算RMS=${repro.reproducedRms} 一致=${repro.rmsMatch}`);

  line("\n---- 投影 / 给定深度反投影往返 ----");
  const original = bundle.cameraPoints[0]!;
  const proj = projectCameraPoint(original, result.intrinsics)!;
  const bp = backproject(proj, original.z, result.intrinsics);
  line(
    `原点=(${original.x.toFixed(4)},${original.y.toFixed(4)},${original.z.toFixed(4)}) ` +
      `投影=(${proj.u.toFixed(3)},${proj.v.toFixed(3)}) 反投影收敛=${bp.converged} ` +
      `往返点=(${bp.point!.x.toFixed(5)},${bp.point!.y.toFixed(5)},${bp.point!.z.toFixed(5)})`,
  );
  const roundtripErr = Math.hypot(
    bp.point!.x - original.x,
    bp.point!.y - original.y,
    bp.point!.z - original.z,
  );
  line(`三维往返误差 = ${roundtripErr.toExponential(2)} 米`);

  line("\n================ 演示 2：退化（同轴/共线）布局 ================");
  // 所有点沿光轴附近、归一化 x/y 几乎相同且极小：fx 与 cx 不可分离
  const deg: Observation[] = Array.from({ length: 20 }, (_, i) => {
    const z = 1 + i * 0.15;
    const x = 1e-7 * z;
    const y = 0;
    const u = 900 * (x / z) + 640;
    const v = 400;
    return { id: `d${i}`, point: { x, y, z }, pixel: { u, v } };
  });
  const dres = runCalibration(deg);
  line(`有效点数=${dres.diagnostics.pointCount}, 设计矩阵秩=${dres.diagnostics.rank}, 可辨识=${dres.diagnostics.identifiable}`);
  line("诊断警告:");
  for (const w of dres.diagnostics.warnings) line(`  - ${w}`);
  line(`结论: ${dres.convergenceReason}`);
  line(`返回内参含 NaN，拒绝给出“看似合理”的标定：fx=${String(dres.intrinsics.fx)}, cx=${String(dres.intrinsics.cx)}`);

  line("\n================ 演示 3：极端畸变不收敛诊断 ================");
  const extreme = generateSynthetic({
    seed: 7,
    pointCount: 40,
    noiseSigmaPx: 0,
    intrinsics: { fx: 900, fy: 900, cx: 640, cy: 400, k1: -3.0, k2: 2.0 },
    xyHalf: 1.2,
  });
  // 用一个偏离像面很远的像素做去畸变，触发不收敛
  const badK = { fx: 900, fy: 900, cx: 640, cy: 400, k1: -3.0, k2: 2.0 };
  const bad = backproject({ u: 640 + 900 * 0.35, v: 400 }, 2.0, badK);
  line(`极端像素去畸变: converged=${bad.converged}, reason=${bad.reason}`);
  const eres = runCalibration(extreme.observations);
  line(`极端畸变数据标定 RMS=${eres.rms.toFixed(3)}, 去畸变失败点数=${eres.undistortFailedCount}, 收敛=${eres.converged}`);

  service.store.close();
  line("\n演示完成。");
}

main();

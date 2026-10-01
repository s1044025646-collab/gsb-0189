# 针孔相机标定与畸变反投影后端

Windows 本地可运行的**离线相机几何实验服务**（仅 API + CLI，无照片读取、无棋盘格检测、无三维渲染、无摄像头）。
使用 **TypeScript + Node.js + SQLite**，最小二乘、径向畸变、迭代去畸变与非线性精修全部**自行实现**，
不调用任何现成相机标定算法。

## 坐标与单位约定

- 世界坐标与相机坐标均为**右手坐标系**，长度单位默认 **毫米(mm)**（可在数据集中声明，全套数据须一致）。
- 相机坐标系：`+X` 向右、`+Y` 向下、`+Z` 沿光轴指向场景，相机看向 `+Z`；`z <= 0` 的点位于镜头后方。
- 像素坐标系：`u` 向右、`v` 向下；主点 `(cx, cy)`、焦距 `fx, fy` 单位为像素。
- 外参：`P_cam = R * P_world + t`，旋转以 **ZYX 内旋欧拉角（弧度）** 给出，`R = Rz(rz) Ry(rx) Rx(rx)`。
- 畸变模型（仅两个径向系数，无切向项）：
  `x_d = x (1 + k1 r^2 + k2 r^4)`，`y_d` 同理，`x = X/Z, y = Y/Z, r^2 = x^2 + y^2`。
- 针孔投影：`u = fx x_d + cx`，`v = fy y_d + cy`。

## 标定方法（写清算法与收敛判断）

外参已知（输入点给在相机坐标系下），估计 `fx, fy, cx, cy, k1, k2`，分三阶段：

1. **线性针孔初值**：置 `k1=k2=0`，每个点给出两个线性方程
   `u = fx*x + cx`、`v = fy*y + cy`，用正规方程 `(A^T A)^-1 A^T b` 求解（高斯-若当求逆自行实现）。
2. **线性交替热启动**（可选，`estimateK=true` 时 3 轮）：固定内参对 `k1,k2` 做线性最小二乘
   （`(u-cx)/fx - x = k1 x r^2 + k2 x r^4`，u/v 两轴各一组），再固定 `k1,k2` 重解内参，
   仅用于把初值带到真值附近。
3. **高斯-牛顿（Levenberg–Marquardt 阻尼）联合精修**：对全部参数用**解析雅可比**迭代，
   `(J^T J + λ diag(J^T J)) δ = -J^T r`；残差不降时增大 λ，下降时减小 λ。

**收敛判断**：最大参数相对变化量 `max|δ_i|/(1+|p_i|) < tol(=1e-10)` 且 RMSE 下降量 `< tol`；
或 LM 在强阻尼下已找不到下降步长（到达噪声底）。**发散/不收敛**：解出非正焦距、
`|k1|,|k2| > 1e3`、参数非有限，或迭代去畸变半径/畸变因子发散。

**几何可辨识性**：对线性设计矩阵做 Jacobi 奇异值分解（自行实现对称特征值），报告秩、
全部奇异值与条件数。秩 `< 4` 直接判 `degenerate` 并**拒绝返回参数**（如同轴点：`x=y=0` 只剩深度变化）。
同时检查有效点数（默认 ≥6）、镜头后方点、归一化坐标 `x=X/z` 与 `y=Y/z` 的取值多样性、深度范围。

**反投影**：由像素得归一化畸变坐标 `(x_d,y_d)`，定点迭代
`x_{n+1} = x_d / (1 + k1 r_n^2 + k2 r_n^4)` 去畸变，再 `X=x*z, Y=y*z, Z=depth`；
半径超过 `undistortMaxRadius` 或达到最大迭代仍未满足容差即报「不收敛」。

## 环境要求

- Windows + Node.js（开发使用 v25.5.0；依赖内置 `node:sqlite`，建议 Node ≥ 22.5 且为 22/24/25）。
- 无需 Docker、WSL、付费服务或外部在线 API。SQLite 数据库位于项目内 `data/`。

## 构建 / 测试 / 启动 / 演示

```powershell
npm install        # 安装 typescript / tsx / @types/node
npm run build      # 编译到 dist/
npm test           # 运行 node:test 全部用例（13 个）
npm run demo       # 端到端演示（生成→标定→比较真值→导出残差 + 退化案例 + 往返 + 极端畸变）
npm start          # 启动 HTTP API（默认 http://localhost:8787）
```

配置：环境变量 `PORT`（默认 8787）、`DATA_DIR`（默认 `./data`）。

## CLI

```powershell
# 固定种子合成观测（可输出 JSON/CSV）
npm run cli -- generate --seed 42 --noise 0.4 --out data/obs.json
npm run cli -- generate --seed 42 --csv --out data/obs.csv

# 导入点集到 SQLite（同名再次导入自动产生新版本）
npm run cli -- import --file data/obs.json --name boardA

# 标定（--no-k 只做针孔；导出每点残差 CSV；--save 保存结果快照）
npm run cli -- calibrate --file data/obs.json --errors-csv data/resid.csv --save

# 投影（相机坐标点）/ 给定外参的世界点投影 / 给定深度反投影
npm run cli -- project --intrinsics examples/intrinsics.json --point "{\"x\":40,\"y\":-25,\"z\":1063}"
npm run cli -- project --intrinsics examples/intrinsics.json --point "{\"x\":40,\"y\":-25,\"z\":60}" --extrinsics examples/extrinsics.json
npm run cli -- unproject --intrinsics examples/intrinsics.json --u 365.2 --v 206.1 --depth 1063.4

# 参数比较 / 观测子集比较 / 异常残差定位 / 历史结果
npm run cli -- compare --a run1.json --b truth.json
npm run cli -- compare-subsets --a sub1.json --b sub2.json
npm run cli -- outliers --file data/obs.json --ratio 0.2
npm run cli -- results
```

## HTTP API（JSON）

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/health` | 健康检查 |
| GET/POST | `/api/cameras`，`/api/cameras/:id` | 相机版本 |
| GET/POST | `/api/pointsets`，`/api/pointsets/:id` | 点集版本（同名自增 version） |
| POST | `/api/synthetic/generate` | 固定种子合成数据 |
| POST | `/api/project` / `/api/project-world` | 投影 / 带外参投影 |
| POST | `/api/unproject` | 给定深度反投影（含不收敛原因） |
| POST | `/api/calibrate` | 标定，返回参数、RMSE、每点残差、退化诊断 |
| POST | `/api/calibrate-save` | 标定并保存配置+输入快照到 SQLite |
| GET | `/api/results`，`/api/results/:id` | 结果列表 / 详情（含原始点集快照） |
| POST | `/api/results/:id/reproduce` | 用保存的原数据与配置复现旧结果 |
| POST | `/api/compare` / `/api/compare-subsets` / `/api/outliers` | 参数比较 / 子集比较 / 异常定位 |

标定结果字段：`status`（`ok/degenerate/diverged/insufficient_points/all_points_behind_camera`）、
`converged`、`iterations`、`convergenceReason`、`rmse`、`maxError`、`errors[]`（每点
`residualU/residualV/distance/depth/behindCamera`）、`diagnostics`（点数、秩、奇异值、条件数、
归一化坐标与深度范围、告警）、`rmseHistory`。

## SQLite 与可复现性

数据库 `data/calib.db`，表：`cameras`、`pointsets`、`results`。
每条结果保存**求解配置 + 输入点集完整快照 + 结果 JSON**；`reproduce` 用原快照重算，
保证旧结果按原数据可复现（见 `test/store.test.ts` 断言 RMSE 与参数逐位一致）。

## 测试覆盖

- 无畸变（及带畸变）零噪声参数精确恢复（对照生成真值）
- 固定种子高斯噪声下的误差与参数范围（断言 RMSE 接近噪声水平）
- 投影↔给定深度反投影往返（无畸变/有畸变）
- 非单位旋转平移投影对照独立手算（`Rz(90°): (1,2,3)→(-2,1,3)`）
- 同轴点、单轴变化布局的退化判定（秩 < 4，拒绝返回参数）
- 镜头后方点剔除 / 全部后方失败 / 非正深度反投影拒绝
- 极端畸变迭代去畸变不收敛诊断
- 固定种子可复现、SQLite 快照复现、CSV 往返

## 目录

```
src/
  types.ts       类型与坐标约定、默认配置
  linalg.ts      矩阵乘/求逆/最小二乘/Jacobi 奇异值
  geometry.ts    欧拉旋转、投影、定点迭代去畸变反投影
  calibration.ts 线性初值+热启动+几何诊断，调用精修
  refine.ts      高斯-牛顿(LM)解析雅可比精修与收敛判定
  synthetic.ts   mulberry32 固定种子 + Box-Muller 高斯噪声
  analyze.ts     参数比较、子集比较、leave-one-out 异常定位
  io.ts          JSON/CSV 导入与残差导出
  store.ts       node:sqlite 版本与结果快照
  server.ts      HTTP API
  cli.ts         命令行
  demo.ts        端到端演示
test/            node:test 测试
examples/        示例内参/外参/观测(JSON 与 CSV)
```

## 已知范围与假设

- 仅两个径向畸变系数，无切向畸变、无倾斜像素、无 Brown–Conrady 高阶项。
- 标定时外参已知；不从单张图同时求解任意 6DOF 姿态，但提供给定旋转平移的投影接口。
- 最小二乘用正规方程，适合本实验的小参数规模（≤6）与良态数据；退化靠 SVD 秩诊断拦截。
- 反投影必须给定正深度（单目像素本身不能恢复尺度）。
- `k1,k2` 在窄视场/少点下统计不确定度较大，属正常现象；演示与测试用宽视场点云说明可辨识条件。

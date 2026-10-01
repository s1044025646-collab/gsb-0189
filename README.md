# 针孔相机标定与畸变反投影后端

离线相机几何实验服务：管理已知三维点与二维像素观测、相机内参/外参，提供针孔投影、两个径向畸变系数、给定深度反投影，以及从“已知相机坐标三维点 + 像素对应”估计内参的能力。**只做 API 与 CLI**，不读照片、不检测棋盘格、不做三维渲染、不接摄像头；所有输入为合成点集与 JSON/CSV。

技术栈：**TypeScript + Node.js（内置 `node:http` / `node:sqlite`）**。零第三方运行时依赖，无需 Docker、WSL 或任何在线服务，Windows 本地直接运行。需要 Node.js **22.5+**（已在 Node v25 上验证；使用内置 `node:sqlite`）。

---

## 1. 坐标、像素与单位约定（明确）

- 世界坐标 `(Xw,Yw,Zw)` 与相机坐标 `(Xc,Yc,Zc)` 均为**右手直角坐标系**，长度单位统一为**米 (m)**。
- 相机朝 **+Zc** 方向观察；有效点必须满足 **Zc > 0（镜头前方）**。`Zc <= 0` 的点在投影/标定时被拒绝或剔除并计数。
- `Xc` 向右、`Yc` **向下**（OpenCV 风格）。
- 像素坐标：`u` 沿列向右增大、`v` 沿行向下增大，原点在图像左上角；`fx,fy,cx,cy` 单位为像素。
- 外参：`Xc = R · Xw + t`。`R` 为 3×3 旋转矩阵（行主序，长度 9），`t` 为平移向量（米）。导入时也支持 3 维旋转向量 `rvec`（罗德里格斯）。

### 针孔 + 径向畸变模型

归一化坐标 `xn = Xc/Zc, yn = Yc/Zc`，径向畸变因子

```
D = 1 + k1·r² + k2·r⁴,   r² = xn² + yn²
u = fx·xn·D + cx,   v = fy·yn·D + cy
```

仅含两个径向系数 `k1,k2`（无切向畸变）。

**反投影**：由像素先得已畸变归一化坐标 `xd=(u-cx)/fx, yd=(v-cy)/fy`，再求解满足 `xd = xn·D(xn)` 的无畸变坐标。利用标量径向结构，对半径 `ρ` 解一维方程 `rd = ρ(1+k1ρ²+k2ρ⁴)`（带步长限幅的牛顿迭代），最后 `Xc=xn·Z, Yc=yn·Z`（深度 Z 由调用方给定）。当畸变映射非单调、像素落在无物理逆解的区间时，迭代判定**不收敛**并返回明确原因，而不是给出假点。

---

## 2. 标定方法（自实现，未调用任何现成标定算法）

标定时**外参已知**（直接输入相机坐标三维点），不从单张图联合估计任意姿态。

1. **剔除镜头后方点**（`Zc<=0`）并计数。
2. **线性初值**（忽略畸变）：构造设计矩阵，列分别为 `[xn,1]` 与 `[yn,1]`，用正规方程最小二乘分别解 `u = fx·xn + cx`、`v = fy·yn + cy`，得到 `(fx,cx)`、`(fy,cy)`。
3. **联合非线性精化**：以线性解为初值、`k1=k2=0`，对参数 `[fx,fy,cx,cy,k1,k2]` 做带解析雅可比的**高斯-牛顿 + Levenberg–Marquardt 阻尼**最小二乘（阻尼在拒绝步长时增大、接受时减小；约束 `fx,fy>0` 与畸变系数合理区间）。

**最小二乘、雅可比、径向畸变、迭代去畸变全部自行实现**（见 `src/linalg.ts`、`src/calibration.ts`、`src/geometry.ts`），无外部数学/视觉库。

**收敛判断**：相邻迭代 RMS 重投影误差相对变化 `< tol`（默认 `1e-10`）即收敛；或无法再找到可接受的下降步长（亦判定为到达当前最优）；达到 `maxIterations`（默认 100）仍未满足则标记未收敛。

**输出**：最终/线性内参、RMS、**每点 `(du,dv,距离)` 残差**、迭代次数、收敛原因、去畸变失败点数，以及退化诊断（见下），不仅返回一组参数。

### 可辨识性 / 退化诊断

对几何设计矩阵 `[xn,yn,1]` 做 3×3 正规矩阵 Jacobi 特征分解，得到奇异值、有效秩（相对阈值）与条件数，并检查点数、深度范围、视场尺度：

- 有效点数 `< 6`：拒绝。
- 秩 `< 3`（点近似共线/共轴，`xn` 或 `yn` 几乎不变）：焦距与主点不可分离，**拒绝并把参数置为 `NaN`**，`identifiable=false`。
- 条件数 `> 1e8`、视场趋近零、深度全部相同等：给出明确警告（深度相同不影响相机坐标点标定，会注明）。
- 返回镜头后方点数量、`Z` 最小/最大值。

---

## 3. 快速开始（Windows / PowerShell）

```powershell
npm install        # 仅安装 TypeScript 类型（devDependencies）
npm run build      # 编译到 dist/
npm test           # 编译并运行 15 个测试
npm run demo       # 完整演示（合成→标定→真值比较→导出残差→退化案例）

# 启动 HTTP 服务（可配置端口与数据目录）
$env:PORT=8080; $env:DATA_DIR="./data"; npm start
```

### CLI

```powershell
node dist/cli/index.js help

# 生成带固定种子噪声的合成点集并存库
node dist/cli/index.js generate --seed 42 --points 80 --noise 0.3 --save-as board

# 导入 JSON/CSV（列 id,x,y,z,u,v）
node dist/cli/index.js import --file examples/observations.csv --name csv-board

# 标定 / 复现 / 导出残差
node dist/cli/index.js calibrate --pointset-id 1
node dist/cli/index.js reproduce --id 1
node dist/cli/index.js export-residuals --id 1 --out data/residuals.csv

# 给定外参的世界点投影；不带 --extrinsics 时按相机坐标投影
node dist/cli/index.js project --intrinsics examples/intrinsics.json `
  --extrinsics examples/extrinsics.json --points examples/worldpoints.json

# 给定深度反投影（pixels 文件为 [{u,v,depth}, ...]）
node dist/cli/index.js backproject --intrinsics examples/intrinsics.json --pixels px.json

node dist/cli/index.js compare --estimated est.json --truth examples/intrinsics.json
node dist/cli/index.js list solves
```

### HTTP API（JSON）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/health` | 健康检查 |
| POST | `/api/synthetic/generate` | 固定种子合成（`config`，可选 `extrinsics`、`saveAs`） |
| POST | `/api/observations/import` | 导入 JSON/CSV 文本并存点集 |
| GET | `/api/cameras` · `/api/pointsets` · `/api/solves` | 列出版本/结果（可加 `?name=`） |
| POST | `/api/project` | 世界点 + 外参投影 |
| POST | `/api/project/camera` | 相机坐标点投影 |
| POST | `/api/backproject` | 像素 + 深度反投影（`pixels`、`depths` 或统一 `depth`） |
| POST | `/api/calibrate` | 对已存点集标定并保存快照（`pointsetId`,`options`） |
| POST | `/api/calibrate/direct` | 直接对请求中的观测标定（不落库） |
| GET | `/api/solve/:id` | 取某次结果（含每点残差与诊断） |
| POST | `/api/solve/:id/reproduce` | 用保存的原始数据+配置重算并比对 |
| GET | `/api/solve/:id/residuals.csv` | 导出残差 CSV |
| POST | `/api/compare` | 两组内参差异（绝对/相对） |

---

## 4. 可复现性与版本化（SQLite）

数据库文件位于 `<DATA_DIR>/calib.db`（默认 `./data/calib.db`）。

- `cameras`、`pointsets` 均带 `version`：同名再次保存生成新版本，**旧版本永不覆盖**。
- `solves` 保存求解配置、算法标识、**完整结果快照**（含每点残差）与关联点集。
- 历史结果可通过 `/api/solve/:id/reproduce`（或 CLI `reproduce`）用其保存的原始观测与配置重新求解，并断言新旧 RMS 完全一致（容差 `1e-12`）。

---

## 5. 测试

`npm test`，使用 Node 内置测试运行器，15 个用例：

- **无畸变参数恢复**：无噪声下 6 个内参误差 `< 1e-4`，RMS≈0（对照生成真值）。
- **固定噪声误差范围**：固定种子、σ=0.3px 宽视场，断言 RMS 上界与各参数偏差（对照真值）。
- **手算投影**：用独立显式数值核对 `u/v`。
- **投影/给定深度反投影往返**：含畸变小点往返误差 `1e-8`；同像素不同深度共射线。
- **非单位旋转平移**：绕 Z 轴 90° + 平移，核对世界点投影。
- **退化布局**：同轴点（秩 1）、共线点（秩<3）、点数不足，均被标记不可辨识/拒绝。
- **极端畸变**：非单调畸变区间去畸变不收敛诊断。
- **异常残差定位**：注入 40px 粗差，MAD 稳健定位。
- **SQLite 版本/复现**、**残差 CSV**、**HTTP API 端到端**。

## 6. 目录结构

```
src/
  types.ts        类型与坐标/单位约定
  geometry.ts     旋转、投影、径向畸变、迭代去畸变、反投影
  linalg.ts       高斯消元/最小二乘/Jacobi 特征值（秩诊断）
  calibration.ts  线性初值 + 联合 LM 精化 + 退化诊断 + 异常定位
  synthetic.ts    固定种子合成数据（mulberry32 + Box–Muller）
  compare.ts      内参比较、观测子集
  io.ts           JSON/CSV 导入导出、内参/外参解析
  db.ts           SQLite 版本与快照存储
  service.ts      业务编排（含可复现重算）
  server.ts       HTTP API
  cli/index.ts    命令行
  demo.ts         完整演示
  test/           测试
examples/         CSV/JSON 示例
data/             运行期数据目录（SQLite、导出）
```

## 7. 范围与限制（自行假设）

- 仅两个径向系数，无切向畸变、无倾斜像素、无 `fx≈fy` 锁定。
- 标定时外参已知；不做单目姿态（PnP）联合估计。
- 深度需调用方给定（针孔模型只能反投影出射线）；反投影不做立体/多视三角化。
- 去畸变使用一维径向牛顿迭代；极端非单调畸变下部分像素**无物理逆解**，按不收敛处理。
- 线性可辨识性基于相机坐标归一化点的设计矩阵；视场过窄时 `k1` 与 `k2` 会强相关（测试中以宽视场配置保证可分离，实际数据请关注诊断警告与条件数）。
- 输入长度一律视为米；不做单位自动换算。

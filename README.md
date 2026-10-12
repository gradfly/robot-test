# elf3 机器人 · MuJoCo 仿真 + VR 沉浸式观看

在浏览器里跑 **MuJoCo 官方 WebAssembly 引擎**，加载 `elf3.xml` + `scene_bx.xml`
（地面 + 15 级台阶），渲染成三维场景，默认**静止站立**，支持 **VR 头显沉浸式观看**。

---

## 启动本地仿真界面（推荐）

进入 `web/` 目录后任选一种：

| 方式 | 操作 |
| --- | --- |
| **一键（Windows）** | 双击 **`web/start.bat`** |
| **一键（macOS / Linux）** | `cd web && ./start.sh` |
| **命令行** | `cd web && node server.mjs --open` |

启动后：

- **桌面浏览器**：打开 `http://localhost:8080/`
- **VR 头显**：在同一局域网内打开终端打印的 `https://192.168.x.x:8443/`，点 **ENTER VR**

> 只需 **Node.js 18+**（仅用于起静态服务器，不装任何 npm 依赖）。
> 关闭命令行窗口，或按 `Ctrl+C` 即停止服务。

完整的启动说明、命令行参数、停止方式、启动失败排查，见
**[`web/README.md`](web/README.md) 第 1 节「启动本地仿真界面」**。

---

## RL 策略仿真（`elf3_play.py`）

把 `play.py` 那条 mjlab 训练链路**在纯 MuJoCo 上等价复刻**（不需要 GPU / mjlab / MuJoCo-Warp），
直接用 `policy.onnx` + `fight.npz` 驱动 elf3：

```bash
# 第 1 步 · 普通任务（基础链路自检）：零动作，PD 保持默认站姿
python elf3_play.py --policy zero --viewer native

# 第 2 步 · motion-tracking：策略跟踪 fight.npz 的"打拳"动作
python elf3_play.py --policy onnx --viewer native
```

| 参数 | 说明 |
| --- | --- |
| `--viewer native / headless` | 原生 OpenGL 窗口 / 无窗口评测 |
| `--scene plane / stairs` | 平面（对应 mjlab TerrainCfg(plane)）/ 保留 `scene_bx.xml` 的地面+台阶 |
| `--steps N` | 控制步数，0 = 跑到动作结束（244.9 s，12244 帧 @ 50 Hz） |
| `--terminate` | 启用 mjlab tracking 的终止条件 + RSI 复位 |
| `--record-dir render/xxx` | 离屏渲染出 PNG 帧（配合 `--record-every` / `--max-record`） |
| `--policy zero / random / onnx` | 零动作自检 / 随机动作 / 训练策略 |
| `--joint-damping xml / zero` | 保留 XML 关节粘性阻尼 / 置零（见下方消融） |
| `--armature metadata / xml` | armature 取自 kp/(2π·10)² / 保留 XML 的 0.01 |
| `--gain-scale` `--scale-mult` | 运行时缩放 kp/kd 与动作幅值（调参用） |

`python elf3_check.py` 做一致性回归：校验 ONNX metadata ↔ 内置常量、ONNX 内嵌动作表 ↔
`fight.npz`、模型 kinematics ↔ 动作数据、传感器参考系、观测维度。

### 逆向得到的对应关系（全部有实证）

| 项目 | 结论 |
| --- | --- |
| 策略类型 | `fight_47000.pt` / `policy.onnx` 都是 **motion-tracking** 策略：actor obs=160、critic obs=277，与 mjlab `Tracking` 任务完全吻合（58+3+6+3+3+29+29+29=160） |
| 训练参数 | `policy.onnx` 的 ONNX metadata 里保存了 `joint_stiffness` / `joint_damping` / `default_joint_pos` / `action_scale` / `observation_names` / `anchor_body_name` / `body_names`（run=2026-08-01_13-45-22），**无需猜测** |
| 执行器 | mjlab 用的 MuJoCo 内置 `<position>`：kp=stiffness、kv=damping、`kd/kp=4/ω`（10 Hz、ζ=2，27/29 关节；ankle_roll 单独 kd/kp=0.1）、`action_scale=0.25·effort/kp` |
| 控制律 | `ctrl = default_joint_pos + action_scale · policy_output`，timestep 0.005 × decimation 4 = **50 Hz**（与动作数据 50 fps 对齐） |
| 动作数据 | `fight.npz` 与 `policy.onnx` 内嵌的参考动作表**逐帧零误差**一致；13 个跟踪 body 的世界位姿与 `elf3.xml` 的 kinematics **完全吻合**（max Δpos = 2.4e-7 m） |
| 传感器 | `velocimeter` / `gyro` 输出的都是**机体系**速度，训练用的观测项是 `robot/imu_lin_vel`、`robot/imu_ang_vel`（原 XML 里陀螺仪叫 `Body_Gyro`，脚本会补一个 `imu_ang_vel`） |

### 实测效果与消融

50 s 无窗口评测（`--steps 2500`），跟踪误差（单位 m，越小越好）：

| 配置 | anchor 误差均值 | body 误差均值 |
| --- | --- | --- |
| 默认（保留 XML 阻尼） | 0.188 | 0.214 |
| `--joint-damping zero` | **0.150** | **0.170** |

`--joint-damping zero` 略好，说明 XML 里的关节粘性阻尼（5.0/3.0/2.0/1.0/0.5）可能是给
查看器/网页演示后加的，训练模型里未必有；两种都保留为可选项。

参考效果：`render/elf3_fight_tracking.gif`（策略跟踪打拳动作）、`render/elf3_stand_base.gif`（零动作 PD 保持站姿）。

> 说明：`elf3.xml` 的 `<motor>`（力矩执行器）会被脚本在内存里替换成 `<position>`，
> **不修改原 XML 文件**。

---

## 目录导航

| 文件 / 目录 | 说明 |
| --- | --- |
| `web/` | **网页端仿真 + VR 观看工程**（启动入口在这里） |
| `elf3_play.py` | **RL 策略仿真**：纯 MuJoCo 复刻 play.py，跑通普通任务 + motion-tracking |
| `elf3_check.py` | 逆向链路的一致性回归校验 |
| `fight.npz` | 参考动作数据（12244 帧 @ 50 Hz，29 关节 + 13 跟踪 body） |
| `fight_47000.pt` | rsl-rl 训练 checkpoint（actor 160→29，critic 277） |
| `policy.onnx` | 策略 + 内嵌参考动作表（metadata 里含训练参数真值） |
| `elf3.xml` | 机器人模型（第 463 行 `<include file="scene_bx.xml"/>`，一并带入地面与楼梯） |
| `scene_bx.xml` | 场景：地面 + 15 级台阶 |
| `meshes/*.STL` | 机器人网格（34 个） |
| `view_mujoco.py` | MuJoCo **原生 OpenGL 查看器**，核对模型原始装配（需 `pip install mujoco numpy`） |
| `render_views.py` | 用 MuJoCo Python 渲染器输出多视角参考图 |
| `demo_vuer_vr.py` | 早期 vuer / WebXR 版本（已被 `web/` 取代，保留备查） |

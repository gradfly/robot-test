# elf3 · MuJoCo 网页端仿真 + VR 沉浸式观看

在浏览器里跑**真正的 MuJoCo 物理引擎**（官方 WebAssembly 构建），把 `elf3.xml` +
`scene_bx.xml`（楼梯 / 地面）渲染成三维场景，并支持用 **VR 头显沉浸式观看**。
默认状态是**静止站立**：机器人保持 XML 里的初始位形，不跑物理。

---

## 1. 快速开始

### Windows
双击 **`start.bat`**。

### macOS / Linux
```bash
./start.sh
```

### 手动启动
```bash
node server.mjs
# 自定义端口 / 只开 HTTP：
node server.mjs --http-port 8080 --https-port 8443
node server.mjs --no-https
```

启动后终端会打印本机与局域网访问地址：

```
  本机桌面浏览器：
     http://localhost:8080/
     https://localhost:8443/

  VR 头显（同一局域网内在头显浏览器打开）：
     https://192.168.x.x:8443/
```

> 需要 **Node.js 18+**（仅用于起静态服务器，不装任何 npm 依赖）。

---

## 2. 桌面端预览

用 Chrome / Edge 打开 `http://localhost:8080/` 即可。
（`localhost` 属于安全上下文，所以 WebXR 在 HTTP 下也可用。）

- **左键拖动** = 旋转 · **滚轮** = 缩放 · **右键拖动** = 平移
- 右侧面板：显示开关（机器人 / 楼梯 / 地面 / 阴影 / 碰撞体）、6 个视角预设、
  物理开关与复位按钮
- 底部「VR NOT SUPPORTED」按钮在支持 XR 的设备上会变成 **ENTER VR**

---

## 3. VR 头显沉浸式观看

1. **电脑和头显连同一个局域网**（同一个 Wi-Fi / 路由器）。
2. 在头显浏览器里打开终端打印的 **HTTPS** 局域网地址，例如
   `https://192.168.22.19:8443/`。
   - ⚠️ 必须用 **https**（WebXR 的安全上下文要求），http 地址在头显里不会出现 ENTER VR。
3. 首次打开会提示「证书不受信任」→ 选 **高级 → 继续访问**。
   （想彻底免提示：把 `certs/cert.pem` 传到头显里安装为受信任证书。）
4. 点页面下方的 **ENTER VR**。

进入 VR 后：

| 操作 | 作用 |
| --- | --- |
| 左摇杆 | 前后左右移动（1.6 m/s） |
| 右摇杆左右 | 30° 转向 |
| A / X | 把机器人摆到你正前方 2.6 m 并正对你 |
| B / Y | 世界复位（机器人回到世界原点） |

技术细节：参考空间用 `local-floor`，头显里**你的脚踩的地面**就是 MuJoCo 的
`z = 0` 地面。机器人身高约 1.43 m，站在离你 2.6 m 处，原地 1:1 比例观看，
可以走近绕一圈看细节。

---

## 4. 界面功能

| 控件 | 说明 |
| --- | --- |
| 机器人 | 显示 / 隐藏 30 个视觉网格（group 2） |
| 楼梯 / 障碍 | 显示 / 隐藏 `scene_bx.xml` 里的 15 级台阶 |
| 地面网格 | 显示 / 隐藏地板 |
| 阴影 | 开关实时阴影 |
| 碰撞体（调试图层） | 显示 group 3 的 36 个碰撞几何（绿色半透明 capsule） |
| 视角预设 | 正面 / 背面 / 侧面 / 头部特写 / 俯视 / 全景 |
| 运行 MuJoCo 物理 | 默认**关闭**；打开后真实跑 `mj_step`，机器人会因重力倒地（模型无控制器） |
| 复位姿态 | 回到落地对齐后的静止站立姿态 |
| 复位世界位置 | 把世界平移 / 旋转复位 |

---

## 5. 目录结构

```
web/
├── index.html            页面骨架（importmap + 三个面板 + 加载层）
├── css/app.css           全部样式
├── js/
│   ├── main.js           启动流程、渲染循环、UI、WebXR 会话、调试句柄 __elf
│   ├── mjscene.js        MuJoCo（Z-up）→ three.js（Y-up）场景同步
│   └── vrnav.js          WebXR 漫游：定位、摇杆移动、快速转向
├── vendor/
│   ├── mujoco.js/.wasm   MuJoCo 官方 WebAssembly 构建（simulate 版，含 mjv*）
│   └── three.module.js   three.js r170
├── jsm/                  three.js addons（OrbitControls / VRButton / RoomEnvironment）
├── assets/
│   ├── elf3.xml          机器人模型（第 463 行 <include file="scene_bx.xml"/>）
│   ├── scene_bx.xml      场景：地面 + 15 级台阶
│   ├── meshes/*.STL      34 个网格
│   └── manifest.json     资源清单（加载进度用）
├── certs/                自签证书（首次启动自动生成）
├── server.mjs            本地服务器：HTTP + HTTPS 双监听
├── start.bat / start.sh  一键启动
└── tools/                构建 / 调试工具（见下）
```

---

## 6. 实现要点（踩过的坑）

### 6.1 网格顶点必须用 MuJoCo 编译后的数据

MuJoCo 编译 STL 时会把顶点**重心归零**，偏移量记在 `model.mesh_pos` 里
（例如大腿 `l_hip_z_link` 的 `mesh_pos.z = -0.2229`）。

> **错误做法**：直接拿 STL 原始顶点 × `geom_xmat` + `geom_xpos`。
> 漏掉 `mesh_pos` 后，腿部会整体错位 22 cm，看起来像"腿断了"。

**正确做法**：直接用 `model.mesh_vert` / `model.mesh_face` / `model.mesh_normal`
（已经做过重心归零，和 MuJoCo 内部渲染逐顶点一致），再用 `data.geom_xpos` /
`data.geom_xmat` 定位。这样也不需要在前端解析 STL。

### 6.2 坐标映射

MuJoCo 是 Z-up，three.js / WebXR 是 Y-up：

```
(x, y, z)_mujoco  →  three.js 里套两层 Group：
  flip.rotation.x = -π/2      Z-up → Y-up
  yaw .rotation.y = +π/2      机器人正面（MuJoCo +x）朝向 three 的 -Z
```

### 6.3 姿态与落地对齐

- 静止站立 = `mj_forward` 后把 `data.geom_xpos/geom_xmat` 一次性写进 three.js 对象，
  默认**不跑物理**（`renderer.shadowMap.autoUpdate = false`，省开销）。
- 模型初始 free joint 高度（`qpos0[2] = 1.1`）会让脚底离地约 3.7 cm，
  所以加载时按视觉网格最低点自动对齐一次，让脚底正好落在 `z = 0`。
  复位按钮用的也是这个对齐后的姿态。

### 6.4 VR 里的漫游

相机固定在世界原点（由 three.js 用头显姿态驱动），**平移 / 旋转的是装着整个
MuJoCo 世界的 `stage` 组**，等价于世界坐标系漫游，不会和 XR 的头显位姿打架。

---

## 7. 工具（`tools/`）

| 脚本 | 用途 |
| --- | --- |
| `verify.mjs` | **端到端自检**：加载 / 模型 / UI / 开关 / 视角 / 物理 / 复位 / XR，24 项 |
| `shoot.mjs` | 用 CDP 等页面就绪后截图（可注入 JS 改相机、改可见性） |
| `dump-geoms.mjs` | 列出每个视觉网格的世界 AABB，用于和 MuJoCo 对账 |
| `make-manifest.mjs` | 重新生成 `assets/manifest.json` |
| `fetch-three.mjs` | 下载 three.js 与 addons |
| `selfsign.mjs` | 纯 JS 生成自签证书（无 openssl 依赖，SAN 含本机所有内网 IP） |
| `make-cert.mjs` | 证书生成的封装 |
| `ref-render.py` | 用 MuJoCo 官方 Python 渲染器出参考图（需另装 `mujoco`） |

自检：

```bash
node tools/verify.mjs http://localhost:8080/
node tools/verify.mjs https://localhost:8443/   # 头显走的这条路
```

---

## 8. 常见问题

**Q：头显里打开页面没有 ENTER VR？**
A：① 必须用 https 地址；② 确认头显和电脑在同一局域网、能 ping 通；③
用头显自带浏览器（Quest 用 Meta Quest Browser），不要用某些内置的简易浏览器。

**Q：提示证书不安全 / NET::ERR_CERT_AUTHORITY_INVALID？**
A：自签证书的正常现象，选「高级 → 继续访问」。想免提示就把
`certs/cert.pem` 装进头显的受信任证书列表。

**Q：换了网络环境（IP 变了）后头显打不开？**
A：删除 `certs/` 目录后重启服务器，会按当前 IP 重新生成证书。

**Q：加载很慢？**
A：首次要下载约 10 MB 的 MuJoCo WASM + 约 12 MB 的 STL 网格，之后浏览器会缓存。
加载页有进度条和实时日志。

**Q：页面显示"启动失败"？**
A：加载层会直接打印错误信息。常见原因是资源没就位——确认是在 `web/` 目录下、
通过 `server.mjs` 访问（直接双击 `index.html` 用 `file://` 打开会因为
CORS 与 WASM 限制而失败）。

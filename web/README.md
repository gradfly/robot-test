# elf3 · MuJoCo 网页端仿真 + VR 沉浸式观看

在浏览器里跑**真正的 MuJoCo 物理引擎**（官方 WebAssembly 构建），把 `elf3.xml` +  
`scene_bx.xml`（楼梯 / 地面）渲染成三维场景，并支持用 **VR 头显沉浸式观看**。  
默认状态是**静止站立**：机器人保持 XML 里的初始位形，不跑物理。

---

## 1. 启动本地仿真界面

> **本地 MuJoCo 仿真界面 = 在浏览器里跑 MuJoCo 官方 WebAssembly 引擎。**
> 不需要安装 Python / MuJoCo / vuer，只用 Node.js 起一个静态服务器即可。
> MuJoCo WASM、three.js、模型与网格都已随工程放在 `vendor/` 和 `assets/` 里，**离线可用**。

### 1.1 前提：Node.js 18+

仅用于起静态服务器，**不安装任何 npm 依赖**。

- 本机已装 WorkBuddy 时，启动脚本会自动找到内置 Node，无需另装；
- 否则到 [nodejs.org](https://nodejs.org/) 下载 LTS 版；
- 自查版本：`node -v`。

### 1.2 方式一：一键启动（推荐）

| 系统 | 操作 |
| --- | --- |
| Windows | 进入 `web/` 目录，**双击 `start.bat`** |
| macOS / Linux | 进入 `web/` 目录，执行 `./start.sh`（首次需 `chmod +x start.sh`） |

脚本会自动完成：定位 Node → 启动服务器（HTTP + HTTPS 双端口）→ 打开默认浏览器。
**关闭那个命令行窗口即停止服务。**

### 1.3 方式二：命令行启动

```bash
cd "E:/mujoco/robot test/web"   # 关键：必须先 cd 到 web/ 目录
node server.mjs --open          # 起服务器并自动打开浏览器
```

去掉 `--open` 就只起服务器，自己手动访问终端打印的地址：

```bash
node server.mjs
```

### 1.4 命令行参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--http-port <n>` | `8080` | HTTP 端口，桌面浏览器用 |
| `--https-port <n>` | `8443` | HTTPS 端口，VR 头显用 |
| `--no-https` | 关闭 | 只开 HTTP，跳过证书生成（纯桌面预览时最省事） |
| `--open` | 关闭 | 启动后自动用默认浏览器打开 |

```bash
node server.mjs --http-port 9000 --https-port 9443   # 换端口
node server.mjs --no-https                          # 只开 HTTP
```

> 端口被占用时会自动向后顺延（+1），**以终端实际打印的地址为准**。

### 1.5 启动成功的标志

终端会打印本机与局域网访问地址：

```
  本机桌面浏览器：
     http://localhost:8080/
     https://localhost:8443/

  VR 头显（同一局域网内在头显浏览器打开）：
     https://192.168.22.19:8443/
```

浏览器打开 `http://localhost:8080/`，等加载进度条走完、出现机器人的三维场景，即启动成功。

### 1.6 停止服务

- 命令行方式：在运行服务器的终端按 **`Ctrl+C`**；
- 双击 `start.bat` 方式：直接关掉那个黑色命令窗口。

### 1.7 启动失败排查

| 现象 | 原因 / 处理 |
| --- | --- |
| 提示 `[ERROR] 未找到 Node.js` | 装 Node.js 18+ 后重试；或手动指定内置 Node：<br>`"%USERPROFILE%\.workbuddy\binaries\node\versions\22.22.2-6\node.exe" server.mjs --open` |
| 端口被占用 | 会自动 +1 顺延，也可用 `--http-port` / `--https-port` 指定其它端口 |
| 页面一直转圈或报错 | 必须通过 `server.mjs` 访问（`http://…`）；**别直接双击 `index.html`**，`file://` 会被 CORS / WASM 限制拦住 |
| 打开是空白页 | 按 `F12` 看 Console 报错，多半是没在 `web/` 目录下启动导致资源 404 |

### 1.8 可选：MuJoCo 原生查看器（Python）

项目根目录的 `view_mujoco.py` 用 MuJoCo 自带的 OpenGL 窗口打开同一个模型，
不经过 three.js，适合核对模型的**原始装配状态**。它依赖本地 Python 装了 `mujoco`：

```bash
pip install mujoco numpy
python view_mujoco.py          # 静止保持初始装配姿态（核对角度用）
python view_mujoco.py --sim    # 真实物理：跑 mj_step（无控制器会倒地）
python view_mujoco.py --pin    # 钉住躯干，只让关节自由摆动
python view_mujoco.py --drop   # 落地对齐：消除脚底悬空
```

> 若本地 Python 没装 `mujoco`，用 1.2 / 1.3 的网页端即可，功能完全不受影响。

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

| 操作    | 作用                    |
| ----- | --------------------- |
| 左摇杆   | 前后左右移动（1.6 m/s）       |
| 右摇杆左右 | 30° 转向                |
| A / X | 把机器人摆到你正前方 2.6 m 并正对你 |
| B / Y | 世界复位（机器人回到世界原点）       |

技术细节：参考空间用 `local-floor`，头显里**你的脚踩的地面**就是 MuJoCo 的  
`z = 0` 地面。机器人身高约 1.43 m，站在离你 2.6 m 处，原地 1:1 比例观看，  
可以走近绕一圈看细节。

---

## 4. 界面功能

| 控件           | 说明                                          |
| ------------ | ------------------------------------------- |
| 机器人          | 显示 / 隐藏 30 个视觉网格（group 2）                   |
| 楼梯 / 障碍      | 显示 / 隐藏 `scene_bx.xml` 里的 15 级台阶            |
| 地面网格         | 显示 / 隐藏地板                                   |
| 阴影           | 开关实时阴影                                      |
| 碰撞体（调试图层）    | 显示 group 3 的 36 个碰撞几何（绿色半透明 capsule）        |
| 视角预设         | 正面 / 背面 / 侧面 / 头部特写 / 俯视 / 全景               |
| 运行 MuJoCo 物理 | 默认**关闭**；打开后真实跑 `mj_step`，机器人会因重力倒地（模型无控制器） |
| 复位姿态         | 回到落地对齐后的静止站立姿态                              |
| 复位世界位置       | 把世界平移 / 旋转复位                                |

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

| 脚本                  | 用途                                                   |
| ------------------- | ---------------------------------------------------- |
| `verify.mjs`        | **端到端自检**：加载 / 模型 / UI / 开关 / 视角 / 物理 / 复位 / XR，24 项 |
| `shoot.mjs`         | 用 CDP 等页面就绪后截图（可注入 JS 改相机、改可见性）                      |
| `dump-geoms.mjs`    | 列出每个视觉网格的世界 AABB，用于和 MuJoCo 对账                       |
| `make-manifest.mjs` | 重新生成 `assets/manifest.json`                          |
| `fetch-three.mjs`   | 下载 three.js 与 addons                                 |
| `selfsign.mjs`      | 纯 JS 生成自签证书（无 openssl 依赖，SAN 含本机所有内网 IP）             |
| `make-cert.mjs`     | 证书生成的封装                                              |
| `ref-render.py`     | 用 MuJoCo 官方 Python 渲染器出参考图（需另装 `mujoco`）             |

自检：

```bash
node tools/verify.mjs http://localhost:8080/
node tools/verify.mjs https://192.168.22.19:8443/   # 头显走的这条路
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

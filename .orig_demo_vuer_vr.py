"""MuJoCo VR 沉浸式观看（vuer + WebXR）—— elf3 人形机器人版。

PC 端运行 MuJoCo elf3 人形机器人物理仿真，通过 vuer 将 3D 场景实时推送到浏览器。
PICO 4 头显浏览器打开页面后点击 VR 按钮即可沉浸式观看。

用法:
  python demo_vuer_vr.py             # HTTPS 模式（供 PICO 头显访问，WebXR 必需）
  python demo_vuer_vr.py --no-ssl    # HTTP 模式（本机调试或 USB 直连头显）

首次运行会自动生成自签名证书 (vuer_cert.pem / vuer_key.pem)。
头显访问 https://<PC IP>:8012 时需手动接受证书警告。
"""
import asyncio
import datetime
import ipaddress
import math
import sys
import time
from pathlib import Path

import numpy as np
import mujoco
from vuer import Vuer, VuerSession
from vuer.schemas import (
    AmbientLight, Box, DirectionalLight, Grid, Group,
    Movable, OrbitControls, Plane, Scene, Stl,
)

# ---------------- 配置 ----------------
PC_IP = "192.168.22.19"  # 本机局域网 IP（头显需与 PC 同一 Wi-Fi）
PORT = 8012

CERT = Path(__file__).parent / "vuer_cert.pem"
KEY = Path(__file__).parent / "vuer_key.pem"
MESH_DIR = Path(__file__).parent / "meshes"
MODEL_XML = Path(__file__).parent / "elf3.xml"

# 场景整体沿 -Z 平移：WebXR 进入后用户出生在原点、面朝 -Z，
# 平移后用户站在正对机器人 2.5m 处，脚下有地板和网格参照。
SCENE_Z = -2.5

# PD 控制：跟踪初始关节角，保持站立姿态（增益需足以抵抗重力，
# 否则关节在重力下塌陷，机器人趴伏在地、零件角度错乱）
KP = 150.0
KD = 5.0

# ---------------- MuJoCo 模型 ----------------
model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

QPOS0 = data.qpos.copy()  # 初始站立姿态（自由基座位姿 + 关节角）

# elf3 顶层 torso_link 为自由浮动基座，占用前 7 个 qpos / 6 个 qvel
JOINT_QPOS_OFFSET = 7
JOINT_QVEL_OFFSET = 6
NU = model.nu  # 29 个电机


def collect_mesh_geoms():
    """收集 (geom_id, mesh_name)：每个 visual mesh geom 一项。

    用 geom 而非 body：MuJoCo 编译时把 mesh 对齐变换并入了 geom 相对位姿，
    STL 原始顶点必须配 geom 世界位姿（geom_xpos/geom_xmat）才能正确渲染，
    body 位姿缺少该补偿会导致部件错位、整体姿态错误。
    """
    items = []
    mesh_type = mujoco.mjtGeom.mjGEOM_MESH
    for gid in range(model.ngeom):
        if model.geom_type[gid] != mesh_type:
            continue
        if model.geom_group[gid] != 2:  # elf3 visual class
            continue
        mesh_id = int(model.geom_dataid[gid])
        if mesh_id < 0:
            continue
        name = model.mesh(mesh_id).name
        if not name:
            continue
        items.append((gid, name))
    return items


MESH_GEOMS = collect_mesh_geoms()
print(f"已收集 {len(MESH_GEOMS)} 个 visual mesh geom 用于渲染", flush=True)


# ---------------- 自签名证书 ----------------
def ensure_cert():
    if CERT.exists() and KEY.exists():
        return
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "mujoco-vr")])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now)
        .not_valid_after(now + datetime.timedelta(days=3650))
        .add_extension(
            x509.SubjectAlternativeName(
                [
                    x509.DNSName("localhost"),
                    x509.IPAddress(ipaddress.ip_address("127.0.0.1")),
                    x509.IPAddress(ipaddress.ip_address(PC_IP)),
                ]
            ),
            critical=False,
        )
        .sign(key, hashes.SHA256())
    )
    CERT.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    KEY.write_bytes(
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.TraditionalOpenSSL,
            serialization.NoEncryption(),
        )
    )
    print(f"已生成自签名证书: {CERT.name}, {KEY.name}")


# ---------------- 3D 场景元素 ----------------
# MuJoCo(Z-up) -> three.js(Y-up) 坐标系基变换矩阵：向量 (x,y,z) -> (x,z,-y)
_ZUP2YUP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])


def geom_matrix(gid):
    """构造 three.js 4x4 变换矩阵（列主序，供 VuerGroup matrix 属性）。

    旋转用相似变换 R' = M·geom_xmat·Mᵀ（与位置变换 (x,z,-y) 配套），
    经用户实测验证：此公式下各部件相对角度正确。直接变换 M·R 会破坏
    各部件朝向（尽管理论上 STL 在 Z-up 时应使用直接变换，实际以
    渲染效果为准）。

    用 matrix 属性绕过欧拉角万向锁问题（5 个部件撞上万向锁），
    VuerGroup 客户端 ce.matrix.fromArray(K), matrixAutoUpdate=false。
    """
    R = _ZUP2YUP @ data.geom_xmat[gid].reshape(3, 3) @ _ZUP2YUP.T
    px, py, pz = data.geom_xpos[gid].tolist()
    tx, ty, tz = px, pz, -py  # 位置 (x,y,z) -> (x,z,-y)
    return [
        R[0, 0], R[1, 0], R[2, 0], 0.0,
        R[0, 1], R[1, 1], R[2, 1], 0.0,
        R[0, 2], R[1, 2], R[2, 2], 0.0,
        tx,      ty,      tz,      1.0,
    ]


def mesh_group(gid, mesh_name, with_asset=False):
    """单个 visual mesh geom 的 Group 节点。

    位姿用 geom 世界位姿（geom_xpos/geom_xmat）：MuJoCo 编译时把 mesh
    对齐变换并入了 geom 相对位姿，STL 原始顶点必须配 geom 世界位姿，
    body 位姿会缺该补偿导致部件错位（表现为侧躺/散架）。

    用 matrix 属性（4x4 列主序）推送位姿，绕过欧拉角万向锁问题。
    with_asset=True：Group 内挂 Stl（含 src），仅初始化构建时用。
    with_asset=False：仅 matrix，用于每帧轻量更新。
    注意：不能每帧 upsert 带 src 的 Stl 节点——客户端会置空 src、
    中止并反复重载 STL（渲染卡顿 + 模型无法显示），因此几何体只建一次，
    每帧仅更新 Group 的 matrix。
    """
    props = dict(key=f"mesh-{gid}", matrix=geom_matrix(gid))
    if with_asset:
        props["children"] = [
            Stl(
                key=f"stl-{gid}",
                src=f"/workspace/{mesh_name}.STL",  # vuer 静态路由为 /workspace/
                material=dict(color=0xBBBBBB, metalness=0.5, roughness=0.5),
            )
        ]
    return Group(**props)


def robot_init():
    """初始机器人：所有 mesh geom 的 Group+Stl 挂在 robot 组下。

    Z-up->Y-up 变换已在 mesh_group 内逐节点完成（位置与矩阵均为
    three.js 语义），此组不再叠加旋转。
    """
    return Group(
        key="robot",
        position=[0, 0, 0],
        children=[mesh_group(g, n, with_asset=True) for g, n in MESH_GEOMS],
    )


def world_static():
    """静态环境（地板/网格）+ 机器人包在 world 组里。

    自动环绕模式每帧 upsert 旋转这个内层 world 组。
    注意：绝不能每帧 upsert Movable 节点本身——客户端会重挂载 Movable、
    其整个子树（地板+机器人）每帧销毁重建，导致 STL 反复中止加载
    （模型无法显示 + VR 严重卡顿）。只 upsert 内层普通 Group。
    """
    return Group(
        key="world",
        position=[0, 0, 0],
        rotation=[0, 0, 0],
        children=[
            Group(
                key="floor",
                children=[
                    Plane(args=[8, 8], rotation=[-math.pi / 2, 0, 0], material=dict(color=0xD8D8D8)),
                    Grid(key="ground-grid"),
                ],
            ),
            robot_init(),
        ],
    )


def world_transform(yaw):
    """只更新内层 world 组的旋转（不带 children，避免客户端重建子树）。"""
    return Group(key="world", rotation=[0, yaw, 0])


def robot_transforms():
    """机器人所有 mesh body 的位姿更新列表（批量 upsert，仅 Group 位姿）。"""
    return [mesh_group(g, n, with_asset=False) for g, n in MESH_GEOMS]


# ---------------- vuer 服务 ----------------
use_ssl = "--no-ssl" not in sys.argv
if use_ssl:
    ensure_cert()
    app = Vuer(
        host="0.0.0.0", port=PORT,
        cert=str(CERT), key=str(KEY),
        workspace=str(MESH_DIR),
    )
else:
    app = Vuer(host="0.0.0.0", port=PORT, workspace=str(MESH_DIR))

scheme = "https" if use_ssl else "http"

# ---------------- 全局交互状态 ----------------
state = {"mode": "auto", "yaw": 0.0, "pending_button": False}
BUTTON_KEY = "mode-button"


def button_box(position=None):
    """模式切换按钮：绿色=自动环绕，橙色=手动模式。"""
    color = 0x33AA55 if state["mode"] == "auto" else 0xE08030
    return Box(key=BUTTON_KEY, args=[0.28, 0.28, 0.06], position=position, material=dict(color=color))


async def on_click(event, session=None):
    """vuer 要求 handler 为 async（内部 create_task 需要协程），且调用时
    传入 (client_event, session_proxy) 两个参数；签名不匹配会导致
    WebSocket 断开、场景重置、STL 重新加载（表现为模型消失+卡顿）。"""
    state["mode"] = "manual" if state["mode"] == "auto" else "auto"
    if state["mode"] == "auto":
        state["yaw"] = 0.0
    state["pending_button"] = True
    print(f"[模式切换] -> {state['mode']} (event={getattr(event, 'etype', '?')})", flush=True)


for evt in ("ON_CLICK", "CLICK"):
    app.add_handler(evt, on_click)


async def event_logger(sess):
    """探测打印所有客户端事件类型（用于确认点击/手柄事件名）。"""
    seen = set()
    while True:
        ev = sess.popleft()
        if ev is not None:
            et = getattr(ev, "etype", "?")
            if et not in seen:
                seen.add(et)
                print(f"[新事件类型] {et}", flush=True)
        await asyncio.sleep(0.05)


@app.spawn(start=True)
async def main(sess: VuerSession):
    print("等待客户端连接...", flush=True)
    while getattr(sess, "CURRENT_WS_ID", None) is None:
        await asyncio.sleep(0.1)

    sess.set @ Scene(
        bgChildren=[
            AmbientLight(intensity=0.6),
            DirectionalLight(position=[3, 5, 2], intensity=1.0),
            # 网页端鼠标操作：左键拖动旋转视角、右键拖动平移、滚轮缩放
            # VR 模式下 OrbitControls 不干扰头显追踪（WebXR 接管相机）
            OrbitControls(key="orbit", makeDefault=True, enableDamping=True),
        ],
        children=[
            # Movable：手动模式下可用手柄扳机抓住"场景把手"拖动（等效传送）
            Movable(
                key="world-rig",
                position=[0, 0, SCENE_Z],
                showFrame=False,
                children=[world_static()],
            ),
            # 模式切换按钮：浮在用户出生点前方，不随场景移动。
            # 客户端 Primitive 原生支持 onClick -> ON_CLICK 事件，无需 Clickable 包裹
            #（该版本客户端无 Clickable 组件，使用会导致 R3F 报错、整个场景渲染失败）。
            button_box(position=[0, 1.0, -0.8]),
        ],
        camPosition=[0.8, 1.4, 0.6],
    )

    asyncio.get_event_loop().create_task(event_logger(sess))

    print(f"\n仿真已启动。本机预览: {scheme}://localhost:{PORT}", flush=True)
    if use_ssl:
        print(f"  - PICO 4 头显(Wi-Fi): {scheme}://{PC_IP}:{PORT}（需接受证书警告）", flush=True)
    else:
        print("  - PICO 4 头显(USB): 先执行 adb reverse tcp:8012 tcp:8012，", flush=True)
        print("    再在头显浏览器打开 http://localhost:8012 并点击 VR 按钮", flush=True)
        print("    注意: 头显直接访问 http://<PC IP> 无法进入 VR（WebXR 仅支持 localhost/HTTPS）", flush=True)
    print("", flush=True)

    # PD 控制目标：跟踪初始站立姿态的关节角（而非假定全 0）
    target = QPOS0[JOINT_QPOS_OFFSET:JOINT_QPOS_OFFSET + NU].copy()
    ctrl_lo = model.actuator_ctrlrange[:, 0]
    ctrl_hi = model.actuator_ctrlrange[:, 1]

    last = time.perf_counter()
    frame = 0
    while True:
        await asyncio.sleep(1 / 60)
        now = time.perf_counter()
        dt = min(now - last, 0.05)
        last = now
        # 按真实时间推进物理（实时仿真）
        nsub = max(1, round(dt / model.opt.timestep))
        for _ in range(nsub):
            # 观看演示：把自由基座钉回初始位姿。双足无平衡控制器，
            # 自由基座会在重力下整体倾倒（表现为趴伏在地、姿态散乱）
            data.qpos[:JOINT_QPOS_OFFSET] = QPOS0[:JOINT_QPOS_OFFSET]
            data.qvel[:JOINT_QVEL_OFFSET] = 0.0
            q = data.qpos[JOINT_QPOS_OFFSET:JOINT_QPOS_OFFSET + NU]
            qd = data.qvel[JOINT_QVEL_OFFSET:JOINT_QVEL_OFFSET + NU]
            ctrl = KP * (target - q) - KD * qd
            ctrl = np.clip(ctrl, ctrl_lo, ctrl_hi)
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)  # 确保 xpos/xquat 最新
        # 自动模式：场景匀速环绕（每 30 秒一圈）；手动模式：停发环绕，用户可拖动场景
        if state["mode"] == "auto":
            state["yaw"] += dt * 2 * math.pi / 30.0
        try:
            if state["mode"] == "auto":
                sess.upsert @ world_transform(state["yaw"])
            if state.pop("pending_button", False):
                sess.upsert @ button_box(position=[0, 1.0, -0.8])
            # 单条批量消息更新全部 mesh 位姿（逐条推送会因消息量过大导致 VR 卡顿）
            sess.upsert @ robot_transforms()
        except AssertionError:
            pass  # 客户端未连接/已断开，跳过本帧，循环继续
        frame += 1
        if frame % 300 == 0:
            print(f"心跳: t={data.time:.1f}s yaw={state['yaw']:+.2f} mode={state['mode']}", flush=True)

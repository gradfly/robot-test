"""MuJoCo VR 沉浸式交互（vuer + WebXR）—— elf3 人形机器人版。

PC 端运行 MuJoCo elf3 人形机器人物理仿真，通过 vuer 将 3D 场景实时推送到浏览器。
PICO 4 头显浏览器打开页面后点击 VR 按钮即可沉浸式观看并与场景交互。

用法:
  python demo_vuer_vr.py                  # HTTP 模式（默认，本地托管客户端，推荐）
  python demo_vuer_vr.py --ssl            # HTTPS 模式（自签名证书，本机调试备选）

机器人位姿调试开关（只影响机器人部件，方块/手柄不受影响）:
  --raw-pose        屏蔽机器人所有角度/坐标变换，直接透传 MuJoCo 原始位姿（默认）
  --identity-pose   屏蔽所有部件旋转，仅保留位置（位置按 Y-up 换算）
  --converted       恢复正常的 MuJoCo(Z-up) -> three.js(Y-up) 换算

访问方式（vuer 0.1.6 有两套入口，本地服务器同时提供客户端页面和 WebSocket）:
  A. 本地托管（推荐，无需外网）:
       本机预览   http://localhost:8012
       头显访问   http://<PC IP>:8012      然后点击页面上的 VR 按钮
  B. 远程客户端（需外网）:
       https://vuer.ai?ws=ws://<PC IP>:8012

注意: WebXR 要求 secure context。浏览器把 localhost / 127.0.0.1 视为安全来源，
所以本机预览用 http 即可点开 VR；但头显通过局域网 IP 访问时属于非安全来源，
需要要么用 HTTPS（--ssl，需手动信任自签名证书），要么在头显端做端口转发使
地址成为 localhost（adb reverse tcp:8012 tcp:8012）。
若本机设置了 HTTP_PROXY/HTTPS_PROXY，请把 localhost / <PC IP> 加入代理例外，
否则本机回环请求会被代理拦截（表现为 502 Bad Gateway）。

----------------------------------------------------------------------------
坐标系与姿态说明（问题 1 修复要点）
----------------------------------------------------------------------------
1. MuJoCo 使用 Z-up 右手系，three.js（vuer 客户端）使用 Y-up。向量换算为
   (x, y, z) -> (x, z, -y)，即绕 X 轴 -90°：
       M = [[1,0,0],[0,0,1],[0,-1,0]]
2. 本模型（elf3.xml）的 STL 是「局部坐标」网格：每个 mesh 的对齐变换在编译时
   被 MuJoCo 烘焙进了 geom 的相对位姿（geom_pos/geom_quat）。因此渲染时
   必须使用 geom 的 **世界位姿** data.geom_xpos / data.geom_xmat，而不能用
   body 位姿——否则会缺少 mesh 对齐补偿，表现为部件错位、姿态散乱。
3. 旋转用相似变换 R_world' = M · R · Mᵀ（与位置换算 (x,z,-y) 配套）。
   由于 M 是纯旋转（det=+1），该式与直接 R' = R·Mᵀ 在数学上等价；
   本实现统一用相似变换，保证与世界位置换算同源。
4. 万向节锁死：欧拉角在特定朝向会丢失一个自由度（本模型 5 个部件会撞上）。
   因此 Group 节点一律用 4x4 矩阵属性（列主序）而非 rotation 欧拉角。
5. 站立：模型脚底 capsule 底面在 z=0.0375，而地面在 z=0，即初始悬空
   3.75cm。本脚本把自由基座下移该偏移，使双脚恰好落地；并对关节施加
   PD 姿态保持，同时把自由基座钉在落地姿态（双足无平衡控制器，自由
   基座必在 1~3s 内倾倒）。
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
from vuer.events import HapticActuatorPulse
from vuer.schemas import (
    AmbientLight, Box, DirectionalLight, Grid, Group,
    MotionControllers, Movable, OrbitControls, Plane, Scene, Stl, Text,
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

# PD 控制：跟踪初始关节角，保持站立姿态。
# 实测：钉住自由基座时 kp=150 / kd=5 即可把关节误差压到 0.02 rad 内。
KP = 150.0
KD = 5.0

# ---------------- MuJoCo 模型 ----------------
model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# 自由基座占用前 7 个 qpos / 6 个 qvel
JOINT_QPOS_OFFSET = 7
JOINT_QVEL_OFFSET = 6
NU = model.nu  # 29 个电机


def foot_clearance():
    """脚底 capsule 底面与地面(z=0)的间隙——初始姿态下脚是悬空的。"""
    z = 1e9
    for g in range(model.ngeom):
        name = model.geom(g).name or ""
        if name.startswith(("l_foot", "r_foot")):
            z = min(z, float(data.geom_xpos[g][2] - model.geom_size[g][0]))
    return 0.0 if z > 1e8 else z


# 落地：把自由基座下移，让双脚恰好踩在地面上
FOOT_GAP = foot_clearance()
QPOS0 = data.qpos.copy()
QPOS0[2] -= FOOT_GAP
data.qpos[:] = QPOS0
mujoco.mj_forward(model, data)
print(f"脚底悬空 {FOOT_GAP:.5f} m，已把基座下移至落地 (base z={QPOS0[2]:.5f})", flush=True)


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


# ---------------- 坐标系变换 ----------------
# MuJoCo(Z-up) -> three.js(Y-up)：向量 (x,y,z) -> (x,z,-y)
_ZUP2YUP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])

# ---------------------------------------------------------------------------
# 调试开关：机器人部件位姿处理模式（只影响机器人，不影响方块 / 手柄）
# ---------------------------------------------------------------------------
#   "raw"       原始状态——完全透传 MuJoCo 的 pose：
#               不做 Z-up -> Y-up 换算、不做相似变换、位置也不换轴。
#               部件之间保持 MuJoCo 里的真实相对关系（整机会呈 Z-up 姿态
#               出现在 Y-up 场景里，即整体相当于绕 X 轴转了 -90°）。
#   "identity"  屏蔽旋转——位置按 Y-up 换算，但旋转置为单位矩阵。
#               用于只看部件的「位置摆位」是否正确，所有部件都不带角度。
#   "converted" 正常模式——MuJoCo(Z-up) -> three.js(Y-up) 完整换算。
# 命令行: --raw-pose / --identity-pose / --converted
_pose_cli = [a for a in sys.argv if a in ("--raw-pose", "--identity-pose", "--converted")]
if "--raw-pose" in _pose_cli:
    POSE_MODE = "raw"
elif "--identity-pose" in _pose_cli:
    POSE_MODE = "identity"
else:
    POSE_MODE = "converted"   # 默认恢复正常 Z-up -> Y-up 换算，机器人直立显示


def matrix_from_pose(pos, rot):
    """把 MuJoCo 世界位姿 (pos, rot 3x3) 转成 three.js 4x4 列主序矩阵。

    行为由 POSE_MODE 决定，见上方调试开关说明。
    """
    if POSE_MODE == "raw":
        # 原始透传：位置与姿态均不做任何换算
        R = np.asarray(rot, dtype=float)
        tx, ty, tz = (float(pos[0]), float(pos[1]), float(pos[2]))
    elif POSE_MODE == "identity":
        # 屏蔽旋转：位置换算到 Y-up，姿态强制单位矩阵
        R = np.eye(3)
        px, py, pz = pos
        tx, ty, tz = float(px), float(pz), -float(py)
    else:
        # 正常换算：相似变换 + 位置换轴
        R = _ZUP2YUP @ rot @ _ZUP2YUP.T
        px, py, pz = pos
        tx, ty, tz = float(px), float(pz), -float(py)
    return [
        float(R[0, 0]), float(R[1, 0]), float(R[2, 0]), 0.0,
        float(R[0, 1]), float(R[1, 1]), float(R[2, 1]), 0.0,
        float(R[0, 2]), float(R[1, 2]), float(R[2, 2]), 0.0,
        tx,             ty,             tz,             1.0,
    ]


def geom_matrix(gid):
    """单个 visual mesh geom 的 three.js 变换矩阵（列主序）。

    用 matrix 属性绕过欧拉角万向锁（client: matrix.fromArray + matrixAutoUpdate=false）。
    位姿换算方式由 POSE_MODE 控制（raw / identity / converted）。
    """
    rot = data.geom_xmat[gid].reshape(3, 3)
    return matrix_from_pose(data.geom_xpos[gid].tolist(), rot)


def quat_to_matrix(q):
    """MuJoCo 四元数 (w, x, y, z) -> 3x3 旋转矩阵。"""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=np.float64))
    return R.reshape(3, 3)


def mesh_offset_matrix(mid):
    """返回 raw STL 顶点在 vuer/three.js 中所需的坐标系修正矩阵。

    MuJoCo 使用 Z-up 右手系，three.js（vuer 客户端）使用 Y-up。vuer 的 Stl
    组件把原始 STL 顶点坐标直接当作 three.js 的 Y-up 数值渲染，但这些数值
    在 MuJoCo 语义下其实是 Z-up 坐标。

    关键发现：raw STL 顶点已经等于 mesh_pos + mesh_quat * mesh_vert，即
    mesh 对齐偏移在 MuJoCo 编译时已经"烘焙"进 STL 文件本身。因此在 vuer 中
    渲染时不需要再补偿 mesh_quat/mesh_pos，只需要把 Y-up 数值解释回 Z-up
    即可。该修正就是固定的 Z-up -> Y-up 旋转（绕 X 轴 -90°）。

    参数 `mid` 保留仅为了保持调用签名统一，实际不再使用。
    """
    if POSE_MODE == "raw":
        # 原始透传：STL 顶点按 Z-up 数值解释
        R = np.eye(3)
        t = np.zeros(3)
    else:
        # converted / identity：把 Y-up 数值先转回 Z-up，再由外层 geom 矩阵做
        # 完整 Z-up -> Y-up 世界变换
        R = _ZUP2YUP
        t = np.zeros(3)

    return [
        float(R[0, 0]), float(R[1, 0]), float(R[2, 0]), 0.0,
        float(R[0, 1]), float(R[1, 1]), float(R[2, 1]), 0.0,
        float(R[0, 2]), float(R[1, 2]), float(R[2, 2]), 0.0,
        float(t[0]),    float(t[1]),    float(t[2]),    1.0,
    ]


def translation_matrix(pos, scale=1.0):
    """仅平移+均匀缩放的 4x4 列主序矩阵。"""
    s = float(scale)
    return [
        s, 0.0, 0.0, 0.0,
        0.0, s, 0.0, 0.0,
        0.0, 0.0, s, 0.0,
        float(pos[0]), float(pos[1]), float(pos[2]), 1.0,
    ]


def mesh_group(gid, mesh_name, with_asset=False):
    """单个 visual mesh geom 的 Group 节点。

    结构：外层 Group（geom 世界位姿） -> 中层 Group（坐标系修正） -> Stl。
    之所以要中层 Group，是因为 vuer 的 Stl 组件把原始 STL 顶点当作 three.js
    的 Y-up 数值使用，而这些数值在 MuJoCo 语义下是 Z-up 坐标；中层负责把
    Y-up 数值先转回 Z-up，再由外层做完整的 Z-up -> Y-up 世界变换。

    with_asset=True：中层 Group 内挂 Stl（含 src），仅初始化构建时用。
    with_asset=False：仅更新外层 Group 的 matrix，用于每帧轻量刷新。
    """
    mesh_id = int(model.geom_dataid[gid])
    outer_props = dict(key=f"mesh-{gid}", matrix=geom_matrix(gid))
    if with_asset:
        outer_props["children"] = [
            Group(
                key=f"mesh-offset-{gid}",
                matrix=mesh_offset_matrix(mesh_id),
                matrixAutoUpdate=False,
                children=[
                    Stl(
                        key=f"stl-{gid}",
                        src=f"/workspace/{mesh_name}.STL",  # vuer 静态路由为 /workspace/
                        material=dict(color=0xBBBBBB, metalness=0.5, roughness=0.5),
                    )
                ],
            )
        ]
    return Group(**outer_props)


def robot_init():
    """初始机器人：所有 mesh geom 的 Group+Stl 挂在 robot 组下。"""
    return Group(
        key="robot",
        position=[0, 0, 0],
        children=[mesh_group(g, n, with_asset=True) for g, n in MESH_GEOMS],
    )


def robot_transforms():
    """机器人所有 mesh geom 的位姿更新列表（批量 upsert，仅 Group 位姿）。"""
    return [mesh_group(g, n, with_asset=False) for g, n in MESH_GEOMS]


def print_pose_diagnostic():
    """启动时打印各部件位姿，便于对照 POSE_MODE 的效果。

    同时验证一件事：部件之间的相对距离在 raw / converted 下必须完全相同
    （换轴是刚体变换，不改部件间距）。若两者不一致，说明问题不在换算本身。
    """
    print(f"\n[位姿模式] POSE_MODE = {POSE_MODE}", flush=True)
    if POSE_MODE == "raw":
        print("  -> 已屏蔽机器人所有角度/坐标变换，直接透传 MuJoCo 原始位姿。", flush=True)
        print("     整机会以 Z-up 姿态出现在 Y-up 场景中（视觉上相当于整体倒下 90°），", flush=True)
        print("     但部件之间的相对关系与 MuJoCo 完全一致。", flush=True)
    elif POSE_MODE == "identity":
        print("  -> 已屏蔽所有部件旋转，仅保留位置（位置按 Y-up 换算）。", flush=True)
    else:
        print("  -> 正常换算 MuJoCo(Z-up) -> three.js(Y-up)。", flush=True)

    raws = []
    for g, n in MESH_GEOMS:
        p = data.geom_xpos[g]
        raws.append((n, float(p[0]), float(p[1]), float(p[2])))

    print(f"\n  部件原始 geom_xpos（MuJoCo Z-up，共 {len(raws)} 个）:", flush=True)
    print(f"    {'名称':<28s} {'X':>9s} {'Y':>9s} {'Z':>9s}", flush=True)
    for n, x, y, z in raws:
        print(f"    {n:<28s} {x:>9.4f} {y:>9.4f} {z:>9.4f}", flush=True)

    # 相对距离不变量：raw 与 converted 下必须一致
    def pairwise(mode):
        import numpy as _np
        pts = []
        for g, n in MESH_GEOMS:
            p = data.geom_xpos[g]
            if mode == "raw":
                pts.append([p[0], p[1], p[2]])
            else:
                pts.append([p[0], p[2], -p[1]])
        pts = _np.array(pts)
        k = min(6, len(pts))
        return [round(float(_np.linalg.norm(pts[i] - pts[j])), 6)
                for i in range(k) for j in range(i + 1, k)]

    same = pairwise("raw") == pairwise("converted")
    print(f"\n  换轴不变量检查：raw 与 converted 的部件间距 {'一致 ✓（换轴是刚体变换）' if same else '不一致 ✗（换算有问题）'}", flush=True)
    print(flush=True)


# ---------------- 交互方块 ----------------
CUBE_KEY = "green-cube"
CUBE_SIZE = 0.14                     # 半边尺寸
CUBE_COLORS = [0x33CC66, 0xFF6B3D, 0x3D8BFF, 0xFFD23D]

# 每个方块各占一个位置：left=左手方块（扳机抓取），right=右手方块（缩放/变色）
LEFT_CUBE_POS = [0.55, 1.15, -0.55]
RIGHT_CUBE_POS = [-0.55, 1.15, -0.55]
cube_state = {
    "left": {"pos": list(LEFT_CUBE_POS), "scale": 1.0, "held": False},
    "right": {"pos": list(RIGHT_CUBE_POS), "scale": 1.0, "held": False},
    "color": 0,
    "hint": "扳机=抓取/放下  A/B(X/Y)=变色/复位  摇杆=推远拉近",
}


def cube_node(which):
    """可交互方块。

    用普通 Group + matrix（而非 Movable）：Movable 在 VR 下的抓取依赖客户端
    内部 raycast 选中 + squeeze，行为不确定；这里改为「服务端驱动」——由
    手柄矩阵直接算出方块位姿并 push，交互可靠、可预期。
    """
    st = cube_state[which]
    color = CUBE_COLORS[cube_state["color"] % len(CUBE_COLORS)]
    m = translation_matrix(st["pos"], scale=st["scale"])
    return Group(
        key=f"{CUBE_KEY}-{which}",
        matrix=m,
        children=[
            Box(
                key=f"{CUBE_KEY}-{which}-geom",
                args=[CUBE_SIZE, CUBE_SIZE, CUBE_SIZE],
                material=dict(color=color, metalness=0.2, roughness=0.4),
            )
        ],
    )


def cube_transforms():
    return [cube_node("left"), cube_node("right")]


# ---------------- VR 手柄显示 ----------------
# MotionControllers 只上报位姿/按键，不渲染模型（组件返回 null），
# 因此这里自己画一对手柄：手柄主体 + 一条指向光标（ray）。
CONTROLLER_KEYS = ["left", "right"]
CONTROLLER_COLORS = {"left": 0x33AAFF, "right": 0xFF5533}
CONTROLLER_SCALE = 0.09
GRAB_REACH = 0.22   # 方块吸附在手柄前方 22cm 处


def controller_node(side, matrix=None, connected=False):
    """手柄模型：未连接时半透明暗淡，连接后点亮。"""
    color = CONTROLLER_COLORS[side]
    mat = matrix if matrix is not None else IDENTITY16
    opacity = 1.0 if connected else 0.25
    return Group(
        key=f"controller-{side}",
        matrix=mat,
        children=[
            # 握把：小立方体
            Box(
                key=f"controller-{side}-grip",
                args=[CONTROLLER_SCALE, CONTROLLER_SCALE * 0.7, CONTROLLER_SCALE * 1.6],
                material=dict(color=color, metalness=0.3, roughness=0.5,
                              transparent=True, opacity=opacity),
            ),
            # 前伸的指向光条（沿 -Z，WebXR targetRay 方向）
            Box(
                key=f"controller-{side}-ray",
                args=[0.006, 0.006, 0.5],
                position=[0, 0, -0.25],
                material=dict(color=color, metalness=0.0, roughness=1.0,
                              transparent=True, opacity=0.5 * opacity),
            ),
        ],
    )


IDENTITY16 = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


# ---------------- 世界（地板/网格/机器人/方块/手柄）----------------
def world_static():
    """静态环境（地板/网格）+ 机器人 + 交互方块 + 手柄，包在 world 组里。

    每帧只 upsert 内层普通 Group（不 upsert Movable 本身），
    避免客户端重挂载整棵子树导致 STL 反复重载。
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
            cube_node("left"),
            cube_node("right"),
            controller_node("left"),
            controller_node("right"),
        ],
    )


def world_transform(yaw):
    return Group(key="world", rotation=[0, yaw, 0])


# 启动即打印位姿诊断（不依赖客户端连接，便于直接在终端查看原始状态）
print_pose_diagnostic()


# ---------------- vuer 服务 ----------------
# vuer 0.1.6 的本地服务器在根路径 / 直接托管 WebXR 客户端页面，并在同一端口
# 提供 WebSocket；因此默认用 HTTP 即可（本机 localhost 属安全来源，可直接进 VR）。
# 只有让头显通过局域网 IP 直连、且不想做端口转发时，才需要 --ssl 走 HTTPS。
use_ssl = "--ssl" in sys.argv
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
ws_scheme = "wss" if use_ssl else "ws"

# ---------------- 全局交互状态 ----------------
state = {
    "mode": "auto",
    "yaw": 0.0,
    "pending_button": False,
    "dirty_cubes": False,
    "ctrls": {"left": None, "right": None},   # 最近一次手柄位姿矩阵
    "conn": {"left": False, "right": False},
    "grab": {"left": False, "right": False},  # 上一帧扳机状态（边沿检测）
    "stick": {"left": (0.0, 0.0), "right": (0.0, 0.0)},
    "haptic": [],
}
BUTTON_KEY = "mode-button"


def button_box(position=None):
    """模式切换按钮：绿色=自动环绕，橙色=手动模式。"""
    color = 0x33AA55 if state["mode"] == "auto" else 0xE08030
    return Box(key=BUTTON_KEY, args=[0.28, 0.28, 0.06], position=position, material=dict(color=color))


def hint_box():
    """提示文字。客户端 Text 组件从 children 读取字符串内容（非 text 属性）。"""
    return Text(
        cube_state["hint"],
        key="hint",
        position=[0, 1.9, -1.4],
        fontSize=0.045,
        color="white",
        anchorX="center",
        anchorY="middle",
    )


async def on_click(event, session=None):
    """vuer 要求 handler 为 async，且调用时传入 (client_event, session_proxy)。"""
    state["mode"] = "manual" if state["mode"] == "auto" else "auto"
    if state["mode"] == "auto":
        state["yaw"] = 0.0
    state["pending_button"] = True
    print(f"[模式切换] -> {state['mode']}", flush=True)


for evt in ("ON_CLICK", "CLICK"):
    app.add_handler(evt, on_click)


def _controller_value(event):
    """从 CONTROLLER_MOVE 事件取 value（容错各种字段形态）。"""
    return getattr(event, "value", None) or {}


async def on_controller_move(event, session=None):
    """VR 手柄位姿 + 按键回调（etype = CONTROLLER_MOVE）。

    value 结构（vuer 客户端 getState）：
      value.left  / value.right        : 16 元素列主序位姿矩阵
      value.leftState / value.rightState:
          {trigger, squeeze, touchpad, thumbstick, aButton, bButton,
           triggerValue, squeezeValue, touchpadValue[2], thumbstickValue[2]}
    """
    val = _controller_value(event)
    if not isinstance(val, dict):
        return
    for side in ("left", "right"):
        pose = val.get(side)
        st = val.get(f"{side}State") or {}
        if pose:
            state["ctrls"][side] = [float(x) for x in pose]
            state["conn"][side] = True
        else:
            state["conn"][side] = False
        if st:
            state["conn"][side] = True
            # 摇杆 -> 方块推远拉近
            ts = st.get("thumbstickValue") or [0.0, 0.0]
            state["stick"][side] = (float(ts[0]), float(ts[1]))
            handle_button_edges(side, st)


def handle_button_edges(side, st):
    """把按键「按下沿」翻译成交互动作。

    抓取模型：按扳机 -> 方块吸附到手柄前方（held）；保持扳机 -> 方块跟随
    手柄移动；松开扳机 -> 方块留在当前世界位置。这套逻辑完全由服务端驱动，
    不依赖客户端 raycast 选中，因此在 PICO 等设备上行为确定。
    """
    trigger = bool(st.get("trigger"))
    squeeze = bool(st.get("squeeze"))
    a_btn = bool(st.get("aButton"))
    b_btn = bool(st.get("bButton"))
    target = "left" if side == "left" else "right"

    # 扳机：按下抓取 / 松开释放（边沿检测）
    if trigger and not state["grab"][side]:
        state["grab"][side] = True
        cube_state[target]["held"] = True
        state["dirty_cubes"] = True
        state["haptic"].append(side)  # 触发触觉反馈脉冲
        print(f"[{side}] 抓取方块", flush=True)
    elif not trigger and state["grab"][side]:
        state["grab"][side] = False
        cube_state[target]["held"] = False
        state["dirty_cubes"] = True
        print(f"[{side}] 释放方块", flush=True)

    # squeeze（握柄）：立即放下 / 复位到初始位
    if squeeze and cube_state[target]["held"]:
        cube_state[target]["held"] = False
        state["grab"][side] = False
        state["dirty_cubes"] = True

    # A / B（左手柄常映射为 X / Y）：A=切换颜色，B=尺寸复位
    if a_btn:
        cube_state["color"] = (cube_state["color"] + 1) % len(CUBE_COLORS)
        state["dirty_cubes"] = True
    if b_btn:
        for k in ("left", "right"):
            cube_state[k]["scale"] = 1.0
        state["dirty_cubes"] = True


def controller_tip_position(side):
    """从手柄 4x4 列主序矩阵取「指尖」世界位置（沿 -Z 前伸 grab_dist）。

    three.js 矩阵列主序：a12,a13,a14 为平移；-Z 方向 = -(a8,a9,a10)。
    """
    m = state["ctrls"][side]
    if not m:
        return None
    fwd = np.array([-m[8], -m[9], -m[10]])
    n = np.linalg.norm(fwd)
    if n > 1e-6:
        fwd = fwd / n
    pos = np.array([m[12], m[13], m[14]])
    return pos + fwd * GRAB_REACH


def update_grabbed_cubes():
    """被抓住的方块跟随对应手柄（服务端驱动）。"""
    for side in ("left", "right"):
        if not cube_state[side]["held"]:
            continue
        tip = controller_tip_position(side)
        if tip is not None:
            cube_state[side]["pos"] = [float(tip[0]), float(tip[1]), float(tip[2])]


app.add_handler("CONTROLLER_MOVE", on_controller_move)


async def event_logger(sess):
    """探测打印所有客户端事件类型（便于确认手柄事件名）。"""
    seen = set()
    while True:
        ev = sess.popleft()
        if ev is not None:
            et = getattr(ev, "etype", "?")
            if et not in seen:
                seen.add(et)
                print(f"[新事件类型] {et}", flush=True)
        await asyncio.sleep(0.05)


async def _run_session(sess: VuerSession):
    # spawn handler 被调用时连接已建立，CURRENT_WS_ID 必定有值；
    # 这里保留一个短暂等待，兼容极少数事件早于连接注册到达的情况。
    for _ in range(50):
        if getattr(sess, "CURRENT_WS_ID", None) is not None:
            break
        await asyncio.sleep(0.1)

    sess.set @ Scene(
        bgChildren=[
            AmbientLight(intensity=0.6),
            DirectionalLight(position=[3, 5, 2], intensity=1.0),
            # 手柄位姿 + 按键事件流（trigger/squeeze/aButton/bButton/thumbstick）
            MotionControllers(key="motion-controllers", stream=True, left=True, right=True),
            # 网页端鼠标操作；VR 模式下 OrbitControls 不干扰头显追踪
            OrbitControls(key="orbit", makeDefault=True, enableDamping=True),
        ],
        children=[
            Movable(
                key="world-rig",
                position=[0, 0, SCENE_Z],
                showFrame=False,
                children=[world_static()],
            ),
            button_box(position=[0, 1.0, -0.8]),
            hint_box(),
        ],
        camPosition=[0.8, 1.4, 0.6],
    )

    asyncio.get_event_loop().create_task(event_logger(sess))

    print(f"\n仿真已启动。", flush=True)
    print(f"  本机预览   {scheme}://localhost:{PORT}", flush=True)
    print(f"  头显访问   {scheme}://{PC_IP}:{PORT}   （点页面上的 VR 按钮进沉浸模式）", flush=True)
    print(f"  远程客户端 https://vuer.ai?ws={ws_scheme}://{PC_IP}:{PORT}   （需外网）", flush=True)
    if not use_ssl:
        print("  提示: 头显若因非安全来源无法进 VR，可改跑 --ssl，或执行", flush=True)
        print(f"        adb reverse tcp:{PORT} tcp:{PORT} 后用 http://localhost:{PORT}", flush=True)
    else:
        print("  提示: HTTPS 使用自签名证书，头显首次访问需手动信任该证书。", flush=True)
    print("  手柄操作: 扳机=抓取方块, A/B(X/Y)=变色/复位, 摇杆=推远拉近\n", flush=True)

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

        # ---- 交互：被抓住的方块跟手；摇杆推远/拉近 ----
        update_grabbed_cubes()
        for side in ("left", "right"):
            ax, ay = state["stick"][side]
            if abs(ay) > 0.15:
                tgt = "left" if side == "left" else "right"
                new_scale = cube_state[tgt]["scale"] * (1.0 - 0.6 * ay * dt)
                cube_state[tgt]["scale"] = float(np.clip(new_scale, 0.3, 3.0))
                state["dirty_cubes"] = True

        # ---- 物理：钉基座 + 关节 PD 保持站立姿态 ----
        nsub = max(1, round(dt / model.opt.timestep))
        for _ in range(nsub):
            data.qpos[:JOINT_QPOS_OFFSET] = QPOS0[:JOINT_QPOS_OFFSET]
            data.qvel[:JOINT_QVEL_OFFSET] = 0.0
            q = data.qpos[JOINT_QPOS_OFFSET:JOINT_QPOS_OFFSET + NU]
            qd = data.qvel[JOINT_QVEL_OFFSET:JOINT_QVEL_OFFSET + NU]
            ctrl = KP * (target - q) - KD * qd
            data.ctrl[:] = np.clip(ctrl, ctrl_lo, ctrl_hi)
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

        # ---- 自动模式：场景匀速环绕 ----
        if state["mode"] == "auto":
            state["yaw"] += dt * 2 * math.pi / 30.0

        # ---- 触觉反馈：抓取瞬间给手柄一个脉冲 ----
        for side in state["haptic"]:
            try:
                pulse = HapticActuatorPulse(
                    left={"strength": 0.6, "duration": 60} if side == "left" else None,
                    right={"strength": 0.6, "duration": 60} if side == "right" else None,
                )
                sess.send @ pulse
            except Exception:
                pass
        state["haptic"].clear()

        # ---- 推送场景 ----
        msgs = []
        try:
            if state["mode"] == "auto":
                msgs.append(world_transform(state["yaw"]))
            if state.pop("pending_button", False):
                msgs.append(button_box(position=[0, 1.0, -0.8]))
            msgs.extend(robot_transforms())
            # 手柄：位姿实时更新
            for side in ("left", "right"):
                msgs.append(controller_node(side, state["ctrls"][side], state["conn"][side]))
            # 方块：抓取中每帧更新；否则仅在状态变化时更新
            if state.pop("dirty_cubes", False) or state["grab"]["left"] or state["grab"]["right"]:
                msgs.extend(cube_transforms())
            sess.upsert @ msgs
        except AssertionError:
            pass  # 客户端未连接/已断开，跳过本帧
        except Exception as exc:  # 容错，避免单帧异常终止整个仿真
            if frame % 120 == 0:
                print(f"[warn] 推送失败: {exc}", flush=True)

        frame += 1
        if frame % 300 == 0:
            sess_conn = f"L={'Y' if state['conn']['left'] else 'n'} R={'Y' if state['conn']['right'] else 'n'}"
            print(f"心跳: t={data.time:.1f}s yaw={state['yaw']:+.2f} mode={state['mode']} {sess_conn}", flush=True)


@app.spawn(start=True)
async def main(sess: VuerSession):
    print("等待客户端连接...", flush=True)
    try:
        await _run_session(sess)
    except Exception as exc:
        import traceback
        print(f"[fatal] spawn handler 异常: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()



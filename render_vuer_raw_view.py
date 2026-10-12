"""用 MuJoCo 渲染器模拟 vuer raw 模式下的默认视角。

vuer raw 模式：直接把 MuJoCo (x,y,z) 当成 three.js (x,y,z)。
three.js 默认相机大概在 (0, 2, 3) 看向原点，Y 向上。
对应到 MuJoCo 坐标系：
  three.js (x,y,z) -> MuJoCo (x, -z, y)
  相机位置        -> (0, -3, 2)
  向上方向        -> (0, 0, -1)
"""

from pathlib import Path
import numpy as np
import mujoco
from PIL import Image

CWD = Path(__file__).resolve().parent

model = mujoco.MjModel.from_xml_path(str(CWD / "elf3.xml"))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# 落地对齐：基座下移 0.0375
data.qpos[2] -= 0.0375
mujoco.mj_forward(model, data)

model.vis.global_.offwidth = 1280
model.vis.global_.offheight = 720
renderer = mujoco.Renderer(model, height=720, width=1280)

opt = mujoco.MjvOption()
mujoco.mjv_defaultOption(opt)
for g in range(6):
    opt.geomgroup[g] = 0
opt.geomgroup[2] = 1
opt.geomgroup[0] = 1

# 构造一个与 vuer raw 模式默认相机等价的相机
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.type = mujoco.mjtCamera.mjCAMERA_FREE

# vuer 默认相机大致在 (0, 2, 3)，看向原点，Y 向上
cam.lookat[:] = [0.0, 0.0, 0.0]
# three.js (0,2,3) -> MuJoCo (0, -3, 2)
cam.distance = np.linalg.norm([0.0, -3.0, 2.0])
cam.azimuth = np.degrees(np.arctan2(-3.0, 2.0)) + 90  # 从 +Z 起算的方位
# 更简单：直接设置相机的位置/朝向 through fixed camera? 用 mjv_camera 的 lookat + distance + azimuth/elevation
# elevation 是相对于水平面的角度。相机在 (0,-3,2)：水平距离 3，高度 2
cam.elevation = np.degrees(np.arctan2(2.0, 3.0))
cam.azimuth = 180.0  # 从 +Y 方向看？在 MuJoCo azimuth 0 = 从 +Y 看；180 = 从 -Y 看

# 向上方向：three.js Y+ = MuJoCo Z-
renderer.update_scene(data, camera=cam, scene_option=opt)
img = renderer.render()
out = CWD / "render" / "vuer_raw_like.png"
Image.fromarray(img).save(str(out))
print(f"saved {out}")

# 再渲染一个从侧面（MuJoCo Y 轴）看过去的图，显示机器人其实是连着的
cam2 = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam2)
cam2.type = mujoco.mjtCamera.mjCAMERA_FREE
cam2.lookat[:] = [0.0, 0.0, 0.85]
cam2.distance = 3.0
cam2.azimuth = 90.0   # 从 +X 看
cam2.elevation = 0.0
renderer.update_scene(data, camera=cam2, scene_option=opt)
img2 = renderer.render()
out2 = CWD / "render" / "vuer_raw_side.png"
Image.fromarray(img2).save(str(out2))
print(f"saved {out2}")

renderer.close()

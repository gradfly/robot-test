"""离线渲染 elf3 机器人初始位形，用于核对部件角度。

用 mujoco.Renderer 从多个视角渲染，保存 PNG。
只渲染 group 2 的视觉 mesh（与 vuer 场景里显示的一致）。
"""

import argparse
from pathlib import Path

import numpy as np
import mujoco
from PIL import Image

CWD = Path(__file__).resolve().parent

VIEWS = {
    # name: (azimuth, elevation, distance, lookat)
    "front": (180.0, -8.0, 2.6, [0.0, 0.0, 0.85]),
    "side": (90.0, -8.0, 2.6, [0.0, 0.0, 0.85]),
    "iso": (135.0, -18.0, 2.8, [0.0, 0.0, 0.85]),
    "back": (0.0, -8.0, 2.6, [0.0, 0.0, 0.85]),
    "top": (135.0, -80.0, 3.0, [0.0, 0.0, 0.85]),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default=str(CWD / "elf3.xml"))
    ap.add_argument("--out", default="render")
    ap.add_argument("--w", type=int, default=900)
    ap.add_argument("--h", type=int, default=900)
    ap.add_argument("--drop", action="store_true", help="落地对齐")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    # 放大离屏帧缓冲，否则默认 640 会限制渲染尺寸
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), args.w)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), args.h)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if args.drop:
        # 找脚部碰撞 geom 最低点，把基座下移
        lowest = 1e9
        for g in range(model.ngeom):
            if int(model.geom_group[g]) != 3:
                continue
            gtype = int(model.geom_type[g])
            if gtype == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                half = float(model.geom_size[g][1])
                ax = data.geom_xmat[g].reshape(3, 3)[:, 2]
                lowest = min(lowest, float(data.geom_xpos[g][2]) - half * abs(float(ax[2])) - float(model.geom_size[g][0]))
            elif gtype == int(mujoco.mjtGeom.mjGEOM_SPHERE):
                lowest = min(lowest, float(data.geom_xpos[g][2]) - float(model.geom_size[g][0]))
        if np.isfinite(lowest):
            data.qpos[2] -= lowest
            mujoco.mj_forward(model, data)
            print(f"落地: 悬空 {lowest:+.5f} -> base z = {data.qpos[2]:.5f}")

    outdir = CWD / args.out
    outdir.mkdir(exist_ok=True)

    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    # 只显示视觉 mesh（和 vuer 里一致），隐藏碰撞体与坐标系
    for g in range(6):
        opt.geomgroup[g] = 0
    opt.geomgroup[2] = 1
    opt.sitegroup[:] = 0
    opt.flags[int(mujoco.mjtVisFlag.mjVIS_JOINT)] = 0
    opt.flags[int(mujoco.mjtVisFlag.mjVIS_ACTUATOR)] = 0
    # 保留地面与楼梯（group 0）
    opt.geomgroup[0] = 1

    renderer = mujoco.Renderer(model, height=args.h, width=args.w)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)

    for name, (az, el, dist, look) in VIEWS.items():
        cam.azimuth = az
        cam.elevation = el
        cam.distance = dist
        cam.lookat[:] = look
        renderer.update_scene(data, camera=cam, scene_option=opt)
        img = renderer.render()
        path = outdir / f"{name}.png"
        Image.fromarray(img).save(str(path))
        print(f"  saved {path}")

    renderer.close()
    print("done")


if __name__ == "__main__":
    main()

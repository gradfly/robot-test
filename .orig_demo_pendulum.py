"""MuJoCo 演示：电机驱动的单摆，带交互式 3D 查看器。

运行: python demo_pendulum.py
操作: 鼠标左键旋转视角，滚轮缩放，双击选中物体；关闭窗口退出。
"""
import time

import mujoco
import mujoco.viewer
import numpy as np

# 内嵌模型：单摆（铰链关节 + 电机执行器）
XML = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="2 2 0.1" rgba="0.8 0.8 0.8 1"/>
    <body name="pole" pos="0 0 1.2">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.8" size="0.04" rgba="0.9 0.3 0.2 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" gear="1" ctrlrange="-2 2"/>
  </actuator>
</mujoco>
"""

FREQ = 0.5       # 控制频率 (Hz)
AMPLITUDE = 1.0  # 控制幅值 (N*m)


def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 相机初始视角
        viewer.cam.distance = 4.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        print("仿真已启动，关闭窗口退出。")
        while viewer.is_running():
            step_start = time.time()

            # 正弦扭矩驱动单摆
            data.ctrl[0] = AMPLITUDE * np.sin(2 * np.pi * FREQ * data.time)
            mujoco.mj_step(model, data)

            # 每 0.5 秒打印一次摆角
            if int(data.time / 0.5) != int((data.time - model.opt.timestep) / 0.5):
                angle_deg = np.degrees(data.qpos[0])
                print(f"t = {data.time:6.2f} s | 摆角 = {angle_deg:7.2f} deg")

            viewer.sync()

            # 保持实时仿真速度
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("仿真结束。")


if __name__ == "__main__":
    main()

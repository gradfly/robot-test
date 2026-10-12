"""ONNX Mujoco仿真，不依赖rsl‑rl导入链，规避tensorboard/tf bug"""
import sys
from pathlib import Path
import torch
import onnxruntime as ort

# ===================== 【自己复刻RslRlVecEnvWrapper，不import mjlab.rl】 =====================
class MyRslRlVecEnvWrapper:
    """复刻原版RslRlVecEnvWrapper功能：仅做动作clip，适配viewer调用接口"""
    def __init__(self, env, clip_actions: bool = True):
        self._env = env
        self.clip_actions = clip_actions
        self.num_envs = env.num_envs
        self.device = env.device

    @property
    def unwrapped(self):
        return self._env

    def step(self, actions: torch.Tensor):
        if self.clip_actions:
            actions = torch.clamp(actions, -1.0, 1.0)
        obs, rew, terminated, truncated, info = self._env.step(actions)
        return obs, rew, terminated, truncated, info

    def reset(self):
        return self._env.reset()

    def close(self):
        self._env.close()
# =========================================================================================

# ===================== 修改这里你的参数 =====================
TASK_ID = "你的task名字"
ONNX_FILE = r"E:\mujoco\robot test\policy.onnx"
MOTION_FILE = r"E:\mujoco\robot test\motion.npz"  # tracking任务填写；普通任务写 None
NUM_ENVS = 1
DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"
VIEWER = "native"   # 无桌面就改成 "viser"
# ==========================================================

# 注意：这里不再写 from mjlab.rl import RslRlVecEnvWrapper！！！
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.utils.os import configure_torch_backends
from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer


def main():
    configure_torch_backends()
    env_cfg = load_env_cfg(TASK_ID, play=True)

    # 处理tracking任务的motion轨迹文件
    is_tracking_task = "motion" in env_cfg.commands and isinstance(env_cfg.commands["motion"], MotionCommandCfg)
    if is_tracking_task:
        motion_cmd = env_cfg.commands["motion"]
        assert isinstance(motion_cmd, MotionCommandCfg)
        if MOTION_FILE is None:
            raise ValueError("tracking任务必须设置MOTION_FILE（motion.npz）")
        motion_cmd.motion_file = MOTION_FILE

    env_cfg.scene.num_envs = NUM_ENVS
    # 创建原始环境
    env = ManagerBasedRlEnv(cfg=env_cfg, device=DEVICE, render_mode=None)
    # 使用我们自己写的MyRslRlVecEnvWrapper，不再调用mjlab.rl模块
    env = MyRslRlVecEnvWrapper(env, clip_actions=True)

    # 加载ONNX模型
    onnx_path = Path(ONNX_FILE)
    if not onnx_path.exists():
        raise FileNotFoundError(f"ONNX文件不存在：{onnx_path}")

    providers = ["CUDAExecutionProvider","CPUExecutionProvider"] if "cuda" in DEVICE else ["CPUExecutionProvider"]
    sess = ort.InferenceSession(str(onnx_path), providers=providers)
    in_name = sess.get_inputs()[0].name
    out_name = sess.get_outputs()[0].name

    def onnx_policy(obs: torch.Tensor) -> torch.Tensor:
        obs_np = obs.detach().cpu().numpy()
        act_np = sess.run([out_name], {in_name: obs_np})[0]
        act = torch.from_numpy(act_np).to(DEVICE)
        return act

    # 启动仿真查看器
    if VIEWER == "native":
        NativeMujocoViewer(env, onnx_policy).run()
    elif VIEWER == "viser":
        ViserPlayViewer(env, onnx_policy).run()
    env.close()


if __name__ == "__main__":
    main()

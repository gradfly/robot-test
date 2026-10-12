"""Play ONNX policy in mjlab mujoco simulation"""
import os
import sys
from dataclasses import dataclass
from pathlib import Path
import torch
import tyro
import onnxruntime as ort
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.registry import list_tasks, load_env_cfg, load_rl_cfg
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.utils.os import configure_torch_backends
from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer

@dataclass
class OnnxPlayConfig:
    onnx_file: str
    motion_file: str | None = None # tracking任务必须提供
    num_envs: int = 1
    device: str | None = None
    viewer: tyro.extras.Literal["native","viser"] = "native"

def run_onnx_play(task_id: str, cfg:OnnxPlayConfig):
    configure_torch_backends()
    device = cfg.device or ("cuda:0" if torch.cuda.is_available() else "cpu")

    env_cfg = load_env_cfg(task_id, play=True)
    _ = load_rl_cfg(task_id)

    # tracking任务设置motion_file
    is_tracking_task = "motion" in env_cfg.commands and isinstance(env_cfg.commands["motion"], MotionCommandCfg)
    if is_tracking_task:
        motion_cmd = env_cfg.commands["motion"]
        assert isinstance(motion_cmd, MotionCommandCfg)
        if cfg.motion_file is None:
            raise ValueError("tracking task must supply --motion-file xxx.npz")
        motion_cmd.motion_file = cfg.motion_file

    env_cfg.scene.num_envs = cfg.num_envs
    env = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode=None)
    env = RslRlVecEnvWrapper(env)

    # load onnx
    onnx_path = Path(cfg.onnx_file)
    if not onnx_path.exists():
        raise FileNotFoundError(onnx_path)
    providers = ["CUDAExecutionProvider","CPUExecutionProvider"] if "cuda" in device else ["CPUExecutionProvider"]
    sess = ort.InferenceSession(str(onnx_path), providers=providers)
    in_name = sess.get_inputs()[0].name
    out_name = sess.get_outputs()[0].name

    def onnx_policy(obs:torch.Tensor) -> torch.Tensor:
        obs_np = obs.detach().cpu().numpy()
        act_np = sess.run([out_name], {in_name:obs_np})[0]
        return torch.from_numpy(act_np).to(device)

    # 启动viewer仿真
    if cfg.viewer == "native":
        NativeMujocoViewer(env, onnx_policy).run()
    elif cfg.viewer == "viser":
        ViserPlayViewer(env, onnx_policy).run()
    env.close()

def main():
    import mjlab.tasks
    all_tasks = list_tasks()
    chosen_task, rest_args = tyro.cli(
        tyro.extras.literal_type_from_choices(all_tasks),
        add_help=False,
        return_unknown_args=True
    )
    args = tyro.cli(OnnxPlayConfig, args=rest_args)
    run_onnx_play(chosen_task, args)

if __name__ == "__main__":
    main()

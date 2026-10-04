"""Remote training and local non-learning environment checks."""
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

from .runtime import ROOT, check_training_host, configure_runtime


def save_agent_config(path, agent):
    """Portable YAML: no Python tuple tags, compatible with export's safe_load."""
    import yaml
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    plain = json.loads(json.dumps(asdict(agent)))
    path.write_text(yaml.safe_dump(plain, sort_keys=False))


def make_env(*, num_envs=2, device="cuda:0", play=True, auto_reset=True):
    configure_runtime()
    import mjlab
    from mjlab.envs import ManagerBasedRlEnv
    from .reach import make_env_cfg
    cfg = make_env_cfg(play=play)
    cfg.scene.num_envs = num_envs
    cfg.auto_reset = auto_reset
    return ManagerBasedRlEnv(cfg, device=device)


def train(*, confirmed=False, num_envs=256, iterations=2000, seed=42, device="cuda:0", resume=None):
    check_training_host(confirmed=confirmed)  # Before constructing environment or runner.
    if num_envs < 4 or iterations < 1 or not device.startswith("cuda"):
        raise ValueError("training requires CUDA, >=4 environments and positive iterations")
    configure_runtime()
    import mjlab
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.utils.os import dump_yaml
    from .contracts import verify_contract
    from .reach import make_env_cfg
    from .reach_model import deployment_contract, load_catalogue
    from .reach_runner import GuardedRunner
    from .tasks import runner_cfg
    torch.manual_seed(seed)
    cfg, agent = make_env_cfg(), runner_cfg()
    cfg.scene.num_envs, cfg.seed = num_envs, seed
    agent.seed, agent.max_iterations = seed, iterations
    contract = deployment_contract()
    if resume:
        resume = Path(resume).resolve()
        verify_contract(json.loads((resume.parent/"deployment_contract.json").read_text()), contract)
    out = ROOT/"runs"/"reach_hold_v1"/datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out.mkdir(parents=True)
    (out/"deployment_contract.json").write_text(json.dumps(contract, indent=2)+"\n")
    (out/"reach_targets.json").write_text(json.dumps(load_catalogue(), indent=2)+"\n")
    dump_yaml(out/"params"/"env.yaml", asdict(cfg))
    save_agent_config(out/"params"/"agent.yaml", agent)
    env = ManagerBasedRlEnv(cfg, device=device)
    try:
        runner = GuardedRunner(RslRlVecEnvWrapper(env), asdict(agent), str(out), device)
        runner.remote_training_confirmed = confirmed
        if resume:
            runner.load(str(resume))
        runner.learn(num_learning_iterations=iterations, init_at_random_ep_len=False)
    finally:
        env.close()
    return {"run": str(out), "additional_iterations": iterations, "resume": str(resume) if resume else None}


def smoke(*, steps=25, num_envs=2, device="cuda:0"):
    if not 1 <= steps <= 500 or not 1 <= num_envs <= 16:
        raise ValueError("smoke check supports 1..500 steps, 1..16 environments")
    import torch
    env = make_env(num_envs=num_envs, device=device)
    from mjlab.rl import RslRlVecEnvWrapper
    from .reach import state
    from .reach_runner import GuardedRunner
    from .tasks import runner_cfg
    try:
        obs, _ = env.reset()
        assert obs["actor"].shape == (num_envs, 34)
        # Construct and call the real RSL actor; never learn or update weights.
        runner = GuardedRunner(RslRlVecEnvWrapper(env), asdict(runner_cfg()), None, device)
        actor = runner.get_inference_policy(device)
        with torch.inference_mode():
            action = actor(runner.env.get_observations())
        assert action.shape == (num_envs, 6) and torch.isfinite(action).all()
        for _ in range(steps):
            obs, rew, terminated, truncated, _ = env.step(torch.zeros(num_envs, 6, device=device))
            if not torch.isfinite(obs["actor"]).all() or not torch.isfinite(rew).all():
                raise RuntimeError("non-finite Reach smoke result")
            if terminated.any():
                raise RuntimeError("unexpected termination while holding initial pose")
        drift = (env.scene["robot"].data.joint_pos-state(env).command).abs().max().item()
        if drift > .02:
            raise RuntimeError(f"initial hold drift too large: {drift}")
        return {"steps": steps, "num_envs": num_envs, "device": device,
                "observation_dim": 34, "action_dim": 6, "max_hold_drift_rad": drift,
                "runner_forward": "passed", "training_updates": 0}
    finally:
        env.close()


def play(checkpoint, viewer="native", device="cuda:0"):
    configure_runtime()
    import mjlab
    from mjlab.scripts.play import PlayConfig, run_play
    from .contracts import verify_contract
    from .reach_model import TASK_ID, deployment_contract
    checkpoint = Path(checkpoint).resolve()
    verify_contract(json.loads((checkpoint.parent/"deployment_contract.json").read_text()), deployment_contract())
    run_play(TASK_ID, PlayConfig(checkpoint_file=str(checkpoint), num_envs=1, device=device,
        viewer=viewer, log_root=str(ROOT/"runs")))
    return {"checkpoint": str(checkpoint), "training_updates": 0}

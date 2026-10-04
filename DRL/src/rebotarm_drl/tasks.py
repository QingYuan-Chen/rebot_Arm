"""MJLab plugin registration; importing never starts training."""
from .runtime import configure_runtime
configure_runtime()

from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from mjlab.tasks.registry import register_mjlab_task
from .reach import make_env_cfg
from .reach_model import TASK_ID
from .reach_runner import GuardedRunner


def runner_cfg():
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(hidden_dims=(128, 128), activation="elu", obs_normalization=True,
            distribution_cfg={"class_name": "GaussianDistribution", "init_std": .5, "std_type": "scalar"}),
        critic=RslRlModelCfg(hidden_dims=(128, 128), activation="elu", obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(learning_rate=3e-4, entropy_coef=.005),
        num_steps_per_env=32, max_iterations=2000, save_interval=100,
        experiment_name="reach_hold_v1", logger="tensorboard", upload_model=False)


register_mjlab_task(TASK_ID, make_env_cfg(), make_env_cfg(play=True), runner_cfg(), runner_cls=GuardedRunner)

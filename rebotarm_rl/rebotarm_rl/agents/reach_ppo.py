"""Reach PPO 配置；算法实现使用 mjlab / RSL-RL。"""
from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg

def _runner_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(
            hidden_dims=(128, 128),
            activation="elu",
            distribution_cfg={
                "class_name": "GaussianDistribution",
                "init_std": 0.5,
                "std_type": "scalar",
            },
        ),
        critic=RslRlModelCfg(hidden_dims=(128, 128), activation="elu"),
        algorithm=RslRlPpoAlgorithmCfg(
            learning_rate=3.0e-4,
            num_learning_epochs=5,
            num_mini_batches=4,
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
        ),
        # RSL-RL builds separate actor and critic networks.  This first task
        # has no privileged state, so both networks intentionally consume the
        # same public actor observation group.
        obs_groups={"actor": ("actor",), "critic": ("actor",)},
        experiment_name="rebotarm_mjlab_reach",
        logger="tensorboard",
        num_steps_per_env=24,
        max_iterations=1000,
        save_interval=100,
    )

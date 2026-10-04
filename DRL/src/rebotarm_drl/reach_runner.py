"""Defence in depth for the project's MJLab task, including upstream train CLI."""
from mjlab.rl import MjlabOnPolicyRunner
from .runtime import check_training_host


class GuardedRunner(MjlabOnPolicyRunner):
    remote_training_confirmed = False

    def learn(self, *args, **kwargs):
        check_training_host(confirmed=self.remote_training_confirmed)
        return super().learn(*args, **kwargs)

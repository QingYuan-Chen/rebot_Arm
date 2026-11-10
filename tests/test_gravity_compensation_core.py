from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

import numpy as np

try:
    import geometry_msgs.msg
except ImportError:
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Pose = type("Pose", (), {})
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

tf_transformations = types.ModuleType("tf_transformations")
tf_transformations.euler_from_quaternion = lambda _q: (0.0, 0.0, 0.0)
tf_transformations.quaternion_from_matrix = lambda _m: (0.0, 0.0, 0.0, 1.0)
sys.modules.setdefault("tf_transformations", tf_transformations)

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "src" / "rebotarmcontroller"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rebotarmcontroller.controller_parameters import apply_gravity_compensation_tau_scale  # type: ignore[import-not-found]


class GravityCompensationCoreTests(unittest.TestCase):
    def test_tau_scale_leaves_default_gravity_torque_unchanged(self) -> None:
        tau = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

        scaled = apply_gravity_compensation_tau_scale(tau)

        np.testing.assert_allclose(scaled, tau)
        np.testing.assert_allclose(tau, np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))

    def test_tau_scale_rejects_non_six_axis_vector(self) -> None:
        with self.assertRaisesRegex(ValueError, "6 values"):
            apply_gravity_compensation_tau_scale(np.array([1.0, 2.0]))


if __name__ == "__main__":
    unittest.main()

"""Pure final-sample bilateral-contact and lift metrics for explicit grasp trials."""
from dataclasses import dataclass
from collections.abc import Mapping
import math


@dataclass(frozen=True)
class GraspQuality:
    contact_detected: bool
    lift_detected: bool
    lift_height_m: float
    success: bool
    status: str


def evaluate_grasp_quality(*, gripper_contacts: Mapping[str, int],
                           initial_object_height_m: float,
                           final_object_height_m: float,
                           min_lift_m: float = 0.03) -> GraspQuality:
    if len(gripper_contacts) != 2 or any(not isinstance(v, int) or v < 0 for v in gripper_contacts.values()):
        raise ValueError("two finger contact counts must be nonnegative integers")
    values = (initial_object_height_m, final_object_height_m, min_lift_m)
    if any(not math.isfinite(v) for v in values) or min_lift_m <= 0:
        raise ValueError("heights must be finite and minimum lift positive")
    contact = all(v > 0 for v in gripper_contacts.values())
    height = final_object_height_m - initial_object_height_m
    lifted = height >= min_lift_m
    success = contact and lifted
    status = "grasp_lift_success" if success else "contact_without_lift" if contact else "no_bilateral_contact"
    return GraspQuality(contact, lifted, height, success, status)

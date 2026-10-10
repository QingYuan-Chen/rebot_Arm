from types import SimpleNamespace
import pytest
from rebotarm_simulation.diagnostics.mujoco_grasp_quality import evaluate_grasp_quality
from rebotarm_simulation.diagnostics.mujoco_runner import target_gripper_contacts


def test_unrelated_contact_cannot_count_as_grasp():
    contacts = [SimpleNamespace(body1=a, body2=b) for a,b in
                [("bottle", "table"), ("left", "table"), ("bottle", "left"), ("right", "arm")]]
    counts = target_gripper_contacts(contacts, "bottle", ("left", "right"))
    assert counts == {"left": 1, "right": 0}
    assert not evaluate_grasp_quality(gripper_contacts=counts,
        initial_object_height_m=0., final_object_height_m=.1).success


@pytest.mark.parametrize("counts,height,success", [
    ({"left":1,"right":1},.1,True), ({"left":1,"right":1},.002,False),
    ({"left":0,"right":0},.1,False), ({"left":1,"right":0},.1,False)])
def test_final_bilateral_contact_and_lift_both_required(counts,height,success):
    assert evaluate_grasp_quality(gripper_contacts=counts,
        initial_object_height_m=0., final_object_height_m=height).success is success


def test_grasp_metrics_reject_missing_or_nonfinite_evidence():
    with pytest.raises(ValueError):
        evaluate_grasp_quality(gripper_contacts={"left":1,"right":1},
            initial_object_height_m=0., final_object_height_m=float("nan"))

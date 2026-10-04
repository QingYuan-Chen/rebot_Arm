from pathlib import Path
import ast
import shutil
from urllib.parse import urlsplit
import xml.etree.ElementTree as ET

from rebotarm_simulation.core import resource_paths
from rebotarm_simulation.control.control_config import load_motor_control_parameters


ROOT = Path(__file__).resolve().parents[1]


def test_bringup_installs_launch_structure_documentation():
    launch_readme = ROOT / "src/rebotarm_bringup/launch/README.md"
    setup_source = (ROOT / "src/rebotarm_bringup/setup.py").read_text(
        encoding="utf-8"
    )

    assert launch_readme.is_file()
    assert "hardware_controller.launch.py" in launch_readme.read_text(encoding="utf-8")
    assert 'glob("launch/*.md")' in setup_source


def test_model_meshes_have_one_declared_owner():
    package = ROOT / "src/rebotarm_moveit_config"
    model = ET.parse(package / "config/rebotarm.urdf").getroot()
    for mesh in model.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        assert uri.startswith("package://rebotarm_moveit_config/meshes/")
        assert (package / uri.removeprefix("package://rebotarm_moveit_config/")).is_file()
    assert not (ROOT / "src/rebotarm_bringup/description/meshes").exists()
    assert not (ROOT / "src/rebotarm_bringup/description/urdf/reBot-DevArm_fixend.urdf").exists()


def test_internal_manifest_graph_is_acyclic():
    graph = {}
    for path in (ROOT / "src").glob("*/package.xml"):
        manifest = ET.parse(path).getroot()
        graph[manifest.findtext("name")] = {
            item.text for item in manifest
            if item.tag in {"depend", "exec_depend", "build_depend", "build_export_depend"}
        }

    def visit(package, stack):
        assert package not in stack, " -> ".join([*stack, package])
        for dependency in graph[package] & graph.keys():
            visit(dependency, [*stack, package])

    for package in graph:
        visit(package, [])


def test_lower_layers_do_not_resolve_bringup_resources():
    for package in ("rebotarm_dashboard", "rebotarm_simulation", "rebotarm_moveit_config"):
        for path in (ROOT / "src" / package).rglob("*.py"):
            source = path.read_text()
            assert "rebotarm_bringup" not in source, path


def test_motor_parameters_load_from_install_without_source_tree(tmp_path, monkeypatch):
    import ament_index_python.packages

    monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
    monkeypatch.setattr(resource_paths.sys, "prefix", str(tmp_path))
    share = tmp_path / "share"
    for package, relative in (
        ("rebotarm_simulation", "config/motor_control_calibration.yaml"),
        ("rebotarm_moveit_config", "config/rebotarm.urdf"),
    ):
        target = share / package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / "src" / package / relative, target)
    monkeypatch.setattr(resource_paths, "__file__", str(tmp_path / "lib/python/pkg/resource_paths.py"))
    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory", lambda p: str(share / p))
    values = load_motor_control_parameters()
    assert values.control_rate_hz == 100
    assert values.arm.joint_names == tuple(f"joint{i}" for i in range(1, 7))
    assert values.arm.pos_kp == (150., 150., 150., 50., 50., 50.)
    assert values.arm.vel_kp == (.0125, .0125, .0125, .0008, .0008, .0008)
    assert values.arm.velocity_limit == (5., 5., 5., 3., 3., 3.)
    assert values.gripper.firmware_default_kp == 8.
    assert values.gripper.firmware_default_kd == 1.


def test_internal_imports_launch_nodes_and_resource_owners_are_declared():
    packages = {p.parent.name: p.parent for p in (ROOT / "src").glob("*/package.xml")}
    ros_packages = {
        "ament_index_python", "builtin_interfaces", "control_msgs", "cv_bridge",
        "geometry_msgs", "launch", "launch_ros", "moveit_configs_utils", "moveit_msgs",
        "rclpy", "rosgraph_msgs", "sensor_msgs", "shape_msgs", "std_msgs", "std_srvs",
        "tf2_ros", "tf_transformations", "trajectory_msgs", "visualization_msgs",
        "joint_state_publisher", "robot_state_publisher", "rviz2", "moveit_ros_move_group",
        "pinocchio", "ruckig",
    }
    checked_packages = packages.keys() | ros_packages
    missing = []
    for name, package in packages.items():
        manifest = ET.parse(package / "package.xml").getroot()
        declared = {name} | {item.text for item in manifest if item.tag in {"depend", "exec_depend"}}
        for path in package.rglob("*.py"):
            references = set()
            tree = ast.parse(path.read_text(encoding="utf-8-sig"))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    references.update(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and not node.level:
                    references.add((node.module or "").split(".")[0])
                elif isinstance(node, ast.Constant) and isinstance(node.value, str):
                    # Includes package=, FindPackageShare and package_resource calls,
                    # plus legacy source-tree resource references.
                    if node.value in checked_packages:
                        references.add(node.value)
                    parts = Path(node.value).parts
                    for i, part in enumerate(parts[:-1]):
                        if part == "src" and parts[i + 1] in packages:
                            references.add(parts[i + 1])
            for dependency in (references & checked_packages) - declared:
                missing.append(f"{path.relative_to(ROOT)} -> {dependency}")
        for path in package.rglob("*.urdf"):
            for element in ET.parse(path).iter():
                for value in element.attrib.values():
                    uri = urlsplit(value)
                    if uri.scheme == "package" and uri.netloc in packages:
                        assert uri.netloc in declared, (path, uri.netloc)
                        assert (packages[uri.netloc] / uri.path.lstrip("/")).exists(), value
    assert not missing, "Undeclared package dependencies:\n" + "\n".join(missing)


def test_hardware_has_no_teach_file_or_workflow_dependency():
    package = ROOT / "src/rebotarmcontroller"
    assert not (package / "rebotarmcontroller/teach_recorder.py").exists()
    for path in (package / "rebotarmcontroller").glob("*.py"):
        source = path.read_text()
        assert "rebotarm_teach" not in source
        assert "teleop/teach_record/" not in source

"""把权威 URDF 转换为 MuJoCo MJCF 模型的离线生成工具。

模型来源永远是运动规划配置包中的 rebotarm.urdf；本模块把它转成仿真包内的
models/rebotarm/robot.xml，并保证转换结果可复现：

1. :func:`stage_urdf` 在临时目录准备一份可被 MuJoCo 解析的 URDF 副本
   （注入 mujoco 编译扩展、给 visual/collision 元素改名、把 package:// 网格拷到 assets/）；
2. 用 MuJoCo 的 MjSpec 解析并编译该副本，得到基础 MJCF；
3. 依次补上编译器默认值、几何分类、接触排除、手指联动、末端 site、
   关节动力学、执行器与传感器；
4. :func:`_canonicalize` 统一缩进与换行并加自动生成警告头，输出稳定字节流。

robot.xml 属于自动生成物，禁止手工编辑；命令行入口为 ``rebotarm_urdf_to_mjcf``，
``--check`` 模式只比较磁盘模型与重新生成的结果，用于 CI/预提交时发现模型过期。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tempfile
from typing import Sequence
import xml.etree.ElementTree as ET

import mujoco
import yaml

from rebotarm_simulation.model_tools.gemini2_payload import add_gemini2_payload


# URDF 网格资源只允许来自该前缀；转换时会被改写为临时目录下的 assets/ 相对路径
PACKAGE_MESH_PREFIX = "package://rebotarm_moveit_config/meshes/"
# 需要生成执行器与传感器的关节顺序：6 个机械臂关节 + 左右手指关节。
# 顺序被仿真运行时按名查找使用，不得重排。
from rebotarm_simulation.core.model_contract import JOINT_NAMES, actuator_name_for_joint

JOINTS = list(JOINT_NAMES)


def authoritative_urdf_path(repo_root: Path) -> Path:
    """返回唯一的权威 URDF 路径（运动规划配置包中的 rebotarm.urdf）。"""
    return repo_root / "src/rebotarm_moveit_config/config/rebotarm.urdf"


def _model_directory(repo_root: Path) -> Path:
    """返回仿真包内的 MuJoCo 模型目录（models/rebotarm，网格资源在其 assets/ 下）。"""
    return repo_root / "src/rebotarm_simulation/models/rebotarm"


def stage_urdf(source: Path, repo_root: Path, temporary_dir: Path) -> Path:
    """在临时目录中生成可供 MuJoCo 加载的 URDF 副本并返回其路径。

    做了三件事：
    1. 在根节点插入 ``<mujoco><compiler .../></mujoco>`` 扩展；
       ``discardvisual=false`` 保留视觉网格、``fusestatic=false`` 不把固定连杆合并、
       ``strippath=false`` 不剥离资源路径，保证 Mesh 名称与 URDF 一致、便于后续按名分类；
    2. 给每个 link 的 visual/collision 元素取唯一名字（``link_role_index``），
       避免 MuJoCo 因重名报错；
    3. 把 ``package://`` 网格 URI 改写为 assets 相对路径，并从模型目录的 assets/
       拷贝到临时目录的 assets/，使编译不依赖安装空间。

    网格 URI 不在允许前缀内、或源文件缺失时抛 ``ValueError`` / ``FileNotFoundError``。
    """
    root = ET.parse(source).getroot()
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        {"discardvisual": "false", "fusestatic": "false", "strippath": "false"},
    )
    source_assets = _model_directory(repo_root) / "assets"
    staged_assets = temporary_dir / "assets"
    staged_assets.mkdir(parents=True, exist_ok=True)

    # 同一 link 的多个 visual/collision 元素会重名，这里按出现顺序编号使其唯一
    for link in root.findall("link"):
        for role in ("visual", "collision"):
            for index, element in enumerate(link.findall(role)):
                element.set("name", f'{link.attrib["name"]}_{role}_{index}')

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename.startswith(PACKAGE_MESH_PREFIX):
            raise ValueError(f"unsupported URDF mesh URI: {filename}")
        basename = filename.removeprefix(PACKAGE_MESH_PREFIX)
        source_mesh = source_assets / basename
        if not source_mesh.is_file():
            raise FileNotFoundError(source_mesh)
        shutil.copy2(source_mesh, staged_assets / basename)
        # 改写为相对 assets/ 的路径，使临时目录自包含
        mesh.set("filename", f"assets/{basename}")

    staged = temporary_dir / source.name
    # 以 UTF-8 + XML 声明写出；临时目录在调用方退出后即被清理
    ET.ElementTree(root).write(staged, encoding="utf-8", xml_declaration=True)
    return staged


def generate_mjcf_bytes(repo_root: Path) -> bytes:
    """执行完整转换流程，返回规范化后的 MJCF 字节串。

    这是纯函数：相同 URDF 与标定必然产生相同字节，便于 ``--check`` 比对。
    整个过程在临时目录中进行，不会改动画布之外的任何文件。
    """
    source = authoritative_urdf_path(repo_root)
    with tempfile.TemporaryDirectory(prefix="rebotarm-urdf-") as directory:
        staged = stage_urdf(source, repo_root, Path(directory))
        # MjSpec 解析并编译一次，既得到基础 MJCF，也顺带校验网格/惯量可用
        spec = mujoco.MjSpec.from_file(str(staged))
        spec.compile()
        root = ET.fromstring(spec.to_xml())
        urdf_root = ET.parse(source).getroot()
        _configure_compiler(root)
        _classify_geoms(root)
        _add_contact_exclusions(root)
        _add_finger_coupling(root)
        _add_sites(root)
        _add_joint_dynamics(root, _load_joint_dynamics(repo_root))
        _add_actuators(root, urdf_root, repo_root)
        _add_sensors(root)
        add_gemini2_payload(root, repo_root)
    return _canonicalize(root)


def _configure_compiler(root: ET.Element) -> None:
    """写入编译器与几何默认参数，减少各 geom 上的重复属性。

    关键默认值：
    - ``angle=radian``：关节角与 site 姿态一律用弧度；
    - 默认 geom 接触参数 ``solref="0.01 1"``（接触时间常数 0.01 s、阻尼比 1，偏硬的稳定接触）、
      ``solimp="0.9 0.95 0.001"``（约束混合参数，控制穿透与刚度过渡）、
      ``friction="0.8 0.02 0.001"``（滑动/扭转/滚动摩擦）；
    - ``visual`` 类：``contype/conaffinity=0`` 不参与接触，``group=2`` 仅用于显示；
    - ``collision`` 类：``contype/conaffinity=1`` 参与接触，``group=3``，
      半透明蓝色 ``rgba`` 便于观察碰撞体。

    ``default`` 节点必须紧跟 ``compiler`` 才能对全模型生效，因此用索引插入。
    """
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    default = ET.Element("default")
    ET.SubElement(
        default,
        "geom",
        {"solref": "0.01 1", "solimp": "0.9 0.95 0.001", "friction": "0.8 0.02 0.001"},
    )
    visual = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(visual, "geom", {"contype": "0", "conaffinity": "0", "group": "2"})
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(
        collision,
        "geom",
        {"contype": "1", "conaffinity": "1", "group": "3", "rgba": "0.2 0.5 0.8 0.12"},
    )
    compiler_index = list(root).index(compiler)
    root.insert(compiler_index + 1, default)


def _classify_geoms(root: ET.Element) -> None:
    """把每个 geom 明确划分为 visual 或 collision 两类并统一命名/分组。

    MuJoCo 从 URDF 转换时用 ``contype=0`` 标记纯视觉网格，这里沿用该判据：
    - 视觉体：不参与接触（contype/conaffinity=0）、group=2，保留材质颜色；
    - 碰撞体：参与接触（=1）、group=3，移除 ``rgba`` 以免遮盖视觉外观；
    两类都去掉 ``density``，质量属性由 URDF 的惯性显式给出，避免重复计算。
    手指碰撞网格额外提高滑动摩擦（1.2），保证抓取时不易打滑。
    """
    for body in root.findall("worldbody//body"):
        body_name = body.attrib["name"]
        mesh_counts: dict[str, int] = {}
        for geom in body.findall("geom"):
            mesh = geom.attrib["mesh"]
            index = mesh_counts.get(mesh, 0)
            mesh_counts[mesh] = index + 1
            is_visual = geom.attrib.get("contype") == "0"
            role = "visual" if is_visual else "collision"
            geom.set("name", f"{body_name}_{mesh}_{role}")
            geom.set("class", role)
            geom.set("contype", "0" if is_visual else "1")
            geom.set("conaffinity", "0" if is_visual else "1")
            geom.set("group", "2" if is_visual else "3")
            geom.attrib.pop("density", None)
            if not is_visual:
                geom.attrib.pop("rgba", None)
            if not is_visual and mesh in {"left_finger", "right_finger"}:
                geom.set("friction", "1.2 0.02 0.001")


def _add_contact_exclusions(root: ET.Element) -> None:
    """禁用相邻/镜像连杆之间不可能发生且会拖慢求解的接触对。

    列表包含相邻父子连杆、隔一个连杆的臂段、末端与两指，以及两指之间；
    这些几何在运动学上本就重叠或被约束住，若参与碰撞检测会产生持续的
    自碰撞力与不必要的求解开销（也会污染碰撞反馈）。
    """
    contact = ET.SubElement(root, "contact")
    for body1, body2 in (
        ("base_link", "link1"),
        ("link1", "link2"),
        ("link2", "link3"),
        ("link3", "link4"),
        ("link4", "link5"),
        ("link5", "link6"),
        ("link6", "end_link"),
        ("end_link", "left_finger_link"),
        ("end_link", "right_finger_link"),
        ("link2", "link4"),
        ("link4", "link6"),
        ("left_finger_link", "right_finger_link"),
    ):
        ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})


def _add_finger_coupling(root: ET.Element) -> None:
    """用 equality 约束把两根手指绑成镜像联动。

    ``polycoef="0 -1 0 0 0"`` 表示 q_right = 0 - 1·q_left，即两指行程大小相等、
    符号相反（对称开合）。夹爪只有一个驱动自由度，靠该约束传递到另一根手指。
    """
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "finger_coupling",
            "joint1": "right_finger_joint",
            "joint2": "left_finger_joint",
            "polycoef": "0 -1 0 0 0",
        },
    )


def _add_sites(root: ET.Element) -> None:
    """在末端连杆上加两个 site，供取末端位姿与挂载腕部相机。

    - ``ee_site``：末端执行器参考点，沿末端 x 轴内缩 0.04 m（TCP 偏移），
      传感器与上层均以它为准；
    - ``wrist_camera_mount``：腕部相机安装点，位置偏移 ``(-0.04, 0, 0.04)``，
      姿态单位四元数表示与末端连杆坐标系对齐。

    找不到 ``end_link`` 时抛 ``ValueError``（说明 URDF 结构已变，转换结果不可信）。
    """
    end_link = root.find('.//body[@name="end_link"]')
    if end_link is None:
        raise ValueError("converted MJCF is missing end_link")
    ET.SubElement(end_link, "site", {"name": "ee_site", "pos": "-0.04 0 0", "size": "0.008"})
    ET.SubElement(
        end_link,
        "site",
        {"name": "wrist_camera_mount", "pos": "-0.04 0 0.04", "quat": "1 0 0 0", "size": "0.005"},
    )


def _load_joint_dynamics(repo_root: Path) -> dict[str, dict[str, float]]:
    """从电机标定文件读取每个关节的 MuJoCo 动力学参数。

    返回 ``关节名 → {damping, armature, frictionloss}``；标定文件缺少
    ``model_dynamics`` 或某个关节条目时抛 ``ValueError``，避免生成一个
    「静默丢参数」的模型。
    """
    path = repo_root / "src/rebotarm_simulation/config/motor_control_calibration.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    dynamics = payload.get("model_dynamics")
    if not isinstance(dynamics, dict):
        raise ValueError(f"expected model_dynamics mapping in {path}")
    result: dict[str, dict[str, float]] = {}
    for joint_name in JOINTS:
        values = dynamics.get(joint_name)
        if not isinstance(values, dict):
            raise ValueError(f"missing model_dynamics for {joint_name}")
        result[joint_name] = {
            "damping": float(values["damping"]),
            "armature": float(values["armature"]),
            "frictionloss": float(values["frictionloss"]),
        }
    return result


def _add_joint_dynamics(root: ET.Element, dynamics: dict[str, dict[str, float]]) -> None:
    """把阻尼/电枢惯量/静摩擦写入对应 joint 元素（未列出的关节保持默认）。"""
    for joint in root.findall("worldbody//joint"):
        name = joint.attrib["name"]
        if name not in dynamics:
            continue
        # :g 去掉多余的尾零，保证生成文本稳定可复现
        for key, value in dynamics[name].items():
            joint.set(key, f"{value:g}")


def _add_actuators(root: ET.Element, urdf_root: ET.Element, repo_root: Path) -> None:
    """为 8 个关节各生成一个力矩型 motor 执行器。

    ``gear=1`` 表示控制量就是关节力矩（N·m），不做额外传动缩放；
    ``ctrlrange``/``forcerange`` 取 URDF 的 ``limit@effort``，手指关节则改用
    夹爪标定的指尖力上限（URDF 的手指数值不反映真实夹持能力）。
    同时开启 ``ctrllimited``/``forcelimited``，使超出范围的指令被 MuJoCo 直接限幅。
    """
    limits = {
        joint.attrib["name"]: joint.find("limit")
        for joint in urdf_root.findall("joint")
        if joint.attrib.get("name") in JOINTS
    }
    actuator = ET.SubElement(root, "actuator")
    finger_force_limit = _load_gripper_force_limit(repo_root)
    for joint_name in JOINTS:
        limit = limits[joint_name]
        if limit is None:
            raise ValueError(f"URDF joint has no limit: {joint_name}")
        effort = (
            finger_force_limit
            if joint_name in {"left_finger_joint", "right_finger_joint"}
            else float(limit.attrib["effort"])
        )
        if joint_name in {"left_finger_joint", "right_finger_joint"}:
            # Keep both force-clamping layers on the same simulation-only setting.
            joint = root.find(f'.//joint[@name="{joint_name}"]')
            if joint is None:
                raise ValueError(f"missing MuJoCo finger joint: {joint_name}")
            joint.set("actuatorfrcrange", f"-{effort:g} {effort:g}")
            joint.set("actuatorfrclimited", "true")
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": actuator_name_for_joint(joint_name),
                "joint": joint_name,
                "gear": "1",
                "ctrlrange": f"-{effort} {effort}",
                "ctrllimited": "true",
                "forcelimited": "true",
                "forcerange": f"-{effort} {effort}",
            },
        )


def _load_gripper_force_limit(repo_root: Path) -> float:
    """读取夹爪指尖力上限（N）并校验为正数，异常时抛 ``ValueError``。"""
    path = repo_root / "src/rebotarm_simulation/config/motor_control_calibration.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    try:
        value = float(payload["gripper"]["finger_force_limit_n"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"expected gripper.finger_force_limit_n in {path}") from exc
    if value <= 0.0:
        raise ValueError(f"gripper.finger_force_limit_n must be positive in {path}")
    return value


def _add_sensors(root: ET.Element) -> None:
    """注册仿真观测传感器。

    每个关节各有 ``*_pos``（rad）与 ``*_vel``（rad/s；手指关节为 m 与 m/s），
    每个执行器各有 ``*_force``（N·m，手指为 N）；另有末端 ``ee_position``
    （m）与 ``ee_orientation``（四元数 wxyz）两个 site 级传感器。
    仿真运行时按这些名字读取状态，改名会破坏接口。
    """
    sensor = ET.SubElement(root, "sensor")
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(sensor, "jointpos", {"name": f"{prefix}_pos", "joint": joint_name})
        ET.SubElement(sensor, "jointvel", {"name": f"{prefix}_vel", "joint": joint_name})
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(
            sensor,
            "actuatorfrc",
            {"name": f"{prefix}_force", "actuator": actuator_name_for_joint(joint_name)},
        )
    ET.SubElement(sensor, "framepos", {"name": "ee_position", "objtype": "site", "objname": "ee_site"})
    ET.SubElement(sensor, "framequat", {"name": "ee_orientation", "objtype": "site", "objname": "ee_site"})


def _canonicalize(root: ET.Element) -> bytes:
    """把 MJCF 序列化为稳定字节流：统一缩进、换行与空元素写法，并加自动生成警告头。

    同时移除 MuJoCo 新版本才输出的 mesh ``content_type`` 属性，
    使不同 MuJoCo 版本生成的结果一致，``--check`` 才不会误报模型过期。
    """
    for mesh in root.findall("asset/mesh"):
        mesh.attrib.pop("content_type", None)
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    warning = "<!-- AUTO-GENERATED from rebotarm.urdf; do not edit robot.xml manually. -->\n"
    return (warning + xml.replace("\r\n", "\n").rstrip() + "\n").encode("utf-8")


def check_generated_model(repo_root: Path, output: Path) -> bool:
    """判断磁盘上的 MJCF 是否与当前 URDF/标定一致（True 表示已同步）。"""
    return output.is_file() and output.read_bytes() == generate_mjcf_bytes(repo_root)


def write_generated_model(repo_root: Path, output: Path) -> None:
    """生成并原子写入 MJCF：先写同目录临时文件再 ``replace``，避免留下半截文件。"""
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    try:
        temporary.write_bytes(generate_mjcf_bytes(repo_root))
        temporary.replace(output)
    finally:
        # 成功时临时文件已被 replace 掉，失败时清理残留（missing_ok 容忍不存在）
        temporary.unlink(missing_ok=True)


def _default_repo_root() -> Path:
    """由本文件位置回推仓库根目录（src/<包>/<包>/本文件 → 上溯 3 级）。"""
    from rebotarm_simulation.core.resource_paths import source_repository_root
    return source_repository_root()


def _default_output(repo_root: Path) -> Path:
    """默认输出路径：模型目录下的 robot.xml。"""
    return _model_directory(repo_root) / "robot.xml"


def main(argv: Sequence[str] | None = None) -> int:
    """命令行入口：生成 MJCF，或用 ``--check`` 仅校验是否过期。

    选项：
        ``--repo-root``: 仓库根目录，默认按本文件位置推断；
        ``--output``: 输出路径，默认 models/rebotarm/robot.xml；
        ``--check``: 只比较不写入，模型一致返回 0、过期返回 1（供 CI 使用）。
    """
    parser = argparse.ArgumentParser(description="Generate reBotArm MJCF from the authoritative URDF")
    parser.add_argument("--repo-root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    repo_root = (args.repo_root or _default_repo_root()).resolve()
    output = (args.output or _default_output(repo_root)).resolve()

    if args.check:
        if check_generated_model(repo_root, output):
            print(f"MJCF is up to date: {output}")
            return 0
        print(f"MJCF is stale; regenerate with rebotarm_urdf_to_mjcf --repo-root \"{repo_root}\"")
        return 1

    write_generated_model(repo_root, output)
    print(f"Generated MJCF: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

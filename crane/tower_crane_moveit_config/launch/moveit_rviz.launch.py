from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()

    # RViz re-loads MoveGroup parameters at runtime. Removing the trolley joint
    # override here prevents type conflicts when RViz writes the parameter tree.
    joint_limits = moveit_config.joint_limits.get("robot_description_planning", {})
    joint_overrides = joint_limits.get("joint_limits", {})
    joint_overrides.pop("trolley_joint", None)

    return generate_moveit_rviz_launch(moveit_config)

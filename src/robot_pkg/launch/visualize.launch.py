from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml
import math
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_pkg')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'wilson_robot.xacro')
    yaml_file = os.path.join(pkg_share, 'config', 'robot_dimensions.yaml')

    # Load YAML and convert units (yaml values are in cm -> convert to meters)
    with open(yaml_file, 'r') as f:
        params = yaml.safe_load(f)

    # convert cm->m for body and leg lengths
    body_length_m = float(params['body']['length']) / 100.0
    body_width_m = float(params['body']['width']) / 100.0
    body_height_m = float(params['body']['height']) / 100.0

    link1_m = float(params['leg']['link1_length']) / 100.0
    link2_m = float(params['leg']['link2_length']) / 100.0
    link3_m = float(params['leg']['link3_length']) / 100.0

    # Convert joint limits (degrees in yaml) to radians for xacro args
    def degs_to_rad(degrees):
        return math.radians(float(degrees))

    joint_limits = params['visualization'].get('joint_limits', {})
    hip_lower = degs_to_rad(joint_limits["hip_lower"])
    hip_upper = degs_to_rad(joint_limits["hip_upper"])
    thigh_lower = degs_to_rad(joint_limits["thigh_lower"])
    thigh_upper = degs_to_rad(joint_limits["thigh_upper"])
    shin_lower = degs_to_rad(joint_limits["shin_lower"])
    shin_upper = degs_to_rad(joint_limits["shin_upper"])

    # Build xacro command substitution to render URDF at launch time
    xacro_args = [
        f'body_length:={body_length_m}',
        f'body_width:={body_width_m}',
        f'body_height:={body_height_m}',
        f'hip_length:={link1_m}',
        f'thigh_length:={link2_m}',
        f'shin_length:={link3_m}',
        f'hip_lower:={hip_lower}',
        f'hip_upper:={hip_upper}',
        f'thigh_lower:={thigh_lower}',
        f'thigh_upper:={thigh_upper}',
        f'shin_lower:={shin_lower}',
        f'shin_upper:={shin_upper}',
    ]

    # Construct a single command string so Command substitution preserves spaces correctly
    xacro_cmd_str = 'xacro ' + urdf_xacro + ' ' + ' '.join(xacro_args)
    robot_desc = ParameterValue(Command(xacro_cmd_str), value_type=str)

    # robot_state_publisher node publishes TF tree from URDF (static joints at default positions without joint states)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # RViz2 - no config file provided; open RViz and add a RobotModel display (Fixed Frame: base_link)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_model.rviz')] if os.path.exists(os.path.join(pkg_share, 'rviz', 'robot_model.rviz')) else []
    )

    ld = LaunchDescription()
    ld.add_action(rsp_node)
    ld.add_action(rviz_node)

    return ld

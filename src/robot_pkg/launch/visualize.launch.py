from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
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

    # hip coordinates (front/back, left/right)
    half_len = body_length_m / 2.0
    half_wid = body_width_m / 2.0
    hip_front_x = half_len
    hip_back_x = -half_len
    hip_left_y = half_wid
    hip_right_y = -half_wid

    # Convert joint limits (degrees in yaml) to radians for xacro args
    def degs_to_rad(deg_list):
        return [math.radians(float(d)) for d in deg_list]

    hip_lim = degs_to_rad(params['joint_limits']['hip'])
    thigh_lim = degs_to_rad(params['joint_limits']['thigh'])
    shin_lim = degs_to_rad(params['joint_limits']['shin'])

    # Build xacro command substitution to render URDF at launch time
    xacro_args = [
        'body_length:=' + str(body_length_m),
        'body_width:=' + str(body_width_m),
        'body_height:=' + str(body_height_m),
        'link1_len:=' + str(link1_m),
        'link2_len:=' + str(link2_m),
        'link3_len:=' + str(link3_m),
        'hip_front_x:=' + str(hip_front_x),
        'hip_back_x:=' + str(hip_back_x),
        'hip_left_y:=' + str(hip_left_y),
        'hip_right_y:=' + str(hip_right_y),
        'hip_lower:=' + str(hip_lim[0]),
        'hip_upper:=' + str(hip_lim[1]),
        'thigh_lower:=' + str(thigh_lim[0]),
        'thigh_upper:=' + str(thigh_lim[1]),
        'shin_lower:=' + str(shin_lim[0]),
        'shin_upper:=' + str(shin_lim[1]),
    ]

    # Construct a single command string so Command substitution preserves spaces correctly
    xacro_cmd_str = 'xacro ' + urdf_xacro + ' ' + ' '.join(xacro_args)
    robot_desc = Command(xacro_cmd_str)

    # robot_state_publisher node will publish TFs based on incoming JointState messages
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

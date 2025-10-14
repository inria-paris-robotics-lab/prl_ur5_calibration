from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import uuid


def launch_setup(context, *args, **kwargs):
    # Retrieve values from launch arguments
    camera_name_space = LaunchConfiguration('namespace_camera').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_topic = LaunchConfiguration('camera_topic').perform(context)
    sample_nb = LaunchConfiguration('sample_nb').perform(context)
    run_loop = LaunchConfiguration('run_loop').perform(context)

    # Generate an anonymous node name like ROS 1 does with $(anon ...)
    tracker_node_name = f"visp_auto_tracker_{uuid.uuid4().hex[:8]}"
    calibrate_node_name = f"calibrate_external_camera_{uuid.uuid4().hex[:8]}"

    # Path to the model
    model_path = PathJoinSubstitution([
        FindPackageShare('prl_ur5_calibration'),
        'files',
        'models',
    ])
    model_name = 'april_pattern'

    camera_topic = camera_name_space + "/" + camera_name + camera_topic if camera_name_space != '' else camera_topic
    print(f"Using camera topic: {camera_topic}")
    # Tracker node
    visp_auto_tracker_node = Node(
        package='visp_auto_tracker',
        executable='visp_auto_tracker_main',
        name=tracker_node_name,
        namespace=tracker_node_name,
        output='screen',
        remappings=[
            (f'/{tracker_node_name}/image_raw', f'{camera_topic}/image_raw'),
            (f'/{tracker_node_name}/camera_info', f'{camera_topic}/camera_info'),
        ],
        parameters=[
            {'model_path': model_path.perform(context)},
            {'model_name': model_name},
            {'debug_display': True},
            {'camera_prefix': camera_topic},
            {'tracker_ref_frame': 'base_link'},
        ],
    )

    # Calibration node
    calibrate_external_camera_node = Node(
        package='prl_ur5_calibration',
        executable='calibrate_external_camera',  # Must match entry point in setup.py
        name=calibrate_node_name,
        output='screen',
        parameters=[
            {'camera_name': camera_name},
            {'tracker_node': tracker_node_name},  # Send resolved string, not LC
            {'sample_nb': int(sample_nb)},
            {'run_loop': run_loop.lower() == 'true'},
        ],
    )

    return [visp_auto_tracker_node, calibrate_external_camera_node]


def generate_launch_description():
    # Declare launch arguments like in ROS 1
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace_camera',
            default_value=TextSubstitution(text=''),
            description='Namespace of the camera'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text='camera'),
            description='Name of the camera'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value=TextSubstitution(text='/camera/color'),
            description='Topic of the camera'
        ),
        DeclareLaunchArgument(
            'sample_nb',
            default_value=TextSubstitution(text='10'),
            description='Number of samples for averaging'
        ),
        DeclareLaunchArgument(
            'run_loop',
            default_value=TextSubstitution(text='False'),
            description='Whether to keep running after first sample'
        ),
        OpaqueFunction(function=launch_setup)
    ])

import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    bool_usbcam = LaunchConfiguration('bool_usbcam')
    usb_video_device = LaunchConfiguration('usb_video_device')
    
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    usbcam_dir = get_package_share_directory('usb_cam')

    # ========== 直接启动 astra_camera_node，并通过 arguments 传递参数 ==========
    astra_camera_node = launch_ros.actions.Node(
        condition=UnlessCondition(bool_usbcam),
        package='astra_camera',
        executable='astra_camera_node',
        name='camera',
        arguments=['--ros-args', '-p', 'enable_color:=true'],
        output='screen'
    )

    bool_usbcam_arg = DeclareLaunchArgument(
        'bool_usbcam',
        default_value='true',
        description='Use usb_cam RGB stream when true; use Astra color stream when false.',
    )

    usb_video_device_arg = DeclareLaunchArgument(
        'usb_video_device',
        default_value='/dev/video21',
        description='USB camera video device for line following.',
    )

    # USB 摄像头分支
    wheeltec_USBcamera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(usbcam_dir, 'launch', 'demo.launch.py')),
        condition=IfCondition(bool_usbcam),
        launch_arguments={'video_device': usb_video_device}.items(),
    )

    # 机器人底盘启动（不变）
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )

    # line_follow 节点（调整话题名称）
    line_follow_astra = launch_ros.actions.Node(
        condition=UnlessCondition(bool_usbcam),
        package='simple_follower_ros2',
        executable='line_follow',
        name='line_follow',
        parameters=[{
            'image_input': '/camera/color/image_raw',
        }]
    )

    line_follow_usb = launch_ros.actions.Node(
        condition=IfCondition(bool_usbcam),
        package='simple_follower_ros2',
        executable='line_follow',
        name='line_follow',
        parameters=[{
            'image_input': '/image_raw',
        }]
    )

    return LaunchDescription([
        bool_usbcam_arg,
        usb_video_device_arg,
        astra_camera_node,
        wheeltec_USBcamera,
        wheeltec_robot,
        line_follow_astra,
        line_follow_usb,
    ])

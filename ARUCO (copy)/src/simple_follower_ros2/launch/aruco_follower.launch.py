import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    bool_usbcam = LaunchConfiguration('bool_usbcam')
    usb_video_device = LaunchConfiguration('usb_video_device')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    usbcam_dir = get_package_share_directory('usb_cam')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
            condition=UnlessCondition(bool_usbcam),
    )
    wheeltec_USBcamera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usbcam_dir, 'launch','demo.launch.py')),
            condition=IfCondition(bool_usbcam),
            launch_arguments={'video_device': usb_video_device}.items(),
    )

    return LaunchDescription([
    DeclareLaunchArgument(
            'bool_usbcam',
            default_value='true',
            description='Use usb_cam RGB stream for ARUCO detection. Set false for cameras that publish /camera/color/image_raw.'),
    DeclareLaunchArgument(
            'usb_video_device',
            default_value='/dev/video21',
            description='USB camera video device for ARUCO detection.'),
    wheeltec_robot,
    wheeltec_camera,
    wheeltec_USBcamera,
    launch_ros.actions.Node(
            condition=UnlessCondition(bool_usbcam),
            package='aruco_ros', 
            executable='single', 
            parameters=[
                {'image_is_rectified': True},
                {'marker_size': 0.1},
                {'marker_id': 582},
                {'reference_frame':'camera_link'},
                {'camera_frame': 'camera_link'},
                {'marker_frame': 'aruco_marker_frame'},
                {'corner_refinement':'LINES'}
                ],
       	    remappings=[('/camera_info', '/camera/color/camera_info'),
                    ('/image', '/camera/color/image_raw')],
            output='screen',
            ),
    launch_ros.actions.Node(
            condition=IfCondition(bool_usbcam),
            package='aruco_ros',
            executable='single',
            parameters=[
                {'image_is_rectified': True},
                {'marker_size': 0.1},
                {'marker_id': 582},
                {'reference_frame':'camera'},
                {'camera_frame': 'camera'},
                {'marker_frame': 'aruco_marker_frame'},
                {'corner_refinement':'LINES'}
                ],
            remappings=[('/camera_info', '/camera_info'),
                    ('/image', '/image_raw')],
            output='screen',
            ),
            
     launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='arfollower', 
            output='screen',
            ),
            ]
            
    )
    
    

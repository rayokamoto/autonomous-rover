from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                output="screen",
                namespace="camera",
                parameters=[
                    {
                        "image_size": [640, 480],
                        "time_per_frame": [1, 6],
                        "camera_frame_id": "camera_link_optical",
                    }
                ],
            ),
            Node(
                package="rplidar_ros",
                executable="rplidar_composition",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0",
                        "frame_id": "laser_frame",
                        "angle_compensate": True,
                        "scan_mode": "Standard",
                    }
                ],
            ),
        ]
    )

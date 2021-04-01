from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_calibrator",
            node_executable="camera_calibrator",
            node_name="calibrator",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"camera_source": 0},  # camera index
                {"frame_width": 1280},  # camera resolution WxH
                {"frame_height": 720},
                {"camera_fps": 30},  # video capturing fps
                {"horizontal_corner": 6},  # no of inner horizontal corners
                {"vertical_corner": 4},  # no of inner vertical corners
                {"square_size": 40},  # edge length of squares in mm (integer)
                {"sample_no": 10},  # number of samples for calibration
                {"delay": 1},  # delay between samples in seconds (integer)
            ]
        )
    ])
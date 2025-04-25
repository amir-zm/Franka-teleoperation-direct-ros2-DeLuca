# Launch Files

This directory contains ROS2 launch scripts to start the impedance controller.

## impedance_launch.py

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('franka_custom_impedance_control')
    param_file = os.path.join(pkg_share, 'config', 'impedance_params.yaml')

    return LaunchDescription([
        Node(
            package='franka_custom_impedance_control',
            executable='impedance_node',
            name='impedance_controller_node',
            output='screen',
            parameters=[param_file],
            remappings=[('joint_states', '/franka/joint_states')],
            # If your robot IP is not the default, you can pass it here:
            # parameters=[param_file, {'robot_ip': '192.168.1.11'}],
        )
    ])
```

### usage
 
 - **Launch with default params**:
ros2 launch franka_custom_impedance_control impedance_launch.py

 - **Override parameters on the command line**:
ros2 launch franka_custom_impedance_control impedance_launch.py \
  param_overrides:=[{'impedance_controller.stiffness.x': 500.0}]

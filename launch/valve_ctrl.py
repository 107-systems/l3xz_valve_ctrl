from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_valve_ctrl',
      namespace='l3xz',
      executable='l3xz_valve_ctrl_node',
      name='l3xz_valve_ctrl',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'KP' : 75.0},
        {'KI' : 25.0},
      ]
    )
  ])

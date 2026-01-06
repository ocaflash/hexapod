import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("hexapod_servo_controller"), '/launch', '/servo_controller_launch.py'])))
    
    ld.add_action(IncludeLaunchDescription(AnyLaunchDescriptionSource([
        FindPackageShare("hexapod_movement"), '/launch', '/movement_launch.py'])))

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("hexapod_bringup"), '/launch', '/joy_teleop_launch.py'])))

    delay_brain_launch = launch.actions.TimerAction(period=3.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            FindPackageShare("hexapod_brain"), '/launch', '/brain_launch.py']))])

    ld.add_action(delay_brain_launch)

    return ld

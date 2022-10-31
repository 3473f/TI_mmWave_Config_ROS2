from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

package_path = Path(__file__).resolve().parent.parent

def generate_launch_description():
    ld = LaunchDescription()

    ti_mmwave_config_node = Node(
        package='ti_mmwave_config',
        executable='ti_mmwave_config_node',
        name='ti_mmwave_config_node',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters = [
                ParameterFile(
                    param_file = package_path / 'config/params.yaml',
                    allow_substs = True)]
    )

    ld.add_action(ti_mmwave_config_node)

    return ld
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import subprocess
import time

def check_simulator_running(context, *args, **kwargs):
    """
    Verifica se o simulador já está rodando.
    Se não estiver, inicia o simulador.
    """
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        if 'custom_turtlesim_simulator' in result.stdout:
            print("✅ Simulador já está rodando.")
            return []
        else:
            print("⚠️ Simulador NÃO encontrado. Iniciando simulador...")
            return [
                ExecuteProcess(
                    cmd=['ros2', 'run', 'custom_turtlesim', 'simulator_node.py'],
                    output='screen'
                )
            ]
    except subprocess.CalledProcessError:
        print("⚠️ Erro ao verificar nós ROS. Assumindo que o simulador NÃO está rodando.")
        return [
            ExecuteProcess(
                cmd=['ros2', 'run', 'custom_turtlesim', 'simulator_node.py'],
                output='screen'
            )
        ]

def launch_teleop_with_params(context, *args, **kwargs):
    """
    Lança o teleop em um terminal interativo (xterm), passando os parâmetros definidos no launch.
    """
    # Obtém os valores dos parâmetros do contexto do launch
    turtle_name = LaunchConfiguration('turtle_name').perform(context)
    turtle_x = LaunchConfiguration('x').perform(context)
    turtle_y = LaunchConfiguration('y').perform(context)
    turtle_theta = LaunchConfiguration('theta').perform(context)

    # Comando para iniciar o teleop no xterm com argumentos
    # Usa bash -c para garantir o ambiente correto e manter o terminal aberto após execução
    teleop_cmd = [
        'xterm', '-e', 'bash', '-c',
        (
            'source ~/ros2_ws/install/setup.bash && '
            f'ros2 run custom_turtlesim teleop_node.py '
            f'--ros-args -p turtle_name:={turtle_name} -p x:={turtle_x} -p y:={turtle_y} -p theta:={turtle_theta}; '
            'echo "Pressione Enter para sair..."; read'
        )
    ]

    return [
        ExecuteProcess(
            cmd=teleop_cmd,
            output='screen'
        )
    ]

def generate_launch_description():
    # Argumentos configuráveis do launch
    default_turtle_name = f'turtle_{int(time.time()) % 10000:04d}'

    turtle_name_arg = DeclareLaunchArgument(
        'turtle_name',
        default_value=default_turtle_name,
        description='Nome da tartaruga a ser spawnada'
    )
    turtle_x_arg = DeclareLaunchArgument(
        'x',
        default_value='5.0',
        description='Posição X inicial da tartaruga'
    )
    turtle_y_arg = DeclareLaunchArgument(
        'y',
        default_value='5.0',
        description='Posição Y inicial da tartaruga'
    )
    turtle_theta_arg = DeclareLaunchArgument(
        'theta',
        default_value='0.0',
        description='Orientação inicial da tartaruga (em radianos)'
    )

    # Ações do launch
    check_and_launch_sim = OpaqueFunction(function=check_simulator_running)
    launch_teleop = OpaqueFunction(function=launch_teleop_with_params)

    return LaunchDescription([
        turtle_name_arg,
        turtle_x_arg,
        turtle_y_arg,
        turtle_theta_arg,
        check_and_launch_sim,
        launch_teleop,
    ])
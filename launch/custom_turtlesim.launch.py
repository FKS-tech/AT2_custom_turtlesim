#!/usr/bin/env python3




from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess
import sys

def check_simulator_running(context, *args, **kwargs):
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
    except subprocess.CalledProcessError:  # Adicionado except
        print("⚠️ Erro ao verificar nós ROS. Assumindo que o simulador NÃO está rodando.")
        return [
            ExecuteProcess(
                cmd=['ros2', 'run', 'custom_turtlesim', 'simulator_node.py'],
                output='screen'
            )
        ]


def generate_launch_description():
    check_and_launch_sim = OpaqueFunction(function=check_simulator_running)
    return LaunchDescription([
        check_and_launch_sim,
    ])




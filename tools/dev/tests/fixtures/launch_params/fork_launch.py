import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction


def fork(context):
    pid = os.fork()
    # Mutation checks may remove the guard: never let a child run launch/pytest.
    if pid == 0:
        os._exit(0)
    os.waitpid(pid, 0)
    return []


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=fork)])

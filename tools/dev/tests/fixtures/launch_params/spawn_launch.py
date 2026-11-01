import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction


def spawn(context):
    pid = os.spawnlp(os.P_NOWAIT, 'true', 'true')
    # Reap even when the process guard is deliberately removed for a test.
    os.waitpid(pid, 0)
    return []


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=spawn)])

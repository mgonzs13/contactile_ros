# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="contactile_ros",
            executable="contactile_node",
            name="contactile_node",
            namespace="contactile",
            parameters=[{
                "hub_id": LaunchConfiguration("hub_id", default=0),
                "num_sensors": LaunchConfiguration("num_sensors", default=10),
                "baud_rate": LaunchConfiguration("baud_rate", default=9600),
                "parity": LaunchConfiguration("parity", default=0),
                "baud_rate": LaunchConfiguration("baud_rate", default=9600),
                "byte_size": LaunchConfiguration("byte_size", default=8),
                "com_port": LaunchConfiguration("com_port", default="/dev/ttyACM0"),
            }]
        )
    ])

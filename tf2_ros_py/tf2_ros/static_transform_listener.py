# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Wim Meeussen
from typing import Optional
from typing import Union

from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.buffer import Buffer
from tf2_msgs.msg import TFMessage

DEFAULT_TF_TOPIC = '/tf'
DEFAULT_STATIC_TF_TOPIC = '/tf_static'


class StaticTransformListener:
    """
    :class:`StaticTransformListener` is a convenient way to establish a TransformListener on only static topics.
    """

    def __init__(
        self,
        buffer: Buffer,
        node: Node,
        *,
        spin_thread: bool = False,
        static_qos: Optional[Union[QoSProfile, int]] = None,
        tf_static_topic: str = DEFAULT_STATIC_TF_TOPIC
    ) -> None:
        """
        Constructor.

        :param buffer: The buffer to propagate changes to when tf info updates.
        :param node: The ROS2 node.
        :param spin_thread: Whether to create a dedidcated thread to spin this node.
        :param static_qos: A QoSProfile or a history depth to apply to tf_static subscribers.
        :param tf_static_topic: Which topic to listen to for static transforms.
        """
        TransformListener(buffer, node, spin_thread, None, static_qos, DEFAULT_TF_TOPIC,
                          tf_static_topic, True)

    def __del__(self) -> None:
        if hasattr(self, 'dedicated_listener_thread') and hasattr(self, 'executor'):
            self.executor.shutdown()
            self.dedicated_listener_thread.join()

        self.unregister()

    def unregister(self) -> None:
        """
        Unregisters all tf_static subscribers.
        """
        self.node.destroy_subscription(self.tf_static_sub)

    def static_callback(self, data: TFMessage) -> None:
        who = 'default_authority'
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)

# Copyright (c) 2019 Open Source Robotics Foundation, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from tf2_ros import ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.static_transform_listener import StaticTransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener


def build_transform(target_frame, source_frame, stamp):
    transform = TransformStamped()
    transform.header.frame_id = target_frame
    transform.header.stamp = stamp
    transform.child_frame_id = source_frame

    transform.transform.translation.x = 42.0
    transform.transform.translation.y = -3.14
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0

    return transform


class TestBroadcasterAndListener:

    @classmethod
    def setup_class(cls):
        rclpy.init()

        cls.buffer = Buffer()
        cls.node = rclpy.create_node('test_broadcaster_listener')
        cls.broadcaster = TransformBroadcaster(cls.node)
        cls.static_broadcaster = StaticTransformBroadcaster(cls.node)
        cls.listener = TransformListener(
            buffer=cls.buffer, node=cls.node, spin_thread=False)
        cls.static_listener = StaticTransformListener(
            buffer=cls.buffer, node=cls.node, spin_thread=False)

        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls):
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    def setup_method(self, method):
        self.buffer = Buffer()
        self.listener.buffer = self.buffer
        self.static_listener.buffer = self.buffer

    def broadcast_transform(self, target_frame, source_frame, time_stamp):
        broadcast_transform = build_transform(
            target_frame=target_frame, source_frame=source_frame, stamp=time_stamp)

        self.broadcaster.sendTransform(broadcast_transform)

        self.executor.spin_once()

        return broadcast_transform

    def broadcast_static_transform(self, target_frame, source_frame, time_stamp):
        broadcast_transform = build_transform(
            target_frame=target_frame, source_frame=source_frame, stamp=time_stamp)

        self.static_broadcaster.sendTransform(broadcast_transform)

        self.executor.spin_once()

        return broadcast_transform

    def test_extrapolation_exception(self):
        self.broadcast_transform(
            target_frame='foo', source_frame='bar',
            time_stamp=rclpy.time.Time(seconds=0.3, nanoseconds=0).to_msg())

        self.broadcast_transform(
            target_frame='foo', source_frame='bar',
            time_stamp=rclpy.time.Time(seconds=0.2, nanoseconds=0).to_msg())

        with pytest.raises(ExtrapolationException) as excinfo:
            self.buffer.lookup_transform(
                target_frame='foo', source_frame='bar',
                time=rclpy.time.Time(seconds=0.1, nanoseconds=0).to_msg())

        assert 'Lookup would require extrapolation into the past' in str(excinfo.value)

        with pytest.raises(ExtrapolationException) as excinfo:
            self.buffer.lookup_transform(
                target_frame='foo', source_frame='bar',
                time=rclpy.time.Time(seconds=0.4, nanoseconds=0).to_msg())

        assert 'Lookup would require extrapolation into the future' in str(excinfo.value)

    def static_transfrom_listener_rclpy_node(self):
        node = rclpy.create_node('test_broadcaster_listener')
        buffer = Buffer()

        tfl = StaticTransformListener(buffer=buffer, node=node, spin_thread=False)
        assert tfl == self.static_listener

    def test_broadcaster_and_listener(self):
        time_stamp = rclpy.time.Time(seconds=1, nanoseconds=0).to_msg()

        broadcasted_transform = self.broadcast_transform(
            target_frame='foo', source_frame='bar', time_stamp=time_stamp)

        listened_transform = self.buffer.lookup_transform(
            target_frame='foo', source_frame='bar', time=time_stamp)

        assert broadcasted_transform == listened_transform

        # Execute a coroutine
        listened_transform_async = None
        coro = self.buffer.lookup_transform_async(
            target_frame='foo', source_frame='bar', time=time_stamp)
        try:
            coro.send(None)
        except StopIteration as e:
            # The coroutine finished; store the result
            coro.close()
            listened_transform_async = e.value

        assert broadcasted_transform == listened_transform_async

    def test_static_broadcaster_and_listener(self):
        broadcasted_tf = self.broadcast_static_transform(
            target_frame='foo', source_frame='bar',
            time_stamp=rclpy.time.Time(seconds=1.1, nanoseconds=0).to_msg())

        listened_tf = self.buffer.lookup_transform(
            target_frame='foo', source_frame='bar',
            time=rclpy.time.Time(seconds=1.5, nanoseconds=0).to_msg())

        assert broadcasted_tf.header.stamp.sec == 1
        assert broadcasted_tf.header.stamp.nanosec == 100000000

        assert listened_tf.header.stamp.sec == 1
        assert listened_tf.header.stamp.nanosec == 500000000

        assert broadcasted_tf.header.frame_id == listened_tf.header.frame_id
        assert broadcasted_tf.child_frame_id == listened_tf.child_frame_id
        assert broadcasted_tf.transform.translation == listened_tf.transform.translation
        assert broadcasted_tf.transform.rotation == listened_tf.transform.rotation

#!/usr/bin/env python3
"""Display the tracker CompressedImage topic without image_transport."""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CompressedImageViewer(Node):
    def __init__(
        self, topic: str, window_name: str, mjpeg_stdout: bool = False
    ) -> None:
        super().__init__('aruco_compressed_image_viewer')
        self.window_name = window_name
        self.frame = None
        self.mjpeg_stdout = mjpeg_stdout
        self.subscription = self.create_subscription(
            CompressedImage,
            topic,
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, message: CompressedImage) -> None:
        if self.mjpeg_stdout:
            try:
                sys.stdout.buffer.write(message.data)
                sys.stdout.buffer.flush()
            except BrokenPipeError:
                rclpy.shutdown()
            return
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is not None:
            self.frame = frame


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--topic', default='/aruco/annotated/compressed'
    )
    parser.add_argument('--window', default='ROSAIA ArUco detection')
    parser.add_argument('--mjpeg-stdout', action='store_true')
    options = parser.parse_args()

    rclpy.init()
    node = CompressedImageViewer(
        options.topic, options.window, options.mjpeg_stdout
    )
    if not options.mjpeg_stdout:
        cv2.namedWindow(options.window, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if options.mjpeg_stdout:
                continue
            if node.frame is not None:
                cv2.imshow(options.window, node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    finally:
        if not options.mjpeg_stdout:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

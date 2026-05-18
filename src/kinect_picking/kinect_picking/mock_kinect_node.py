#!/usr/bin/env python3
"""
Mock Kinect Node  —  Simulation Mode
════════════════════════════════════════════════════════════════════════════
Publishes synthetic RGB + depth frames on /kinect/rgb and /kinect/depth.
No physical Kinect or freenect library required.

Simulated scene (3 objects at depth = 0.65 m):
  1. Pink   Circle   — pixel centre (200, 200)
  2. Green  Square   — pixel centre (420, 280)
  3. Pink   Triangle — pixel centre (300, 360)

Color values are chosen so they land inside the detection_node's HSV ranges:
  Pink  BGR=(180,20,255) → HSV H≈160  [range 140-180]  ✓
  Green BGR=(30,200,30)  → HSV H≈60   [range  35-85]   ✓

With cam_tx=0.5, cam_ty=0.1, cam_tz=-0.2 the coord_transform output is:
  Circle   → robot frame (≈0.35, 0.15, 0.45)  dist ≈ 0.58 m  ← reachable ✓
  Square   → robot frame (≈0.66, 0.26, 0.45)  dist ≈ 0.80 m  ← reachable ✓
  Triangle → robot frame (≈0.47, 0.30, 0.45)  dist ≈ 0.62 m  ← reachable ✓
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# ── Scene constants ──────────────────────────────────────────────────────── #
PINK  = (180,  20, 255)   # BGR, HSV H≈160 → within detection range 140-180
GREEN = ( 30, 200,  30)   # BGR, HSV H≈60  → within detection range  35-85
DEPTH_M = 0.65            # simulated depth for every object [metres]

# Each row: (color_bgr, shape_type, cx_px, cy_px, size_px)
# size_px → radius for circle, half-side for square, half-height for triangle
# All areas >> 2 000 px²  (circle r=55 → 9 503 px², square 110²=12 100 px²)
SCENE = [
    (PINK,  'circle',   200, 200, 55),
    (GREEN, 'square',   420, 280, 55),
    (PINK,  'triangle', 300, 360, 60),
]


# ═══════════════════════════════════════════════════════════════════════════ #
class MockKinectNode(Node):
    """
    Drop-in replacement for kinect_node in simulation.
    Registers under the same node name ('kinect_node') and publishes to the
    same topics (/kinect/rgb  and  /kinect/depth) so the rest of the pipeline
    (detection_node, depth_extractor_node, …) needs zero changes.
    """

    def __init__(self):
        super().__init__('kinect_node')     # same name as the real driver
        self.bridge = CvBridge()
        self.rgb_pub   = self.create_publisher(Image, '/kinect/rgb',   10)
        self.depth_pub = self.create_publisher(Image, '/kinect/depth', 10)
        self._t = 0.0                       # animation clock
        self.create_timer(1.0 / 10.0, self._publish)   # 10 Hz

        self.get_logger().info(
            'Mock Kinect node started  [SIMULATION — no Kinect hardware needed]')
        self.get_logger().info(
            f'  Publishing {len(SCENE)} synthetic shapes at {DEPTH_M} m depth')
        for (col, sh, cx, cy, sz) in SCENE:
            col_name = 'Pink' if col == PINK else 'Green'
            self.get_logger().info(
                f'    {col_name:5s} {sh:8s}  centre=({cx},{cy})  size={sz}px')

    # ── Frame generation ─────────────────────────────────────────────────── #

    def _draw_rgb(self) -> np.ndarray:
        """640×480 BGR image with coloured shapes.  Slight wobble for freshness."""
        img = np.full((480, 640, 3), 25, dtype=np.uint8)   # near-black background

        for (col, shape, cx0, cy0, sz) in SCENE:
            # Small sine-wave offset so the pipeline sees a "live" stream
            cx = cx0 + int(4 * math.sin(self._t * 0.4))
            cy = cy0 + int(4 * math.cos(self._t * 0.5))
            r  = sz

            if shape == 'circle':
                cv2.circle(img, (cx, cy), r, col, -1)

            elif shape == 'square':
                cv2.rectangle(img,
                              (cx - r, cy - r),
                              (cx + r, cy + r),
                              col, -1)

            elif shape == 'triangle':
                pts = np.array([
                    [cx,     cy - r],
                    [cx - r, cy + r],
                    [cx + r, cy + r],
                ], dtype=np.int32)
                cv2.fillPoly(img, [pts], col)

        return img

    def _draw_depth(self) -> np.ndarray:
        """640×480 float32 depth image in metres.  Objects at DEPTH_M, rest = 0."""
        depth = np.zeros((480, 640), dtype=np.float32)

        for (_, shape, cx0, cy0, sz) in SCENE:
            cx = cx0 + int(4 * math.sin(self._t * 0.4))
            cy = cy0 + int(4 * math.cos(self._t * 0.5))
            r  = sz + 15     # patch slightly larger than the drawn shape

            y1, y2 = max(0, cy - r), min(480, cy + r)
            x1, x2 = max(0, cx - r), min(640, cx + r)
            depth[y1:y2, x1:x2] = DEPTH_M

        return depth

    # ── Timer callback ───────────────────────────────────────────────────── #

    def _publish(self):
        self._t += 0.1
        now = self.get_clock().now().to_msg()

        rgb_msg = self.bridge.cv2_to_imgmsg(self._draw_rgb(), encoding='bgr8')
        rgb_msg.header.stamp    = now
        rgb_msg.header.frame_id = 'kinect_rgb'
        self.rgb_pub.publish(rgb_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(self._draw_depth(), encoding='32FC1')
        depth_msg.header.stamp    = now
        depth_msg.header.frame_id = 'kinect_depth'
        self.depth_pub.publish(depth_msg)


# ═══════════════════════════════════════════════════════════════════════════ #
def main(args=None):
    rclpy.init(args=args)
    node = MockKinectNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import time
import math

def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z), 2.0*(x*z + w*y)],
        [2.0*(x*y + w*z), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x)],
        [2.0*(x*z - w*y), 2.0*(y*z + w*x), 1.0 - 2.0*(x*x + y*y)]
    ]

class TFPrinter(Node):
    def __init__(self):
        super().__init__('tf_printer')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.get_logger().info("TFPrinter started, waiting for link3 -> end_effector_link...")

    def on_timer(self):
        try:
            t = self.buffer.lookup_transform('link3', 'end_effector_link', rclpy.time.Time())
            trans = t.transform.translation
            rot = t.transform.rotation
            dist = math.sqrt(trans.x**2 + trans.y**2 + trans.z**2)
            self.get_logger().info(f"Transform link3 -> end_effector_link:")
            self.get_logger().info(f"  Translation: [{trans.x:.6f}, {trans.y:.6f}, {trans.z:.6f}] (distance: {dist:.6f})")
            self.get_logger().info(f"  Rotation (xyzw): [{rot.x:.6f}, {rot.y:.6f}, {rot.z:.6f}, {rot.w:.6f}]")

            # Print rotation matrix
            R = quaternion_to_matrix(rot)
            self.get_logger().info(f"  Rotation Matrix:")
            self.get_logger().info(f"    [{R[0][0]:.6f}, {R[0][1]:.6f}, {R[0][2]:.6f}]")
            self.get_logger().info(f"    [{R[1][0]:.6f}, {R[1][1]:.6f}, {R[1][2]:.6f}]")
            self.get_logger().info(f"    [{R[2][0]:.6f}, {R[2][1]:.6f}, {R[2][2]:.6f}]")

            # Also lookup end_effector_link -> link3 in ee frame
            t_inv = self.buffer.lookup_transform('end_effector_link', 'link3', rclpy.time.Time())
            trans_inv = t_inv.transform.translation
            self.get_logger().info(f"Transform end_effector_link -> link3:")
            self.get_logger().info(f"  Translation: [{trans_inv.x:.6f}, {trans_inv.y:.6f}, {trans_inv.z:.6f}]")

            rclpy.shutdown()
        except Exception as e:
            self.get_logger().warn(f"Waiting: {e}")

def main():
    rclpy.init()
    node = TFPrinter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

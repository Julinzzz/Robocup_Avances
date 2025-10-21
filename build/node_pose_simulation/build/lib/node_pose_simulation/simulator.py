#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class PoseSimulator(Node):
    def __init__(self):
        super().__init__('node_pose_simulation')
        self.publisher_ = self.create_publisher(Int8, '/person_detected', 10)
        self.timer = self.create_timer(30.0, self.toggle_state)
        self.current_state = 0
        self.get_logger().info("Nodo de simulación iniciado. Alternando 0/1 cada 30 segundos")

    def toggle_state(self):
        self.current_state = 1 - self.current_state
        msg = Int8()
        msg.data = self.current_state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado: {self.current_state}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

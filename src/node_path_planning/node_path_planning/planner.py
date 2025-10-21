#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PathPlanner(Node):
    def __init__(self):
        super().__init__('node_path_planning')
        self.publisher_ = self.create_publisher(String, '/movement_commands', 10)
        self.sequence = ["adelante", "izquierda", "derecha", "atras"]
        self.current_index = 0
        self.timer = self.create_timer(15.0, self.publish_command)
        self.get_logger().info("Nodo de planificación de ruta iniciado. Secuencia cada 15 segundos")

    def publish_command(self):
        msg = String()
        msg.data = self.sequence[self.current_index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Comando publicado: {msg.data}')
        
        # Avanzar al siguiente comando (cíclico)
        self.current_index = (self.current_index + 1) % len(self.sequence)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

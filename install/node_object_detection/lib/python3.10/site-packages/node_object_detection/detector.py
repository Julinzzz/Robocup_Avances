#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('node_object_detection')
        self.publisher_ = self.create_publisher(String, '/detected_objects', 10)
        self.objects = ["bolsa", "lata", "camisa", "gancho", "celular"]
        self.current_index = 0
        self.timer = self.create_timer(30.0, self.publish_object)
        self.get_logger().info("Nodo de detección de objetos iniciado. Alternando entre 5 objetos cada 30 segundos")

    def publish_object(self):
        msg = String()
        msg.data = self.objects[self.current_index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Objeto detectado: {msg.data}')
        
        # Avanzar al siguiente objeto (cíclico)
        self.current_index = (self.current_index + 1) % len(self.objects)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

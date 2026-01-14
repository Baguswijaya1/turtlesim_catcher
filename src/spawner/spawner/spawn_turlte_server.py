import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import numpy as np

class Spawner_server(Node):
    def __init__(self):
        super().__init__('spawner_server')
        self.create_service(
            srv_type=Spawn,
            srv_name='spawn_turlte',
            callback=self.spawn()
        )

    def spawn(self, request:Spawn.Request, response:Spawn.Response):
        response.name = 'newTurtle'
        self.get_logger().info(f'new turtle sawned at x={request.x} y={request.y}')
        return response

    def kill(self, request:Kill.Request):
        request.name = 'newTurtle'

def main(args=None):
    rclpy.init(args=args)
    node = Spawner_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
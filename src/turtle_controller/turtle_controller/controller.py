import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
import numpy as np

Kp_linear = 2
Kp_angular = 4
catch_distance = 0.5
# first_try = True

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.x = 0.
        self.y = 0.
        self.yaw = 0.
        self.linear_vel = 0.
        self.angular_vel = 0.
        self.get_logger().info('controller started')
        self.first_try = True

        self.get_pose_sub = self.create_subscription(
            msg_type=Pose,
            topic='/turtle1/pose',
            callback=self.get_pose,
            qos_profile=10
        )

        self.get_destination_sub = self.create_subscription(
            msg_type=Pose,
            topic = '/newTurtle/pose',
            callback=self.move,
            qos_profile=10
        )

        self.move_pub = self.create_publisher(
            Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )

        self.kill_client = self.create_client(
            Kill, '/kill'
        )

        self.spawn_client = self.create_client(
            Spawn, '/spawn'
        )

        if self.first_try:
            self.autoSpawn()
            self.first_try = False

    def get_pose(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta
        self.linear_vel = msg.linear_velocity
        self.angular_vel = msg.angular_velocity

    def move(self, destination):
        error_x = destination.x - self.x
        error_y = destination.y - self.y
        error_linear = np.sqrt(error_x**2 + error_y**2)

        desired_angle = np.arctan2(error_y, error_x)
        error_angular = desired_angle - self.yaw
        error_angular = np.arctan2(np.sin(error_angular), np.cos(error_angular)) # normalize to [-pi, pi]


        msg = Twist()
        msg.linear.x = Kp_linear * error_linear
        msg.angular.z = Kp_angular * error_angular

        self.move_pub.publish(msg)

        if error_linear <= catch_distance:
            self.get_logger().info('caught newTurtle')
            self.kill('newTurtle')

    def kill(self, turtle_name):
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Kill service not available')
            
        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client.call_async(request)
        future.add_done_callback(self.callback_kill)

    def callback_kill(self, future):
        try:
            response = future.result()  
            self.get_logger().info(' newTurtle elimintaed')
            self.autoSpawn()
        except Exception as e:
            self.get_logger().info(f'kill service failed {e}')

    def autoSpawn(self):
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('spawn service not available')
            
        request = Spawn.Request()
        request.name = 'newTurtle'
        request.x =  np.random.uniform(1.,10.)
        request.y =  np.random.uniform(1.,10.)
        request.theta = np.random.uniform(0., 3.14)
        request.name = 'newTurtle'

        future = self.spawn_client.call_async(request)

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.callback_spawn)

    def callback_spawn(self, future):
        try:
            response = future.result()  
            self.get_logger().info(' newTurtle spawned')
            turtle_killed += 1
        except Exception as e:
            self.get_logger().info(f'spawn service failed {e}')

        
        

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
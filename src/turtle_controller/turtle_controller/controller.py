import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
import numpy as np

# change these values to set turtle's sensitivity
Kp_linear = 4
Ki_linear = 0.01
Kd_linear = 2

Kp_angular = 12
Ki_angular = 0.2
Kd_angular = 7

catch_distance = 0.5

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


        self.dt = 0.4
        self.t = 0

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
        # linear errors
        error_x = destination.x - self.x
        error_y = destination.y - self.y
        error_linear = np.sqrt(error_x**2 + error_y**2)
        
        if self.t == 0:
            self.past_error_linear = 0
            self.e_int_linear = 0
            self.past_e_int_linear = 0
            self.e_dot_linear = 0

            self.past_error_angular = 0
            self.e_int_angular = 0
            self.past_e_int_angular = 0
            self.e_dot_angular = 0
        
        e_int_linear = self.past_e_int_linear + (self.past_error_linear + error_linear) / 2 * self.dt
        e_dot_linear = (error_linear - self.past_error_linear) / self.dt
        
        # angular errors
        desired_angle = np.arctan2(error_y, error_x)
        error_angular = desired_angle - self.yaw
        error_angular = np.arctan2(np.sin(error_angular), np.cos(error_angular))

        
        e_int_angular = self.past_e_int_angular + (self.past_error_angular + error_angular) / 2 * self.dt
        e_dot_angular = (error_angular - self.past_error_angular) / self.dt

        # Implement PID
        msg = Twist()
        msg.linear.x = Kp_linear * error_linear + Ki_linear * e_int_linear + Kd_linear * e_dot_linear
        msg.angular.z = Kp_angular * error_angular + Ki_angular  * e_int_angular + Kd_angular * e_dot_angular

        self.past_error_linear = error_linear
        self.past_error_angular = error_angular
        self.past_e_int_linear = e_int_linear
        self.past_e_int_angular = e_int_angular
        self.t += self.dt

        self.move_pub.publish(msg)

        if error_linear <= catch_distance:
            self.get_logger().info('caught newTurtle')
            self.kill('newTurtle')
            self.past_e_int_linear = 0;
            self.past_e_int_angular = 0;

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
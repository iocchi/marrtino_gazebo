import time, threading, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class MARRtinoController(Node):

    def __init__(self):
        super().__init__('marrtino_control_client')
        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/marrtino_controller/cmd_vel', 10)
        self.get_logger().info(f'MARRtino Controller Node initialized ')
        
        self.rate10 = self.create_rate(10) # Hz
        self.rate100 = self.create_rate(100) # Hz
        self.sub_odom = self.create_subscription(
            Odometry,           # Message type
            '/marrtino_controller/odom',      # Topic name
            self.odom_callback, # Callback function
            10                  # QoS (Quality of Service) history depth
        )
        self.sub_cmd_vel = self.create_subscription(
            TwistStamped,           # Message type
            '/marrtino_controller/cmd_vel',      # Topic name
            self.cmd_vel_callback,  # Callback function
            10                      # QoS (Quality of Service) history depth
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        vx = msg.twist.twist.linear.x
        #print(f"odom x: {x:.3f} vx: {vx:.3f}")

    def cmd_vel_callback(self, msg):
        pass
        #print(f"cmd_vel {msg.twist.linear.x:.3f} {msg.twist.angular.z:.3f}")

    def publish_cmd_vel(self, lx, az, ts=1):
        msg = TwistStamped()
        msg.twist.linear.x = lx
        msg.twist.angular.z = az
        self.get_logger().info(f'Publishing cmd_vel: {lx:.3f} {az:.3f}')
        for _ in range(ts*100):
            self.pub_cmd_vel.publish(msg)
            self.rate100.sleep()
        for _ in range(10):   # wait to complete the movement
            self.rate100.sleep()

    def test1(self):
        for _ in range(4):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,math.pi/8,4)
        

def main(args=None):
    rclpy.init(args=args)
    marrtino_controller = MARRtinoController()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(marrtino_controller, ), daemon=True)
    thread.start()

    marrtino_controller.test1()
    
    marrtino_controller.publish_cmd_vel(0.0,0.0)

    marrtino_controller.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()


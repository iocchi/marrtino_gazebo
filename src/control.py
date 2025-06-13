import time, threading, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

import matplotlib.pyplot as plt

ROBOT_NAME='marrtino'

class MARRtinoController(Node):

    def __init__(self):
        super().__init__('marrtino_control_client')
        self.pub_cmd_vel = self.create_publisher(TwistStamped, f'/{ROBOT_NAME}_controller/cmd_vel', 10)
        self.get_logger().info(f'MARRtino Controller Node initialized ')
        
        self.rate10 = self.create_rate(10) # Hz
        self.rate100 = self.create_rate(100) # Hz
        
        # Data storage for plotting
        self.odom_ts = []
        self.poses = [[],[],[]]
        self.velocities = [[],[]]        
        self.inputs_ts = []
        self.inputs = [[],[]]

        # reference values
        self.ts0 = None
        self.ts = 0

        self.sub_odom = self.create_subscription(
            Odometry,           # Message type
            f'/{ROBOT_NAME}_controller/odom',      # Topic name
            self.odom_callback, # Callback function
            10                  # QoS (Quality of Service) history depth
        )
        self.sub_cmd_vel = self.create_subscription(
            TwistStamped,           # Message type
            f'/{ROBOT_NAME}_controller/cmd_vel',      # Topic name
            self.cmd_vel_callback,  # Callback function
            10                      # QoS (Quality of Service) history depth
        )


    def odom_callback(self, msg):
        if self.ts0 is None:
            self.ts0 = msg.header.stamp.sec + msg.header.stamp.nanosec/1.0e9
        self.ts = msg.header.stamp.sec + msg.header.stamp.nanosec/1.0e9 - self.ts0
        self.odom_ts.append(self.ts)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        vx = msg.twist.twist.linear.x
        az = msg.twist.twist.angular.z

        self.poses[0].append(x)
        self.poses[1].append(y)
        self.poses[2].append(yaw)
        self.velocities[0].append(vx)
        self.velocities[1].append(az)
        #print(f"odom x: {x:.3f} vx: {vx:.3f}")

    def cmd_vel_callback(self, msg):
        self.inputs_ts.append(self.ts)
        self.inputs[0].append(msg.twist.linear.x)
        self.inputs[1].append(msg.twist.angular.z)

    def publish_cmd_vel(self, lx, az, ts=1):
        msg = TwistStamped()
        msg.twist.linear.x = lx
        msg.twist.angular.z = az
        self.get_logger().info(f'Publishing cmd_vel: {lx:.3f} {az:.3f} time: {ts:.2f} s')
        for _ in range(ts*100+10):
            self.pub_cmd_vel.publish(msg)
            self.rate100.sleep()
        for _ in range(10):   # wait to complete the movement
            self.rate100.sleep()

    def subplot(self, axs, ts, values, label, color):
        axs.plot(ts, values, label=label, color=color)
        axs.set_ylabel(label)
        axs.grid(True)
        axs.legend()
    


    def plot_ctrl(self):

        fig, axs = plt.subplots(7, 1, sharex=True, figsize=(14, 8))
        fig.suptitle(f'Input/Output Velocities & Positions')

        self.subplot(axs[0], self.inputs_ts, self.inputs[0], "input linear vel", color='red')
        self.subplot(axs[1], self.inputs_ts, self.inputs[1], "input angular vel", color='red')
        self.subplot(axs[2], self.odom_ts, self.velocities[0], "linear vel", color='blue')
        self.subplot(axs[3], self.odom_ts, self.velocities[1], "angular vel", color='blue')
        self.subplot(axs[4], self.odom_ts, self.poses[0], "position x", color='green')
        self.subplot(axs[5], self.odom_ts, self.poses[1], "position y", color='green')
        self.subplot(axs[6], self.odom_ts, self.poses[2], "orientation th", color='green')
        
        axs[-1].set_xlabel('Time (s)')

        plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout to prevent title overlap
        plt.show()
        self.get_logger().info("Plot displayed. Close the plot window to terminate the script.")


    def plot_traj(self):

        plt.title(f'Trajectory')
        
        plt.plot(self.poses[0], self.poses[1])

        plt.show()
        self.get_logger().info("Plot displayed. Close the plot window to terminate the script.")

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

    marrtino_controller.plot_ctrl()
    marrtino_controller.plot_traj()

if __name__ == '__main__':
    main()


import time, threading, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 

from tf_transformations import euler_from_quaternion

from rcl_interfaces.srv import GetParameters, ListParameters
from rcl_interfaces.msg import ParameterType

import matplotlib.pyplot as plt

ROBOT_NAME='marrtino'




class MARRtinoController(Node):

    def __init__(self):
        super().__init__('marrtino_control_client')

        # only node local access        
        robot_name = self.get_ext_parameters(['robot_name'])[0]

        # self.set_parameters([Parameter('new_parameter', Parameter.Type.STRING, 'New value')])



        self.pub_cmd_vel = self.create_publisher(TwistStamped, f'/{ROBOT_NAME}_controller/cmd_vel', 10)
        self.pub_arm_effort = self.create_publisher(Float64MultiArray, f'/arm_effort_controller/commands', 100)
        self.get_logger().info(f'{ROBOT_NAME} controller node initialized ')

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



    def get_ext_parameters(self, param_names):

        self.target_node_name = 'marrtino_parameters'

        self.get_parameters_client = self.create_client(
            GetParameters,
            f'{self.target_node_name}/get_parameters'
        )
        
        # --- Blocking call to get specific parameters ---
        get_request = GetParameters.Request()
        # Request the specific parameters you are interested in
        get_request.names = param_names
        get_future = self.get_parameters_client.call_async(get_request)
        # The "single blocking instruction" for the get call:
        rclpy.spin_until_future_complete(self, get_future)

        if not get_future.done():
            self.get_logger().warn('GetParameters future did not complete.')
            return

        values = []

        try:
            get_response = get_future.result()
            self.get_logger().info(f'Received parameter values from {self.target_node_name}:')
            self.get_logger().info(f'  {get_response}')
            for i,param in enumerate(get_response.values):

                param_name = param_names[i]
                param_type = param.type
                param_value = None

                # Extract value based on type
                if param_type == ParameterType.PARAMETER_BOOL:
                    param_value = param.bool_value
                elif param_type == ParameterType.PARAMETER_INTEGER:
                    param_value = param.integer_value
                elif param_type == ParameterType.PARAMETER_DOUBLE:
                    param_value = param.double_value
                elif param_type == ParameterType.PARAMETER_STRING:
                    param_value = param.string_value
                # Add more types if needed (e.g., array types)

                self.get_logger().info(f'  {param_name}: {param_value}')
                
                values.append(param_value)

        except Exception as e:
            self.get_logger().error(f'Service call failed for get_parameters: {e}')


        return values



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
        for _ in range(int(ts*100+10)):
            self.pub_cmd_vel.publish(msg)
            self.rate100.sleep()
        for _ in range(10):   # wait to complete the movement
            self.rate100.sleep()

    def publish_arm_effort(self, fl, fr, ts=1):
        msg = Float64MultiArray()
        msg.data = [fl, fr]
        self.get_logger().info(f'Publishing effort: {fl:.3f} {fr:.3f}')
        for _ in range(int(ts*100)):
            self.pub_arm_effort.publish(msg)
            self.rate100.sleep()
        for _ in range(10):   # wait to complete the movement
            self.rate100.sleep()

    def sleep(self, ts=1):
        for _ in range(ts*10):
            self.rate10.sleep()
    


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
            self.publish_cmd_vel(0.3,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.0,math.pi/12,6)
            self.publish_cmd_vel(0.0,0.0,0.5)

    def test2(self):
        self.publish_arm_effort(-0.2, -0.2, 5)
        self.publish_arm_effort(0.1, 0.1, 7)
        self.publish_arm_effort(0.0, 0.0)

    def test3(self):
        k = 1
        self.publish_arm_effort(0.1, 0.1, 3)
        self.publish_arm_effort(0, 0, 0.5)
        for _ in range(4):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.0,math.pi/12,6)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_arm_effort(-k*0.2, k*0.2, 5)
            self.publish_arm_effort(0, 0, 0.5)
            k *= -1        

        # lower both arms
        self.publish_arm_effort(0.1, 0.1, 3)
        self.publish_arm_effort(0, 0, 0.5)

def main(args=None):
    rclpy.init(args=args)
    marrtino_controller = MARRtinoController()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(marrtino_controller, ), daemon=True)
    thread.start()

    marrtino_controller.test1()
    
    marrtino_controller.publish_cmd_vel(0.0,0.0)
    marrtino_controller.publish_arm_effort(0.0, 0.0)

    marrtino_controller.destroy_node()
    rclpy.shutdown()
    thread.join()

    #marrtino_controller.plot_ctrl()
    #marrtino_controller.plot_traj()

if __name__ == '__main__':
    main()


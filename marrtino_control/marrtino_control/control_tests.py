import time, threading, math, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 

from tf_transformations import euler_from_quaternion

import rcl_interfaces.msg

from rcl_interfaces.srv import GetParameters, ListParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

import matplotlib.pyplot as plt

NODE_NAME = 'marrtino_control_client'

class MARRtinoController(Node):

    def __init__(self):
        
        rclpy.init()

        super().__init__(NODE_NAME)

        params = self.get_ext_parameters(['robot_name', 'control_interface'])
        self.robot_name = params[0]
        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.control_interface = params[1]
        self.get_logger().info(f'control_interface: {self.control_interface}')

        self.declare_parameter('fn', 'square')
        self.fn = self.get_parameter('fn').value
        self.get_logger().info(f'control function: {self.fn}')

        self.declare_parameter('plot', 'none')
        self.toplot = self.get_parameter('plot').value
        self.get_logger().info(f'plot: {self.toplot}')
        
        # publishers
        self.pub_cmd_vel = self.create_publisher(TwistStamped, f'/diff_drive_controller/cmd_vel', 10)
        self.pub_arm_cmd = self.create_publisher(Float64MultiArray, f'/arm_{self.control_interface}_controller/commands', 100)
        self.pub_head_cmd = self.create_publisher(Float64MultiArray, f'/head_{self.control_interface}_controller/commands', 100)

        # position joint limits
        self.head_pan_limit = math.pi/2 - 0.001
        self.head_tilt_limit = math.pi/4 - 0.001
        if self.robot_name == 'smarrtino':
            self.arm_lower_limit = -5*math.pi/4 + 0.001
            self.arm_upper_limit = math.pi/4 - 0.001
        else:
            self.arm_lower_limit = -5*math.pi/4 + 0.001
            self.arm_upper_limit = math.pi/4 - 0.001

        # rates
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

        # user request to stop the robot
        self.user_stop = False


        self.sub_odom = self.create_subscription(
            Odometry,           # Message type
            f'/diff_drive_controller/odom',      # Topic name
            self.odom_callback, # Callback function
            10                  # QoS (Quality of Service) history depth
        )
        self.sub_cmd_vel = self.create_subscription(
            TwistStamped,           # Message type
            f'/diff_drive_controller/cmd_vel',      # Topic name
            self.cmd_vel_callback,  # Callback function
            10                      # QoS (Quality of Service) history depth
        )

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()

        self.rate10.sleep()

    
        os.system(f"ros2 param set {NODE_NAME} use_sim_time True")

        '''  DOES NOT WORK !!!
        pp = Parameter('use_sim_time', ParameterType.PARAMETER_BOOL, True)
        self.set_parameters([pp])
        '''    

        self.rate10.sleep()
 
        current_use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f"Current use_sim_time value: {current_use_sim_time}")

        self.get_logger().info(f'{self.robot_name} controller node initialized ')



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
        msg.twist.linear.x = float(lx)
        msg.twist.angular.z = float(az)
        self.get_logger().info(f'Publishing cmd_vel: {lx:.3f} {az:.3f} time: {ts:.2f} s')
        for _ in range(int(ts*100)):
            self.pub_cmd_vel.publish(msg)
            self.rate100.sleep()
            if self.user_stop:
                break
        if self.user_stop:
            self.user_stop = False
            self.stop()

    
    def setSpeed(self, lx, az, ts=1):
        self.publish_cmd_vel(lx, az, ts)

    def publish_arm_command(self, fl, fr, ts=1, interface=None):
        if interface is None:
            interface = self.control_interface
        if self.control_interface == interface:
        
            if interface=='position':
                if fl < self.arm_lower_limit:
                    fl = self.arm_lower_limit
                if fl > self.arm_upper_limit:
                    fl = self.arm_upper_limit
                if fr < self.arm_lower_limit:
                    fr = self.arm_lower_limit
                if fr > self.arm_upper_limit:
                    fr = self.arm_upper_limit

            msg = Float64MultiArray()
            msg.data = [float(fl), float(fr)]
            self.get_logger().info(f'Publishing arm {interface}: {fl:.3f} {fr:.3f}')
            for _ in range(int(ts*100)):
                self.pub_arm_cmd.publish(msg)
                self.rate100.sleep()
                if self.user_stop:
                    break
            if self.user_stop:
                self.user_stop = False
                self.stop()
        else:
            self.get_logger().warning(f'Cannot send arm {interface} command! Current controller is {self.control_interface}.')


    def publish_head_command(self, hpan, htilt, ts=1, interface=None):
        if interface is None:
            interface = self.control_interface

        if self.control_interface == interface:
        
            if interface=='position':
                if hpan < -self.head_pan_limit:
                    hpan = -self.head_pan_limit
                if hpan > self.head_pan_limit:
                    hpan = self.head_pan_limit
                if htilt < -self.head_tilt_limit:
                    htilt = -self.head_tilt_limit
                if htilt > self.head_tilt_limit:
                    htilt = self.head_tilt_limit

            msg = Float64MultiArray()
            msg.data = [float(hpan), float(htilt)]
            self.get_logger().info(f'Publishing head {interface}: {hpan:.3f} {htilt:.3f}')
            for _ in range(int(ts*100)):
                self.pub_head_cmd.publish(msg)
                self.rate100.sleep()
                if self.user_stop:
                    break
            if self.user_stop:
                self.user_stop = False
                self.stop()
        else:
            self.get_logger().warning(f'Cannot send head {interface} command! Current controller is {self.control_interface}.')




    def sleep(self, ts=1):
        for _ in range(int(ts*10)):
            self.rate10.sleep()
    


    def subplot(self, axs, ts, values, label, color):
        axs.plot(ts, values, label=label, color=color)
        axs.set_ylabel(label)
        axs.grid(True)
        axs.legend()
    


    def plot_velctrl(self):

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


    def plot_odom(self):

        plt.title(f'Odometry')
        
        plt.plot(self.poses[0], self.poses[1])

        plt.show()
        self.get_logger().info("Plot displayed. Close the plot window to terminate the script.")

    def quit(self):
        self.destroy_node()
        rclpy.shutdown()


    def run(self):
        if self.fn != 'none':
            eval(f'self.{self.fn}()')
        else:
            print('No control function!')

    def stop(self):
        self.publish_cmd_vel(0.0, 0.0)
        if self.robot_name!='marrtino' and self.control_interface in ['effort', 'velocity']:
            self.publish_arm_command(0.0, 0.0)
        if self.robot_name=='smarrtino' and self.control_interface in ['effort', 'velocity']:
            self.publish_head_command(0.0, 0.0)
        self.rate10.sleep()
        self.user_stop = False

    def square(self):
        for _ in range(4):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.0,math.pi/8,4)
            self.publish_cmd_vel(0.0,0.0,0.5)
        self.stop()
        
        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()
        

    def circle(self):
        tm = 10
        r = 1.0
        vx = 0.2
        tm = 2 * math.pi * r / vx
        az = 2 * math.pi / tm

        self.publish_cmd_vel(vx,az,tm)
        self.stop()
        
        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()
        
    # relative forward/backward
    def forward(self, m):
        lx = 0.2
        if (m<0):
            lx *= -1
        tm = abs(m) / abs(lx)
        self.setSpeed(lx,0,tm)
        self.setSpeed(0,0,0.2)

    # relative turn
    def turn(self, deg):
        az = 0.5
        if (deg<0):
            az *= -1
        tm = abs(deg)/180.0*math.pi / abs(az)
        self.setSpeed(0,az,tm)
        self.setSpeed(0,0,0.2)


    def arms(self):
        if self.control_interface == 'effort':
            self.publish_arm_command(-0.2, -0.2, 5)
            self.publish_arm_command(0.1, 0.1, 7)
            self.publish_arm_command(0.0, 0.0)

        elif self.control_interface == 'velocity':
            self.publish_arm_command(-0.5, -0.5, 5)
            self.publish_arm_command(0.5, 0.5, 7)
            self.publish_arm_command(0.0, 0.0)

        if self.control_interface == 'position':
            self.publish_arm_command(self.arm_lower_limit, self.arm_lower_limit, 5)
            self.publish_arm_command(self.arm_upper_limit, self.arm_upper_limit, 7)
            self.publish_arm_command(0.0, 0.0, 2)

        self.stop()


    def head(self):
        if self.control_interface == 'effort':
            self.publish_head_command(-0.1, 0, 2)
            self.publish_head_command(+0.1, 0, 4)
            self.publish_head_command(-0.1, 0, 2)
            self.publish_head_command(0, -0.1, 1)
            self.publish_head_command(0, +0.1, 2)
            self.publish_head_command(0, 0)
        elif self.control_interface == 'velocity':
            self.publish_head_command(-0.7, 0, 2)
            self.publish_head_command(+0.7, 0, 4)
            self.publish_head_command(-0.7, 0, 2)
            self.publish_head_command(0, -0.5, 1)
            self.publish_head_command(0, +0.5, 2)
            self.publish_head_command(0, 0)
        elif self.control_interface == 'position':
            self.publish_head_command(-self.head_pan_limit, 0, 3)
            self.publish_head_command(+self.head_pan_limit, 0, 6)
            self.publish_head_command(0, 0, 3)
            self.publish_head_command(0, -self.head_tilt_limit, 2)
            self.publish_head_command(0, +self.head_tilt_limit, 5)
            self.publish_head_command(0, 0, 2)

        self.stop()


    def all(self):
        k = 1
        self.publish_arm_command(0.1, 0.1, 3)
        self.publish_arm_command(0, 0, 0.5)
        for _ in range(4):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.0,math.pi/8,4)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_arm_command(-k*0.2, k*0.2, 5)
            self.publish_arm_command(0, 0, 0.5)
            self.publish_head_command(-0.1, 0, 1)
            self.publish_head_command(+0.1, 0, 2)
            self.publish_head_command(-0.1, 0, 1)
            self.publish_head_command(0, 0)
            k *= -1        

        # lower both arms
        self.publish_arm_command(0.1, 0.1, 3)
        self.publish_arm_command(0, 0, 0.5)

        self.publish_head_command(0, -0.1, 1)
        self.publish_head_command(0, +0.1, 2)
        self.publish_head_command(0, 0)

        self.stop()

        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()

    # my custom open-loop control functions
    def rhombus(self):
        beta:float = math.pi/6
        alpha:float = math.pi - beta
        walk_time:int = 5
        turn_time:int = 4
        stall_time:float = 0.5
        for _ in range(2):
            self.publish_cmd_vel(0.0, (alpha/2)/turn_time, turn_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
            self.publish_cmd_vel(0.2,0.0,walk_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
            self.publish_cmd_vel(0.0,beta/turn_time,turn_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
            self.publish_cmd_vel(0.2,0.0,walk_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
            self.publish_cmd_vel(0.0, (alpha/2)/turn_time, turn_time)
            #self.publish_cmd_vel(0.0,0.0,stall_time)
        self.stop()
        
        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()

    def regular_poly(self, n=0):
        '''Executes the appropriate commands to make the robot follow an n-sided regular polygon trajectory.'''
        if n == 0:
            if not self.has_parameter('n'):
                self.declare_parameter('n', 0)
            n = self.get_parameter('n').value
        
        if n < 3:
            self.get_logger().error(f"Could not perform action! Invalid sides value provided (it must be at least 3, but the provided value was {n}).")
            return
        
        walk_time:int = 5
        turn_time:int = 4
        stall_time:float = 0.5

        angle:float = math.pi - (1 - 2/n) * math.pi
        self.get_logger().info(f"Turn angle is {angle}")

        for _ in range(n):
            self.publish_cmd_vel(0.2,0.0,walk_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
            self.publish_cmd_vel(0, angle/turn_time, turn_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
        
        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()

    def triangle(self):
        self.regular_poly(3)

    def square_alt(self):
        self.regular_poly(4)
    
    def pentagon(self):
        self.regular_poly(5)
    
    def hexagon(self):
        self.regular_poly(6)

    def dome(self):
        for i in range(3):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            if i < 2:
                self.publish_cmd_vel(0.0,math.pi/8,4)
                self.publish_cmd_vel(0.0,0.0,0.5)
        self.publish_cmd_vel(0.3142,math.pi/5,5)
        self.publish_cmd_vel(0.0,0.0,0.5)
        self.publish_cmd_vel(0.0,math.pi/8,4)
        self.publish_cmd_vel(0.0,0.0,0.5)

        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()

    def pill(self):
        for _ in range(2):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.3142,-math.pi/5,5)
            self.publish_cmd_vel(0.0,0.0,0.5)

        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()

    def flower(self, n=0):
        '''Executes the appropriate commands to make the robot follow a flower-shaped trajectory.'''
        if n == 0:
            if not self.has_parameter('n'):
                self.declare_parameter('n', 0)
            n = self.get_parameter('n').value
        
        if n < 3:
            self.get_logger().error(f"Could not perform action! Invalid petals amount provided (it must be at least 3, but the provided value was {n}).")
            return
        
        walk_time:int = 5
        turn_time:int = 4
        stall_time:float = 0.5

        angle:float = (1 - 2/n) * math.pi
        self.get_logger().info(f"Turn angle is {angle}")

        speed:float = math.pi*0.5/5

        for _ in range(n):
            self.publish_cmd_vel(0.2,-math.pi/walk_time,walk_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
            self.publish_cmd_vel(0, angle/turn_time, turn_time)
            self.publish_cmd_vel(0.0,0.0,stall_time)
        
        if 'velctrl' in self.toplot:
            self.plot_velctrl()
        if 'odom' in self.toplot:
            self.plot_odom()

def main(args=None):
    
    robot = MARRtinoController()

    robot.run()

    robot.quit()


if __name__ == '__main__':
    main()


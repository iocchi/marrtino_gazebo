import time, threading, math, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 
from sensor_msgs.msg import JointState, LaserScan

from tf_transformations import euler_from_quaternion

import rcl_interfaces.msg

from rcl_interfaces.srv import GetParameters, ListParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

import matplotlib.pyplot as plt

def euler_from_orientation(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    return euler_from_quaternion(orientation_list)


# managing concurrent action execution
class ActionFuture:

    def __init__(self):
        self.thread = None
        self.stop_event = None
        self.result = None
        self.callback_fn = None
    
    # start the activity in a separate thread (non-blocking)
    def start(self, target, args, daemon=True):
        assert self.thread is None, "ActionFuture cannot start a new activity on an active thread !!!"
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=target, args=args, daemon=daemon)
        self.thread.start()

    # to be called only by target function upon termination
    def notify_termination(self, result=None):
        self.result = result
        self.thread = None
        self.stop_event = None
        if self.callback_fn is not None:
            self.callback_fn(self.result)

    # request to stop
    def stop_request(self):
        if self.stop_event is not None:
            self.stop_event.set()      

    # check if running
    def is_running(self):
        return self.thread is not None and self.thread.is_alive()

    # wait until termination (blocking)
    def wait(self):
        if self.thread is not None:
            self.thread.join()

    # callback on termination
    def set_callback(self, fn):
        self.callback_fn = fn



NODE_NAME = 'marrtino_control_client'

class MARRtinoController(Node):

    def __init__(self):
        
        rclpy.init()

        super().__init__(NODE_NAME)

        # Data storage for plotting
        self.reset_data()
        self.plot_data_collect = False

        # reference values
        self.ts0 = None
        self.ts = 0

        # Callback values
        self.odom = None
        self.joint_states = None
        self.gtpose = None
        self.scan = None

        # joint name-id map
        self.jointid = None

        # user request to stop the robot
        self.user_stop = False

        # simulated speech variables
        self.simulated_say = None
        self.simulated_asr = None

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()
        time.sleep(0.5)

        print("Setting use_sim_time to True ...")
        os.system(f"ros2 param set {NODE_NAME} use_sim_time True")
        '''  DOES NOT WORK !!!
        pp = Parameter('use_sim_time', ParameterType.PARAMETER_BOOL, True)
        self.set_parameters([pp])
        '''    

        print(f"use_sim_time = {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")

        # Wait for /clock to be active if using sim time
        print("Waiting for clock ...")
        if self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            self.get_logger().info('Using simulated time. Waiting for /clock to be active...')
            while not self.get_clock().now().to_msg().sec > 0 and rclpy.ok():
                time.sleep(0.1) # Use real-time sleep while waiting for sim time
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(f"Simulated time is now active: {self.get_clock().now().to_msg().sec} seconds")

        # rates
        rate10 = self.create_rate(10) # Hz
        rate10.sleep()

        # Parameters

        params = self.get_ext_parameters(['robot_name', 'control_interface','individual_arm_control'])
        self.robot_name = params[0]
        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.control_interface = params[1]
        self.get_logger().info(f'control_interface: {self.control_interface}')
        self.individual_arm_control = params[2]
        self.get_logger().info(f'individual_arm_control: {self.individual_arm_control}')

        self.declare_parameter('fn', 'square')
        self.fn = self.get_parameter('fn').value
        self.get_logger().info(f'control function: {self.fn}')

        self.declare_parameter('plot', 'none')
        self.toplot = self.get_parameter('plot').value
        self.get_logger().info(f'plot: {self.toplot}')

        # position joint limits

        self.head_pan_limit = math.pi/2 - 0.001
        self.head_tilt_limit = math.pi/4 - 0.001
        if self.robot_name == 'smarrtino':
            self.arm_lower_limit = -5*math.pi/4 + 0.001
            self.arm_upper_limit = math.pi/4 - 0.001
        else:
            self.arm_lower_limit = -5*math.pi/4 + 0.001
            self.arm_upper_limit = math.pi/4 - 0.001

        # publishers

        self.pub_cmd_vel = self.create_publisher(TwistStamped, f'/diff_drive_controller/cmd_vel', 10)
        if self.individual_arm_control:
            self.pub_lshp_cmd = self.create_publisher(Float64MultiArray, f'/left_shoulder_pitch_{self.control_interface}_controller/commands', 100)
            self.pub_rshp_cmd = self.create_publisher(Float64MultiArray, f'/right_shoulder_pitch_{self.control_interface}_controller/commands', 100)
        else:
            self.pub_arm_cmd = self.create_publisher(Float64MultiArray, f'/arm_{self.control_interface}_controller/commands', 100)
        self.pub_head_pan_cmd = self.create_publisher(Float64MultiArray, f'/head_pan_{self.control_interface}_controller/commands', 100)
        self.pub_head_tilt_cmd = self.create_publisher(Float64MultiArray, f'/head_tilt_{self.control_interface}_controller/commands', 100)

        # subscribers

        self.sub_gtpose = self.create_subscription(
            PoseStamped,           # Message type
            f'/model/{self.robot_name}/pose',      # Topic name
            self.gtpose_callback, # Callback function
            10                  # QoS (Quality of Service) history depth
        )
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
        self.sub_joints = self.create_subscription(
            JointState,           # Message type
            '/joint_states',      # Topic name
            self.joint_states_callback, # Callback function
            10                    # QoS (Quality of Service) history depth
        )
        self.sub_scan = self.create_subscription(
            LaserScan,    # Message type
            '/scan',      # Topic name
            self.scan_callback, # Callback function
            10                    # QoS (Quality of Service) history depth
        )

        rate10.sleep()
 
        self.get_logger().info(f'{self.robot_name} controller node initialized ')


    def reset_data(self):
        self.gt_ts = []
        self.odom_ts = []
        self.gtposes = [[],[],[]]
        self.odomposes = [[],[],[]]
        self.velocities = [[],[]]        
        self.inputs_ts = []
        self.inputs = [[],[]]


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


    def set_ts(self, msg):
        if self.ts0 is None:
            self.ts0 = msg.header.stamp.sec + msg.header.stamp.nanosec/1.0e9
            print(f"ts0 = {self.ts0}")
        self.ts = msg.header.stamp.sec + msg.header.stamp.nanosec/1.0e9 - self.ts0
        return self.ts  # needed in callbacks as it can changes during other callbacks

    # Callbacks

    def gtpose_callback(self, msg):
        if msg.header.frame_id == 'default':
            ts = self.set_ts(msg)
            self.gtpose = msg

            if self.plot_data_collect:
                self.gt_ts.append(ts)
                x = self.gtpose.pose.position.x
                y = self.gtpose.pose.position.y
                (_, _, yaw) = euler_from_orientation(self.gtpose.pose.orientation)

                self.gtposes[0].append(x)
                self.gtposes[1].append(y)
                self.gtposes[2].append(yaw)
                '''
                print(f"ts: {msg.header.stamp.sec} ", end="")
                print(f"frame_id: {msg.header.frame_id}", end="")
                self.print_gtpose()
                '''


    def odom_callback(self, msg):
        ts = self.set_ts(msg)       
        self.odom = msg

        if self.plot_data_collect:
            self.odom_ts.append(ts)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            (roll, pitch, yaw) = euler_from_orientation(msg.pose.pose.orientation)
            vx = msg.twist.twist.linear.x
            az = msg.twist.twist.angular.z

            self.odomposes[0].append(x)
            self.odomposes[1].append(y)
            self.odomposes[2].append(yaw)
            self.velocities[0].append(vx)
            self.velocities[1].append(az)

    def cmd_vel_callback(self, msg):
        ts = self.set_ts(msg)
        if self.plot_data_collect:
            self.inputs_ts.append(ts)
            self.inputs[0].append(msg.twist.linear.x)
            self.inputs[1].append(msg.twist.angular.z)

    def joint_states_callback(self, msg):
        self.set_ts(msg)
        self.joint_states = msg
        if self.jointid is None:
            self.jointid = {}
            for i, joint_name in enumerate(self.joint_states.name):
                self.jointid[joint_name] = i


    def scan_callback(self, msg):
        self.scan = msg

    # print functions

    def print_odom(self):
        print_once = True
        rate10 = self.create_rate(10) # Hz
        while self.odom is None: 
            rate10.sleep()
            if print_once:
                print_once = False
                print("Waiting for odom message ...")
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        (_, _, th) = euler_from_orientation(self.odom.pose.pose.orientation)
        vx = self.odom.twist.twist.linear.x
        az = self.odom.twist.twist.angular.z
        print(f"odom x: {x:.3f} y: {y:.3f} th:{th:.3f} rad - vel linear: {vx:.3f} m/s angular: {az:.3f} rad/s")

    def print_gtpose(self):
        print_once = True
        rate10 = self.create_rate(10) # Hz
        while self.gtpose is None:
            rate10.sleep()
            if print_once:
                print_once = False
                print("Waiting for gtpose message ...")
        x = self.gtpose.pose.position.x
        y = self.gtpose.pose.position.y
        (_, _, th) = euler_from_orientation(self.gtpose.pose.orientation)
        print(f"gtpose x: {x:.3f} y: {y:.3f} th:{th:.3f} rad")

    def print_joint_states(self):
        print_once = True
        rate10 = self.create_rate(10) # Hz
        while self.joint_states is None: 
            rate10.sleep()
            if print_once:
                print_once = False
                print("Waiting for joint_states message ...")
        for i, joint_name in enumerate(self.joint_states.name):
            self.get_logger().info(
                f' [{self.ts:.3f}] Joint: {joint_name}'
                f' | Position: {self.joint_states.position[i]:.4f}'
                f' | Velocity: {self.joint_states.velocity[i]:.4f}'
                f' | Effort: {self.joint_states.effort[i]:.4f}'
            )

    def print_scan(self):
        print(f"scan num points = {len(self.scan.ranges)}")
        print(f"     min-max angle = {self.scan.angle_min}, {self.scan.angle_max} - inc {self.scan.angle_increment} [rad]")

        np = (self.scan.angle_max - self.scan.angle_min) / self.scan.angle_increment
        print(f"     mp = {np}")

        for deg in [-90, 0, 90]:
            r = deg/180.0*math.pi
            i = int((r - self.scan.angle_min) / self.scan.angle_increment)
            if (i>len(self.scan.ranges)):
                i = -1
            d = self.scan.ranges[i]
            print(f"     {deg} -> range[{i}] = {d}")

    # Publishers (with stop_event handling)
    # stop_event is a threading.Event object
    # to stop the publishing behavior use stop_event.set()

    def publish_cmd_vel(self, lx, az, ts=1, stop_on_end=False, afuture=None):
        msg = TwistStamped()
        msg.twist.linear.x = float(lx)
        msg.twist.angular.z = float(az)
        self.get_logger().info(f'Publishing cmd_vel: {lx:.3f} {az:.3f} time: {ts:.2f} s')
        rate100 = self.create_rate(100) # Hz
        for _ in range(int(ts*100)):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd_vel.publish(msg)
            rate100.sleep()
            if self.user_stop or (afuture is not None and afuture.stop_event.is_set()):
                break
        if stop_on_end:
            for _ in range(5):
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(msg)
                rate100.sleep()

        if self.user_stop:
            self.stop()

        if afuture is not None:
            afuture.notify_termination()
    

    def publish_arms_command(self, fl, fr, ts=1, afuture=None):
        if self.robot_name in ['smarrtino', 'marrtino_2_arms']:
            interface = self.control_interface        
            if interface=='position':
                if fl < self.arm_lower_limit:
                    fl = self.arm_lower_limit
                if fl > self.arm_upper_limit:
                    fl = self.arm_upper_limit
                if fr < self.arm_lower_limit:
                    fr = self.arm_lower_limit
                if fr > self.arm_upper_limit:
                    fr = self.arm_upper_limit

            if self.individual_arm_control:
                msgl = Float64MultiArray()
                msgl.data = [float(fl)]
                msgr = Float64MultiArray()
                msgr.data = [float(fr)]
                self.get_logger().info(f'Publishing individual arm {interface} left:{fl:.3f} right:{fr:.3f}')
                rate100 = self.create_rate(100) # Hz
                for _ in range(int(ts*100)):
                    self.pub_lshp_cmd.publish(msgl)
                    self.pub_rshp_cmd.publish(msgr)
                    rate100.sleep()
                    if self.user_stop or (afuture is not None and afuture.stop_event.is_set()):
                        break
            else:
                msg = Float64MultiArray()
                msg.data = [float(fl), float(fr)]
                self.get_logger().info(f'Publishing arm {interface}: {fl:.3f} {fr:.3f}')
                rate100 = self.create_rate(100) # Hz
                for _ in range(int(ts*100)):
                    self.pub_arm_cmd.publish(msg)
                    rate100.sleep()
                    if self.user_stop or (stop_event is not None and stop_event.is_set()):
                        break
            if self.user_stop:
                self.stop()

        if afuture is not None:
            afuture.notify_termination()


    def publish_left_shoulder_pitch_command(self, val, ts, afuture=None):
        self._publish_onearm_command(self.pub_lshp_cmd, val, ts, afuture)

    def publish_right_shoulder_pitch_command(self, val, ts, afuture=None):
        self._publish_onearm_command(self.pub_rshp_cmd, val, ts, afuture)

    def _publish_onearm_command(self, pub, val, ts=1, afuture=None):
        if self.robot_name in ['smarrtino', 'marrtino_2_arms']:
            interface = self.control_interface
            
            if interface=='position':
                if val < self.arm_lower_limit:
                    val = self.arm_lower_limit
                if val > self.arm_upper_limit:
                    val = self.arm_upper_limit

            msg = Float64MultiArray()
            msg.data = [float(val)]
            which = 'left' if pub==self.pub_lshp_cmd else 'right'
            self.get_logger().info(f'Publishing {which} arm {interface}: {val:.3f}')
            rate100 = self.create_rate(100) # Hz
            for _ in range(int(ts*100)):
                pub.publish(msg)
                rate100.sleep()
                if self.user_stop or (afuture is not None and afuture.stop_event.is_set()):
                    break
            if self.user_stop:
                self.stop()

        if afuture is not None:
            afuture.notify_termination()


    def publish_head_pan_command(self, value, ts=1, afuture=None):
        self.publish_head_command(self.pub_head_pan_cmd, value, ts, afuture)

    def publish_head_tilt_command(self, value, ts=1, afuture=None):
        self.publish_head_command(self.pub_head_tilt_cmd, value, ts, afuture)



    def publish_head_command(self, pub, value, ts=1, afuture=None):
        if self.robot_name == 'smarrtino':
            interface = self.control_interface

            which = 'pan' if pub==self.pub_head_pan_cmd else 'tilt'

            if interface=='position':
                if which == 'pan':
                    if value < -self.head_pan_limit:
                        value = -self.head_pan_limit
                    if value > self.head_pan_limit:
                        value = self.head_pan_limit
                else: # tilt
                    if value < -self.head_tilt_limit:
                        value = -self.head_tilt_limit
                    if value > self.head_tilt_limit:
                        value = self.head_tilt_limit

            msg = Float64MultiArray()
            msg.data = [float(value)]
            self.get_logger().info(f'Publishing head {which} {interface}: {value:.3f}')
            rate100 = self.create_rate(100) # Hz
            for _ in range(int(ts*100)):
                pub.publish(msg)
                rate100.sleep()
                if self.user_stop or (afuture is not None and afuture.stop_event.is_set()):
                    break
            if self.user_stop:
                self.stop()

        if afuture is not None:
            afuture.notify_termination()


    # sleep function

    def sleep(self, ts=1):
        rate10 = self.create_rate(10) # Hz
        for _ in range(int(ts*10)):
            rate10.sleep()
            # rclpy.spin_once(self, timeout_sec=0.1)
    

    # Plot functions

    def subplot(self, axs, ts, values, label, color):
        axs.plot(ts, values, label=label, color=color)
        axs.set_ylabel(label)
        axs.grid(True)
        axs.legend()
    


    def plot_velctrl(self):

        fig, axs = plt.subplots(5, 1, sharex=True, figsize=(14, 6))
        fig.suptitle(f'Input/Output Velocities & Positions')

        #self.subplot(axs[0], self.inputs_ts, self.inputs[0], "input linear vel", color='red')
        #self.subplot(axs[1], self.inputs_ts, self.inputs[1], "input angular vel", color='red')
        self.subplot(axs[0], self.odom_ts, self.velocities[0], "linear vel", color='blue')
        self.subplot(axs[1], self.odom_ts, self.velocities[1], "angular vel", color='blue')
        self.subplot(axs[2], self.odom_ts, self.odomposes[0], "position x", color='green')
        self.subplot(axs[3], self.odom_ts, self.odomposes[1], "position y", color='green')
        self.subplot(axs[4], self.odom_ts, self.odomposes[2], "orientation th", color='green')
        
        axs[-1].set_xlabel('Time (s)')

        plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout to prevent title overlap
        plt.show()
        self.get_logger().info("Plot displayed. Close the plot window to terminate the script.")


    def plot_odom(self):

        plt.title(f'Odometry')
        
        print(f"gtposes len: {len(self.gtposes[0])} {len(self.gtposes[1])} {len(self.gtposes[2])}  ")

        plt.plot(self.odomposes[0], self.odomposes[1], label="odom", color='red')
        plt.plot(self.gtposes[0], self.gtposes[1], label="gt", color='green')
        plt.legend()

        plt.show()
        self.get_logger().info("Plot displayed. Close the plot window to terminate the script.")

    def quit(self):
        self.destroy_node()
        rclpy.shutdown()


    # high level control

    def setSpeed(self, lx, az, time=1, stopend=False, _async=False):
        if lx<-1:
            lx = -1.0
        if lx>1:
            lx = 1.0
        if _async:
            afuture = ActionFuture()
            afuture.start(target=self.publish_cmd_vel, args=(lx, az, time, stopend, afuture), daemon=True)
            return afuture
        else:
            self.publish_cmd_vel(lx, az, time, stop_on_end=stopend) # blocking function
            if stopend:
                self.publish_cmd_vel(0, 0, 0.5) # blocking function        
        return None


    def setHeadPanPosition(self, rad, time=1, _async=False):

        if self.control_interface == 'position':
            if _async:
                afuture = ActionFuture()
                afuture.start(target=self.publish_head_pan_command, args=(rad, time, afuture), daemon=True)
                return afuture
            else:
                self.publish_head_pan_command(rad, time)

        return None, None


    def setHeadTiltPosition(self, rad, time=1, _async=False):
        if self.control_interface == 'position':
            if _async:
                afuture = ActionFuture()
                afuture.start(target=self.publish_head_tilt_command, args=(rad, time, afuture), daemon=True)
                return afuture
            else:
                self.publish_head_tilt_command(rad, time)
        return None


    def setLeftArmPosition(self, rad, time=1, _async=False):
        if self.control_interface == 'position':
            if _async:
                afuture = ActionFuture()
                afuture.start(target=self.publish_left_shoulder_pitch_command, args=(rad, time, afuture), daemon=True)
                return afuture
            else:
                self.publish_left_shoulder_pitch_command(rad, time)
        return None

    def setRightArmPosition(self, rad, time=1, _async=False):
        if self.control_interface == 'position':
            if _async:
                afuture = ActionFuture()
                afuture.start(target=self.publish_right_shoulder_pitch_command, args=(rad, time, afuture), daemon=True)
                return afuture
            else:
                self.publish_right_shoulder_pitch_command(rad, time)
        return None

    '''
    def setArmsPosition(self, target_left, target_right, time=1, _async=False):

        if self.control_interface == 'position':
            if _async:
                stop_event = threading.Event()
                thread = threading.Thread(target=self.publish_arms_command, args=(target_left, target_right, time, None, afuture ), daemon=True)
                thread.start()
                return thread, stop_event
            else:
                self.publish_arms_command(target_left, target_right, time)

        elif self.control_interface == 'velocity':
    
            for i, joint_name in enumerate(self.joint_states.name):
                if joint_name=='left_arm_joint':
                    p_left = self.joint_states.position[i]
                if joint_name=='right_arm_joint':
                    p_right = self.joint_states.position[i]

            err_left = target_left-p_left
            err_right = target_right-p_right
            K = 0.5
            while abs(err_left)+abs(err_right)>0.02:
                v_left = K * err_left
                v_right = K * err_right
                self.publish_arms_command(v_left, v_right, 0.1)
                
                for i, joint_name in enumerate(self.joint_states.name):
                    if joint_name=='left_arm_joint':
                        p_left = self.joint_states.position[i]
                    if joint_name=='right_arm_joint':
                        p_right = self.joint_states.position[i]

                print(f"arms pos: {p_left:.3f} {p_right:.3f}")

                err_left = target_left-p_left
                err_right = target_right-p_right

        return None, None
    '''



    # Run behaviors

    def run(self):
        if self.fn != 'none':

            self.print_odom()
            self.print_gtpose()

            self.user_stop = False

            if self.toplot != 'none':
                self.plot_data_collect = True
            if '(' in self.fn:
                eval(f'self.{self.fn}')
            else:
                eval(f'self.{self.fn}()')
            if self.toplot != 'none':
                self.plot_data_collect = False

            self.print_odom()
            self.print_gtpose()

            if 'velctrl' in self.toplot:
                self.plot_velctrl()
            if 'odom' in self.toplot:
                self.plot_odom()

        else:
            print('No control function!')

    def stop(self):

        rate100 = self.create_rate(100) # Hz
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd_vel.publish(msg)
            rate100.sleep()

        if self.robot_name!='marrtino' and self.control_interface in ['effort', 'velocity']:
            msg = Float64MultiArray()
            msg.data = [ 0.0 ]
            for _ in range(3):
                self.pub_lshp_cmd.publish(msg)
                self.pub_rshp_cmd.publish(msg)
                rate100.sleep()

        if self.robot_name=='smarrtino' and self.control_interface in ['effort', 'velocity']:
            msg = Float64MultiArray()
            msg.data = [ 0.0 ]
            for _ in range(3):
                self.pub_head_pan_cmd.publish(msg)
                self.pub_head_tilt_cmd.publish(msg)
                rate100.sleep()

    def square(self):
        for _ in range(4):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.0,math.pi/8,4)
            self.publish_cmd_vel(0.0,0.0,0.5)
        self.stop()

    def circle(self):
        tm = 10
        r = 1.0
        vx = 0.2
        tm = 2 * math.pi * r / vx
        az = 2 * math.pi / tm  # = vx / r

        self.publish_cmd_vel(vx,az,tm)
        self.stop()
        


    def arms(self):
        if self.control_interface == 'effort':
            self.publish_arms_command(-0.2, -0.2, 5)
            self.publish_arms_command(0.1, 0.1, 7)
            self.publish_arms_command(0.0, 0.0)

        elif self.control_interface == 'velocity':
            self.publish_arms_command(-0.5, -0.5, 5)
            self.publish_arms_command(0.5, 0.5, 7)
            self.publish_arms_command(0.0, 0.0)

        if self.control_interface == 'position':

            if self.individual_arm_control:
                self.publish_left_shoulder_pitch_command(self.arm_lower_limit, 5)
                self.publish_right_shoulder_pitch_command(self.arm_lower_limit, 5)
                self.publish_left_shoulder_pitch_command(self.arm_upper_limit, 5)
                self.publish_right_shoulder_pitch_command(self.arm_upper_limit, 5)
                self.publish_left_shoulder_pitch_command(0, 2)
                self.publish_right_shoulder_pitch_command(0, 2)
            else:
                self.publish_arms_command(self.arm_lower_limit, self.arm_lower_limit, 5)
                self.publish_arms_command(self.arm_upper_limit, self.arm_upper_limit, 7)
                self.publish_arms_command(0.0, 0.0, 2)

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
        self.publish_arms_command(0.1, 0.1, 3)
        self.publish_arms_command(0, 0, 0.5)
        for _ in range(4):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.0,math.pi/8,4)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_arms_command(-k*0.2, k*0.2, 5)
            self.publish_arms_command(0, 0, 0.5)
            self.publish_head_command(-0.1, 0, 1)
            self.publish_head_command(+0.1, 0, 2)
            self.publish_head_command(-0.1, 0, 1)
            self.publish_head_command(0, 0)
            k *= -1        

        # lower both arms
        self.publish_arms_command(0.1, 0.1, 3)
        self.publish_arms_command(0, 0, 0.5)

        self.publish_head_command(0, -0.1, 1)
        self.publish_head_command(0, +0.1, 2)
        self.publish_head_command(0, 0)

        self.stop()


    # async call:
    #   -- run and wait until finished 
    #   f = fn(..., _async=True)
    #   f.wait()
    #
    #   -- run and do somthing until finished
    #   f = fn(..., _async=True)
    #   while (f.is_running()):
    #     do ... 
    #   f.wait()
    #
    #   -- run and quit the behavior when condition
    #   f = fn(..., _async=True)
    #   while (whatever and f.is_running()):
    #     do ...
    #     if condition:
    #        f.stop_request()   # this will request to stop the activity
    #   f.join()


    def walk(self):
        i = 0
        while (i<5) and not self.user_stop:
            print(f"---- {i} ----")
            timestep = 3
            f1 = self.setSpeed(0.2, 0, timestep, _async=True)
            a = 1 if i%2==0 else -1
            f2 = self.setLeftArmPosition(a*math.pi/4, timestep, _async=True)
            f3 = self.setRightArmPosition(-a*math.pi/4, timestep, _async=True)
            f4 = self.setHeadPanPosition(a*math.pi/4, timestep, _async=True)
            # waiting for future
            f1.wait()
            f2.wait()
            f3.wait()
            f4.wait()
            i += 1
    
        print("---- stop ----")
        self.stop()
        f1 = self.setLeftArmPosition(0,2, _async=True)
        f2 = self.setRightArmPosition(0,2, _async=True)
        f3 = self.setHeadPanPosition(0,2, _async=True)
        f1.wait()
        f2.wait()
        f3.wait()




    def walk2(self):
        i = 0
        while (i<5) and not self.user_stop: 
            print(f"---- {i} ----")
            timestep = 3
            a = 1 if i%2==0 else -1
            f1 = self.setSpeed(0.2, 0, 100, _async=True)
            f2 = self.setLeftArmPosition(a*math.pi/4, 100, _async=True)
            f3 = self.setRightArmPosition(-a*math.pi/4, 100, _async=True)
            f4 = self.setHeadPanPosition(a*math.pi/4, 100, _async=True)
            # threads running ...
            ts = 0
            dt = 0.5
            while ts<timestep:
                self.sleep(dt)
                ts += dt
                print(f"walking ... {ts}/{timestep}")

            # send stop events to all threads
            f1.stop_request()
            f2.stop_request()
            f3.stop_request()
            f4.stop_request()
            f1.wait()
            f2.wait()
            f3.wait()
            f4.wait()
            i += 1


        print("---- stop ----")
        self.stop()
        f1 = self.setLeftArmPosition(0, 2, _async=True)
        f2 = self.setRightArmPosition(0, 2, _async=True)
        f3 = self.setHeadPanPosition(0, 2, _async=True)
        f1.wait()
        f2.wait()
        f3.wait()



    # robot interface

    '''
    odometry position control
    robot.forward(m)    move forward m [m]
    robot.backward(m)   move backward m [m]
    robot.left(r)       turn left r [deg]
    robot.right(r)      turn right r [deg]
    '''

    # relative forward/backward
    def forward(self, m=1, _async=False):
        lx = 0.2 # linear velocity
        if (m<0):
            lx *= -1
        tm = abs(m) / abs(lx)
        return self.setSpeed(lx,0,tm,stopend=True,_async=_async)

    def backward(self, m=1, _async=False):
        return self.forward(-m, _async=_async)

    # relative turn
    def turn(self, deg=90, _async=False):
        az = 0.5 # angular velocity
        if (deg<0):
            az *= -1
        tm = abs(deg)/180.0*math.pi / abs(az)
        return self.setSpeed(0,az,tm,stopend=True,_async=_async)

    def left(self, deg=90, _async=False):
        return self.turn(deg, _async=_async)

    def right(self, deg=90, _async=False):
        return self.turn(-deg, _async=_async)

    '''
    odometry velocity control
    robot.setSpeed(lx,az,time,stopend=False)
                  lx: linear velocity [m/s]
				  az: angular velocity [rad/s]
                  time: time [s]
                  stopend: stop after motion 

    '''


    # Head position

    '''
    robot.pan(deg): positive left
    robot.tilt(deg): positive up
    '''

    def pan(self, deg, _async=False):
        c = self.get_pan_pos()
        ts = abs(deg-c)/60 + 0.5
        return self.setHeadPanPosition(deg/180*math.pi, ts, _async=_async)

    def tilt(self, deg, _async=False):
        c = self.get_tilt_pos()
        ts = abs(deg-c)/60 + 0.5
        return self.setHeadTiltPosition(-deg/180*math.pi, ts, _async=_async) # positive up


    # Arms

    '''
    robot.left_arm(deg): positive ahead     
    robot.right_arm(deg): positive ahead
    '''

    def left_arm(self, deg, _async=False):
        c = self.get_left_arm_pos()
        ts = abs(deg-c)/60 + 0.5
        return self.setLeftArmPosition(-deg/180*math.pi, ts, _async=_async)

    def right_arm(self, deg, _async=False):
        c = self.get_left_arm_pos()
        ts = abs(deg-c)/60 + 0.5
        return self.setRightArmPosition(-deg/180*math.pi, ts, _async=_async)

    # Wait

    def wait(self, t=1):
        self.sleep(t)


    # Speech
    
    '''
    robot.say(sentence,language)  # language = 'it' | 'en'
    sentence = robot.listen(timeout=5)    
    '''

    def say(self, sentence, language='en'):
        print(f"saying '{sentence}' ...")
        self.simulated_say = sentence
        self.sleep(0.5)  # wait for say message to go through

    def asr(self, timeout=5):
        self.listen(timeout)

    def listen(self, timeout=5):
        t = 0
        dt = 0.5
        print("listening ....")
        while (t<timeout) and self.simulated_asr is None and not self.user_stop:
            self.sleep(dt)
            t += dt
        s = self.simulated_asr
        self.simulated_asr = None
        if s is not None:
            print(f"listened: {s}")
        return s

    # get. robot pose in the specified frame [m, deg]

    def get_pose(self, frame='odom'):
        if frame=='odom':
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            (_, _, th_rad) = euler_from_orientation(self.odom.pose.pose.orientation)
        elif frame=='gt' or frame=='ground_truth':
            x = self.gtpose.pose.pose.position.x
            y = self.gtpose.pose.pose.position.y
            (_, _, th_rad) = euler_from_orientation(self.odom.pose.pose.orientation)
        else:
            print(f"getpose: Unknown frame {frame}")
            return None
        th_deg = th_rad/math.pi*180
        return x,y,th_deg


    # return jount position [deg] and velocity [deg/s]
    def get_joint_pos(self, jname):
        i = self.jointid[jname]
        return self.joint_states.position[i]/math.pi*180 

    def get_joint_vel(self, jname):
        i = self.jointid[jname]
        return self.joint_states.velocity[i]/math.pi*180

    # get joint positions [deg]

    def get_pan_pos(self):
        return self.get_joint_pos('pan_head_joint')
    
    def get_tilt_pos(self):
        return -self.get_joint_pos('tilt_head_joint')   # positive up

    def get_left_arm_pos(self):
        return self.get_joint_pos('left_arm_joint')
    
    def get_right_arm_pos(self):
        return self.get_joint_pos('right_arm_joint')

    # obstacle distance [deg] -> [m]

    def obstacle_distance(self, deg=0):
        r = deg/180.0*math.pi
        i = int((r - self.scan.angle_min) / self.scan.angle_increment)
        if (i>len(self.scan.ranges)):
            i = -1
        if (i<0):
            i = 0
        d = self.scan.ranges[i]
        print(f"Laser scan obstacle {deg} : range[{i}] = {d}")
        return d

    '''
robot.emotion(“normal”)    set normal face 
   robot.emotion(“happy”)     set happy face
   robot.emotion(“sad”)       set sad face 
   robot.emotion(“sings”)     set singing face
   robot.emotion(“surprise”)  set face surprised  
   robot.emotion(“angry”)     set angry face
   



robot.tag_id()     return tag id (??? -1 if not present)
   robot.tag_clean()  clear the tag id variable

   display(robot.tagID()) display value of tag id
   robot.getImage()       display image from camera


    '''


def main(args=None):
    
    robot = MARRtinoController()

    robot.run()

    robot.quit()


if __name__ == '__main__':
    main()


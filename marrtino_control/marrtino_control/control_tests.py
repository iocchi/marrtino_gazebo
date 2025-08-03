import time, threading, math, os
from rclpy.clock import Clock
from marrtino_control.control import MARRtinoController

class MARRtinoController_Test(MARRtinoController):
    def __init__(self):
        super().__init__()
    
    #Open-Loop Control Functions
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

    def pill(self):
        for _ in range(2):
            self.publish_cmd_vel(0.2,0.0,5)
            self.publish_cmd_vel(0.0,0.0,0.5)
            self.publish_cmd_vel(0.3142,-math.pi/5,5)
            self.publish_cmd_vel(0.0,0.0,0.5)

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

    #Closed-Loop Controllers
    def __PID(self, system, target, Kp=0.5, Ki=0.3, Kd=0.05):
        '''Universal PID Controller. --> WIP'''

        self.get_logger().info("PID Thread Started!")
        current_pos = self.joint_states.position[system]
        
        error = target-current_pos
        integral = 0
        derivative = 0

        prev_err = 0

        dt = 0.01

        while abs(error) > 0.01:
            integral += error * dt
            derivative = (error - prev_err) / dt

            control_output = Kp * error + Ki * integral + Kd * derivative

            prev_err = error
            current_pos = self.joint_states.position[system]
            error = target - current_pos
        
        self.get_logger().info("PID Thread Ended!")
    
    def __setArmsPosition(self, target_left, target_right, continuous_mode=False):
        '''Arms PID Controller'''

        if self.control_interface == 'effort': #reads position, controls effort
            for i, joint_name in enumerate(self.joint_states.name):
                if joint_name=='left_arm_joint':
                    p_left = self.joint_states.position[i]
                    effort_left = 0 #self.joint_states.effort[i]
                if joint_name=='right_arm_joint':
                    p_right = self.joint_states.position[i]
                    effort_right = 0 #self.joint_states.effort[i]

            err_left = target_left-p_left
            err_right = target_right-p_right
            
            #PID Gains
            Kp_left = 0.5   #Proportional Gain
            Ki_left = 0.4   #Integral Gain
            Kd_left = 0.2   #Derivative Gain

            Kp_right = 0.5   #Proportional Gain
            Ki_right = 0.4   #Integral Gain
            Kd_right = 0.2   #Derivative Gain
            
            integral_left = 0
            integral_right = 0

            derivative_left = 0
            derivative_right = 0
            prev_err_left = 0
            prev_err_right = 0

            dt = 0.01
            bound = 0.1

            while abs(err_left) + abs(prev_err_left) > 0.01 or abs(err_right) + abs(prev_err_right) > 0.01:
                if abs(err_left) + abs(prev_err_left) < 0.1:
                    Kp_left = 0.0
                    Ki_left = 0.1
                    Kd_left = 0.4
                elif abs(err_left) + abs(prev_err_left) < 0.2:
                    Kp_left = 0.05
                    Ki_left = 0.2
                    Kd_left = 0.4
                else:
                    Kp_left = 0.5
                    Ki_left = 0.4
                    Kd_left = 0.2

                if abs(err_right) + abs(prev_err_right) < 0.1:
                    Kp_right = 0.0
                    Ki_right = 0.1
                    Kd_right = 0.4
                elif abs(err_right) + abs(prev_err_right) < 0.2:
                    Kp_right = 0.05
                    Kp_right = 0.2
                    Kp_right = 0.4
                else:
                    Kp_right = 0.5
                    Ki_right = 0.4
                    Kd_right = 0.2
                
                #saturators
                err_left = max(err_left, -bound)
                err_left = min(err_left, bound)
                err_right = max(err_right, -bound)
                err_right = min(err_right, bound)

                integral_left = max(integral_left, -bound)
                integral_left = min(integral_left, bound)
                integral_right = max(integral_right, -bound)
                integral_right = min(integral_right, bound)

                derivative_left = max(derivative_left, -bound)
                derivative_left = min(derivative_left, bound)
                derivative_right = max(derivative_right, -bound)
                derivative_right = min(derivative_right, bound)

                #control computation
                integral_left += err_left * dt
                integral_right += err_right * dt

                derivative_left = (err_left - prev_err_left) / dt
                derivative_right = (err_right - prev_err_right) / dt

                #self.get_logger().info(f"effort_left: [P: {Kp_left} * {err_left}, I: {Ki_left} * {integral_left}, D: {Kd_left} * {derivative_left}]\n effort_right: [P: {Kp_right} * {err_right}, I: {Ki_right} * {integral_right}, D: {Kd_right} * {derivative_right}]")

                effort_left = Kp_left * err_left + Ki_left * integral_left + Kd_left * derivative_left
                effort_right = Kp_right * err_right + Ki_right * integral_right + Kd_right * derivative_right
                
                self.publish_arm_command(effort_left, effort_right, dt)
                
                for i, joint_name in enumerate(self.joint_states.name):
                    if joint_name=='left_arm_joint':
                        p_left = self.joint_states.position[i]
                    if joint_name=='right_arm_joint':
                        p_right = self.joint_states.position[i]

                print(f"arms pos: {p_left:.3f} {p_right:.3f}")

                prev_err_left = err_left
                prev_err_right = err_right
                err_left = target_left-p_left
                err_right = target_right-p_right

        elif self.control_interface == 'velocity': #reads position, controls velocity
            for i, joint_name in enumerate(self.joint_states.name):
                if joint_name=='left_arm_joint':
                    p_left = self.joint_states.position[i]
                if joint_name=='right_arm_joint':
                    p_right = self.joint_states.position[i]
            
            err_left = target_left-p_left
            err_right = target_right-p_right
            
            #PID Gains
            Kp = 0.5    #Proportional Gain
            Ki = 0.3    #Integral Gain
            Kd = 0.05    #Derivative Gain
            
            integral_left = 0
            integral_right = 0

            derivative_left = 0
            derivative_right = 0
            prev_err_left = 0
            prev_err_right = 0

            dt = 0.01

            v_left = 0
            v_right = 0

            while continuous_mode or (abs(err_left) + abs(prev_err_left) >0.01 or abs(err_right) + abs(prev_err_right) > 0.01):

                integral_left += err_left * dt
                integral_right += err_right * dt
                
                derivative_left = (err_left - prev_err_left) / dt
                derivative_right = (err_right - prev_err_right) / dt

                #self.get_logger().info(f"v_left: [P: {Kp} * {err_left}, I: {Ki} * {integral_left}, D: {Kd} * {derivative_left}]\n v_right: [P: {Kp} * {err_right}, I: {Ki} * {integral_right}, D: {Kd} * {derivative_right}]")

                v_left = Kp * err_left + Ki * integral_left + Kd * derivative_left
                v_right = Kp * err_right + Ki * integral_right + Kd * derivative_right    
                
                self.publish_arm_command(v_left, v_right, dt)
                
                for i, joint_name in enumerate(self.joint_states.name):
                    if joint_name=='left_arm_joint':
                        p_left = self.joint_states.position[i]
                    if joint_name=='right_arm_joint':
                        p_right = self.joint_states.position[i]

                print(f"arms pos: {p_left:.3f} {p_right:.3f}")

                prev_err_left = err_left
                prev_err_right = err_right
                err_left = target_left-p_left
                err_right = target_right-p_right
            
            self.publish_arm_command(0, 0, 0.1) #Prevents the arms to keep moving after target position is reached.

    def position_arms(self, left=0.0, right=0.0):
        '''Executes the Arms Controller in order to make them reach a specified position.'''
        
        if not self.has_parameter('l'):
            if self.control_interface == 'effort':
                self.declare_parameter('l', left)
            elif self.control_interface == 'velocity':
                self.declare_parameter('l', left)
        left = self.get_parameter('l').value
        
        if not self.has_parameter('r'):
            if self.control_interface == 'effort':
                self.declare_parameter('r', right)
            elif self.control_interface == 'velocity':
                self.declare_parameter('r', right)
        right = self.get_parameter('r').value
        
        self.__setArmsPosition(left, right)
    
    def __setHeadPosition(self, target_pan, target_tilt, continuous_mode=False):
        '''Head PID Controller'''

        if self.control_interface == 'effort': #reads position, controls effort
            for i, joint_name in enumerate(self.joint_states.name):
                    if joint_name=='pan_head_joint':
                        p_pan = self.joint_states.position[i]
                    if joint_name=='tilt_head_joint':
                        p_tilt = self.joint_states.position[i]

            err_left = target_pan-p_pan
            err_right = target_tilt-p_tilt
            
            #PID Gains
            Kp_pan = 0.1   #Proportional Gain
            Ki_pan = 0.4   #Integral Gain
            Kd_pan = 0.05   #Derivative Gain

            Kp_tilt = 0.1   #Proportional Gain
            Ki_tilt = 0.4   #Integral Gain
            Kd_tilt = 0.05   #Derivative Gain
            
            integral_pan = 0
            integral_tilt = 0

            derivative_pan = 0
            derivative_tilt = 0
            prev_err_left = 0
            prev_err_right = 0

            dt = 0.1
            bound = 0.5

            while abs(err_left) + abs(prev_err_left) > 0.01 or abs(err_right) + abs(prev_err_right) > 0.01:
                #saturators
                err_left = max(err_left, -bound)
                err_left = min(err_left, bound)
                err_right = max(err_right, -bound)
                err_right = min(err_right, bound)

                integral_pan = max(integral_pan, -bound)
                integral_pan = min(integral_pan, bound)
                integral_tilt = max(integral_tilt, -bound)
                integral_tilt = min(integral_tilt, bound)

                derivative_pan = max(derivative_pan, -bound)
                derivative_pan = min(derivative_pan, bound)
                derivative_tilt = max(derivative_tilt, -bound)
                derivative_tilt = min(derivative_tilt, bound)

                #control computation
                integral_pan += err_left * dt
                integral_tilt += err_right * dt

                derivative_pan = (err_left - prev_err_left) / dt
                derivative_tilt = (err_right - prev_err_right) / dt

                #self.get_logger().info(f"effort_left: [P: {Kp_left} * {err_left}, I: {Ki_left} * {integral_left}, D: {Kd_left} * {derivative_left}]\n effort_right: [P: {Kp_right} * {err_right}, I: {Ki_right} * {integral_right}, D: {Kd_right} * {derivative_right}]")

                effort_pan = Kp_pan * err_left + Ki_pan * integral_pan + Kd_pan * derivative_pan
                effort_tilt = Kp_tilt * err_right + Ki_tilt * integral_tilt + Kd_tilt * derivative_tilt
                
                self.publish_head_command(effort_pan, effort_tilt, dt)
                
                for i, joint_name in enumerate(self.joint_states.name):
                    if joint_name=='pan_head_joint':
                        p_pan = self.joint_states.position[i]
                    if joint_name=='tilt_head_joint':
                        p_tilt = self.joint_states.position[i]

                print(f"head pos: {p_pan:.3f} {p_tilt:.3f}")

                prev_err_left = err_left
                prev_err_right = err_right
                err_left = target_pan-p_pan
                err_right = target_tilt-p_tilt
        
        elif self.control_interface == 'velocity':
            for i, joint_name in enumerate(self.joint_states.name):
                if joint_name=='pan_head_joint':
                    p_pan = self.joint_states.position[i]
                if joint_name=='tilt_head_joint':
                    p_tilt = self.joint_states.position[i]
            
            err_pan = target_pan-p_pan
            err_tilt = target_tilt-p_tilt
            
            #PID Gains
            Kp = 0.8    #Proportional Gain
            Ki = 0.4    #Integral Gain
            Kd = 0.1    #Derivative Gain
            
            integral_pan = 0
            integral_tilt = 0

            derivative_pan = 0
            derivative_tilt = 0
            prev_err_pan = 0
            prev_err_tilt = 0

            dt = 0.01

            v_pan = 0
            v_tilt = 0

            while continuous_mode or (abs(err_pan) + abs(prev_err_pan) >0.01 or abs(err_tilt) + abs(prev_err_tilt) > 0.01):

                integral_pan += err_pan * dt
                integral_tilt += err_tilt * dt
                
                derivative_pan = (err_pan - prev_err_pan) / dt
                derivative_tilt = (err_tilt - prev_err_tilt) / dt

                #self.get_logger().info(f"v_left: [P: {Kp} * {err_pan}, I: {Ki} * {integral_pan}, D: {Kd} * {derivative_pan}]\n v_right: [P: {Kp} * {err_tilt}, I: {Ki} * {integral_tilt}, D: {Kd} * {derivative_tilt}]")

                v_pan = Kp * err_pan + Ki * integral_pan + Kd * derivative_pan
                v_tilt = Kp * err_tilt + Ki * integral_tilt + Kd * derivative_tilt
                
                self.publish_head_command(v_pan, v_tilt, dt)
                
                for i, joint_name in enumerate(self.joint_states.name):
                    if joint_name=='pan_head_joint':
                        p_pan = self.joint_states.position[i]
                    if joint_name=='tilt_head_joint':
                        p_tilt = self.joint_states.position[i]

                print(f"head pos: {p_pan:.3f} {p_tilt:.3f}")

                prev_err_pan = err_pan
                prev_err_tilt = err_tilt
                err_pan = target_pan-p_pan
                err_tilt = target_tilt-p_tilt
            
            self.publish_head_command(0, 0, 0.1) #Prevents the head to keep moving after target position is reached.

            
    def position_head(self, pan=0.0, tilt=0.0):
        '''Executes the Head Controller in order to make it reach a specified pan/tilt target.'''

        if not self.has_parameter('p'):
            self.declare_parameter('p', pan)
        pan = pan|self.get_parameter('p').value
        if not self.has_parameter('t'):
            self.declare_parameter('t', tilt)
        tilt = self.get_parameter('t').value
        
        self.get_logger().info(f"p: {pan}, tilt: {tilt}")
        self.__setHeadPosition(pan, tilt)

    def stable_square(self, m=1, d=90):
        '''Uses previously defined controllers to make the robot move following a precise square-shaped trajectory. (WIP)'''
        
        """ head_thread = threading.Thread(target=self.__setHeadPosition, args=(0.0, 0.0, True)) #WIP --> DOES NOT WORK
        arms_thread = threading.Thread(target=self.__setArmsPosition, args=(0.0, 0.0, True))

        head_thread.start()
        arms_thread.start() """

        self.forward(m)
        self.turn(d)
        self.forward(m)
        self.turn(d)
        self.forward(m)
        self.turn(d)
        self.forward(m)
        self.turn(d)

        """ head_thread.join(0)
        arms_thread.join(0) """

def main(args=None):
    
    robot = MARRtinoController_Test()

    robot.run()

    robot.quit()


if __name__ == '__main__':
    main()


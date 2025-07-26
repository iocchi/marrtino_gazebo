import time, threading, math, os
from marrtino_control.control import MARRtinoController

class MARRtinoController_Test(MARRtinoController):
    def __init__(self):
        super().__init__()
    
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

def main(args=None):
    
    robot = MARRtinoController_Test()

    robot.run()

    robot.quit()


if __name__ == '__main__':
    main()


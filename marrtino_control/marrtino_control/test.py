import time

from control import MARRtinoController




robot = MARRtinoController()

robot.left(-20)

robot.get_image()

time.sleep(3)

# Shutdown the node after saving the image
robot.destroy_node()





    
    

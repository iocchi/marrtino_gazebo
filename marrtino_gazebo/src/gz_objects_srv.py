#!/usr/bin/env python3

import os
import json
import rclpy
from rclpy.node import Node
from marrtino_gazebo.srv import GazeboObjects

SERVICE_NAME = 'gz_current_objects'

class GazeboObjectsService(Node):
    def __init__(self):
        super().__init__('gz_objects_srv')

        # Internal storage
        self._keys = []
        self._items = []

        upath = os.getenv('PATH_USERS')
        if upath is None:
            self.get_logger().warn(f"Gazebo Objects Service: PATH_USERS env not found. Using '/op/users'")
            upath = '/opt/users'

        if not os.path.isdir(upath):
            os.mkdir(upath)

        self.file_path = os.path.join(upath, f'{SERVICE_NAME}.json')

        # Load list from file
        self._items = self.load_from_file()
        self._keys = [ item.split(' ')[0].strip() for item in self._items ]

        print(self._items)
        print(self._keys)

        # Create the service
        self.srv = self.create_service(
            GazeboObjects, 
            SERVICE_NAME, 
            self.handle_request
        )
        self.get_logger().info(f"Gazebo Objects Service {SERVICE_NAME} is ready.")

    def handle_request(self, request, response):
        action = request.action.lower()
        val = request.value
        key = val.split(' ')[0].strip()

        list_changed = False
        response.list = self._items

        if action == "add":
            if key not in self._keys:                
                self._items.append(val)
                self._keys.append(key)
                response.success = True
                response.message = f"Added '{val}'"
                list_changed = True
                response.list = self._items
            else:
                response.success = False
                response.message = f"Object {val} already exists."

        elif action == "del":
            if val == "*ALL*":
                self._keys = []
                self._items = []
                response.success = True
                response.message = f"Removed all objects"
                list_changed = True
                response.list = self._items

            elif key in self._keys:
                to_remove = None
                for item in self._items:
                    if key == item.split(' ')[0].strip():
                        to_remove = item
                        break
                assert to_remove is not None
                self._keys.remove(key)
                self._items.remove(to_remove)
                response.success = True
                response.message = f"Removed '{val}'"
                list_changed = True
                response.list = self._items
            else:
                response.success = False
                response.message = f"Object '{val}' not found."

        elif action == "list":
            response.success = True
            response.message = "Current list returned."
            response.list = self._items

        elif action == "get":
            if val in self._keys:
                response.success = True
                response.message = "Current list returned."
                response.list = [ self._items[self._keys.index(val)] ]
            else:
                response.success = False
                response.message = f"Object '{val}' not found."

        else:
            response.success = False
            response.message = f"Invalid action {action}. Use 'add', 'del', 'get', or 'list'."

        # Save list on file
        if list_changed:
            # write on file
            self.save_to_file()  # Persistence!


        # Return the current state of the list in the response
        
        return response


    def load_from_file(self):
        """Read the list from the JSON file."""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    data = json.load(f)
                    #self.get_logger().info(f"Loaded existing list: {data}")
                    return data
            except Exception as e:
                self.get_logger().error(f"Failed to load file: {e}")
        return [] # Return empty list if file doesn't exist or fails

    def save_to_file(self):
        """Write the current list to the JSON file."""
        try:
            with open(self.file_path, 'w') as f:
                json.dump(self._items, f)
        except Exception as e:
            self.get_logger().error(f"Failed to save file: {e}")


def main():
    rclpy.init()
    node = GazeboObjectsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()


'''
Examples:


ros2 service call /gz_current_objects marrtino_gazebo/srv/GazeboObjects "{action: 'list', value: ''}"

ros2 service call /gz_current_objects marrtino_gazebo/srv/GazeboObjects "{action: 'add', value: 'sensor_front'}"
'''

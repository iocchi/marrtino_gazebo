#!/usr/bin/env python

# adapted from https://boschresearch.github.io/pcg_gazebo_pkgs/python_api/pcg_gazebo.simulation/

from __future__ import print_function

import time
import os
import sys
import math

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler

from marrtino_gazebo.srv import GazeboObjects


class ModelManager(Node):
    def __init__(self):
        super().__init__('model_spawner')

        self.get_logger().info('ModelManager node initialized and service client created.')

        self.GAZEBO_MODELS = None

        self.get_gazebo_models()

        # object to exclude from state and delete
        self.excluding = ['smarrtino', 'ground_plane', 
                     'black_wall1','black_wall2','black_wall3','black_wall4']

        # dict of static object strings (for save/load worlds)
        # name: obj_str

        # Wait for the service to be available
        self.cli_gz = self.create_client(GazeboObjects, 'gz_current_objects')
        while not self.cli_gz.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service gz_current_objects not available, waiting...')
        self.gz_currobj_req = GazeboObjects.Request()


    def safe_spin_until_future_complete(self, future):
        complete = False
        while not complete:
            try:
                rclpy.spin_until_future_complete(self, future)
                complete = True
            except IndexError as e:
                self.get_logger().error(f"Index Error: {e}")
                time.sleep(0.1)
            except ValueError as e:
                self.get_logger().error(f"Value Error: {e}")
                time.sleep(0.1)



    # blocking call for service gz_current_objects
    def send_srv_request(self, action, value=""):
        self.gz_currobj_req.action = action
        self.gz_currobj_req.value = value
        # Call the service and return the future
        future = self.cli_gz.call_async(self.gz_currobj_req)

        # Spin until the response is received
        self.safe_spin_until_future_complete(future)

        response = future.result()
        if response.success:
            #self.get_logger().info(f"Success: {response.message}")
            #self.get_logger().info(f"Current List: {response.list}")
            return response.list
        else:
            self.get_logger().error(f"Failed: {response.message}")

        return None


    '''
    def spawn_box(self, model_name, x, y, z):
        # Define the SDF for a simple red box
        box_sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.7">
          <model name="{model_name}">
            <pose>0 0 0 0 0 0</pose>
            <link name="box_link">
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.166667</ixx>
                  <ixy>0.0</ixy>
                  <ixz>0.0</ixz>
                  <iyy>0.166667</iyy>
                  <iyz>0.0</iyz>
                  <izz>0.166667</izz>
                </inertia>
              </inertial>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>1 1 1</size>
                  </box>
                </geometry>
                <material>
                  <diffuse>1 0 0 1</diffuse> </material>
              </visual>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>1 1 1</size>
                  </box>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>"""

        self.srv_name = '/world/default/create'

        self.create_cli = self.create_client(SpawnEntity, self.srv_name)
        while not self.create_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.srv_name} not available, waiting...')
        self.create_req = SpawnEntity.Request()

        self.create_req.entity_factory.sdf = box_sdf
        #self.create_req.entity_factory.sdf_filename = 'ball.sdf'
        self.create_req.entity_factory.name = model_name
        # The pose in the EntityFactory message can also be set directly
        # using the Pose message, but for simple SDF injection, it's often
        # included in the SDF string itself.
        # However, if you're loading from a file and want to override the pose,
        # you would do:
        self.create_req.entity_factory.pose.position.x = float(x)
        self.create_req.entity_factory.pose.position.y = float(y)
        self.create_req.entity_factory.pose.position.z = float(z)
        self.create_req.entity_factory.pose.orientation.w = 1.0 # identity quaternion

        self.get_logger().info(f'Attempting to spawn model: {model_name} at ({x}, {y}, {z})')
        
        future = self.create_cli.call_async(self.create_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Successfully spawned {model_name}')
            else:
                self.get_logger().error(f'Failed to spawn {model_name}: {future.result().message}')
        else:
            self.get_logger().error('Service call failed (no response)')
    '''



    def list_objects(self):
        os.system(f"gz model --list 2> /dev/null > /tmp/objlist.txt ")
        listobj = []
        with open("/tmp/objlist.txt", "r") as f:
            for line in f:
                #print(line.rstrip())
                if line[0:5] == "    -":
                    objname = line[6:].strip()
                    listobj.append(objname)
        print(listobj)
        return listobj

    def get_object_state(self, name):
        '''
        os.system(f"gz model -m {model_name}  2> /dev/null > /tmp/objstate.txt")
        with open("/tmp/objstate.txt", "r") as f:
            pos = 0
            for line in f:
                if line[0:8] == "  - Pose":
                    pos = 1  # next line is poss
                elif pos==1:
                    print(f"Pos: {line}")
                    pos += 1  # next line is RPY
                elif pos==2:
                    print(f"RPY: {line}")
                    pos = 0  # nothing else
                    break
        '''
        if name in self.excluding:
            return None
        r = self.send_srv_request("get", name)
        return r[0] if r is not None else None

    def print_all_objects_state(self):
            lo = self.list_objects()
            for o in lo:
                s = self.get_object_state(o)
                if s is not None:
                    print(s)



    def print_object_state(self, model_name):
        os.system(f"gz model -m {model_name}")

    def get_gazebo_model_folders(self, dir_path):

        assert os.path.isdir(dir_path), \
            'Invalid directory path, path={}'.format(dir_path)

        models_paths = dict()
        for item in os.listdir(dir_path):
            if os.path.isdir(os.path.join(dir_path, item)):
                subfolder_items = os.listdir(os.path.join(dir_path, item))
                has_config = False
                has_sdf = False
                sdf_files = list()

                for subitem in os.listdir(os.path.join(dir_path, item)):
                    if os.path.isfile(os.path.join(dir_path, item, subitem)):
                        if '.config' in subitem:
                            has_config = True
                        if '.sdf' in subitem:
                            has_sdf = True
                            sdf_files.append(subitem)
                
                if has_config and has_sdf:
                    models_paths[item] = dict(
                        path=os.path.join(dir_path, item), 
                        sdf=sdf_files)
                else:
                    models_paths.update(self.get_gazebo_model_folders(os.path.join(dir_path, item)))        
        return models_paths


    def get_gazebo_models(self):
        """Search for Gazebo models in $GZ_SIM_RESOURCE_PATH folders
        *Returns*
        `dict`: Information of all Gazebo models found
        """

        self.GAZEBO_MODELS = dict()

        paths = os.getenv("GZ_SIM_RESOURCE_PATH")
        vpaths = paths.split(":");
        for path in vpaths:
            if path!="":
                dd = self.get_gazebo_model_folders(path)
                self.GAZEBO_MODELS.update(dd)

        # Load all models from ~/.gz/models
        home_folder = os.path.expanduser('~')
        gazebo_folder = os.path.join(home_folder, '.gz', 'models')
        if os.path.isdir(gazebo_folder):
            self.GAZEBO_MODELS.update(get_gazebo_model_folders(gazebo_folder))
                
        return self.GAZEBO_MODELS


    def print_models(self):
        models = self.GAZEBO_MODELS
        for tag in sorted(models):
            print("%s\t%s" %(tag, models[tag]['path']))


    def coords2pose(self, coords):
        """Creates a Pose object 'geometry_msgs.msg.Pose()' with the given coordinates in the format [x, y, z, roll, pitch, yaw]

        :param coords: Coordinates of the object in the format [x, y, z, roll, pitch, yaw]
        :return: Pose object
        """
        object_pose = Pose()
        quaternion = quaternion_from_euler(coords[3], coords[4], coords[5])
        object_pose.position.x = float(coords[0])
        object_pose.position.y = float(coords[1])
        object_pose.position.z = float(coords[2])
        object_pose.orientation.x = quaternion[0]
        object_pose.orientation.y = quaternion[1]
        object_pose.orientation.z = quaternion[2]
        object_pose.orientation.w = quaternion[3]
        return object_pose



    def save_model(self, model, collision=True):
        filename = f'/tmp/tmp_{model}.sdf'
        print(f"Creating model {model} on file {filename} ...")
        ms = model.split('_')

        # standard sizes
        sizes = {
            'cylinder': [0.03, 0.12],
            'sphere': [0.05],
            'box': [0.06, 0.06, 0.06],
            'door': [0.06, 1.0, 1.0],
            'line': [1.0, 0.06, 0.002],
        }
        if len(ms)>2: # custom size
            for i in range(2,len(ms)):
                sizes[ms[0]][i-2] = float(ms[i])

        if ms[0] == "cylinder":
            geometry = f"<cylinder> <radius>{sizes[ms[0]][0]}</radius> <length>{sizes[ms[0]][1]}</length> </cylinder>"
        elif ms[0] == "sphere":
            geometry = f"<sphere> <radius>{sizes[ms[0]][0]}</radius> </sphere>"
        elif ms[0] == "box":
            geometry = f"<box> <size>{sizes[ms[0]][0]} {sizes[ms[0]][1]} {sizes[ms[0]][2]}</size> </box>"
        elif ms[0] == "door":
            geometry = f"<box> <size>{sizes[ms[0]][0]} {sizes[ms[0]][1]} {sizes[ms[0]][2]}</size> </box>"
        elif ms[0] == "line":
            geometry = f"<box> <size>{sizes[ms[0]][0]} {sizes[ms[0]][1]} {sizes[ms[0]][2]}</size> </box>"
            collision = False
        #elif ms[0] == "poster":
        #   cmd = "cd ../models/poster/materials/textures && ln -sf  .. && cd -"
            

        if ms[1] == "red":
            material = "<ambient>0.8 0.1 0.1 1.0</ambient> <diffuse>0.8 0.1 0.1 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"
        elif ms[1] == "blue":
            material = "<ambient>0.1 0.1 0.8 1.0</ambient> <diffuse>0.1 0.1 0.8 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"
        elif ms[1] == "yellow":
            material = "<ambient>0.9 0.9 0.0 1.0</ambient> <diffuse>0.9 0.9 0.0 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"
        elif ms[1] == "green":
            material = "<ambient>0.1 0.8 0.1 1.0</ambient> <diffuse>0.1 0.8 0.1 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"
        elif ms[1] == "orange":
            material = "<ambient>1.0 0.5 0.0 1.0</ambient> <diffuse>1.0 0.5 0.0 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"
        elif ms[1] == "brown":
            material = "<ambient>0.5 0.2 0.0 1.0</ambient> <diffuse>0.5 0.2 0.0 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"            
        elif ms[1] == "black":
            material = "<ambient>0.0 0.0 0.0 1.0</ambient> <diffuse>0.0 0.0 0.0 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"
        elif ms[1] == "white":
            material = "<ambient>1.0 1.0 1.0 1.0</ambient> <diffuse>1.0 1.0 1.0 1.0</diffuse> <specular>0.3 0.3 0.3 1</specular>"

        with open(filename, "w") as f:
            f.write("<?xml version=\"1.0\" ?>\n<sdf version=\"1.5\">\n\n")
            f.write(f"<model name=\"{model}\">\n")
            if ms[0] == "door":
                f.write("  <pose>0 0 0.5 0 0 0</pose>\n")
            else:
                f.write("  <pose>0 0 0 0 0 0</pose>\n")
            if collision:
                f.write("  <static>false</static>\n")
            else:
                f.write("  <static>true</static>\n")
            f.write("  <link name=\"link\">\n")
            f.write(f"    <visual name=\"visual\"> <geometry> {geometry} </geometry> <material> {material} </material> </visual>\n")
            if collision:
                f.write("     <inertial auto=\"true\"/>\n")
                f.write(f"    <collision name=\"collision\"> <geometry> {geometry} </geometry> </collision>\n")
            f.write("  </link>\n")
            f.write("</model>\n\n")
            f.write("</sdf>\n")

        return filename


    def move_object(self, name, model, pose):
        self.del_object(name)
        self.add_object(name, model, pose)


    def add_object(self, name, model, pose):

        self.srv_name = '/world/default/create'

        self.create_cli = self.create_client(SpawnEntity, self.srv_name)
        while not self.create_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.srv_name} not available, waiting...')
        self.create_req = SpawnEntity.Request()

        models = self.GAZEBO_MODELS
        #print(models)

        if model not in models.keys():
            self.get_logger().error(f'Unknown model {model}')
            return

        try:
            filename = os.path.join(models[model]['path'], models[model]['sdf'][0])
            print(filename)
        except Exception as e:
            print(e)
            filename = self.save_model(model)


        self.create_req.entity_factory.sdf_filename = filename
        self.create_req.entity_factory.name = name
        # The pose in the EntityFactory message can also be set directly
        # using the Pose message, but for simple SDF injection, it's often
        # included in the SDF string itself.
        # However, if you're loading from a file and want to override the pose,
        # you would do:

        if type(pose) == str:
            pose_str = pose
            pose = []
            vp = pose_str.split(" ");
            for x in vp:
                if x.strip() != '':
                    pose.append(float(x))

        object_pose = self.coords2pose(pose)

        self.create_req.entity_factory.pose = object_pose

        self.get_logger().info(f'Attempting to spawn model: {name} of type {model} at {pose}')

        future = self.create_cli.call_async(self.create_req)
        self.safe_spin_until_future_complete(future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Successfully spawned {name}')
                pose_str = ' '.join([str(v) for v in pose])
                r = self.send_srv_request("add", value=f"{name} {model} {pose_str}")
                #self.get_logger().info(f'Service gz_current_objects result: {r}')
            else:
                self.get_logger().error(f'Failed to spawn {name}: {future.result().message}')
        else:
            self.get_logger().error('Service call failed (no response)')


    def add_objects(self, infile):
        with open(infile,'r') as f:
            l = f.readline()
            while l!='' and l[0:4] != '#END':
                l = l.strip()
                if len(l)>1 and l[0]!='#':
                    v = l.split()
                    if len(v)>7:
                        try:
                            name = v[0]
                            model = v[1]
                            pose = [ float(x) for x in v[2:] ]
                            self.add_object(name, model, pose)
                            time.sleep(0.2)
                        except Exception as e:
                            print("%s%s\n" %(l,e))
                    else:
                        print("Parse error: %s" %l)
                l = f.readline()
            f.close()


    def pose_str(self, pose):
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        a = tft.euler_from_quaternion(quat)
        return '%.4f %.4f %.4f  %.2f  %.2f  %.2f' \
            %(pose.position.x,pose.position.y,pose.position.z,a[0],a[1],a[2])


    def del_object(self, model_name):
        self.delete_object(model_name)

    def delete_object(self, model_name):

        if type(model_name) is list:
            for m in model_name:
                self.delete_object(m)
            return

        if model_name in self.excluding:
            return

        self.delete_cli = self.create_client(DeleteEntity, '/world/default/remove')
        while not self.delete_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service remove not available, waiting...')
        self.delete_req = DeleteEntity.Request()
    
        self.delete_req.entity.id = 0 # ???
        self.delete_req.entity.name = model_name
        self.delete_req.entity.type = 2  # MODEL

        future = self.delete_cli.call_async(self.delete_req)
        self.safe_spin_until_future_complete(future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Successfully removed {model_name}')
                r = self.send_srv_request("del", value=model_name)
                #self.get_logger().info(f'Service gz_current_objects result: {r}')
            else:
                self.get_logger().error(f'Failed to remove {model_name}: {future.result().message}')
        else:
            self.get_logger().error('Service call failed (no response)')

        time.sleep(0.1)

    # name = 'abc*'
    # delete all objects starting with abc
    def delete_all_objects_like(self, name):
        l = list_objects()
        for obj in l:
            if obj.startswith(name[:-1]):
                self.delete_object(obj)

    def del_objects(self, objs):
        if type(objs) == list:
            for ob in objs:
                self.del_object(ob)
        elif type(objs) == str:
            with open(objs,'r') as f:
                l = f.readline()
                while l!=''  and l[0:4] != '#END':
                    l = l.strip()
                    if len(l)>1 and l[0]!='#':
                        v = l.split()
                        name = v[0]
                        if '*' in name:
                            self.delete_all_objects_like(name)
                        else:
                            self.delete_object(name)
                    l = f.readline()
                f.close()
        else:
            print(f"Error del_objects unkwnown type {obsj}")

    def del_all_objects(self):
        l = self.list_objects()
        excluded = ['ground_plane','robot','smarrtino',
'black_wall1', 'black_wall2', 'black_wall3', 'black_wall4' ]
        for obj in l:
            if obj not in excluded:
                self.delete_object(obj)
                time.sleep(0.1)


    def move_robot(self, x, y, th_deg):
        stype = "-s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000"

        spos = "{ " + f"x: {x}, y: {y}, z: 0.25" + " }"
        quaternion = quaternion_from_euler(0, 0, th_deg/180.0*math.pi)
        squat = "{ " + f"x: {quaternion[0]}, y: {quaternion[1]}, z: {quaternion[2]}, w: {quaternion[3]}" + " }"

        spar = f"name: \\\"smarrtino\\\", position: {spos}, orientation: {squat}"
        cmd = f"gz service {stype} --req \"{spar}\""
        print(cmd)
        os.system(cmd)
        time.sleep(1)

    def get_user_path(self):
        upath = os.getenv('PATH_USERS')
        if upath is None:
            self.get_logger().warn(f"Gazebo Objects Service: PATH_USERS env not found. Using '/op/users'")
            upath = '/opt/users'

        cuf = os.path.join(upath,"current_user")
        try:
            with open(cuf, "r") as f:
                l = f.readline().strip()
        except:
            l = ''
        if l == '':
            l = 'anonymous'

        userpath = os.path.join(upath, l)
        if not os.path.isdir(userpath):
            os.mkdir(userpath)

        return userpath

    def save_world(self, world_file):
        lo = self.list_objects()
        world_path = os.path.join(self.get_user_path(), world_file)
        with open(world_path, "w") as f:
            for o in lo:
                if o not in self.excluding:
                    r = self.send_srv_request("get", o)
                    if r is not None:
                        f.write(r[0]+'\n')
        print(f"Saved gazebo world file {world_path}")

    def load_world(self, world_file):
        world_path = os.path.join(self.get_user_path(), world_file)
        if os.path.isfile(world_path):
            self.del_all_objects()
            self.add_objects(world_path)
            print(f"Loaded gazebo world file {world_path}")
            return True
        return False



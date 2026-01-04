#!/usr/bin/env python

from gz_models import *


if __name__ == '__main__':

    rclpy.init(args=None)
    mm = ModelManager()

    if len(sys.argv)==1 or sys.argv[1]=='-h':
            print("Options")
            print("-h\thelp")
            print("-l\tlist of existing objects in the simulation")
            print("-s <obj>\tobject properties and state")
            print("-m\tlist of available models")
            print("-a <name> <type> <x> <y> <z> <yaw> <pitch> <roll>|<filename>\tadd an object or all objects in config file")
            print("-d <obj>|<filename>\tdelete one object or all objects in config file")
            #print("-w\tworld properties")

    else:
        if sys.argv[1]=='-m':
            mm.print_models()

        elif sys.argv[1]=='-l':
            l = mm.list_objects()
            '''
            l = mm.list_object_names()
            for obj in l:
                p = get_object_pose(obj)
                #s = get_model_state(obj)
                print("%s  %s" %(obj,pose_str(p)))
            '''

        elif sys.argv[1]=='-s' and len(sys.argv)>2:
            name = sys.argv[2]
            print("Object %s" %name)
            mm.print_object_state(name)

        #elif sys.argv[1]=='-w':
        #    print(get_world_properties())

        elif sys.argv[1]=='-a' and len(sys.argv)>2:
            name = sys.argv[2]
            if os.path.isfile(name):
                mm.add_objects(name)
            elif len(sys.argv)>9:
                model = sys.argv[3]
                pose = [ float(x) for x in sys.argv[4:] ]
                mm.add_object(name, model, pose)

        elif sys.argv[1]=='-d' and len(sys.argv)>2:
            name = sys.argv[2]
            if name=='all':
                mm.del_all_objects()
            elif os.path.isfile(name):
                mm.del_objects(name)
            else:
                mm.delete_object(name)




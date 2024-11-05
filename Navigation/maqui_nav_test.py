#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import rospy
from maqui_skills import robot_factory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


"""
HOW TO RUN THIS EXAMPLE:

# change folder to "home/nao/uchile_robocup/src/Navigation_Maqui" inside Maqui and launch:
# python maqui_nav_test.py
"""

# Lo que se hará de momento es hacer un código bastante cutre para activar distintas
# funcionalidades dependiendo de lo que el usuario quiere. Las funciones que estarán
# disponibles son las siguientes:
# where_i_am()
# go_to_point()
# go_to_pose_stamped()

posiciones =    {"point_a": {"x":0.22277,
                             "y":-0.64685,
                             "w":0},
                 "point_b": {"x":1.67090,
                             "y":-0.72750,
                             "w":0}
                }

origin =    {"position":    {"x":-0.0877829449669,
                             "y":0.225132778968,
                             "w":0.819547923365},
             "orientation": {"x":0.00204631127958,
                             "y":0,
                             "z":0,
                             "w":0.999514665218}
            }

bedroom =   {"position":    {"x":2.83044644989,
                             "y":2.38252281101,
                             "w":0.819810934721},
             "orientation": {"x":0.0103895182194,
                             "y":0,
                             "z":0,
                             "w":0.999601311485}
            }

bookshelf = {"position":    {"x":1.48090985806,
                             "y":0.227425328803,
                             "w":0.819892635388},
             "orientation": {"x":0.0127112794599,
                             "y":0,
                             "z":0,
                             "w":0.967326792615}
            }

home =  {"position":    {"x":0.539619772159,
                         "y":-1.7261572062,
                         "w":0.819926688557},
         "orientation": {"x":-0.00359755658839,
                         "y":0,
                         "z":0,
                         "w":0.688131207292}
        }

kitchen =   {"position":    {"x":2.91157241032,
                             "y":-0.761650199358,
                             "w":0.819909074865},
             "orientation": {"x":0.00645726847743,
                             "y":0,
                             "z":0,
                             "w":0.819600251233}
            }

living =    {"position":    {"x":1.3767427427697,
                             "y":1.10933593568,
                             "w":0.819855930546},
             "orientation": {"x":0.00605359633475,
                             "y":0,
                             "z":0,
                             "w":0.983431722932}
            }

rooms = {"origin":origin, "bedroom":bedroom, "bookshelf":bookshelf, "home":home, "kitchen":kitchen, "living":living}

if __name__ == "__main__":

    rospy.init_node("core_bot_example")

    print("--------------------------------------------------------------------")
    robot = robot_factory.build(["navigation", "knowledge"], core=False)
    print( "--------------------------------------------------------------------")    

    # Carga de mapa
    robot.knowledge.pose.load_from_map('map.sem_map')

    while not rospy.is_shutdown():
        action = raw_input("¿Qué quieres hacer? ")
        if action == "donde estas":
            print(robot.navigation.where_i_am()) # FUNCIONA
        elif action == "ir hacia":
            place = raw_input("Lugar: ")
            robot.navigation.go(place) # NO FUNCIONA
            robot.navigation.wait_for_result()
        elif action == "ir a lugar":
            loc = raw_input("indique lugar: ")
            room = rooms.get(loc)
            x, y, w = room["position"]["x"], room["position"]["y"], room["position"]["w"]
            robot.navigation.go_to_point(x, y, w) # FUNCIONA
            robot.navigation.wait_for_result()
            robot.navigation.reached()
        elif action == "mapa":
            robot.knowledge.map_name() # NO FUNCIONA
        elif action == "habitaciones":
            robot.knowledge.keys() # NO FUNCIONA
        else:
            exit()
        

    # goal = Pose()
    # goal.position.x = 2
    # goal.orientation.w = 1
    # robot.knowledge.pose.load_from_map('mapadeprueba.sem_map')


    ## Go commands (FUNCIONA)
    #print("Robot, where are you? ... ")
    #print(robot.navigation.where_i_am())

    #goal = PoseStamped()  (FUNCIONA)
    #goal.header.stamp = rospy.Time.now()
    #goal.header.frame_id = "map"
    #goal.pose.position.x = 0.2
    #goal.pose.position.y = -1
    #goal.pose.orientation.w = 1.0

    #robot.navigation.go_to_point(0.22277, -0.64685, 0)  (FUNCIONA)
    #robot.navigation.go_to_point(1.67090, -0.72750, 0)  (FUNCIONA)
    #robot.navigation.rotate(-90)  (FUNCIONA)
    
    # print("Robot in map? ", robot.navigation.is_robot_in_map())
    # print("Current rooms: ", robot.navigation.get_current_rooms())

    # robot.navigation.go_to_pose_stamped(goal)  (FUNCIONA)

    # print("Setting Goal ...")
    #robot.navigation.go("kitchen")
    # # robot.navigation.go_to_point(x=1, degrees=180, use_robot_frame=True)
    # # robot.navigation.go_to_point(x=2, y=-1, degrees=90)
    # # robot.navigation.go_to_pose(goal)
    # print "- has reached?: " + str(robot.navigation.reached())
    # print "- is moving?: " + str(robot.navigation.is_moving())
    # robot.navigation.wait_for_result()

    # # print "cancelling goal ..."
    # # rospy.sleep(2)
    # # robot.navigation.cancel()
    # # print "- has reached?: " + str(robot.navigation.reached())
    # # print "- is moving?: " + str(robot.navigation.is_moving())

    
    # ## Look commands
    #print "Setting Look Goal ..."
    #robot.navigation.look("kitchen")
    #print "- has reached?: " + str(robot.navigation.reached())
    #print "- is moving?: " + str(robot.navigation.is_moving())

    # print "waiting for result ..."
    # robot.navigation.wait_for_result()
    # print "- has reached?: " + str(robot.navigation.reached())
    # print "- is moving?: " + str(robot.navigation.is_moving())


    # print "Setting another Goal ..."
    # robot.navigation.go("bedroom")
    # robot.navigation.wait_for_result(5)
    # print "- has reached?: " + str(robot.navigation.reached())
    # print "- is moving?: " + str(robot.navigation.is_moving())


    #robot.navigation.look_to_point(y=1, use_robot_frame=True)
    #robot.navigation.look_to_point(x=2, y=-1)
    #robot.navigation.look_to_pose(goal)
    #robot.navigation.rotate(270)

# Cómo darle al usuario la opción de moverse a una ubicación en específico?

import rospy
from json import load
from bender_skills import robot_factory
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

posiciones = load(open("posiciones.json", "r"))

if __name__ == "__main__":

    rospy.init_node("core_bot_example")

    print("--------------------------------------------------------------------")
    robot = robot_factory.build(["navigation", "knowledge"], core=False)
    print( "--------------------------------------------------------------------")    

    # Carga de mapa
    robot.knowledge.pose.load_from_map('map.sem_map')

    while not rospy.is_shutdown():
        action = input("¿Qué quieres hacer? ")
        if action == "where i am":
            print(robot.navigation.where_i_am()) # FUNCIONA
        elif action == "go to":
            loc = input("Indique lugar: ")
            room = posiciones.get(loc)
            x, y, w = room["position"]["x"], room["position"]["y"], room["position"]["w"]
            robot.navigation.go_to_point(x, y, w) # FUNCIONA
            robot.navigation.wait_for_result()
            robot.navigation.reached()
        elif action == "mapa":
            robot.knowledge.map_name()
        elif action == "habitaciones":
            robot.knowledge.keys()
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

# Approach Smach
import rospy
import smach
import random
import math
from smach_ros import IntrospectionServer
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from point_to_approach import estimate_person_map

# Robot building
from maqui_skills import robot_factory

# Origin
origin = {
        "position": {
            "x": 0.0,
            "y": 0.0,
            "w": 0.0
        },
        "orientation": {
            "x": 0.0,
            "y": 0,
            "z": 0,
            "w": 0.0
        }
    }

# Ideal distance
ideal_distance = 900.0
#####################################################################

class Init(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['cmd_succeeded'])
        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo('Estimate Position SMACH for Maqui started')
        return 'cmd_succeeded'

# Estimacion de distancia a la persona
class estimate(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded'],
                             output_keys = ['distance'])
        self.robot = robot

    def execute(self, userdata):
        distance_str = rospy.wait_for_message('/depth/average_distance', Float32)
        distance = float(distance_str.data)

        userdata.distance = distance
        rospy.loginfo('Estimated distance: %s', userdata.distance)
        return 'succeeded'

# Estimar punto en el mapa
class estimate_point(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded'],
                             input_keys = ['distance'],
                             output_keys = ['x_position', 'y_position', 'w_position', 'distance'])
        self.robot = robot

    def execute(self, userdata):
        # Insertar función para estimar punto en el mapa
        actual_position = self.robot.navigation.where_i_am()
        userdata.x_position, userdata.y_position, userdata.w_position = estimate_person_map(userdata.distance, actual_position)

        return 'succeeded'

class go_to(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded'],
                                   input_keys = ['x_position', 'y_position', 'w_position', 'distance'],
                                   output_keys = ['distance'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.navigation.go_to_point(userdata.x_position, userdata.y_position, userdata.w_position)
        self.robot.navigation.wait_for_result()
        return 'succeeded'
        
# class new_estimate(smach.State):
#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes = ['succeeded', 'recalculated', 'end'],
#                              input_keys = ['distance'],
#                              output_keys = ['distance'])
#         self.robot = robot

#     def execute(self, userdata):
#         new_distance_str = rospy.wait_for_message('/maqui/interactions/person_detector', String)
#         new_distance = float(new_distance_str.data)

#         # Si Pepper llegó a una distancia ideal de conversación, se detiene
#         if (new_distance < userdata.distance and new_distance >= ideal_distance and new_distance <= ideal_distance + 20.0):
#             return 'succeeded'
        
#         # Si Pepper sigue lejos de la persona y, la distania se va acortando, se sigue moviendo
#         elif (new_distance < userdata.distance and new_distance > ideal_distance):
#             userdata.distance = new_distance
#             return 'recalculate'
        
#         # Si Pepper, aún así de haberse acercado se alejó más de la persona, se detiene
#         elif (new_distance > userdata.distance):
#             return 'end'

#####################################################################

def getInstance(robot):
    sm = smach.StateMachine(
        outcomes = ['SUCCESS', 'FAILED', 'END']
    )

    # sm.userdata.room_name = None # Deberia ser un ingreso por voz
    # Almacenar última habitación visitada
    # sm.userdata.last_rooms = None

    with sm:

        smach.StateMachine.add('Init', Init(robot),
                transitions = {'cmd_succeeded':'waiting_room'})
        
        smach.StateMachine.add('estimate', estimate(robot),
                transitions = {'succeeded':'estimate_point'})
        
        smach.StateMachine.add('estimate_point', estimate_point(robot),
                transitions = {'succeeded':'go_to'})
        
        smach.StateMachine.add('go_to', go_to(robot),
                transitions = {'succeeded':'succeeded'})
        
        # smach.StateMachine.add('new_estimate', new_estimate(robot),
        #         transitions = {'succeeded':'SUCCESS',
        #                        'recalculated':'estimate_point',
        #                        'end':'END'})

    return sm
#####################################################################

def main():
    #try:
        rospy.init_node('maqui_smach')
        robot = robot_factory.build(['navigation', 'knowledge'], core = False)

        sm = getInstance(robot)

        sis = IntrospectionServer('nav_smach', sm, '/Maqui_Nav_SMACH')
        sis.start()
        outcome = sm.execute()
        rospy.spin()
        sis.stop()

    #except:
        #exit()

if __name__ == "__main__":
    main()
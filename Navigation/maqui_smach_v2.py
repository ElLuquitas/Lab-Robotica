# SMACH de Navegacion Maqui
import json
import rospy
import smach
import random
from smach_ros import IntrospectionServer
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

# Robot building
from maqui_skills import robot_factory

# Rooms import
file_path = 'positions.json'

with open(file_path, 'r') as file:
    rooms_list = json.load(file)

#####################################################################

class Init(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['cmd_succeeded'])
        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo('Navigation SMACH for Maqui started')
        return 'cmd_succeeded'
    
class waiting_room(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded', 'timeout'],
                             output_keys = ['room_name'])
        self.robot = robot

    def execute(self, userdata):
        # debe esperar un tiempo de x segundos para recibir una habitación
        # Si el usuario le da una habitación, entonces guarda esa habitación.
        # Si no, entonces se pasa a elegir una habitación al azar.
        timeout = 120
        required_room = rospy.wait_for_message('/required_room', String, timeout)
        if required_room is None:
            return 'timeout'
        
        userdata.room_name = required_room
        rospy.loginfo('Required room: %s', userdata.room_name)
        return 'succeeded'

class random_room(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded'],
                                   input_keys = ['last_rooms', 'rooms_list'],
                                   output_keys = ['room_name'])
        self.robot = robot

    def execute(self, userdata):

        userdata.room_name = random.choice(list(userdata.rooms_list.keys()))
        userdata.last_rooms = userdata.room_name
        rospy.loginfo('Random room: %s', userdata.room_name)

        return 'succeeded'

class check_room(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['valid_room', 'invalid_room'],
                                   input_keys = ['room_name', 'rooms_list'],
                                   output_keys = ['x_position', 'y_position', 'w_position'])
        self.robot = robot

    def execute(self, userdata):
        if userdata.room_name not in userdata.rooms_list:
            rospy.loginfo('Room %s is invalid', userdata.room_name)
            userdata.room_name = None
            return 'invalid_room'
        
        else:
            rospy.loginfo('Room %s is valid', userdata.room_name)
            room = userdata.rooms_list.get(userdata.room_name)
            userdata.x_position = room['position']['x']
            userdata.y_position = room['position']['y']
            userdata.w_position = room['position']['w']
            return 'valid_room'

class go_to_room(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded'],
                                   input_keys = ['x_position', 'y_position', 'w_position'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.navigation.go_to_point(userdata.x_position, userdata.y_position, userdata.w_position)
        return 'succeeded'
        
class waiting_result(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded', 'end', 'failed'])
        self.robot = robot

    def execute(self, userdata):
        person_in_front = rospy.wait_for_message('/person_in_front', String)
        while person_in_front is None:
            
            if self.robot.navigation.wait_for_result():
                return 'succeeded'
            else:
                return 'failed'
        
        return 'end'

class stop_movement(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes = ['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.navigation.cancel()

        return 'succeeded'
    
#####################################################################

def getInstance(robot):
    sm = smach.StateMachine(
        outcomes = ['SUCCESS', 'FAILED', 'END']
    )

    sm.userdata.rooms_list = rooms_list
    sm.userdata.room_name = None # Deberia ser un ingreso por voz
    # Almacenar última habitación visitada
    sm.userdata.last_rooms = None

    with sm:

        smach.StateMachine.add('Init', Init(robot),
                transitions = {'cmd_succeeded':'waiting_room'})
        
        smach.StateMachine.add('waiting_room', waiting_room(robot),
                transitions = {'received':'check_room',
                               'timeout':'random_room'})
        
        smach.StateMachine.add('check_room', check_room(robot),
                transitions = {'valid_room':'go_to_room',
                               'invalid_room':'END'})
        
        smach.StateMachine.add('random_room', random_room(robot),
                transitions = {'succeeded':'go_to_room'})
        
        smach.StateMachine.add('go_to_room', go_to_room(robot),
                transitions = {'succeeded':'SUCCESS'})
        
        smach.StateMachine.add('waiting_result', waiting_result(robot),
                transitions = {'succeeded':'SUCCESS',
                               'end':'stop_movement',
                               'failed':'FAILED'})
        
        smach.StateMachine.add('stop_movement', stop_movement(robot),
                transitions = {'succeeded':'END'})

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
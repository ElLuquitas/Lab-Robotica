# SMACH de Navegacion Maqui
import json
import rospy
import smach
import random
from smach_ros import IntrospectionServer
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
        smach.State.__init__(self, outcomes = ['cmd_succeeded'],
                                   input_keys = ['room_name', 'rooms_list'],
                                   output_keys = ['room_name'])

        self.robot = robot

    def execute(self, userdata):
        if userdata.room_name is None:
            rospy.sleep(120)
            userdata.room_name = random.choice(list(userdata.rooms_list.keys()))
            rospy.loginfo('Random room: %s', userdata.room_name)
        else:
            room_name = userdata.room_name
            rospy.loginfo('Required room: %s', room_name)

        return 'cmd_succeeded'

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
        smach.State.__init__(self, outcomes = ['succeeded', 'failed', 'end'],
                                   input_keys = ['x_position', 'y_position', 'w_position'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.navigation.go_to_point(userdata.x_position, userdata.y_position, userdata.w_position)
        self.robot.navigation.wait_for_result()

        # Se debe agregar la opción de escuchar tópicos relacionados a conversación e identificación,
        # con tal de gatillar el cambio de estado a 'end' si es necesario.
        # Mejor implementación: crear otro estado que reciba el resultado del movimiento

        if self.robot.navigation.reached():
            return 'succeeded'
        
        elif not self.robot.navigation.reached():
            return 'failed'
        
        else:
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

    with sm:

        smach.StateMachine.add('Init', Init(robot),
                transitions = {'cmd_succeeded':'check_room'})
        
        smach.StateMachine.add('check_room', check_room(robot),
                transitions = {'valid_room':'go_to_room',
                               'invalid_room':'END'})
        
        smach.StateMachine.add('go_to_room', go_to_room(robot),
                transitions = {'succeeded':'SUCCESS',
                               'failed':'FAILED',
                               'end':'stop_movement'})
        
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
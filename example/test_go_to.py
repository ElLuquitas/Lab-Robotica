#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import PoseStamped, Point
from math import cos, sin

class CalculateMidpointNode:
    def __init__(self):
        rospy.init_node('calculate_midpoint_node', anonymous=True)

        # Suscriptor para la posición de la persona
        rospy.Subscriber('/person_position', PoseStamped, self.person_pose_callback)

        # Suscriptor para la posición del robot
        rospy.Subscriber('/robot_position', Point, self.robot_position_callback)

        # Variables para almacenar la posición del robot
        self.robot_position = None

    def robot_position_callback(self, msg):
        """
        Callback al recibir la posición del robot desde el tópico `/robot_position`.
        """
        self.robot_position = msg

    def person_pose_callback(self, msg):
        """
        Callback al recibir la posición de la persona (PoseStamped).
        Calcula el punto intermedio entre el robot y la persona.
        """
        if self.robot_position is None:
            rospy.logwarn("La posición del robot no está disponible aún.")
            return

        # Extraer la distancia desde el campo `z`
        distance_to_person = msg.pose.position.z

        # Extraer yaw de la orientación del robot (suponiendo yaw=0 si no está disponible)
        yaw = 0  # Ajustar según necesidades si el yaw se incluye más adelante

        # Calcular la posición de la persona en el espacio global
        person_x = self.robot_position.x + distance_to_person * cos(yaw)
        person_y = self.robot_position.y + distance_to_person * sin(yaw)

        # Calcular el punto intermedio
        midpoint_x = (self.robot_position.x + person_x) / 2
        midpoint_y = (self.robot_position.y + person_y) / 2

        rospy.loginfo(f"Punto intermedio calculado: x={midpoint_x}, y={midpoint_y}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        midpoint_calculator = CalculateMidpointNode()
        midpoint_calculator.run()
    except rospy.ROSInterruptException:
        pass

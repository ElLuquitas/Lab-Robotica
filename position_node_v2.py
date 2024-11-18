#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from math import sqrt

from bender_skills import robot_factory

rospy.init_node("core_bot_example")
print("--------------------------------------------------------------------")
robot = robot_factory.build(["navigation", "knowledge"], core=False)
print( "--------------------------------------------------------------------")    

class PositionTracker:
    def __init__(self):
        rospy.init_node('position_tracker', anonymous=True)
        
        rospy.Subscriber('/kalman_filter/estimated_position', Pose, self.person_position_callback)
        #rospy.Subscriber('/robot_position', Point, self.robot_position_callback)
        
        # Publicadores
        self.target_pub = rospy.Publisher('/robot_target_position', Point, queue_size=10)
        self.status_pub = rospy.Publisher('/person_tracker/status', String, queue_size=10)
        
        # Almacena posiciones de la persona y el robot
        self.person_position = None
        self.last_person_position = None
        self.robot_position = None
        
        # Umbrales
        self.dist_stop_threshold = 0.01  # Umbral para detectar si la persona está quieta
        self.dist_move = 1.0  # Distancia fija al moverse cuando la persona está quieta

    def euclidean_distance(self, pos1, pos2):
        """Calcula la distancia euclidiana entre dos posiciones"""
        return sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

    def robot_position_callback(self):
        """Actualiza la posición actual del robot"""
        self.robot_position = robot.navigation.where_i_am()

    def person_position_callback(self, msg):
        """Callback al recibir la posición de la persona"""
        self.last_person_position = self.person_position  # Guarda la posición anterior
        self.person_position = msg.position  # Actualiza la posición actual

        if self.person_position is None or self.robot_position is None:
            return  # Espera a tener datos tanto del robot como de la persona
        
        # Verifica si la persona está quieta
        if self.is_person_stopped():
            # Persona está quieta: calcula un punto fijo a 1 metro de distancia
            target_position = self.calculate_fixed_position(self.person_position, self.robot_position)
            self.publish_target(target_position)
            self.publish_status("Persona detenida, moviéndose a 1 metro.")
            rospy.loginfo(f"Persona detenida. Moviéndose a: {target_position}")
        else:
            # Persona en movimiento: calcula el punto medio
            target_position = self.calculate_midpoint(self.robot_position, self.person_position)
            self.publish_target(target_position)
            rospy.loginfo(f"Persona en movimiento. Punto medio calculado: {target_position}")

    def is_person_stopped(self):
        """Determina si la persona está quieta comparando con la posición anterior"""
        if self.last_person_position is None:
            return False  # No hay posición previa para comparar
        distance = self.euclidean_distance(self.person_position, self.last_person_position)
        return distance < self.dist_stop_threshold  # Persona quieta si la distancia es menor al umbral

    def calculate_midpoint(self, pos1, pos2):
        """Calcula el punto medio entre dos posiciones"""
        return Point(
            x=(pos1.x + pos2.x) / 2,
            y=(pos1.y + pos2.y) / 2,
            z=(pos1.z + pos2.z) / 2
        )

    def calculate_fixed_position(self, person_position, robot_position):
        """Calcula un punto fijo a 1 metro de distancia desde la persona"""
        direction_x = robot_position.x - person_position.x
        direction_y = robot_position.y - person_position.y
        norm = sqrt(direction_x**2 + direction_y**2)
        if norm == 0:  # Evitar división por cero
            norm = 1.0
        return Point(
            x=person_position.x + self.dist_move * (direction_x / norm),
            y=person_position.y + self.dist_move * (direction_y / norm),
            z=person_position.z  # Mantener el mismo nivel
        )

    def publish_target(self, target_position):
        """Publica la posición objetivo del robot"""
        self.target_pub.publish(target_position)

    def publish_status(self, message):
        """Publica el estado actual del seguimiento"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tracker = PositionTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
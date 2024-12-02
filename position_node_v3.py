#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
from math import sqrt

from bender_skills import robot_factory

# Inicializa el nodo principal de ROS
rospy.init_node("position_to_go_node")
print("--------------------------------------------------------------------")
robot = robot_factory.build(["navigation", "knowledge"], core=False)
print("--------------------------------------------------------------------")

class DistanceBasedPositionTracker:
    def __init__(self):
        rospy.init_node('distance_position_tracker', anonymous=True)
        
        # Suscriptor para la distancia de la persona
        #rospy.Subscriber('/person_distance', Float32, self.person_distance_callback)
        rospy.Subscriber('/person_pose', PoseStamped, self.person_distance_callback)
        
        # Publicadores
        self.target_pub = rospy.Publisher('/robot_target_position', Point, queue_size=10)
        self.status_pub = rospy.Publisher('/person_tracker/status', String, queue_size=10)
        
        # Almacena la posición actual del robot
        self.robot_position = None
        
        # Umbrales
        self.dist_stop_threshold = 500.0  # Umbral para detenerse si la persona está muy cerca
        self.dist_move = 1000.0  # Distancia fija para moverse hacia la persona

    def euclidean_distance(self, pos1, pos2):
        """Calcula la distancia euclidiana entre dos posiciones"""
        return sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

    def robot_position_callback(self):
        """Actualiza la posición actual del robot"""
        self.robot_position = robot.navigation.where_i_am()

    def person_distance_callback(self, msg):
        """Callback al recibir la distancia de la persona"""
        distance_to_person = msg.pose.position.z

        if self.robot_position is None:
            rospy.logwarn("Esperando la posición del robot...")
            return  # Espera a tener datos de la posición del robot

        if distance_to_person < self.dist_stop_threshold:
            self.publish_status("Persona muy cerca. Deteniendo movimiento.")
            rospy.loginfo("Persona muy cerca. Deteniendo movimiento.")
            return

        # Calcula la posición objetivo en función de la distancia
        target_position = self.calculate_fixed_position_using_distance(self.robot_position, distance_to_person)
        self.publish_target(target_position)
        rospy.loginfo(f"Distancia recibida: {distance_to_person}. Moviéndose a: {target_position}")

    def calculate_fixed_position_using_distance(self, robot_position, distance_to_person):
        """Calcula un punto fijo a 1 metro de distancia desde la persona"""
        # Supone que la persona está directamente en la línea entre el robot y el origen
        direction_x = self.robot_position.pose.position.x - 0  # Cambiar si la orientación es relevante
        direction_y = self.robot_position.pose.position.y - 0
        norm = sqrt(direction_x**2 + direction_y**2)

        # Evitar divisiones por cero
        if norm == 0:
            rospy.logwarn("La distancia es cero; manteniendo posición actual.")
            return Point(x=robot_position.x, y=robot_position.y, z=robot_position.z)

        # Normaliza la dirección y escala a la distancia deseada
        return Point(
            x=robot_position.pose.position.x - (distance_to_person + self.dist_move) * (direction_x / norm),
            y=robot_position.pose.position.y - (distance_to_person + self.dist_move) * (direction_y / norm),
            z=robot_position.pose.position.z  # Mantener el mismo nivel
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
        tracker = DistanceBasedPositionTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
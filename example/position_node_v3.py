#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from math import cos, sin, atan2, sqrt
import tf.transformations as tf_trans  # Para conversiones de cuaterniones a Euler

from bender_skills import robot_factory

# Inicializa el nodo principal de ROS
rospy.init_node("position_to_go_node")
print("--------------------------------------------------------------------")
robot = robot_factory.build(["navigation", "knowledge"], core=False)
print("--------------------------------------------------------------------")

class DistanceBasedPositionTracker:
    def __init__(self):
        rospy.init_node('distance_position_tracker', anonymous=True)
        
        # Suscriptor para la posición de la persona (PoseStamped)
        rospy.Subscriber('/person_pose', PoseStamped, self.person_pose_callback)
        
        # Publicadores
        self.target_pub = rospy.Publisher('/robot_target_position', Point, queue_size=10)
        self.status_pub = rospy.Publisher('/person_tracker/status', String, queue_size=10)
        
        # Umbrales
        self.dist_stop_threshold = 1.0  # Umbral para detenerse si la persona está muy cerca
        self.dist_move = 1.0  # Distancia fija para moverse hacia la persona

    def get_robot_position_and_orientation(self):
        """Obtiene la posición y orientación actual del robot."""
        robot_state = robot.navigation.where_i_am()
        
        # Extraer posición
        position = robot_state["position"]
        robot_position = Point(x=position["x"], y=position["y"], z=0)  # Usamos 0 para z
        
        # Extraer orientación (en cuaterniones) y convertir a yaw
        orientation = robot_state["orientation"]
        quaternion = (orientation["x"], orientation["y"], orientation["z"], orientation["w"])
        euler = tf_trans.euler_from_quaternion(quaternion)  # Convierte a ángulos de Euler
        yaw = euler[2]  # El ángulo yaw (rotación en el plano XY)
        
        return robot_position, yaw

    def person_pose_callback(self, msg):
        """
        Callback al recibir la posición de la persona (PoseStamped).
        Se extrae la distancia `z` desde la posición y se calcula la posición
        de la persona en relación con el robot.
        """
        # Obtener la posición y orientación del robot
        robot_position, yaw = self.get_robot_position_and_orientation()

        # Extraer la distancia desde el campo `z`
        distance_to_person = msg.pose.position.z

        # Verificar si la distancia está por debajo del umbral
        if distance_to_person < self.dist_stop_threshold:
            self.publish_status("Persona muy cerca. Deteniendo movimiento.")
            rospy.loginfo("Persona muy cerca. Deteniendo movimiento.")
            return

        # Calcular la posición de la persona en relación al robot
        person_position = self.calculate_person_position(robot_position, distance_to_person, yaw)

        # Calcular la posición objetivo a una distancia fija de la persona
        target_position = self.calculate_fixed_position(robot_position, person_position)
        
        # Publicar la posición objetivo y el estado
        self.publish_target(target_position)
        rospy.loginfo(f"Distancia recibida: {distance_to_person}. Moviéndose a: {target_position}")

    def calculate_person_position(self, robot_position, distance_to_person, yaw):
        """
        Calcula la posición de la persona usando la distancia y la orientación del robot.
        """
        # Calcular la posición de la persona en el espacio global
        person_x = robot_position.x + distance_to_person * cos(yaw)
        person_y = robot_position.y + distance_to_person * sin(yaw)

        return Point(x=person_x, y=person_y, z=robot_position.z)  # Mantener el mismo nivel en z

    def calculate_fixed_position(self, robot_position, person_position):
        """
        Calcula un punto fijo a `dist_move` metros desde la persona, alejándose en línea recta.
        """
        # Dirección del robot hacia la persona
        direction_x = person_position.x - robot_position.x
        direction_y = person_position.y - robot_position.y
        norm = sqrt(direction_x**2 + direction_y**2)

        # Evitar divisiones por cero
        if norm == 0:
            rospy.logwarn("La dirección es cero; manteniendo posición actual.")
            return Point(x=robot_position.x, y=robot_position.y, z=robot_position.z)

        # Normalizar y calcular el punto objetivo
        return Point(
            x=person_position.x - self.dist_move * (direction_x / norm),
            y=person_position.y - self.dist_move * (direction_y / norm),
            z=person_position.z  # Mantener el nivel en z
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

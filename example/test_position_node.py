#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from math import cos, sin, sqrt
import random
from tf.transformations import quaternion_from_euler

class DistanceBasedPositionTrackerWithRandomRobot:
    def __init__(self):
        rospy.init_node('distance_position_tracker_random_robot', anonymous=True)

        # Suscriptor para la posición de la persona (PoseStamped)
        rospy.Subscriber('/person_position', PoseStamped, self.person_pose_callback)

        # Publicadores
        self.target_pub = rospy.Publisher('/robot_target_position', Point, queue_size=10)
        self.status_pub = rospy.Publisher('/person_tracker/status', String, queue_size=10)
        self.robot_pub = rospy.Publisher('/robot_position', Point, queue_size=10)  # Nuevo tópico para la posición del robot

        # Parámetros
        self.dist_stop_threshold = 2.0  # Umbral para detenerse si la persona está muy cerca
        self.dist_move = 2.0  # Distancia fija para moverse hacia la persona

        # Generar posición aleatoria del robot
        self.robot_position = self.generate_random_robot_position()
        self.robot_yaw = self.generate_random_robot_orientation()

        # Publicar la posición inicial del robot
        self.publish_robot_position()

    def generate_random_robot_position(self):
        """Genera una posición aleatoria para el robot en un rango determinado."""
        return Point(
            x=random.uniform(-5.0, 5.0),  # Rango aleatorio en X
            y=random.uniform(-5.0, 5.0),  # Rango aleatorio en Y
            z=0  # Nivel fijo en Z
        )

    def generate_random_robot_orientation(self):
        """Genera una orientación aleatoria (yaw) para el robot."""
        return random.uniform(-3.14, 3.14)  # Rango de -π a π

    def person_pose_callback(self, msg):
        """
        Callback al recibir la posición de la persona (PoseStamped).
        Calcula la posición objetivo y la publica.
        """
        # Extraer la distancia desde el campo `z`
        distance_to_person = msg.pose.position.z

        # Verificar si la distancia está por debajo del umbral
        if distance_to_person < self.dist_stop_threshold:
            self.publish_status("Persona muy cerca. Deteniendo movimiento.")
            rospy.loginfo("Persona muy cerca. Deteniendo movimiento.")
            return

        # Calcular la posición de la persona en relación al robot
        person_position = self.calculate_person_position(distance_to_person, self.robot_yaw)

        # Calcular la posición objetivo a una distancia fija de la persona
        target_position = self.calculate_fixed_position(person_position)

        # Publicar la posición objetivo, la posición del robot y el estado
        self.publish_target(target_position)
        self.publish_robot_position()

        rospy.loginfo(f"Distancia recibida: {distance_to_person}. Moviéndose a: {target_position}")

    def calculate_person_position(self, distance_to_person, yaw):
        """
        Calcula la posición de la persona usando la distancia y la orientación del robot.
        """
        # Calcular la posición de la persona en el espacio global
        person_x = self.robot_position.x + distance_to_person * cos(yaw)
        person_y = self.robot_position.y + distance_to_person * sin(yaw)

        return Point(x=person_x, y=person_y, z=self.robot_position.z)

    def calculate_fixed_position(self, person_position):
        """
        Calcula un punto fijo a `dist_move` metros desde la persona, alejándose en línea recta.
        """
        # Dirección del robot hacia la persona
        direction_x = person_position.x - self.robot_position.x
        direction_y = person_position.y - self.robot_position.y
        norm = sqrt(direction_x**2 + direction_y**2)

        # Evitar divisiones por cero
        if norm == 0:
            rospy.logwarn("La dirección es cero; manteniendo posición actual.")
            return self.robot_position

        # Normalizar dirección y calcular objetivo
        return Point(
            x=person_position.x - self.dist_move * (direction_x / norm),
            y=person_position.y - self.dist_move * (direction_y / norm),
            z=person_position.z  # Mantener el nivel en z
        )

    def publish_target(self, target_position):
        """Publica la posición objetivo del robot."""
        self.target_pub.publish(target_position)

    def publish_status(self, message):
        """Publica el estado actual del seguimiento."""
        self.status_pub.publish(message)

    def publish_robot_position(self):
        """Publica la posición actual del robot en el tópico correspondiente."""
        self.robot_pub.publish(self.robot_position)

    def run(self):
        rospy.loginfo(f"Posición inicial del robot: x={self.robot_position.x}, y={self.robot_position.y}, yaw={self.robot_yaw}")
        rospy.spin()

if __name__ == '__main__':
    try:
        tracker = DistanceBasedPositionTrackerWithRandomRobot()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
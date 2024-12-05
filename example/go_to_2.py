#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import PoseStamped
from math import sqrt
import tf.transformations as tf_trans  # Para conversiones de cuaterniones a Euler

from bender_skills import robot_factory

rospy.init_node("move_to_person_node")
print("--------------------------------------------------------------------")
robot = robot_factory.build(["navigation", "knowledge"], core=False)
print("--------------------------------------------------------------------")

class MoveToPersonNode:
    def __init__(self):
        # rospy.init_node('move_to_person_node', anonymous=True)

        # Suscriptor para la posición de la persona
        rospy.Subscriber('/fixed_person_pose', PoseStamped, self.person_pose_callback)

        # Umbrales
        self.dist_stop_threshold = 0.7  # Distancia mínima para no moverse

    def get_robot_position_and_orientation(self):
        """Obtiene la posición y orientación actual del robot."""
        robot_state = robot.navigation.where_i_am()
        
        # Extraer posición y orientación como atributos del objeto Pose
        robot_position = {
            "x": robot_state.position.x,
            "y": robot_state.position.y,
            "z": robot_state.position.z
        }

        # Extraer orientación (en cuaterniones) y convertir a yaw
        quaternion = (
            robot_state.orientation.x,
            robot_state.orientation.y,
            robot_state.orientation.z,
            robot_state.orientation.w,
        )
        euler = tf_trans.euler_from_quaternion(quaternion)  # Convierte a ángulos de Euler
        yaw = euler[2]  # El ángulo yaw (rotación en el plano XY)

        return robot_position, yaw


    def person_pose_callback(self, msg):
        """
        Callback al recibir la posición de la persona (PoseStamped).
        Calcula el punto intermedio entre el robot y la persona y navega hacia él.
        """
        # Obtener la posición y orientación del robot
        robot_position, _ = self.get_robot_position_and_orientation()

        # Obtener la posición de la persona desde el mensaje
        person_x = msg.pose.position.x
        person_y = msg.pose.position.y

        # Calcular la distancia entre el robot y la persona
        distance_to_person = sqrt((person_x - robot_position["x"])**2 + (person_y - robot_position["y"])**2)

        # Verificar si la distancia está por debajo del umbral
        if distance_to_person < self.dist_stop_threshold:
            rospy.loginfo("Persona muy cerca. Deteniendo movimiento.")
            return

        # Calcular el punto intermedio
        midpoint_x = (robot_position["x"] + person_x) / 2
        midpoint_y = (robot_position["y"] + person_y) / 2

        rospy.loginfo(f"Punto intermedio calculado: x={midpoint_x}, y={midpoint_y}")

        # Mover el robot al punto intermedio
        self.move_robot_to_target({"x": midpoint_x, "y": midpoint_y})

    def move_robot_to_target(self, target_position):
        """
        Usa la skill `go_to_point` para mover al robot a la posición objetivo.
        """
        try:
            x = target_position["x"]
            y = target_position["y"]
            degrees = 0  # Puedes cambiar si necesitas orientar el robot
            rospy.loginfo(f"Moviendo el robot a la posición objetivo: x={x}, y={y}")
            result = robot.navigation.go_to_point(x=x, y=y, degrees=degrees)
            if result:
                rospy.loginfo("Movimiento completado con éxito.")
            else:
                rospy.logwarn("El robot no pudo alcanzar el objetivo.")
        except Exception as e:
            rospy.logerr(f"Error al mover el robot: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        mover = MoveToPersonNode()
        mover.run()
    except rospy.ROSInterruptException:
        pass
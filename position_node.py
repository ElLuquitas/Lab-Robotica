#!/usr/bin/env python3.10

import rospy
from geometry_msgs.msg import Pose
from math import sqrt

class PositionTracker:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('position_tracker', anonymous=True)
        
        # Suscripción al topic donde recibe la posición estimada (ajusta el nombre del topic según tu sistema)
        rospy.Subscriber('/kalman_filter/estimated_position', Pose, self.position_callback)
        
        # Publicación del rastro de puntos, si es necesario
        self.track_pub = rospy.Publisher('/person_tracker/path', PoseArray, queue_size=10)
        
        # Lista para almacenar las posiciones de la persona
        self.positions = []
        
        # Parámetros de distancia
        self.dist_far = 3.0  # Umbral para "far"
        self.dist_near = 1.0  # Umbral para "near"
        self.dist_same = 0.6  # Umbral para "same" (detención)
        
        # Temporizador para actualizar cada segundo
        self.timer = rospy.Timer(rospy.Duration(2), self.update_position)

    def euclidean_distance(self, pos1, pos2):
        """Calcula la distancia euclidiana entre dos posiciones"""
        return sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

    def position_callback(self, msg):
        """Callback para recibir la posición estimada y decidir si almacenarla"""
        current_position = msg.position
        
        if self.positions:
            # Compara con la última posición almacenada
            last_position = self.positions[-1]
            distance = self.euclidean_distance(current_position, last_position)

            if distance > self.dist_far:
                # Si la posición es lejana, guárdala en la lista
                self.positions.append(current_position)
                rospy.loginfo("Posición lejana detectada, agregando punto.")
            elif self.dist_near <= distance <= self.dist_far:
                # Si la posición es cercana, reemplaza el último punto
                self.positions[-1] = current_position
                rospy.loginfo("Posición cercana, reemplazando el último punto.")
            elif distance < self.dist_same:
                # Si la posición es muy cercana (persona detenida), no hagas nada
                rospy.loginfo("Persona detenida, no se guarda el punto.")
        else:
            # Si la lista está vacía, guarda el primer punto
            self.positions.append(current_position)
            rospy.loginfo("Inicializando con el primer punto.")
        
        # Publica el rastro de puntos si necesitas visualizarlo en RViz
        self.publish_path()

    def publish_path(self):
        """Publica el rastro de puntos almacenados en un topic"""
        path_msg = PoseArray()
        path_msg.poses = [Pose(position=pos) for pos in self.positions]
        self.track_pub.publish(path_msg)
        
    def update_position(self, event):
        """Función de actualización cada segundo"""
        rospy.loginfo("Actualizando la posición...")
        # Aquí puedes agregar lógica adicional para la actualización si es necesario

    def run(self):
        # Mantiene el nodo activo
        rospy.spin()

if __name__ == '__main__':
    try:
        tracker = PositionTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
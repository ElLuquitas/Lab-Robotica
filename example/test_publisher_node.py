#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import PoseStamped
import random
from tf.transformations import quaternion_from_euler

class TestPublisherNode:
    def __init__(self):
        rospy.init_node('test_publisher_node', anonymous=True)

        # Publicador para la posición de la persona
        self.person_pub = rospy.Publisher('/person_position', PoseStamped, queue_size=10)

        # Frecuencia de publicación
        self.rate = rospy.Rate(0.001)  # 1 Hz

    def publish_test_data(self):
        """Genera y publica datos de prueba."""
        while not rospy.is_shutdown():
            # Crear un mensaje PoseStamped para la posición de la persona
            person_msg = PoseStamped()
            person_msg.header.stamp = rospy.Time.now()
            person_msg.header.frame_id = "map"

            # Generar una distancia aleatoria en z (representa la distancia a la persona)
            distance_to_person = random.uniform(0.5, 5.0)  # Entre 0.5 m y 5.0 m

            # Establecer valores aleatorios en x, y para la persona (puedes ignorarlos si no los usas)
            person_msg.pose.position.x = 0  # Ignorado en tu lógica actual
            person_msg.pose.position.y = 0  # Ignorado en tu lógica actual
            person_msg.pose.position.z = distance_to_person

            # Generar una orientación aleatoria en yaw (solo para probar)
            yaw = random.uniform(-3.14, 3.14)  # Rango de -π a π
            quaternion = quaternion_from_euler(0, 0, yaw)
            person_msg.pose.orientation.x = quaternion[0]
            person_msg.pose.orientation.y = quaternion[1]
            person_msg.pose.orientation.z = quaternion[2]
            person_msg.pose.orientation.w = quaternion[3]

            # Publicar la posición de la persona
            rospy.loginfo(f"Publicando posición de la persona con distancia z={distance_to_person:.2f} m")
            self.person_pub.publish(person_msg)

            # Dormir hasta la próxima iteración
            self.rate.sleep()

if __name__ == '__main__':
    try:
        test_publisher = TestPublisherNode()
        test_publisher.publish_test_data()
    except rospy.ROSInterruptException:
        pass

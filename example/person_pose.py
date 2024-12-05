#!/usr/bin/env python3.10
import rospy
from geometry_msgs.msg import PoseStamped

class FixedPoseStampedPublisher:
    def __init__(self):
        rospy.init_node('fixed_pose_stamped_publisher', anonymous=True)

        # Publicador
        self.publisher = rospy.Publisher('/fixed_person_pose', PoseStamped, queue_size=10)

        # Solicitar al usuario la posición deseada

        # Crear PoseStamped con la posición ingresada
        self.fixed_pose = PoseStamped()
        self.fixed_pose.header.frame_id = "map"  # Cambiar según el marco de referencia que uses
        self.fixed_pose.pose.position.x = 1.6433
        self.fixed_pose.pose.position.y = 3.0000
        self.fixed_pose.pose.position.z = 0.0

        # Orientación en cero (sin rotación)
        self.fixed_pose.pose.orientation.x = 0.0
        self.fixed_pose.pose.orientation.y = 0.0
        self.fixed_pose.pose.orientation.z = 0.0
        self.fixed_pose.pose.orientation.w = 1.0

        # Frecuencia de publicación (en Hz)
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_pose(self):
        rospy.loginfo(f"Publicando PoseStamped fija: posición ({self.fixed_pose.pose.position.x}, {self.fixed_pose.pose.position.y})")
        while not rospy.is_shutdown():
            # Actualizar el tiempo en la cabecera
            self.fixed_pose.header.stamp = rospy.Time.now()

            # Publicar el mensaje
            self.publisher.publish(self.fixed_pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FixedPoseStampedPublisher()
        node.publish_pose()
    except rospy.ROSInterruptException:
        pass
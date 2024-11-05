import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Inicializar CvBridge
bridge = CvBridge()

def stereo_image_callback(data):
    # Convertir el mensaje de ROS Image a un formato que OpenCV pueda manejar
    try:
        # Si la imagen es de tipo color, el encoding probablemente sea 'bgr8'
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        # Mostrar la imagen usando OpenCV
        cv2.imshow("Stereo Camera", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        print("Error al convertir la imagen:", e)

def main():
    # Inicializar el nodo ROS
    rospy.init_node('stereo_image_listener', anonymous=True)

    # Suscribirse al topico
    rospy.Subscriber('/maqui/camera/front/image_raw', Image, stereo_image_callback)

    # Mantener el script en ejecucion
    rospy.spin()

if __name__ == '__main__':
    main()
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

# Inicializar CvBridge
bridge = CvBridge()

# Crear un publicador global
pub = None

def depth_image_callback(data):
    global pub
    try:
        # Convertir el mensaje de ROS Image a un formato que OpenCV pueda manejar
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        # Definir un area cuadrada en el centro de la imagen y luego moverla unos pixeles hacia abajo
        height_orig, width_orig = cv_image.shape
        center_x, center_y = width_orig // 2, height_orig // 2
        center_y += 50

        # Tamano del area cuadrada
        box_size = 20  # Puedes ajustar el tamano del area segun tus necesidades
        half_box = box_size // 2

        # Extraer la region de interes (ROI) alrededor del centro
        roi = cv_image[center_y - half_box:center_y + half_box, center_x - half_box:center_x + half_box]

        # Calcular la distancia promedio en la region de interes
        mean_depth = np.mean(roi)

        # Mostrar la distancia promedio en el area cuadrada
        # print("Average depth in the central area: {} millimeters".format(mean_depth))

        # Publicar la distancia promedio en un topico
        pub.publish(mean_depth)

    except Exception as e:
        print("Error al convertir la imagen:", e)

def main():
    global pub

    # Inicializar el nodo ROS
    rospy.init_node('depth_image_listener', anonymous=True)

    # Configurar el publicador
    pub = rospy.Publisher('/depth/average_distance', Float32, queue_size=10)

    # Suscribirse al topico de imagenes de profundidad
    rospy.Subscriber('/maqui/camera/depth/image_raw', Image, depth_image_callback)

    # Mantener el script en ejecucion
    rospy.spin()

if __name__ == '__main__':
    main()
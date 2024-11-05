import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Inicializar CvBridge
bridge = CvBridge()

def depth_image_callback(data):
    try:
        # Convertir el mensaje de ROS Image a un formato que OpenCV pueda manejar
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        # Si es necesario, puedes normalizar o convertir la imagen para su visualizacion
        cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        cv_image_uint8 = cv_image_normalized.astype('uint8')

        # Redimensionar la imagen (por ejemplo, duplicar el tamano original)
        scale = 2
        width = int(cv_image_uint8.shape[1] * scale)
        height = int(cv_image_uint8.shape[0] * scale)
        dim = (width, height)
        cv_image_resized = cv2.resize(cv_image_uint8, dim, interpolation=cv2.INTER_LINEAR)

        # Definir un area cuadrada en el centro de la imagen y luego moverla unos pixeles hacia abajo
        # para no tomar en cuenta el verdadero centro de la imagen
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
        print("Average depth in the central area: {} milimeters".format(mean_depth))

        # Dibujar el cuadrado en la imagen redimensionada con los pixeles de la region de interes,
        # teniendo en cuenta que la region fue movida del centro de la imagen real
        top_left = (width // 2 - half_box * 2, height // 2 - half_box * 2 + 50)
        bottom_right = (width // 2 + half_box * 2, height // 2 + half_box * 2 + 50)
        cv2.rectangle(cv_image_resized, top_left, bottom_right, (255, 0, 0), 2)

        # Mostrar la imagen redimensionada con el cuadrado
        cv2.imshow("Depth Image", cv_image_resized)
        cv2.waitKey(1)

    except Exception as e:
        print("Error al convertir la imagen:", e)

def main():
    # Inicializar el nodo ROS
    rospy.init_node('depth_image_listener', anonymous=True)

    # Suscribirse al topico
    rospy.Subscriber('/maqui/camera/depth/image_raw', Image, depth_image_callback)

    # Mantener el script en ejecucion
    rospy.spin()

if __name__ == '__main__':
    main()
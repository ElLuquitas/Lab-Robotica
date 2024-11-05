import torch
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np

# Inicializar CvBridge
bridge = CvBridge()

# Crear un publicador global para ROS
pub = None

# Parámetros de la cámara
KNOWN_TRUNK_HEIGHT = 0.75  # Altura promedio del tronco humano en metros
KNOWN_FACE_HEIGHT = 0.24   # Altura promedio del rostro humano en metros
FOCAL_LENGTH = 800         # Distancia focal estimada en píxeles (ajústala según tu cámara)

# Cargar el modelo YOLOv5 preentrenado para detección de personas
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Cargar el modelo de detección de rostros de OpenCV
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Variable para guardar el ROI de la detección de YOLO (será actualizado en cada frame)
current_roi = None

def estimate_distance(bbox_height, known_height, focal_length):
    """Calcular la distancia entre la cámara y el objeto (tronco o rostro)."""
    if bbox_height == 0:
        return None
    distance = (known_height * focal_length) / bbox_height
    return distance

def depth_image_callback(data):
    """Callback para manejar imágenes de profundidad desde ROS."""
    global pub, current_roi
    if current_roi is None:
        return  # Si no hay ROI definida aún, no hacemos nada

    try:
        # Convertir el mensaje de ROS Image a un formato que OpenCV pueda manejar
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        # Extraer la región de interés (ROI) alrededor del tronco o rostro detectado
        (x_min, y_min, x_max, y_max) = current_roi
        roi = cv_image[int(y_min):int(y_max), int(x_min):int(x_max)]

        # Calcular la distancia promedio en la región de interés
        mean_depth = np.mean(roi)

        # Publicar la distancia promedio en un tópico de ROS
        pub.publish(mean_depth)

        # Mostrar la distancia en la consola
        print(f"Distancia promedio en el ROI (cámara de profundidad): {mean_depth:.2f} mm")

    except Exception as e:
        print("Error al procesar la imagen de profundidad:", e)

def yolo_detection_and_depth_estimation():
    """Función principal para detección con YOLOv5 y estimación de distancias con la cámara de profundidad."""
    global current_roi

    # Capturar video de la cámara
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error al abrir la cámara")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("No se pudo obtener el frame de la cámara.")
            break

        # Realizar predicciones de YOLOv5 para personas
        results = model(frame)

        # Filtrar detecciones de personas
        detections = results.xyxy[0]  # [x_min, y_min, x_max, y_max, confidence, class]
        for detection in detections:
            x_min, y_min, x_max, y_max, confidence, class_id = detection
            if int(class_id) == 0:  # Clase 0 corresponde a 'persona'
                bbox_height = y_max - y_min
                trunk_height = bbox_height * 0.5  # Aproximación de la altura del tronco

                # Detectar el rostro dentro de la bounding box de la persona usando Haar cascades
                person_roi = frame[int(y_min):int(y_max), int(x_min):int(x_max)]
                gray_person_roi = cv2.cvtColor(person_roi, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray_person_roi, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

                if len(faces) > 0:
                    # Si se detecta un rostro, usamos el rostro para la distancia
                    for (fx, fy, fw, fh) in faces:
                        face_height = fh  # Altura del rostro en píxeles
                        distance = estimate_distance(face_height, KNOWN_FACE_HEIGHT, FOCAL_LENGTH)

                        # Actualizar el ROI para la cámara de profundidad basado en la detección del rostro
                        current_roi = (int(x_min + fx), int(y_min + fy), int(x_min + fx + fw), int(y_min + fy + fh))

                        # Dibujar la bounding box del rostro
                        cv2.rectangle(person_roi, (fx, fy), (fx + fw, fy + fh), (255, 0, 0), 2)

                        # Mostrar la distancia estimada en la consola
                        print(f"Distancia estimada al rostro: {distance:.2f} metros")
                        break
                else:
                    # Si no se detecta un rostro, usamos el tronco para la distancia
                    distance = estimate_distance(trunk_height, KNOWN_TRUNK_HEIGHT, FOCAL_LENGTH)

                    # Actualizar el ROI para la cámara de profundidad basado en la detección del tronco
                    current_roi = (int(x_min), int(y_min + (y_max - y_min) * 0.25), int(x_max), int(y_min + (y_max - y_min) * 0.75))

                    # Dibujar la bounding box de la persona
                    cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)

                    # Mostrar la distancia estimada en la consola
                    print(f"Distancia estimada al tronco: {distance:.2f} metros")

        # Mostrar el frame con las detecciones
        cv2.imshow('Detección YOLOv5 + Detección de Rostros', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    global pub

    # Inicializar el nodo ROS
    rospy.init_node('depth_and_yolo_combination', anonymous=True)

    # Configurar el publicador para la distancia promedio
    pub = rospy.Publisher('/depth/average_distance', Float32, queue_size=10)

    # Suscribirse al tópico de imágenes de profundidad
    rospy.Subscriber('/maqui/camera/depth/image_raw', Image, depth_image_callback)

    # Iniciar detección de YOLOv5 y estimación de distancia
    yolo_detection_and_depth_estimation()

    # Mantener el script en ejecución
    rospy.spin()

if __name__ == '__main__':
    main()

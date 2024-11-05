import torch
import cv2

# Parámetros conocidos
KNOWN_TRUNK_HEIGHT = 0.75  # Altura promedio del tronco humano en metros
KNOWN_FACE_HEIGHT = 0.24   # Altura promedio del rostro humano en metros
FOCAL_LENGTH = 650         # Distancia focal estimada en píxeles (ajústala según tu cámara)

def estimate_distance(bbox_height, known_height, focal_length):
    """Calcular la distancia entre la cámara y el objeto (tronco o rostro)."""
    if bbox_height == 0:
        return None
    distance = (known_height * focal_length) / bbox_height
    return distance

# Cargar el modelo YOLOv5 preentrenado para detección de personas
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Cargar el modelo de detección de rostros de OpenCV
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Capturar video de la cámara
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error al abrir la cámara")
    exit()

while cap.isOpened():
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
        if int(class_id) == 0:  # Clase 0 corresponde a 'persona' en COCO dataset
            bbox_height = y_max - y_min
            trunk_height = bbox_height * 0.5  # Aproximación de la altura del tronco (50% de la persona)

            # Detectar el rostro dentro de la bounding box de la persona usando Haar cascades
            person_roi = frame[int(y_min):int(y_max), int(x_min):int(x_max)]
            gray_person_roi = cv2.cvtColor(person_roi, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray_person_roi, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            if len(faces) > 0:
                # Si se detecta un rostro, calcular la distancia usando el rostro
                for (fx, fy, fw, fh) in faces:
                    face_height = fh  # Altura del rostro en píxeles
                    distance = estimate_distance(face_height, KNOWN_FACE_HEIGHT, FOCAL_LENGTH)

                    # Dibujar la bounding box del rostro
                    cv2.rectangle(person_roi, (fx, fy), (fx + fw, fy + fh), (255, 0, 0), 2)

                    # Mostrar la distancia estimada en la consola
                    print(f"Distancia estimada al rostro: {distance:.2f} metros")

                    # Mostrar la distancia sobre la imagen
                    cv2.putText(frame, f"Distancia: {distance:.2f}m", (int(x_min), int(y_min)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

                    break  # Solo usamos el primer rostro detectado
            else:
                # Si no se detecta un rostro, calcular la distancia usando el tronco
                distance = estimate_distance(trunk_height, KNOWN_TRUNK_HEIGHT, FOCAL_LENGTH)

                # Mostrar la distancia estimada en la consola
                print(f"Distancia estimada al tronco: {distance:.2f} metros")

                # Dibujar la bounding box de la persona
                cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)

                # Mostrar la distancia sobre la imagen
                cv2.putText(frame, f"Distancia: {distance:.2f}m", (int(x_min), int(y_min)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    # Mostrar el frame con las detecciones
    cv2.imshow('Detección YOLOv5 + Detección de Rostros', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
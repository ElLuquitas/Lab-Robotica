import math

def estimate_person_map(distance, actual_position, ideal_distance = 900):
    # Ubicación de la persona en coordenadas del mapa según posición de Pepper
    x1 = actual_position['position']['x'] + distance * math.cos(actual_position['orientation']['w'])
    y1 = actual_position['position']['y'] + distance * math.sin(actual_position['orientation']['w'])

    # Pepper queda a 900 mm aprox. de la persona para interactuar
    x_position = x1 - ideal_distance
    y_position = y1 - ideal_distance
    w_position = actual_position['orientation']['w']

    return x_position, y_position, w_position
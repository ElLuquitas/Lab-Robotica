#!/usr/bin/env python3.9

import mediapipe as mp
import cv2
import csv 
import os 
import numpy as np
import pickle 
import pandas as pd
import warnings



warnings.filterwarnings(action='ignore', category=UserWarning)

def obtener_indice_mayor_probabilidad(lista):
    indice_mayor_probabilidad = 0
    mayor_probabilidad = max(lista[0])
    if len(lista)==1:
        return 0
    for i in range(1,len(lista)):
        max_probabilidad_actual = max(lista[i])
        print(max_probabilidad_actual, mayor_probabilidad)
        if max_probabilidad_actual > mayor_probabilidad:
            mayor_probabilidad = max_probabilidad_actual
            indice_mayor_probabilidad = i

    return indice_mayor_probabilidad

class Hand_Gesture_Detector():

    def __init__(self):
        self.mp_drawing = mp.solutions.drawing_utils # Drawing helpers
        self.mp_holistic = mp.solutions.holistic # Mediapipe Solutions

        with open('hand_gestures.pkl', 'rb') as f:
            self.model = pickle.load(f)
    
    def start(self):
        cap = cv2.VideoCapture(0)

        # Initiate holistic model
        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            
            while cap.isOpened():
                ret, frame = cap.read()
                
                # Recolor Feed
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False        
                
                # Make Detections
                results = holistic.process(image)
                # print(results.face_landmarks)
                
                # face_landmarks, pose_landmarks, left_hand_landmarks, right_hand_landmarks
                
                # Recolor image back to BGR for rendering
                image.flags.writeable = True   
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                # 1. Draw face landmarks
                self.mp_drawing.draw_landmarks(image, results.face_landmarks, self.mp_holistic.FACEMESH_TESSELATION, 
                                        self.mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                        self.mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                        )
                
                # 2. Right hand
                self.mp_drawing.draw_landmarks(image, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS, 
                                        self.mp_drawing.DrawingSpec(color=(80,22,10), thickness=2, circle_radius=4),
                                        self.mp_drawing.DrawingSpec(color=(80,44,121), thickness=2, circle_radius=2)
                                        )

                # 3. Left Hand
                self.mp_drawing.draw_landmarks(image, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS, 
                                        self.mp_drawing.DrawingSpec(color=(121,22,76), thickness=2, circle_radius=4),
                                        self.mp_drawing.DrawingSpec(color=(121,44,250), thickness=2, circle_radius=2)
                                        )

                # 4. Pose Detections
                self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS, 
                                        self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                        self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                        )
                # Export coordinates
                try:
                    row1, row2 = None, None
                    body_language_prob_1, body_language_prob_2 = None, None
                    body_language_class_1, body_language_class_2 = None, None
                    if type(results.right_hand_landmarks) != type(None):
                        # Extract Right Hand landmarks
                        right_hand = results.right_hand_landmarks.landmark
                        right_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in right_hand]).flatten())
                    
                        row1 = right_hand_row

                        # Make Detections
                        X1 = pd.DataFrame([row1])
                        body_language_class_1 = self.model.predict(X1)[0]
                        body_language_prob_1 = self.model.predict_proba(X1)[0]
                        print('Right Hand', body_language_class_1, body_language_prob_1)

                    if type(results.left_hand_landmarks) != type(None):
                        # Extract Left Hand landmarks
                        left_hand = results.left_hand_landmarks.landmark
                        left_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in left_hand]).flatten())
                        
                        row2 = left_hand_row

                        # Make Detections
                        X2 = pd.DataFrame([row2])
                        body_language_class_2 = self.model.predict(X2)[0]
                        body_language_prob_2 = self.model.predict_proba(X2)[0]
                        print('Left Hand', body_language_class_2, body_language_prob_2)

                    probs = [blp for blp in [body_language_prob_1, body_language_prob_2] if type(blp) != type(None)]   
                    classes = [blc for blc in [body_language_class_1, body_language_class_2] if type(blc) != type(None)]
                    if len(probs) != 0:
                        idx_max = obtener_indice_mayor_probabilidad(probs)

                    body_language_prob = probs[idx_max]
                    body_language_class = classes[idx_max] 

                    if max(body_language_prob) > 0.5:             
                        # Grab ear coords
                        coords = tuple(np.multiply(
                                        np.array(
                                            (results.pose_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_EAR].x, 
                                            results.pose_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_EAR].y))
                                    , [640,480]).astype(int))
                        
                        cv2.rectangle(image, 
                                    (coords[0], coords[1]+5), 
                                    (coords[0]+len(body_language_class)*20, coords[1]-30), 
                                    (245, 117, 16), -1)
                        cv2.putText(image, body_language_class, coords, 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                        
                        # Get status box
                        cv2.rectangle(image, (0,0), (250, 60), (245, 117, 16), -1)
                        
                        # Display Class
                        cv2.putText(image, 'CLASS'
                                    , (95,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                        cv2.putText(image, body_language_class.split(' ')[0]
                                    , (90,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                        
                        # Display Probability
                        cv2.putText(image, 'PROB'
                                    , (15,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                        cv2.putText(image, str(round(body_language_prob[np.argmax(body_language_prob)],2))
                                    , (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    
                except:
                    pass
                                
                cv2.imshow('Raw Webcam Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

        cap.release()
        cv2.destroyAllWindows()

    def detect_single_frame(self, frame):
        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:

            # Recolor Feed
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False        
            
            # Make Detections
            results = holistic.process(image)
            # print(results.face_landmarks)
            
            # face_landmarks, pose_landmarks, left_hand_landmarks, right_hand_landmarks
            
            # Recolor image back to BGR for rendering
            image.flags.writeable = True   
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # 1. Draw face landmarks
            self.mp_drawing.draw_landmarks(image, results.face_landmarks, self.mp_holistic.FACEMESH_TESSELATION, 
                                    self.mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                    self.mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                    )
            
            # 2. Right hand
            self.mp_drawing.draw_landmarks(image, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS, 
                                    self.mp_drawing.DrawingSpec(color=(80,22,10), thickness=2, circle_radius=4),
                                    self.mp_drawing.DrawingSpec(color=(80,44,121), thickness=2, circle_radius=2)
                                    )

            # 3. Left Hand
            self.mp_drawing.draw_landmarks(image, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS, 
                                    self.mp_drawing.DrawingSpec(color=(121,22,76), thickness=2, circle_radius=4),
                                    self.mp_drawing.DrawingSpec(color=(121,44,250), thickness=2, circle_radius=2)
                                    )

            # 4. Pose Detections
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS, 
                                    self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                    self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                    )
            # Export coordinates
            try:
                row1, row2 = None, None
                body_language_prob_1, body_language_prob_2 = None, None
                body_language_class_1, body_language_class_2 = None, None
                if type(results.right_hand_landmarks) != type(None):
                    # Extract Right Hand landmarks
                    right_hand = results.right_hand_landmarks.landmark
                    right_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in right_hand]).flatten())
                
                    row1 = right_hand_row

                    # Make Detections
                    X1 = pd.DataFrame([row1])
                    body_language_class_1 = self.model.predict(X1)[0]
                    body_language_prob_1 = self.model.predict_proba(X1)[0]
                    print('Right Hand', body_language_class_1, body_language_prob_1)

                if type(results.left_hand_landmarks) != type(None):
                    # Extract Left Hand landmarks
                    left_hand = results.left_hand_landmarks.landmark
                    left_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in left_hand]).flatten())
                    
                    row2 = left_hand_row

                    # Make Detections
                    X2 = pd.DataFrame([row2])
                    body_language_class_2 = self.model.predict(X2)[0]
                    body_language_prob_2 = self.model.predict_proba(X2)[0]
                    print('Left Hand', body_language_class_2, body_language_prob_2)

                probs = [blp for blp in [body_language_prob_1, body_language_prob_2] if type(blp) != type(None)]   
                classes = [blc for blc in [body_language_class_1, body_language_class_2] if type(blc) != type(None)]
                if len(probs) != 0:
                    idx_max = obtener_indice_mayor_probabilidad(probs)

                body_language_prob = probs[idx_max]
                body_language_class = classes[idx_max] 

                if max(body_language_prob) > 0.5:             
                    # Grab ear coords
                    coords = tuple(np.multiply(
                                    np.array(
                                        (results.pose_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_EAR].x, 
                                        results.pose_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_EAR].y))
                                , [640,480]).astype(int))
                    
                    cv2.rectangle(image, 
                                (coords[0], coords[1]+5), 
                                (coords[0]+len(body_language_class)*20, coords[1]-30), 
                                (245, 117, 16), -1)
                    cv2.putText(image, body_language_class, coords, 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    
                    # Get status box
                    cv2.rectangle(image, (0,0), (250, 60), (245, 117, 16), -1)
                    
                    # Display Class
                    cv2.putText(image, 'CLASS'
                                , (95,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(image, body_language_class.split(' ')[0]
                                , (90,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    
                    # Display Probability
                    cv2.putText(image, 'PROB'
                                , (15,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(image, str(round(body_language_prob[np.argmax(body_language_prob)],2))
                                , (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                
                return image
            
            except:
                return None
            
    def get_proba(self,frame):

        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:

            # Recolor Feed
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False        
            
            # Make Detections
            results = holistic.process(image)
            # print(results.face_landmarks)
            
            # face_landmarks, pose_landmarks, left_hand_landmarks, right_hand_landmarks
            
            # Recolor image back to BGR for rendering
            image.flags.writeable = True   
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Export coordinates
            try:
                row1, row2 = None, None
                body_language_prob_1, body_language_prob_2 = None, None
                body_language_class_1, body_language_class_2 = None, None
                if type(results.right_hand_landmarks) != type(None):
                    # Extract Right Hand landmarks
                    right_hand = results.right_hand_landmarks.landmark
                    right_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in right_hand]).flatten())
                
                    row1 = right_hand_row

                    # Make Detections
                    X1 = pd.DataFrame([row1])
                    body_language_class_1 = self.model.predict(X1)[0]
                    body_language_prob_1 = self.model.predict_proba(X1)[0]

                if type(results.left_hand_landmarks) != type(None):
                    # Extract Left Hand landmarks
                    left_hand = results.left_hand_landmarks.landmark
                    left_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in left_hand]).flatten())
                    
                    row2 = left_hand_row

                    # Make Detections
                    X2 = pd.DataFrame([row2])
                    body_language_class_2 = self.model.predict(X2)[0]
                    body_language_prob_2 = self.model.predict_proba(X2)[0]

                probs = [blp for blp in [body_language_prob_1, body_language_prob_2] if type(blp) != type(None)]   
                classes = [blc for blc in [body_language_class_1, body_language_class_2] if type(blc) != type(None)]
                if len(probs) != 0:
                    idx_max = obtener_indice_mayor_probabilidad(probs)

                body_language_prob = probs[idx_max]
                body_language_class = classes[idx_max] 

                print(f'{body_language_class}|{body_language_prob}')
                return [body_language_class, body_language_prob]
            
            except:
                return None


if __name__ == "__main__":
    detector = Hand_Gesture_Detector()
    detector.start()
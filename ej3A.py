import cv2
import os
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import time
import serial

flag_serial = True

if flag_serial:
    ser = serial.Serial('/dev/ttyACM0', 115200)

def serialConnect(message, flag=flag_serial):
    global ser
    if flag:
        ser.write(message.encode())
    print(message)



# Flags y listas
shapes_list = ["Triangulo", "Cuadrado", "Circulo"]
shape_index = 1
shape_haunt = shapes_list[shape_index]

color_list = ["RED", "YELLOW", "GREEN", "BLUE"]
color_index = 0
color_haunt = color_list[color_index]

tracker = None
initBB = None
tracking = False

comando = ""
comando_anterior = ""

def toggle_haunt(key):
    global shape_index, color_index, shape_haunt, color_haunt, tracking
    if key == ord('a'):  # a
        shape_index = (shape_index - 1) % len(shapes_list)
        shape_haunt = shapes_list[shape_index]
        print("[Shape]", shape_haunt)
        tracking = False
    elif key == ord('d'):  # d
        shape_index = (shape_index + 1) % len(shapes_list)
        shape_haunt = shapes_list[shape_index]
        print("[Shape]", shape_haunt)
        tracking = False
    elif key == ord('w'):  # Flecha arriba
        color_index = (color_index + 1) % len(color_list)
        color_haunt = color_list[color_index]
        print("[Color]", color_haunt)
        tracking = False
    elif key == ord('s'):  # Flecha abajo
        color_index = (color_index - 1) % len(color_list)
        color_haunt = color_list[color_index]
        print("[Color]", color_haunt)
        tracking = False


def save_image(frame, contador=[1]):
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        nombre = f"imagen{contador[0]}.jpg"
        cv2.imwrite(nombre, frame)
        print(f"[INFO] Imagen guardada: {nombre}")
        contador[0] += 1


def detect_shapes(frame, masked):
    global shape_haunt, initBB, tracker, tracking
    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = frame.copy()
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= 3500]

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)
        x, y, w, h = cv2.boundingRect(approx)

        if len(approx) == 3:
            shape = "Triangulo"
        elif len(approx) == 4:
            shape = "Cuadrado"
        elif len(approx) > 4:
            area = cv2.contourArea(largest_contour)
            perimeter = cv2.arcLength(largest_contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter + 1e-6)
            if circularity > 0.75:
                shape = "Circulo"
            else:
                shape = "unrecognized"
        else:
            shape = "unrecognized"

        cv2.drawContours(output, [approx], -1, (0, 255, 0), 2)
        cv2.putText(output, shape+str(cv2.contourArea(largest_contour)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        if shape == shape_haunt and not tracking:
            print("[INFO] OBJETO DETECTADO, INICIANDO TRACKER")
            initBB = (x, y, w, h)
            tracker = cv2.legacy.TrackerCSRT_create()
            tracker.init(frame, initBB)
            tracking = True

    return output


def look_color(frame, color, secondary=False):
    bias = 5 if secondary else 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    if color == "RED":
        lower1 = np.array([0, 0, 0])
        upper1 = np.array([0, 0, 0])
        lower2 = np.array([102-bias, 77-bias, 160-bias])
        upper2 = np.array([158+bias, 158+bias, 255+bias])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
#ROSADO
    elif color == "YELLOW":
        lower = np.array([70-bias, 134-bias, 0-bias])
        upper = np.array([100+bias, 199+bias, 245+bias])
        mask = cv2.inRange(hsv, lower, upper)

    elif color == "GREEN":
        lower = np.array([64-bias, 39-bias, 0-bias])
        upper = np.array([103+bias, 255+bias, 225+bias])
        mask = cv2.inRange(hsv, lower, upper)

    elif color == "BLUE":
        lower = np.array([4-bias, 35-bias, 108-bias])
        upper = np.array([34+bias, 175+bias, 215+bias])
        mask = cv2.inRange(hsv, lower, upper)

    else:
        print("not recpgnized")
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    output = cv2.bitwise_and(frame, frame, mask=mask)
    return mask, output


def get_direction(frame, bbox):
    global comando, comando_anterior
    (x, y, w, h) = [int(v) for v in bbox]
    cx = x + w // 2
    cy = y + h // 2
    area_bbox = w*h
    center_x = frame.shape[1] // 2
    center_y = frame.shape[0] // 2
    margin = 30

    if cx < center_x - margin:
        direccion = "L"
    elif cx > center_x + margin:
        direccion = "R"
    else:
        direccion = "S"

    if area_bbox < 3600:
        distancia = "A"
    elif area_bbox > 31800:
        distancia = "r"
    elif 10000 <= area_bbox <= 20000:
        distancia = "S"
    else:
        distancia = "S"
    
    if distancia=="r": 
        comando="r\n"
    else:
        comando = direccion+distancia+"\n"

    if comando != comando_anterior:
        print("SE ESTA ENVIANDO: ", comando)
        serialConnect(comando)
        comando_anterior = comando

##############################3


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("No se pudo abrir la cámara.")

counter = 0
#_______________________loop_________________________________
while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (320, 240))
    if not ret:
        break

    # Cambio de flags con teclado
    key = cv2.waitKey(1) & 0xFF
    if key != 255:
        print(key)
    if key == 27:
        break
    elif key == ord('c'):
        save_image(frame)
    else:
        toggle_haunt(key)

    if tracking and tracker is not None: 
        success, box = tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            roi_mask = color_mask[y:y+h, x:x+w]
            area_mask = cv2.countNonZero(roi_mask)
            area_bbox = w * h
            ratio = area_mask / area_bbox if area_bbox > 0 else 0

            if shape_haunt=="Triangulo" and ratio > 0.4 or ratio>0.65:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                get_direction(frame, box)
            else:
                counter +=1
                if counter ==15:
                    print("[INFO] Objeto ya no coincide con color esperado. Tracker detenido.")
                    print(f"[DEBUG] Ratio en ROI: {ratio:.2f}")
                    tracking = False
                    tracker = None
                    initBB = None
                    counter=0
        else:
            print("[INFO] Tracker perdido")
            tracking = False
            tracker = None
            initBB = None

    else:
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        color_mask, frame_masked = look_color(frame, color_haunt)
        kernel = np.ones((5, 5), np.uint8)
        opened = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilated = cv2.dilate(opened, kernel)
        shapes = detect_shapes(frame, masked=dilated)
        cv2.imshow("frame color", dilated)
        cv2.imshow("shape", shapes)
        comando = "P\n"
        if comando != comando_anterior:
            print("SE ESTA ENVIANDO: ", comando)
            serialConnect(comando)
            comando_anterior = comando
        

    cv2.imshow("tracking", frame)

cap.release()
cv2.destroyAllWindows()

## 3600 area debe acercarse
## 31800 area que debe alejarse
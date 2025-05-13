import cv2
import numpy as np
import serial
import threading
from queue import Queue

# =============================================
#           CONFIGURACIÓN INICIAL
# =============================================
flag_serial = True  # Cambia a False si no usas serial

# Listas de figuras y colores
shapes_list = ["Triangulo", "Cuadrado", "Circulo"]
color_list = ["RED", "YELLOW", "GREEN", "BLUE"]
shape_index, color_index = 1, 0
shape_haunt = shapes_list[shape_index]
color_haunt = color_list[color_index]

# Variables de estado
tracker = None
initBB = None
tracking = False
comando = ""
comando_anterior = ""
color_mask = None  # Para almacenar la máscara de color

# Configuración serial (si está habilitada)
if flag_serial:
    ser = serial.Serial('/dev/ttyACM0', 115200)

# =============================================
#           COLAS PARA COMUNICACIÓN
# =============================================
frame_queue = Queue(maxsize=1)    # Frames de la cámara
command_queue = Queue(maxsize=1)  # Comandos para enviar por serial
display_queues = {                # Ventanas para mostrar
    "tracking": Queue(maxsize=1),
    "shape": Queue(maxsize=1),
    "frame color": Queue(maxsize=1)
}

# =============================================
#               FUNCIONES
# =============================================
def serialConnect(message, flag=flag_serial):
    if flag:
        ser.write(message.encode())
    print(message)

def toggle_haunt(key):
    global shape_index, color_index, shape_haunt, color_haunt, tracking
    if key == ord('a'):  # Tecla 'a'
        shape_index = (shape_index - 1) % len(shapes_list)
        shape_haunt = shapes_list[shape_index]
        print("[Shape]", shape_haunt)
        tracking = False
    elif key == ord('d'):  # Tecla 'd'
        shape_index = (shape_index + 1) % len(shapes_list)
        shape_haunt = shapes_list[shape_index]
        print("[Shape]", shape_haunt)
        tracking = False
    elif key == ord('w'):  # Tecla 'w'
        color_index = (color_index + 1) % len(color_list)
        color_haunt = color_list[color_index]
        print("[Color]", color_haunt)
        tracking = False
    elif key == ord('s'):  # Tecla 's'
        color_index = (color_index - 1) % len(color_list)
        color_haunt = color_list[color_index]
        print("[Color]", color_haunt)
        tracking = False

def save_image(frame, contador=[1]):
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
            shape = "Circulo" if circularity > 0.75 else "unrecognized"
        else:
            shape = "unrecognized"

        cv2.drawContours(output, [approx], -1, (0, 255, 0), 2)
        cv2.putText(output, f"{shape} {cv2.contourArea(largest_contour)}", (x, y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        if shape == shape_haunt and not tracking:
            print("[INFO] OBJETO DETECTADO, INICIANDO TRACKER")
            initBB = (x, y, w, h)
            tracker = cv2.legacy.TrackerCSRT_create()
            tracker.init(frame, initBB)
            tracking = True

    return output

def look_color(frame, color, secondary=False):
    bias = 15 if secondary else 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    if color == "RED":
        lower1 = np.array([0, 0, 0])
        upper1 = np.array([0, 0, 0])
        lower2 = np.array([74-bias, 121-bias, 156-bias])
        upper2 = np.array([141+bias, 193+bias, 217+bias])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == "YELLOW":
        lower = np.array([83-bias, 84-bias, 170-bias])
        upper = np.array([94+bias, 186+bias, 232+bias])
        mask = cv2.inRange(hsv, lower, upper)
    elif color == "GREEN":
        lower = np.array([66-bias, 100-bias, 88-bias])
        upper = np.array([90+bias, 209+bias, 179+bias])
        mask = cv2.inRange(hsv, lower, upper)
    elif color == "BLUE":
        lower = np.array([0-bias, 159-bias, 77-bias])
        upper = np.array([60+bias, 255+bias, 254+bias])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        print("Color no reconocido")
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    output = cv2.bitwise_and(frame, frame, mask=mask)
    return mask, output

def get_direction(frame, bbox):
    global comando, comando_anterior, color_mask
    (x, y, w, h) = [int(v) for v in bbox]
    cx = x + w // 2
    area_bbox = w * h
    center_x = frame.shape[1] // 2

    # Dirección (L/R/S)
    if cx < center_x - 30:
        direccion = "L"
    elif cx > center_x + 30:
        direccion = "R"
    else:
        direccion = "S"

    # Distancia (A/r/S) - ¡Mantiene tus prints originales!
    if area_bbox < 3600:
        distancia = "A"
        print("[DEBUG] Objeto pequeño → Acercarse (A)")
    elif area_bbox > 31800:
        distancia = "r"
        print("[DEBUG] Objeto grande → Alejarse (r)")
    else:
        distancia = "S"
        print("[DEBUG] Distancia adecuada → Mantener (S)")

    comando = direccion + distancia
    if comando != comando_anterior:
        command_queue.put(comando)  # Encola el comando para el hilo serial
        comando_anterior = comando

# =============================================
#                   HILOS
# =============================================
def video_capture_thread():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (320, 240))
        if frame_queue.empty():
            frame_queue.put(frame)
    cap.release()

def processing_thread():
    global tracker, initBB, tracking, color_mask
    
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            
            if tracking and tracker is not None:
                success, box = tracker.update(frame)
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    roi_mask = color_mask[y:y+h, x:x+w]
                    area_mask = cv2.countNonZero(roi_mask)
                    ratio = area_mask / (w * h) if (w * h) > 0 else 0

                    if (shape_haunt == "Triangulo" and ratio > 0.4) or ratio > 0.65:
                        frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        get_direction(frame, box)
                else:
                    print("[INFO] Tracker perdido")
                    tracking = False
                    tracker = None
            else:
                frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
                color_mask, _ = look_color(frame_blur, color_haunt)
                opened = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                dilated = cv2.dilate(opened, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                shapes = detect_shapes(frame, dilated)
                
                # Encolar frames para mostrar
                display_queues["frame color"].put(dilated)
                display_queues["shape"].put(shapes)
            
            display_queues["tracking"].put(frame)

def serial_thread():
    while True:
        if not command_queue.empty():
            command = command_queue.get()
            serialConnect(command)

def interface_thread():
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # Tecla ESC
            break
        elif key == ord('c'):  # Guardar imagen
            if not frame_queue.empty():
                save_image(frame_queue.get())
        else:
            toggle_haunt(key)  # Cambiar figura/color
        
        # Actualizar ventanas
        for window_name, queue in display_queues.items():
            if not queue.empty():
                cv2.imshow(window_name, queue.get())

    cv2.destroyAllWindows()
    if flag_serial:
        ser.close()

# =============================================
#               INICIO DEL PROGRAMA
# =============================================
if __name__ == "__main__":
    # Crear e iniciar hilos
    threads = [
        threading.Thread(target=video_capture_thread),
        threading.Thread(target=processing_thread),
        threading.Thread(target=serial_thread),
        threading.Thread(target=interface_thread)
    ]

    for t in threads:
        t.daemon = True
        t.start()

    # Esperar a que la interfaz termine
    for t in threads:
        t.join()
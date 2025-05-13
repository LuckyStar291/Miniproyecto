import cv2
import numpy as np

def nothing(x):
    pass

# Cargar imagen en formato BGR
cap = cv2.VideoCapture(0)


# Crear ventana y trackbars
cv2.namedWindow("Filtro HSV")

cv2.createTrackbar("Min H", "Filtro HSV", 0, 179, nothing)
cv2.createTrackbar("Max H", "Filtro HSV", 179, 179, nothing)
cv2.createTrackbar("Min S", "Filtro HSV", 0, 255, nothing)
cv2.createTrackbar("Max S", "Filtro HSV", 255, 255, nothing)
cv2.createTrackbar("Min V", "Filtro HSV", 0, 255, nothing)
cv2.createTrackbar("Max V", "Filtro HSV", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    # Leer valores de las trackbars
    min_h = cv2.getTrackbarPos("Min H", "Filtro HSV")
    max_h = cv2.getTrackbarPos("Max H", "Filtro HSV")
    min_s = cv2.getTrackbarPos("Min S", "Filtro HSV")
    max_s = cv2.getTrackbarPos("Max S", "Filtro HSV")
    min_v = cv2.getTrackbarPos("Min V", "Filtro HSV")
    max_v = cv2.getTrackbarPos("Max V", "Filtro HSV")

    # Convertir a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Definir rangos y aplicar máscara
    lower = np.array([min_h, min_s, min_v])
    upper = np.array([max_h, max_s, max_v])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Mostrar resultado
    cv2.imshow("Filtro HSV", result)

    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

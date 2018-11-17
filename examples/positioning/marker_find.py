import numpy as np
import cv2
import cv2.aruco as aruco

from motor import Motor
import line_fit

motor = Motor()

CAMERA = 1
cap = cv2.VideoCapture(CAMERA)

inverse = lambda x,a,b: a+b/x
inverse = lambda x, a, b, c: a / x + b / x ** 4 + c

inv = lambda x, a, b, c: a / x + b / x ** 4 + c

Y = list(range(40, 360, 20))
Xa = [
    227, # 40
    150,
    121,
    98, # 100
    80.5,
    68.5,
    60.5,
    53.2,
    47.0, # 200
    43,
    39,
    35.5,
    32.5,
    29.2, # 300
    27.0,
    25.2,
]

Xb = [
    211, # 40
    160,
    121,
    100, # 100
    84.5,
    73.0,
    64,
    57.3,
    51.9, # 200
    47.7,
    44.0,
    41,
    37.5,
    35.5, # 300
    33,
    30.5,
]


fa = line_fit.function_fit(inv, Xa, Y)
fb = line_fit.function_fit(inv, Xb, Y)

heigth_to_dist = line_fit.function_fit(
    inverse,
    Y = list(range(30,331,5)),
    X = [
        317, 279, 248, 224, 203, 185, 172, 160, 150, 141, 133, 125, 118,
        112.5, 107, 102, 98.5, 94, 90.3, 87.1, 83.2, 80, 77.9, 75, 72.3,
        70.3, 68, 65.9, 64.4, 62.3, 60.5, 59.5,58.1,56,54.9,53.8, 52.4,
        51, 49.6, 48.2, 47.8, 46.1,
        45.3, 44.4, # 240 
        43.5, 42.7, 
        42., 41.3, 
        40.75, 40.17, 
        39.3, 38.8, 
        38.2, 37.5, 
        37.1, 36.1,
        35.4, 35.1,
        34.5, 34,
        33.4,
    ], 
)

dist_to_pwm = line_fit.interpolate(
    X = list(range(40,331,10)),
    Y = [
        77, 77, 77, 77.5, 78.5,78.9,79,79,79.5,79.5,81,82,82,82,83,83,
        83.5,84.7,86,86.3,87.7,88.5,89,90.5, 
        91.5, # 280
        96.5, 
        99.5, # 300
        100, 
        102.5, 
        106,
    ],
)

def draw_text(img, pos, text):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, text, pos, font, 1,  (255,255,255),2,cv2.LINE_AA)

def close():
    print("closing")
    cap.release()
    cv2.destroyAllWindows()
    motor.close()

heigth_list = []

while True:
    ret, frame = cap.read()
    if frame is None:
        break

    # frame = np.rot90(frame, -1)
    # frame = cv2.flip(frame, 2)
    frame = cv2.transpose(frame)  
    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        mapping = {corenr_ids[0]:corner for corenr_ids, corner in zip(ids, corners)}
        valid_markers = list(mapping[i] for i in [10,] if i in mapping)

        height = 0

        for marker in valid_markers:
            a,b,c,d = [corner for corner in marker[0]]
            vertical = sum(corner[1] for corner in marker[0])/4
            print(vertical)

            d1, d2 = a-d, b - c

            dist = lambda D: (D[0]**2 + D[1]**2)**0.5 
            d1, d2 = dist(d1),  dist(d2)
            height = (d1 + d2) / 2

            for corner in marker[0]:
                cv2.circle(frame, tuple(corner), 5, (129,0,0), 1)

            # average
            heigth_list.append(height)
            heigth_list = heigth_list[-30:]
            height = sum(heigth_list) / len(heigth_list)

            dist = heigth_to_dist(height)
            dist = (fb(height) + fa(vertical)) / 2
            # dist = fa(height)
            pwm = float(dist_to_pwm(dist))
            motor.servo(pwm)

            draw_text(frame, (0,50), "vertical: {:.1f}".format(vertical))
            draw_text(frame, (0,100), "height: {:.1f}".format(height))
            draw_text(frame, (0,150), "dist: {:.1f}".format(dist))
            draw_text(frame, (0,200), "factor: {:.1f}".format(pwm))

        gray = aruco.drawDetectedMarkers(frame, valid_markers)

    # Display the resulting frame
    res = cv2.resize(gray, None, fx=2, fy=2, interpolation = cv2.INTER_CUBIC)
    cv2.imshow('frame', res)
    
    try:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        break

close()
import json
from random import shuffle, randint
from typing import List, Tuple, NamedTuple

import matplotlib.pyplot as plt
import numpy as np
import cv2
import cv2.aruco as aruco

from os import listdir
from os.path import isfile, join, dirname, realpath

import line_fit

# type hints
FileList = List[str]
Marker = List[np.ndarray]


# DTO
class Frame(NamedTuple):
    x: float
    y: float
    distance: float
    markers: List[Marker]


def list_files(image_dir: str) -> FileList:
    dir_path = dirname(realpath(__file__))
    path = join(dir_path, image_dir)
    only_files = list(sorted([join(path, f), f] for f in listdir(path) if isfile(join(path, f))))

    shuffle(only_files)
    return only_files


# https://github.com/kyle-elsalhi/opencv-examples/blob/master/SimpleMarkerDetection/DetectMarkersAndPrint.py
def detect_aruco(file_path_list: FileList, valid_marker_ids: Tuple[int]) -> List[Frame]:
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    results = []
    for i, (full_path, file_name) in enumerate(file_path_list):
        if i % 100 == 0:  # print progress
            print(i)

        file_name = file_name.replace('.jpg', '').replace('.png', '')
        distance, x, y = map(float, file_name.split())

        # aruco wants gray
        gray = cv2.imread(full_path, cv2.IMREAD_GRAYSCALE)
        gray = np.rot90(gray)

        # _ are rejected points
        squares, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        valid_markers = []
        if ids is not None:
            # corner_ids[0] is aruco library specific
            valid_markers = [corners for corner_ids, corners in zip(ids, squares) if corner_ids[0] in valid_marker_ids]

        current_frame = Frame(x, y, distance, valid_markers)
        results.append(current_frame)

    return results


def vector_length(vector: Tuple[float]):
    return (vector[0] ** 2 + vector[1] ** 2) ** 0.5


def process(image_dir, cache=False, limit=100000):
    cache_name = '%s.txt' % image_dir
    if cache and isfile(cache_name):
        with open(cache_name, 'r') as file:
            frames_list = json.load(file)
            frames = []
            for e in frames_list:
                markers = e.pop('markers')
                markers = [np.asarray(m, ) for m in markers]
                frame = Frame(**e, markers=markers)
                frames.append(frame)
    else:
        only_files = list_files(image_dir)[:limit]
        frames: List[Frame] = detect_aruco(only_files, (10,))
        if cache:
            with open(cache_name, 'w') as outfile:
                flattened = []
                for f in frames:
                    f_dict = f._asdict()
                    f_dict['markers'] = [m.tolist() for m in f_dict['markers']]
                    flattened.append(f_dict)
                json.dump(flattened, outfile)

    result = []
    for frame in frames:
        valid_markers = frame.markers
        x, y, distance = frame.x, frame.y, frame.distance

        for marker in valid_markers:
            # marker[0] is aruco library specific
            a, b, c, d = marker[0]  # list of point, which are numpy arrays with 2 elem

            # vectors
            d_ac, d_bd = a - c, b - d
            h_ad, h_bc = a - d, b - c

            d_ac_len, d_bd_len = vector_length(d_ac), vector_length(d_bd)
            h_ad, h_bc = vector_length(h_ad), vector_length(h_bc)

            h = (h_ad + h_bc) / 2

            vertical = (a[1] + b[1] + c[1] + d[1]) / 4
            vertical_half = (a[1] + b[1]) / 2

            result.append(
                dict(
                    real_dist=distance,
                    # x=x,
                    # y=y,
                    diagonal_a=d_ac_len,
                    diagonal_b=d_bd_len,
                    height_a=h_ad,
                    height_b=h_bc,
                    diagonal_ab=(d_ac_len * d_bd_len) ** 0.5,
                    height=h,
                    vertical=vertical,
                    vertical_half=vertical_half,
                )
            )

    return result


image_dir = 'result5'
data_list = process(image_dir, cache=True)


def calculate(plt, data, key, function, initial_guess=None, maxfev=1000000):
    X = [d[key] for d in data]
    Y = [d['real_dist'] for d in data]

    plt.plot(X, Y, '.', markersize=2)

    fitted = line_fit.function_fit(function, X, Y, initial_guess=initial_guess, maxfev=maxfev, debug=True)

    xp = np.linspace((min(X) * 0.99), (max(X) * 1.01), 300)
    plt.plot(xp, fitted(xp), '-')

    plt.set_title(key)

    return Y, [fitted(x) for x in X]


full_plot, axarr = plt.subplots(2, 2)

# fit to this function
function = lambda x, a, b, c, d: a * (x ** 2 * b * x) / (x ** 2 + c * x + d)  # e = 0.15, max-e = 2.67, t = 0.16
function = lambda x, a, b, c, d: 1 / (a + b * x ** c)  # e = 0.12, max-e = 0.62, t = 1.747
function = lambda x, a, b, c, d: 1 / (a + b * x ** c) ** d  # e = 0.08, max-e = 0.47 t = 5.7
function = lambda x, a, b, c, d: x / (a + b * x) ** c + d  # e = 0.08, max-e = 0.443, t = 0.08
function = lambda x, a, b, c, d: a / (b + x) + c / (d + x)  # e = 0.08, max-e = 0.43, t = 0.04

function = lambda x, a, b, c, d, e: c * x / 1000 + a * d ** (x) + b
function = lambda x, a, b, c, d: ((b + x) * (d + x)) / (c * (b + x) + a * (d + x))

function = lambda x, a, b, c, d: (a + b * x) ** c + d  # 0.08, 0.44, 0.01
initial_guess = [3.1449758744968997E-01, -3.6898904768990819E-04, -9.8335463757939712E-01, -2.3892817796836106E+00]
Y, Ya = calculate(axarr[0, 0], data_list, 'vertical', function, initial_guess)

function = lambda x, a, b, c: a / x + b / x ** 4 + c  # e 0.28 e-max 2.58 t 0.02
function = lambda x, a, b, c: a * x + b / x + c  # e 0.27, max-e 2.53, t 0.04
# function = lambda x, a, b, c: a * x + c # 0.29, 2.48, 0.01
Y, Yb = calculate(axarr[0, 1], data_list, 'height', function)

axarr[1, 0].plot(Y, Ya, '.', markersize=2)
axarr[1, 0].plot(Y, Y, '.', markersize=1)
axarr[1, 1].plot(Y, Yb, '.', markersize=1)


example_point_keys = list(data_list[0].keys())
attribute_length = len(example_point_keys[0])
full_plot, axarr = plt.subplots(2, attribute_length // 2 + 1)

half = attribute_length // 2 + 1
for i, key in enumerate(example_point_keys):
    Y, Yx = calculate(axarr[i // half, i % half], data_list, key, function)

plt.show()

import numpy as np
import scipy.optimize
import scipy.interpolate

inverse = lambda x, a, b: a + b / x


def function_fit(func, X, Y):
    constants, _ = scipy.optimize.curve_fit(func, np.array(X), np.array(Y))
    f = lambda x: func(x, *constants)

    delta = 0
    for _x, _y in zip(X, Y):
        d = (f(_x) - _y)
        delta += abs(d)

    print(", ".join(str(c) for c in constants), "average delta:", delta / len(X), '\n')
    return f


def interpolate(X, Y, kind='slinear', backup=inverse):
    interpolate = scipy.interpolate.interp1d(X, Y, kind=kind)

    if backup:
        constants, _ = scipy.optimize.curve_fit(backup, np.array(X), np.array(Y))
        approximate = lambda x: backup(x, *constants)

        def dual(x):
            try:
                return interpolate(x)
            except:
                return approximate(x)

        return dual
    return interpolate


inv = lambda x, a, b, c: a / x + b / x ** 4 + c

# distance
Y = list(range(40, 360, 20))

# vertical pos 
Xa = [
    227,  # 40
    150,
    121,
    98,  # 100
    80.5,
    68.5,
    60.5,
    53.2,
    47.0,  # 200
    43,
    39,
    35.5,
    32.5,
    29.2,  # 300
    27.0,
    25.2,
]

# height values
Xb = [
    211,  # 40
    160,
    121,
    100,  # 100
    84.5,
    73.0,
    64,
    57.3,
    51.9,  # 200
    47.7,
    44.0,
    41,
    37.5,
    35.5,  # 300
    33,
    30.5,
]

# function that takes vertical pos and gives distance
vertical_to_dist = function_fit(inv, Xa, Y)
height_to_dist = function_fit(inv, Xb, Y)

goal_distance = [
    (420, 50),
    (178, 100),
    (112, 150),
    (80, 200),
    (62, 250),
    (50, 300),
    (42, 350),
]
gX = [e[0] for e in goal_distance]
gY = [e[1] for e in goal_distance]
ginv = lambda x, a, b, c: a / x + b / x ** 2 + c
# ginv = lambda x, a, b, c, d: ((b + (800 - x)) * (d + (800 - x))) / (c * (b + (800 - x)) + a * (d + (800 - x)))
print("goal_to_dist")
goal_to_dist = function_fit(ginv, gX, gY)

# 20 deg tuesday, low voltage
pwm_distance = [
    (56, 1),
    (57, 50),
    (58, 90),
    (59, 110),
    (60, 140),
    (61, 160),
    (62, 170),
    (63, 180),
    (64, 200),
    (65, 210),
    (66, 220),
    (67, 230),
    (69, 240),
    (70, 250),
    (71, 260),
    (72, 270),
    (73, 280),
    (78, 290),
    (82, 310),
    (90, 360),
]

# 12.5 deg, ~13,1v
pwm_distance = [
    # (50, 1),
    (50, 50),
    (51, 50),
    (52, 50),
    (53, 100),
    (54, 100),
    (57, 150),
    (58, 150),
    (62, 200),
    (63, 200),
    (67, 250),
    (68, 250),
    (76, 300),
    (77, 300),
    (78, 300),
]
throw_function = lambda x, a, b: a * (b + x / 100) ** 2 + 50

# 12.5 deg, ~13,1v
pwm_distance = [
    (56, 20),
    (57, 50),
    (57, 50),
    (58, 100),
    (59, 100),
    (61, 150),
    (62, 150),
    (64, 200),
    (65, 200),
    (66, 200),
    (71, 250),
    (72, 250),
    (73, 250),
    (77, 300),
    (78, 300),
    (79, 300),
]

# distance
X = [dist for pwm, dist in pwm_distance]
# pwm
Y = [pwm for pwm, dist in pwm_distance]

throw_function = lambda x, a, b, c: a * (b + x / 100) ** 2 + 50 + c
dist_to_pwm = function_fit(
    throw_function,
    X,
    Y,
)

dist_to_pwm_interpolated = interpolate(
    X,
    Y,
    backup=throw_function,
)

rpm_distance = [
    (5550, 60),
    (5600, 70),
    (5650, 80),
    (5750, 90),
    (5850, 100),
    (6000, 110),
    (6100, 120),
    (6300, 130),
    (6450, 140),
    (7350, 175),
    (7600, 190),
    (8750, 255),
    (9050, 280),
]

rpm_distance = [
    (5850, 80),
    # (6250, 106),
    (6675, 128),
    (7050, 151),
    # (7850, 171),
    (9050, 205),
    # (11000, 295),
]

# distance
X = [dist for rpm, dist in rpm_distance]
# rpm
Y = [rpm for rpm, dist in rpm_distance]

rpm_throw_function = lambda x, a, b, c, d: a * x + b * x ** 3 + d * x ** 4 + c
print("DIST to RPM")
dist_to_rpm = function_fit(
    rpm_throw_function,
    X,
    Y,
)

print(60, dist_to_rpm(60))
print(70, dist_to_rpm(70))
print(80, dist_to_rpm(80))
print(105, dist_to_rpm(105))
print(140, dist_to_rpm(140))
print(150, dist_to_rpm(150))
print(174, dist_to_rpm(174))
print(204, dist_to_rpm(204))

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    xp = np.linspace(round(min(X) - 30), round(max(X) + 30), 300)
    plt.plot(X, Y, '.', )
    plt.plot(xp, dist_to_rpm(xp), '-')
    # plt.plot(xp, dist_to_pwm_interpolated(xp), '--')
    plt.show()

    # xp = np.linspace(round(min(gX) - 30), round(max(gX) + 30), 300)
    # plt.plot(gX, gY, '.', )
    # plt.plot(xp, goal_to_dist(xp), '-')
    # plt.show()

    exit()

    # old examples

    Y = list(range(30, 331, 5))
    X = [
        317, 279, 248, 224, 203, 185, 172, 160, 150, 141, 133, 125, 118,
        112.5, 107, 102, 98.5, 94, 90.3, 87.1, 83.2, 80, 77.9, 75, 72.3,
        70.3, 68, 65.9, 64.4, 62.3, 60.5, 59.5, 58.1, 56, 54.9, 53.8, 52.4,
        51, 49.6, 48.2, 47.8, 46.1,
        45.3, 44.4,  # 240
        43.5, 42.7,
        42., 41.3,
        40.75, 40.17,
        39.3, 38.8,
        38.2, 37.5,
        37.1, 36.1,
        35.4, 35.1,
        34.5, 34,
        33.4,
    ]

    f = function_fit(inverse, X, Y)

    plt.plot(X, Y, '.', )
    xp = np.linspace(round(X[0]), round(X[-1]), 300)
    plt.plot(xp, f(xp), '-')
    plt.show()

    X = list(range(40, 331, 10))
    Y = [
        77, 77, 77, 77.5, 78.5, 78.9, 79, 79, 79.5, 79.5, 81, 82, 82, 82, 83, 83,
        83.5, 84.7, 86, 86.3, 87.7, 88.5, 89, 90.5,
        91.5,  # 280
        96.5,
        99.5,  # 300
        100,
        102.5,
        106,
    ]

    f = interpolate(X, Y, backup=None)

    plt.plot(X, Y, '.', )
    xp = np.linspace(round(X[0]), round(X[-1]), 300)
    plt.plot(xp, f(xp), '-')
    plt.show()

    inv = lambda x, a, b, c: b / x ** 4 + c

    Y = list(range(40, 360, 20))
    Xa = [
        227,  # 40
        150,
        121,
        98,  # 100
        80.5,
        68.5,
        60.5,
        53.2,
        47.0,  # 200
        43,
        39,
        35.5,
        32.5,
        29.2,  # 300
        27.0,
        25.2,
    ]

    Xb = [
        211,  # 40
        160,
        121,
        100,  # 100
        84.5,
        73.0,
        64,
        57.3,
        51.9,  # 200
        47.7,
        44.0,
        41,
        37.5,
        35.5,  # 300
        33,
        30.5,
    ]

    fa = function_fit(inv, Xa, Y)
    fb = function_fit(inv, Xb, Y)
    # fc = lambda x: (fa(x) + fb(x)) / 2

    xp = np.linspace(round(Xa[0]) - 20, round(Xa[-1]) + 20, 300)

    # for xa, xb in zip(Xa, Xb):
    #     ya, yb = 

    Ya = [fa(x) for x in Xa]
    Yb = [fb(x) for x in Xb]
    Yc = [(fa(xa) + fb(xb)) / 2 for xa, xb in zip(Xa, Xb)]

    print(sum(abs(y1 - y2) for y1, y2 in zip(Y, Yc)) / len(Y))

    plt.plot(Y, '-')
    plt.plot(Ya, '-')
    plt.plot(Yb, '-')
    plt.plot(Yc, '.')
    plt.show()

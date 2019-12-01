import numpy as np
import scipy.optimize
import scipy.interpolate

inverse = lambda x, a, b: a + b / x


def function_fit(func, X, Y):
    constants, _ = scipy.optimize.curve_fit(func, np.array(X), np.array(Y))
    f = lambda x: func(x, *constants)

    delta = 0
    squared_error = 0
    elem, d_max = 2 ** 20, 0
    for _x, _y in zip(X, Y):
        d = (f(_x) - _y)
        delta += abs(d)
        if abs(d) > d_max:
            d_max = abs(d)
            elem = _x, f(_x)
        squared_error = d ** 2

    print(", ".join(str(c) for c in constants), "\naverage delta:", delta / len(X), '\nsquared error', squared_error,'\n')
    print("max", d_max, elem)
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


rpm_distance = [ # min should be 4100
    (4775, 100),
    (5250, 150),
    (6050, 195),
    (6800, 250),
    (7450, 300),
    (8900, 400),
]

rpm_throw_function = lambda x, a, b, c, d, e: e + a ** x * b + c * x ** (1 + d)

XX = [dist for rpm, dist in rpm_distance]
YY = [rpm for rpm, dist in rpm_distance]

# rpm_throw_function = lambda x, a, b, c, d, e: a ** x * b + c * x ** (1 + d) + e
dist_to_rpm = function_fit(
    rpm_throw_function,
    XX,
    YY,
)
#
# class Distance:
#     constants = [0.2215416616277008, 0.8260044802493705, -20.362320905330222]
#     poly = lambda x, a, b, e: a * x ** b + e
#
#     @classmethod
#     def __call__(cls, x: float) -> float:
#         return cls.poly(x, *cls.constants)

for i in range(50, 400, 10):
    print(f"DIST {i} -> {dist_to_rpm(float(i)):.0f}")

goal_distance = [
    (442, 47),
    (334, 59),
    (296, 67),
    (272, 76),
    (226, 89),
    (204, 100),
    (190, 107),
    (178, 120),
    (166, 130),
    (154, 140),
    (142, 150),
    (132, 162),
    (126, 170),
    (118, 188),
    (114, 200),
    (108, 210),
    (104, 221),
    (96, 241),
    (84, 300),
    (80, 330),
    (72, 390),
]

gX = [e[0] for e in goal_distance]
gY = [e[1] for e in goal_distance]
ginv = lambda x, a, b, c: a / x + b / x ** 2 + c
goal_to_dist = function_fit(ginv, gX, gY)
for k, v in goal_distance:
    calc = round(goal_to_dist(k))

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    XX_SPACE = np.linspace(round(min(XX)) * .8, round(max(XX)) * 1.2, 100)
    plt.figure(figsize=(12, 12))
    plt.grid()
    plt.plot(XX_SPACE, [dist_to_rpm(x) for x in XX_SPACE], '-')

    plt.plot(XX, YY, 'o')

    plt.errorbar(XX, YY, [(dist_to_rpm(x) - y) * 2 for x, y in zip(XX, YY)])
    plt.show()

    for i in range(350, 500, 10):
        print(f"{i} -> {dist_to_rpm(i):.0f}")

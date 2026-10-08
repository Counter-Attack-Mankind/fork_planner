import xlrd
import numpy as np
from scipy.interpolate import CubicSpline
from matplotlib import pyplot as plt


def calc_station(xs, ys):
    ss = []
    dx = np.diff(xs, prepend=xs[0])
    dy = np.diff(ys, prepend=ys[0])
    cum = 0.0
    xs_real = []
    ys_real = []

    for i in range(len(dx)):
        distance = np.hypot(dx[i], dy[i])
        if distance == 0:
            continue
        xs_real.append(xs[i])
        ys_real.append(ys[i])
        cum += distance
        ss.append(cum)

    return ss, xs_real, ys_real


class FrenetFrame:
    def __init__(self, xs, ys):
        (self.s, xs, ys) = calc_station(xs, ys)
        self.sx = CubicSpline(self.s, xs)
        self.sy = CubicSpline(self.s, ys)

    def calc_yaw(self, s):
        return np.arctan2(self.sy(s, 1), self.sx(s, 1))

    def find_s(self, x, y, s0=0.0):
        """newton method to find optimze s"""
        opt = s0
        opt_last = opt
        for i in range(20):
            (ox, oy) = (self.sx(opt), self.sy(opt))
            (dx, dy) = (self.sx(opt, 1), self.sy(opt, 1))
            (ddx, ddy) = (self.sx(opt, 2), self.sy(opt, 2))
            (error_x, error_y) = (ox - x, oy - y)
            jac = 2.0 * error_x * dx + 2.0 * error_y * dy
            hessian = 2.0 * dx * dx + 2.0 * error_x * ddx + 2.0 * dy * dy + 2.0 * error_y * ddy

            opt -= jac / hessian

            if np.abs(opt - opt_last) < 1e-5:
                print("final %d" % i)
                return opt

            opt_last = opt

        return s0


if __name__ == '__main__':
    time_profile = []
    vehicles = []

    sheet = xlrd.open_workbook('./Data.xlsx').sheet_by_index(0)
    vehicle_count = int((sheet.ncols - 1) / 4)

    for i in range(1, sheet.nrows):
        time_profile.append(sheet.cell_value(i, 0))

    for v in range(vehicle_count):
        traj = {
            "x": [],
            "y": [],
            "yaw": [],
            "speed": [],
            "time": [],
        }

        for i in range(1, sheet.nrows):
            traj['x'].append(sheet.cell_value(i, v * 4 + 1))
            traj['y'].append(sheet.cell_value(i, v * 4 + 2))
            traj['yaw'].append(sheet.cell_value(i, v * 4 + 3))
            traj['speed'].append(sheet.cell_value(i, v * 4 + 4))

        vehicles.append(traj)

    frame = FrenetFrame(vehicles[0]['x'], vehicles[0]['y'])
    ts = np.linspace(0.0, frame.s[-1], 50)

    plt.axis('equal')
    plt.plot([frame.sx(t) for t in ts], [frame.sy(t) for t in ts])
    plt.show()


import numpy as np
import math
import bisect


class PathInterpolation:
    def __init__(self, path):
        self.path = path
        self.cum_len = np.array([])

    def calc_cum_len(self):
        dx = self.path[:, 1:-1] - self.path[:, 0:-2]
        end = self.path[:, -1] - self.path[:, -2]
        ends = np.array([[end[0]], [end[1]]])
        dx = np.append(dx, ends, axis=1)
        dl = np.sqrt(np.sum(dx * dx, axis=0))
        start = np.array([0])
        tem = np.insert(dl, 0, start, axis=0)
        res = np.cumsum(tem)
        return res

    def eval(self, l):
        if l < 0 or l > self.cum_len[-1]:
            print("l is out of range")
        I = bisect.bisect_right(self.cum_len, l)
        if I < len(self.path[1, :]):
            x = self.path[:, I - 1]
            y = self.path[:, I]
            lam = (l - self.cum_len[I - 1]) / (self.cum_len[I] - self.cum_len[I - 1])
            res = (1 - lam) * x + lam * y
        else:
            res = self.path[:, len(self.path) - 1]
        return res

    def path_normalize(self, n):
        self.cum_len = self.calc_cum_len()
        L = np.linspace(0, self.cum_len[-1], n)
        res = np.array([[], []])
        for i in range(len(L)):
            tmp = np.array([[self.eval(L[i])[0]], [self.eval(L[i])[1]]])
            res = np.append(res, tmp, axis=1)
        return res

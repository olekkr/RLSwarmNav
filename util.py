import numpy as np

class Box:
    # FORMAT: [First Corner, Offsets: [l,w,h]]
    def __init__(self, min_pt, size):
        self.min = np.array(min_pt)
        self.max = np.add(min_pt , size)

    def contains(self, p):
        return all(self.min[i] <= p[i] <= self.max[i] for i in range(3))

    def sample(self):
        return self.min + np.random.rand(3) * (self.max - self.min)

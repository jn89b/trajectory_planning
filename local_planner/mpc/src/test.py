#take in all numbers and add them
def sum(*args):
    total = 0
    for i in args:
        total += i
    return total

import numpy as np
import math as m
from matplotlib import pyplot as plt

theta = np.linspace(0, 2*np.pi, 1000)

r = 3* np.sin(8 * theta)

plt.polar(theta, r, 'r')

plt.show()


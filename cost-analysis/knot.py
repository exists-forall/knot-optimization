import numpy as np
import json

# Knot Object
class Knot:
    def __init__(self, angles, final_angle, cost):
        self.angles = angles
        self.angles.append(round(final_angle))
        self.cost = cost

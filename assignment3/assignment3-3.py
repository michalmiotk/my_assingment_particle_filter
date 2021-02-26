import matplotlib.pyplot as plt
import math
import numpy as np
import random as r


class Robot:
    def __init__(self, pos):
        self.pos = pos
        self.pole_dist = -100
        self.max_measurement = 3

    # Measurement is perfectly accurate even though we are assuming it isn't.
    def measure(self, poles):
        ### START STUDENT CODE
        # Set self.pole_dist to the distance to the closest pole.
        min_dist = None
        for pole in poles:
            dist = pole - self.pos
            if 0<dist< self.max_measurement:
                if min_dist is None or dist < min_dist:
                    min_dist = dist
        if min_dist is not None:
            self.pole_dist = min_dist
        else:
            self.pole_dist = -100
        ### END STUDENT CODE

class Particle(Robot):
    def __init__(self, pos):
        Robot.__init__(self, pos)



poles = [1, 10]
particle = Particle(0.1)
particle.measure(poles)
print("Output Should Be 0.9")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 11
particle.measure(poles)
print("Output Should Be -100")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 6.9
particle.measure(poles)
print("Output Should Be -100")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 7.1
particle.measure(poles)
print("Output Should Be 2.9")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 9.5
particle.measure(poles)
print("Output Should Be 0.5")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

from sim.a35 import plot

import random as r
import numpy as np


class Robot:
    def __init__(self, pos):
        self.pos = pos
        self.move_dist = 1


class Particle(Robot):
    def __init__(self, pos):
        Robot.__init__(self, pos)
        self.weight = 0
        self.movement_sigma = 1
        self.move_dist = 4  # Overwrite move distance for this example
        self.color = (0, 0, 1, 1)

    def predict(self):
        ### START STUDENT CODE
        self.pos += self.move_dist
        ### END STUDENT CODE

def resample_particles(particles):
    ### START STUDENT CODE
    # Please fill this array with the output of the r.choices function.
    resample = []
    for particle in particles:
        resample.append(particle.weight)
    # Please fill this array with resampled partciles.
    resampled_particles = []
    for _ in range(num_particles):
        resampled_particles += r.choices(particles, resample)
    ### END STUDENT CODE

    # Set all resampled particles to a different color.
    for i in range(len(resample)):
        resampled_particles[i] = Particle(resampled_particles[i].pos) #do it initialization instead deepcopy
        resampled_particles[-1].color = (0, 1, 0, 1)

    # Plot and return resampled particles.
    plot(particles, resampled_particles, resample, distance)
    return resampled_particles


num_particles = 10
distance = 40
particles = []

# Set first few particles to different weight and color.
for i in range(num_particles):
    particles += [Particle(r.uniform(0, distance))]
    if i < 3:
        particles[-1].weight = 1
        particles[-1].color = (1, 0, 0, 1)
    else:
        particles[-1].weight = 0.1

# Resample Particles
resampled_particles = resample_particles(particles)

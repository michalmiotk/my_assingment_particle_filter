import matplotlib.pyplot as plt
import numpy as np
import random as r
import math
from sim.plot import plot, print_particle_error


AUTORUN = False
robot_start = 7
num_particles = 3
distance = 40
poles = [10, 15, 17, 19, 30, 39]


### START STUDENT CODE
class Robot:
    def __init__(self, pos):
        self.pos = pos
        self.pole_dist = 0
        self.range = 3
    # Movement is perfectly accurate, even though we are assuming it isn't.
    def move(self):
        self.pos += 1
    # Measurement is perfectly accurate even though we are assuming it isn't.
    def measure(self, poles):
        new_dist = None
        for p in poles:
            dist = p - self.pos
            if 0<= dist < self.range:
                if new_dist is None or dist < new_dist:
                    new_dist = dist
        if new_dist is None:
            self.pole_dist = -100
        else:
            self.pole_dist = new_dist

class Particle(Robot):
    def __init__(self, pos, weight=1):
        super().__init__(pos)
        self.weight = weight
        self.var = 0.5

    def predict(self):
        self.move()
    def probability_density_function(self, mu, x):
        numerator = np.exp(-0.5*((x-mu)/self.var)**2)
        denominator = self.var*2*np.pi
        return numerator/denominator
    def update_weight(self, robot_dist):
        print(self.pole_dist, robot_dist)
        self.weight = self.probability_density_function(self.pole_dist, robot_dist)
        print(self.weight)
def resample_particles(particles):
    # Potentially resample uniformly if weights are so low.
    resampled =[]
    weights = [p.weight for p in particles]
    if sum(weights) < 0.05:
        initialize_particles(resampled)
    else:
        for i in range(len(particles)):
            resampled += r.choices(particles,weights)
            
        for i in range(len(particles)):
            resampled[i] = Particle(resampled[i].pos, resampled[i].weight)
    return resampled

def exact_uniform(particles):
    for x in range(num_particles):
        particles.append(Particle(x*distance/num_particles))

def random_init(particles):
    for i in range(num_particles):
        particles += [Particle(r.uniform(0, distance))]
        particles[-1].color = (1, 0, 0, 1)

def initialize_particles(particles):
    #random_init(particles)
    exact_uniform(particles)

### END STUDENT CODE

robot = Robot(robot_start)

# Setup particles.
particles = []
initialize_particles(particles)

# Plot starting distribution, no beliefs
plot(particles, poles, robot.pos)

# Begin Calculating
for j in range(39 - robot.pos):
    # Move
    if j != 0:
        robot.move()
        for particle in particles:
            particle.predict()

    # Measure
    robot.measure(poles)
    for particle in particles:
        particle.measure(poles)

        # Update Beliefs
        particle.update_weight(robot.pole_dist)

    print_particle_error(robot, particles)

    # Resample
    resampled_particles = resample_particles(particles)
    plot(particles, poles, robot.pos, resampled_particles, j, AUTORUN)
    particles = resampled_particles

plot(particles, poles, robot.pos, resampled_particles)

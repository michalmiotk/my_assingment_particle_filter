from sim.plot2d import plot
import random as r
import math
import numpy as np

class Position:
    def __init__(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.theta = pos[2]


class Pole(Position):
    def __init__(self, pos):
        Position.__init__(self, pos)


class Measurement:
    def __init__(self, distance, angle):
        self.distance = distance
        self.angle = angle


class Robot(Position):
    def __init__(self, pos):
        Position.__init__(self, pos)
        self.measurements = []
        self.max_measurement = 200

    # Movement is perfectly accurate, even though we are assuming it isn't.
    def move(self, speed, theta_dot):
        ### START STUDENT CODE
        self.theta += theta_dot
        self.x += speed*math.cos(self.theta)
        self.y += speed*math.sin(self.theta)
        ### END STUDENT CODE

    def move_with_error(self, speed, theta_dot):
        ### START STUDENT CODE
        speed_err = speed/10
        theta_dot_err = theta_dot/10
        self.move(speed+r.uniform(-speed_err, speed_err), theta_dot+r.uniform(-theta_dot_err,theta_dot_err))
        ### END STUDENT CODE

    # Measurement is perfectly accurate even though we are assuming it isn't.
    def measure(self, poles):
        ### START STUDENT CODE
        self.measurements = []
        for p in poles:
            dist_x = p.x - self.x
            dist_y = p.y - self.y
            dist = math.sqrt(dist_x**2+dist_y**2)
            if dist < self.max_measurement:
                if round(dist_x,4) == 0.0:
                    angle = math.pi/2
                else:    
                    angle = math.atan(dist_y/dist_x)

                angle -= self.theta
                '''
                if angle > math.pi:
                    angle -= math.pi
                elif angle <-math.pi:
                    angle += math.pi
                '''
                self.measurements.append(Measurement(dist, angle))
        ### END STUDENT CODE


class Particle(Robot):
    def __init__(self, pos, weight=1):
        Robot.__init__(self, pos)
        self.weight = weight
        self.distance_sigma = 5
        self.distance_weight = 1
        self.distance_distribution_peak = 1 / \
            (math.sqrt(2 * math.pi) * self.distance_sigma)
        self.angle_sigma = 0.5
        self.angle_distribution_peak = 1 / \
            (math.sqrt(2 * math.pi) * self.angle_sigma)
        self.angle_weight = 0.1
        self.theta_dot_sigma = 0.2
        self.speed_sigma = 0.5

    def predict(self, speed, theta_dot):
        ### START STUDENT CODE
        self.move(r.normalvariate(speed, self.speed_sigma), r.normalvariate(theta_dot, self.theta_dot_sigma))
        ### END STUDENT CODE

    def probability_density_function(self, mu, sigma, x):
        ### START STUDENT CODE
        numerator = np.exp(-0.5*((x-mu)/sigma)**2)
        denominator = sigma*2*np.pi
        return numerator/denominator
        ### END STUDENT CODE

    def update_weight(self, robot_measurements):
        ### START STUDENT CODE
        weights = []
        distance = math.sqrt(self.x**2+self.y**2)
        for m in robot_measurements:
            w = self.probability_density_function(distance-m.distance, self.distance_sigma, self.distance_distribution_peak)
            w += self.probability_density_function(self.theta-m.angle, self.angle_sigma, self.angle_distribution_peak)
            weights.append(w)
        self.weight = sum(weights)
        ### END STUDENT CODE


def resample_particles(particles):
    ### START STUDENT CODE
    resampled =[]
    weights = [p.weight for p in particles]
    scale = 10

    for i in range(len(particles)):
        resampled += r.choices(particles,weights)
    scale = len(resampled)/(5*sum([r.weight for r in resampled]))
    if scale>10:
        scale = 10
    for i in range(len(particles)):
        new_x = r.normalvariate(resampled[i].x, resampled[i].speed_sigma*scale)
        new_y = r.normalvariate(resampled[i].y, resampled[i].speed_sigma*scale)
        new_theta = r.normalvariate(resampled[i].theta, resampled[i].theta_dot_sigma*scale)
        resampled[i] = Particle([new_x, new_y, new_theta], resampled[i].weight)
    return resampled
    ### END STUDENT CODE

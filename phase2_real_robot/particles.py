#! /usr/bin/env python3
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
from copy import deepcopy


class Particle():
    def __init__(self, map, numbers_of_particles):
        self.map = map
        self.particle_number = numbers_of_particles
        self.particles_pose = np.zeros((numbers_of_particles, 3)) # x, y, theta
        self.particle_measurment = []
    

    def randomize(self):
        self.particles_pose[:, 0] = np.random.uniform(self.map.map_boundry[0], self.map.map_boundry[1], size=self.particle_number) + self.map.global_map_position[0]
        self.particles_pose[:, 1] = np.random.uniform(self.map.map_boundry[2], self.map.map_boundry[3], size=self.particle_number) + self.map.global_map_position[1]
        self.particles_pose[:, 2] = np.random.choice([-90, 90, 180, 0], size=self.particle_number)*np.pi/180.0
        for i in range(self.particle_number):
            while self.map.check_is_collition([self.particles_pose[i][0], self.particles_pose[i][1]]):
                self.particles_pose[i][0] = np.random.uniform(self.map.map_boundry[0], self.map.map_boundry[1]) + self.map.global_map_position[0]
                self.particles_pose[i][1] = np.random.uniform(self.map.map_boundry[2], self.map.map_boundry[3]) + self.map.global_map_position[1]

    
    def calculate_weight(self, sensor_measurment):
        ranges = [50, 100, 200, 300, 370, 400]
        means = [-11.056, -14.186, -20.846, -22.808, -17.824, 0.001]
        stds = [1.62871, 1.591039, 2.333727, 4.183197, 5.247954, 0.0001]

        i = np.argmin(np.abs(np.array(ranges) - sensor_measurment))
        mean, std = means[i], stds[i]

        self.weights = stats.norm(np.array(self.particle_measurment) + mean, std).pdf(sensor_measurment)
        for i in range(self.particle_number):
            if np.isnan(self.particle_measurment[i]):
                self.weights[i] = 0

    def calculate_particles_sensors(self):
        self.sensor_line = []
        self.particle_measurment = []
        for p in self.particles_pose:
            sensor_line = [[p[0], p[1]], [400*np.cos(p[2])+p[0], 400*np.sin(p[2])+p[1]]]
            
            min_distance = 10000
            collission_point = False
            for line in self.map.all_map_lines:

                intersection_point = self.map.find_intersection(line[0], line[1], sensor_line[0], sensor_line[1])
                if intersection_point != False:
                    distance = np.sqrt((p[0]-intersection_point[0])**2 + (p[1]-intersection_point[1])**2)
                    if min_distance >= distance:
                        min_distance = distance
                        collission_point = intersection_point

            if collission_point != False:
                particle_sensor_line = [[p[0] ,p[1]], collission_point]
            else:
                particle_sensor_line = sensor_line
                min_distance = 400

            self.particle_measurment.append(min_distance)
            self.sensor_line.append(particle_sensor_line)


    def plot_particle(self, color):
        for i, p in enumerate(self.particles_pose):
            if not self.map.out_of_range(p[0], p[1]):
                plt.plot(p[0], p[1], color=color, marker='o', alpha=0.5, markersize=4, zorder=1)
                sensor_line = self.sensor_line[i]
                plt.plot([sensor_line[0][0], sensor_line[1][0]], [sensor_line[0][1], sensor_line[1][1]], color=color, linestyle=':', alpha=0.5, zorder=1)


    def move_angular(self, delta_theta):
        linear_velocity = 50
        r = 176 / (2 * np.pi)
        o = linear_velocity / r
        u = (r / (48 * 2)) * 2 * o
        t = np.pi / (2 * u) 
        
        angular_velocity = delta_theta/t

        delta_thetas = [0, np.pi/2, -np.pi/2]
        angular_angular_mean = [0, 0.113429, 0.111114]
        angular_angular_std = [0, 0.045065, 0.037397]

        i = np.argmin(np.abs(np.array(delta_thetas) - delta_theta))
        aa_mean = angular_angular_mean[i]
        aa_std = angular_angular_std[i]

        w = angular_velocity #+ np.random.normal(aa_mean, aa_std)
        self.particles_pose[:, 2] += w * t


    def move_linear(self, delta_trans):
        linear_velocity = 0.05
        old = deepcopy(self.particles_pose)
        t = delta_trans/linear_velocity

        deta_transes = [0, 0.05, 0.1, 0.15]
        linear_linear_mean = [0, 0.0083, 0.004767, 0.003744]
        linear_linear_std = [0, 0.007067, 0.003799, 0.002204]

        i = np.argmin(np.abs(np.array(deta_transes) - delta_trans))
        ll_mean = linear_linear_mean[i]
        ll_std = linear_linear_std[i]

        v = linear_velocity + np.random.normal(ll_mean, ll_std)
        self.particles_pose[:, 0] += v*1000 * np.cos(self.particles_pose[:, 2]) * t
        self.particles_pose[:, 1] += v*1000 * np.sin(self.particles_pose[:, 2]) * t


        for i, p in enumerate(self.particles_pose):
            old_loc = [old[i][0], old[i][1]]
            if self.map.check_is_collition([p[0], p[1]]) or self.map.out_of_range(p[0], p[1]) or self.map.check_particle_path(old_loc, [p[0], p[1]]):
                self.particles_pose[i][0] = -10000
                self.sensor_line[i] = np.nan


    def resampling(self):
        # 40, 50, 10
        # 30, 35, 35
        # 30, 55, 15
        # sum_weights = np.sum(self.weights)
        # if sum_weights ==  0:
        #     probs = np.ones(self.particle_number)/self.particle_number
        # else:
        #     probs = self.weights/sum_weights
        # random_most_important_indices = np.random.choice(list(range(self.particle_number)), size=int(0.60*self.particle_number), p=probs, replace=True)
        # random_most_important = []
        # for i in random_most_important_indices:
        #     temp = deepcopy(self.particles_pose[i])
        #     temp[0] += random.uniform(-0.01, +0.01)
        #     temp[1] += random.uniform(-0.01, +0.01)
        #     random_most_important.append(temp)


        best_number = int(0.30*self.particle_number)
        best_random_number = int(0.55*self.particle_number)
        best_prtcls = [deepcopy(self.particles_pose[p]) for _, p in sorted(zip(self.weights, list(range(self.particle_number))), reverse=True)[:best_number]]
        best_prtcls = np.array(best_prtcls)

        random_most_important_indices = np.random.choice(best_prtcls.shape[0], size=best_random_number)
        random_most_important = np.zeros((best_random_number, 3))
        random_most_important[:, 0] = np.random.uniform(-10, 10, size=best_random_number)
        random_most_important[:, 1] = np.random.uniform(-10, 10, size=best_random_number)
        random_most_important += best_prtcls[random_most_important_indices]

        self.randomize()

        best = np.concatenate([best_prtcls, random_most_important])
        self.particles_pose[:len(best)] = best
        self.calculate_particles_sensors()


    def center_point(self):
        best_prtcls = np.array(deepcopy([self.particles_pose[p] for _, p in sorted(zip(self.weights, list(range(self.particle_number))), reverse=True)[:int(0.30*self.particle_number)]]))
        dist = np.zeros(len(best_prtcls))
        for i, particle in enumerate(best_prtcls):
            dist[i] = sum(np.sqrt((particle[0] - best_prtcls[:, 0])**2 + (particle[1] - best_prtcls[:, 1])**2))
        return best_prtcls[np.argmin(dist)]

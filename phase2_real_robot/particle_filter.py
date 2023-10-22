#! /usr/bin/env python3
import matplotlib.pyplot as plt
from robot import Robot
from particles import Particle
from map import Map
import numpy as np
import rospy


class ParticleFilter():
    def __init__(self, map, numbers_of_particles):
        self.map = map
        self.particles = Particle(map, numbers_of_particles)
        self.particles.randomize()
        self.particles.calculate_particles_sensors()
        self.particles_pose = self.particles.particles_pose
        self.number_of_particles = numbers_of_particles   
        self.robot = Robot()
        self.iter = 0
    

    def plot_current_state(self):
        plt.clf()
        self.map.plot_map()
        
        self.particles.plot_particle('blue')

        plt.draw()
        plt.pause(0.0000001)


    def step(self):
        
        delta_trans = 0
        delta_theta = 0

        self.iter += 1
        cmd = input(f'\nIteration {self.iter}:\nInsert the command : ')

        if 'www' in cmd:
            delta_trans = 150
        elif 'ww' in cmd:
            delta_trans = 100
        elif 'w' in cmd:
            delta_trans = 50
        if 'a' in cmd:
            delta_theta = 1
        elif 'd' in cmd:
            delta_theta = -1

        print('------- Moving Robot')
        self.robot.rotation(delta_theta)
        self.robot.translation(delta_trans)  

        print('------- Updating Particles')
        self.particles.move_angular(delta_theta*np.pi/2)
        self.particles.move_linear(delta_trans*0.001) 
        self.particles.calculate_particles_sensors()
        self.particles.calculate_weight(self.robot.laser)
        
        # Checking convergence conditions
        estimate = self.particles.center_point()
        around_estimate_x = self.particles_pose[np.where(abs(self.particles_pose[:, 0] - estimate[0]) <= 50)]
        around_estimate_y = around_estimate_x[np.where(abs(around_estimate_x[:, 1] - estimate[1]) <= 50)]
        print('------- Convergence =', '{:.2f}%'.format(len(around_estimate_y)*100/self.number_of_particles))
        if len(around_estimate_y) >= 0.80*self.number_of_particles:
            self.plot_current_state()
            print('******* Estimated Position =', estimate[0], estimate[1])
            cmd = input('Tap b to break or c to continue ')
            if cmd == 'b':
                return True
                

        print('------- Resampling')
        self.particles.resampling()
        

    def particle_filter_algo(self):
        while True:
            print('------- Ploting (wait...)')
            self.plot_current_state()
            if particle_filter.step():
                break


if __name__=="__main__":

    numbers_of_particles = int(input('number of particles? '))
    map = Map()
    particle_filter = ParticleFilter(map, numbers_of_particles)
    particle_filter.particle_filter_algo()

    rospy.spin()
    
  

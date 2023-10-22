#! /usr/bin/env python3
from copy import deepcopy
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
        plt.gca().invert_yaxis()
        self.map.plot_map()
        
        self.particles.plot_particle('blue')

        x = self.robot.x
        y = self.robot.y
        theta = self.robot.theta
        plt.arrow(y, x, 0.00001*np.sin(theta), 0.00001*np.cos(theta), color = 'red', head_width = 0.04, overhang = 0.6, zorder=5)
        plt.plot( [y, y+0.4*np.sin(theta)], [x, x+0.4*np.cos(theta)], color='red', zorder=5)
        plt.xlim(-0.1, 1.2)
        plt.ylim(0.7, -0.7)
        plt.draw()
        plt.pause(0.0000001)


    def step(self):
        
        delta_trans = 0
        delta_theta = 0

        self.iter += 1
        cmd = input(f'\nIteration {self.iter}:\nInsert the command : ')

        if 'www' in cmd:
            delta_trans = 0.20
        elif 'ww' in cmd:
            delta_trans = 0.10
        elif 'w' in cmd:
            delta_trans = 0.05
        if 'a' in cmd:
            delta_theta = np.pi/2
        elif 'd' in cmd:
            delta_theta = -np.pi/2

        print('------- Moving Robot')
        self.robot.rotation(delta_theta)
        self.robot.translation(delta_trans)  

        print('------- Updating Particles')
        self.particles.move_angular(delta_theta, self.robot.angular_speed)
        self.particles.move_linear(delta_trans, self.robot.linear_speed) 
        self.particles.calculate_particles_sensors()
        self.particles.calculate_weight(self.robot.laser)
        
        # Checking convergence conditions
        estimate = self.particles.center_point()
        around_estimate_x = self.particles_pose[np.where(abs(self.particles_pose[:, 0] - estimate[0]) <= 0.05)]
        around_estimate_y = around_estimate_x[np.where(abs(around_estimate_x[:, 1] - estimate[1]) <= 0.05)]
        print('------- Convergence =', '{:.2f}%'.format(len(around_estimate_y)*100/self.number_of_particles))
        if len(around_estimate_y) >= 0.80*self.number_of_particles:
            self.plot_current_state()
            print('******* Estimated Position =', estimate[0], estimate[1])
            print('******* Real Position =', self.robot.x, self.robot.y, '\n')
            cmd = input('Tap b to break or c to continue ')
            if cmd == 'b':
                return True

        print('------- Resampling')
        self.particles.resampling()
        

    def particle_filter_algo(self):
        while not rospy.is_shutdown():
            print('------- Ploting (wait...)')
            self.plot_current_state()
            if particle_filter.step():
                break


if __name__=="__main__":

    world = input('which world(1:5)? ')
    numbers_of_particles = int(input('number of particles? '))
    map_address = f'/home/mahyar/catkin_ws/src/anki_description/world/sample{world}.world'
    map = Map(map_address)
    particle_filter = ParticleFilter(map, numbers_of_particles)
    particle_filter.particle_filter_algo()

    rospy.spin()
    
  

#! /usr/bin/env python3
from shapely.geometry import LineString,Point
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon
import numpy as np



class Map():
    def __init__(self, map_address):
        rects = []
        centers = []
        tree = ET.parse(map_address)
        for links in tree.iter('model'):
            if 'vector_world' in links.attrib['name']:
                for link in links:
                    if link.tag == 'pose':
                        global_map_pose = link.text.split(' ')

                    if link.tag == 'link':
                        if link.find('collision') != None:
                            for _pose in link:
                                if _pose.tag == 'pose':
                                    pose = _pose.text.split(' ')

                            for collision in link.iter('collision'):
                                    geometry = collision[3][0][0].text.split(' ')

                            p1 = [float(pose[0]) + (float(geometry[0]) * np.cos(float(pose[5])) / 2)
                                    + float(geometry[1]) * np.sin(float(pose[5])) / 2 ,
                                    float(pose[1]) + float(geometry[0]) * np.sin(float(pose[5])) / 2
                                        - float(geometry[1]) * np.cos(float(pose[5])) / 2 ]

                            p2 = [float(pose[0]) + float(geometry[0]) * np.cos(float(pose[5])) / 2
                                -float(geometry[1])*np.sin(float(pose[5]))/2 ,
                                    float(pose[1]) + float(geometry[0])*np.sin(float(pose[5]))/2
                                        +float(geometry[1])*np.cos(float(pose[5]))/2]

                            p3 = [float(pose[0]) - float(geometry[0])*np.cos(float(pose[5]))/2
                                +float(geometry[1])*np.sin(float(pose[5]))/2 ,
                                    float(pose[1]) - float(geometry[0])*np.sin(float(pose[5]))/2
                                        -float(geometry[1])*np.cos(float(pose[5]))/2]

                            p4 = [float(pose[0]) - float(geometry[0])*np.cos(float(pose[5]))/2
                                -float(geometry[1])*np.sin(float(pose[5]))/2 ,
                                    float(pose[1]) - float(geometry[0])*np.sin(float(pose[5]))/2
                                        +float(geometry[1])*np.cos(float(pose[5]))/2]

                            rects.append([p1 ,p2 ,p3 ,p4])
                            centers.append([pose[0],pose[1]])


        self.global_map_position = [float(global_map_pose[0]), float(global_map_pose[1])]
        self.rects = self.add_offset(rects)
        self.map_boundry = self.map_boundry_func(centers)
        self.all_map_lines = self.convert_point_to_line()
        self.polygan = self.convert_to_poly()


    def convert_point_to_line(self):
        lines = []
        for points in self.rects:
            lines.append([ points[0] , points[1]] )
            lines.append([ points[1] , points[3]] )
            lines.append([ points[3] , points[2]] )
            lines.append([ points[2] , points[0]] )
        return lines


    def add_offset(self, rects):
        offset = self.global_map_position
        new_rects = []
        for points in rects: 
            new_rects.append(
                [  
                    [points[0][0] + offset[0] , points[0][1] + offset[1]  ] ,
                    [points[1][0] + offset[0] , points[1][1] + offset[1]  ] ,
                    [points[2][0] + offset[0] , points[2][1] + offset[1]  ] ,
                    [points[3][0] + offset[0] , points[3][1] + offset[1]  ] 
                ]
            )
        return new_rects


    def convert_to_poly(self):
        polygons = []
        for points in self.rects:
            polygons.append(Polygon(
                [tuple(points[0]),
                 tuple(points[1]),
                 tuple(points[3]),
                 tuple(points[2])
                 ]))
        return polygons


    def map_boundry_func(self, centers):
        X = []
        Y = [] 

        for item in centers:
            X.append(float(item[0]))
            Y.append(float(item[1]))

        return min(X), max(X), min(Y), max(Y)
        

    def plot_map(self):
        
        # Plot each rectangle and fill inside
        for rect_edges in self.rects:
            X = [rect_edges[0][0], rect_edges[1][0], rect_edges[3][0], rect_edges[2][0]]
            Y = [rect_edges[0][1], rect_edges[1][1], rect_edges[3][1], rect_edges[2][1]]
            plt.fill(Y, X, c='gray', zorder=10)
            

    def find_intersection(self, p1, p2, p3, p4):

        line1 = LineString([tuple(p1), tuple(p2)])
        line2 = LineString([tuple(p3), tuple(p4)])
        if line1.is_valid and line2.is_valid:
            int_pt = line1.intersection(line2)
            if int_pt:
                point_of_intersection = int_pt.x, int_pt.y
                return point_of_intersection
            else:
                return False
        else:
            return False


    def check_is_collition(self, point):
        p = Point(tuple(point))
        for rect in self.polygan:
            if rect.contains(p):
                return True
        return False
    

    def out_of_range(self, x, y):
        if x - self.global_map_position[0] > self.map_boundry[1] or x - self.global_map_position[0] < self.map_boundry[0]:
            return True
        elif y - self.global_map_position[1] > self.map_boundry[3] or y - self.global_map_position[1] < self.map_boundry[2]:
            return True
        else:
            return False
        

    def check_particle_path(self, path_start, path_end):
        flag = False
        for line in self.all_map_lines:
            flag = self.find_intersection(line[0], line[1] , path_start, path_end)
            if flag != False:
                return True

        return False
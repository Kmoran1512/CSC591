#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2

class CollisionDetection(object) :
    counter = 0
    radar_arr = []

    def __init__(self, role_name = 'hero'):
        self.role_name = role_name
        self.object_in_front = False

    def detectCollision(self, control):

        if self.object_in_front :
            print('in in front')
            deadStop(control)

        return control
    
    def isObjectInFront(self, info):
        gen = pc2.read_points(info, skip_nans=True)

        for p in gen:
            self.radar_arr.append(p[0])
        
        if self.counter == 0 :
            self.radar_arr.sort()
            
            self.object_in_front = self.radar_arr[0] < 10

            self.radar_arr = []

        self.counter = (self.counter + 1) % 2

def deadStop(control):
    control.throttle = 0.0
    control.brake = 1.0
    control.steer = 0.0
    
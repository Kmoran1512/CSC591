#!/usr/bin/env python

from collision_detection import deadStop


class AssistedCruiseControl(object) :
    MAX_ACCELERATION = 17
    
    def __init__(self, role_name = 'hero'):
        self.role_name = role_name
        self.velocity = 0.0

    def setThrottle(self, target, control):
        control.throttle = self.setAcceleration(target)

        return control
    
    def setVelocity(self, info):
        self.velocity = info.velocity

    def setAcceleration(self, targetVelocity) :
        dampen = 2
        if self.velocity < targetVelocity :
            goalAccel = (targetVelocity - self.velocity) / dampen
            return min(goalAccel, AssistedCruiseControl.MAX_ACCELERATION) / AssistedCruiseControl.MAX_ACCELERATION
        else :
            print('Too fast!')
            return 0

#!/usr/bin/env python

import sys
import time

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo, CarlaEgoVehicleStatus
from sensor_msgs.msg import PointCloud2

from assisted_cruise_control import AssistedCruiseControl as Acc
from collision_detection import CollisionDetection, deadStop

class TwistToVehicleControl(object):
    programTime = 15
    targetVelocity = 10

    def __init__(self, role_name):
        self.role_name = role_name

        maxSteeringAngle = self.pollVehicleInfo()

        self.pub = rospy.Publisher("/carla/{}/vehicle_control_cmd".format(role_name),
                                   CarlaEgoVehicleControl)
        
        start_time = time.time()

        cruiseControl = Acc(role_name)
        detector = CollisionDetection(role_name)


        rospy.Subscriber(name="/carla/{}/vehicle_status".format(self.role_name), data_class=CarlaEgoVehicleStatus, callback=cruiseControl.setVelocity)
        rospy.Subscriber(name="/carla/{}/radar/default/radar_points".format(self.role_name), data_class=PointCloud2, callback=detector.isObjectInFront)

        controller = CarlaEgoVehicleControl()

        while (time.time() - start_time) < self.programTime:
            # Stay in Lane

            # Collision Detection
            detector.detectCollision(controller)

            # Assisted Cruise Control
            if (not detector.object_in_front):
                cruiseControl.setThrottle(self.targetVelocity, controller)
            else:
                deadStop(controller)

            # Publish
            self.publishControl(controller)

        deadStop(controller)
        self.publishControl(controller)

        exit()

    def publishControl(self, control):
        try:
            self.pub.publish(control)
        except rospy.ROSException as e:
            if not rospy.is_shutdown():
                rospy.logwarn("Error while publishing control: {}".format(e))

    def pollVehicleInfo(self):
        rospy.loginfo("Wait for vehicle info...")
        try:
            vehicle_info = rospy.wait_for_message("/carla/{}/vehicle_info".format(self.role_name),
                                                  CarlaEgoVehicleInfo)
        except rospy.ROSInterruptException as e:
            if not rospy.is_shutdown():
                raise e
            else:
                sys.exit(0)
        if not vehicle_info.wheels:  # pylint: disable=no-member
            rospy.logerr("Cannot determine max steering angle: Vehicle has no wheels.")
            sys.exit(1)

        max_steering_angle = vehicle_info.wheels[0].max_steer_angle  # pylint: disable=no-member
        if not max_steering_angle:
            rospy.logerr("Cannot determine max steering angle: Value is %s",
                         max_steering_angle)
            sys.exit(1)
        rospy.loginfo("Vehicle info received. Max steering angle=%s",
                      max_steering_angle)
        
        return float(max_steering_angle)


def main():
    rospy.init_node('convert_twist_to_vehicle_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    twist_to_vehicle_control = TwistToVehicleControl(role_name)
    try:
        rospy.spin()
    finally:
        del twist_to_vehicle_control
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()

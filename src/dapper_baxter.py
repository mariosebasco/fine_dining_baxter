#!/usr/bin/env python
import sys
import rospy
import baxter_interface
from example.msg import ArmMovement

def calibrate():
    zval = -1
    rospy.set_param('/ZOFF',-1)
    while zval == -1 and not rospy.is_shutdown():
        zval = rospy.get_param('/ZOFF')
    return zval

def move_to_pos(pub,data):
    rospy.set_param("moved",-1)
    pub.publish(data)
    while rospy.get_param("moved")!=1:
        pass
    rospy.set_param("moved",-1)

def main():
    pub = rospy.Publisher("/move_dat_arm",ArmMovement, queue_size=10)
    rospy.init_node("dapper_baxter")
    rate = rospy.Rate(10)
    baxter_interface.RobotEnable().enable()
    gripper = baxter_interface.Gripper('left')
    gripper.open()
    rospy.sleep(1)
    rospy.set_param("/try_again",-1)
    x_home = 0.7
    y_home = 0.2
    z_home = 0.2
    while not rospy.is_shutdown():
        data = ArmMovement()
        data.x = x_home
        data.y = y_home
        data.z = z_home
        data.arm = 'left'
        move_to_pos(pub,data)
        z_off = calibrate()
        rospy.loginfo("CALIBRATED")
        rospy.set_param("zoff",z_off)   #
        data.x = 0.82#0.82143
        data.y = 0.23#0.42813
        data.z = z_home-z_off
        data.arm = 'left'
        move_to_pos(pub,data)
        gripper.calibrate()
        gripper.close()

if __name__ == "__main__":
    sys.exit(main())

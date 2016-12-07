#!/usr/bin/env python

import argparse
import sys
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import baxter_interface
from baxter_final.msg import ArmMovement
from baxter_final.srv import get_ik

#x_prev = 0.0
#y_prev = 0.0
#z_prev = 0.0
#limb_prev = ''

def listener():
    rospy.Service('/handle_ik',get_ik,ik_test)
    rospy.spin()
    #just called the servce
#    subber = rospy.Subscriber("/move_dat_arm",ArmMovement,ik_test)

def ik_test(data):
#    global x_prev
#    global y_prev
#    global z_prev
#    global limb_prev
    x_pos = data.x
    y_pos = data.y
    z_pos = data.z
    limb = data.limb
    print x_pos,y_pos,z_pos
    
#    if not (x_pos != x_prev or y_pos != y_prev or z_pos != z_prev or limb != limb_prev):
#        return 0
    
#    x_prev = x_pos
#    y_prev = y_pos
#    z_prev = z_pos
#    limb_prev = limb
    
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x_pos,
                    y=y_pos,
                    z=z_pos,
                ),
                orientation=Quaternion(
                    x=1.00,
                    y=0.00,
                    z=0.00,
                    w=0.00,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x_pos,
                    y=y_pos,
                    z=z_pos,
                ),
                orientation=Quaternion(
                    x=1.00,
                    y=0.00,
                    z=0.00,
                    w=0.00,
                ),
            ),
        ),
    }
    ikreq.pose_stamp.append(poses[limb])    

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    whichlimb = baxter_interface.Limb(limb)
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print limb_joints
        whichlimb.move_to_joint_positions(limb_joints)
#       rospy.set_param("/should_continue",0)
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")
        rospy.set_param("/try_again",1)
#        rospy.set_param("/should_continue",0)
        whichlimb.move_to_neutral()
#        rospy.set_param("moved",1)
    return 0

def main():
    rospy.init_node("ik_node")
#    rospy.set_param("/should_continue",0)    
    while not rospy.is_shutdown():
 #       if rospy.get_param("/should_continue") == 1:
        listener()
        

if __name__ == "__main__":
    main()

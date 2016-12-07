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


def ik_test(limb):
    rospy.init_node("mover_node")

    baxter_interface.RobotEnable().enable()
    rospy.sleep(1)

    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.8203,
                    y=0.2255,
                    z=-0.1,
                ),
                orientation=Quaternion(
                    x=1.0,
                    y=0.0,
                    z=0.0,
                    w=0.0,

                   # x=-0.3,
                   # y=0.8,
                   # z=0.108155782462,
                   # w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0,
                    y=0,
                    z=0,
                    w=1,
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

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print limb_joints
        whichlimb = baxter_interface.Limb(limb)
        whichlimb.move_to_joint_positions(limb_joints)
        #left_gripper = baxter_interface.Gripper('left',baxter_interface.CHECK_VERSION)
        #left_gripper.calibrate()
        #left_gripper.open()
        #cus_head=baxter_interface.Head()
        #cus_head.set_pan(0.0)
        
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        whichlimb = baxter_interface.Limb(limb)
        #zero = {'left_s0': 0, 'left_s1': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0}
        whichlimb.move_to_neutral()
 
    return 0





def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    return ik_test(args.limb)
    

if __name__ == "__main__":
    sys.exit(main())

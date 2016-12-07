#!/usr/bin/env python
import sys
import rospy
import baxter_interface
from baxter_final.srv import get_ik
from baxter_final.srv import image_proc
#from baxter_final.srv import ArmMovement
from baxter_core_msgs.msg import EndpointState


def main():
    
    #Subscribe to the node that gives xb and yb


    rospy.init_node("dapper_baxter")
    baxter_interface.RobotEnable().enable()
    rospy.sleep(1)
    rospy.set_param("/try_again",0)


    #Let's set up the services for the ik node and the image processing node
    rospy.wait_for_service('/handle_ik')
    my_ik_serv = rospy.ServiceProxy('/handle_ik',get_ik)

    rospy.wait_for_service('/handle_image_proc')
    my_color_serv = rospy.ServiceProxy('/handle_image_proc',image_proc)

    left_gripper = baxter_interface.Gripper('left')
    right_gripper = baxter_interface.Gripper('right')

    home_x = 0.7
    home_y = -0.5
    home_z = 0.15
    init_down = 0.25


    #STEP 1:  go to the home position to start everything
    init_lift = rospy.set_param('/init_lift',init_down)
#   z_ground = rospy.get_param('/Bpz')
    x = home_x
    y = home_y
    z = home_z 
    limb = 'right'
    my_ik_serv(x,y,z,limb)
    rospy.sleep(5)
    if rospy.get_param("/try_again") == 1:
        response = my_service(x,y,z,limb)
        rospy.sleep(5)
        rospy.set_param("/try_again",0)


    #STEP 2 Call image proc service to obtain position of the first object
    color = 'red'
    my_color_serv(color)
    #get xb and yb
    xb = rospy.get_parameter('/x_baxter')
    yb = rospy.get_parameter('/y_baxter')
    rospy.sleep(2)

    #sTEP 3: move to point given by image proc
    x = xb
    y = yb
    z = home_z - init_down
    arm = 'right'
    my_service(x,y,z,arm)
    rospy.sleep(5)
    if rospy.get_param("/try_again") == 1:
        my_service(x,y,z,limb)
        rospy.sleep(5)
        rospy.set_param("/try_again",0)
    left_gripper.calibrate()
    left_gripper.close()
'''
    #STEP 4: move up by init_lift
    x = xb
    y = yb
    z = z_ground + init_lift
    arm = 'left'
    my_service(x,y,z,arm)
    rospy.sleep(5)
    if rospy.get_param("/try_again") == 1:
        my_service(x,y,z,limb)
        rospy.sleep(5)
        rospy.set_param("/try_again",0)

    #STEP 5: Move to position next to plate
    x = plate_pos_x + x1_offset
    y = plate_pos_y
    z = z_ground + init_lift
    arm = 'left'
    my_service(x,y,z,arm)
    rospy.sleep(5)
    if rospy.get_param("/try_again") == 1:
        my_service(x,y,z,limb)
        rospy.sleep(5)
        rospy.set_param("/try_again",0)

    #STEP 6: Set down on table
    x = plate_pos_x + x1_offset
    y = plate_pos_y
    z = z_ground - stand_height
    arm = 'left'
    my_service(x,y,z,arm)
    rospy.sleep(5)
    if rospy.get_param("/try_again") == 1:
        my_service(x,y,z,limb)
        rospy.sleep(5)
        rospy.set_param("/try_again",0)

    #STEP 7: Go back up
    x = plate_pos_x + x1_offset
    y = plate_pos_y
    z = z_ground + init_lift
    arm = 'left'
    my_service(x,y,z,arm)
    rospy.sleep(5)
    if rospy.get_param("/try_again") == 1:
        my_service(x,y,z,limb)
        rospy.sleep(5)
        rospy.set_param("/try_again",0)

    #STEP 8: Go back to home
    x = home_x
    y = home_y
    z = z_ground + init_lift
    arm = 'left'
    my_service(x,y,z,arm)
    rospy.sleep(5)
    if rospy.get_param("/try_again") == 1:
        my_service(x,y,z,limb)
        rospy.sleep(5)
        rospy.set_param("/try_again",0)

    #STEP 9: call image proc for blue
    color = 'blue'
    my_color_serv(color)
    #get xb and yb
    xb = rospy.get_parameter('/x_baxter')
    yb = rospy.get_parameter('/y_baxter')
    rospy.sleep(2)
    
'''
    


    



if __name__ == "__main__":
    main()

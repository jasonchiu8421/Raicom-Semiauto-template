#!/usr/bin/env python3
# -*- coding: utf-8 -*
 
import  os
import  sys
import  tty, termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
 
# å¨å±€åé
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)
arm_stop_pub = rospy.Publisher('/arm_stop', String, queue_size=1)
interrupt_pub = rospy.Publisher('/interrupt', String, queue_size=1)
global height

# global can_grasp
# global can_release

# def grasp_status_cp(msg):
#     global can_release,can_grasp
#     # ç©ä½æåæå,è®©æºå¨äººåèµ·å§ç¹
#     if msg.data=='1':
#         can_release=True
#     if msg.data=='0' or msg.data=='-1':
#         can_grasp=True
# grasp_status=rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=1)

def keyboardLoop():
    rospy.init_node('teleop')
    #åå§åçå¬é®çæé®æ¶é´é´é
    rate = rospy.Rate(rospy.get_param('~hz', 30))
 
    #é€åº¦åé
    # æ¢é€
    walk_vel_ = rospy.get_param('walk_vel', 0.7)
    # å¿«é€
    run_vel_ = rospy.get_param('run_vel', 1.7)
    yaw_rate_ = rospy.get_param('yaw_rate', 1.4)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 1.4)
    # walk_vel_ååé€åº¦
    max_tv = walk_vel_
    # yaw_rate_æè½¬é€åº¦
    max_rv = yaw_rate_
    # åæ°åå§å
    speed=0
    # global can_release,can_grasp
    # can_grasp=True
    # can_release=False
    
    print ("""
    w: forward  
    s: backward 
    a: turn left 
    d: turn right 
    shift: hold to speed up the robot
    Grabbing actions:
    j: bottom layer
    k: second layer
    i: drop the block
    u: grab the block on the floor
    9: press to reset the robotic arm if mistakes were made and the click sound occured""")
  
    #è¯»åæé®å¾ªç¯
    while not rospy.is_shutdown():
        # linuxä¸è¯»åé®çæé®
        fd = sys.stdin.fileno()
        turn =0
        old_settings = termios.tcgetattr(fd)
		#ä¸äº§çåæ¾ææ
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # chä»£è¡¨è·åçé®çæé®
        if ch == '9': # home        ### reset is not in grasp
            # if can_release:
            msg=String()
            msg.data='reset'
            grasp_pub.publish(msg)
            # can_release=False
            speed = 0
            turn = 0
        elif ch == 'b':
            msg = String()
            msg.data = 'stop'
            arm_stop_pub.publish(msg)
        elif ch == 'j':
            # if can_grasp:
            msg=String()
            msg.data='go_to_level_1'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'k':
            # if can_grasp:
            msg=String()
            msg.data='go_to_level_2'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'l':
            # if can_grasp:
            msg=String()
            msg.data='go_to_level_3'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'i':
            # if can_grasp:
            msg=String()
            msg.data='release'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'u':
            # if can_grasp:
            msg=String()
            msg.data='grab'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'w':
            max_tv = walk_vel_
            speed = 0.2
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -0.2
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 0.1
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -0.1
        elif ch == 'W':
            max_tv = run_vel_
            speed = 1.4
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -1.4
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 1
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -1
        #elif ch == 'q':
           # exit()
        #elif ch == 'b':
         #   msg = String()
          #  msg.data = 'stop'
           # interrupt_pub.publish(msg)
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0

        #åé€æ¶æ¯
        cmd.linear.x = speed * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        rate.sleep()
		#åæ­¢æºå¨äºº
        #stop_robot()
 
def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)
 
if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import random
import rospy
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from swiftpro.msg import *  # Use swiftpro message types

class UArmGraspObject():
    
    def __init__(self):
        '''
        UArm-specific initialization matching uarm.cpp
        '''
        # Instance variables instead of global variables
        self.abcda = 0
        self.abcdb = 0
        self.abcdc = 0
        
        # Arm position tracking
        self.arm_x = 0.0
        self.arm_y = 0.0
        self.arm_z = 0.0
        self.arm_status = 0
        
        # Image center point (assuming 640x480 resolution)
        self.center_x = 320
        self.center_y = 240

        self.interrupt = False

        # Get calibration file data
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]
        rospy.logwarn('X axis k and b value: ' + str(self.x_kb))
        rospy.logwarn('Y axis k and b value: ' + str(self.y_kb))

        # Publishers - EXACTLY matching uarm.cpp
        self.pub_position = rospy.Publisher('position_write_topic', position, queue_size=10)
        self.pub_status = rospy.Publisher('swiftpro_status_topic', status, queue_size=1)
        self.pub_pump = rospy.Publisher('pump_topic', status, queue_size=1)
        self.pub_gripper = rospy.Publisher('gripper_topic', status, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub2 = rospy.Subscriber('/grasp', String, self.grasp_cp, queue_size=1)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1)
        
        # Subscribe to UArm state - EXACTLY matching uarm.cpp
        self.sub_arm_state = rospy.Subscriber('SwiftproState_topic', SwiftproState, self.arm_state_callback, queue_size=1)
        # Allow external stop/interrupt
        rospy.Subscriber('/arm_stop', String, self.interrupt_cb, queue_size=1)
        
        # Initialize arm position
        self.arm_position_reset()
        pos = position()
        pos.x = 20
        pos.y = 150
        pos.z = 35
        self.pub_position.publish(pos)

    def interrupt_cb(self, msg):
        if msg.data == 'stop':
            rospy.logwarn("interrupt received!")
            self.interrupt = True

    def image_cb(self, data):
        """Image callback function (no longer detecting water source, only showing center point)"""
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('CvBridge Error:', e)
            return
            
        # Draw center point
        cv2.circle(cv_image1, (self.center_x, self.center_y), 5, (0, 0, 255), -1)
        
        # Add arm position display
        arm_pos_text = f"UArm: ({self.arm_x:.0f},{self.arm_y:.0f},{self.arm_z:.0f})"
        cv2.putText(cv_image1, arm_pos_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.imshow("center_point", cv_image1)
        cv2.waitKey(1)

    def arm_state_callback(self, msg):
        """Callback for UArm SwiftproState updates - EXACTLY matching uarm.cpp"""
        self.arm_x = msg.x
        self.arm_y = msg.y
        self.arm_z = msg.z
        self.arm_status = msg.swiftpro_status
        rospy.loginfo(f"Current UArm State: X={self.arm_x:.2f}, Y={self.arm_y:.2f}, Z={self.arm_z:.2f}, Status={self.arm_status}")

    def get_arm_position(self):
        """Get current arm position"""
        return {
            'x': self.arm_x,
            'y': self.arm_y,
            'z': self.arm_z,
            'status': self.arm_status
        }

    def grasp_cp(self, msg):
        rate = rospy.Rate(1)
        if msg.data == 'go_to_level_1':
            # Grasp at image center with height -50
            print("grasping at image center!")
            self.grasp(self.center_x, self.center_y, -40)
        elif msg.data == 'go_to_level_2':
            # Grasp at second layer height (90)
            print("grasping at second layer!")
            self.grasp(self.center_x, self.center_y, 85)
        elif msg.data == 'go_to_level_3':
            # Grasp at third layer height (180)
            print("grasping at third layer!")
            self.grasp(self.center_x, self.center_y, 190)

        elif msg.data == 'down_small_step':
            # Use stored coordinates
            pos = position()
            print(f"Stored coordinates: x={self.abcdb}, y={self.abcdc}, z={self.abcda}")
            pos.z = self.abcda - 3
            pos.x = self.abcdb
            pos.y = self.abcdc
            if pos.z < -60:
                pos.z = -60
            print(f"Moving to: {pos}")
            self.pub_position.publish(pos)
            self.abcda = pos.z
            self.abcdb = pos.x
            self.abcdc = pos.y

        elif msg.data == 'up_small_step':
            # Use stored coordinates
            pos = position()
            print(f"Stored coordinates: x={self.abcdb}, y={self.abcdc}, z={self.abcda}")
            pos.z = self.abcda + 3
            pos.x = self.abcdb
            pos.y = self.abcdc
            # if pos.z > 190:
            #     pos.z = 190
            print(f"Moving to: {pos}")
            self.pub_position.publish(pos)
            self.abcda = pos.z
            self.abcdb = pos.x
            self.abcdc = pos.y

        elif msg.data == 'down_large_step':
            # Use stored coordinates
            pos = position()
            print(f"Stored coordinates: x={self.abcdb}, y={self.abcdc}, z={self.abcda}")
            pos.z = self.abcda - 10
            pos.x = self.abcdb
            pos.y = self.abcdc
            if pos.z < -60:
                pos.z = -60
            print(f"Moving to: {pos}")
            self.pub_position.publish(pos)
            self.abcda = pos.z
            self.abcdb = pos.x
            self.abcdc = pos.y

        elif msg.data == 'up_large_step':
            # Use stored coordinates
            pos = position()
            print(f"Stored coordinates: x={self.abcdb}, y={self.abcdc}, z={self.abcda}")
            pos.z = self.abcda + 10
            pos.x = self.abcdb
            pos.y = self.abcdc
            if pos.z > 190:
                pos.z = 190
            print(f"Moving to: {pos}")
            self.pub_position.publish(pos)
            self.abcda = pos.z
            self.abcdb = pos.x
            self.abcdc = pos.y

        elif msg.data == 'release':
            # Release pump after sucking
            print("Releasing pump...")
            pump_msg = status()
            pump_msg.status = 0
            self.pub_pump.publish(pump_msg)
            print("Pump released!")

        elif msg.data == 'grab':
            # Move down a bit
            pos = position()
            pos.z = self.abcda - 20
            pos.x = self.abcdb
            pos.y = self.abcdc
            print(f"Moving to: {pos}")
            self.pub_position.publish(pos)
            self.abcda = pos.z
            self.abcdb = pos.x
            self.abcdc = pos.y

            # Activate pump - EXACTLY matching uarm.cpp
            pump_msg = status()
            pump_msg.status = 1
            self.pub_pump.publish(pump_msg)
            rospy.sleep(0.1)

            # Move up a bit
            pos = position()
            pos.z = self.abcda + 20
            pos.x = self.abcdb
            pos.y = self.abcdc
            print(f"Moving to: {pos}")
            self.pub_position.publish(pos)
            self.abcda = pos.z
            self.abcdb = pos.x
            self.abcdc = pos.y
        
        elif msg.data =='reset':
            self.arm_position_reset()
        
        elif msg.data == 'down_to_reset':
            # Stepwise lower until reaching the mechanical/firmware floor.
            # We stop when reported Z no longer decreases across a few steps, or we hit a hard safety cap.
            hard_min_z = -120
            step_mm = 3
            pos = position()
            # Use stored coordinates if available, else fall back to current arm state
            x = self.abcdb if self.abcdb != 0 else self.arm_x
            y = self.abcdc if self.abcdc != 0 else self.arm_y
            current_z = self.abcda if self.abcda != 0 else self.arm_z
            print(f"Lowering from z={current_z} toward floor at ({x},{y})")
            self.interrupt = False
            prev_z = None
            consecutive_no_move = 0
            while not self.interrupt:
                # Check safety cap
                if current_z <= hard_min_z:
                    rospy.logwarn(f"Hit safety cap at z={current_z} (hard_min_z={hard_min_z}).")
                    break
                # Compute next step and publish
                next_z = max(current_z - step_mm, hard_min_z)
                pos.x = x
                pos.y = y
                pos.z = next_z
                self.pub_position.publish(pos)
                # Update stored coords
                self.abcda = pos.z
                self.abcdb = pos.x
                self.abcdc = pos.y
                # Allow state to update
                rospy.sleep(0.1)
                # Read back reported Z from arm state
                reported_z = self.arm_z
                if prev_z is not None and reported_z >= prev_z - 0.5:
                    consecutive_no_move += 1
                else:
                    consecutive_no_move = 0
                # Stop if Z no longer decreases for a few iterations (reached floor)
                if consecutive_no_move >= 3:
                    rospy.loginfo(f"Reached mechanical/firmware floor around z={reported_z:.1f}.")
                    break
                # Prepare next iteration
                prev_z = reported_z
                current_z = reported_z
            if self.interrupt:
                rospy.logwarn("Lowering interrupted by user.")
        
        elif msg.data == 'go_to_max_height':
            # Move to configured max height at current/stored XY
            max_height_z = 242.5
            pos = position()
            pos.x = self.abcdb if self.abcdb != 0 else self.arm_x
            pos.y = self.abcdc if self.abcdc != 0 else self.arm_y
            pos.z = max_height_z
            rospy.loginfo(f"Moving to max height at ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
            self.pub_position.publish(pos)
            self.abcda = pos.z
            self.abcdb = pos.x
            self.abcdc = pos.y
        
        else:
            height = int(msg.data)
            self.release_object(height)
            rate.sleep()

    def grasp(self, y, x, height2):        
        print("start to grasp\n")
        self.interrupt = False
        
        # Get current arm position for reference
        current_arm_pos = self.get_arm_position()
        print(f"Current UArm position: X={current_arm_pos['x']:.2f}, Y={current_arm_pos['y']:.2f}, Z={current_arm_pos['z']:.2f}")
        
        # Calculate target position with calibration
        pos = position()
        pos.x = self.x_kb[0] * x + self.x_kb[1] +50
        pos.y = self.y_kb[0] * y + self.y_kb[1] - 25 
        pos.z = height2
        
        # Store coordinates as instance variables (FIXED)
        self.abcda = pos.z
        self.abcdb = pos.x
        self.abcdc = pos.y
        
        print(f"Target position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        self.pub_position.publish(pos)

        #if not self.sleep_interrupt_check(2): 
        #    return

        # Move to grasp position
        #pos.z = height2
        #self.pub_position.publish(pos)

        #if not self.sleep_interrupt_check(1): 
        #    return

        #if not self.sleep_interrupt_check(1): 
        #    return
        
    def sleep_interrupt_check(self, duration):
        for i in range(int(duration*10)):
            if self.interrupt:
                print("action interrupted.")
                return False
            rospy.sleep(0.1)
        return True

    def release_object(self, height):
        r1 = rospy.Rate(1)
        pos = position()

        if height == 9:
            self.arm_position_reset()
            print("release at home position")
            pos.x = 230
            pos.y = 150
            pos.z = 160
            self.pub_position.publish(pos)
            r1.sleep()
        
        elif height == 1:
            # Use calculated coordinates instead of hardcoded values
            pos.x = self.abcdb - 25
            pos.y = self.abcdc
            pos.z = -50
            self.pub_position.publish(pos)
            r1.sleep()

        elif height == 2:
            pos.x = self.abcdb - 25
            pos.y = self.abcdc
            pos.z = 90  # Changed from 55 to 90
            self.pub_position.publish(pos)
            r1.sleep()

        elif height == 3:
            pos.x = self.abcdb - 25
            pos.y = self.abcdc
            pos.z = 180  # Changed from 160 to 180
            self.pub_position.publish(pos)
            r1.sleep()

        # Stop pump - EXACTLY matching uarm.cpp
        pump_msg = status()
        pump_msg.status = 0
        self.pub_pump.publish(pump_msg)
        r1.sleep()
        
        self.arm_to_home()
    
    def arm_to_home(self):
        pos = position()
        pos.z = 165
        self.pub_position.publish(pos)
        rospy.sleep(1)

    def arm_position_reset(self):
        print("UArm reset\n")
        r1 = rospy.Rate(1)
        # Reset UArm status - EXACTLY matching uarm.cpp
        status_msg = status()
        status_msg.status = 0
        self.pub_status.publish(status_msg)
        r1.sleep()
        status_msg.status = 1
        self.pub_status.publish(status_msg)
        r1.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('UArmGraspObject', anonymous=False)
        rospy.loginfo("Init UArm GraspObject main")   
        UArmGraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End UArm GraspObject main") 
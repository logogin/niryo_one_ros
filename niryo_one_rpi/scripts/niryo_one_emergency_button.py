#!/usr/bin/env python

# niryo_one_button.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import RPi.GPIO as GPIO
import subprocess
import sys, traceback

from niryo_one_rpi.rpi_ros_utils import * 

from niryo_one_msgs.srv import SetInt
from niryo_one_msgs.srv import OpenGripper
from niryo_one_msgs.srv import PingDxlTool

import digital_io_panel

TOOL_GRIPPER_1_ID = 11
EMERGENCY_BUTTON_GPIO = digital_io_panel.GPIO_1_A


class NiryoEmergencyButton:
    
    def read_value(self):
        return GPIO.input(self.pin)

    def __init__(self):
        self.pin = EMERGENCY_BUTTON_GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        rospy.loginfo("GPIO setup : " + str(self.pin) + " as input")
        rospy.sleep(0.1)

        self.last_state = self.read_value()
        self.consecutive_pressed = 0
        self.activated = True
        
        self.timer_frequency = 20.0
        # self.learning_mode_action = False
        #self.hotspot_action = False

        self.button_timer = rospy.Timer(rospy.Duration(1.0/self.timer_frequency), self.check_button)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Niryo One Emergency Button started")

    def shutdown(self):
        rospy.loginfo("Shutdown button, cleanup GPIO")
        self.button_timer.shutdown()

    def check_button(self, event):
        if not self.activated:
            return

        # Execute action if flag True
        # if self.learning_mode_action:
        #     send_learning_mode_command()
        #     self.learning_mode_action = False
        #     #self.shutdown_action = False
        # elif self.shutdown_action:
        #     send_shutdown_command()
        #     self.hotspot_action = False
        #     self.shutdown_action = False

        # Read button state
        state = self.read_value()

        # Check if there is an action to do
        if state == 0:
            self.consecutive_pressed += 1
        elif state == 1: # button released
            if self.consecutive_pressed > self.timer_frequency * 20:
                self.activated = False # deactivate button if pressed more than 20 seconds
            # elif self.consecutive_pressed > self.timer_frequency * 6:
            #     self.send_learning_mode_command = True
            # elif self.consecutive_pressed > self.timer_frequency * 3:
            #     self.shutdown_action = True
            elif self.consecutive_pressed >= 1:
                self.activate_learning_mode(True)
                self.ping_dxl_tool(TOOL_GRIPPER_1_ID, "Gripper 1")
                self.open_gripper()
            self.consecutive_pressed = 0
            
        # Use LED to help user know which action to execute
        # if self.consecutive_pressed > self.timer_frequency * 20:
        #     send_led_state(1)
        # elif self.consecutive_pressed > self.timer_frequency * 6:
        #     send_led_state(5)
        # elif self.consecutive_pressed > self.timer_frequency * 3:
        #     send_led_state(1)

    def activate_learning_mode(self, activate):
        try:
            rospy.wait_for_service('/niryo_one/activate_learning_mode', 1)
            srv = rospy.ServiceProxy('/niryo_one/activate_learning_mode', SetInt)
            resp = srv(int(activate))
            rospy.loginfo('Learning mode activation %s', resp)
        except (rospy.ServiceException, rospy.ROSException), e:
            traceback.print_exc(file=sys.stderr)
            rospy.logerror('Error activating learning mode %s', e)

    def open_gripper(self):
        try:
            rospy.wait_for_service('niryo_one/tools/open_gripper', 1)
            srv = rospy.ServiceProxy('niryo_one/tools/open_gripper', OpenGripper)
            # gripper_id=11, open_position=600, open_speed=300, open_hold_torque=128
            resp = srv(TOOL_GRIPPER_1_ID, 600, 300, 128)
            rospy.loginfo('Open gripper response %s', resp)
        except (rospy.ServiceException, rospy.ROSException), e:
            traceback.print_exc(file=sys.stderr)
            rospy.logerror('Error opening gripper %s', e)

    def ping_dxl_tool(self, tool_id, tool_name):
        try:
            rospy.wait_for_service('niryo_one/tools/ping_and_set_dxl_tool', 1)
            srv = rospy.ServiceProxy('niryo_one/tools/ping_and_set_dxl_tool', PingDxlTool)
            resp = srv(tool_id, tool_name)
            rospy.loginfo('Ping DXL response %s', resp)
            return resp.state
        except (rospy.ServiceException, rospy.ROSException), e:
            traceback.print_exc(file=sys.stderr)
            rospy.logerror('Error ping DXL %s', e)
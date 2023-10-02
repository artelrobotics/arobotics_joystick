#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from joy_to_robot import Layout

class HookControl:
    def __init__(self):
        rospy.loginfo("Joystick HookControl Intitializing")
        self._hz = rospy.get_param("~frequency", default=20)
        self.rate = rospy.Rate(self._hz)
        self.service_name = rospy.get_param("~hooks/service_name", default="hooks_ctrl")
        self.joystick_layout = Layout()
        if rospy.has_param("~layout"):
            for key, value in rospy.get_param("~layout").items():
                self.joystick_layout.__setattr__(key, value)
        self.last_states = {key:1 for key in self.joystick_layout.__dict__.keys() if "BUTTON" in key}
        self.joy_attrs = []
        for attr in Joy.__slots__:
            if attr.startswith("axes") or attr.startswith("buttons"):
                self.joy_attrs.append(attr)
        rospy.Subscriber("joy", Joy, self.joystick_callback, queue_size=1)
        # rospy.wait_for_service(self.service_name)
        self.hooks_ctrl = rospy.ServiceProxy(self.service_name, SetBool)
        self.hooks_ctrl_state = False
        rospy.loginfo("Joystick HookControl Intitialized")


    def joystick_callback(self, joy_msg):
        joystick_input_vals = {}
        for attr in self.joy_attrs:
            joystick_input_vals[attr] = getattr(joy_msg, attr)
        buttons =  joystick_input_vals.get("buttons")
        if buttons[self.joystick_layout.BUTTON_TRIANGLE] and not self.last_states["BUTTON_TRIANGLE"]:
            rospy.wait_for_service(self.service_name)
            try:
                self.hooks_ctrl(self.hooks_ctrl_state)
                self.hooks_ctrl_state = not self.hooks_ctrl_state
            except rospy.ServiceException as e:
                rospy.logerr(e)
        self.last_states["BUTTON_TRIANGLE"] = buttons[self.joystick_layout.BUTTON_TRIANGLE]


def main():
    rospy.init_node("arobotics_joystick_hooks_control")
    HookControl()
    rospy.spin()

if __name__ == "__main__":
    main()

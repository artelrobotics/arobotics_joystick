#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from roboteq_driver.srv import emergency_stop_srv
from joy_to_robot import Layout

class EmergencyStop:
    def __init__(self):
        rospy.loginfo("Joystick CmdVel Intitializing")
        self._hz = rospy.get_param("~frequency", default=20)
        self.rate = rospy.Rate(self._hz)
        self.service_name = rospy.get_param("~service_name", default="driver/emergency_stop_service")
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
        self.emergency_stop = rospy.ServiceProxy(self.service_name, emergency_stop_srv)
        rospy.loginfo("Joystick EmeergencyStop Intitialized")


    def joystick_callback(self, joy_msg):
        joystick_input_vals = {}
        for attr in self.joy_attrs:
            joystick_input_vals[attr] = getattr(joy_msg, attr)
        buttons =  joystick_input_vals.get("buttons")
        if buttons[self.joystick_layout.BUTTON_L1] and not self.last_states["BUTTON_L1"]:
            rospy.wait_for_service(self.service_name)
            try:
                self.emergency_stop(True)
            except rospy.ServiceException as e:
                rospy.logerr(e)
        self.last_states["BUTTON_L1"] = buttons[self.joystick_layout.BUTTON_L1]
        if buttons[self.joystick_layout.BUTTON_R1] and not self.last_states["BUTTON_R1"]:
            rospy.wait_for_service(self.service_name)
            try:
                self.emergency_stop(False)
            except rospy.ServiceException as e:
                rospy.logerr(e)
        self.last_states["BUTTON_R1"] = buttons[self.joystick_layout.BUTTON_R1]


def main():
    rospy.init_node("arobotics_joystick_emergency_stop")
    EmergencyStop()
    rospy.spin()

if __name__ == "__main__":
    main()

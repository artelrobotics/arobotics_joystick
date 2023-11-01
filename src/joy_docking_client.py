#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from docking_server.msg import DockingFeedback, DockingAction, DockingResult, DockingGoal
from joy_to_robot import Layout
from std_msgs.msg import Empty

class DockingClient:
    def __init__(self):
        rospy.loginfo("Joystick DockingClient Intitializing")
        self._hz = rospy.get_param("~frequency", default=20)
        self.rate = rospy.Rate(self._hz)
        self.docking_server_name = rospy.get_param("~docking_client/Action_name", default="DockingServer")
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
        self.cancel = rospy.Publisher("docking_as/cancel", Empty)
        self.goal = rospy.Publisher("docking_as/goal", DockingGoal)
        
        rospy.loginfo("Joystick Docking Client Intitialized")


    def joystick_callback(self, joy_msg):
        joystick_input_vals = {}
        for attr in self.joy_attrs:
            joystick_input_vals[attr] = getattr(joy_msg, attr)
        buttons =  joystick_input_vals.get("buttons")
        if buttons[self.joystick_layout.BUTTON_CIRCLE] and not self.last_states["BUTTON_CIRCLE"]:
            goal = DockingGoal()
            goal.goal.aruco_id = 1
            goal.goal.type = "docking"
            self.goal.pub(goal)
        self.last_states["BUTTON_CIRCLE"] = buttons[self.joystick_layout.BUTTON_CIRCLE]
        
        if buttons[self.joystick_layout.BUTTON_SQUARE] and not self.last_states["BUTTON_SQUARE"]:
            cancel = Empty()
            self.cancel.pub(cancel)
        self.last_states["BUTTON_SQUARE"] = buttons[self.joystick_layout.BUTTON_SQUARE]


def main():
    rospy.init_node("arobotics_joystick_docking_client")
    DockingClient()
    rospy.spin()

if __name__ == "__main__":
    main()

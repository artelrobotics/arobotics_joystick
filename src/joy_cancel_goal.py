#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from joy_to_robot import Layout

class CancelGoal:
    def __init__(self):
        rospy.loginfo("Joystick CancelGoal Intitializing")
        self._hz = rospy.get_param("~frequency", default=20)
        self.rate = rospy.Rate(self._hz)
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
        self._pub_goal_cancel = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self.gc_msg = GoalID()
        rospy.loginfo("Joystick CancelGoal Intitialized")

    def joystick_callback(self, joy_msg):
        joystick_input_vals = {}
        for attr in self.joy_attrs:
            joystick_input_vals[attr] = getattr(joy_msg, attr)
        buttons =  joystick_input_vals.get("buttons")
        if buttons[self.joystick_layout.BUTTON_CROSS] and not self.last_states["BUTTON_CROSS"]:
            self._pub_goal_cancel.publish(self.gc_msg)
        self.last_states["BUTTON_CROSS"] = buttons[self.joystick_layout.BUTTON_CROSS]

def main():
    rospy.init_node("arobotics_joystick_goal_cancel")
    CancelGoal()
    rospy.spin()

if __name__ == "__main__":
    main()
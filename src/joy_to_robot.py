#!/usr/bin/env python
from dataclasses import dataclass
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

@dataclass
class Layout:
    AXES_LEFT_X:     int = 0
    AXES_LEFT_Y:     int = 1
    AXES_RIGHT_X:    int = 2
    AXES_RIGHT_Y:    int = 3
    BUTTON_TRIANGLE: int = 0
    BUTTON_CIRCLE:   int = 1
    BUTTON_CROSS:    int = 2
    BUTTON_SQUARE:   int = 3
    BUTTON_L1:       int = 4
    BUTTON_R1:       int = 5
    BUTTON_L2:       int = 6
    BUTTON_R2:       int = 7
    BUTTON_SELECT:   int = 8
    BUTTON_START:    int = 9
    BUTTON_L3:       int = 10
    BUTTON_R3:       int = 11
    AXES_ARROW_LEFT_RIGHT: int = 4
    AXES_ARROW_UP_DOWN:    int = 5


class JoyToRobot:
    def __init__(self):
        self._hz = rospy.get_param("~frequency", default=20)
        self.rate = rospy.Rate(self._hz)
        self.joystick_layout = Layout()
        self.can_publish = True
        self.is_stop_published = False
        if rospy.has_param("~inputs"):
            self._inputs = rospy.get_param("~inputs")
        if rospy.has_param("~scales"):
            self._scales = rospy.get_param("~scales")
        if rospy.has_param("~layout"):
            for key, value in rospy.get_param("~layout").items():
                self.joystick_layout.__setattr__(key, value)
        self.max_speed = rospy.get_param("~max_speed", default=1.2)
        self.min_speed = rospy.get_param("~min_speed", default=0.1)
        self.vel_increment = rospy.get_param("~vel_increment", default=0.1)
        self.last_states = {key:1 for key in self.joystick_layout.__dict__.keys() if "BUTTON" in key}
        self.joy_attrs = []
        for attr in Joy.__slots__:
            if attr.startswith("axes") or attr.startswith("buttons"):
                self.joy_attrs.append(attr)
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.speed = self.vel_increment
        rospy.Subscriber("joy", Joy, self.joystick_callback, queue_size=1)

    def speedup(self):
        if self.speed <= self.max_speed - self.vel_increment and self.can_publish:
            self.speed += self.vel_increment
    def speeddown(self):
        if self.speed >= self.min_speed + self.vel_increment and self.can_publish:
            self.speed -= self.vel_increment

    def joystick_callback(self, joy_msg):
        joystick_input_vals = {}
        for attr in self.joy_attrs:
            joystick_input_vals[attr] = getattr(joy_msg, attr)
        buttons =  joystick_input_vals.get("buttons")
        if buttons[self.joystick_layout.BUTTON_R2] and not self.last_states["BUTTON_R2"]:
            self.speedup()
        if buttons[self.joystick_layout.BUTTON_L2] and not self.last_states["BUTTON_L2"]:
            self.speeddown()
        self.last_states["BUTTON_L2"] = buttons[self.joystick_layout.BUTTON_L2]
        self.last_states["BUTTON_R2"] = buttons[self.joystick_layout.BUTTON_R2]
        msg_to_pub = Twist()
        stop_msg = Twist()
        for vel_type in self._inputs:
            vel_vec = getattr(msg_to_pub, vel_type)
            for cordinate, layout in self._inputs[vel_type].items():
                scale = self._scales[vel_type].get(cordinate, 1.0)
                axes = joystick_input_vals.get("axes")
                val = axes[self.joystick_layout.__getattribute__(layout)]
                setattr(vel_vec, cordinate, scale * self.speed * val)
        if buttons[self.joystick_layout.BUTTON_START] and not self.last_states["BUTTON_START"]:
            self.can_publish = True
            self.is_stop_published = False
        self.last_states["BUTTON_START"] = buttons[self.joystick_layout.BUTTON_START]
        if buttons[self.joystick_layout.BUTTON_SELECT] and not self.last_states["BUTTON_SELECT"]:
            self.can_publish = False
        self.last_states["BUTTON_SELECT"] = buttons[self.joystick_layout.BUTTON_SELECT]
        if self.can_publish:
            self._pub.publish(msg_to_pub)
            self.rate.sleep()
        else:
            if not self.is_stop_published:
                self._pub.publish(stop_msg)
                self.is_stop_published = True

def main():
    rospy.init_node("arobotics_joystick")
    JoyToRobot()
    rospy.spin()

if __name__ == "__main__":
    main()
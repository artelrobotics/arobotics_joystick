#!/usr/bin/env python
from dataclasses import dataclass
import rospy
import importlib

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

class DynamicClassLoader:
    def __init__(self):
        # Get parameters from rospy
        param_config = rospy.get_param("~class_params", [])
        rospy.loginfo(param_config)
        # Load classes dynamically
        self.loaded_classes = []
        for class_spec in param_config:
            # Parse the class specification
            file_path, class_name = class_spec.split('/')

            # Dynamic import using importlib
            module = importlib.import_module(file_path)
            class_ = getattr(module, class_name)

            # Instantiate the class and store it
            instance = class_()
            self.loaded_classes.append(instance)


if __name__ == "__main__":
    rospy.init_node("arobotics_joystick")
    loader = DynamicClassLoader()
    rospy.spin()
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy


"""
Class to manage a HTC Vive controller.
Allows to add callbacks to different button and axes
events (press, change, release) and also query for current button and
axes state.

Based on the Joy message definition from the
package htc_vive_teleop_stuff
# Trigger, Trackpad X, Trackpad Y
axes: [0.0, 0.0, 0.0]
# Trigger, Trackpad touched, Trackpad pressed, Menu, Gripper
buttons: [0, 0, 0, 0, 0]

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class ViveController(object):
    TRIGGER_ID = 0
    TRACKPAD_TOUCHED_ID = 1
    TRACKPAD_PRESSED_ID = 2
    TRACKPAD_X_ID = 1
    TRACKPAD_Y_ID = 2
    MENU_ID = 3
    GRIPPER_ID = 4

    def __init__(self, controller_topic,
                 on_trigger_press=None,
                 on_trigger_change=None,
                 on_trigger_release=None,
                 on_menu_press=None,
                 on_menu_change=None,
                 on_menu_release=None,
                 on_touchpad_touch=None,
                 on_touchpad_press=None,
                 on_touchpad_unpress=None,
                 on_touchpad_release=None,
                 on_touchpad_change=None,
                 on_gripper_press=None,
                 on_gripper_change=None,
                 on_gripper_release=None):
        """
        Class to manage a HTC Vive Controller with ease.
        """
        self._on_trigger_press = on_trigger_press
        self._on_trigger_change = on_trigger_change
        self._on_trigger_release = on_trigger_release
        self._on_menu_press = on_menu_press
        self._on_menu_change = on_menu_change
        self._on_menu_release = on_menu_release
        self._on_touchpad_touch = on_touchpad_touch
        self._on_touchpad_press = on_touchpad_press
        self._on_touchpad_unpress = on_touchpad_unpress
        self._on_touchpad_release = on_touchpad_release
        self._on_touchpad_change = on_touchpad_change
        self._on_gripper_press = on_gripper_press
        self._on_gripper_change = on_gripper_change
        self._on_gripper_release = on_gripper_release
        # Initialize with nothing touched
        self.last_joy = Joy()
        self.last_joy.axes = [0.0] * 3
        self.last_joy.buttons = [0] * 5
        self.joy_sub = rospy.Subscriber(controller_topic,
                                        Joy,
                                        self._joy_cb,
                                        queue_size=1)
        rospy.loginfo("ViveController initialized for topic: " +
                      self.joy_sub.resolved_name)

    def _check(self, curr_joy, prev_joy, id_device,
               on_press_cb, on_release_cb, on_change_cb,
               type_device):
        """
        Check for a button or axe or touchpad to call the provided callbacks.
        :arg type_device str: 'axe', 'button'
        """
        if type_device == 'axe':
            val = curr_joy.axes[id_device]
            prev_val = prev_joy.axes[id_device]
        elif type_device == 'button':
            val = curr_joy.buttons[id_device]
            prev_val = prev_joy.buttons[id_device]
        # If newly pressed
        if val != 0.0 and prev_val == 0.0:
            if callable(on_press_cb):
                on_press_cb(val)
        # If it changed
        if val != prev_val:
            if callable(on_change_cb):
                on_change_cb(val)
        # If released
        if val == 0.0 and prev_val != 0.0:
            if callable(on_release_cb):
                on_release_cb(val)

    def _check_touchpad(self, curr_joy, prev_joy,
                        on_touch_cb, on_press_cb, on_unpress_cb,
                        on_release_cb, on_change_cb):
        """
        Specific check for the touchpad.
        """
        val_x = curr_joy.axes[self.TRACKPAD_X_ID]
        val_y = curr_joy.axes[self.TRACKPAD_Y_ID]
        val_touched = curr_joy.buttons[self.TRACKPAD_TOUCHED_ID]
        val_pressed = curr_joy.buttons[self.TRACKPAD_PRESSED_ID]
        prev_val_x = prev_joy.axes[self.TRACKPAD_X_ID]
        prev_val_y = prev_joy.axes[self.TRACKPAD_Y_ID]
        prev_val_touched = prev_joy.buttons[self.TRACKPAD_TOUCHED_ID]
        prev_val_pressed = prev_joy.buttons[self.TRACKPAD_PRESSED_ID]
        # If newly pressed
        if val_pressed != 0 and prev_val_pressed == 0:
            if callable(on_press_cb):
                on_press_cb(val_x, val_y)
        # If newly touched
        if val_touched != 0 and prev_val_touched == 0:
            if callable(on_touch_cb):
                on_touch_cb(val_x, val_y)
        # If it changed
        if val_x != prev_val_x or val_y != prev_val_y:
            if callable(on_change_cb):
                on_change_cb(val_x, val_y)
        # If unpressed
        if val_pressed == 0 and prev_val_pressed != 0:
            if callable(on_unpress_cb):
                on_unpress_cb(val_x, val_y)
        # If released
        if val_touched == 0 and prev_val_touched != 0:
            if callable(on_release_cb):
                on_release_cb(val_x, val_y)

    def _joy_cb(self, data):
        """
        Callback for the Joy topic.
        Takes care of calling the set callbacks.
        """
        rospy.logdebug("Joy cb: " + str(data))
        # Initial case
        if self.last_joy is None:
            self.last_joy = data
            return

        # Check trigger
        self._check(curr_joy=data,
                    prev_joy=self.last_joy,
                    id_device=self.TRIGGER_ID,
                    on_press_cb=self._on_trigger_press,
                    on_release_cb=self._on_trigger_release,
                    on_change_cb=self._on_trigger_change,
                    type_device='axe')

        # Check menu
        self._check(curr_joy=data,
                    prev_joy=self.last_joy,
                    id_device=self.MENU_ID,
                    on_press_cb=self._on_menu_press,
                    on_release_cb=self._on_menu_release,
                    on_change_cb=self._on_menu_change,
                    type_device='button')

        # Check touchpad
        self._check_touchpad(curr_joy=data,
                             prev_joy=self.last_joy,
                             on_touch_cb=self._on_touchpad_touch,
                             on_press_cb=self._on_touchpad_press,
                             on_unpress_cb=self._on_touchpad_unpress,
                             on_release_cb=self._on_touchpad_release,
                             on_change_cb=self._on_touchpad_change)

        # Check gripper
        self._check(curr_joy=data,
                    prev_joy=self.last_joy,
                    id_device=self.GRIPPER_ID,
                    on_press_cb=self._on_gripper_press,
                    on_release_cb=self._on_gripper_release,
                    on_change_cb=self._on_gripper_change,
                    type_device='button')

        self.last_joy = data

    def __del__(self):
        self.joy_sub.unregister()

    def get_trigger(self):
        """
        Get the latest trigger button reading.
        """
        return self.last_joy.axes[self.TRIGGER_ID]

    def is_trigger_pressed(self):
        return bool(self.last_joy.buttons[self.TRIGGER_ID])

    def is_menu_pressed(self):
        return bool(self.last_joy.buttons[self.MENU_ID])

    def get_touchpad_x_y(self):
        x = self.last_joy.axes[self.TRACKPAD_X_ID]
        y = self.last_joy.axes[self.TRACKPAD_Y_ID]
        return x, y

    def is_touchpad_pressed(self):
        return bool(self.last_joy.buttons[self.TRACKPAD_TOUCHED_ID])

    def is_touchpad_touched(self):
        return bool(self.last_joy.buttons[self.TRACKPAD_PRESSED_ID])

    def is_gripper_pressed(self):
        return bool(self.last_joy.buttons[self.GRIPPER_ID])


if __name__ == '__main__':
    rospy.init_node('test_controller')
    pub = rospy.Publisher('/vive_left', Joy, queue_size=1)
    j0 = Joy()
    j0.axes = [0.0, 0.0, 0.0]
    j0.buttons = [0, 0, 0, 0, 0]

    j1 = Joy()
    j1.axes = [1.0, 1.0, 1.0]
    j1.buttons = [1, 1, 1, 1, 1]

    # To create dummy callbacks and check they are all called
    def create_cb(info_str):
        def cb(*args, **kwargs):
            rospy.loginfo("Callback: " + info_str +
                          " called with values: " + str(args))
        return cb
# Creation generated with:
# s = """on_trigger_press=None,
# on_trigger_change=None,
# on_trigger_release=None,
# on_menu_press=None,
# on_menu_change=None,
# on_menu_release=None,
# on_touchpad_touch=None,
# on_touchpad_press=None,
# on_touchpad_unpress=None,
# on_touchpad_release=None,
# on_touchpad_change=None,
# on_gripper_press=None,
# on_gripper_change=None,
# on_gripper_release=None"""
# news = ""
# for l in s.split('\n'):
#     print l
#     on = l.split('=')[0]
#     print on
#     news += on + "=create_cb('" + on + "'),\n"
# print news

    # Try simple methods with no callbacks
    vc = ViveController('/vive_left')
    # Give time to initialize
    rospy.sleep(1.0)
    # Initial publish
    print("Publishing all values to 0.0 / 0")
    pub.publish(j0)
    rospy.sleep(0.01)

    # Call all methods
    print("vc.get_trigger(): " + str(vc.get_trigger()))
    print("vc.get_touchpad_x_y(): " + str(vc.get_touchpad_x_y()))
    print("vc.is_trigger_pressed(): " + str(vc.is_trigger_pressed()))
    print("vc.is_menu_pressed(): " + str(vc.is_menu_pressed()))
    print("vc.is_touchpad_touched(): " + str(vc.is_touchpad_touched()))
    print("vc.is_touchpad_pressed(): " + str(vc.is_touchpad_pressed()))
    print("vc.is_gripper_pressed(): " + str(vc.is_gripper_pressed()))

    print("Publishing all values to 1.0 / 1")
    pub.publish(j1)
    rospy.sleep(0.01)

    # Call all methods
    print("vc.get_trigger(): " + str(vc.get_trigger()))
    print("vc.get_touchpad_x_y(): " + str(vc.get_touchpad_x_y()))
    print("vc.is_trigger_pressed(): " + str(vc.is_trigger_pressed()))
    print("vc.is_menu_pressed(): " + str(vc.is_menu_pressed()))
    print("vc.is_touchpad_touched(): " + str(vc.is_touchpad_touched()))
    print("vc.is_touchpad_pressed(): " + str(vc.is_touchpad_pressed()))
    print("vc.is_gripper_pressed(): " + str(vc.is_gripper_pressed()))

    # Try callbacks
    vc = ViveController('/vive_left',
                        on_trigger_press=create_cb('on_trigger_press'),
                        on_trigger_change=create_cb('on_trigger_change'),
                        on_trigger_release=create_cb('on_trigger_release'),
                        on_menu_press=create_cb('on_menu_press'),
                        on_menu_change=create_cb('on_menu_change'),
                        on_menu_release=create_cb('on_menu_release'),
                        on_touchpad_touch=create_cb('on_touchpad_touch'),
                        on_touchpad_press=create_cb('on_touchpad_press'),
                        on_touchpad_unpress=create_cb('on_touchpad_unpress'),
                        on_touchpad_release=create_cb('on_touchpad_release'),
                        on_touchpad_change=create_cb('on_touchpad_change'),
                        on_gripper_press=create_cb('on_gripper_press'),
                        on_gripper_change=create_cb('on_gripper_change'),
                        on_gripper_release=create_cb('on_gripper_release'))
    # Give time to initialize
    rospy.sleep(1.0)

    # Publish to trigger callbacks
    print("Publishing all values to 0.0 / 0 (no cb should trigger)")
    pub.publish(j0)
    rospy.sleep(0.01)

    print("Publishing all values to 1.0 / 1 (all press/touch/change should trigger)")
    pub.publish(j1)
    rospy.sleep(0.01)

    print("Publishing all values to 0.0 / 0 (all release/unpress should trigger)")
    pub.publish(j0)

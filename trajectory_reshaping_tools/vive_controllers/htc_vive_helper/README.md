# htc_vive_helper
Helper classes and scripts to work with the HTC Vive on ROS (through [htc_vive_teleop_stuff](https://github.com/uts-magic-lab/htc_vive_teleop_stuff)).

## Useful scripts
* `frame_as_posestamped.py`: Creates a topic called `/YOUR_FRAME_NAME_as_posestamped` with a `sensor_msgs/PoseStamped` message with the current pose of frame in reference to `reference_frame` and published optionally at rate `rate` (defaults to 10Hz).

```bash
Usage:
./frame_as_posestamped.py frame_to_posestamped reference_frame [rate]
```

* `frame_as_odometry.py`: Same as `frame_as_posestamped.py` but with `nav_msgs/Odometry` messages.

```bash
Usage:
./frame_as_odometry.py frame_to_odometry reference_frame [rate]
```

Usage in launch file:
```xml
<launch>
  <node name="left_controller_ps" 
    pkg="htc_vive_helper" 
    type="frame_as_posestamped.py"
    args="left_controller hmd 30"/>
</launch>
```

## Useful classes

### Class to manage HTC Vive controllers
```python
from htc_vive_helper.controller import ViveController

# You need rospy.init_node('node_name') being done previously

# You can set callbacks to any of the events
def my_callback(value):
    print("Callback got value: " + str(value))
vc = ViveController('/vive_left'
                    on_trigger_press=my_callback,
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
                    on_gripper_release=None)
# You can also query for the last known state
vc.get_trigger()
vc.get_touchpad_x_y()
vc.is_trigger_pressed()
vc.is_menu_pressed()
vc.is_touchpad_touched()
vc.is_touchpad_pressed()
vc.is_gripper_pressed()
```
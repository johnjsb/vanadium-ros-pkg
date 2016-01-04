# Introduction #
Maxwell is a custom mobile manipulator.

[Parts List](https://spreadsheets.google.com/spreadsheet/pub?hl=en_US&hl=en_US&key=0AoEGbFgI5kOhdG9fRkFTVnc5VXkyZ0JYcW5laDE3NkE&single=true&gid=0&output=html)

## Head details ##
```
pan servo id = 1
tilt servo id = 2
```

## Arm Details ##
```
base rotate id = 3 (RX-64)
shoulder id = 4 (EX-106)
elbow id = 5 (EX-106)
wrist id = 6 (RX-64)
twist id = 7 (AX-12)
grip left = 8 (AX-12)
grip right = 9 (AX-12)
```

# ROS Setup #
Maxwell has been developed against diamondback-desktop-full variant of ROS. Additionally, you'll need to install the following stacks:
```
laser_drivers
kinematics
openni_kinect
urdf_tools (not in debs)
maxwell (not in debs)
arbotix (not in debs)
```
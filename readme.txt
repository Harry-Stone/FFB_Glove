This is the repository for a three fingered force feedback glove.
The Software is provided as is and is not maintined, but if you have questions I can be contacted through Github
or you can steal the email address associted with the drive link below and use that

There is a meshes folder for the URDF that is not included in the GIT repo for best practices
it can be found at:
https://drive.google.com/file/d/1w8HkZ7gCtW_Y7MOri8pdyC3q7gXZgteO/view?usp=sharing

unzip it and leave it in the src/glove_description/meshes folder, it also includes the STLs used to print the mechanical parts,
So i guess you could print it that way, but I do intend to do full mechanical doccumentation
as well as upload the microcontroller code for the encoder reader.

This was developed on ROS kilted, but also works on humble
-Any hope of cross compatability died with gazebo

The haply node uses:
https://docs.haply.co/inverseSDK/service/
systemctl start haply-inverse-service.service

I have a tendency to forget this...
ros2 run robot_state_publisher robot_state_publisher   --ros-args --params-file /home/harry/glove/src/hand_description.yaml

To demo the glove in its full setup at the moment

Plug in:
    Encoder reader USB
    U2D2
    ^^^
As long as Encoder is before U2D2 order is not important

Plug in:
    Haply
    Haply dongle
    Audio amps

Calibrate haply through the haply service GUI
Power on and Calibrate haply pen
Attach haply pen to glove with ball end facing the user

There are 3 demo sctips at the moment,
forces.launch.py -> applies haptic feedback on fingertip collision
visualisation.launch.py -> no haptic feedback, visualises full pose with physics sim
glove_visualiser.py -> physicsless raw pose visualiser

after running source and build these files can be run with
ros2 launch forces.launch.py

Vibration is not yet finished and does not really do anything

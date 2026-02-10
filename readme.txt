This is the repository for a three fingered force feedback glove.
The Software is provided as is and is not maintined, but if you have questions I can be contacted through Github
or you can steal the email address associted with the drive link below and use that

There is a meshes folder for the URDF that is not included in the GIT repo for best practices
it can be found at:
https://drive.google.com/file/d/1w8HkZ7gCtW_Y7MOri8pdyC3q7gXZgteO/view?usp=sharing

unzip it and leave it in the src folder, it also includes the STLs used to print the mechanical parts,
So i guess you could print it that way, but I do intend to do full mechanical doccumentation
as well as upload the microcontroller code for the encoder reader.

This was developed on ROS kilted, but also works on humble

The haply node uses:
https://docs.haply.co/inverseSDK/service/
systemctl start haply-inverse-service.service



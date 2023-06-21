FROM ros:noetic-ros-core

RUN rm -rf /var/lib/apt/lists

# Install gstreamer dependencies 
RUN sudo apt-get update 

RUN sudo apt-get install -y python3 python3-pip  ros-noetic-rospy ros-noetic-cv-bridge ros-noetic-tf ros-noetic-mavros ros-noetic-mavros-msgs

RUN pip3 install vincenty
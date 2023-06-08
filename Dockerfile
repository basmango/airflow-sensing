FROM ros:noetic-ros-core

RUN rm -rf /var/lib/apt/lists

# Install gstreamer dependencies 
RUN sudo apt-get update 

RUN sudo apt-get install -y python3  ros-noetic-rospy ros-noetic-cv-bridge ros-noetic-tf ros-noetic-mavros ros-noetic-mavros-msgs


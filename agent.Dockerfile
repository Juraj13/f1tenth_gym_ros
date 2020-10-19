# Base image
FROM ros:melodic-robot-bionic

# Update apt repo and pip2, and install python3, pip3
RUN apt-get update --fix-missing && \
    apt-get install -y python-pip \
                       python3-dev \
                       python3-pip

# Install apt dependencies, add your apt dependencies to this list
RUN apt-get install -y git \
                       build-essential \
                       cmake \
                       vim \
                       ros-melodic-ackermann-msgs \
                       ros-melodic-genpy

# Upgrade pip
RUN pip install --upgrade pip

# Install pip dependencies, add your pip dependencies to this list
RUN pip install numpy==1.16.0 \
                scipy==1.2.0 \
                pyyaml
# RUN pip3 install numpy==1.16.0 \
#                  scipy==1.2.0 \
#                  pyyaml

# Creating a catkin workspace
RUN mkdir -p /catkin_ws/src

# Clone or copy over your source code

# Copying
COPY /home/ivan/catkin_ws/src/f1tenth_gym_ros/ /catkin_ws/src/

# Cloning
#RUN cd /catkin_ws/src/ && \
#    git clone https://github.com/Juraj13/f1tenth_gym_ros.git && \
#    git checkout ipavlak-dev

# Building your ROS packages
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"

# Uncomment set the shell to bash to provide the "source" command
SHELL ["/bin/bash", "-c"] 

# Setting entry point command that runs when the container is brought up
CMD source /catkin_ws/devel/setup.bash; roslaunch --wait f1tenth_gym_ros trajectory_publisher.launch

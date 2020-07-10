FROM ros:melodic
WORKDIR /project

# Install git
RUN apt-get update
RUN apt-get install -y git
RUN git clone https://github.com/aravindsrj/plume_nav.git

WORKDIR plume_nav/src

# Get the project files
RUN git clone https://github.com/shantanuwadnerkar/gaden.git
RUN git clone https://github.com/MAPIRlab/olfaction_msgs.git
WORKDIR ..

# Dependencies
RUN apt-get update && \
    apt install -y ros-melodic-pcl-ros
RUN apt-get install -y ros-melodic-rviz
RUN  apt-get install -y libopencv-dev python3-opencv
RUN rosdep update && \
     rosdep install -y \
      --from-paths \
        src/gaden/gaden_filament_simulator \
        --ignore-src && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash && catkin_make'
RUN echo "PROJECT BUILT!!"
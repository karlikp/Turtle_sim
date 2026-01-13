FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
SHELL ["/bin/bash", "-c"]

# Podstawowe narzędzia + repozytoria
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    software-properties-common \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

# Locale (często wymagane przez ROS)
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS 2 apt repo + klucz
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# Instalacja ROS 2 + TurtleBot4 + Gazebo (Fortress) + narzędzia
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # TurtleBot4 simulator (Ignition/Gazebo Sim) + zależności Create 3
    ros-humble-turtlebot4-simulator \
    ros-humble-irobot-create-nodes \
    # "Out of box" integracja ROS<->Gazebo dla Humble (instaluje rekomendowaną wersję Gazebo dla Humble)
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# rosdep init (w kontenerze często bywa przydatne)
RUN rosdep init || true
RUN rosdep update

# Przygotuj środowisko przy starcie kontenera
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

RUN sudo apt-get update && sudo apt-get install wget
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update

RUN apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    ros-humble-turtlebot4-simulator \
 && rm -rf /var/lib/apt/lists/*


# uav_camera_det (pip: numpy + opencv)
RUN pip3 install ultralytics && \
    python3 -m pip uninstall -y \
      numpy opencv-python opencv-python-headless opencv-contrib-python opencv-contrib-python-headless \
      || true && \
    python3 -m pip install --no-cache-dir "numpy==1.26.4" "opencv-python<4.10"

# Użytkownik nie-root (praktyczne przy X11/plikach)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG dialout,video,audio $USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

CMD ["bash"]

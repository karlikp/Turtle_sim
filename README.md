# Turtle_sim

### Requirements
- **Docker** (Docker CLI recommended)
  - Add your user to the Docker group to grant the necessary permissions; otherwise, you must prefix commands with sudo.
  
- **Visual Studio Code** (recommended) with the following extensions:
  - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Docker build
```bash
docker build -t turtle_sim:latest . --build-arg USER_UID=$(id -u)
```

### Run for development
Running docker with GPU support requires [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), but if you don't have NVIDIA GPU, you can remove the `--gpus=all` flag in `devcontainer.json` and run this container only with CPU.

Click `Ctrl+Shift+P` and select `Dev Containers: Rebuild and Reopen in Container`. 
This will open the repository in the container and you can start developing.

To rebuild workspace use shortcut `Ctrl+Shift+B` in the vscode.

### Run simulation in the office

1. Launch simulation in the gazebo:

In first terminal:
```bash
colcon build
source install/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=office model:=lite x:=4 y:=7
```
2. Execute visualization in the RVIZ2:

In second terminal:
```bash
source install/setup.bash
rviz2
```

Add -> By topic -> /oakd/rgb/preview/depth/Image

Set bellow settings:
![alt text](images/image.png)

Add -> By topic -> /oakd/rgb/preview/image_raw/Image

3. Execute object detection:

In third terminal
```bash
source install/setup.bash
python3 turtlebot_sim/uav_camera_det.py
```



# Simulation-To-STARS Bridge (CARLA as Example)

## First time setup (Ubuntu 20.04)
Make sure to have `docker` installed. The `nvidia-docker2-runtime` is needed aswell [as described in the nvidia documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
Additional add `xhost +` to the `~/.bashrc` to allow access to the systems displays.

## First time setup (Windows with WSL)

### WSL and Docker setup
Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/). Proceed to [enable WSL2 Support in Windows](https://learn.microsoft.com/en-us/windows/wsl/install). Then install [Ubuntu 20.04 from the Microsoft Store](https://apps.microsoft.com/store/detail/9N6SVWS3RX71?hl=de-de&gl=DE&rtc=1) and activate it in the Docker Desktop settings. To do that, navigate to the `Resources->WSL Integration` tab and activate the integration for Ubuntu-20.04. Right after restart Docker Desktop to apply the changes. Now Docker will be available inside Ubuntu running in WSL.

### Running the stack
Just run `docker-compose up` in the root of the project to start all containers; they will be pulled from their own repositories respectively.

### Run in VSCode Devcontainer
If so desired, the setup included in this repository can be used to open and develop inside a devcontainer provided by VSCode.
To be fully operational the configs inside the `.devcontainer` and `.vscode` folders can be used but the Python/Pylance extension (@ext:ms-python.vscode-pylance) must be reconfigured. The analysis include path must be extended by the `/opt/ros/humble/lib/python3.10/site-packages/**` path as well as the `/app/install/lib/python3.10/site-packages/**` path. The following paths must be added to the analysis excluded paths: `**/stars_ros_exporter/test/**`, `**/build/**` as well as `**/install/**`.
By following these instructions VSCode should now recognize the custom ROS node fully. It is then possible to use the `Ctrl+Shift+P` shortcut in VSCode to `Dev Containers: Reopen in Container` the current folder and successfully open the repository inside the now running docker-compose stack. There it can be changed as it were open locally. To rebuild an restart the node use the `Ctrl+Shift+B` shortcut.
To view the logs of the docker-compose environment, open a new local (meaning host system) shell by using the `Ctrl+Shift+P` shortcut and using the `Terminal: Create New Integrated Terminal (Local)` command.
The return to the local workspace is possibly by clicking on the bottom left environment and then selecting `Reopen Folder Locally` in the prompt.

## Update versions
Change the version numbers in the `.env` file in the root of the project.

## Error messages on startup
The error messages starting with `ALSA lib` can be ignored as they only reflect issues with the sound playback which is not required.
The error messages starting with `sh: 1: xdg-user-dir: not found` can be ignored as well. It is a "red herring" according to [CARLA's GitHub issue section](https://github.com/carla-simulator/carla/issues/3514#issuecomment-875545929).

## Other repositories used:
[STARS-ROS-Messages](https://github.com/valentinrusche/stars-ros-messages)
[CARLA-ROS-Bridge](https://github.com/valentinrusche/ros-bridge)
[CARLA-STARS-ROS-Mapper](https://github.com/valentinrusche/stars-carla-ros-mapper)
[OpenDRIVE-Parser](https://github.com/valentinrusche/python-opendrive-parser)
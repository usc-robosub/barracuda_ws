# barracuda_ws
ROS2 workspace with all of the packages used by the onboard Jetson

## ROS2 package notes
* The barracuda_onboard package contains the launch file that launches all of the nodes that run on the Jetson
* See https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html for a guide on how to create a new ROS2 package
* **Dependencies should be specified in package.xml** -  `rosdep install` is run in the Dockerfile to install system dependencies (see https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
* **The default launch file should follow the naming convention pkg_name.launch.py**
  * `barrracuda_onboard.launch.py` will scan the `src` directory in the workspace for the package names and then launch `pkg_name.launch.py` for each package - this way, we don’t have to manually update `barracuda_onboard.launch.py` whenever we add a new package to the workspace
  * these launchfiles can act as wrappers for more descriptively named launch files; we can maintain flexibility in what we want to launch with this approach: as an example, if we have `pkg_name.launch.py`, `do_thing1.launch.py`, `do_thing2.launch.py` in the `pkg_name/launch` directory, we can use ROS params to determine if `do_thing1.launch.py` and/or `do_thing2.launch.py` gets included in `pkg_name.launch.py`
* All packages with launchfiles meant to be launched directly from `barracuda_onboard.launch.py` should be located in `barracuda_ws/src` and follow the guidelines above
* Packages that need to be built as dependencies but don't contain launchfiles meant to be launched directly should go in `barracuda_ws/src/dependency-pkgs` (for example the packages in the zed-ros2-wrapper submodule)


### Why the Docker base image changed
* The workspace now builds from `ghcr.io/usc-robosub/barracuda-camera-image:latest` instead of `ros:humble-ros-base-jammy`.
* This base image is the team-maintained camera-ready image published in GitHub Container Registry for `usc-robosub`.
* It is intended to bundle Jetson/camera prerequisites so this workspace can focus on workspace-specific packages and launch behavior.
* Package listing: https://github.com/orgs/usc-robosub/packages
* Source image Dockerfile: open the package entry from the org package listing above and follow its linked source artifact/repository.

### Why these docker-compose environment variables are set
* `NO_LAUNCH=true` (in `*-no-launch` services): tells `entrypoint.sh` to skip `ros2 launch barracuda_onboard barracuda_onboard.launch.py`, which is useful for interactive debugging and manual launch testing.
* `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`: forces UDP transport selection in Fast DDS for straightforward network behavior across Jetson/LAN setups.
* `ROS_DOMAIN_ID=0`: keeps all nodes in the same default ROS domain unless intentionally overridden.
* `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`: pins ROS2 middleware inside the container to Fast DDS (`rmw_fastrtps_cpp`) so node behavior stays consistent across environments.
* Together with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, DDS traffic to the host machine and other devices is carried over UDPv4 (instead of relying on middleware defaults).

### Why these docker-compose volume mounts are set
* `${PWD}/src:/root/barracuda_ws/src`
  * Needed for live-edit development of in-workspace ROS2 packages from the host without rebuilding the image every code change.
  * Required by the package auto-discovery behavior in `barracuda_onboard.launch.py` (it scans `/root/barracuda_ws/src` for package launch directories).
* `/tmp:/tmp`
  * Provides access to host-side temporary files and shared IPC artifacts that some tools/drivers use during development and debugging.
* `/var/run/nvargus-daemon/socket:/var/run/nvargus-daemon/socket`
  * Required on Jetson for camera services that talk to NVIDIA Argus through the host daemon socket.
  * Relevant for camera bring-up paths (for example, when launching camera stacks via `barracuda_camera`).
* `/dev/nvhost-ctrl`, `/dev/nvhost-ctrl-gpu`, `/dev/nvhost-gpu`
  * Expose required Jetson GPU/host control devices to the container for camera and accelerated processing paths.

## Setting up your development environment
You can set up a development environment for yourself both on the Jetson in the lab, and on your local machine.
Working directly on the Jetson will be useful for testing code for sensors & actuators, but you can still test program behavior with ros2 commands and Foxglove on your local machine.

**Before proceeding on the Jetson, create an admin account for yourself and log in.**
Before proceeding on your local machine, make sure you have [Docker Engine](https://docs.docker.com/engine/install/) or [Docker Desktop](https://docs.docker.com/desktop/) installed.

### Cloning this repo
On the Jetson (and on your local machine if you haven't already done so), generate an ssh key and [add it to your Github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
Then run 

```bash
git config --global user.name "<your name>"
``` 

and 

```bash
git config --global user.email "<your email>"
``` 

so you can push to the remote later on.
Use the SSH url to clone and cd into this repo 

```bash 
git clone git@github.com:usc-robosub/barracuda_ws.git && cd barracuda_ws
```

 **then run** 

```bash
git submodule update --init --recursive
``` 

**to initialize the submodule dirs (they will be empty before you do this)**.

## Commands for building docker images & working with containers
In the following commands, the optional `IMG_ID` and `PKG_SEl` environment variables are set inline to make it clear which commands they affect, but it will often be more convenient to set them in the `.env` file.
From `barracuda_ws/`, you can run 

```bash
cp .env.template .env
``` 

to create your `.env` file from the template.

```bash
[PKG_SEL="<pkg_name_1 pkg_name_2 ...>"] [IMG_ID=<string-appended-to-default-image-name>] docker compose build
```

* Creates an image with all included ROS2 packages built, named `$USER-barracuda-onboard` by default if `PKG_SEL` and `IMG_ID` are not set
* If `PKG_SEL` is set, only the specified packages get built during the image build process, and `IMG_ID` specifies the string appended to the default image name when set
* These two env vars can be used together to create uniquely named images with different combinations of ROS2 packages built in them

```bash
[IMG_ID="<id>"] docker compose up [-d] [service-name]
```

* Spins up a container from a custom-named image if `IMG_ID` is set, or from the image with the default name otherwise
* The container will have the same name as the image
* If `service-name` isn't specified, the default service is `host`
  * There are four services to choose from: `host`, `host-no-launch`, `bridge`, `bridge-no-launch`
  * Different services use different network modes and specify whether or not to launch the master launchfile upon spinning up the container - see `docker-compose.yaml` for more info

```bash
[PKG_SEL="<pkg_name_1 pkg_name_2 ...>"] [IMG_ID=<id>] docker compose up --build [--no-cache] [service-name]
```

* Combines the previous two commands
* Setting `service-name` will only affect how the container gets spun up; it will not affect the build process (all the services use the same values for the `build` key in `docker-compose.yaml`)

```bash
docker exec -it <container-name> bash
```

* Creates a bash shell session in a running container

```bash
docker compose down [service-name]
```

* Stops and removes a container that was created with `docker compose up`
* If you ran `docker compose up <service-name>` with a service other than `host`, you will need to run `docker compose down <service-name>`

```bash
[IMG_ID="<id>"] docker compose run --rm -it [service-name]
```

* Spins up a one-off container from either the default image or a custom one when `IMG_ID` is set, which gets removed after exiting the container
* Useful for quickly getting into a container shell when used with one of the `X-no-launch` services - a good use case for this is debugging launch files
  * `docker compose run --rm -it <host/bridge>-no-launch` will put you in a bash shell without needing to run both the `compose up` and `exec -it` commands

## Contributing
* Create a branch for the issue you're working on, or check out the corresponding branch if you're working in a group and someone else has already created it
* If there are pushes to main while you're working on your branch, rebase your branch onto main so we can keep a linear commit history when we merge your branch into main later on
* When you finish working on an issue, make sure that the onboard image builds on the Jetson and that you can spin up a container without any errors
* Update any relevant documentation
* Make a pull request based on main

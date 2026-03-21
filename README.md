# barracuda_ws
ROS2 workspace with all of the packages used by the onboard Jetson included as submodules, as well as the barracuda_onboard package, which contains the launch file that launches all of the nodes that run on the Jetson. 

## ROS2 package repo strucure  
* The package repos are structured as pure colcon packages (the package.xml file should be in the top level of the repo, see https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
* Dependencies should be specified in package.xml -  ``` rosdep install ``` is run in the Dockerfile to install system dependencies (see https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
* The default launch file should follow the naming convention pkg_name.launch.py
  * onboard_launch.py will scan the src directory in the workspace for the package names and then launch pkg_name.launch.py for each package - this way, we don’t have to manually update barracuda_onboard.launch.py whenever we add a new package to the workspace
  * these launchfiles can act as wrappers for more descriptively named launch files; we can maintain flexibility in what we want to launch with this approach: as an example, if we have pkg_name.launch.py, do_thing1.launch.py, do_thing2.launch.py in the launch dir of the pkg_name package, we can use ROS params to determine if do_thing1.launch.py and/or do_thing2.launch.py gets included in pkg_name.launch.py

## Setting up your development environment
You can set up a development environment for yourself both on the Jetson in the lab, and on your local machine.
Working directly on the Jetson will be useful for testing code for sensors & actuators, but you can still test program behavior with ros2 commands and Foxglove on your local machine.

Before proceeding on the Jetson, create an admin account for yourself and log in.
Before proceeding on your local machine, make sure you have [Docker Engine](https://docs.docker.com/engine/install/) or [Docker Desktop](https://docs.docker.com/desktop/) installed.

### Cloning this repo
On the Jetson (and on your local machine if you haven't already done so), generate an ssh key and [add it to your Github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
Then run ``` git config --global "<your name>" ``` and ``` git config --global "<your email>" ``` so you can push to the remote later on.
Use the SSH url to clone this repo (``` git clone git@github.com:usc-robosub/barracuda_ws.git ```)
cd into barracuda_ws, then run ``` git submodule update --init --recursive ``` to initialize the submodule dirs (they will be empty before you do this).

## Commands for building docker images & working with containers
```[PKG_SEL=<"pkg_name_1 pkg_name_2 ...">] [IMG_ID=<string-appended-to-default-image-name>] docker compose build```
* Creates an image with all included ROS2 packages built, named $USER-barracuda-onboard by default if PKG_SEL and IMG_ID are not set
* If PKG_SEL is set, only the specified packages get built during the image build process, and IMG_ID specifies the string appended to the default image name when set
* These two env vars can be used together to create uniquely named images with different combinations of ROS2 packages built in them

```[IMG_ID=<id>] docker compose up [-d] [service-name]```
* Spins up a container from a custom-named image if IMG_ID is set, or from the image with the default name otherwise
* The container will have the same name as the image
* If service-name isn't specified, the default service is ```host```
  * There are four services to choose from: ```host```, ```host-no-launch```, ```bridge```, ```bridge-no-launch```
  * Different services use different network modes and specify whether or not to launch the master launchfile upon spinning up the container - see docker-compose.yaml for more info

```[PKG_SEL=<"pkg_name_1 pkg_name_2 ...">] [IMG_ID=<id>] docker compose up --build [--no-cache] [service-name]```
* Combines the previous two commands
* Setting ```service-name``` will only affect how the container gets spun up; it will not affect the build process (all the services use the same values for the "build" key in docker-compose.yaml)

```docker exec -it <container-name> bash```
* Creates a bash shell session in a running container

```docker compose down [service-name]```
* Stops and removes a container that was created with ```docker compose up```
* If you ran ```docker compose up service-name``` with a service other than ```host```, you will need to run ```docker compose down``` service name

```[IMG_ID=<id>] docker compose run --rm -it [service-name]```
* Spins up a one-off container from either the default image or a custom one when IMG_ID is set, which gets removed after exiting the container
* Useful for quickly getting into a container shell when used with one of the x-no-launch services
  * ```docker compose run --rm -it <host/bridge>-no-launch``` will put you in a bash shell without needing to run both the compose up and exec -it commands

## Contributing
### barracuda_ws issues
[TODO]
#### Making changes to a package already included as a submodule
[TODO]
#### Bringing up a new package in barracuda_ws/src
[TODO]
#### Making changes to the Dockerfile, docker-compose.yaml, and/or entrypoint.sh
[TODO]

## Submodule notes
* General background: https://git-scm.com/book/en/v2/Git-Tools-Submodules
* You can see/change which branch a submodule tracks in the .gitmodules file
* Currently, all package repo submodules are set to track the ros2-pkg branch, except for the barracuda_thrusters repo which tracks the ros2-pkg-jetson branch
* To update a submodule repo to the latest commit on its tracked branch, run ``` git submodule update --remote ```
* If you want to do development on a given package, make sure you are working on the intended branch (``` cd src/<pkg name>```, ``` git branch ``` to check)
* To add a new submodule, run ```git submodule add [-b <branch to be tracked>] <repo ssh url> [submodule path] ```
  * For example, from barracuda_ws you could run: ``` git submodule add -b ros2-pkg git@github.com:usc-robosub/barracuda_example_package.git src/barracuda_example_package ```
  * Or from barracuda_ws/src, you could run ``` git submodule add -b ros2-pkg git@github.com:usc-robosub/barracuda_example_package.git ```
* To remove a submodule, run ``` git rm <path/to/submodule> ```

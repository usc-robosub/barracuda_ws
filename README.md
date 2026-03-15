# barracuda_ws
ROS2 workspace with all of the packages used by the onboard Jetson included as submodules, as well as the barracuda_onboard package, which contains the launch file that launches all of the nodes that run on the Jetson. 

## ROS2 package repo strucure  
* The package repos are structured as pure colcon packages (the package.xml file should be in the top level of the repo, see https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
* Dependencies should be specified in package.xml -  ``` rosdep install ``` is run in the Dockerfile to install system dependencies (see https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)

## Setting up your development environment on the Jetson
1. Create an admin account for yourself and log in
2. Generate an ssh key and [add it to your Github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
3. Run ``` git config --global "<your name>" ``` and ``` git config --global "<your email>" ```
4. Use the SSH url to clone this repo (``` git clone git@github.com:usc-robosub/barracuda_ws.git ```)
5. cd into barracuda_ws, then run ``` git submodule update --init --recursive ```
6. To build the onboard image, run ``` docker compose build ```. This will build a new image whose name starts with your username (your_username-barracuda-onboard-jetson). This is to make it harder to accidentaly mess with an image/container that someone else is using for development. The docker directory is stored on a 1TB SSD, so we should have enough space to accomodate this setup (see https://www.jetson-ai-lab.com/tutorials/ssd-docker-setup/#docker).
7. To spin up the container and launch all nodes, run ``` docker compose up ```. The container will have the same name as the image you just built. 
8. To stop and remove the container, run ``` docker compose down ```

## Suggested development workflow
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

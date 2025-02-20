# Pointcloud Multi-target Tracking and Estimation

This is an implementation of the multi-target tracking algorithm proposed in the paper "On Onboard LiDAR-based Flying Object Detection" (see bottom of this Readme).
It is designed to complement the [Volumetric Flying Object Detection (VoFOD) system](https://github.com/ctu-mrs/vofod) by providing detection association, sporadic false positive filtering, and state estimation of the tracked targets with reduced latency.
Although the code is set up to use VoFOD's detections and pointclouds from a LiDAR sensor, it can easily be modified to work with other detectors and sensors providing pointcloud data.
For more details, see the paper.

## Installation

Prequisites:

* Ubuntu 20.04 (may work on other distros, this one is tested)
* git (`sudo apt install git`)
* VoFOD (see installation instructions in [the repository](https://github.com/ctu-mrs/vofod))
 - the detections can be supplied by any system that provides 3D positions of the targets with minor code modifications

Installation instructions (these assume that you installed VoFOD using the instructions above):

1. Clone this repository and build it:
```
# it is good practice to not clone ROS packages directly into a workspace, so let's use a separate directory for this
git clone git@github.com:ctu-mrs/lidar_tracker.git ~/git/lidar_tracker        # clone this repository
# git clone https://github.com/ctu-mrs/lidar_tracker.git ~/git/lidar_tracker  # if you do not have a private key set up on Github, you can use https instead of ssh
ln -s ~/git/lidar_tracker ~/workspace/src                                     # create a symbolic link of the repository to the workspace
cd ~/workspace/src && catkin build lidar_tracker                              # build the package within the workspace
```

That's it! Now you can start the basic simulation example:
```
source ~/workspace/devel/setup.bash     # source the workspace to see the packages within
roscd lidar_tracker/tmux/simulation     # change directory to the one containing the tmux script
./start.sh
```

After this, you should see something like the following:

![LiDAR tracker running in the Gazebo simulator](https://github.com/ctu-mrs/lidar_tracker/raw/gifs/lidar_tracker_sim.gif)

The red sphere in Rviz on the right is the detection of the other UAV, and the green cubes represent the occupied voxels.
The red arrow represents the filtered position of the target, its direction corresponds to the direction of the estimated velocity, and the smaller purple sphere represents the uncertainty covariance of the position estimate.
Finally, the red line is the target's predicted trajectory for the next two seconds.

## See also

* Volumetric Flying Object Detection: [https://github.com/ctu-mrs/vofod](https://github.com/ctu-mrs/vofod)
* MRS UAV System: [https://github.com/ctu-mrs/mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system)
* Webpage with related multimedia materials: [https://mrs.felk.cvut.cz/flying-object-detection](https://mrs.felk.cvut.cz/flying-object-detection)

## Cite as

Matouš Vrba, Viktor Walter, Václav Pritzl, Michal Pliska, Tomáš Báča, Vojtěch Spurný, Daniel Heřt, and Martin Saska, "On Onboard LiDAR-based Flying Object Detection," IEEE Transactions on Robotics, vol. 41, pp 593-611, 2025.

```
@article{vrba2025OnboardLiDARBasedFlying,
  title = {On Onboard LiDAR-Based Flying Object Detection},
  author = {Vrba, Matouš and Walter, Viktor and Pritzl, Václav and Pliska, Michal and Báča, Tomáš and Spurný, Vojtěch and Heřt, Daniel and Saska, Martin},
  year = {2025},
  journal = {IEEE Transactions on Robotics},
  volume = {41},
  pages = {593--611},
  issn = {1941-0468},
  doi = {10.1109/TRO.2024.3502494}
}
```

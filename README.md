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

![LiDAR tracker running in the Gazebo simulator](https://github.com/ctu-mrs/vofod/raw/gifs/lidar_tracker_gazebo.gif)

The red sphere in Rviz on the right is the detection of the other UAV, and the green cubes represent the occupied voxels.

## See also

* Volumetric Flying Object Detection: [https://github.com/ctu-mrs/vofod](https://github.com/ctu-mrs/vofod)
* MRS UAV System: [https://github.com/ctu-mrs/mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system)
* Webpage with related multimedia materials: [https://mrs.felk.cvut.cz/flying-object-detection](https://mrs.felk.cvut.cz/flying-object-detection)

## Cite as

Matouš Vrba, Viktor Walter, Václav Pritzl, Michal Pliska, Tomáš Báča, Vojtěch Spurný, Daniel Heřt, and Martin Saska, "**On Onboard LiDAR-based Flying Object Detection**," arXiv preprint 2303.05404, 2023.

```
@misc{vrba2023fod,
	title = "On Onboard {LiDAR}-based Flying Object Detection",
	author = "Matouš Vrba and Viktor Walter and Václav Pritzl and Michal Pliska and Tomáš Báča and Vojtěch Spurný and Daniel Heřt and Martin Saska",
	note = "preprint, arXiv 2303.05404",
	year = 2023,
	eprint = "2303.05404",
	archiveprefix = "arXiv",
	primaryclass = "cs.RO",
	pdf = "https://arxiv.org/pdf/2303.05404"
}
```

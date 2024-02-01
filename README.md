# uav_dataset_generation_ros

This is a ROS 2  package for automated generating of large UAV datasets. It currently supports generation of synchronized 3D position trajectories, GPS coordinates, IMU readings, RGB and depth images. 

**NOTE** This work is submitted for publication to ICUAS 2024. Dataset will be released upon acceptance.

## Dependencies

* ROS 2 humble + Gazebo `garden`
* PX4 Atuopilot

## Installation

The simulation deveopment environment is available in a self-contained docker image. First follow the instructions at [d2dtracker_sim_docker](https://github.com/mzahana/d2dtracker_sim_docker)

## Run

* Combile the workspace using `colcon build`

* Source the workspace `source install/setup.bash`

* **Launch Simulation:** In your first terminal, initiate the simulation by running:

    ```bash
    ros2 launch uav_dataset_generation_ros random_trajectories.launch.py
    ```

    Upon execution, Gazebo should display a quadcopter.

* **Run QGroundControl:** Open a second terminal and launch QGroundControl:

    ```bash
    ./QGroundControl.AppImage
    ```

    This action should showcase the quadcopter within QGroundControl.

Once the setup is complete, initiate takeoff and switch to offboard mode to observe the drone execute random trajectories within the simulation environment.

# Citation
If you use this repository, please cite
```
@software{Abdelkader_uav_dataset_generation_ros_2024,
author = {Abdelkader, Mohamed and Gabr, Khaled},
month = jan,
title = {{uav_dataset_generation_ros}},
url = {https://github.com/mzahana/uav_dataset_generation_ros},
version = {1.0.0},
year = {2024}
}
```

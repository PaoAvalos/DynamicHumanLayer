# DynamicHumanLayer
Custom Costmap Layer for representing camera-based human detections into Gaussian shape human poses- can be added as a plugin to the existing costmap

This Costmap 2D Layer is part of my MSc Thesis involving improving the dynamic obstacle representation for humans in indoor navigation. This package is for the layer only. 
<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites
* Clone the ROS YOLO package [https://github.com/Jyrijoul/ros_yolo] and follow the installation instructions.
* Get the package for the YOLO bounding box prediction and visualization (https://github.com/Jyrijoul/ros_3d_bb) and follow the installation instructions.
### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/PaoAvalos/DynamicHumanLayer
   ```
2. Go to your workspace root and build it
   ```sh
   catkin build your_workspace
   ```
3. Add layer plugin to local_costmap_params.yaml
   ```
   local_costmap:
    plugins:
      - {name: static_layer,    type: "costmap_2d::StaticLayer"}
      - {name: obstacle_layer,  type: "costmap_2d::ObstacleLayer"}
      - {name: DynamicHumanLayer, type: "costmap_2d::ObstacleLayer"}
      - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

   ```

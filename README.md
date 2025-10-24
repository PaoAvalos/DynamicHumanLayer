# Dynamic Human Layer

> Layer that represents human obstacles as Velocity-Depent Gaussian Distributed Shape for the path planner

![Demo](docs/demo.gif)

## 🎯 Key Features

- **Feature 1**: Achieved 10% improvement in navigation success rate 
- **Feature 2**: Achieves real-time performances and changes without lag
- **Feature 3**: Works on ROS/ROS2 and is hardware agnostic

## 📊 Results

| Metric | Baseline | This Work | Improvement |
|--------|----------|-----------|-------------|
| Success Rate | 80% | 90% | +10% |
| Latency | 300ms | 50ms | -83% |
| Collisions | 0.3/trial | 0.1/trial | -67% |

## 🏗️ Architecture

![Architecture Diagram](docs/architecture.png)

Brief explanation of system components and how they interact.

## 🚀 Quick Start

### Prerequisites
- Ubuntu 20.04
- ROS Noetic / ROS2 Foxy
- Python 3.8+

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
## 🔧 Configuration

Key parameters can be configured in `config/params.yaml`:
```yaml
navigation:
  max_speed: 0.5
  obstacle_threshold: 0.3
```

## 📚 Related Work

This project was developed as part of my Master's thesis at University of Tartu.

**Publications:**
- Socially Aware Planning for Indoor Navigation - [Link]

⭐ If you find this useful, consider giving it a star!

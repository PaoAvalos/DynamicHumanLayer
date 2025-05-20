#ifndef DYNAMIC_HUMAN_LAYER_H
#define DYNAMIC_HUMAN_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <mutex>
#include <string>
#include <map>

#include "dynamic_human_layer/DynamicHumanLayerConfig.h"

namespace dynamic_human_layer {

struct Person {
  std::string id;
  geometry_msgs::Point position;
  geometry_msgs::Vector3 velocity;
  double reliability;
  double radius;
  ros::Time last_seen;
};

class DynamicHumanLayer : public costmap_2d::CostmapLayer, public costmap_2d::Layer {
public:
  DynamicHumanLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void matchSize();

private:
  void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void reconfigureCallback(DynamicHumanLayerConfig &config, uint32_t level);

  std::mutex mutex_;
  ros::Subscriber marker_sub_;
  ros::Subscriber pointcloud_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  dynamic_reconfigure::Server<DynamicHumanLayerConfig> *dsrv_;

  std::map<std::string, Person> people_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_lidar_cloud_;
  std::string global_frame_;
  std::string camera_frame_;
  double robstacle_;
  double alpha_;
  double sigma_max_;
  double sigma_perp_;
  double transform_tolerance_;
  ros::Duration person_timeout_;
};

} // namespace dynamic_human_layer

#endif // DYNAMIC_HUMAN_LAYER_H
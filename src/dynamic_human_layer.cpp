#include <dynamic_human_layer/dynamic_human_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(dynamic_human_layer::DynamicHumanLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace dynamic_human_layer {

DynamicHumanLayer::DynamicHumanLayer()
: tf_listener_(tf_buffer_)
{
  latest_lidar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

void DynamicHumanLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  nh.param<std::string>("global_frame", global_frame_, "map");
  nh.param<std::string>("camera_frame", camera_frame_, "camera_link");
  nh.param("transform_tolerance", transform_tolerance_, 0.3);
  double person_timeout;
  nh.param("person_timeout", person_timeout, 2.0);
  person_timeout_ = ros::Duration(person_timeout);

  dsrv_ = new dynamic_reconfigure::Server<DynamicHumanLayerConfig>(nh);
  dynamic_reconfigure::Server<DynamicHumanLayerConfig>::CallbackType cb =
      boost::bind(&DynamicHumanLayer::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);

  marker_sub_ = nh.subscribe<visualization_msgs::MarkerArray>("/detected_people", 1, &DynamicHumanLayer::markerArrayCallback, this);
  pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/scan_cloud", 1, &DynamicHumanLayer::pointCloudCallback, this);
}

void DynamicHumanLayer::reconfigureCallback(DynamicHumanLayerConfig &config, uint32_t level)
{
  robstacle_ = config.robstacle;
  alpha_ = config.alpha;
  sigma_max_ = config.sigma_max;
  sigma_perp_ = config.sigma_perp;
}

void DynamicHumanLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void DynamicHumanLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                     double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_)
    return;

  ros::Time now = ros::Time::now();
  for (auto it = people_.begin(); it != people_.end();) {
    if (now - it->second.last_seen > person_timeout_) {
      it = people_.erase(it);
    } else {
      ++it;
    }
  }

  double tmp_min_x = std::numeric_limits<double>::max();
  double tmp_min_y = std::numeric_limits<double>::max();
  double tmp_max_x = -std::numeric_limits<double>::max();
  double tmp_max_y = -std::numeric_limits<double>::max();

  for (const auto& pair : people_) {
    const Person& person = pair.second;

    double sigma_parallel = std::min(alpha_ * std::hypot(person.velocity.x, person.velocity.y), sigma_max_);
    double inflation_radius = robstacle_ + 3 * std::max(sigma_parallel, sigma_perp_);

    tmp_min_x = std::min(tmp_min_x, person.position.x - inflation_radius);
    tmp_min_y = std::min(tmp_min_y, person.position.y - inflation_radius);
    tmp_max_x = std::max(tmp_max_x, person.position.x + inflation_radius);
    tmp_max_y = std::max(tmp_max_y, person.position.y + inflation_radius);
  }

  *min_x = std::min(*min_x, tmp_min_x);
  *min_y = std::min(*min_y, tmp_min_y);
  *max_x = std::max(*max_x, tmp_max_x);
  *max_y = std::max(*max_y, tmp_max_y);
}

void DynamicHumanLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_)
    return;

  for (const auto& pair : people_) {
    const Person& person = pair.second;

    double speed = std::hypot(person.velocity.x, person.velocity.y);
    double sigma_parallel = std::min(alpha_ * speed, sigma_max_);
    if (sigma_parallel <= 0) continue;

    double theta = std::atan2(person.velocity.y, person.velocity.x);
    double inflation_radius = robstacle_ + 3 * std::max(sigma_parallel, sigma_perp_);

    double min_x = person.position.x - inflation_radius;
    double min_y = person.position.y - inflation_radius;
    double max_x = person.position.x + inflation_radius;
    double max_y = person.position.y + inflation_radius;

    int start_ix, start_iy, end_ix, end_iy;
    if (!worldToMap(min_x, min_y, start_ix, start_iy) || !worldToMap(max_x, max_y, end_ix, end_iy))
      continue;

    start_ix = std::max(0, start_ix);
    start_iy = std::max(0, start_iy);
    end_ix = std::min(static_cast<int>(getSizeInCellsX()), end_ix);
    end_iy = std::min(static_cast<int>(getSizeInCellsY()), end_iy);

    for (int j = start_iy; j < end_iy; ++j) {
      for (int i = start_ix; i < end_ix; ++i) {
        double wx, wy;
        mapToWorld(i, j, wx, wy);
        double dx = wx - person.position.x;
        double dy = wy - person.position.y;
        double distance = std::hypot(dx, dy);

        if (distance <= robstacle_) {
          master_grid.setCost(i, j, LETHAL_OBSTACLE);
          continue;
        }

        double dx_parallel = dx * std::cos(theta) + dy * std::sin(theta);
        double dx_perp = -dx * std::sin(theta) + dy * std::cos(theta);

        double mahalanobis_sq = (dx_parallel * dx_parallel) / (sigma_parallel * sigma_parallel) +
                                (dx_perp * dx_perp) / (sigma_perp_ * sigma_perp_);
        if (mahalanobis_sq > 9.0) continue;

        double cost_value = std::exp(-0.5 * mahalanobis_sq) * 253;
        unsigned char cost = static_cast<unsigned char>(cost_value);

        if (cost > master_grid.getCost(i, j))
          master_grid.setCost(i, j, cost);
      }
    }
  }
}

void DynamicHumanLayer::markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto& marker : msg->markers) {
    if (marker.points.size() < 2) {
      ROS_WARN("Marker has less than 2 points, skipping.");
      continue;
    }

    geometry_msgs::Point centroid_cam;
    centroid_cam.x = (marker.points[0].x + marker.points[1].x) / 2.0;
    centroid_cam.y = (marker.points[0].y + marker.points[1].y) / 2.0;
    centroid_cam.z = 0.0;

    geometry_msgs::PointStamped centroid_cam_stamped, centroid_map_stamped;
    centroid_cam_stamped.header = marker.header;
    centroid_cam_stamped.point = centroid_cam;

    try {
      tf_buffer_.transform(centroid_cam_stamped, centroid_map_stamped, global_frame_, ros::Duration(transform_tolerance_));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("Failed to transform centroid: %s", ex.what());
      continue;
    }

    double vx = 0.0, vy = 0.0;
    if (!marker.text.empty()) {
      std::vector<std::string> parts;
      boost::split(parts, marker.text, boost::is_any_of(","));
      if (parts.size() >= 2) {
        try {
          vx = std::stod(parts[0]);
          vy = std::stod(parts[1]);
        } catch (const std::exception& e) {
          ROS_WARN("Velocity parse error: %s", marker.text.c_str());
        }
      }
    }

    geometry_msgs::Vector3Stamped velocity_cam, velocity_map;
    velocity_cam.header = marker.header;
    velocity_cam.vector.x = vx;
    velocity_cam.vector.y = vy;

    try {
      tf_buffer_.transform(velocity_cam, velocity_map, global_frame_, ros::Duration(transform_tolerance_));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("Failed to transform velocity: %s", ex.what());
      continue;
    }

    Person person;
    person.id = marker.ns + "_" + std::to_string(marker.id);
    person.position = centroid_map_stamped.point;
    person.velocity.x = velocity_map.vector.x;
    person.velocity.y = velocity_map.vector.y;
    person.reliability = 1.0;
    person.radius = 0.5;
    person.last_seen = ros::Time::now();

    if (latest_lidar_cloud_ && !latest_lidar_cloud_->empty()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      double search_radius = 1.0;

      for (const auto& point : *latest_lidar_cloud_) {
        double dx = point.x - person.position.x;
        double dy = point.y - person.position.y;
        if (std::hypot(dx, dy) <= search_radius) {
          cluster_cloud->push_back(point);
        }
      }

      if (!cluster_cloud->empty()) {
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.2);
        ec.setMinClusterSize(10);
        ec.setInputCloud(cluster_cloud);
        ec.extract(cluster_indices);

        if (!cluster_indices.empty()) {
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*cluster_cloud, cluster_indices[0].indices, centroid);
          double dx = centroid[0] - person.position.x;
          double dy = centroid[1] - person.position.y;
          double distance = std::hypot(dx, dy);

          if (distance > robstacle_) {
            person.position.x += dx * 0.5;
            person.position.y += dy * 0.5;
          }
        }
      }
    }

    people_[person.id] = person;
  }
}

void DynamicHumanLayer::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  try {
    pcl_ros::transformPointCloud(global_frame_, *cloud, *latest_lidar_cloud_, tf_buffer_);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Point cloud transform failed: %s", ex.what());
  }
}

} // namespace dynamic_human_layer
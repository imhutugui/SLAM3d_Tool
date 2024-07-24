#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "scancontext/Scancontext.h"
#include "aloam_velodyne/common.h"

namespace pgo {

std::string padZeros(int val, int num_digits);

std::string getVertexStr(const int _node_idx, const gtsam::Pose3& _Pose);

void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, std::vector<std::string>& edges_str);

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p);

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename);

void initNoises(void);

Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom);

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2);

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& cloudIn, const Pose6D& tf);

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn);

class MapOptimization {
public:
  MapOptimization();
  ~MapOptimization();
  void loadParams(std::string yaml_file);
  void startLoopDetection();
  void stopLoopDetection();
  void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter);
  void saveGTSAMgraphG2oFormat(const gtsam::Values& _estimates);
  void saveOdometryVerticesKITTIformat(std::string _filename);
  void updatePoses(void);
  void runISAM2opt(void);
  void loopFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx);
  std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx);


  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& _laserOdometry);
  void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& _laserCloudFullRes);
  void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& _gps);
protected:
  void process_pg();
  void performSCLoopClosure(void);
  void process_lcd(void);
  void process_icp(void);
  void process_isam(void);

private:
  std::string save_directory;
};

}  // namespace pgo

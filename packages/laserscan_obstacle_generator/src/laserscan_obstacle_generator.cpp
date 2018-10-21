#include <laserscan_obstacle_generator/laserscan_obstacle_generator.h>
#include <visualization_msgs/MarkerArray.h>

#include <limits>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>

LaserscanObstacleGenerator::LaserscanObstacleGenerator(const ros::NodeHandle& pnh):
  nh_(),
  pnh_(pnh),
  minWidth_(0.0),
  maxWidth_(1.0),
  minLength_(0.0),
  maxLength_(1.0),
  obstacle_init_trust_(0.3),
  minBlobElements_(5),
  maxBlobElements_(600),
  blobMaxDistance_(0.1),
  projector_()
{
  // give GDB time to attach
#ifndef DNDEBUG
  ros::Duration(2).sleep();
#endif
  obstacle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacles_out",1);
  collision_angle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("dangerous_angle_markers", 1);
}

LaserscanObstacleGenerator::~LaserscanObstacleGenerator() {
  return;
}

bool LaserscanObstacleGenerator::init() {
  scan_sub_ = nh_.subscribe("scan_in", 10, &LaserscanObstacleGenerator::laserscanCallback, this);

//  std::vector<float> pose_cov_vec, twist_cov_vec;

//  bool ret = true;
//  ret &= pnh_.getParam("obstacleInitTrust", obstacle_init_trust_);
//  ret &= pnh_.getParam("minWidth",          minWidth_);
//  ret &= pnh_.getParam("maxWidth",          maxWidth_);
//  ret &= pnh_.getParam("minLength",         minLength_);
//  ret &= pnh_.getParam("maxLength",         maxLength_);
//  ret &= pnh_.getParam("minBlobElements",   minBlobElements_);
//  ret &= pnh_.getParam("maxBlobElements",   maxBlobElements_);
//  ret &= pnh_.getParam("blobMaxDistance",   blobMaxDistance_);
//  ret &= pnh_.getParam("pose_covariance",   pose_cov_vec);
//  ret &= pnh_.getParam("twist_covariance",  twist_cov_vec);

//  std::copy(pose_cov_vec.begin(), pose_cov_vec.begin() + 36, pose_covariance.begin());
//  std::copy(twist_cov_vec.begin(), twist_cov_vec.begin() + 36, twist_covariance.begin());

//  ROS_ASSERT(36 == pose_cov_vec.size());
//  ROS_ASSERT(36 == twist_cov_vec.size());

  return true;
}

struct Bbox {
  double max_x, min_x, max_y, min_y;
};

void LaserscanObstacleGenerator::laserscanCallback(const sensor_msgs::LaserScanConstPtr& scan_in) {

  sensor_msgs::PointCloud2 incoming_pointcloud;
  projector_.projectLaser(*scan_in, incoming_pointcloud);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(incoming_pointcloud, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*input_cloud);
  /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  /* Here we are creating a vector of PointIndices, which contains the actual index
  * information in a vector<int>. The indices of each detected cluster are saved here.
  * Cluster_indices is a vector containing one instance of PointIndices for each detected
  * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
  */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(blobMaxDistance_);
  ec.setMinClusterSize(minBlobElements_);
  ec.setMaxClusterSize(maxBlobElements_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract(cluster_indices);

  visualization_msgs::MarkerArray obstacles_out;
  visualization_msgs::Marker obstacle;
  obstacle.header.frame_id = scan_in->header.frame_id;
  obstacle.header.stamp = ros::Time::now();
  obstacle.ns = "obstacle_markers";
  obstacle.type = visualization_msgs::Marker::SPHERE;
//  obstacle.action = visualization_msgs::Marker::DELETEALL;
//  obstacles_out.markers.push_back(obstacle);
  // clear all markers on each step
//  obstacle_pub_.publish(obstacles_out);
//  obstacles_out.markers.clear();
  obstacle.action = visualization_msgs::Marker::ADD;
  obstacle.scale.x = 0.1;
  obstacle.scale.y = 0.1;
  obstacle.scale.z = 0.1;
  obstacle.color.g = 1.0;
  obstacle.color.a = 1.0;
  int running_id = 0;
  /* To separate each cluster out of the vector<PointIndices> we have to
  * iterate through cluster_indices, create a new PointCloud for each
  * entry and write all points of the current cluster in the PointCloud.
  */

  // min, max x and y extents
  std::vector<Bbox> cluster_extents;
  Bbox bounding_box;
  for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

    obstacle.id = running_id;
    pcl::CentroidPoint<pcl::PointXYZ> centroid;

    float max_x=FLT_MIN, max_y=FLT_MIN, max_z=FLT_MIN,
          min_x=FLT_MAX, min_y=FLT_MAX, min_z=FLT_MAX;

    for(auto pit = it->indices.begin(); pit != it->indices.end(); pit++) {
      // messages doesnt have a constructor apparently
/*      marker.pose.position.x = input_cloud->points[*pit].x;
      marker.pose.position.y = input_cloud->points[*pit].y;
      marker.pose.position.z = input_cloud->points[*pit].z;
      temp_obstacle.polygon.points.push_back(point);*/

      // get min/max values
      max_x = std::max(max_x, input_cloud->points[*pit].x);
      max_y = std::max(max_y, input_cloud->points[*pit].y);
      max_z = std::max(max_z, input_cloud->points[*pit].z);

      min_x = std::min(min_x, input_cloud->points[*pit].x);
      min_y = std::min(min_y, input_cloud->points[*pit].y);
      min_z = std::min(min_z, input_cloud->points[*pit].z);

      // add point to centroid
      centroid.add(input_cloud->points[*pit]);
      Bbox temp_box;
      temp_box.max_x = max_x;
      temp_box.max_y = max_y;
      temp_box.min_x = min_x;
      temp_box.min_y = min_y;
    }

    // currently no rotation is supported :(
    geometry_msgs::Quaternion q;
    q.x=0; q.y=0; q.z=0; q.w=0;
    obstacle.pose.orientation = q;

//    temp_obstacle.length = std::max(max_x - min_x, float(0.0));
//    temp_obstacle.width =  std::max(max_y - min_y, float(0.0));
//    temp_obstacle.height = std::max(max_z - min_z, float(0.0));

//    if(temp_obstacle.length > maxLength_ ||
//       temp_obstacle.length < minLength_ ||
//       temp_obstacle.width  > maxWidth_  ||
//       temp_obstacle.width  < minWidth_  )
//    {
//        ROS_DEBUG("Rejecting Object.");
//        continue;
//    }


    // get centroid point
    pcl::PointXYZ pcl_centroid_pt;
    centroid.get(pcl_centroid_pt);
    obstacle.pose.position.x = pcl_centroid_pt.x;
    obstacle.pose.position.y = pcl_centroid_pt.y;
    obstacle.pose.position.z = pcl_centroid_pt.z;

//    obstacle.header.stamp = scan_in->header.stamp;
//    obstacle.header.frame_id = scan_in->header.frame_id;

//    // set trust and type
//    temp_obstacle.trust = obstacle_init_trust_;
//    temp_obstacle.obstacle_type = drive_ros_msgs::Obstacle::TYPE_LIDAR;

//    // set covarinaces
//    temp_obstacle.centroid_pose.covariance = pose_covariance;
//    temp_obstacle.centroid_twist.covariance = twist_covariance;

    obstacles_out.markers.push_back(obstacle);
    running_id++;

  }

  ROS_DEBUG_STREAM("Found "<<obstacles_out.markers.size()<<" obstacles in laser scanner pointcloud");
  obstacle_pub_.publish(obstacles_out);

  // check if any of the clusterinos are in front of the robot and steer in the other direction
  std::vector<double> angles;
  for (const Bbox& box: cluster_extents)
  {
    angles.push_back(std::atan2(box.min_x, box.min_y));
    angles.push_back(std::atan2(box.min_x, box.max_y));
    angles.push_back(std::atan2(box.max_x, box.max_y));
    angles.push_back(std::atan2(box.max_x, box.min_y));
  }
  // assume scan is tied to odom as it should be which means the scan is facing with the robot
  obstacles_out.markers.clear();
  for (const double& angle: angles)
  {
    if (angle < 0.3 && angle > 0.3) {
      ROS_WARN_STREAM("ANGLE "<<angle<<" WOULD LEAD TO COLLISION!!!!");
      obstacle.type = visualization_msgs::Marker::ARROW;
      obstacle.pose.position.x = 0.0;
      obstacle.pose.position.y = 0.0;
      obstacle.pose.position.z = 0.0;
      tf::QuaternionFromEuler(0.0, 0.0, angle);
    }
  }
}

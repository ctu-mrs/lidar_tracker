#pragma once
#ifndef ROS_LIDAR_EDGE_DETECT_H
#define ROS_LIDAR_EDGE_DETECT_H

/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* TF2 related ROS includes */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* lidar messages */
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <thread>
#include <unistd.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* uav_detect */
#include <uav_detect/Detection.h>
#include <uav_detect/DetectionStamped.h>
#include <uav_detect/Detections.h>
#include <uav_detect/ProfilingInfo.h>

/* parallelism */
#include <mrs_lib/subscribe_handler.h>

/* some STL includes */
#include <mutex>
#include <deque>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/geometry/cyclic.h>

/* custom Ouster point types */
#include "lidar_tracker/point_types.h"

/* kalman filter*/
#include <mrs_lib/lkf.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <nav_msgs/Odometry.h>

/* dynamic reconfigure */
#include <dynamic_reconfigure/server.h>
#include <lidar_tracker/covmatConfig.h>
#include <lidar_tracker/Tracks.h>

//}

namespace lidar_tracker
{
  const int n_states = 9;
  const int n_inputs = 0;
  const int n_measurements = 3;
  using lkf_t = mrs_lib::LKF<n_states, n_inputs, n_measurements>;
  // Some helpful aliases to make writing of types shorter
  using A_t = lkf_t::A_t;
  using B_t = lkf_t::B_t;
  using H_t = lkf_t::H_t;
  using Q_t = lkf_t::Q_t;
  using x_t = lkf_t::x_t;
  using P_t = lkf_t::P_t;
  using u_t = lkf_t::u_t;
  using z_t = lkf_t::z_t;
  using R_t = lkf_t::R_t;
  using statecov_t = lkf_t::statecov_t;

  using vec3_t = Eigen::Vector3d;
  using mat3_t = Eigen::Matrix3d;

  /* pcl pointcloud */
  using Point = ouster_ros::Point;
  using PointCloud = pcl::PointCloud<Point>;
  using PointXYZ = pcl::PointXYZ;
  using PointCloudXYZ = pcl::PointCloud<PointXYZ>;

  struct track_t
  {
    track_t(const uint32_t id, statecov_t sc0, const ros::Time& stamp, const double confidence, const bool external=false)
      : id(id), sc(std::move(sc0)), last_prediction(stamp), last_correction(stamp), n_corrections(1), confidence(confidence), point_cloud(nullptr), point_cloud_indices(nullptr)
    {
      if (external)
      {
        n_external_detections = 1;
        n_onboard_detections = 0;
        last_onboard_det_stamp = ros::Time(0);
      }
      else
      {
        n_external_detections = 0;
        n_onboard_detections = 1;
        last_onboard_det_stamp = stamp;
      }
    };
    uint32_t id;
    statecov_t sc;
    z_t innovation;
    R_t innovation_cov;
    ros::Time last_onboard_det_stamp; // last time an onboard detection was associated to this track
    ros::Time last_prediction; // last time this track was updated using pointcloud tracking
    ros::Time last_correction; // last time this track was corrected using any means
    uint32_t n_external_detections;
    uint32_t n_onboard_detections;
    uint32_t n_corrections;
    double confidence;

    PointCloud::ConstPtr point_cloud;
    pcl::IndicesConstPtr point_cloud_indices;

    [[nodiscard]] auto gotOnboardDetection() const -> bool
    {
      return n_onboard_detections > 0;
    }

    [[nodiscard]] auto isTrackedOnboard(const ros::Duration& onboard_timeout) const -> bool
    {
      return gotOnboardDetection() && ros::Time::now() - last_onboard_det_stamp < onboard_timeout;
    }
  };

  /* class LidarTracker //{ */
  class LidarTracker : public nodelet::Nodelet
  {

    enum class profile_routines_t
    {
      process_lidar = 1,
      update_track = 2,
      process_detection = 3,
      process_detections = 4,
      process_bg = 5,
    };


  public:
    /* onInit() is called when nodelet is launched (similar to main() in regular node) */
    void onInit() override;

  private:
    /* flags */
    bool is_initialized_ = false;

    std::string _uav_name_;
    uint32_t m_last_track_id = 1;
    static constexpr uint32_t tentative_track_id = 0;

    // | ------------------ ROS-related variables ----------------- |
    mrs_lib::SubscribeHandler<uav_detect::Detections> shandler_detection_;
    mrs_lib::SubscribeHandler<uav_detect::DetectionStamped> shandler_init_detection_;
    mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> shandler_pointcloud_;
    mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> shandler_bg_pointcloud_;

    ros::Publisher pub_lidar_;
    ros::Publisher pub_posearr_;
    ros::Publisher pub_prediction_;
    ros::Publisher pub_tracks_;
    ros::Publisher pub_target_;
    ros::Publisher pub_target_points_;
    ros::Publisher pub_innovation_;
    std::mutex pub_profiling_info_mtx_;
    ros::Publisher pub_profiling_info_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

    // | ---------------------- msg callbacks --------------------- |
    std::unique_ptr<mrs_lib::DynamicReconfigureMgr<lidar_tracker::covmatConfig>> m_drmgr;

    std::thread init_detection_thread_;
    std::thread detection_thread_;
    std::thread pointcloud_thread_;
    std::thread bg_pointcloud_thread_;
    void initDetectionLoop();
    void detectionLoop();
    void pointcloudLoop();
    void bgPointcloudLoop();

    ros::Subscriber sub_drone_;
    void callbackDroneClicked(const geometry_msgs::PointStamped::ConstPtr& ps);
    void loadDynRecConfig();

    void processSingleDetection(const uav_detect::Detection& detection, const std_msgs::Header& header, const Eigen::Affine3d& msg2world_tf, const bool external = false);
    void processInitDetection(const uav_detect::DetectionStamped::ConstPtr& msg);
    void processDetections(const uav_detect::Detections::ConstPtr& msg);
    void processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void processBgPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // | --------------- drone detected variables -------------- |
    std::mutex m_tracking_mtx; // locks the following variables
    std::vector<track_t> m_latest_tracks;
    boost::circular_buffer<std::pair<PointCloud::ConstPtr, Eigen::Affine3d>> m_pc_buffer;
    PointCloudXYZ::Ptr m_bg_pointcloud;
    pcl::search::KdTree<PointXYZ> m_bg_tree;
    bool no_track_pub_empty_;
    statecov_t empty_sc_;

    // | ------------------- vars for clustering ---------------------- |
    float tolerance_;
    int min_cluster_pts_;
    int max_cluster_pts_;
    float max_cluster_size_;
    float min_background_dist_;

    // | --------------- variables for kalman --------------------- |
    double radius_multiplier_;
    double radius_min_;
    double radius_max_;
    int min_onboard_detection_count_;
    int min_external_detection_count_;
    ros::Duration transform_lookup_timeout_;
    ros::Duration throttle_period_;
    ros::Duration tracking_timeout_;
    ros::Duration prediction_horizon_;
    ros::Duration prediction_sampling_period_;

    // | ----------------- pc processing variables ---------------- |
    std::string static_frame_id_;

    float m_exclude_box_offset_x;
    float m_exclude_box_offset_y;
    float m_exclude_box_offset_z;
    float m_exclude_box_size_x;
    float m_exclude_box_size_y;
    float m_exclude_box_size_z;

    void publishUpdatedMessages(const ros::Time& cloud_stamp);

    void publishTracks(const std::vector<track_t>& tracks, const std_msgs::Header& header, const std::vector<track_t>::const_iterator& best_track_it);
    void publishPosearr(const std::vector<track_t>& tracks, const std_msgs::Header& header);
    void publishState(const statecov_t& statecov);
    void publishInnovation(const z_t& inn, const R_t inn_cov);
    void publishPrediction(const statecov_t& statecov_cur, const ros::Duration& horizon, const ros::Duration& sample_period);
    void publishTargetPoints(const PointCloud::ConstPtr& cloud, const pcl::IndicesConstPtr& indices);

    // | --------------------- other functions -------------------- |
    void printMatrices();
    double getRadius(const P_t& P, const bool raw = false);
    void mergeSimilarTracks(std::vector<track_t>& tracks);
    void removeUncertainTracks(std::vector<track_t>& tracks);
    double distance(const x_t& x0, const x_t& x1);
    void updateTrack(track_t& track, const PointCloud::ConstPtr& cloud);
    bool tooUncertain(const statecov_t& sc);

    std::unordered_map<uint32_t, uint64_t> m_profile_last_seq;
    void publish_profile_start(const profile_routines_t routine_id);
    void publish_profile_end(const profile_routines_t routine_id);
    void publish_profile_event(const uint32_t routine_id, const uint8_t type);

    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
    std::optional<Eigen::Affine3d> getTransformToWorld(const std::string& frame_id, const ros::Time& stamp) const;

    double R_coeff_;
    Q_t Q_;
    P_t P0_;
    lkf_t lkf;
  };
  //}

}  // namespace lidar_tracker
#endif

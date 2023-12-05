#include "lidar_tracker/LidarTracker.h"

#include <nav_msgs/Path.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
#include <pcl/filters/extract_indices.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <optional>
#include <ros/duration.h>
#include <algorithm>
#include <limits>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pluginlib/class_list_macros.h>

#include <boost/type_index.hpp>

namespace lidar_tracker
{
  /* onInit() method //{ */
  void LidarTracker::onInit()
  {
    /* obtain node handle */
    ros::NodeHandle nh("~");
  
    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();
  
    // | ------------------- load ros parameters ------------------ |
  
    mrs_lib::ParamLoader param_loader(nh, "LidarTracker");
  
    param_loader.loadParam("static_frame_id", static_frame_id_);
    param_loader.loadParam("min_onboard_detection_count", min_onboard_detection_count_);
    param_loader.loadParam("transform_lookup_timeout", transform_lookup_timeout_);
    param_loader.loadParam("message_throttle_period", throttle_period_);
    param_loader.loadParam("no_track/publish_empty", no_track_pub_empty_);
    if (no_track_pub_empty_)
    {
      param_loader.loadMatrixStatic("no_track/empty_state", empty_sc_.x);
      const auto emtpy_covariance_value = param_loader.loadParam2<double>("no_track/empty_covariance_value");
      empty_sc_.P.fill(emtpy_covariance_value);
    }
  
    param_loader.loadParam("prediction/horizon", prediction_horizon_);
    param_loader.loadParam("prediction/sampling_period", prediction_sampling_period_);
  
    param_loader.loadParam("lkf/P/radius/multiplier", radius_multiplier_);
    param_loader.loadParam("lkf/P/radius/min", radius_min_);
    param_loader.loadParam("lkf/P/radius/max", radius_max_);
  
    param_loader.loadParam("association/clustering_tolerance", tolerance_);
    param_loader.loadParam("association/cluster/min_points", min_cluster_pts_);
    param_loader.loadParam("association/cluster/max_points", max_cluster_pts_);
    param_loader.loadParam("association/cluster/max_size", max_cluster_size_);
    param_loader.loadParam("association/cluster/min_background_dist", min_background_dist_);
  
    param_loader.loadParam("input_filter/exclude_box/offset/x", m_exclude_box_offset_x);
    param_loader.loadParam("input_filter/exclude_box/offset/y", m_exclude_box_offset_y);
    param_loader.loadParam("input_filter/exclude_box/offset/z", m_exclude_box_offset_z);
    param_loader.loadParam("input_filter/exclude_box/size/x", m_exclude_box_size_x);
    param_loader.loadParam("input_filter/exclude_box/size/y", m_exclude_box_size_y);
    param_loader.loadParam("input_filter/exclude_box/size/z", m_exclude_box_size_z);
    m_exclude_box_offset_z = m_exclude_box_offset_z + m_exclude_box_size_z / 2.0f;
  
    const int buffer_length = param_loader.loadParam2<int>("buffer_length");
  
    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR_STREAM("[LidarTracker]: failed to load non-optional parameters!");
      ros::shutdown();
      return;
    }
  
    // | ------------ initialize the pointcloud buffer ------------ |
    m_pc_buffer.set_capacity(buffer_length);
    m_bg_pointcloud = boost::make_shared<PointCloudXYZ>();
  
    // | ----------------- initialize subscribers ----------------- |
    mrs_lib::SubscribeHandlerOptions shopts(nh);
    shopts.no_message_timeout = ros::Duration(5.0);
    mrs_lib::construct_object(shandler_detection_, shopts, "detections_in");
    mrs_lib::construct_object(shandler_pointcloud_, shopts, "points_in");
    mrs_lib::construct_object(shandler_bg_pointcloud_, shopts, "bg_points_in");
    sub_drone_ = nh.subscribe("/clicked_point", 1, &LidarTracker::callbackDroneClicked, this);
    m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer);
  
    detection_thread_ = std::thread(&LidarTracker::detectionLoop, this);
    pointcloud_thread_ = std::thread(&LidarTracker::pointcloudLoop, this);
    bg_pointcloud_thread_ = std::thread(&LidarTracker::bgPointcloudLoop, this);
  
    // dynamic reconfigure
    m_drmgr = std::make_unique<mrs_lib::DynamicReconfigureMgr<lidar_tracker::covmatConfig>>(nh, "LidarTracker");
  
    // | ------------------ initialize publishers ----------------- |
    pub_lidar_ = nh.advertise<PointCloud>("filtered_points", 10);
    pub_tracks_ = nh.advertise<lidar_tracker::Tracks>("tracks", 10);
    pub_posearr_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("dbg_tracks", 10);
    pub_prediction_ = nh.advertise<nav_msgs::Path>("tracked_drone_prediction", 10);
    pub_target_ = nh.advertise<nav_msgs::Odometry>("tracked_drone", 10);
    pub_target_points_ = nh.advertise<PointCloud>("tracked_drone_points", 10);
    pub_innovation_ = nh.advertise<nav_msgs::Odometry>("tracked_drone_innovation", 10);
    pub_profiling_info_ = nh.advertise<vofod::ProfilingInfo>("profiling_info", 1, true);
  
    /* kalman */
    // only position is measured
    loadDynRecConfig();
    // the first two parameters are matrices A and B
    // A is set based on the dt before the prediction step
    // B is not used in this specific system model
    lkf = lkf_t({}, {}, H_t::Identity());
    ROS_INFO_STREAM_ONCE("[LidarTracker]: initialized");
    printMatrices();
  
    is_initialized_ = true;
  }
  //}

  A_t getA(double dt)
  {
    // the first three states are position, the next three states are velocity and the last three are acceleration
    A_t ret = A_t::Identity();
    // fill the position states
    ret.block<3, 3>(0, 3) = dt*mat3_t::Identity();
    ret.block<3, 3>(0, 6) = 0.5*dt*dt*mat3_t::Identity();
    // fill the velocity states
    ret.block<3, 3>(3, 6) = dt*mat3_t::Identity();
    return ret;
  }

  void LidarTracker::printMatrices()
  {
    ROS_INFO_STREAM("[LidarTracker]: LKF matrices are:\nA =\n" << getA(0.1) << "\nH =\n" << lkf.H << "\nQ =\n" << Q_ << "\nP0 =\n" << P0_ << "\nR =\n" << R_coeff_*R_t::Identity());
  }

  void LidarTracker::loadDynRecConfig()
  {
    // make a copy for thread "safety"
    const auto cfg = m_drmgr->config;
    const auto& Q_pos = cfg.lkf__Q__position;
    const auto& Q_vel = cfg.lkf__Q__velocity;
    const auto& Q_acc = cfg.lkf__Q__acceleration;
    Q_ = Q_t::Zero();
    Q_.diagonal() << Q_pos, Q_pos, Q_pos, Q_vel, Q_vel, Q_vel, Q_acc, Q_acc, Q_acc;

    const auto& P_pos = cfg.lkf__P__init__position;
    const auto& P_vel = cfg.lkf__P__init__velocity;
    const auto& P_acc = cfg.lkf__P__init__acceleration;
    P0_ = P_t::Zero();
    P0_.diagonal() << P_pos, P_pos, P_pos, P_vel, P_vel, P_vel, P_acc, P_acc, P_acc;

    R_coeff_ = cfg.lkf__R__coeff;
  }

  void LidarTracker::processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    publish_profile_start(profile_routines_t::process_lidar);
    mrs_lib::ScopeTimer tim("new pc", throttle_period_);

    PointCloud::Ptr cloud_filtered = boost::make_shared<PointCloud>();
    pcl::fromROSMsg(*msg, *cloud_filtered);
    ros::Time cloud_stamp;
    pcl_conversions::fromPCL(cloud_filtered->header.stamp, cloud_stamp);

    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.5f, 0.5f, 0.5f);
    vg.filter(*cloud_filtered);

    const ros::Time& msg_stamp = msg->header.stamp;

    // crop points inside a box, relative to the sensor (the carrier itself)
    {
      const Eigen::Vector4f box_point1(m_exclude_box_offset_x + m_exclude_box_size_x / 2, m_exclude_box_offset_y + m_exclude_box_size_y / 2,
                                       m_exclude_box_offset_z + m_exclude_box_size_z / 2, 1);
      const Eigen::Vector4f box_point2(m_exclude_box_offset_x - m_exclude_box_size_x / 2, m_exclude_box_offset_y - m_exclude_box_size_y / 2,
                                       m_exclude_box_offset_z - m_exclude_box_size_z / 2, 1);
      pcl::CropBox<Point> cb;
      cb.setMax(box_point1);
      cb.setMin(box_point2);
      cb.setInputCloud(cloud_filtered);
      cb.setNegative(true);
      cb.filter(*cloud_filtered);
    }

    // transform the cloud to the static coordinate frame
    Eigen::Affine3d tf;
    {
      const auto tf_opt = getTransformToWorld(msg->header.frame_id, msg_stamp);
      tim.checkpoint("tf lookup");
      if (!tf_opt.has_value())
      {
        NODELET_ERROR_THROTTLE(1.0, "[processLidar]: Could not transform cloud to the static coordinate frame, skipping.");
        return;
      }
      tf = tf_opt.value();
      pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, tf.cast<float>());
      cloud_filtered->header.frame_id = static_frame_id_;
    }

    // ensure that the pointcloud buffer and tracking state are not updated while the new detections are being propagated
    std::scoped_lock tracking_lck(m_tracking_mtx);
    loadDynRecConfig();
    tim.checkpoint("mtx lock");

    const Eigen::Affine3d world2sensor_tf = tf.inverse();
    m_pc_buffer.push_back({cloud_filtered, world2sensor_tf});
    for (auto& track : m_latest_tracks)
      updateTrack(track, cloud_filtered);
    const size_t n_tracks_orig = m_latest_tracks.size();

    mergeSimilarTracks(m_latest_tracks);
    const size_t n_tracks_merged = n_tracks_orig - m_latest_tracks.size();
    removeUncertainTracks(m_latest_tracks);
    const size_t n_tracks_removed = n_tracks_orig - n_tracks_merged - m_latest_tracks.size();

    NODELET_INFO_THROTTLE(1.0, "[processLidar]: Updated %lu tracks, merged %lu tracks, removed %lu uncertain tracks, %lu tracks remain.", n_tracks_orig, n_tracks_merged, n_tracks_removed, m_latest_tracks.size());

    publishUpdatedMessages(cloud_stamp);
    pub_lidar_.publish(cloud_filtered);
    publish_profile_end(profile_routines_t::process_lidar);
  }
  //}

  void LidarTracker::processBgPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    publish_profile_start(profile_routines_t::process_bg);
    mrs_lib::ScopeTimer tim("bg pc", throttle_period_);

    PointCloudXYZ::Ptr bg_pointcloud = boost::make_shared<PointCloudXYZ>();
    pcl::fromROSMsg(*shandler_bg_pointcloud_.getMsg(), *bg_pointcloud);
    const auto tf_opt = getTransformToWorld(msg->header.frame_id, msg->header.stamp);
    tim.checkpoint("tf lookup bg");

    if (!tf_opt.has_value())
    {
      NODELET_WARN_THROTTLE(1.0, "[processBgPointcloud]: Could not transform background cloud to the static coordinate frame, skipping.");
    } else
    {
      const auto tf = tf_opt.value();
      std::scoped_lock tracking_lck(m_tracking_mtx);
      loadDynRecConfig();
      tim.checkpoint("mtx lock");

      pcl::transformPointCloud(*bg_pointcloud, *bg_pointcloud, tf);
      m_bg_pointcloud = bg_pointcloud;
      if (m_bg_pointcloud->empty())
        m_bg_tree = {};  // clear the tree if the pointcloud is empty
      else
        m_bg_tree.setInputCloud(m_bg_pointcloud);
    }
    publish_profile_end(profile_routines_t::process_bg);
  }

  /* publishUpdatedMessages() method //{ */
  void LidarTracker::publishUpdatedMessages(const ros::Time& cloud_stamp)
  {
    // find the best track (with most detections) and publish it if applicable
    const auto best_track_it = std::max_element(std::begin(m_latest_tracks), std::end(m_latest_tracks),
                                                [](const auto& el1, const auto& el2)
                                                {
                                                  return el1.n_onboard_detections < el2.n_onboard_detections;
                                                });
    const bool best_track_exists = best_track_it != std::end(m_latest_tracks);
    const bool best_track_confirmed = best_track_exists && best_track_it->n_onboard_detections >= (uint32_t)min_onboard_detection_count_;
    const bool best_track_updated_now = best_track_exists && best_track_it->last_correction == cloud_stamp;
    if (best_track_exists && best_track_confirmed && best_track_updated_now)
    {
      publishState(best_track_it->sc);
      publishInnovation(best_track_it->innovation, best_track_it->innovation_cov);
      publishPrediction(best_track_it->sc, prediction_horizon_, prediction_sampling_period_);
      publishTargetPoints(best_track_it->point_cloud, best_track_it->point_cloud_indices);
    }
    else if (no_track_pub_empty_)
    {
      publishState(empty_sc_);
      publishInnovation(z_t::Zero(), R_t::Zero());
      publishPrediction(empty_sc_, prediction_horizon_, prediction_sampling_period_);
      publishTargetPoints(nullptr, nullptr);
    }
    std_msgs::Header header;
    header.stamp = cloud_stamp;
    header.frame_id = static_frame_id_;
    publishTracks(m_latest_tracks, header, best_track_it);
    publishPosearr(m_latest_tracks, header);
  }
  //}

  /* updateTrack() method //{ */
  void LidarTracker::updateTrack(track_t& track, const PointCloud::ConstPtr& cloud)
  {
    publish_profile_start(profile_routines_t::update_track);
    // predict the current track's position using LKF
    ros::Time cloud_stamp;
    pcl_conversions::fromPCL(cloud->header.stamp, cloud_stamp);
    const double dt = (cloud_stamp - track.last_prediction).toSec();
    lkf.A = getA(dt);  // don't forget to update A according to the current dt
    track.sc = lkf.predict(track.sc, {}, Q_, dt);
    track.last_prediction = cloud_stamp;

    // if the input cloud is empty, we can do nothing else, end here
    if (cloud->empty())
      return;

    // get the radius of the track, corresponding to its uncertainty
    const double radius = getRadius(track.sc.P);
    // find indices of all points within radius from the current track's position
    pcl::search::KdTree<Point>::Ptr tree = boost::make_shared<pcl::search::KdTree<Point>>();
    const pcl::IndicesConstPtr indices_within_radius = [&tree, &track, &cloud, radius]() {
      pcl::IndicesPtr ret = boost::make_shared<pcl::Indices>();
      std::vector<float> sqr_distances;
      Point p;
      p.getVector3fMap() = track.sc.x.head<3>().cast<float>();
      tree->setInputCloud(cloud);
      tree->radiusSearch(p, radius, *ret, sqr_distances);
      return ret;
    }();

    // extract points within radius from the current track's position
    const PointCloud::ConstPtr radius_cloud = [&cloud, &indices_within_radius]() {
      PointCloud::Ptr ret = boost::make_shared<PointCloud>();
      pcl::ExtractIndices<Point> ei;
      ei.setInputCloud(cloud);
      ei.setIndices(indices_within_radius);
      ei.filter(*ret);
      return ret;
    }();

    // find indices of point clusters within the radius
    const std::vector<pcl::PointIndices> cluster_indices = [this, &tree, &radius_cloud]() {
      std::vector<pcl::PointIndices> ret;
      pcl::EuclideanClusterExtraction<Point> ec;
      ec.setClusterTolerance(tolerance_);
      ec.setMinClusterSize(min_cluster_pts_);
      ec.setMaxClusterSize(max_cluster_pts_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(radius_cloud);
      ec.extract(ret);
      return ret;
    }();

    // find the closest cluster to the track with valid dimensions
    // that is not too close to the background pointcloud

    // prepare some variables
    const Eigen::Vector3f track_pos = track.sc.x.block<3, 1>(0, 0).cast<float>();
    struct closest_cluster_t
    {
      Eigen::Vector3f obb_center = std::numeric_limits<float>::quiet_NaN() * Eigen::Vector3f::Ones();
      float dist = std::numeric_limits<float>::max();
      struct obb_t
      {
        Eigen::Vector3f center_pt, min_pt, max_pt;
        Eigen::Matrix3f rotation;
      } obb;
      bool valid = false;
      pcl::IndicesConstPtr indices;
    } closest_cluster;

    pcl::MomentOfInertiaEstimation<Point> moie;
    moie.setInputCloud(radius_cloud);
    // go through all the clusters and evaluate them
    for (auto&& cluster : cluster_indices)
    {
      // move the cluster indices since we won't be needing them after this forloop is done
      const pcl::IndicesConstPtr tmp_inds = boost::make_shared<pcl::Indices>(std::move(cluster.indices));
      moie.setIndices(tmp_inds);
      moie.compute();
      Point min_pt, max_pt, center_pt;
      Eigen::Matrix3f obb_rotation;
      moie.getOBB(min_pt, max_pt, center_pt, obb_rotation);

      const Eigen::Vector3f obb_min = min_pt.getVector3fMap();
      const Eigen::Vector3f obb_max = max_pt.getVector3fMap();
      const Eigen::Vector3f obb_sz = obb_max - obb_min;
      const float obb_diagonal = obb_sz.norm();
      // skip too large clusters
      if (obb_diagonal > max_cluster_size_)
        continue;

      // only apply this step if the background pointcloud is available
      if (m_bg_tree.getInputCloud())
      {
        PointXYZ pt_xyz(center_pt.x, center_pt.y, center_pt.z);
        pcl::IndicesPtr tmp = boost::make_shared<pcl::Indices>();
        std::vector<float> sqr_distances;
        const int n_bg_pts = m_bg_tree.radiusSearch(pt_xyz, obb_diagonal + min_background_dist_, *tmp, sqr_distances);
        // skip clusters that are too close to the background pointcloud
        if (n_bg_pts > 0)
          continue;
      } else
      {
        NODELET_WARN_STREAM_THROTTLE(1.0, "[updateTrack]: Background pointcloud unavailable, cannot use it for cluster filtering.");
      }

      const Eigen::Vector3f obb_ctr = center_pt.getVector3fMap();
      const float dist = (obb_ctr - track_pos).norm();
      if (!closest_cluster.valid || dist < closest_cluster.dist)
        // move the tmp indices since we won't be needing them after this forloop is done
        closest_cluster = closest_cluster_t{obb_ctr, dist, {obb_ctr, obb_min, obb_max, obb_rotation}, true, tmp_inds};
    }

    if (closest_cluster.valid)
    {
      const z_t z = closest_cluster.obb_center.cast<double>();
      const R_t R = R_coeff_*R_t::Identity();
      track.innovation = z - lkf.H*track.sc.x;
      track.innovation_cov = lkf.H*track.sc.P*lkf.H.transpose() + R;
      track.sc = lkf.correct(track.sc, z, R);
      track.last_correction = cloud_stamp;
      track.n_corrections++;
      track.point_cloud = radius_cloud;
      track.point_cloud_indices = closest_cluster.indices;
      NODELET_INFO_STREAM_THROTTLE(1.0, "[updateTrack]: \033[1;32mTrack #" << track.id << " corrected from pointcloud.\033[0m");
    }
    publish_profile_end(profile_routines_t::update_track);
  }
  //}

  /* removeUncertainTracks() method //{ */

  void LidarTracker::removeUncertainTracks(std::vector<track_t>& tracks)
  {
    for (auto track_it = std::begin(tracks); track_it != std::end(tracks); track_it++)
    {
      if (tooUncertain(track_it->sc))
      {
        NODELET_INFO_STREAM("[removeUncertainTracks]: Removing track #" << track_it->id << " with uncertainty radius " << getRadius(track_it->sc.P)
                                                                        << "m larger than " << radius_max_ << "m");
        track_it = tracks.erase(track_it);
        track_it--;
      }
    }
  }

  //}

  /* mergeSimilarTracks() method //{ */
  void LidarTracker::mergeSimilarTracks(std::vector<track_t>& tracks)
  {
    for (auto track_it1 = std::begin(tracks); track_it1 != std::end(tracks); track_it1++)
    {
      for (auto track_it2 = track_it1 + 1; track_it2 != std::end(tracks); track_it2++)
      {
        // this has to be in the inner loop as well because the tracks may be swapped during merging
        const auto& sc1 = track_it1->sc;
        const double radius1 = getRadius(sc1.P, true);

        const auto& sc2 = track_it2->sc;
        const double radius2 = getRadius(sc2.P, true);

        const double dist = distance(sc1.x, sc2.x);
        if (dist < radius1 + radius2)
        {  // if these two tracks have converged, delete the less confident one
          // if the track 2 is more confident than track 1, first swap them before removing
          if (radius2 < radius1)
            std::swap(*track_it1, *track_it2);
          NODELET_INFO_STREAM("[mergeSimilarTracks]: Removing track #" << track_it2->id << " that is too close to track #" << track_it1->id << " (they are "
                                                                       << dist << "m apart)");
          track_it2 = tracks.erase(track_it2);
          track_it2--;
        }
      }
    }
  }
  //}

  double LidarTracker::distance(const x_t& x0, const x_t& x1)
  {
    return (x0.head<3>() - x1.head<3>()).norm();
  }

  double LidarTracker::getRadius(const P_t& P, const bool raw)
  {
    /* return radius_scalar_ * std::max(P(0, 0), std::max(P(1, 1), P(2, 2))); */
    // volume of the uncertainty ellipsoid is proportional to the determinant of the covariance matrix
    // radius of an ellipsoid is proportional to the cube root of its volume
    const double volume = P.block<3, 3>(0, 0).determinant();
    if (raw)
      return radius_multiplier_ * std::cbrt(volume);
    else
      return std::max(radius_multiplier_ * std::cbrt(volume), radius_min_);
  }

  /* publishTracks() method //{ */
  void LidarTracker::publishTracks(const std::vector<track_t>& tracks, const std_msgs::Header& header, const std::vector<track_t>::const_iterator& best_track_it)
  {
    lidar_tracker::Tracks::Ptr ret = boost::make_shared<lidar_tracker::Tracks>();
    ret->header = header;
    ret->tracks.reserve(tracks.size());
    for (auto it = std::cbegin(tracks); it != std::cend(tracks); ++it)
    {
      const auto& track = *it;
      lidar_tracker::Track msg;
      msg.id = track.id;
      msg.n_detections = track.n_onboard_detections;
      msg.n_clusters = track.n_corrections;
      msg.confidence = track.confidence;
      msg.last_prediction = track.last_prediction;
      msg.last_correction = track.last_correction;
      msg.last_detection = track.last_onboard_det_stamp;
      msg.selected = it == best_track_it;

      msg.position.x = track.sc.x.x();
      msg.position.y = track.sc.x.y();
      msg.position.z = track.sc.x.z();

      const auto N_DIMS = 3;
      const auto VEL_OFFSET = N_DIMS;
      if (n_states >= N_DIMS + VEL_OFFSET)
      {
        const vec3_t& velocity = track.sc.x.segment<N_DIMS>(VEL_OFFSET);
        msg.velocity.x = velocity.x();
        msg.velocity.y = velocity.y();
        msg.velocity.z = velocity.z();
      }

      const auto ACC_OFFSET = VEL_OFFSET + N_DIMS;
      if (n_states >= N_DIMS + ACC_OFFSET)
      {
        const vec3_t& acceleration = track.sc.x.segment<N_DIMS>(ACC_OFFSET);
        msg.acceleration.x = acceleration.x();
        msg.acceleration.y = acceleration.y();
        msg.acceleration.z = acceleration.z();
      }

      if (track.point_cloud != nullptr && track.point_cloud_indices != nullptr)
      {
        PointCloud track_points;
        pcl::ExtractIndices<Point> ei;
        ei.setInputCloud(track.point_cloud);
        ei.setIndices(track.point_cloud_indices);
        ei.filter(track_points);
        track_points.header = track.point_cloud->header;
        pcl::toROSMsg(std::move(track_points), msg.points);
      }

      const auto N_MSG_STATES = 9;
      for (int r = 0; r < n_states; r++)
        for (int c = 0; c < n_states; c++)
          msg.covariance.at(N_MSG_STATES * r + c) = track.sc.P(r, c);

      ret->tracks.push_back(msg);
    }
    pub_tracks_.publish(ret);
  }
  //}

  vec3_t msg_to_position(const vofod::Detection& det, const Eigen::Affine3d& tf)
  {
    return tf * vec3_t(det.position.x, det.position.y, det.position.z);
  }

  mat3_t msg_to_covariance(const vofod::Detection& det, const Eigen::Affine3d& tf)
  {
    mat3_t ret;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        ret(r, c) = det.covariance.at(3 * r + c);
    ret = tf.rotation() * ret * tf.rotation().transpose();
    return ret;
  }

  template <size_t n>
  void covariance_to_msg(const mat3_t& cov, boost::array<double, n>& msg_cov_out)
  {
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        msg_cov_out.at(3 * r + c) = cov(r, c);
  }

  geometry_msgs::Pose pose_to_msg(const vec3_t& position, const vec3_t& velocity)
  {
    geometry_msgs::Pose msg;
    msg.position.x = position.x();
    msg.position.y = position.y();
    msg.position.z = position.z();

    auto eigen_quat = Eigen::Quaterniond::FromTwoVectors(vec3_t::UnitX(), velocity);
    msg.orientation.x = eigen_quat.x();
    msg.orientation.y = eigen_quat.y();
    msg.orientation.z = eigen_quat.z();
    msg.orientation.w = eigen_quat.w();
    return msg;
  }

  /* publishPosearr() method //{ */
  void LidarTracker::publishPosearr(const std::vector<track_t>& tracks, const std_msgs::Header& header)
  {
    mrs_msgs::PoseWithCovarianceArrayStamped msg;
    msg.header = header;

    for (const auto& track : tracks)
    {
      const vec3_t pos = track.sc.x.head<3>();
      const vec3_t vel = track.sc.x.segment<3>(3);
      const mat3_t pos_cov = track.sc.P.block<3, 3>(0, 0);

      mrs_msgs::PoseWithCovarianceIdentified pose;
      pose.id = track.id;

      pose.pose = pose_to_msg(pos, vel);
      covariance_to_msg(pos_cov, pose.covariance);
      msg.poses.push_back(pose);
    }
    pub_posearr_.publish(msg);
  }
  //}

  /* tooUncertain() method //{ */
  bool LidarTracker::tooUncertain(const statecov_t& sc)
  {
    return getRadius(sc.P) > radius_max_ || !sc.P.allFinite();
  }
  //}

  /* processSingleDetection() method //{ */
  void LidarTracker::processSingleDetection(const vofod::Detection& detection, const std_msgs::Header& header, const Eigen::Affine3d& msg2world_tf)
  {
    publish_profile_start(profile_routines_t::process_detection);
    const vec3_t pos = msg_to_position(detection, msg2world_tf);
    const mat3_t pos_cov = msg_to_covariance(detection, msg2world_tf);
    x_t x0 = x_t::Zero();
    x0.head<3>() = pos;
    P_t P0 = P0_;
    P0.block<3, 3>(0, 0) = pos_cov;
    const statecov_t sc0{x0, P0};

    const auto INIT_CONFIDENCE = 0.5;
    track_t cur_track(tentative_track_id, sc0, header.stamp, INIT_CONFIDENCE);
    bool lost_track = false;
    // update the track to the latest received pointcloud
    for (const auto& [cloud, world2sensor_tf] : m_pc_buffer)
    {
      ros::Time cloud_stamp;
      pcl_conversions::fromPCL(cloud->header.stamp, cloud_stamp);
      // update the track if applicable
      if (cloud_stamp >= header.stamp)
        updateTrack(cur_track, cloud);
      // remove the track if it became too uncertain
      if (tooUncertain(cur_track.sc))
      {
        break;
        lost_track = true;
      }
    }

    // if this track got lost during propagation through the pc buffer, there is no salvation for it, just skip
    if (lost_track)
    {
      NODELET_INFO_STREAM("[ProcessSingleDetection]: Track of detection #" << detection.id << " was lost!");
      return;
    }

    // try to associate this track with one of the existing tracks
    const double cur_uncertainty_radius = getRadius(cur_track.sc.P);
    // find the closest existing track to this one (within the sum of their uncertainty radii)
    auto closest_track_it = std::end(m_latest_tracks);
    double closest_track_dist = std::numeric_limits<double>::max();
    for (auto track_it = std::begin(m_latest_tracks); track_it != std::end(m_latest_tracks); track_it++)
    {
      const statecov_t& sc = track_it->sc;
      const double association_radius = getRadius(sc.P) + cur_uncertainty_radius;
      const double cur_dist = distance(sc.x, cur_track.sc.x);
      if (cur_dist < association_radius && cur_dist < closest_track_dist)
      {
        closest_track_it = track_it;
        closest_track_dist = cur_dist;
      }
    }

    if (closest_track_it == std::end(m_latest_tracks))
    { // if no association was made, start a new track
      // assign the track a new ID instead of the tentative ID
      cur_track.id = m_last_track_id++;
      m_latest_tracks.emplace_back(cur_track);
      NODELET_INFO_STREAM("[ProcessSingleDetection]: New track # " << m_last_track_id - 1 << " was initialized from detection #" << detection.id);
      publishUpdatedMessages(cur_track.last_correction);
    } else
    { // otherwise if an association was found, just increase its number of detections
      closest_track_it->n_onboard_detections++;
      closest_track_it->last_onboard_det_stamp = header.stamp;
      NODELET_INFO_STREAM_THROTTLE(1.0, "[ProcessSingleDetection]: \033[1;32mA detection #" << detection.id << " was associated to track #" << closest_track_it->id << " with distance "
                                                                    << closest_track_dist << "m\033[0m");
      publishUpdatedMessages(closest_track_it->last_correction);
    }
    publish_profile_end(profile_routines_t::process_detection);
  }
  //}

  /* processDetections() method //{ */
  void LidarTracker::processDetections(const vofod::Detections::ConstPtr& msg)
  {
    publish_profile_start(profile_routines_t::process_detections);
    mrs_lib::ScopeTimer tim("new det", throttle_period_);
    if (msg->detections.empty())
    {
      NODELET_INFO_STREAM_THROTTLE(1.0, "[ProcessDetections]: Detection message empty, skipping.");
      return;
    }

    // ensure that the pointcloud buffer and tracking state are not updated while the new detections are being propagated
    std::scoped_lock tracking_lck(m_tracking_mtx);
    loadDynRecConfig();
    NODELET_INFO_STREAM_THROTTLE(1.0, "[ProcessDetections]: Processing " << msg->detections.size() << " new detections.");
    tim.checkpoint("mtx lock");

    const auto tf_opt = getTransformToWorld(msg->header.frame_id, msg->header.stamp);
    if (!tf_opt.has_value())
    {
      NODELET_ERROR_THROTTLE(1.0, "[ProcessDetections]: Could not find transformation of message to the static coordinate frame, skipping.");
      return;
    }
    const Eigen::Affine3d tf = tf_opt.value();
    tim.checkpoint("tf lookup");

    // go through all detections and propagate them through the pointcloud buffer
    for (const auto& detection : msg->detections)
      processSingleDetection(detection, msg->header, tf);
    publish_profile_end(profile_routines_t::process_detections);
  }
  //}

  void LidarTracker::detectionLoop()
  {
    const ros::WallDuration timeout(0.1);
    while (ros::ok())
    {
      const auto msg_ptr = shandler_detection_.waitForNew(timeout);
      if (msg_ptr)
        processDetections(msg_ptr);
    }
  }

  void LidarTracker::pointcloudLoop()
  {
    const ros::WallDuration timeout(0.1);
    while (ros::ok())
    {
      const auto msg_ptr = shandler_pointcloud_.waitForNew(timeout);
      if (msg_ptr)
        processLidar(msg_ptr);
    }
  }

  void LidarTracker::bgPointcloudLoop()
  {
    const ros::WallDuration timeout(0.1);
    while (ros::ok())
    {
      const auto msg_ptr = shandler_bg_pointcloud_.waitForNew(timeout);
      if (msg_ptr)
        processBgPointcloud(msg_ptr);
      timeout.sleep();  // this thread doesn't have to be run so often
    }
  }

  /* publishState() method //{ */
  void LidarTracker::publishState(const statecov_t& statecov)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = static_frame_id_;
    const x_t state = statecov.x;
    if (n_states >= 6)
      odom.pose.pose = pose_to_msg(state.head<3>(), state.segment<3>(3));
    else
      odom.pose.pose = pose_to_msg(state.head<3>(), vec3_t::UnitX());

    if (n_states >= 6)
    {
      odom.twist.twist.linear.x = state(3);
      odom.twist.twist.linear.y = state(4);
      odom.twist.twist.linear.z = state(5);
    }

    const int pose_cov_dim = 6;
    const int n = std::min(pose_cov_dim, n_states);
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        odom.pose.covariance.at(pose_cov_dim * i + j) = statecov.P(i, j);

    odom.child_frame_id = "tracked_target";
    pub_target_.publish(odom);
  }
  //}

  /* publishInnovation() method //{ */
  void LidarTracker::publishInnovation(const z_t& inn, const R_t inn_cov)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = static_frame_id_;
    odom.pose.pose = pose_to_msg(inn, vec3_t::UnitX());

    const int n = 3;
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        odom.pose.covariance.elems[6 * i + j] = inn_cov(i, j);
    odom.child_frame_id = "tracked_target";
    pub_innovation_.publish(odom);
  }
  //}

  /* publishPrediction() method //{ */
  void LidarTracker::publishPrediction(const statecov_t& statecov_cur, const ros::Duration& horizon, const ros::Duration& sample_period)
  {
    nav_msgs::Path pred;
    pred.header.stamp = ros::Time::now();
    pred.header.frame_id = static_frame_id_;

    const double dt = sample_period.toSec();
    const int n_steps = horizon.toSec() / dt + 1;

    statecov_t statecov = statecov_cur;
    lkf.A = getA(dt);
    for (int it = 0; it < n_steps; it++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = pred.header;
      pose.header.stamp += ros::Duration(it*dt);
      pose.pose = pose_to_msg(statecov.x.head<3>(), statecov.x.segment<3>(3));
      pred.poses.push_back(pose);
      statecov = lkf.predict(statecov, u_t::Zero(), Q_, dt);
    }
    pub_prediction_.publish(pred);
  }
  //}

  /* publishTargetPoints() method //{ */
  void LidarTracker::publishTargetPoints(const PointCloud::ConstPtr& cloud, const pcl::IndicesConstPtr& indices)
  {
    PointCloud points;
    if (cloud == nullptr || indices == nullptr)
    {
      pub_target_points_.publish(points);
      return;
    }

    pcl::ExtractIndices<Point> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(points);
    points.header = cloud->header;

    pub_target_points_.publish(points);
  }
  //}

  /* callbackDroneClicked() method //{ */
  void LidarTracker::callbackDroneClicked(const geometry_msgs::PointStamped::ConstPtr& ps)
  {
    const geometry_msgs::Point& loc = ps->point;
    const vec3_t pos(loc.x, loc.y, loc.z);

    vofod::Detection det;
    det.id = -1;
    det.confidence = 1.0;
    det.position.x = loc.x;
    det.position.y = loc.y;
    det.position.z = loc.z;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        det.covariance.at(3 * r + c) = P0_(r, c);

    vofod::Detections::Ptr dets = boost::make_shared<vofod::Detections>();
    dets->header = ps->header;
    dets->detections.push_back(det);

    NODELET_INFO_STREAM("A new track is manually initialized at position [" << pos.transpose() << "]");
    processDetections(dets);
  }
  //}

  /* getTransformToWorld() method //{ */
  std::optional<Eigen::Affine3d> LidarTracker::getTransformToWorld(const std::string& frame_id, const ros::Time& stamp) const
  {
    try
    {
      const ros::Duration timeout(transform_lookup_timeout_);
      // Obtain transform from sensor into world frame
      geometry_msgs::TransformStamped transform;
      transform = m_tf_buffer.lookupTransform(static_frame_id_, frame_id, stamp, timeout);
      return tf2::transformToEigen(transform.transform);
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_WARN_THROTTLE(1.0, "[LidarTracker]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", frame_id.c_str(),
                            static_frame_id_.c_str(), ex.what());
      return std::nullopt;
    }
  }
  //}

  void LidarTracker::publish_profile_start(const profile_routines_t routine_id)
  {
    publish_profile_event(static_cast<uint32_t>(routine_id), vofod::ProfilingInfo::EVENT_TYPE_START);
  }

  void LidarTracker::publish_profile_end(const profile_routines_t routine_id)
  {
    publish_profile_event(static_cast<uint32_t>(routine_id), vofod::ProfilingInfo::EVENT_TYPE_END);
  }

  void LidarTracker::publish_profile_event(const uint32_t routine_id, const uint8_t type)
  {
    vofod::ProfilingInfo msg;
    msg.stamp = ros::Time::fromBoost(ros::WallTime::now().toBoost());
    msg.routine_id = routine_id;
    if (m_profile_last_seq.count(routine_id) == 0)
      m_profile_last_seq.insert({routine_id, 0});
    msg.event_sequence = m_profile_last_seq.at(routine_id);
    msg.event_type = type;
    if (type == vofod::ProfilingInfo::EVENT_TYPE_END)
      m_profile_last_seq.at(routine_id)++;
    std::scoped_lock lck(pub_profiling_info_mtx_);
    pub_profiling_info_.publish(msg);
  }

}  // namespace lidar_tracker

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(lidar_tracker::LidarTracker, nodelet::Nodelet)

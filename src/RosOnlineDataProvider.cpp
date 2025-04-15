/**
 * @file   RosOnlineDataProvider.cpp
 * @brief  ROS wrapper for online processing.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/RosOnlineDataProvider.h"

#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <vector>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosOnlineDataProvider::RosOnlineDataProvider(const VioParams& vio_params)
    : RosDataProviderInterface(vio_params),
      it_(nullptr),
      frame_count_(FrameId(0)),
      left_img_subscriber_(),
      right_img_subscriber_(),
      sync_img_(),
      imu_subscriber_(),
      gt_odom_subscriber_(),
      external_odom_subscriber_(),
      reinit_flag_subscriber_(),
      reinit_pose_subscriber_(),
      imu_queue_(),
      imu_async_spinner_(nullptr),
      async_spinner_(nullptr) {
  // Wait until time is non-zero and valid: this is because at the ctor level
  // we will be querying for gt pose and/or camera info.
  while (ros::ok() && !ros::Time::now().isValid()) {
    if (ros::Time::isSimTime()) {
      LOG_FIRST_N(INFO, 1)
          << "Waiting for ROS time to be valid... \n"
          << "(Sim Time is enabled; run rosbag with --clock argument)";
    } else {
      LOG_FIRST_N(INFO, 1) << "Waiting for ROS time to be valid...";
    }
  }

  // Define ground truth odometry Subsrciber
  static constexpr size_t kMaxGtOdomQueueSize = 10u;
  if (vio_params_.backend_params_->autoInitialize_ == 0 || log_gt_data_) {
    gt_odom_subscriber_ = nh_.subscribe("gt_odom",
                                        kMaxGtOdomQueueSize,
                                        &RosOnlineDataProvider::callbackGtOdom,
                                        this);
  }

  if (vio_params_.backend_params_->autoInitialize_ == 0) {
    LOG(INFO) << "Requested initialization from ground-truth. "
              << "Initializing ground-truth odometry one-shot subscriber.";
    // We wait for the gt pose.
    LOG(WARNING) << "Waiting for ground-truth pose to initialize VIO "
                 << "on ros topic: " << gt_odom_subscriber_.getTopic().c_str();

    double gt_pose_wait_time_s = 10.0;
    nh_private_.getParam("gt_pose_wait_time_s", gt_pose_wait_time_s);
    const ros::Duration kMaxTimeSecsForGtPose(gt_pose_wait_time_s);

    ros::Time start = ros::Time::now();
    ros::Time current = ros::Time::now();
    while (!gt_init_pose_received_ &&
           (current - start) < kMaxTimeSecsForGtPose) {
      if (nh_.ok() && ros::ok() && !ros::isShuttingDown() && !shutdown_) {
        ros::spinOnce();
      } else {
        LOG(FATAL) << "Ros is not ok... Shutting down.";
      }
      current = ros::Time::now();
      CHECK(current.isValid());
    }

    if (!gt_init_pose_received_) {
      LOG(ERROR)
          << "Missing ground-truth pose while trying for "
          << (current - start).toSec() << " seconds.\n"
          << "Enabling autoInitialize and continuing without ground-truth "
             "pose.";
      vio_params_.backend_params_->autoInitialize_ = true;
    }
  } else {
    // disable message about using gt odom for init when only logging gt odom
    gt_init_pose_received_ = true;
  }

  // decide whether or not to force timestamp synchronization
  nh_private_.getParam("force_same_image_timestamp",
                       force_same_image_timestamp_);

  //! IMU Subscription
  // Create a dedicated queue for the Imu callback so that we can use an async
  // spinner on it to process the data lighting fast.
  static constexpr size_t kMaxImuQueueSize = 1000u;
  ros::SubscribeOptions imu_subscriber_options =
      ros::SubscribeOptions::create<sensor_msgs::Imu>(
          "imu",
          kMaxImuQueueSize,
          boost::bind(&RosOnlineDataProvider::callbackIMU, this, _1),
          ros::VoidPtr(),
          &imu_queue_);
  imu_subscriber_options.transport_hints.tcpNoDelay(true);

  // Start IMU subscriber
  imu_subscriber_ = nh_.subscribe(imu_subscriber_options);

  //! Vision Subscription
  // Subscribe to stereo images. Approx time sync, should be exact though...
  // We set the queue to only 1, since we prefer to drop messages to reach
  // real-time than to be delayed...
  static constexpr size_t kMaxImagesQueueSize = 1u;
  it_ = std::make_unique<image_transport::ImageTransport>(nh_);
  switch (vio_params_.frontend_type_) {
    case FrontendType::kMonoImu: {
      subscribeMono(kMaxImagesQueueSize);
      break;
    }
    case FrontendType::kStereoImu: {
      subscribeStereo(kMaxImagesQueueSize);
      break;
    }
    case FrontendType::kRgbdImu: {
      subscribeRgbd(kMaxImagesQueueSize);
      break;
    }

    default: {
      LOG(FATAL) << "Frontend type not recognized.";
    }
  }

  // Define Reinitializer Subscriber
  static constexpr size_t kMaxReinitQueueSize = 1u;
  reinit_flag_subscriber_ =
      nh_.subscribe("reinit_flag",
                    kMaxReinitQueueSize,
                    &RosOnlineDataProvider::callbackReinit,
                    this);
  reinit_pose_subscriber_ =
      nh_.subscribe("reinit_pose",
                    kMaxReinitQueueSize,
                    &RosOnlineDataProvider::callbackReinitPose,
                    this);

  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(nh_private_.getParam("left_cam_frame_id", left_cam_frame_id_));
  CHECK(!left_cam_frame_id_.empty());
  CHECK(nh_private_.getParam("right_cam_frame_id", right_cam_frame_id_));
  CHECK(!right_cam_frame_id_.empty());

  // External Odometry Subscription
  CHECK(nh_private_.getParam("use_external_odom", use_external_odom_));
  if (use_external_odom_) {
    static constexpr size_t kMaxExternalOdomQueueSize = 1000u;
    external_odom_subscriber_ =
        nh_.subscribe("external_odom",
                      kMaxExternalOdomQueueSize,
                      &RosOnlineDataProvider::callbackExternalOdom,
                      this);
  }

  // UWB Subscription
  CHECK(nh_private_.getParam("use_uwb", use_uwb_));
  if (use_uwb_) {
    // Get UWB parameters
    int num_robots;
    CHECK(nh_private_.getParam("num_robots", num_robots));
    num_robots_ = static_cast<size_t>(num_robots);

    int robot_id;
    CHECK(nh_private_.getParam("robot_id", robot_id));
    robot_id_ = static_cast<uint16_t>(robot_id);

    CHECK(nh_private_.getParam("uwb_topic", uwb_topic_));
    CHECK(nh_private_.getParam("dis_topic", dis_topic_));

    // Initialize UWB body positions
    t_uwb_body_.resize(num_robots_, std::vector<double>(3, 0.0));
    last_dis_.resize(3, std::vector<double>(3 * num_robots_, -1.0));
    for (size_t id = 0; id < num_robots_; id++) {
      for (size_t uid = 0; uid < 3; uid++) {
        ros::param::get(
            "~t_body_uwb" + std::to_string(id) + "_" + std::to_string(uid),
            t_uwb_body_[id][uid]);
      }
    }

    // Set up UWB subscribers and publisher
    subscribeUWB();

    // Create timer for processing UWB data
    // TODO(RonghaiHe): make this a parameter
    double uwb_process_rate = 10.0;  // 10 Hz by default
    nh_private_.getParam("uwb_process_rate", uwb_process_rate);
    uwb_process_timer_ = nh_.createTimer(
        ros::Duration(1.0 / uwb_process_rate),
        [this](const ros::TimerEvent&) { this->processUWBFrames(); });
  }

  publishStaticTf(vio_params_.camera_params_.at(0).body_Pose_cam_,
                  base_link_frame_id_,
                  left_cam_frame_id_);
  if (vio_params_.camera_params_.size() == 2) {
    publishStaticTf(vio_params_.camera_params_.at(1).body_Pose_cam_,
                    base_link_frame_id_,
                    right_cam_frame_id_);
  }

  //! IMU Spinner
  if (vio_params_.parallel_run_) {
    // Imu Async Spinner: will process the imu queue only, instead of ROS'
    // global
    // queue. A value of 0 means to use the number of processor cores.
    static constexpr size_t kImuSpinnerThreads = 2u;
    imu_async_spinner_ =
        std::make_unique<ros::AsyncSpinner>(kImuSpinnerThreads, &imu_queue_);

    //! Vision Spinner
    // This async spinner will process the regular Global callback queue of ROS.
    static constexpr size_t kGlobalSpinnerThreads = 2u;
    async_spinner_ = std::make_unique<ros::AsyncSpinner>(kGlobalSpinnerThreads);
  } else {
    LOG(INFO) << "RosOnlineDataProvider running in sequential mode.";
  }
}

RosOnlineDataProvider::~RosOnlineDataProvider() {
  VLOG(1) << "RosOnlineDataProvider destructor called.";
  imu_queue_.disable();

  if (imu_async_spinner_) imu_async_spinner_->stop();
  if (async_spinner_) async_spinner_->stop();

  LOG(INFO) << "RosOnlineDataProvider successfully shutdown.";
}

void RosOnlineDataProvider::subscribeMono(const size_t& kMaxImagesQueueSize) {
  left_img_subscriber_.subscribe(
      *it_, "left_cam/image_raw", kMaxImagesQueueSize);
  left_img_subscriber_.registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackMonoImage, this, _1));
}

void RosOnlineDataProvider::subscribeStereo(const size_t& kMaxImagesQueueSize) {
  left_img_subscriber_.subscribe(
      *it_, "left_cam/image_raw", kMaxImagesQueueSize);
  right_img_subscriber_.subscribe(
      *it_, "right_cam/image_raw", kMaxImagesQueueSize);
  static constexpr size_t kMaxImageSynchronizerQueueSize = 10u;
  sync_img_ = std::make_unique<message_filters::Synchronizer<sync_pol_img>>(
      sync_pol_img(kMaxImageSynchronizerQueueSize),
      left_img_subscriber_,
      right_img_subscriber_);

  DCHECK(sync_img_);
  sync_img_->registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackStereoImages, this, _1, _2));
}

void RosOnlineDataProvider::subscribeRgbd(const size_t& kMaxImagesQueueSize) {
  left_img_subscriber_.subscribe(
      *it_, "left_cam/image_raw", kMaxImagesQueueSize);
  depth_img_subscriber_.subscribe(
      *it_,
      "depth_cam/image_raw",
      kMaxImagesQueueSize,
      image_transport::TransportHints(
          "raw", ros::TransportHints(), nh_private_, "image_transport_depth"));
  static constexpr size_t kMaxImageSynchronizerQueueSize = 10u;
  sync_img_ = std::make_unique<message_filters::Synchronizer<sync_pol_img>>(
      sync_pol_img(kMaxImageSynchronizerQueueSize),
      left_img_subscriber_,
      depth_img_subscriber_);

  DCHECK(sync_img_);
  sync_img_->registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackRgbdImages, this, _1, _2));
}

void RosOnlineDataProvider::subscribeUWB() {
  static constexpr size_t kMaxUWBQueueSize = 1000u;
  uwb0_subscriber_ = nh_.subscribe(uwb_topic_ + "/0",
                                   kMaxUWBQueueSize,
                                   &RosOnlineDataProvider::callbackUWB0,
                                   this);
  uwb1_subscriber_ = nh_.subscribe(uwb_topic_ + "/1",
                                   kMaxUWBQueueSize,
                                   &RosOnlineDataProvider::callbackUWB1,
                                   this);
  uwb2_subscriber_ = nh_.subscribe(uwb_topic_ + "/2",
                                   kMaxUWBQueueSize,
                                   &RosOnlineDataProvider::callbackUWB2,
                                   this);

  uwb_pub_ = nh_.advertise<pose_graph_tools_msgs::UWBFrame>(
      dis_topic_, kMaxUWBQueueSize * 18);
}

void RosOnlineDataProvider::callbackUWB0(
    const nlink_parser::LinktrackNodeframe2ConstPtr& uwb_msg) {
  int64_t stamp = uwb_msg->stamp.sec * 100 + uwb_msg->stamp.nsec / 1e7;
  {
    std::lock_guard<std::mutex> lock(uwb0_mutex_);
    uwb0_map_[stamp] = uwb_msg;
  }
}

void RosOnlineDataProvider::callbackUWB1(
    const nlink_parser::LinktrackNodeframe2ConstPtr& uwb_msg) {
  int64_t stamp = uwb_msg->stamp.sec * 100 + uwb_msg->stamp.nsec / 1e7;
  {
    std::lock_guard<std::mutex> lock(uwb1_mutex_);
    uwb1_map_[stamp] = uwb_msg;
  }
}

void RosOnlineDataProvider::callbackUWB2(
    const nlink_parser::LinktrackNodeframe2ConstPtr& uwb_msg) {
  int64_t stamp = uwb_msg->stamp.sec * 100 + uwb_msg->stamp.nsec / 1e7;
  {
    std::lock_guard<std::mutex> lock(uwb2_mutex_);
    uwb2_map_[stamp] = uwb_msg;
  }
}

void RosOnlineDataProvider::processUWBFrames() {
  // Early return if no data from the first UWB sensor is available
  if (uwb0_map_.empty()) {
    return;
  }

  // Log the size of UWB data buffers (only once)
  // ROS_INFO_ONCE(
  //     "UWB maps sizes: uwb0 size: %lu, uwb1 size: %lu, uwb2 size: %lu",
  //     uwb0_map_.size(),
  //     uwb1_map_.size(),
  //     uwb2_map_.size());

  // Create maps to store UWB measurements with matching timestamps
  std::map<int64_t, nlink_parser::LinktrackNodeframe2ConstPtr> uwb0_map_copy,
      uwb1_map_copy, uwb2_map_copy, matching_uwb0_map, matching_uwb1_map,
      matching_uwb2_map;

  {
    std::lock_guard<std::mutex> lock0(uwb0_mutex_);
    if (uwb0_map_.empty()) {
      return;
    }
    uwb0_map_copy = uwb0_map_;
    uwb0_map_.clear();
  }

  {
    std::lock_guard<std::mutex> lock1(uwb1_mutex_);
    uwb1_map_copy = uwb1_map_;
    uwb1_map_.clear();
  }

  {
    std::lock_guard<std::mutex> lock2(uwb2_mutex_);
    uwb2_map_copy = uwb2_map_;
    uwb2_map_.clear();
  }

  // Find timestamps in uwb0_map that match with timestamps in uwb1_map and
  // uwb2_map
  for (const auto& uwb0_pair : uwb0_map_copy) {
    int64_t stamp = uwb0_pair.first;
    auto it1 = uwb1_map_copy.find(stamp);
    auto it2 = uwb2_map_copy.find(stamp);

    // If matching timestamp found in uwb1_map, add to matching map
    if (it1 != uwb1_map_copy.end()) {
      matching_uwb1_map[stamp] = it1->second;
    }

    // If matching timestamp found in uwb2_map, add to matching map
    if (it2 != uwb2_map_copy.end()) {
      matching_uwb2_map[stamp] = it2->second;
    }
    // Always include the original uwb0 data in the matching map
    matching_uwb0_map[stamp] = uwb0_pair.second;
  }

  // Define processing parameters
  // epsilon: Threshold for temporal consistency check (minimum change in
  // distance between frames)
  auto epsilon = 1e-8;
  // sigma: Standard deviation of measurement noise (tripled for conservative
  // filtering)
  auto sigma = 0.0383 * 3;
  // epsilon3: Threshold for RSSI difference between received and first path
  // signals
  auto epsilon3 = 10.0;

  // Get UWB positions of the three UWB sensors in body frame
  // TODO (RonghaiHe) Different robot, different t_uwb_body_
  // Extract position of UWB sensor 0 from configuration
  auto u0_pos =
      gtsam::Point3(t_uwb_body_[0][0], t_uwb_body_[0][1], t_uwb_body_[0][2]);
  auto u1_pos =
      gtsam::Point3(t_uwb_body_[1][0], t_uwb_body_[1][1], t_uwb_body_[1][2]);
  auto u2_pos =
      gtsam::Point3(t_uwb_body_[2][0], t_uwb_body_[2][1], t_uwb_body_[2][2]);

  // Store UWB positions in a vector for easier access
  std::vector<gtsam::Point3> uwb_pos(3);
  uwb_pos[0] = u0_pos;
  uwb_pos[1] = u1_pos;
  uwb_pos[2] = u2_pos;

  // Calculate ground truth distances between UWB sensors (used for outlier
  // rejection) Initialize matrix to store distances between sensors
  Eigen::Matrix3d gt_dis = Eigen::Matrix3d::Zero();
  gt_dis(0, 1) = (u0_pos - u1_pos).norm();  // Distance between UWB0 and UWB1
  gt_dis(0, 2) = (u0_pos - u2_pos).norm();  // Distance between UWB0 and UWB2
  gt_dis(1, 0) = gt_dis(0, 1);              // Distance is symmetric
  gt_dis(1, 2) = (u1_pos - u2_pos).norm();  // Distance between UWB1 and UWB2
  gt_dis(2, 0) = gt_dis(0, 2);              // Distance is symmetric
  gt_dis(2, 1) = gt_dis(1, 2);              // Distance is symmetric
  // gt_dis(2, 2) = 0.0;                       // Self-distance is zero
  // gt_dis(1, 1) = 0.0;                       // Self-distance is zero
  // gt_dis(0, 0) = 0.0;                       // Self-distance is zero

  // Store iterators to previous frames for temporal consistency checks
  // auto it0_prev = matching_uwb0_map.begin();
  // auto it1_prev = matching_uwb1_map.begin();
  // auto it2_prev = matching_uwb2_map.begin();

  // Process each UWB frame with matching timestamps
  for (auto it0 = matching_uwb0_map.begin(); it0 != matching_uwb0_map.end();
       ++it0) {
    // Find corresponding UWB frames from sensors 1 and 2
    auto it1 = matching_uwb1_map.find(it0->first);
    auto it2 = matching_uwb2_map.find(it0->first);

    u_int num_abnormal = 0, num_normal_meas = 9;
    std::vector<bool> unormal(3, false);

    // Check if only bot0's UWB data is valid
    // If so, skip to next frame for only 3 measurements for any other robots
    if (it1 == matching_uwb1_map.end()) {
      ++num_abnormal;
      num_normal_meas -= 3;
      unormal[1] = true;
    }
    if (it2 == matching_uwb2_map.end()) {
      if (num_abnormal == 1) {
        ROS_WARN("Only UWB0 sensor is available");
        continue;
      }
      ++num_abnormal;
      unormal[2] = true;
    }

    // bool abnormal = false;
    for (auto& node : it0->second->nodes) {
      if (node.id == robot_id_ * 3) {
        continue;
      }
      if (last_dis_[0][node.id] < 0.0 ||
          fabs(last_dis_[0][node.id] - node.dis) < epsilon) {
        // ROS_WARN("Step 1: UWB0: %u, %f, %f",
        //          node.id,
        //          node.dis,
        //          last_dis_[0][node.id]);
        // abnormal = true;
        --num_normal_meas;
      }
      last_dis_[0][node.id] = node.dis;
    }
    // if (abnormal) {
    //   if (num_abnormal == 1) {
    //     ROS_WARN("Step 1: Less than 6 distances are available");
    //     continue;
    //   }
    //   unormal[0] = true;
    //   ++num_abnormal;
    // }
    if (!unormal[1]) {
      // abnormal = false;
      for (auto& node : it1->second->nodes) {
        if (node.id == robot_id_ * 3 + 1) {
          continue;
        }
        if (last_dis_[1][node.id] < 0.0 ||
            fabs(last_dis_[1][node.id] - node.dis) < epsilon) {
          // ROS_WARN("Step 1: UWB01: %u, %f, %f",
          //          node.id,
          //          node.dis,
          //          last_dis_[1][node.id]);
          // abnormal = true;
          --num_normal_meas;
        }
        last_dis_[1][node.id] = node.dis;
      }
      // if (abnormal) {
      //   if (num_abnormal == 1) {
      //     ROS_WARN("Step 1: Only 1 UWB(not 1) sensor is available");
      //     continue;
      //   }
      //   unormal[1] = true;
      //   ++num_abnormal;
      // }
    }
    if (!unormal[2]) {
      // abnormal = false;
      for (auto& node : it2->second->nodes) {
        if (node.id == robot_id_ * 3 + 2) {
          continue;
        }
        if (last_dis_[2][node.id] < 0.0 ||
            fabs(last_dis_[2][node.id] - node.dis) < epsilon) {
          // ROS_WARN("Step 1: UWB2: %u, %f, %f",
          //          node.id,
          //          node.dis,
          //          last_dis_[2][node.id]);
          // abnormal = true;
          --num_normal_meas;
        }
        last_dis_[2][node.id] = node.dis;
      }
      // if (abnormal) {
      //   if (num_abnormal == 1) {
      //     ROS_WARN("Step 1: Only 1 UWB(not 2) sensor is available");
      //     continue;
      //   }
      //   unormal[2] = true;
      //   ++num_abnormal;
      // }
    }
    if (num_normal_meas < 6) {
      ROS_WARN("Step 1: Less than 6 distances are available");
    }

    // Check validity of current bot's 3 UWB sensors by examining
    // inter-sensor distances
    // Extract distances between UWB sensors from data (d01 = distance from
    // sensor 0 to 1)
    if (!unormal[0]) {
      auto d01 = it0->second->nodes[robot_id_ * 3 + 1].dis;
      auto d02 = it0->second->nodes[robot_id_ * 3 + 2].dis;
      unormal[0] =
          fabs(d01 - gt_dis(0, 1)) > sigma | fabs(d02 - gt_dis(0, 2)) > sigma;
      if (unormal[0]) {
        // ROS_WARN("Step 2: UWB0: %f vs %f, %f vs %f",
        //          d01,
        //          gt_dis(0, 1),
        //          d02,
        //          gt_dis(0, 2));
        if (num_abnormal == 1) {
          ROS_WARN("Step 2: Only 1 UWB(not 0) sensor is available");
          continue;
        }
        ++num_abnormal;
      }
    }
    if (!unormal[1]) {
      auto d10 = it1->second->nodes[robot_id_ * 3 + 0].dis;
      auto d12 = it1->second->nodes[robot_id_ * 3 + 2].dis;
      unormal[1] =
          fabs(d10 - gt_dis(1, 0)) > sigma | fabs(d12 - gt_dis(1, 2)) > sigma;
      if (unormal[1]) {
        // ROS_WARN("Step 2: UWB1: %f vs %f, %f vs %f",
        //          d10,
        //          gt_dis(1, 0),
        //          d12,
        //          gt_dis(1, 2));
        if (num_abnormal == 1) {
          ROS_WARN("Step 2: Only 1 UWB(not 1) sensor is available");
          continue;
        }
        ++num_abnormal;
      }
    }
    if (!unormal[2]) {
      auto d20 = it2->second->nodes[robot_id_ * 3 + 0].dis;
      auto d21 = it2->second->nodes[robot_id_ * 3 + 1].dis;
      unormal[2] =
          fabs(d20 - gt_dis(2, 0)) > sigma | fabs(d21 - gt_dis(2, 1)) > sigma;
      if (unormal[2]) {
        // ROS_WARN("Step 2: UWB2: %f vs %f, %f vs %f",
        //          d20,
        //          gt_dis(2, 0),
        //          d21,
        //          gt_dis(2, 1));
        if (num_abnormal == 1) {
          ROS_WARN("Step 2: Only 1 UWB(not 2) sensor is available");
          continue;
        }
        ++num_abnormal;
      }
    }

    // Check each destination robot's UWB data
    // Iterate through each possible destination robot
    for (auto dest_id = 0; dest_id < num_robots_; dest_id++) {
      if (dest_id == robot_id_) {
        continue;
      }
      // Create a vector of maps to store valid UWB measurements from each
      // sensor (0,1,2)
      // Each map relates destination UWB sensor ID to its measurements
      std::vector<std::map<int, nlink_parser::LinktrackNode2ConstPtr>>
          effect_uwb(3);

      // Get reference to nodes from UWB sensor 0
      auto& src_u0_uwbs = it0->second->nodes;

      // Step 3: Process distance data by RSSI
      // Process each of the 3 UWB sensors on the destination robot with
      // local ID(uid)
      for (auto uid = 0; uid < 3; uid++) {
        // Calculate global UID based on destination robot ID and sensor ID
        auto global_uid = dest_id * 3 + uid;

        // Process UWB0 measurements if they exist and are valid (distance > 0)
        if (!unormal[0]) {
          if (src_u0_uwbs.size() > global_uid &&
              src_u0_uwbs[global_uid].dis > 0) {
            // Skip if rx-fx > epsilon3
            // Skip if RSSI difference exceeds threshold (potential multipath
            // interference)
            // rx_rssi: received signal strength indicator
            // fp_rssi: first path signal strength indicator
            if (src_u0_uwbs[global_uid].fp_rssi -
                    src_u0_uwbs[global_uid].rx_rssi >
                epsilon3) {
              // ROS_WARN("Step 3: UWB0: %u, %f, %f",
              //          global_uid,
              //          src_u0_uwbs[global_uid].fp_rssi,
              //          src_u0_uwbs[global_uid].rx_rssi);
              if (num_abnormal == 1) {
                // ROS_WARN("Step 3: Less than 6 distances are available");
                break;
              }
              continue;
            }
            // TODO (RonghaiHe) Drop the measurement if the distance is >50
            if (src_u0_uwbs[global_uid].dis > 50.0) {
              if (num_abnormal == 1) {
                // ROS_WARN("Step 3: Less than 6 distances are available");
                break;
              }
              continue;
            }

            // Store valid measurement from UWB0 sensor
            effect_uwb[0][uid] =
                boost::make_shared<nlink_parser::LinktrackNode2>(
                    (src_u0_uwbs[global_uid]));
            // ROS_INFO("effect_uwb[0].size() = %d", effect_uwb[0].size());
          }
        }

        // u1 checks
        if (!unormal[1]) {
          auto& src_u1_uwbs = it1->second->nodes;
          if (src_u1_uwbs.size() > global_uid &&
              src_u1_uwbs[global_uid].dis > 0) {
            // Skip if rx-fx > epsilon3
            if (src_u1_uwbs[global_uid].fp_rssi -
                    src_u1_uwbs[global_uid].rx_rssi >
                epsilon3) {
              // ROS_WARN("Step 3: UWB01: %u, %f, %f",
              //          global_uid,
              //          src_u1_uwbs[global_uid].fp_rssi,
              //          src_u1_uwbs[global_uid].rx_rssi);
              if (num_abnormal == 1) {
                // ROS_WARN("Step 3: Less than 6 distances are available");
                break;
              }
              continue;
            }
            // TODO (RonghaiHe) Drop the measurement if the distance is >50
            if (src_u1_uwbs[global_uid].dis > 50.0) {
              if (num_abnormal == 1) {
                // ROS_WARN("Step 3: Less than 6 distances are available");
                break;
              }
              continue;
            }

            effect_uwb[1][uid] =
                boost::make_shared<nlink_parser::LinktrackNode2>(
                    (src_u1_uwbs[global_uid]));
            // ROS_INFO("effect_uwb[1].size() = %d", effect_uwb[1].size());
          }
        }

        // u2 checks
        if (!unormal[2]) {
          auto& src_u2_uwbs = it2->second->nodes;
          if (src_u2_uwbs.size() > global_uid &&
              src_u2_uwbs[global_uid].dis > 0) {
            // Skip if rx-fx > epsilon3
            if (src_u2_uwbs[global_uid].fp_rssi -
                    src_u2_uwbs[global_uid].rx_rssi >
                epsilon3) {
              // ROS_WARN("Step 3: UWB2: %u, %f, %f",
              //          global_uid,
              //          src_u2_uwbs[global_uid].fp_rssi,
              //          src_u2_uwbs[global_uid].rx_rssi);
              if (num_abnormal == 1) {
                // ROS_WARN("Step 3: Less than 6 distances are available");
                break;
              }
              continue;
            }
            // TODO (RonghaiHe) Drop the measurement if the distance is >50
            if (src_u2_uwbs[global_uid].dis > 50.0) {
              if (num_abnormal == 1) {
                // ROS_WARN("Step 3: Less than 6 distances are available");
                break;
              }
              continue;
            }

            effect_uwb[2][uid] =
                boost::make_shared<nlink_parser::LinktrackNode2>(
                    (src_u2_uwbs[global_uid]));
            // ROS_INFO("effect_uwb[2].size() = %d", effect_uwb[2].size());
          }
        }
      }

      // Skip if not enough effective data (less than 6)
      // Ensure we have enough valid measurements (at least 6 across all 3
      // sensors)
      auto effect_size =
          effect_uwb[0].size() + effect_uwb[1].size() + effect_uwb[2].size();
      if (effect_size < 6) {
        // LOG(WARNING) << "UWB data is not enough, only " << effect_size
        //              << " nodes After 3rd check for no enough data to robot"
        //              << dest_id;
        continue;  // Skip to next destination robot
      }

      // Triangle constraint checks
      // Maps to track consistency of measurements using triangle inequality
      // constraint
      // checknum: counts how many times an edge (src_id, dest_id) is checked
      // errornum: counts how many times an edge fails the triangle inequality
      // test
      std::map<std::tuple<int, int>, int> checknum;
      std::map<std::tuple<int, int>, int> errornum;

      // Lambda to increment check count for an edge
      auto check = [&](int src_id, int dest_id) {
        if (checknum.find(std::make_tuple(src_id, dest_id)) == checknum.end()) {
          checknum[std::make_tuple(src_id, dest_id)] = 1;
        } else {
          checknum[std::make_tuple(src_id, dest_id)] += 1;
        }
        if (errornum.find(std::make_tuple(src_id, dest_id)) == errornum.end()) {
          errornum[std::make_tuple(src_id, dest_id)] = 0;
        }
      };

      // Lambda to increment error count for an edge
      auto error = [&](int src_id, int dest_id) {
        if (errornum.find(std::make_tuple(src_id, dest_id)) == errornum.end()) {
          LOG(WARNING) << "errornum not found";
          errornum[std::make_tuple(src_id, dest_id)] = 1;
        } else {
          errornum[std::make_tuple(src_id, dest_id)] += 1;
        }
      };

      // Check edges where one UWB is an endpoint
      // First triangle inequality check: for each UWB sensor, test triangle
      // formed by
      // this sensor and pairs of destination UWB sensors
      for (auto uid = 0; uid < 3; uid++) {
        auto& uimap = effect_uwb[uid];
        // Find non-duplicate pair combinations on corresponding edges
        // Examine all unique pairs of edges from this source sensor
        for (auto it = uimap.begin(); it != uimap.end(); ++it) {
          for (auto it2 = std::next(it); it2 != uimap.end(); ++it2) {
            auto uid1 = it->first;
            auto uid2 = it2->first;
            auto uid1_dis = it->second->dis;
            auto uid2_dis = it2->second->dis;

            // Record that these edges are being checked
            check(uid, uid1);
            check(uid, uid2);

            auto gt_u1_u2 = gt_dis(uid1, uid2);

            // Check if they can form a triangle
            if (fabs(uid1_dis - uid2_dis) > 3 * sqrt(2) * sigma + gt_u1_u2) {
              error(uid, uid1);
              error(uid, uid2);
            }
          }
        }
      }

      // Check edges where two UWBs are endpoints
      // Second triangle inequality check: test triangles formed by two source
      // UWB sensors
      // and one destination UWB sensor
      for (auto it1_uid = 0; it1_uid < 3; it1_uid++) {
        for (auto it2_uid = it1_uid + 1; it2_uid < 3; it2_uid++) {
          auto& uimap1 = effect_uwb[it1_uid];
          auto& uimap2 = effect_uwb[it2_uid];
          // Find nodes that exist in both edges
          for (auto it1 = uimap1.begin(); it1 != uimap1.end(); ++it1) {
            auto uid1 = it1->first;
            auto uid1_dis = it1->second->dis;
            auto it2 = uimap2.find(uid1);
            if (it2 != uimap2.end()) {
              auto uid2_dis = it2->second->dis;

              check(it1_uid, uid1);
              check(it2_uid, uid1);

              auto gt_u1_u2 = gt_dis(it1_uid, it2_uid);
              // Check if they can form a triangle
              if (fabs(uid1_dis - uid2_dis) > 3 * sqrt(2) * sigma + gt_u1_u2) {
                error(it1_uid, uid1);
                error(it2_uid, uid1);
              }
            }
          }
        }
      }

      // Create and publish UWB frame if enough valid edges
      pose_graph_tools_msgs::UWBFrame uwb;
      uwb.stamp = it0->second->stamp;
      uwb.distances.assign(9, -1.0f);
      uwb.src_robot_id = robot_id_;
      uwb.dst_robot_id = dest_id;

      // Calculate valid edges where error/check ratio <= 0.5
      int valid_num = 0;
      for (auto& it : checknum) {
        auto& kt = it.first;
        auto& src_bot_id = std::get<0>(kt);
        auto& dest_bot_id = std::get<1>(kt);
        auto& check_num = it.second;
        auto& error_num = errornum[kt];

        if (error_num * 1.0 / check_num < 0.5) {
          valid_num++;
          uwb.distances[src_bot_id * 3 + dest_bot_id] =
              effect_uwb[src_bot_id][dest_bot_id]->dis;
        }
      }

      if (valid_num < 6) {
        // LOG(WARNING) << "UWB data is not enough, only " << valid_num
        //              << " valid edges";
        continue;
      }

      // Publish UWB frame
      uwb_pub_.publish(uwb);
    }

    // it0_prev = it0;
    // it1_prev = it1;
    // it2_prev = it2;
  }

  // Clear processed data to avoid memory buildup
  // uwb0_map_.clear();
  // uwb1_map_.clear();
  // uwb2_map_.clear();
}

bool RosOnlineDataProvider::spin() {
  if (!shutdown_) {
    if (vio_params_.parallel_run_) {
      return parallelSpin();
    } else {
      return sequentialSpin();
    }
  } else {
    return false;
  }
}

bool RosOnlineDataProvider::parallelSpin() {
  CHECK(vio_params_.parallel_run_);
  // Start async spinners to get input data (only once!).
  if (!started_async_spinners_) {
    VLOG(10) << "Starting Async spinners.";
    CHECK(imu_async_spinner_);
    imu_async_spinner_->start();
    CHECK(async_spinner_);
    async_spinner_->start();
    started_async_spinners_ = true;
  } else {
    VLOG(10) << "Async spinners already started.";
  }
  return true;
}

bool RosOnlineDataProvider::sequentialSpin() {
  CHECK(!vio_params_.parallel_run_)
      << "This should be only running if we are in sequential mode!";
  CHECK(!imu_async_spinner_) << "There should not be an IMU async spinner "
                                "constructed if in sequential mode.";
  CHECK(!async_spinner_) << "There should not be a general async spinner "
                            "constructed if in sequential mode.";
  CHECK(imu_queue_.isEnabled());
  // First call callbacks in our custom IMU queue.
  imu_queue_.callAvailable(ros::WallDuration(0.1));
  // Then call the rest of callbacks.
  // which is the same as:
  // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
  ros::spinOnce();
  return true;
}

// TODO(marcus): with the readRosImage, this is a slow callback. Might be too
// slow...
void RosOnlineDataProvider::callbackMonoImage(
    const sensor_msgs::ImageConstPtr& img_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 1u);
  const CameraParams& cam_info = vio_params_.camera_params_.at(0);

  CHECK(img_msg);
  const Timestamp& timestamp = img_msg->header.stamp.toNSec();

  if (!shutdown_) {
    CHECK(left_frame_callback_)
        << "Did you forget to register the left frame callback?";
    left_frame_callback_(std::make_unique<Frame>(
        frame_count_, timestamp, cam_info, readRosImage(img_msg)));
    frame_count_++;
  }
}

// TODO(marcus): with the readRosImage, this is a slow callback. Might be too
// slow...
void RosOnlineDataProvider::callbackStereoImages(
    const sensor_msgs::ImageConstPtr& left_msg,
    const sensor_msgs::ImageConstPtr& right_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 2u);
  const CameraParams& left_cam_info = vio_params_.camera_params_.at(0);
  const CameraParams& right_cam_info = vio_params_.camera_params_.at(1);

  CHECK(left_msg);
  CHECK(right_msg);
  const Timestamp& timestamp_left = left_msg->header.stamp.toNSec();
  const Timestamp& timestamp_right = right_msg->header.stamp.toNSec();

  if (!shutdown_) {
    CHECK(left_frame_callback_)
        << "Did you forget to register the left frame callback?";
    left_frame_callback_(std::make_unique<Frame>(
        frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));

    if (vio_params_.frontend_type_ == VIO::FrontendType::kStereoImu) {
      CHECK(right_frame_callback_)
          << "Did you forget to register the right frame callback?";
      right_frame_callback_(std::make_unique<Frame>(
          frame_count_,
          force_same_image_timestamp_ ? timestamp_left : timestamp_right,
          right_cam_info,
          readRosImage(right_msg)));
    }
    frame_count_++;
  }
}

void RosOnlineDataProvider::callbackRgbdImages(
    const sensor_msgs::ImageConstPtr& color_msg,
    const sensor_msgs::ImageConstPtr& depth_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 1u);
  const CameraParams& cam_info = vio_params_.camera_params_.at(0);

  CHECK(color_msg);
  CHECK(depth_msg);
  const Timestamp& timestamp_color = color_msg->header.stamp.toNSec();
  const Timestamp& timestamp_depth = depth_msg->header.stamp.toNSec();

  if (!shutdown_) {
    CHECK(left_frame_callback_)
        << "Did you forget to register the color frame callback?";
    left_frame_callback_(std::make_unique<Frame>(
        frame_count_, timestamp_color, cam_info, readRosImage(color_msg)));

    CHECK(depth_frame_callback_)
        << "Did you forget to register the depth frame callback?";
    depth_frame_callback_(std::make_unique<DepthFrame>(
        frame_count_,
        force_same_image_timestamp_ ? timestamp_color : timestamp_depth,
        readRosDepthImage(depth_msg)));
  }

  frame_count_++;
}

void RosOnlineDataProvider::callbackIMU(
    const sensor_msgs::ImuConstPtr& imu_msg) {
  // TODO(TONI): detect jump backwards in time?

  VIO::ImuAccGyr imu_accgyr;

  imu_accgyr(0) = imu_msg->linear_acceleration.x;
  imu_accgyr(1) = imu_msg->linear_acceleration.y;
  imu_accgyr(2) = imu_msg->linear_acceleration.z;
  imu_accgyr(3) = imu_msg->angular_velocity.x;
  imu_accgyr(4) = imu_msg->angular_velocity.y;
  imu_accgyr(5) = imu_msg->angular_velocity.z;

  // Adapt imu timestamp to account for time shift in IMU-cam
  Timestamp timestamp = imu_msg->header.stamp.toNSec();

  if (!shutdown_) {
    CHECK(imu_single_callback_)
        << "Did you forget to register the IMU callback?";
    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
  }
}

// Ground-truth odometry callback
void RosOnlineDataProvider::callbackGtOdom(
    const nav_msgs::Odometry::ConstPtr& gt_odom_msg) {
  CHECK(gt_odom_msg);
  if (!gt_init_pose_received_) {
    LOG(WARNING) << "Using initial ground-truth state for initialization.";
    utils::rosOdometryToVioNavState(
        *gt_odom_msg,
        nh_private_,
        &vio_params_.backend_params_->initial_ground_truth_state_);

    // Signal receptance of ground-truth pose.
    gt_init_pose_received_ = true;
  }

  CHECK(gt_init_pose_received_);
  if (log_gt_data_) {
    logGtData(gt_odom_msg);
  } else {
    // Shutdown to prevent more than one message being processed.
    gt_odom_subscriber_.shutdown();
  }
}

void RosOnlineDataProvider::callbackExternalOdom(
    const nav_msgs::Odometry::ConstPtr& odom_msg) {
  CHECK(odom_msg);
  VIO::VioNavState kimera_odom;
  utils::rosOdometryToVioNavState(*odom_msg, nh_private_, &kimera_odom);
  external_odom_callback_(ExternalOdomMeasurement(
      odom_msg->header.stamp.toNSec(),
      gtsam::NavState(kimera_odom.pose_, kimera_odom.velocity_)));
}

// Reinitialization callback
void RosOnlineDataProvider::callbackReinit(
    const std_msgs::Bool::ConstPtr& reinitFlag) {
  // TODO(Sandro): Do we want to reinitialize at specific pose or just at
  // origin? void RosDataProvider::callbackReinit( const
  // nav_msgs::Odometry::ConstPtr& msgReinit) {

  // Set reinitialization to "true"
  reinit_flag_ = true;

  if (getReinitFlag()) {
    ROS_INFO("Reinitialization flag received!\n");
  }
}

// Getting re-initialization pose
void RosOnlineDataProvider::callbackReinitPose(
    const geometry_msgs::PoseStamped& reinitPose) {
  // Set reinitialization pose
  gtsam::Rot3 rotation(gtsam::Quaternion(reinitPose.pose.orientation.w,
                                         reinitPose.pose.orientation.x,
                                         reinitPose.pose.orientation.y,
                                         reinitPose.pose.orientation.z));
  gtsam::Point3 position(reinitPose.pose.position.x,
                         reinitPose.pose.position.y,
                         reinitPose.pose.position.z);
  reinit_packet_.setReinitPose(gtsam::Pose3(rotation, position));
}

void RosOnlineDataProvider::publishStaticTf(const gtsam::Pose3& pose,
                                            const std::string& parent_frame_id,
                                            const std::string& child_frame_id) {
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transform_stamped;
  // TODO(Toni): Warning: using ros::Time::now(), will that bring issues?
  static_transform_stamped.header.stamp = ros::Time::now();
  static_transform_stamped.header.frame_id = parent_frame_id;
  static_transform_stamped.child_frame_id = child_frame_id;
  utils::gtsamPoseToRosTf(pose, &static_transform_stamped.transform);
  static_broadcaster.sendTransform(static_transform_stamped);
}

}  // namespace VIO

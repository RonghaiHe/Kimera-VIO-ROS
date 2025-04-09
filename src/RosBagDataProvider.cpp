/**
 * @file   RosBagDataProvider.cpp
 * @brief  Parse rosbag and run Kimera-VIO.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/RosBagDataProvider.h"

#include <glog/logging.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <pose_graph_tools_msgs/UWBFrame.h>
#include <rosgraph_msgs/Clock.h>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosbagDataProvider::RosbagDataProvider(const VioParams& vio_params)
    : RosDataProviderInterface(vio_params),
      rosbag_data_(),
      rosbag_path_(""),
      left_imgs_topic_(""),
      right_imgs_topic_(""),
      imu_topic_(""),
      gt_odom_topic_(""),
      external_odom_topic_(""),
      uwb_topic_(""),
      clock_pub_(),
      imu_pub_(),
      left_img_pub_(),
      right_img_pub_(),
      gt_odometry_pub_(),
      external_odometry_pub_(),
      uwb_pub_(),
      timestamp_last_frame_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_kf_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_imu_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_gt_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_odom_(std::numeric_limits<Timestamp>::min()),
      k_(0u),
      k_last_kf_(0u),
      k_last_imu_(0u),
      k_last_gt_(0u),
      k_last_odom_(0u),
      use_external_odom_(false),
      num_robots_(3),
      robot_id(0),
      use_uwb_(false) {
  CHECK(nh_private_.getParam("rosbag_path", rosbag_path_));
  CHECK(nh_private_.getParam("left_cam_rosbag_topic", left_imgs_topic_));
  if (vio_params_.frontend_type_ == FrontendType::kStereoImu) {
    CHECK(nh_private_.getParam("right_cam_rosbag_topic", right_imgs_topic_));
  }
  if (vio_params_.frontend_type_ == FrontendType::kRgbdImu) {
    CHECK(nh_private_.getParam("depth_cam_rosbag_topic", depth_imgs_topic_));
  }
  CHECK(nh_private_.getParam("imu_rosbag_topic", imu_topic_));
  CHECK(nh_private_.getParam("ground_truth_odometry_rosbag_topic",
                             gt_odom_topic_));
  CHECK(nh_private_.getParam("use_external_odom", use_external_odom_));
  CHECK(nh_private_.getParam("external_odometry_rosbag_topic",
                             external_odom_topic_));

  // for Multi-robot
  CHECK(nh_private_.getParam("use_uwb", use_uwb_));
  CHECK(nh_private_.getParam("num_robots", num_robots_));
  CHECK(nh_private_.getParam("robot_id", robot_id));

  // TODO: UWB topic in params
  CHECK(nh_private_.getParam("uwb_rosbag_topic", uwb_topic_));

  // TODO: robot name file in rosparams


  LOG(INFO) << "Constructing RosbagDataProvider from path: \n"
            << " - Rosbag Path: " << rosbag_path_.c_str() << '\n'
            << "With ROS topics: \n"
            << " - Left cam: " << left_imgs_topic_.c_str() << '\n'
            << " - Right cam: " << right_imgs_topic_.c_str() << '\n'
            << " - Depth cam: " << depth_imgs_topic_ << '\n'
            << " - IMU: " << imu_topic_.c_str() << '\n'
            << " - GT odom: " << gt_odom_topic_.c_str() << '\n'
            << " - External odom: " << external_odom_topic_.c_str();

  CHECK(!rosbag_path_.empty());
  CHECK(!left_imgs_topic_.empty());
  if (vio_params_.frontend_type_ == FrontendType::kStereoImu) {
    CHECK(!right_imgs_topic_.empty());
  }
  CHECK(!imu_topic_.empty());

  std::map<unsigned, std::string> mRobotNames;
  for (size_t id = 0; id < num_robots_ ;id++) {
    std::string robot_name = "kimera" + std::to_string(id);
    ros::param::get("~robot" + std::to_string(id) + "_name", robot_name);
    mRobotNames[id] = robot_name;
  }

  // Ros publishers specific to rosbag data provider
  static constexpr size_t kQueueSize = 10u;

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", kQueueSize);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, kQueueSize);
  left_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(left_imgs_topic_, kQueueSize);
  if (vio_params_.frontend_type_ == FrontendType::kStereoImu) {
    right_img_pub_ =
        nh_.advertise<sensor_msgs::Image>(right_imgs_topic_, kQueueSize);
  }

  if (!gt_odom_topic_.empty()) {
    gt_odometry_pub_ =
        nh_.advertise<nav_msgs::Odometry>(gt_odom_topic_, kQueueSize);
  }

  if (!uwb_topic_.empty()) {
    uwb_pub_ =
        nh_.advertise<pose_graph_tools_msgs::UWBFrame>(uwb_topic_, kQueueSize * 18);
  }

  // 初始化bot
  t_uwb_body_.resize(num_robots_, std::vector<double>(3, 0.0));
  for (size_t id = 0; id < num_robots_; id++) {
    for (size_t uid = 0; uid < 3; uid++) {
      ros::param::get("~t_body_uwb" +
        std::to_string(uid),
      t_uwb_body_[id][uid]);
    }
  }


  if (use_external_odom_) {
    use_external_odom_ = true;
    if (!external_odom_topic_.empty()) {
      external_odometry_pub_ =
          nh_.advertise<nav_msgs::Odometry>(external_odom_topic_, kQueueSize);
    } else {
      LOG(WARNING) << "use_external_odom set to true but no topic provided.";
    }
  }
}

void RosbagDataProvider::initialize() {
  CHECK_EQ(k_, 0u);
  LOG(INFO) << "Initialize Rosbag Data Provider.";
  // Parse data from rosbag first thing:
  CHECK(parseRosbag(rosbag_path_, &rosbag_data_));

  // Autoinitialize if necessary: this changes the values for anyone
  // holding vio_params (such as the VIO), since backend params are a ptr.
  if (vio_params_.backend_params_->autoInitialize_ == 0) {
    vio_params_.backend_params_->initial_ground_truth_state_ =
        getGroundTruthVioNavState(0u);  // Send first gt state.
    LOG(WARNING) << "Using initial ground-truth state for initialization:";
    vio_params_.backend_params_->initial_ground_truth_state_.print();
  }
}

void RosbagDataProvider::sendImuDataToVio() {
  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
  for (const sensor_msgs::ImuConstPtr& imu_msg : rosbag_data_.imu_msgs_) {
    const ImuStamp& imu_data_timestamp = imu_msg->header.stamp.toNSec();
    ImuAccGyr imu_accgyr;
    imu_accgyr(0) = imu_msg->linear_acceleration.x;
    imu_accgyr(1) = imu_msg->linear_acceleration.y;
    imu_accgyr(2) = imu_msg->linear_acceleration.z;
    imu_accgyr(3) = imu_msg->angular_velocity.x;
    imu_accgyr(4) = imu_msg->angular_velocity.y;
    imu_accgyr(5) = imu_msg->angular_velocity.z;
    imu_single_callback_(ImuMeasurement(imu_data_timestamp, imu_accgyr));
  }
}

void RosbagDataProvider::sendExternalOdometryToVio() {
  CHECK(external_odom_callback_)
      << "Did you forget to register the external odometry callback?";
  for (const nav_msgs::OdometryConstPtr& odom_msg :
       rosbag_data_.external_odom_) {
    const Timestamp timestamp = odom_msg->header.stamp.toNSec();

    VIO::VioNavState kimera_odom;
    utils::rosOdometryToVioNavState(*odom_msg, nh_private_, &kimera_odom);

    external_odom_callback_(ExternalOdomMeasurement(
        timestamp, gtsam::NavState(kimera_odom.pose_, kimera_odom.velocity_)));
  }
}

// 与sendImuDataToVio紧邻执行
void RosbagDataProvider::sendUWBFrames() {
  // 检查3个uwb消息，并根据时间匹配，若s部分相同且ns部分前2位相同，则认为是同一帧
  std::map<int64_t, nlink_parser::LinktrackNodeframe2ConstPtr>
      uwb0_map, uwb1_map, uwb2_map;
  // 取ns 前2位有效数字插入uwb0_map
  for (const auto& uwb_msg : rosbag_data_.uwb0_msgs_) {
    // CHECK_EQ(msg->id, config_id * 3 + 0);
    int64_t stamp = uwb_msg->stamp.sec * 100 + uwb_msg->stamp.nsec / 1e7;
    uwb0_map[stamp] = uwb_msg;
  }

  // uwb1 时间变换，若时间在uwb0map中则加入 uwb1map
  // uwb2 时间变换， 若在uwb0map中则加入uwb2map
  for (const auto& uwb_msg : rosbag_data_.uwb1_msgs_) {
    // CHECK_EQ(msg->id, config_id * 3 + 1);
    int64_t stamp = uwb_msg->stamp.sec * 100 + uwb_msg->stamp.nsec / 1e7;
    if (uwb0_map.find(stamp) != uwb0_map.end()) {
      uwb1_map[stamp] = uwb_msg;
    }
  }
  for (const auto& uwb_msg : rosbag_data_.uwb2_msgs_) {
    // CHECK_EQ(msg->id, config_id * 3 + 2);
    int64_t stamp = uwb_msg->stamp.sec * 100 + uwb_msg->stamp.nsec / 1e7;
    if (uwb0_map.find(stamp) != uwb0_map.end()) {
      uwb2_map[stamp] = uwb_msg;
    }
  }

  // 时间序列遍历
  auto it0_prev = uwb0_map.begin();
  auto it1_prev = uwb1_map.begin();
  auto it2_prev = uwb2_map.begin();
  auto epsilon = 1e-4;
  auto sigma = 0.0383 * 3;
  auto epsilon3 = 10.0;

  auto u0_pos = Point3(t_uwb_body_[robot_id][0]);
  auto u1_pos = Point3(t_uwb_body_[robot_id][1]);
  auto u2_pos = Point3(t_uwb_body_[robot_id][2]);

  std::vector<Point3> uwb_pos(3);
  uwb_pos[0] = u0_pos;
  uwb_pos[1] = u1_pos;
  uwb_pos[2] = u2_pos;

  Eigen::Matrix3d gt_dis;
  gt_dis(0, 1) = (u0_pos - u1_pos).norm();
  gt_dis(0, 2) = (u0_pos - u2_pos).norm();
  gt_dis(1, 0) = gt_dis(0, 1);
  gt_dis(1, 2) = (u1_pos - u2_pos).norm();
  gt_dis(2, 0) = gt_dis(0, 2);
  gt_dis(2, 1) = gt_dis(1, 2);
  gt_dis(2, 2) = 0.0;
  gt_dis(1, 1) = 0.0;
  gt_dis(0, 0) = 0.0;



  for (auto it0 = uwb0_map.begin(); it0 != uwb0_map.end(); ++it0) {
    auto it1 = uwb1_map.find(it0->first);
    auto it2 = uwb2_map.find(it0->first);

    // 检查当前bot 3个uwb的正确性
    auto d01 = it0->second->nodes[ robot_id * 3 + 1].dis;
    auto d02 = it0->second->nodes[ robot_id * 3 + 2].dis;
    auto d10 = it1 == uwb1_map.end() ? -1 : it1->second->nodes[ robot_id * 3 + 0].dis;
    auto d12 = it1 == uwb1_map.end() ? -1 : it1->second->nodes[ robot_id * 3 + 2].dis;
    auto d20 = it2 == uwb2_map.end() ? -1 : it2->second->nodes[ robot_id * 3 + 0].dis;
    auto d21 = it2 == uwb2_map.end() ? -1 : it2->second->nodes[ robot_id * 3 + 1].dis;

    // auto gt01 = (u0_pos - u1_pos).norm();
    // auto gt02 = (u0_pos - u2_pos).norm();
    // auto gt12 = (u1_pos - u2_pos).norm();

    std::vector<bool> unormal(3, false);

    unormal[0] = fabs(d01 - gt_dis(0, 1)) < sigma | 
                    fabs(d02 - gt_dis(0, 2)) < sigma;

    unormal[1] = fabs(d10 - gt_dis(1,0)) < sigma |
                    fabs(d12 - gt_dis(1, 2)) < sigma;

    unormal[2] = fabs(d20 - gt_dis(2, 0)) < sigma |
                    fabs(d21 - gt_dis(2, 1)) < sigma;

    // 检查当前bot 对 dest_id的bot 的各项uwb数据
    for (auto dest_id = 0; dest_id < num_robots_ ; dest_id++ ) {

      std::vector<std::map<int, nlink_parser::LinktrackNode2ConstPtr>> effect_uwb(3);

      auto & src_u0_uwbs = it0->second->nodes;
      // 遍历目标的uwb数据
      for (auto uid = 0; uid < 3 ; uid ++) {

        auto globle_uid = dest_id * 3 + uid;
        if (src_u0_uwbs.size() > globle_uid && src_u0_uwbs[globle_uid].dis > 0) {
          // 若当前uwb数据不正常，则跳过
          if (unormal[uid]) {
            continue;
          }
          // 若rx-fx > epsilon3，则跳过
          if (src_u0_uwbs[globle_uid].rx_rssi - src_u0_uwbs[globle_uid].fp_rssi > epsilon3) {
            continue;
          }
          // 前向判断 若当前dis与前一帧差值小于epsilon，则认为无效
          if (it0_prev != it0) {
            auto & src_u0_prev_uwbs = it0_prev->second->nodes;
            if (src_u0_prev_uwbs.size() > globle_uid && src_u0_prev_uwbs[globle_uid].dis > 0) {
              if (fabs(src_u0_uwbs[globle_uid].dis - src_u0_prev_uwbs[globle_uid].dis) < epsilon) {
                continue;
              }
            }
          }

          effect_uwb[0][uid] =
              boost::make_shared<nlink_parser::LinktrackNode2>(
                  &(src_u0_uwbs[globle_uid]));
        }
        
        // u1 判断
        if (it1 != uwb1_map.end()) {
          auto & src_u1_uwbs = it1->second->nodes;
          if (src_u1_uwbs.size() > globle_uid && src_u1_uwbs[globle_uid].dis > 0) {
            // 若当前uwb数据不正常，则跳过
            if (unormal[uid]) {
              continue;
            }
            // 若rx-fx > epsilon3，则跳过
            if (src_u1_uwbs[globle_uid].rx_rssi - src_u1_uwbs[globle_uid].fp_rssi > epsilon3) {
              continue;
            }
            // 前向判断 若当前dis与前一帧差值小于epsilon，则认为无效
            if (it1_prev != it1) {
              auto & src_u1_prev_uwbs = it1_prev->second->nodes;
              if (src_u1_prev_uwbs.size() > globle_uid && src_u1_prev_uwbs[globle_uid].dis > 0) {
                if (fabs(src_u1_uwbs[globle_uid].dis - src_u1_prev_uwbs[globle_uid].dis) < epsilon) {
                  continue;
                }
              }
            }
            effect_uwb[1][uid] =
                boost::make_shared<nlink_parser::LinktrackNode2>(
                    &(src_u1_uwbs[globle_uid]));
          }
        }

        // u2 判断
        if (it2 != uwb2_map.end()) {
          auto & src_u2_uwbs = it2->second->nodes;
          if (src_u2_uwbs.size() > globle_uid && src_u2_uwbs[globle_uid].dis > 0) {
            // 若当前uwb数据不正常，则跳过
            if (unormal[uid]) {
              continue;
            }
            // 若rx-fx > epsilon3，则跳过
            if (src_u2_uwbs[globle_uid].rx_rssi - src_u2_uwbs[globle_uid].fp_rssi > epsilon3) {
              continue;
            }
            // 前向判断 若当前dis与前一帧差值小于epsilon，则认为无效
            if (it2_prev != it2) {
              auto & src_u2_prev_uwbs = it2_prev->second->nodes;
              if (src_u2_prev_uwbs.size() > globle_uid && src_u2_prev_uwbs[globle_uid].dis > 0) {
                if (fabs(src_u2_uwbs[globle_uid].dis - src_u2_prev_uwbs[globle_uid].dis) < epsilon) {
                  continue;
                }
              }
            }
            effect_uwb[2][uid] =
                boost::make_shared<nlink_parser::LinktrackNode2>(
                    &(src_u2_uwbs[globle_uid]));
          }
        }
      }
      // 若有效数据小于6，则跳过此次
      auto effect_size = effect_uwb[0].size() + effect_uwb[1].size() + effect_uwb[2].size();
      if (effect_size < 6) {
        LOG(WARNING) << "UWB data is not enough, only " << effect_size
                      << " nodes";
        continue;
      }

      // 三角约束 
      std::map<std::tuple<int ,int>, int> checknum;
      std::map<std::tuple<int ,int>, int> errornum;
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

      auto error = [&](int src_id, int dest_id) {
        if (errornum.find(std::make_tuple(src_id, dest_id)) == errornum.end()) {
          LOG(WARNING) << "errornum not found";
          errornum[std::make_tuple(src_id, dest_id)] = 1;
        }
        else {
          errornum[std::make_tuple(src_id, dest_id)] += 1;
        }
      };

      // 先遍历本机 1个uwb为端点的边组
      for (auto uid = 0; uid < 3 ; uid ++) {
        auto & uimap = effect_uwb[uid];
        // 找出对应边上的不重复两两组合
        for (auto it = uimap.begin(); it != uimap.end(); ++it) {
          for (auto it2 = std::next(it); it2 != uimap.end(); ++it2) {
            auto uid1 = it->first;
            auto uid2 = it2->first;
            auto uid1_dis = it->second->dis;
            auto uid2_dis = it2->second->dis;
            
            check(uid, uid1);
            check(uid, uid2);

            auto gt_u1_u2 = gt_dis(uid1, uid2);

            // 如果 u1距离与u2 距离差值大于 3 * sqrt 2 * sigma + gt_u1_u2 
            // 即认为 uid - uid1, uid-uid2 uid1-uid2 构不成三角形 异常
            if (fabs(uid1_dis - uid2_dis) > 3 * sqrt(2) * sigma + gt_u1_u2) {
              error(uid, uid1);
              error(uid, uid2);
            }
          }
        }
      }

      // 再遍历本机 2个uwb为端点的边组
      for (auto it1_uid = 0; it1_uid < 3 ; it1_uid ++) {
        for (auto it2_uid = it1_uid + 1; it2_uid < 3 ; it2_uid ++) {
          auto & uimap1 = effect_uwb[it1_uid];
          auto & uimap2 = effect_uwb[it2_uid];
          // 找出对应边上的同时存在两个边的节点
          for (auto it1 = uimap1.begin(); it1 != uimap1.end(); ++it1) {
            auto uid1 = it1->first;
            auto uid1_dis = it1->second->dis;
            auto it2 = uimap2.find(uid1);
            if (it2 != uimap2.end()) {
              auto uid2_dis = it2->second->dis;

              check(it1_uid, uid1);
              check(it2_uid, uid1);

              auto gt_u1_u2 = gt_dis(it1_uid, it2_uid);
              // 如果 u1距离与u2 距离差值大于 3 * sqrt 2 * sigma + gt_u1_u2
              // 即认为 uid - uid1, uid-uid2 uid1-uid2 构不成三角形 异常
              if (fabs(uid1_dis - uid2_dis) > 3 * sqrt(2) * sigma + gt_u1_u2) {
                error(it1_uid, uid1);
                error(it2_uid, uid1);
              }

            }
          }
        }
      }

  

      pose_graph_tools_msgs::UWBFrame uwb;
      uwb.stamp = it0->second->stamp;
      uwb.distances.assign(9, -1.0f);
      uwb.src_robot_id = robot_id;
      uwb.dst_robot_id = dest_id;
      // 计算 errornum / checknum <= 0.5的边数量，若小于6则不发布
      int valid_num = 0;
      for ( auto &it: checknum) {
        auto &kt = it.first;
        auto &src_id = std::get<0>(kt);
        auto &dest_id = std::get<1>(kt);
        auto &check_num = it.second;
        auto &error_num = errornum[kt];

        if (error_num * 1.0 / check_num < 0.5) {
          valid_num++;
          uwb.distances[src_id * 3 + dest_id] = effect_uwb[src_id][dest_id]->dis;
        }
      }

      if (valid_num < 6) {
        LOG(WARNING) << "UWB data is not enough, only " << valid_num
                      << " nodes";
        continue;
      }
      uwb_pub_.publish(uwb);

    } 

    it0_prev = it0;
    it1_prev = it1;
    it2_prev = it2;
  }

}

bool RosbagDataProvider::spin() {
  if (k_ == 0u) {
    // Send IMU data directly to VIO for speed boost
    sendImuDataToVio();
    // Send external odometry directly to VIO as well
    if (use_external_odom_) {
      sendExternalOdometryToVio();
    }
  }

  const Timestamp last_imu =
      rosbag_data_.imu_msgs_.back()->header.stamp.toNSec();

  // We break the while loop (but increase k_!) if we run in sequential mode.
  while (k_ < rosbag_data_.left_imgs_.size()) {
    if (nh_.ok() && ros::ok() && !ros::isShuttingDown() && !shutdown_) {
      const Timestamp& timestamp_frame_k = rosbag_data_.timestamps_.at(k_);
      if (timestamp_frame_k > last_imu) {
        const auto num_left = rosbag_data_.left_imgs_.size() - k_;
        LOG(WARNING) << "Discarding " << num_left
                     << " images past last IMU message";
        k_ = rosbag_data_.left_imgs_.size();
        break;
      }

      static const CameraParams& left_cam_info =
          vio_params_.camera_params_.at(0);

      if (vio_params_.frontend_type_ == VIO::FrontendType::kRgbdImu) {
        if (k_ >= rosbag_data_.depth_imgs_.size()) {
          break;
        }
      }

      if (vio_params_.frontend_type_ == VIO::FrontendType::kStereoImu) {
        if (k_ >= rosbag_data_.right_imgs_.size()) {
          break;
        }
      }

      if (timestamp_frame_k > timestamp_last_frame_) {
        // Send left frame data to Kimera:
        CHECK(left_frame_callback_)
            << "Did you forget to register the left frame callback?";
        left_frame_callback_(std::make_unique<Frame>(
            k_,
            timestamp_frame_k,
            left_cam_info,
            readRosImage(rosbag_data_.left_imgs_.at(k_))));

        // Send right frame data to Kimera:
        if (vio_params_.frontend_type_ == VIO::FrontendType::kStereoImu) {
          CHECK(right_frame_callback_)
              << "Did you forget to register the right frame callback?";
          static const CameraParams& right_cam_info =
              vio_params_.camera_params_.at(1);
          right_frame_callback_(std::make_unique<Frame>(
              k_,
              timestamp_frame_k,
              right_cam_info,
              readRosImage(rosbag_data_.right_imgs_.at(k_))));
        }

        if (vio_params_.frontend_type_ == VIO::FrontendType::kRgbdImu) {
          CHECK(depth_frame_callback_)
              << "Did you forget to register the depth frame callback?";
          depth_frame_callback_(std::make_unique<DepthFrame>(
              k_,
              timestamp_frame_k,
              readRosDepthImage(rosbag_data_.depth_imgs_.at(k_))));
        }

        VLOG(10) << "Sent left/right images to VIO for frame k = " << k_;

        // Publish VIO output if any.
        // TODO(Toni) this could go faster if running in another thread or
        // node...

        // // Publish LCD output if any.
        // LcdOutput::Ptr lcd_output = nullptr;
        // if (lcd_output_queue_.pop(lcd_output)) {
        //   publishLcdOutput(lcd_output);
        // }

        timestamp_last_frame_ = timestamp_frame_k;
      } else {
        if (timestamp_frame_k == timestamp_last_frame_) {
          LOG(WARNING)
              << "Timestamps for current and previous frames are equal! \n"
              << "This should not happen, dropping this frame... \n "
              << "- Offending timestamp: " << timestamp_frame_k;
        } else {
          LOG(WARNING) << "Skipping frame: " << k_ << '\n'
                       << " Frame timestamps out of order:\n"
                       << " Timestamp Current Frame: " << timestamp_frame_k
                       << "\n"
                       << " Timestamp Last Frame:    " << timestamp_last_frame_;
        }
      }

      publishRosbagInfo(timestamp_frame_k);
      ros::spinOnce();
    } else {
      LOG(ERROR) << "ROS SHUTDOWN requested, stopping rosbag spin.";
      ros::shutdown();
      return false;
    }

    // Next iteration
    k_++;
    if (!vio_params_.parallel_run_) {
      // Break while loop (but keep increasing k_!) if we run in sequential
      // mode. We actually return instead of break, to avoid re-printing that
      // the rosbag processing has finished.
      return true;
    }
  }  // End of for loop over rosbag images.
  LOG(INFO) << "Rosbag processing finished.";

  shutdown();
  return true;
}

bool RosbagDataProvider::parseRosbag(const std::string& bag_path,
                                     RosbagData* rosbag_data) {
  LOG(INFO) << "Parsing rosbag data.";
  CHECK_NOTNULL(rosbag_data);

  // Fill in rosbag to data_
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  // Generate list of topics to parse:
  std::vector<std::string> topics;
  topics.push_back(left_imgs_topic_);
  if (vio_params_.frontend_type_ == FrontendType::kStereoImu) {
    topics.push_back(right_imgs_topic_);
  }
  if (vio_params_.frontend_type_ == FrontendType::kRgbdImu) {
    topics.push_back(depth_imgs_topic_);
  }

  topics.push_back(imu_topic_);
  if (!gt_odom_topic_.empty()) {
    LOG_IF(WARNING, vio_params_.backend_params_->autoInitialize_ != 0)
        << "Provided a gt_odom_topic; but autoInitialize "
           "(BackendParameters.yaml) is not set to 0,"
           " meaning no ground-truth initialization will be done... "
           "Are you sure you don't want to use gt?)";
    topics.push_back(gt_odom_topic_);
  } else {
    // TODO(Toni): autoinit should be a bool...
    CHECK_EQ(vio_params_.backend_params_->autoInitialize_, 1)
        << "Requested ground-truth initialization, but no gt_odom_topic "
           "was given. Make sure you set ground_truth_odometry_rosbag_topic, "
           "or turn autoInitialize to false in BackendParameters.yaml.";
  }
  if (use_external_odom_) {
    topics.push_back(external_odom_topic_);
  }
  topics.push_back(uwb_topic_ + "/0");
  topics.push_back(uwb_topic_ + "/1");
  topics.push_back(uwb_topic_ + "/2");

  std::stringstream ss;
  ss << "query topics:" << std::endl;
  ss << "=============" << std::endl;
  for (const auto& topic : topics) {
    ss << " - " << topic << std::endl;
  }
  VLOG(2) << ss.str();

  // Query rosbag for given topics
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  int imu_msg_count = 0;
  // Keep track of this since we expect IMU data before an image.
  bool start_parsing_stereo = false;
  // For some datasets, we have duplicated measurements for the same time.
  Timestamp last_imu_timestamp = 0;
  for (const rosbag::MessageInstance& msg : view) {
    const std::string& msg_topic = msg.getTopic();

    // Check if msg is an IMU measurement.
    sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
    if (imu_msg != nullptr && msg_topic == imu_topic_) {
      const ImuStamp& imu_data_timestamp = imu_msg->header.stamp.toNSec();
      if (imu_data_timestamp > last_imu_timestamp) {
        VLOG(10) << "IMU msg count: " << imu_msg_count++;
        rosbag_data->imu_msgs_.push_back(imu_msg);
        last_imu_timestamp = imu_data_timestamp;
      } else {
        if (imu_data_timestamp - last_imu_timestamp == 0u) {
          LOG(WARNING) << "IMU timestamps in rosbag are repeated!\n"
                       << "Offending timestamp: " << imu_data_timestamp;
        } else {
          LOG(FATAL) << "IMU timestamps in rosbag are out of order: consider "
                     << "re-ordering rosbag: \n"
                     << "- Current IMU timestamp: " << imu_data_timestamp
                     << '\n'
                     << "- Last IMU timestamp: " << last_imu_timestamp << '\n'
                     << "Difference (current - last) = "
                     << imu_data_timestamp - last_imu_timestamp;
        }
      }
      start_parsing_stereo = true;
      continue;
    }

    // Check if msg is an image.
    sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
    if (img_msg != nullptr) {
      if (start_parsing_stereo) {
        // Check left or right image.
        if (msg_topic == left_imgs_topic_) {
          // Timestamp is in nanoseconds
          rosbag_data->timestamps_.push_back(img_msg->header.stamp.toNSec());
          rosbag_data->left_imgs_.push_back(img_msg);
        } else if (vio_params_.frontend_type_ == FrontendType::kStereoImu &&
                   msg_topic == right_imgs_topic_) {
          rosbag_data->right_imgs_.push_back(img_msg);
        } else if (vio_params_.frontend_type_ == FrontendType::kRgbdImu &&
                   msg_topic == depth_imgs_topic_) {
          rosbag_data->depth_imgs_.push_back(img_msg);
        } else {
          LOG(WARNING) << "Img with unexpected topic: " << msg_topic;
        }
      } else {
        LOG(WARNING) << "Skipping first frame in rosbag, since IMU data not "
                        "yet available.";
      }
      continue;
    }

    // Check if msg is a ground-truth odometry message.
    nav_msgs::OdometryConstPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();
    if (odom_msg != nullptr) {
      // handle gt
      if (msg_topic == gt_odom_topic_) {
        rosbag_data->gt_odometry_.push_back(odom_msg);
        if (log_gt_data_) {
          logGtData(odom_msg);
        }
      } else if (msg_topic != external_odom_topic_) {
        LOG(ERROR) << "Unrecognized topic name for odometry msg. We were"
                      " expecting ground-truth odometry on this topic: "
                   << msg_topic;
      }
      // handle odom
      if (use_external_odom_ && msg_topic == external_odom_topic_) {
        rosbag_data->external_odom_.push_back(odom_msg);
      }
    } else {
      LOG(ERROR) << "Could not find the type of this rosbag msg from topic:\n"
                 << msg_topic;
    }

    // 检查是否为0号UWB消息
    nlink_parser::LinktrackNodeframe2ConstPtr uwb_msg =
        msg.instantiate<nlink_parser::LinktrackNodeframe2>();
    if (uwb_msg != nullptr) {
      if (msg_topic == uwb_topic_ + "/0") {
        rosbag_data->uwb0_msgs_.push_back(uwb_msg);
      } else if (msg_topic == uwb_topic_ + "/1") {
        rosbag_data->uwb1_msgs_.push_back(uwb_msg);
      } else if (msg_topic == uwb_topic_ + "/2") {
        rosbag_data->uwb2_msgs_.push_back(uwb_msg);
      } else {
        LOG(ERROR) << "Unrecognized topic name for UWB msg. We were"
                      " expecting ground-truth odometry on this topic: "
                   << msg_topic;
      }
    } else {
      LOG(ERROR) << "Could not find the type of this rosbag msg from topic:\n"
                 << msg_topic;
    }
  }
  bag.close();

  // Sanity check:
  LOG_IF(FATAL,
         rosbag_data->left_imgs_.size() == 0 ||
             (vio_params_.frontend_type_ == FrontendType::kStereoImu &&
              rosbag_data->right_imgs_.size() == 0))
      << "No images parsed from rosbag.";
  if (vio_params_.frontend_type_ == FrontendType::kStereoImu)
    LOG_IF(FATAL,
           rosbag_data->left_imgs_.size() != rosbag_data->right_imgs_.size())
        << "Unequal number of images from left and right cameras.";
  LOG_IF(FATAL, rosbag_data->imu_msgs_.size() <= rosbag_data->left_imgs_.size())
      << "Less than or equal number of imu data as image data.";
  LOG_IF(FATAL,
         !gt_odom_topic_.empty() && rosbag_data->gt_odometry_.size() == 0)
      << "Requested to parse ground-truth odometry, but parsed 0 msgs.";
  LOG_IF(WARNING,
         !gt_odom_topic_.empty() &&
             rosbag_data->gt_odometry_.size() < rosbag_data->left_imgs_.size())
      << "Fewer ground_truth data than image data.";
  LOG_IF(
      WARNING,
      !external_odom_topic_.empty() && use_external_odom_ &&
          rosbag_data->external_odom_.size() < rosbag_data->left_imgs_.size())
      << "Fewer external odometry messages than image data.";
  LOG(INFO) << "Finished parsing rosbag data.";
  return true;
}

VioNavState RosbagDataProvider::getGroundTruthVioNavState(
    const size_t& k_frame) const {
  CHECK_LT(k_frame, rosbag_data_.gt_odometry_.size());
  nav_msgs::Odometry gt_odometry = *(rosbag_data_.gt_odometry_.at(k_frame));
  VioNavState vio_nav_state;
  utils::rosOdometryToVioNavState(gt_odometry, nh_private_, &vio_nav_state);
  return vio_nav_state;
}

void RosbagDataProvider::publishRosbagInfo(const Timestamp& timestamp) {
  publishInputs(timestamp);
  publishClock(timestamp);
}

void RosbagDataProvider::publishClock(const Timestamp& timestamp) const {
  rosgraph_msgs::Clock clock;
  clock.clock.fromNSec(timestamp);
  clock_pub_.publish(clock);
}

void RosbagDataProvider::publishInputs(const Timestamp& timestamp_kf) {
  // Publish all imu messages to ROS:
  if (k_last_imu_ < rosbag_data_.imu_msgs_.size()) {
    while (timestamp_last_imu_ < timestamp_kf &&
           k_last_imu_ < rosbag_data_.imu_msgs_.size()) {
      imu_pub_.publish(rosbag_data_.imu_msgs_.at(k_last_imu_));
      timestamp_last_imu_ =
          rosbag_data_.imu_msgs_.at(k_last_imu_)->header.stamp.toNSec();
      k_last_imu_++;
    }
  }

  // Publish ground-truth data if available:
  if (k_last_gt_ < rosbag_data_.gt_odometry_.size()) {
    while (timestamp_last_gt_ < timestamp_kf &&
           k_last_gt_ < rosbag_data_.gt_odometry_.size()) {
      gt_odometry_pub_.publish(rosbag_data_.gt_odometry_.at(k_last_gt_));
      timestamp_last_gt_ =
          rosbag_data_.gt_odometry_.at(k_last_gt_)->header.stamp.toNSec();
      k_last_gt_++;
    }
  }

  // Publish external odometry data if available:
  if (k_last_odom_ < rosbag_data_.external_odom_.size()) {
    while (timestamp_last_odom_ < timestamp_kf &&
           k_last_odom_ < rosbag_data_.external_odom_.size()) {
      external_odometry_pub_.publish(
          rosbag_data_.external_odom_.at(k_last_odom_));
      timestamp_last_odom_ =
          rosbag_data_.external_odom_.at(k_last_odom_)->header.stamp.toNSec();
      k_last_odom_++;
    }
  }

  // Publish input images if available:
  switch (vio_params_.frontend_type_) {
    case FrontendType::kMonoImu: {
      // Publish left images:
      publishMonoImages(timestamp_kf);
      break;
    }
    case FrontendType::kStereoImu: {
      // Publish left and right images:
      publishStereoImages(timestamp_kf);
      break;
    }
    case FrontendType::kRgbdImu: {
      break;
    }
    default: {
      LOG(FATAL) << "Don't know this frontend type.";
      break;
    }
  }
}

void RosbagDataProvider::publishMonoImages(const Timestamp& timestamp_kf) {
  if (k_last_kf_ < rosbag_data_.left_imgs_.size()) {
    while (timestamp_last_kf_ < timestamp_kf &&
           k_last_kf_ < rosbag_data_.left_imgs_.size()) {
      left_img_pub_.publish(rosbag_data_.left_imgs_.at(k_last_kf_));
      timestamp_last_kf_ = rosbag_data_.timestamps_.at(k_last_kf_);
      k_last_kf_++;
    }
  }
}

void RosbagDataProvider::publishStereoImages(const Timestamp& timestamp_kf) {
  if (k_last_kf_ < rosbag_data_.left_imgs_.size() &&
      k_last_kf_ < rosbag_data_.right_imgs_.size()) {
    while (timestamp_last_kf_ < timestamp_kf &&
           k_last_kf_ < rosbag_data_.left_imgs_.size() &&
           k_last_kf_ < rosbag_data_.right_imgs_.size()) {
      left_img_pub_.publish(rosbag_data_.left_imgs_.at(k_last_kf_));
      right_img_pub_.publish(rosbag_data_.right_imgs_.at(k_last_kf_));
      timestamp_last_kf_ = rosbag_data_.timestamps_.at(k_last_kf_);
      k_last_kf_++;
    }
  }
}

}  // namespace VIO

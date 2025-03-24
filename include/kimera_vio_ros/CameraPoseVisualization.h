/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All
 * Rights Reserved
 * @Author: Ronghai He
 * @Date: 2025-03-20 21:09:43
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-03-20 23:00:29
 * @FilePath: /src/kimera_ros/include/kimera_vio_ros/CameraPoseVisualization.h
 * @Version:
 * @Description:
 *
 */
/**
 * @file   CameraPoseVisualization.h
 * @brief  Class to visualize camera poses as frustums
 * @author Ronghai He with copilot
 */

#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>

namespace VIO {

class CameraPoseVisualization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Static camera frustum points in camera frame
  static const Eigen::Vector3d imlt;  // Image left top
  static const Eigen::Vector3d imrt;  // Image right top
  static const Eigen::Vector3d imlb;  // Image left bottom
  static const Eigen::Vector3d imrb;  // Image right bottom
  static const Eigen::Vector3d lt0;   // Left indicator
  static const Eigen::Vector3d lt1;
  static const Eigen::Vector3d lt2;
  static const Eigen::Vector3d oc;  // Optical center

  CameraPoseVisualization();
  CameraPoseVisualization(Eigen::Vector3d rgb, float alpha = 1.0);

  // Set visualization parameters
  void setImageBoundaryColor(float r, float g, float b, float a = 1.0);
  void setOpticalCenterConnectorColor(float r, float g, float b, float a = 1.0);
  void setScale(const double scale);
  void setLineWidth(double width);

  // Add geometric elements
  void addPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
  void addPose(const Eigen::Vector3d& p,
               const Eigen::Quaterniond& q,
               const Eigen::Vector3d& color,
               double alpha = 1.0);
  void addEdge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
  void addLoopEdge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

  // Reset and publish
  void reset();
  //   void publishBy(ros::Publisher& pub, const std_msgs::Header& header);

  std::vector<visualization_msgs::Marker> markers_;

 private:
  std_msgs::ColorRGBA image_boundary_color_;
  std_msgs::ColorRGBA optical_center_connector_color_;
  double camera_scale_;
  double camera_line_width_;
};

}  // namespace VIO

/**
 * @file   CameraPoseVisualization.cpp
 * @brief  Implementation for camera frustum visualization
 * @author Ronghai He with copilot
 */

#include "kimera_vio_ros/CameraPoseVisualization.h"

namespace VIO {

// Define static frustum points in camera frame
const Eigen::Vector3d CameraPoseVisualization::imlt =
    Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrt =
    Eigen::Vector3d(1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imlb =
    Eigen::Vector3d(-1.0, 0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrb =
    Eigen::Vector3d(1.0, 0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt0 =
    Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt1 =
    Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt2 =
    Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::oc =
    Eigen::Vector3d(0.0, 0.0, 0.0);

// Helper function to convert Eigen::Vector3d to geometry_msgs::Point
void Eigen2Point(const Eigen::Vector3d& v, geometry_msgs::Point& p) {
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
}

CameraPoseVisualization::CameraPoseVisualization()
    : camera_scale_(0.2), camera_line_width_(0.01) {
  image_boundary_color_.r = 1.0;
  image_boundary_color_.g = 0.0;
  image_boundary_color_.b = 0.0;
  image_boundary_color_.a = 1.0;
  optical_center_connector_color_.r = 1.0;
  optical_center_connector_color_.g = 0.0;
  optical_center_connector_color_.b = 0.0;
  optical_center_connector_color_.a = 1.0;
}

CameraPoseVisualization::CameraPoseVisualization(Eigen::Vector3d rgb, float a)
    : camera_scale_(0.2), camera_line_width_(0.1) {
  image_boundary_color_.r = rgb.x();
  image_boundary_color_.g = rgb.y();
  image_boundary_color_.b = rgb.z();
  image_boundary_color_.a = a;
  optical_center_connector_color_.r = rgb.x();
  optical_center_connector_color_.g = rgb.y();
  optical_center_connector_color_.b = rgb.z();
  optical_center_connector_color_.a = a;
}

void CameraPoseVisualization::setImageBoundaryColor(float r,
                                                    float g,
                                                    float b,
                                                    float a) {
  image_boundary_color_.r = r;
  image_boundary_color_.g = g;
  image_boundary_color_.b = b;
  image_boundary_color_.a = a;
}

void CameraPoseVisualization::setOpticalCenterConnectorColor(float r,
                                                             float g,
                                                             float b,
                                                             float a) {
  optical_center_connector_color_.r = r;
  optical_center_connector_color_.g = g;
  optical_center_connector_color_.b = b;
  optical_center_connector_color_.a = a;
}

void CameraPoseVisualization::setScale(const double s) { camera_scale_ = s; }

void CameraPoseVisualization::setLineWidth(double width) {
  camera_line_width_ = width;
}

void CameraPoseVisualization::addEdge(const Eigen::Vector3d& p0,
                                      const Eigen::Vector3d& p1) {
  visualization_msgs::Marker marker;

  marker.id = markers_.size() + 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.005;

  marker.color.g = 1.0f;
  marker.color.a = 1.0;

  geometry_msgs::Point point0, point1;

  Eigen2Point(p0, point0);
  Eigen2Point(p1, point1);

  marker.points.push_back(point0);
  marker.points.push_back(point1);

  markers_.push_back(marker);
}

void CameraPoseVisualization::addLoopEdge(const Eigen::Vector3d& p0,
                                          const Eigen::Vector3d& p1) {
  visualization_msgs::Marker marker;

  //   marker.ns = m_marker_ns;
  marker.id = markers_.size() + 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.04;

  marker.color.r = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  geometry_msgs::Point point0, point1;

  Eigen2Point(p0, point0);
  Eigen2Point(p1, point1);

  marker.points.push_back(point0);
  marker.points.push_back(point1);

  markers_.push_back(marker);
}

void CameraPoseVisualization::addPose(const Eigen::Vector3d& p,
                                      const Eigen::Quaterniond& q) {
  visualization_msgs::Marker marker;

  //   marker.ns = m_marker_ns;
  marker.id = markers_.size() + 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = camera_line_width_;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  geometry_msgs::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1,
      pt_lt2;

  Eigen2Point(q * (camera_scale_ * imlt) + p, pt_lt);
  Eigen2Point(q * (camera_scale_ * imlb) + p, pt_lb);
  Eigen2Point(q * (camera_scale_ * imrt) + p, pt_rt);
  Eigen2Point(q * (camera_scale_ * imrb) + p, pt_rb);
  Eigen2Point(q * (camera_scale_ * lt0) + p, pt_lt0);
  Eigen2Point(q * (camera_scale_ * lt1) + p, pt_lt1);
  Eigen2Point(q * (camera_scale_ * lt2) + p, pt_lt2);
  Eigen2Point(q * (camera_scale_ * oc) + p, pt_oc);

  // image boundaries
  marker.points.push_back(pt_lt);
  marker.points.push_back(pt_lb);
  marker.colors.push_back(image_boundary_color_);
  marker.colors.push_back(image_boundary_color_);

  marker.points.push_back(pt_lb);
  marker.points.push_back(pt_rb);
  marker.colors.push_back(image_boundary_color_);
  marker.colors.push_back(image_boundary_color_);

  marker.points.push_back(pt_rb);
  marker.points.push_back(pt_rt);
  marker.colors.push_back(image_boundary_color_);
  marker.colors.push_back(image_boundary_color_);

  marker.points.push_back(pt_rt);
  marker.points.push_back(pt_lt);
  marker.colors.push_back(image_boundary_color_);
  marker.colors.push_back(image_boundary_color_);

  // top-left indicator
  marker.points.push_back(pt_lt0);
  marker.points.push_back(pt_lt1);
  marker.colors.push_back(image_boundary_color_);
  marker.colors.push_back(image_boundary_color_);

  marker.points.push_back(pt_lt1);
  marker.points.push_back(pt_lt2);
  marker.colors.push_back(image_boundary_color_);
  marker.colors.push_back(image_boundary_color_);

  // optical center connector
  marker.points.push_back(pt_lt);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(optical_center_connector_color_);
  marker.colors.push_back(optical_center_connector_color_);

  marker.points.push_back(pt_lb);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(optical_center_connector_color_);
  marker.colors.push_back(optical_center_connector_color_);

  marker.points.push_back(pt_rt);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(optical_center_connector_color_);
  marker.colors.push_back(optical_center_connector_color_);

  marker.points.push_back(pt_rb);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(optical_center_connector_color_);
  marker.colors.push_back(optical_center_connector_color_);

  markers_.push_back(marker);
}

void CameraPoseVisualization::addPose(const Eigen::Vector3d& p,
                                      const Eigen::Quaterniond& q,
                                      const Eigen::Vector3d& color,
                                      double alpha) {
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color_pose;
  color_pose.r = color.x();
  color_pose.g = color.y();
  color_pose.b = color.z();
  color_pose.a = alpha;

  //   marker.ns = m_marker_ns;
  marker.id = markers_.size() + 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = camera_line_width_;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  geometry_msgs::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1,
      pt_lt2;

  Eigen2Point(q * (camera_scale_ * imlt) + p, pt_lt);
  Eigen2Point(q * (camera_scale_ * imlb) + p, pt_lb);
  Eigen2Point(q * (camera_scale_ * imrt) + p, pt_rt);
  Eigen2Point(q * (camera_scale_ * imrb) + p, pt_rb);
  Eigen2Point(q * (camera_scale_ * lt0) + p, pt_lt0);
  Eigen2Point(q * (camera_scale_ * lt1) + p, pt_lt1);
  Eigen2Point(q * (camera_scale_ * lt2) + p, pt_lt2);
  Eigen2Point(q * (camera_scale_ * oc) + p, pt_oc);

  // image boundaries
  marker.points.push_back(pt_lt);
  marker.points.push_back(pt_lb);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  marker.points.push_back(pt_lb);
  marker.points.push_back(pt_rb);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  marker.points.push_back(pt_rb);
  marker.points.push_back(pt_rt);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  marker.points.push_back(pt_rt);
  marker.points.push_back(pt_lt);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  // top-left indicator
  marker.points.push_back(pt_lt0);
  marker.points.push_back(pt_lt1);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  marker.points.push_back(pt_lt1);
  marker.points.push_back(pt_lt2);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  // optical center connector
  marker.points.push_back(pt_lt);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  marker.points.push_back(pt_lb);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  marker.points.push_back(pt_rt);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  marker.points.push_back(pt_rb);
  marker.points.push_back(pt_oc);
  marker.colors.push_back(color_pose);
  marker.colors.push_back(color_pose);

  markers_.push_back(marker);
}

void CameraPoseVisualization::reset() { markers_.clear(); }

// void CameraPoseVisualization::publishBy(ros::Publisher& pub,
//                                         const std_msgs::Header& header) {
//   visualization_msgs::MarkerArray markerArray_msg;

//   for (auto& marker : markers_) {
//     marker.header = header;
//     markerArray_msg.markers.push_back(marker);
//   }

//   pub.publish(markerArray_msg);
// }

}  // namespace VIO

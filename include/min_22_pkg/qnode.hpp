/**
 * @file /include/hsv_detect/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date January 2025
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef min_22_pkg_QNODE_HPP_
#define min_22_pkg_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QThread>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlebot3_msgs/msg/sensor_state.hpp>
// #include <mobile_base_msgs/msg/mani_vision.hpp>


/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread {
  Q_OBJECT
 public:
  QNode();
  ~QNode();

  cv::Mat* imgRaw = NULL;   // 원본 이미지 저장
  bool isreceived = false;  // 이미지 수신 여부
  std::atomic<bool> running_;
  std::shared_ptr<rclcpp::Node> node;

  std::mutex img_mutex;
  std::vector<cv::Point> pts;
  cv::Rect rect;

  uint16_t depth_in_mm = 0;
  uint16_t max_depth = 0;
  uint16_t min_depth = 0;

  double current_linear_x = 0.0;   // 전진 속도
  double current_linear_y = 0.0;   // 측면 속도
  double current_angular_z = 0.0;  // 회전 속도
  bool speed_received = false;

  std::vector<float> lidar_ranges;
  float lidar_angle_min;
  float lidar_angle_max;
  float lidar_angle_increment;
  int lidar_range_size;
  bool lidar_received = false;
  bool detectObstacle(double min_distance = 0.5, double angle_range = 0.52) const;
  bool detectObstacleInSector(double min_distance, double angle_center, double angle_range) const;
  float getMinObstacleDistance(double angle_range = 0.52) const;

  void drive(double linear_x, double angular_z);

 protected:
  void run();

 private:
  void initPubSub();

  //static bool ros_initialized;


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  // rclcpp::Publisher<mobile_base_msgs::msg::mani_vision>::SharedPtr mani_vision_pub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img);
  // void callbackDepth(const sensor_msgs::msg::Image::SharedPtr image_msg);
  // void callbackCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr info_msg);
  void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void callbackLidar(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

 Q_SIGNALS:
  void rosShutDown();
  void sigRcvImg();
  void speedUpdated();
  void lidarReceived();
};

#endif /* hsv_detect_QNODE_HPP_ */

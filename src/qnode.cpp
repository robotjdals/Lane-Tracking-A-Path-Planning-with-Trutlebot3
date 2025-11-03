/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date January 2025
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/min_22_pkg/qnode.hpp"

//bool QNode::ros_initialized = false;

QNode::QNode() {

  running_ = false;
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("min_22_pkg");
  running_ = true;
  this->start();
  initPubSub();
}

void QNode::initPubSub() {
  image_sub_ = node->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&QNode::callbackImage, this, std::placeholders::_1));
  // depth_image_sub_ = node->create_subscription<sensor_msgs::msg::Image>("camera/aligned_depth_to_color/image_raw", 10, std::bind(&QNode::callbackDepth, this, std::placeholders::_1));
  // camera_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>("camera/aligned_depth_to_color/camera_info", 10, std::bind(&QNode::callbackCameraInfo, this, std::placeholders::_1));

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,  std::bind(&QNode::callbackOdom, this, std::placeholders::_1));
  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  lidar_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&QNode::callbackLidar, this, std::placeholders::_1));
}

QNode::~QNode() {
  std::cout << "QNode destructor called" << std::endl;  // 디버그 추가

  running_ = false;  // 스레드 종료 신호

  if (this->isRunning()) {
    this->quit();
    this->wait(3000);  // 3초 대기
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
    std::cout << "QNode thread started" << std::endl;  // 디버그 추가

  rclcpp::WallRate loop_rate(20);
  while (running_ && rclcpp::ok()) {  // running_ 조건 추가
    try {
      rclcpp::spin_some(node);

      if (!running_) break;  // 안전한 종료 체크

      loop_rate.sleep();
    } catch (const std::exception& e) {
      std::cout << "Error in spin: " << e.what() << std::endl;
      break;  // 에러 발생시 루프 종료
    }
  }

  std::cout << "QNode thread ending" << std::endl;  // 디버그 추가

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  Q_EMIT rosShutDown();
}
  /*
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
  */

void QNode::callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img)
{
  std::lock_guard<std::mutex> lock(img_mutex);

  if (imgRaw == NULL && !isreceived)  // imgRaw -> NULL, isreceived -> false
  {
    try {
      // ROS2 이미지 메시지를 OpenCV Mat 형식으로 변환, 이미지 객체에 할당
      imgRaw = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);

      if (imgRaw != NULL)  // imgRaw 변환 성공
      {
        Q_EMIT sigRcvImg();  // 이미지 수신을 알리는 시그널 발생
        isreceived = true;
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
}

void QNode::callbackOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // Twist 메시지에서 속도 정보 추출
    current_linear_x = odom_msg->twist.twist.linear.x;   // 전진 속도
    current_linear_y = odom_msg->twist.twist.linear.y;   // 측면 속도
    current_angular_z = odom_msg->twist.twist.angular.z; // 회전 속도

    speed_received = true;

    emit speedUpdated();
}

void QNode::drive(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_x;    // 전진/후진 속도
    twist_msg.angular.z = angular_z;  // 회전 속도

    cmd_vel_pub_->publish(twist_msg);
}

void QNode::callbackLidar(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  lidar_ranges = scan_msg->ranges;
  lidar_angle_min = scan_msg->angle_min;
  lidar_angle_max = scan_msg->angle_max;
  lidar_angle_increment = scan_msg->angle_increment; //각도
  lidar_range_size = lidar_ranges.size();

  lidar_received = true;
  // 시그널 발생 (UI 업데이트용)
  emit lidarReceived();
}

bool QNode::detectObstacle(double min_distance, double angle_range) const {
    if (!lidar_received || lidar_ranges.empty()) return false;

    int center_index = lidar_ranges.size() / 2;
    int range_indices = static_cast<int>(angle_range / lidar_angle_increment);

    int start_idx = std::max(0, center_index - range_indices);
    int end_idx = std::min((int)lidar_ranges.size()-1, center_index + range_indices);

    for (int i = start_idx; i <= end_idx; i++) {
      float range = lidar_ranges[i];
      if (range > 0.05 && range < min_distance && !std::isinf(range)) {
          return true;
      }
    }
    return false;
}

bool QNode::detectObstacleInSector(double min_distance, double angle_center, double angle_range) const {
    if (!lidar_received || lidar_ranges.empty()) return false;

    int center_angle_index = static_cast<int>((angle_center - lidar_angle_min) / lidar_angle_increment);
    int range_indices = static_cast<int>(angle_range / lidar_angle_increment);

    int start_idx = std::max(0, center_angle_index - range_indices);
    int end_idx = std::min((int)lidar_ranges.size()-1, center_angle_index + range_indices);

    for (int i = start_idx; i <= end_idx; i++) {
        float range = lidar_ranges[i];
        if (range > 0.05 && range < min_distance && !std::isinf(range)) {
            return true;
        }
    }
    return false;
}

float QNode::getMinObstacleDistance(double angle_range) const {
    if (!lidar_received || lidar_ranges.empty()) return 10.0; // 최대값 반환

    int center_index = lidar_ranges.size() / 2;
    int range_indices = static_cast<int>(angle_range / lidar_angle_increment);

    int start_idx = std::max(0, center_index - range_indices);
    int end_idx = std::min((int)lidar_ranges.size()-1, center_index + range_indices);

    float min_dist = 10.0; // Lidar max range (10.0m) 사용

    for (int i = start_idx; i <= end_idx; i++) {
        float range = lidar_ranges[i];
        if (range > 0.05 && !std::isinf(range) && !std::isnan(range)) {
            min_dist = std::min(min_dist, range);
        }
    }
    return min_dist;
}


// void QNode::callbackDepth(const sensor_msgs::msg::Image::SharedPtr image_msg)
// {
//   cv::Mat image;
//   cv_bridge::CvImagePtr input_bridge;
//   try {
//     input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
//     image = input_bridge->image;
//   } catch (cv_bridge::Exception& e) {
//     RCLCPP_ERROR(node->get_logger(), "[draw_frames] Failed to convert image: %s", e.what());
//     return;
//   }

//   // 이미지의 중심 좌표 계산
//   int center_x = image.cols / 2;
//   int center_y = image.rows / 2;

//   max_depth = std::numeric_limits<short int>::min();
//   min_depth = std::numeric_limits<short int>::max();

//   for (int y = 0; y < image.rows; ++y) {
//     for (int x = 0; x < image.cols; ++x) {
//       float depth = image.at<short int>(cv::Point(x, y));

//       if (depth == 0) continue;

//       if (depth > max_depth) {
//         max_depth = depth;
//       }
//       if (depth < min_depth) {
//         min_depth = depth;
//       }
//     }
//   }

//   depth_in_mm = image.at<short int>(cv::Point(center_x, center_y));
// }

// void QNode::callbackCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr info_msg)
// {
//   // 카메라 정보 처리 (필요한 경우 구현)
//   // 예: 카메라 파라미터 추출 등
// }

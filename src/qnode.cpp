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
// tf2 헤더 대신 기본 수학 라이브러리 사용
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm> // std::min, std::max

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

// 쿼터니언을 오일러 각도로 변환하는 간단한 함수 (tf2 라이브러리 대신 사용)
double quaternionToYaw(double x, double y, double z, double w) {
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

void QNode::callbackOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // Twist 메시지에서 속도 정보 추출
    current_linear_x = odom_msg->twist.twist.linear.x;   // 전진 속도
    current_linear_y = odom_msg->twist.twist.linear.y;   // 측면 속도
    current_angular_z = odom_msg->twist.twist.angular.z; // 회전 속도
    speed_received = true;

    // 🌍 Odometry 위치 (절대 좌표)
    odom_x = odom_msg->pose.pose.position.x;
    odom_y = odom_msg->pose.pose.position.y;

    // 🌍 Odometry 자세 (쿼터니언 -> Yaw 각도)
    // tf2 라이브러리 대신 직접 계산
    odom_yaw = quaternionToYaw(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );

    odom_received = true; // Odometry 수신 완료 플래그 설정

    // 🔧 로봇 위치 디버깅 출력 (매 10번째만 출력하여 스팸 방지)
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0) {
        std::cout << "\n🤖 ROBOT POSITION DEBUG:" << std::endl;
        std::cout << "   World Position: (" << std::fixed << std::setprecision(6)
                  << odom_x << ", " << odom_y << ", " << odom_yaw << ")" << std::endl;

        // 참고: YAML 좌표계 변환은 A* planner에서 처리됨
        // map2.yaml: resolution=0.00506991, origin=[1.01712, 0.0240227]
        std::cout << "   ✅ Coordinate system managed by A* planner" << std::endl;
    }

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
/*
bool QNode::detectObstacle(double min_distance, double angle_range) const {
    if (!lidar_received || lidar_ranges.empty()) return false;

    int center_index = lidar_ranges.size() / 2;
    // Radian -> Index 변환 시 Angle_increment로 나누어 계산
    int range_indices = static_cast<int>((angle_range / 2.0) / lidar_angle_increment);

    int start_idx = std::max(0, center_index - range_indices);
    int end_idx = std::min((int)lidar_ranges.size()-1, center_index + range_indices);

    for (int i = start_idx; i <= end_idx; i++) {
      float range = lidar_ranges[i];
      if (range > 0.05 && range < min_distance && !std::isinf(range) && !std::isnan(range)) {
          return true;
      }
    }
    return false;
}*/

bool QNode::detectObstacle(double min_distance, double angle_range_rad) const {
    if (!lidar_received || lidar_ranges.empty()) return false;
    if (std::isnan(lidar_angle_increment) || lidar_angle_increment == 0.0) return false;

    // angle_increment가 음수인 경우를 방어
    const double inc = (lidar_angle_increment > 0) ? lidar_angle_increment : -lidar_angle_increment;
    const double a_min = (lidar_angle_increment > 0) ? lidar_angle_min : lidar_angle_max;

    // 정면(0 rad)의 인덱스 = (0 - angle_min) / angle_increment
    int front_index = static_cast<int>(std::lround((0.0 - a_min) / inc));

    // 검색 범위 인덱스 폭 (라디안 -> 인덱스)
    if (angle_range_rad <= 0.0) angle_range_rad = inc; // 최소 한 빔이라도 보게
    int half_span = std::max(1, static_cast<int>(std::lround((angle_range_rad * 0.5) / inc)));

    int start_idx = std::max(0, front_index - half_span);
    int end_idx   = std::min(static_cast<int>(lidar_ranges.size()) - 1, front_index + half_span);

    for (int i = start_idx; i <= end_idx; ++i) {
        float r = lidar_ranges[i];
        if (r > 0.05f && !std::isinf(r) && !std::isnan(r) && r < static_cast<float>(min_distance)) {
            return true;
        }
    }
    return false;
}

bool QNode::detectObstacleInSector(double min_distance, double angle_center, double angle_range) const {
    if (!lidar_received || lidar_ranges.empty()) return false;

    // 절대각(angle_center)을 인덱스로 변환
    int center_angle_index = static_cast<int>((angle_center - lidar_angle_min) / lidar_angle_increment);
    int range_indices = static_cast<int>((angle_range / 2.0) / lidar_angle_increment);

    int start_idx = std::max(0, center_angle_index - range_indices);
    int end_idx = std::min((int)lidar_ranges.size()-1, center_angle_index + range_indices);

    for (int i = start_idx; i <= end_idx; i++) {
        float range = lidar_ranges[i];
        if (range > 0.05 && range < min_distance && !std::isinf(range) && !std::isnan(range)) {
            return true;
        }
    }
    return false;
}

float QNode::getMinObstacleDistance(double angle_range) const {
    if (!lidar_received || lidar_ranges.empty()) return 10.0; // 최대값 반환

    int center_index = lidar_ranges.size() / 2;
    int range_indices = static_cast<int>((angle_range / 2.0) / lidar_angle_increment);

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

float QNode::getMinObstacleDistanceInSector(double angle_center, double angle_range) const {
    if (!lidar_received || lidar_ranges.empty()) return 10.0;

    int center_angle_index = static_cast<int>((angle_center - lidar_angle_min) / lidar_angle_increment);
    int range_indices = static_cast<int>((angle_range / 2.0) / lidar_angle_increment);

    int start_idx = std::max(0, center_angle_index - range_indices);
    int end_idx = std::min((int)lidar_ranges.size()-1, center_angle_index + range_indices);

    float min_dist = 10.0;

    for (int i = start_idx; i <= end_idx; i++) {
        float range = lidar_ranges[i];
        if (range > 0.05 && !std::isinf(range) && !std::isnan(range)) {
            min_dist = std::min(min_dist, range);
        }
    }
    return min_dist;
}


/*
// void QNode::callbackDepth(const sensor_msgs::msg::Image::SharedPtr image_msg)
// {
//   //... (주석 처리된 기존 코드)
// }

// void QNode::callbackCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr info_msg)
// {
//   //... (주석 처리된 기존 코드)
// }
*/

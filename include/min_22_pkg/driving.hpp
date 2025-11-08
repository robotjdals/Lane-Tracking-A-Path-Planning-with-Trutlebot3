#ifndef DRIVING_HPP
#define DRIVING_HPP

#include <QObject>
#include <chrono>
#include "qnode.hpp"
#include <vector>
#include <opencv2/core.hpp> // cv::Point2f 사용을 위해

class MainWindow;
class Astar;

class Driving : public QObject {
  Q_OBJECT

 public:
  Driving(QObject* parent = nullptr);
  ~Driving() = default;

  enum Current_state{
    LANE_TRACKING,
    PLANNING,      // 새로운 상태: 경로 계획 중
    AVOIDANCE,
    RETURN_LANE,
    PATH_TRACK     // A* 경로 추종 상태
  };

  void setQNode(QNode* qnode_ptr);
  void setMainWindow(MainWindow* main_window_ptr);
  void setPlanner(Astar* planner);

  static constexpr double pixel_to_meter = -0.0018;
  static constexpr double w_lim = 1.8;
  static constexpr double b = 0.16; // 휠베이스
  double current_speed;

  std::vector<cv::Point2f>a_waypoints;
  cv::Point2f start, goal;
  int current_path_index = 0;
  Current_state state = LANE_TRACKING;

  // 🌍 글로벌 경로 저장용 변수들
  std::chrono::steady_clock::time_point path_start_time;  // 경로 시작 시간
  bool global_path_ready = false;
  bool has_avoidance_goal = false;                        // 회피 목표 존재 여부
  cv::Point2f saved_avoidance_goal;                       // 저장된 회피 목표점

  // A* 경로 추종용 변수 (Pure Pursuit)
  std::vector<cv::Point2f> path_m_; // 추적할 전체 경로 (월드 좌표)
  size_t wp_idx_ = 0;               // 현재 목표 웨이포인트 인덱스
  const double lookahead_ = 0.3;    // Pure Pursuit Lookahead 거리 [m]
  const double arrive_thresh_ = 0.1; // 목표점에 도착했다고 판정할 거리 [m]

  // 시각화를 위한 현재 목표 웨이포인트 정보 제공
  cv::Point2f getCurrentTargetWaypoint() const {
    if (wp_idx_ < path_m_.size()) {
      return path_m_[wp_idx_];
    }
    return cv::Point2f(-1.0f, -1.0f); // 유효하지 않은 값
  }

 public Q_SLOTS:
  void go(const std::vector<int>& waypoints);
  void tracking(const std::vector<int>& waypoints);

  // 주석처리된 함수들 - 현재 사용하지 않음
  // void a_tracking(const cv::Point2f& pixel_target); // Pure Pursuit 사용으로 대체

  double angular_velocity(double R, double v);
  double R_track(double L, int x);
  double Look_aheadDistance(double v);
  void drive(double linear_x, double angular_z);
  void avoidanceMode();

  // 주석처리된 함수 - 현재 사용하지 않음
  // bool followPath();

  // 🆕 새로운 함수들
  bool planCompleteAvoidancePath();          // 완전한 회피 경로 계획
  bool shouldEmergencyReplan();
  void setPath(const std::vector<cv::Point2f>& path_m); // 경로 설정
  void startPathTracking();                             // 경로 추종 시작
  bool executePathStep();                    // 경로 한 스텝 실행 (Pure Pursuit)
  void stopRobot();                         // 로봇 정지

 private:
  QNode* qnode;
  MainWindow* main_window;
  Astar* a_planner = nullptr;

  // 주석처리된 변수 - a_waypoints 및 path_m_로 대체됨
  // std::vector<cv::Point2f> a_path;
};

#endif

#ifndef DRIVING_HPP
#define DRIVING_HPP

#include <QObject>
#include <chrono>
#include "qnode.hpp"

class MainWindow;
class Astar;

class Driving : public QObject {
  Q_OBJECT

 public:
  Driving(QObject* parent = nullptr);
  ~Driving() = default;

  enum Current_state{
    LANE_TRACKING,
    AVOIDANCE,
    RETURN_LANE
  };

  void setQNode(QNode* qnode_ptr);
  void setMainWindow(MainWindow* main_window_ptr);
  void setPlanner(Astar* planner);

  static constexpr double pixel_to_meter = -0.0018;
  static constexpr double w_lim = 1.8;
  static constexpr double b = 0.16;
  double current_speed;

  std::vector<int>a_waypoints;
  cv::Point2f start, goal;
  int current_path_index = 0;
  Current_state state = LANE_TRACKING;

  // 🌍 글로벌 경로 저장용 변수들
  std::chrono::steady_clock::time_point path_start_time;  // 경로 시작 시간
  std::vector<cv::Point2f> global_path;
  bool global_path_ready = false;
  bool has_avoidance_goal = false;                        // 회피 목표 존재 여부
  cv::Point2f saved_avoidance_goal;                       // 저장된 회피 목표점

  // 시각화를 위한 현재 목표 웨이포인트 정보 제공
  int getCurrentTargetWaypoint() const {
    if (current_path_index < a_waypoints.size()) {
      return a_waypoints[current_path_index];
    }
    return -1; // 유효하지 않은 값
  }

 public Q_SLOTS:
  void go(const std::vector<int>& waypoints);
  void tracking(const std::vector<int>& waypoints);
  void a_tracking(const std::vector<int>& waypoints);
  double angular_velocity(double R, double v);
  double R_track(double L, int x);
  double Look_aheadDistance(double v);
  void drive(double linear_x, double angular_z);
  void avoidanceMode();
  bool followPath();

  // 🆕 새로운 함수들
  bool planCompleteAvoidancePath();          // 완전한 회피 경로 계획
  bool shouldEmergencyReplan();
  bool executePathStep();                    // 경로 한 스텝 실행
  void stopRobot();                         // 로봇 정지

 private:
  QNode* qnode;
  MainWindow* main_window;
  Astar* a_planner = nullptr;
  std::vector<cv::Point2f> a_path;

};

#endif

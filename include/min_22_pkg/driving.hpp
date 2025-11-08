#ifndef DRIVING_HPP
#define DRIVING_HPP

#include <QObject>
#include "qnode.hpp"
#include <vector>
#include <opencv2/core.hpp>

class MainWindow;
class Astar;

class Driving : public QObject {
  Q_OBJECT

 public:
  Driving(QObject* parent = nullptr);
  ~Driving() = default;

  enum Current_state{
    LANE_TRACKING,
    PLANNING,
    RETURN_LANE,
    PATH_TRACK
  };

  void setQNode(QNode* qnode_ptr);
  void setMainWindow(MainWindow* main_window_ptr);
  void setPlanner(Astar* planner);

  static constexpr double pixel_to_meter = -0.0018;
  static constexpr double w_lim = 1.8;
  static constexpr double b = 0.16;
  double current_speed;

  std::vector<cv::Point2f>a_waypoints;
  cv::Point2f start, goal;
  int current_path_index = 0;
  Current_state state = LANE_TRACKING;

  // 🌍 글로벌 경로 저장용 변수들
  bool global_path_ready = false;
  bool has_avoidance_goal = false;                        // 회피 목표 존재 여부
  cv::Point2f saved_avoidance_goal;                       // 저장된 회피 목표점

  // A* 경로 추종용 변수 (Pure Pursuit)
  std::vector<cv::Point2f> path_m_; // 추적할 전체 경로 (월드 좌표)
  size_t wp_idx_ = 0;               // 현재 목표 웨이포인트 인덱스
  const double lookahead_ = 0.3;    // Pure Pursuit Lookahead 거리 [m]
  const double arrive_thresh_ = 0.01; // 목표점에 도착했다고 판정할 거리 [m]

 public Q_SLOTS:
  void go(const std::vector<int>& waypoints);
  void tracking(const std::vector<int>& waypoints);

  double angular_velocity(double R, double v);
  double R_track(double L, int x);
  double Look_aheadDistance(double v);
  void drive(double linear_x, double angular_z);

  bool planCompleteAvoidancePath();
  void setPath(const std::vector<cv::Point2f>& path_m);
  void startPathTracking();
  bool executePathStep();
  void stopRobot();

 private:
  QNode* qnode;
  MainWindow* main_window;
  Astar* a_planner = nullptr;

};

#endif

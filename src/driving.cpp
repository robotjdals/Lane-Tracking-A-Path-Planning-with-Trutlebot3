#include "../include/min_22_pkg/driving.hpp"
#include "../include/min_22_pkg/main_window.hpp"
#include "../include/min_22_pkg/astar.hpp"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

Driving::Driving(QObject* parent) : QObject(parent) {
  qnode = nullptr;
  current_speed = 0.08;  //기본 속도 8cm/s
  has_avoidance_goal = false;
  state = LANE_TRACKING;
}

void Driving::setQNode(QNode* qnode_ptr) {
  qnode = qnode_ptr;
}

void Driving::setMainWindow(MainWindow* main_window_ptr) {
  main_window = main_window_ptr;
}

void Driving::setPlanner(Astar* planner) {
  a_planner = planner;
}

void Driving::go(const std::vector<int>& waypoints){
  if (qnode && !qnode->odom_received) {
    drive(0.0, 0.0);
    return;
  }

  if(qnode && qnode->speed_received){
    current_speed = qnode->current_linear_x;
  }

  switch(state){
    case LANE_TRACKING:
    {
      /*
      if (!main_window->system_running) {
        drive(0.0, 0.0);  // 정지
        return;
      }*/

      // === 차선 추종 모드 ===
      if(qnode && qnode->detectObstacle(0.4, 0.3)){

        if (main_window) {
          main_window->showMapUI();
        }
        stopRobot();
        state = PLANNING;
      }
      else{
        tracking(waypoints);
      }
    }
    break;

    case PLANNING:
    {
      if (planCompleteAvoidancePath()) {
        state = PATH_TRACK;
        wp_idx_ = 0;
      } else {
        static int retry_count = 0;
        retry_count++;
        if (retry_count > 10) {
          state = LANE_TRACKING;
          if (main_window) main_window->hideMapUI();
          retry_count = 0;
        }
      }
    }
    break;

    case PATH_TRACK:
    {
      bool finished = executePathStep();
      if (finished) {
        // UI 끄기
        std::cout << "목표점 찾음" << std::endl;
        if (main_window) main_window->hideMapUI();
        state = RETURN_LANE;
        path_m_.clear();
        global_path_ready = false;
        has_avoidance_goal = false;
        wp_idx_ = 0;
        //qnode->drive(0.0, 0.0);

      }
    }
    break;

    case RETURN_LANE:
    {
      static int return_counter = 0;
      if (return_counter < 10) { // 10프레임 정도 정지
        qnode->drive(0.0, 0.0);
        return_counter++;
      } else {
        if (!waypoints.empty()) {
          state = LANE_TRACKING;
          return_counter = 0;
          tracking(waypoints);
        }
      }
    }
    break;
  }
}


void Driving::tracking(const std::vector<int>& waypoints){
  if(waypoints.empty()) {
    drive(0.0, 0.0);
    return;
  }

  std::vector<cv::Point2f> target_idx;
  for (int i = 0; i < waypoints.size(); ++i) {
  double x_px = waypoints[i];
  double y_px = 360 - (i*36 + 18);

  double x_m = (320 - x_px) * pixel_to_meter;
  double y_m = (360 - y_px) * pixel_to_meter;
  target_idx.push_back(cv::Point2f(x_m, y_m));
}

  cv::Vec3d coeffients = curve_fitting(target_idx);
  double a = coeffients[0]; //곡률계수(얼마나 휘었는지)
  double b = coeffients[1]; //기울기(방향)
  double c = coeffients[2]; //y절편(로봇의 횡방향 크기)

  double curvature = std::abs(a);
  target_speed = changedspeed(curvature, current_speed);


  double L = Look_aheadDistance(target_speed, curvature);
  std::cout<< "L "<<L<<std::endl;
  double X_center = a*L*L + b*L + c;
  double R = R_track(L, X_center);
  double w = angular_velocity(R, target_speed);
  std::cout<< "speed "<<target_speed<<std::endl;
  drive(target_speed, w);
}

double Driving::changedspeed(double curvature, double current_speed){
  const double min_speed = 0.08;
  const double max_speed = 0.18;
  const double straight = 0.01;
  const double curve = 0.03;

  double target;

  if(curvature < straight){
    target = max_speed;
  }else if(curvature > curve){
    target = min_speed;
  }else {
    double speed_factor = 1.0 - (curvature - straight) / (curve);
    target = min_speed + (max_speed - min_speed) * speed_factor;
  }

  const double max_accel = 0.008;
  const double max_decel = 0.025;

  if (target > current_speed) {
        // 가속
        return std::min(target, current_speed + max_accel);
    } else {
        // 감속
        return std::max(target, current_speed - max_decel);
    }

}

cv::Vec3d Driving::curve_fitting(const std::vector<cv::Point2f>& target){

  cv::Mat A(target.size(), 3, CV_64F);
  cv::Mat X(target.size(), 1, CV_64F);

  for (int i = 0; i < target.size(); ++i) {
    double y = target[i].y;
    double x = target[i].x;
    A.at<double>(i, 0) = y*y;
    A.at<double>(i, 1) = y;
    A.at<double>(i, 2) = 1.0;
    X.at<double>(i, 0) = x;
  }

  cv::Mat p;
  cv::solve(A ,X, p, cv::DECOMP_QR);

  double a = p.at<double>(0);
  double b = p.at<double>(1);
  double c = p.at<double>(2);


  return cv::Vec3d(a,b,c);
}


double Driving::angular_velocity(double R, double v){
  if(abs(R) > 100.0) return 0.0;

  double w = v / R;
  return std::max(-w_lim, std::min(w_lim, w));
}

double Driving::R_track(double L, double x){

  if(abs(x) < 0.01) {
    return 1000.0;
  }
  double R = (L * L) / (2 * x);
  if(abs(R) > 100.0) {
    return (R > 0) ? 100.0 : -100.0;
  }

  return R;
}

double Driving::Look_aheadDistance(double v, double curvature){

  double MIN_L, MAX_L;

    std::cout<< "curvature "<<curvature<<std::endl;
    if (curvature < 0.008) {
        // 직선
        MIN_L = 0.45;
        MAX_L = 0.60;
    } else if (curvature < 0.03) {
        // 완만한 커브
        MIN_L = 0.12;
        MAX_L = 0.25;
    } else {
        // 급커브
        MIN_L = 0.10;
        MAX_L = 0.20;
    }

  double L = 2 * v / w_lim;
  return std::max(MIN_L, std::min(MAX_L, L));
}

void Driving::drive(double linear_x, double angular_z){
  if(qnode)
    qnode->drive(linear_x, angular_z);
}

// 회피 경로 계획 함수
bool Driving::planCompleteAvoidancePath() {
  if (!a_planner || !qnode || !qnode->odom_received) {
      return false;
  }
  // 시작 위치
  cv::Point2f start_pos(qnode->odom_x, qnode->odom_y);
  double yaw = qnode->odom_yaw;  // 로봇의 회전 각도 (라디안)
  double offset = 0.08; // TurtleBot Burger 반지름

  start_pos.x -= offset * std::cos(yaw);
  start_pos.y -= offset * std::sin(yaw);


  // 도착점
  //cv::Point2f final_goal(1.764391, 1.396047); // 장애물 옆
  cv::Point2f final_goal(1.506287, 1.752307); //장애물 뒤
  //cv::Point2f final_goal(1.786765, 1.678603);

  cv::Point2i robot_grid = a_planner->worldToGrid(start_pos.x, start_pos.y);
  cv::Point2i goal_grid = a_planner->worldToGrid(final_goal.x, final_goal.y);

  // 경계 체크
  if (!a_planner->inBounds(robot_grid.x, robot_grid.y)) {
      return false;
  }
  if (!a_planner->inBounds(goal_grid.x, goal_grid.y)) {
      return false;
  }

  // 거리 계산
  double distance = std::sqrt(std::pow(final_goal.x - start_pos.x, 2) + std::pow(final_goal.y - start_pos.y, 2));
  // A*로 글로벌 경로 계획
  std::vector<cv::Point2f> path = a_planner->planGlobalPath(start_pos, final_goal, 5.0); // 5.0은 사용되지 않지만 인자 유지

  if (!path.empty()) {
    // 경로 저장 및 추적 준비
    setPath(path); // path_m_ = path; wp_idx_ = 0;
    global_path_ready = true;
    has_avoidance_goal = true;
    saved_avoidance_goal = final_goal;

    return true;
  } else {
  }

  return false;
}

void Driving::setPath(const std::vector<cv::Point2f>& path_m){
  path_m_ = path_m; wp_idx_ = 0;
}

void Driving::startPathTracking(){
  if(path_m_.empty()) return;
  state = PATH_TRACK;
}

bool Driving::executePathStep(){
  if(state != PATH_TRACK || !qnode) return false;

  // 현재 로봇 포즈 (절대 위치)
  double rx = qnode->odom_x;  // [m]
  double ry = qnode->odom_y;  // [m]
  double rth = qnode->odom_yaw; // [rad]

  // 다음 목표 웨이포인트 선택 (Lookahead)
  while(wp_idx_ + 1 < path_m_.size()){
    double dx = path_m_[wp_idx_].x - rx;
    double dy = path_m_[wp_idx_].y - ry;
    if (std::hypot(dx,dy) > lookahead_ * 1.2) break; // Lookahead 거리보다 1.2배 멀 때
    wp_idx_++;
  }

  // 최종 목표점에 도달했는지 확인
    double final_dx = path_m_.back().x - rx;
    double final_dy = path_m_.back().y - ry;

    if (std::hypot(final_dx, final_dy) < arrive_thresh_ * 2) { // 2배 임계값 사용
      std::cout << final_dx <<","<< final_dy << std::endl;
        qnode->drive(0.0, 0.0);
        state = RETURN_LANE;
        return true;
    }


  // 목표점 (tx, ty)
  double tx = path_m_[wp_idx_].x;
  double ty = path_m_[wp_idx_].y;
  double dx = tx - rx, dy = ty - ry;

  // lx = 전방 거리 (종방향), ly = 횡방향 거리
  // atan2(dy, dx) - rth -> 목표점까지의 상대 각도
  double angle_to_target = std::atan2(dy, dx);
  double alpha = angle_to_target - rth; // 목표점과 로봇 헤딩의 각도 차이

  // 각도 차이를 [-pi, pi] 범위로 정규화
  while (alpha > M_PI) alpha -= 2 * M_PI;
  while (alpha < -M_PI) alpha += 2 * M_PI;

  double dist = std::hypot(dx, dy); // 목표점까지의 거리 (Lookahead 거리)
  double k = 2.0 * std::sin(alpha) / dist;
  const double base_speed = 0.10;
  const double min_speed  = 0.04;
  double predicted_w = base_speed * k;
  double normalized_turn = std::min(std::abs(predicted_w) / w_lim, 1.0);
  double curve_factor = 1.0 - 0.5 * normalized_turn * normalized_turn;
  double v = std::max(min_speed, base_speed * curve_factor);
  double w = v * k;
  w = std::clamp(w, -w_lim, w_lim);

  qnode->drive(v,w);
  return false;
}

void Driving::stopRobot() {
    drive(0.0, 0.0);
}

#include "../include/min_22_pkg/driving.hpp"
#include "../include/min_22_pkg/main_window.hpp"
#include "../include/min_22_pkg/astar.hpp"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

Driving::Driving(QObject* parent) : QObject(parent) {
  qnode = nullptr;
  current_speed = 0.08;  // 🔧 기본 속도 8cm/s로 설정
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
  // 1. Odometry 수신 대기 (글로벌 플래닝 필수)
  if (qnode && !qnode->odom_received) {
      std::cout << "Odometry data not yet received, waiting..." << std::endl;
      drive(0.0, 0.0);
      return;
  }

  if(qnode && qnode->speed_received){
    current_speed = qnode->current_linear_x;
  }

  switch(state){
    case LANE_TRACKING:
    {
      // === 1. 차선 추종 모드 ===
      if(qnode && qnode->detectObstacle(0.5, 0.3)){  // 감지 거리를 1.5→3.0m, 각도를 1.0→1.2로 늘림
        std::cout << "\n🚨 장애물 감지: UI 표시 후 경로 계획 시작" << std::endl;

        // 🎯 1단계: 즉시 UI 표시
        if (main_window) {
            main_window->showMapUI();
            std::cout << "🗺️ Map UI 먼저 활성화 완료" << std::endl;
        }

        // 🎯 2단계: 로봇 정지
        stopRobot();
        std::cout << "🛑 로봇 완전 정지" << std::endl;

        // 🎯 3단계: 경로 계획 상태로 전환
        state = PLANNING;
        std::cout << "🔄 상태 변경: LANE_TRACKING → PLANNING" << std::endl;
      }
      else{
        // 장애물 없음 → 차선 추종 계속
        tracking(waypoints);
      }
    }
    break;

    case PLANNING:
    {
      // === 2. 경로 계획 모드 ===
      std::cout << "🧠 A* 경로 계획 실행 중..." << std::endl;

      if (planCompleteAvoidancePath()) {
          std::cout << "✅ 경로 계획 성공! PATH_TRACK 모드로 전환" << std::endl;
          state = PATH_TRACK;
          wp_idx_ = 0;
          path_start_time = std::chrono::steady_clock::now();
      } else {
          std::cout << "❌ 경로 계획 실패, 재시도 중..." << std::endl;
          // 계속 PLANNING 상태에서 재시도
          static int retry_count = 0;
          retry_count++;
          if (retry_count > 10) { // 10번 실패하면 포기
              std::cout << "⚠️ 경로 계획 포기, 차선 추종으로 복귀" << std::endl;
              state = LANE_TRACKING;
              if (main_window) main_window->hideMapUI();
              retry_count = 0;
          }
      }
    }
    break;

    case PATH_TRACK:
    case AVOIDANCE: // PATH_TRACK으로 통일
    {
        // === 2. 경로 추종 모드 ===
        std::cout << "🛤️ 경로 추종 실행 중..." << std::endl;

        bool finished = executePathStep();

        if (finished) {
            std::cout << "\n🎯 목표점 도달 완료!" << std::endl;
            std::cout << "🗺️ Map UI 비활성화" << std::endl;
            std::cout << "🔄 차선 복귀 모드로 전환" << std::endl;

            // UI 끄기
            if (main_window) main_window->hideMapUI();

            // 차선 복귀 상태로 전환
            state = RETURN_LANE;

            // 경로 데이터 정리
            path_m_.clear();
            global_path_ready = false;
            has_avoidance_goal = false;
            wp_idx_ = 0;

            std::cout << "✅ 경로 추종 완료, 차선 복귀 대기 중" << std::endl;
        }

        // 긴급 재계획 조건 체크
        if (!finished && shouldEmergencyReplan()) {
            std::cout << "⚠️ 긴급 재계획 필요!" << std::endl;
            if (planCompleteAvoidancePath()) {
                wp_idx_ = 0;
                path_start_time = std::chrono::steady_clock::now();
                std::cout << "✅ 긴급 재계획 성공" << std::endl;
            } else {
                std::cout << "❌ 긴급 재계획 실패, 차선 복귀 모드로 전환" << std::endl;
                stopRobot();
                state = RETURN_LANE;
                if (main_window) main_window->hideMapUI();
            }
        }
    }
    break;

    case RETURN_LANE:
    {
      // === 3. 차선 복귀 모드 ===
      bool lanes_detected = (main_window->left_detected || main_window->right_detected);
      bool is_center = abs(waypoints.empty() ? 320 : waypoints[std::min(4, (int)waypoints.size() - 1)] - 320) < 50;

      if(lanes_detected && is_center){
        std::cout << "\n✅ 차선 복구 완료! 정상 차선 추종으로 복귀" << std::endl;
        state = LANE_TRACKING;
        tracking(waypoints);
      }
      else{
        std::cout << "🔍 차선 복구 대기 중... (waypoints: " << waypoints.size() << ", lanes: " << lanes_detected << ")" << std::endl;
        // 차선 중앙으로 천천히 복귀
        int target_waypoint = waypoints.empty() ? 320 : waypoints[std::min(4, (int)waypoints.size() - 1)];
        double deviation = (target_waypoint - 320) * pixel_to_meter;
        double target_w = -0.5 * deviation; // P 제어
        drive(0.08, std::clamp(target_w, -w_lim/2.0, w_lim/2.0));  // 천천히 주행
      }
    }
    break;
  }
}

void Driving::tracking(const std::vector<int>& waypoints){
  // 기존 차선 트래킹 로직 (변경 없음)
  int target_idx = std::min(4, (int)waypoints.size() - 1);
  int target_waypoint = waypoints[target_idx];

  if(waypoints.empty()) {
    drive(0.0, 0.0); // 차선이 없으면 정지
    return;
  }

  int deviation = abs(target_waypoint - 320);

  double base_speed = (current_speed > 0.001) ? current_speed : 0.08;

  if(deviation < 10) {
    double target_speed = std::min(0.15, base_speed + 0.002);
    drive(target_speed, 0.0);
  }
  else {
    double normalized_deviation = std::min(deviation / 320.0, 1.0);
    double curve_factor = 1.0 - 0.3 * normalized_deviation * normalized_deviation;
    double target_speed = std::max(0.05, base_speed * curve_factor);
    double L = Look_aheadDistance(target_speed);
    double R = R_track(L, target_waypoint);
    double w = angular_velocity(R, target_speed);
    drive(target_speed, w);
  }
}

// 주석처리된 함수 - 현재 사용하지 않음
// void Driving::a_tracking(const cv::Point2f& pixel_target) { /* Pure Pursuit 사용으로 대체 */ }

double Driving::angular_velocity(double R, double v){
  if(abs(R) > 100.0) return 0.0;

  double w = v / R;
  return std::max(-w_lim, std::min(w_lim, w));
}

double Driving::R_track(double L, int x){
  double y = (x - 320) * pixel_to_meter;

  if(abs(y) < 0.001) {
    return 1000.0;
  }
  double R = (L * L) / (2 * y);
  if(abs(R) > 100.0) {
    return (R > 0) ? 100.0 : -100.0;
  }

  return R;
}

double Driving::Look_aheadDistance(double v){
  const double MIN_L = 0.16;  // 0.16m
  const double MAX_L = 0.48;  // 0.48m

  double L = 2 * v / w_lim;
  return std::max(MIN_L, std::min(MAX_L, L));
}

void Driving::drive(double linear_x, double angular_z){
  if(qnode)
    qnode->drive(linear_x, angular_z);
}

void Driving::avoidanceMode() {
  std::cout << "⚠️ avoidanceMode() called. Use planCompleteAvoidancePath() instead." << std::endl;
}


// 완전한 회피 경로 계획 함수
bool Driving::planCompleteAvoidancePath() {
    if (!a_planner || !qnode || !qnode->odom_received) {
        std::cout << "❌ A* 플래너 또는 Odometry 정보가 없습니다!" << std::endl;
        return false;
    }

    std::cout << "\n========== COMPLETE AVOIDANCE PATH PLANNING ==========" << std::endl;

    // 🌍 글로벌 시작 위치 (Odometry)
    cv::Point2f start_pos(qnode->odom_x, qnode->odom_y);

    // 🌍 가제보상 정확한 좌표 사용
    cv::Point2f final_goal(1.764391, 1.396047);  // 빨간색 도착점 (가제보상 확인)

    std::cout << "\n📍 COORDINATE DEBUG INFO:" << std::endl;
    std::cout << "🤖 Robot Current Position:" << std::endl;
    std::cout << "   World: (" << std::fixed << std::setprecision(6) << start_pos.x << ", " << start_pos.y << ")" << std::endl;

    // 로봇 현재 위치를 그리드로 변환 (실제 맵 좌표계 사용)
    cv::Point2i robot_grid = a_planner->worldToGrid(start_pos.x, start_pos.y);
    std::cout << "   Grid:  (" << robot_grid.x << ", " << robot_grid.y << ")" << std::endl;

    std::cout << "\n🎯 Target Goal Position:" << std::endl;
    std::cout << "   World: (" << std::fixed << std::setprecision(6) << final_goal.x << ", " << final_goal.y << ")" << std::endl;

    // 목표점을 그리드로 변환 (실제 맵 좌표계 사용)
    cv::Point2i goal_grid = a_planner->worldToGrid(final_goal.x, final_goal.y);
    std::cout << "   Grid:  (" << goal_grid.x << ", " << goal_grid.y << ")" << std::endl;

    std::cout << "\n🗺️  Map Information:" << std::endl;
    std::cout << "   Size: " << a_planner->W_ << "x" << a_planner->H_ << " pixels" << std::endl;
    std::cout << "   Origin: (" << a_planner->origin_m_.x << ", " << a_planner->origin_m_.y << ") meters" << std::endl;
    std::cout << "   Resolution: " << a_planner->res_ << " m/px" << std::endl;
    std::cout << "   Coverage: X[" << a_planner->origin_m_.x << " to " << (a_planner->origin_m_.x + a_planner->W_ * a_planner->res_)
              << "], Y[" << a_planner->origin_m_.y << " to " << (a_planner->origin_m_.y + a_planner->H_ * a_planner->res_) << "] meters" << std::endl;

    std::cout << "\n📋 GAZEBO VERIFICATION (All Coordinates):" << std::endl;
    std::cout << "   Gazebo Start (Blue):   World(1.752260, 0.454965) → Grid("
              << a_planner->worldToGrid(1.752260, 0.454965).x << ", "
              << a_planner->worldToGrid(1.752260, 0.454965).y << ")" << std::endl;
    std::cout << "   Gazebo Goal (Red):     World(1.752248, 1.519646) → Grid("
              << a_planner->worldToGrid(1.752248, 1.519646).x << ", "
              << a_planner->worldToGrid(1.752248, 1.519646).y << ")" << std::endl;
    std::cout << "   Obstacle 1:            World(1.490000, 0.540000) → Grid("
              << a_planner->worldToGrid(1.490000, 0.540000).x << ", "
              << a_planner->worldToGrid(1.490000, 0.540000).y << ")" << std::endl;
    std::cout << "   Obstacle 2:            World(1.740000, 1.000000) → Grid("
              << a_planner->worldToGrid(1.740000, 1.000000).x << ", "
              << a_planner->worldToGrid(1.740000, 1.000000).y << ")" << std::endl;
    std::cout << "   Obstacle 3:            World(1.490000, 1.464000) → Grid("
              << a_planner->worldToGrid(1.490000, 1.464000).x << ", "
              << a_planner->worldToGrid(1.490000, 1.464000).y << ")" << std::endl;

    // 경계 체크
    if (!a_planner->inBounds(robot_grid.x, robot_grid.y)) {
        std::cout << "❌ Robot is outside map bounds!" << std::endl;
        return false;
    }

    if (!a_planner->inBounds(goal_grid.x, goal_grid.y)) {
        std::cout << "❌ Goal is outside map bounds!" << std::endl;
        return false;
    }

    // 거리 계산
    double distance = std::sqrt(std::pow(final_goal.x - start_pos.x, 2) + std::pow(final_goal.y - start_pos.y, 2));
    std::cout << "\n📏 Planning Distance: " << std::fixed << std::setprecision(3) << distance << " meters" << std::endl;

    std::cout << "\n🚀 Starting A* Path Planning..." << std::endl;

    // A*로 글로벌 경로 계획
    std::vector<cv::Point2f> path = a_planner->planGlobalPath(start_pos, final_goal, 5.0); // 5.0은 사용되지 않지만 인자 유지

    if (!path.empty()) {
        std::cout << "✅ A* Path Planning SUCCESS!" << std::endl;
        std::cout << "   Generated Waypoints: " << path.size() << std::endl;

        // 처음 몇 개와 마지막 몇 개 웨이포인트 출력
        std::cout << "   First 3 waypoints:" << std::endl;
        for (size_t i = 0; i < std::min((size_t)3, path.size()); i++) {
            cv::Point2i grid_pos = a_planner->worldToGrid(path[i].x, path[i].y);
            std::cout << "     [" << i << "] World(" << std::fixed << std::setprecision(6)
                      << path[i].x << ", " << path[i].y << ") Grid(" << grid_pos.x << ", " << grid_pos.y << ")" << std::endl;
        }

        if (path.size() > 3) {
            std::cout << "   Last 3 waypoints:" << std::endl;
            for (size_t i = std::max((size_t)0, path.size()-3); i < path.size(); i++) {
                cv::Point2i grid_pos = a_planner->worldToGrid(path[i].x, path[i].y);
                std::cout << "     [" << i << "] World(" << std::fixed << std::setprecision(6)
                          << path[i].x << ", " << path[i].y << ") Grid(" << grid_pos.x << ", " << grid_pos.y << ")" << std::endl;
            }
        }

        // 경로 저장 및 추적 준비
        setPath(path); // path_m_ = path; wp_idx_ = 0;
        global_path_ready = true;
        has_avoidance_goal = true;
        saved_avoidance_goal = final_goal;

        std::cout << "🎯 Path tracking setup complete, starting PATH_TRACK mode" << std::endl;
        std::cout << "================================\n" << std::endl;

        return true;
    } else {
        std::cout << "❌ A* Path Planning FAILED!" << std::endl;
        std::cout << "   Possible causes:" << std::endl;
        std::cout << "   - Start or goal in obstacle" << std::endl;
        std::cout << "   - No path exists due to obstacles" << std::endl;
        std::cout << "   - Coordinate transformation issues" << std::endl;
    }

    return false;
}

bool Driving::shouldEmergencyReplan() {
    // 1. 너무 오래 걸리는 경우 (30초 이상)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - path_start_time);
    if (elapsed.count() > 30) {
        std::cout << "경로 실행 시간 초과 (30초)" << std::endl;
        return true;
    }

    // 2. 로봇이 심각하게 막힘 경우 (같은 웨이포인트에 5초 이상 머무름)
    static int stuck_counter = 0;
    static size_t last_path_index = (size_t)-1;

    if (wp_idx_ == last_path_index) {
        stuck_counter++;
        if (stuck_counter > 150) {  // 5초 (30fps 기준)
            std::cout << "로봇이 같은 지점에 5초 이상 정체" << std::endl;
            stuck_counter = 0;
            return true;
        }
    } else {
        stuck_counter = 0;
        last_path_index = wp_idx_;
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

  // 🌍 현재 로봇 포즈 (절대 위치)
  double rx = qnode->odom_x;  // [m]
  double ry = qnode->odom_y;  // [m]
  double rth = qnode->odom_yaw; // [rad]

  // 다음 목표 웨이포인트 선택 (Lookahead)
  // 현재 위치에서 Lookahead 거리보다 멀리 떨어진 웨이포인트를 찾습니다.
  while(wp_idx_ + 1 < path_m_.size()){
    double dx = path_m_[wp_idx_].x - rx;
    double dy = path_m_[wp_idx_].y - ry;
    if (std::hypot(dx,dy) > lookahead_ * 1.5) break; // Lookahead 거리보다 1.5배 멀 때
    wp_idx_++;
  }

  // 최종 목표점에 도달했는지 확인
  if (wp_idx_ >= path_m_.size()){
    // 마지막 웨이포인트까지의 거리가 도착 임계값 이내인지 최종 확인
    double final_dx = path_m_.back().x - rx;
    double final_dy = path_m_.back().y - ry;
    if (std::hypot(final_dx, final_dy) < arrive_thresh_ * 2) { // 2배 임계값 사용
        qnode->drive(0.0, 0.0);
        return true; // 경로 추종 완료
    }
    // 마지막 웨이포인트가 있지만, 아직 충분히 가깝지 않다면 마지막 웨이포인트를 목표로 설정
    wp_idx_ = path_m_.size() - 1;
  }

  // 목표점 (tx, ty)
  double tx = path_m_[wp_idx_].x;
  double ty = path_m_[wp_idx_].y;
  double dx = tx - rx, dy = ty - ry;

  // 1. 목표점을 로봇 좌표계로 변환 (Pure Pursuit에 필요한 횡방향 거리)
  // lx = 전방 거리 (종방향), ly = 횡방향 거리
  // atan2(dy, dx) - rth -> 목표점까지의 상대 각도
  double angle_to_target = std::atan2(dy, dx);
  double alpha = angle_to_target - rth; // 목표점과 로봇 헤딩의 각도 차이

  // 각도 차이를 [-pi, pi] 범위로 정규화
  while (alpha > M_PI) alpha -= 2 * M_PI;
  while (alpha < -M_PI) alpha += 2 * M_PI;

  double dist = std::hypot(dx, dy); // 목표점까지의 거리 (Lookahead 거리)

  // 2. Pure Pursuit 곡률 (Curvature) 계산
  // 곡률 k = 2 * sin(alpha) / dist
  double k = 2.0 * std::sin(alpha) / dist;

  // 1. 기본/최소 속도 정의
  const double base_speed = 0.10; // m/s (최대 속도)
  const double min_speed  = 0.05; // m/s (최소 속도)

  // 2. '최대 속도'로 이 '곡률'을 돌 때의 예상 각속도를 계산
  double predicted_w = base_speed * k;

  // 3. 'tracking' 함수와 동일하게 커브 팩터 계산
  //    (예상 각속도 / 최대 각속도)로 정규화
  double normalized_turn = std::min(std::abs(predicted_w) / w_lim, 1.0);

  // 4. 회전이 심할수록 속도를 줄임 (0.5는 감속 강도, 조절 가능)
  double curve_factor = 1.0 - 0.5 * normalized_turn * normalized_turn;

  // 5. 최종 속도 v 결정
  double v = std::max(min_speed, base_speed * curve_factor);

  // 6. '최종 속도 v'를 기준으로 '실제 각속도 w'를 다시 계산
  double w = v * k;

  // 7. 각속도만 w_lim으로 제한 (v는 이미 min/max 처리됨)
  w = std::clamp(w, -w_lim, w_lim);
/*
  // 속도/각속도 제한
  v = std::clamp(v, 0.05, 0.25);
  w = std::clamp(w, -w_lim, w_lim);
*/
  // 현재 추적 웨이포인트 출력 (디버그)
  std::cout << "Tracking: WP " << wp_idx_ << "/" << path_m_.size() - 1
            << " | Dist: " << std::fixed << std::setprecision(2) << dist << "m"
            << " | v: " << v << ", w: " << w << std::endl;

  qnode->drive(v,w);
  return false; // 경로 추종 미완료
}

void Driving::stopRobot() {
    drive(0.0, 0.0);
    std::cout << "로봇 완전 정지" << std::endl;
}

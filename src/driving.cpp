#include "../include/min_22_pkg/driving.hpp"
#include "../include/min_22_pkg/main_window.hpp"
#include "../include/min_22_pkg/astar.hpp"

#include <cmath>

Driving::Driving(QObject* parent) : QObject(parent) {
  qnode = nullptr;
  current_speed = 0.08;  // 🔧 기본 속도 8cm/s로 설정
  has_avoidance_goal = false;
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
  if(qnode && qnode->speed_received){
    current_speed = qnode->current_linear_x;
  }

  switch(state){
    case LANE_TRACKING:
    {
      // 🔧 매 프레임마다 장애물 체크
      if(qnode && qnode->detectObstacle(0.2, 1.0)){
        std::cout << "🚨 장애물 감지! 즉시 회피 경로 계획 시작..." << std::endl;

        // 🚀 즉시 회피 경로 계획 시도
        if (planCompleteAvoidancePath()) {
            state = AVOIDANCE;
            current_path_index = 0;
            path_start_time = std::chrono::steady_clock::now();
            std::cout << "✅ 회피 경로 계획 완료, 즉시 실행 시작" << std::endl;
        } else {
            std::cout << "❌ 회피 경로 계획 실패, 응급 정지" << std::endl;
            stopRobot();
            // 다음 프레임에서 다시 시도
        }
      }
      else{
        tracking(waypoints);
      }
    }
    break;

    // 🗑️ EMERGENCY_STOP 상태 제거됨

case AVOIDANCE:
    {
        // 🎯 경로 끝까지 따라가기 (중단 없음)
        if (current_path_index < a_waypoints.size()) {

            // 현재 진행률 표시
            double progress = (double)(current_path_index + 1) / a_waypoints.size() * 100.0;

            std::cout << "🎯 경로 실행: " << (current_path_index + 1) << "/" << a_waypoints.size()
                      << " (" << std::fixed << std::setprecision(1) << progress << "%)" << std::endl;

            // 🚀 경로 실행
            bool reached = executePathStep();

            if (reached) {
                current_path_index++;
                std::cout << "✅ 웨이포인트 " << current_path_index << "/" << a_waypoints.size() << " 통과" << std::endl;

                // 🎊 경로 완주 체크
                if (current_path_index >= a_waypoints.size()) {
                    std::cout << "\n🎊 회피 경로 완주 성공!" << std::endl;
                    std::cout << "🔄 정상 주행 모드로 복귀" << std::endl;

                    // 정상 상태로 복귀
                    state = LANE_TRACKING;  // 또는 원하는 다음 상태
                    a_waypoints.clear();
                    global_path_ready = false;
                    has_avoidance_goal = false;
                    current_path_index = 0;
                }
            }

        } else {
            // 예외 상황: 웨이포인트가 없음
            std::cout << "❌ 웨이포인트 없음, 정상 모드로 복귀" << std::endl;
            state = LANE_TRACKING;
        }

        // 🚨 매우 제한적인 재계획 조건 (선택사항)
        if (shouldEmergencyReplan()) {
            std::cout << "🚨 긴급 재계획 필요!" << std::endl;
            // 즉시 재계획 시도
            if (planCompleteAvoidancePath()) {
                current_path_index = 0;
                path_start_time = std::chrono::steady_clock::now();
                std::cout << "✅ 긴급 재계획 성공" << std::endl;
            } else {
                std::cout << "❌ 긴급 재계획 실패, 정상 모드로 복귀" << std::endl;
                state = LANE_TRACKING;
            }
        }
    }
    break;

    case RETURN_LANE:
    {
      bool lanes_detected = (main_window->left_detected || main_window->right_detected);

      if(waypoints.size() > 5 && lanes_detected){
        std::cout << "Lane recovered, returning to normal tracking" << std::endl;
        state = LANE_TRACKING;
        current_path_index = 0;
        a_waypoints.clear();
        tracking(waypoints);
      }
      else{
        std::cout << "Waiting for lane recovery... waypoints: " << waypoints.size() << ", lanes: " << lanes_detected << std::endl;
        drive(0.05, 0.0);  // 천천히 직진하며 차선 찾기
      }
    }
    break;
  }
}

void Driving::tracking(const std::vector<int>& waypoints){
  int target_idx = std::min(4, (int)waypoints.size() - 1);
  int target_waypoint = waypoints[target_idx];

  if(waypoints.empty()) return;

  int deviation = abs(target_waypoint - 320);

  // 🔧 속도 초기화 문제 해결
  double base_speed = (current_speed > 0.001) ? current_speed : 0.08;  // 기본 속도 8cm/s

  if(deviation < 10) {
    double target_speed = std::min(0.15, base_speed + 0.002);  // 최대 15cm/s
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

void Driving::a_tracking(const std::vector<int>& waypoints){
  if(waypoints.empty()) return;


  std::cout << "=== A* TRACKING (DIFFERENTIAL) DEBUG ===" << std::endl;
  std::cout << "Waypoints received: " << waypoints.size() << std::endl;
  std::cout << "Current path index: " << current_path_index << std::endl;

  // 🔧 차분 제어: 이전 웨이포인트와 현재 웨이포인트의 차이로 방향 결정
  static int previous_waypoint = 320;  // 이전 웨이포인트 (초기값: 중앙)

  int current_waypoint = waypoints[0];  // 현재 목표 웨이포인트
  int pixel_diff = current_waypoint - previous_waypoint + 320;  // 픽셀 차이 계산
  int deviation = abs(pixel_diff - 320);

  std::cout << "Previous: " << previous_waypoint << ", Current: " << current_waypoint << ", Diff: " << pixel_diff << std::endl;

  double base_speed = (current_speed > 0.001) ? current_speed : 0.08;  // 기본 속도 8cm/s

  if(deviation < 2) {
    double target_speed = std::min(0.02, base_speed + 0.002);  // 최대 15cm/s
    drive(target_speed, 0.0);
  }
  else {
    double normalized_deviation = std::min(deviation / 320.0, 1.0);
    double curve_factor = 1.0 - 0.3 * normalized_deviation * normalized_deviation;
    double target_speed = std::max(0.05, base_speed * curve_factor);
    double L = Look_aheadDistance(target_speed);
    double R = R_track(L, pixel_diff);
    double w = angular_velocity(R, target_speed);
    drive(target_speed, w* 1.2 );
  }

  // 이전 웨이포인트 업데이트
  previous_waypoint = current_waypoint;

  std::cout << "==================" << std::endl;
}

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
  const double MIN_L = 0.16;  // 0.1m
  const double MAX_L = 0.48;  // 0.5m

  double L = 2 * v / w_lim;
  return std::max(MIN_L, std::min(MAX_L, L));
}

void Driving::drive(double linear_x, double angular_z){
  if(qnode)
    qnode->drive(linear_x, angular_z);
}

void Driving::avoidanceMode() {
  // 🚨 기존 함수는 유지 (호환성을 위해)
  std::cout << "⚠️ avoidanceMode() called. Use planCompleteAvoidancePath() instead." << std::endl;
}

bool Driving::followPath() {
  // 🚨 기존 함수는 유지 (호환성을 위해)
  std::cout << "⚠️ followPath() called. Use executePathStep() instead." << std::endl;
  return true;
}

// 🆕 새로운 함수들 구현

// 완전한 회피 경로 계획 함수
bool Driving::planCompleteAvoidancePath() {
    if (!a_planner) {
        std::cout << "❌ A* 플래너가 없습니다!" << std::endl;
        return false;
    }

    std::cout << "\n=== 🗺️ 완전한 회피 경로 계획 시작 ===" << std::endl;

    // 현재 위치
    cv::Point2f current_pos(0.0, 0.0);

    // 🗺️ LiDAR 맵 업데이트
    if(qnode && qnode->lidar_received && !qnode->lidar_ranges.empty()) {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg->ranges = qnode->lidar_ranges;
        scan_msg->angle_min = qnode->lidar_angle_min;
        scan_msg->angle_max = qnode->lidar_angle_max;
        scan_msg->angle_increment = qnode->lidar_angle_increment;
        scan_msg->range_min = 0.1;
        scan_msg->range_max = 10.0;
        a_planner->updateMap(scan_msg, 0.0, 0.0, 0.0);
        std::cout << "✅ LiDAR 맵 업데이트 완료" << std::endl;
    }

    // 🎯 다양한 목표점 시도 (측면 우선)
        // cv::Point2f goal = cv::Point2f(0.0, 1.2);
        cv::Point2f goal = cv::Point2f(0.6, 0.0);
        std::cout << "\n🎯 회피 목표: (" << goal.x << ", " << goal.y << ")" << std::endl;

        std::vector<int> path = a_planner->planPath(current_pos, goal, 2000);

        if (!path.empty()) {
            // 🔍 경로 품질 분석
            int min_pixel = *std::min_element(path.begin(), path.end());
            int max_pixel = *std::max_element(path.begin(), path.end());
            int max_deviation = std::max(abs(min_pixel - 320), abs(max_pixel - 320));

            // 초기 우회 확인 (처음 30% 구간)
            int early_check = std::min(30, (int)(path.size() * 0.3));
            int early_deviation_sum = 0;
            for (int i = 0; i < early_check; i++) {
                early_deviation_sum += abs(path[i] - 320);
            }

            double avg_early_deviation = (double)early_deviation_sum / early_check;

            std::cout << "📊 경로 분석:" << std::endl;
            std::cout << "   웨이포인트 수: " << path.size() << std::endl;
            std::cout << "   픽셀 범위: " << min_pixel << " ~ " << max_pixel << std::endl;
            std::cout << "   최대 편차: " << max_deviation << " 픽셀" << std::endl;
            std::cout << "   초기 평균 편차: " << avg_early_deviation << " 픽셀" << std::endl;

            // 경로 미리보기
            std::cout << "   경로 미리보기: ";
            for (size_t i = 0; i < std::min((size_t)10, path.size()); i++) {
                std::cout << path[i] << " ";
            }
            if (path.size() > 10) std::cout << "...";
            std::cout << std::endl;

            // 🎯 경로 품질 판정
            bool good_path = false;

            if (goal.x != 0.0) {  // 측면 목표인 경우
                good_path = (max_deviation > 5) && (avg_early_deviation > 2.0);
            } else {  // 정면 목표인 경우
                good_path = (path.size() > 10);  // 일단 경로만 있으면 OK
            }

            if (good_path) {
                a_waypoints = path;
                global_path_ready = true;
                has_avoidance_goal = true;
                saved_avoidance_goal = goal;

                std::cout << "✅ 완전한 회피 경로 생성 성공!" << std::endl;
                std::cout << "   목표: (" << goal.x << ", " << goal.y << ")" << std::endl;
                std::cout << "   웨이포인트 수: " << path.size() << std::endl;
                std::cout << "================================\n" << std::endl;

                return true;
            } else {
                std::cout << "   ❌ 경로 품질 부족" << std::endl;
            }
        } else {
            std::cout << "   ❌ 경로 계획 실패" << std::endl;
        }


    std::cout << "❌ 모든 목표점에 대해 경로 계획 실패!" << std::endl;
    return false;
}

// 매우 제한적인 재계획 조건
bool Driving::shouldEmergencyReplan() {
    // 🚨 매우 예외적인 상황에서만 재계획

    // 1. 너무 오래 걸리는 경우 (30초 이상)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - path_start_time);
    if (elapsed.count() > 30) {
        std::cout << "⏰ 경로 실행 시간 초과 (30초)" << std::endl;
        return true;
    }

    // 2. 로봇이 심각하게 막힌 경우 (같은 웨이포인트에 5초 이상 머무름)
    static int stuck_counter = 0;
    static int last_path_index = -1;

    if (current_path_index == last_path_index) {
        stuck_counter++;
        if (stuck_counter > 150) {  // 5초 (30fps 기준)
            std::cout << "🚧 로봇이 같은 지점에 5초 이상 정체" << std::endl;
            stuck_counter = 0;
            return true;
        }
    } else {
        stuck_counter = 0;
        last_path_index = current_path_index;
    }

    // 3. 기타 긴급 상황들...
    // (필요시 추가)

    return false;  // 대부분은 재계획 안함
}

bool Driving::executePathStep() {
    if (current_path_index >= a_waypoints.size()) {
        std::cout << "✅ 경로 완주! 차선 추적 모드로 복귀" << std::endl;
        return false;  // 경로 완료
    }

    // 🎯 현재 목표 웨이포인트로 이동 (여러 웨이포인트 사용)
    int look_ahead = std::min(3, (int)(a_waypoints.size() - current_path_index));
    std::vector<int> current_waypoints;

    for(int i = 0; i < look_ahead; i++) {
        if(current_path_index + i < a_waypoints.size()) {
            current_waypoints.push_back(a_waypoints[current_path_index + i]);
        }
    }

    std::cout << "🎯 경로 실행: " << (current_path_index+1) << "/" << a_waypoints.size();
    std::cout << " (목표: ";
    for(int wp : current_waypoints) {
        std::cout << wp << " ";
    }
    std::cout << ")" << std::endl;

    // a_tracking 함수로 부드러운 추적
    a_tracking(current_waypoints);

    // 📍 다음 웨이포인트로 진행 조건 확인
    int target_x = a_waypoints[current_path_index];
    if (abs(target_x - 320) < 100) {  // 목표점에 충분히 가까워지면
        current_path_index++;
        std::cout << "✅ 웨이포인트 " << current_path_index << "/" << a_waypoints.size() << " 통과" << std::endl;
    }

    return true;  // 경로 계속 진행
}

void Driving::stopRobot() {
    drive(0.0, 0.0);
    std::cout << "🛑 로봇 완전 정지" << std::endl;
}

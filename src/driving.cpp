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
      // 매 프레임마다 장애물 체크
      if(qnode && qnode->detectObstacle(0.2, 1.0)){
        std::cout << "장애물 감지" << std::endl;

        // 즉시 회피 경로 계획 시도
        if (planCompleteAvoidancePath()) {
            state = AVOIDANCE;
            current_path_index = 0;
            path_start_time = std::chrono::steady_clock::now();
            std::cout << "회피 경로 계획 완료" << std::endl;
        } else {
            std::cout << "회피 경로 계획 실패, 정지" << std::endl;
            stopRobot();
            // 다음 프레임에서 다시 시도
        }
      }
      else{
        tracking(waypoints);
      }
    }
    break;


case AVOIDANCE:
    {
        // 경로 끝까지 따라가기
        if (current_path_index < a_waypoints.size()) {

            // 현재 진행률 표시
            double progress = (double)(current_path_index + 1) / a_waypoints.size() * 100.0;

            std::cout << "경로 실행: " << (current_path_index + 1) << "/" << a_waypoints.size()
                      << " (" << std::fixed << std::setprecision(1) << progress << "%)" << std::endl;

            // 경로 실행
            bool reached = executePathStep();

            if (reached) {
                //current_path_index++;
                std::cout << "웨이포인트 " << current_path_index << "/" << a_waypoints.size() << " 통과" << std::endl;

                //  경로 완주 체크
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
            std::cout << "웨이포인트 없음, 정상 모드로 복귀" << std::endl;
            state = LANE_TRACKING;
        }

        // 매우 제한적인 재계획 조건 (선택사항)
        if (shouldEmergencyReplan()) {
            std::cout << " 긴급 재계획 필요!" << std::endl;
            // 즉시 재계획 시도
            if (planCompleteAvoidancePath()) {
                current_path_index = 0;
                path_start_time = std::chrono::steady_clock::now();
                std::cout << " 긴급 재계획 성공" << std::endl;
            } else {
                std::cout << " 긴급 재계획 실패, 정상 모드로 복귀" << std::endl;
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

  //  속도 초기화 문제 해결
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

void Driving::a_tracking(const cv::Point2f& world_target) {

    // 속도 설정 (이것은 Pure Pursuit의 L과는 별개로 제어 속도를 결정)
    double base_speed = (current_speed > 0.001) ? current_speed : 0.08;
    double target_speed = std::min(0.15, base_speed + 0.002);

    double x_t = world_target.x; // 횡방향 편차
    double y_t = world_target.y; // 종방향 거리

    // 목표점까지의 실제 직선 거리 L 계산
    double L_squared = x_t * x_t + y_t * y_t;

    // 안전을 위한 최소 거리 체크
    if (L_squared < 0.001) {
        drive(target_speed, 0.0);
        return;
    }

    // Pure Pursuit Curvature (곡률 kappa) 계산
    // kappa = (2 * x_t) / L^2
    double kappa = (2.0 * x_t) / L_squared;

    // 각속도 (w) 계산
    double w = target_speed * kappa;

    // w_lim 제한
    w = std::max(-this->w_lim, std::min(this->w_lim, w));

    // 최종 주행 명령
    drive(target_speed, w);
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

bool Driving::followPath() {

  std::cout << "⚠️ followPath() called. Use executePathStep() instead." << std::endl;
  return true;
}

// 완전한 회피 경로 계획 함수
bool Driving::planCompleteAvoidancePath() {
    if (!a_planner) {
        std::cout << "A* 플래너가 없습니다!" << std::endl;
        return false;
    }

    std::cout << "\n=== 완전한 회피 경로 계획 시작 ===" << std::endl;

    // 현재 위치
    cv::Point2f current_pos(0.0, 0.0);

    // LiDAR 맵 업데이트
    if(qnode && qnode->lidar_received && !qnode->lidar_ranges.empty()) {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg->ranges = qnode->lidar_ranges;
        scan_msg->angle_min = qnode->lidar_angle_min;
        scan_msg->angle_max = qnode->lidar_angle_max;
        scan_msg->angle_increment = qnode->lidar_angle_increment;
        scan_msg->range_min = 0.1;
        scan_msg->range_max = 10.0;
        a_planner->updateMap(scan_msg, 0.0, 0.0, 0.0);
        std::cout << "LiDAR 맵 업데이트 완료" << std::endl;
    }

        // cv::Point2f goal = cv::Point2f(0.0, 1.2);
        cv::Point2f goal = cv::Point2f(0.1, 0.3);
        std::cout << "\n 회피 목표: (" << goal.x << ", " << goal.y << ")" << std::endl;

        std::vector<cv::Point2f> path = a_planner->planPath(current_pos, goal, 2000);

        if (!path.empty()) {
            // 경로 품질 분석

            auto min_it = std::min_element(path.begin(), path.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });
            auto max_it = std::max_element(path.begin(), path.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });

            double min_world_x = (min_it != path.end()) ? min_it->x : 0.0;
            double max_world_x = (max_it != path.end()) ? max_it->x : 0.0;

            double max_deviation = std::max(std::abs(min_world_x), std::abs(max_world_x));

            // 초기 우회 확인 (처음 30% 구간)
            int early_check = std::min(30, (int)(path.size() * 0.3));
            double early_deviation_sum = 0.0;
            for (int i = 0; i < early_check; i++) {
                early_deviation_sum += std::abs(path[i].x);
            }

            double avg_early_deviation = (double)early_deviation_sum / early_check;

            std::cout << "경로 분석:" << std::endl;

            std::cout << "경로 분석:" << std::endl;
            std::cout << "   웨이포인트 수: " << path.size() << std::endl;
            std::cout << "   월드 X 범위: " << std::fixed << std::setprecision(3) << min_world_x << " ~ " << max_world_x << " m" << std::endl;
            std::cout << "   최대 편차: " << max_deviation << " m" << std::endl;
            std::cout << "   초기 평균 편차: " << avg_early_deviation << " m" << std::endl;

            // 경로 미리보기
            std::cout << "   경로 미리보기: ";
            //for (size_t i = 0; i < std::min((size_t)10, path.size()); i++) {
            for (size_t i = 0; i < path.size(); i++) {
                std::cout << "("<< path[i].x<< "," << path[i].y<< ")";
            }
            //if (path.size() > 10) std::cout << "...";
            std::cout << std::endl;

            // 경로 품질 판정
            bool good_path = false;

            if (goal.x != 0.0) {  // 측면 목표인 경우
                good_path = (max_deviation > 0.05) && (avg_early_deviation > 0.01);
            } else {  // 정면 목표인 경우
                good_path = (path.size() > 10);  // 일단 경로만 있으면 OK
            }

            if (good_path) {
                a_waypoints = path;
                global_path_ready = true;
                has_avoidance_goal = true;
                saved_avoidance_goal = goal;

                std::cout << "완전한 회피 경로 생성 성공!" << std::endl;
                std::cout << "   목표: (" << goal.x << ", " << goal.y << ")" << std::endl;
                std::cout << "   웨이포인트 수: " << path.size() << std::endl;
                std::cout << "================================\n" << std::endl;

                return true;
            } else {
                std::cout << "   경로 품질 부족" << std::endl;
            }
        } else {
            std::cout << "  경로 계획 실패" << std::endl;
        }


    std::cout << " 모든 목표점에 대해 경로 계획 실패!" << std::endl;
    return false;
}


bool Driving::shouldEmergencyReplan() {
    // 매우 예외적인 상황에서만 재계획

    // 1. 너무 오래 걸리는 경우 (30초 이상)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - path_start_time);
    if (elapsed.count() > 30) {
        std::cout << "경로 실행 시간 초과 (30초)" << std::endl;
        return true;
    }

    // 2. 로봇이 심각하게 막힌 경우 (같은 웨이포인트에 5초 이상 머무름)
    static int stuck_counter = 0;
    static int last_path_index = -1;

    if (current_path_index == last_path_index) {
        stuck_counter++;
        if (stuck_counter > 150) {  // 5초 (30fps 기준)
            std::cout << "로봇이 같은 지점에 5초 이상 정체" << std::endl;
            stuck_counter = 0;
            return true;
        }
    } else {
        stuck_counter = 0;
        last_path_index = current_path_index;
    }
    return false;  // 대부분은 재계획 안함
}

bool Driving::executePathStep() {
    if (current_path_index >= a_waypoints.size()) {
        std::cout << "경로 완주! 차선 추적 모드로 복귀" << std::endl;
        return false;  // 경로 완료
    }

    // Lookahead Distance (Ld) 설정
    double base_speed = (current_speed > 0.001) ? current_speed : 0.08;
    double L_d = Look_aheadDistance(base_speed); // L_d: Lookahead Distance (m)

    // Lookahead Point 검색
    cv::Point2f target_world_waypoint = a_waypoints.back();

    for (size_t i = current_path_index; i < a_waypoints.size(); i++) {
        const cv::Point2f& world_pos = a_waypoints[i];

        // 로봇과의 실제 직선 거리가 L_d보다 크거나 같은지 확인
        double actual_distance_L = std::sqrt(world_pos.x * world_pos.x + world_pos.y * world_pos.y);

        if (actual_distance_L >= L_d) {
            target_world_waypoint = a_waypoints[i];
            break;
        }
    }

    a_tracking(target_world_waypoint);

    double target_x = a_waypoints[current_path_index].x;
    double target_y = a_waypoints[current_path_index].y;

    // 목표점까지의 2차원 거리 계산
    double distance_sq = target_x * target_x + target_y * target_y;
    const double PASS_THRESHOLD_SQ = 0.01 * 0.01; // 예: 10cm 이내 (0.01m^2)

    if (distance_sq < PASS_THRESHOLD_SQ) {
        current_path_index++;
        std::cout << "웨이포인트 " << current_path_index << "/" << a_waypoints.size() << " 통과 (거리)" << std::endl;
    }

    return true;  // 경로 계속 진행
}

void Driving::stopRobot() {
    drive(0.0, 0.0);
    std::cout << "로봇 완전 정지" << std::endl;
}

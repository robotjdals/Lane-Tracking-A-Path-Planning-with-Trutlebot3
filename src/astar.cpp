#include "../include/min_22_pkg/astar.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>


Astar::Astar() {
    gridmap = cv::Mat::zeros(height, width, CV_8UC1); //300x300
    visualization_callback = nullptr;
    std::cout << "🚀 Simple & Reliable A* initialized!" << std::endl;
}


std::vector<int> Astar::planPath(cv::Point2f start, cv::Point2f goal, int max_iterations) {
    std::cout << "\n========== A* PLANNING ==========" << std::endl;
    std::cout << "Planning: (" << start.x << ", " << start.y << ") → (" << goal.x << ", " << goal.y << ")" << std::endl;

    cv::Point2i start_grid = worldToGrid(start.x, start.y);
    cv::Point2i goal_grid = worldToGrid(goal.x, goal.y);

    std::cout << "Grid: (" << start_grid.x << ", " << start_grid.y << ") → (" << goal_grid.x << ", " << goal_grid.y << ")" << std::endl;

    if (goal_grid.x < 0 || goal_grid.x >= width || goal_grid.y < 0 || goal_grid.y >= height ||
        start_grid.x < 0 || start_grid.x >= width || start_grid.y < 0 || start_grid.y >= height) {
        std::cout << "Outside bounds!" << std::endl;
        return {};
    }

    // 🔧 헤더의 AstarNode 사용 (SimpleNode 제거)
    auto cmp = [](std::shared_ptr<AstarNode> a, std::shared_ptr<AstarNode> b) {
        return a->f_cost > b->f_cost;
    };
    std::priority_queue<std::shared_ptr<AstarNode>,
                        std::vector<std::shared_ptr<AstarNode>>,
                        decltype(cmp)> open_set(cmp);

    // 방문 체크 (shared_ptr 사용)
    std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
    std::vector<std::vector<std::shared_ptr<AstarNode>>> nodes(width,
        std::vector<std::shared_ptr<AstarNode>>(height, nullptr));

    // 시작 노드 생성
    double h_start = std::abs(goal_grid.x - start_grid.x) + std::abs(goal_grid.y - start_grid.y);
    auto start_node = std::make_shared<AstarNode>(start_grid.x, start_grid.y, 0.0, h_start);

    open_set.push(start_node);
    nodes[start_grid.x][start_grid.y] = start_node;

    // 8방향 이동
    int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
    int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};
    double costs[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    std::shared_ptr<AstarNode> goal_node = nullptr;
    int iterations = 0;

    // A* 메인 루프
    while (!open_set.empty() && iterations < max_iterations) {
        auto current = open_set.top();
        open_set.pop();

        if (visited[current->x][current->y]) continue;
        visited[current->x][current->y] = true;

        iterations++;

        // 목표 도달
        if (current->x == goal_grid.x && current->y == goal_grid.y) {
            goal_node = current;
            std::cout << "🎉 SUCCESS in " << iterations << " iterations!" << std::endl;
            break;
        }

        // 이웃 탐색
        for (int i = 0; i < 8; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            // 범위 체크
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            if (visited[nx][ny]) continue;

            // 🔧 개선된 3x3 영역 충돌 검사
            bool collision = false;
            for(int dy = -1; dy <= 1; dy++) {
                for(int dx = -1; dx <= 1; dx++) {
                    int check_x = nx + dx;
                    int check_y = ny + dy;

                    if(check_x >= 0 && check_x < width &&
                       check_y >= 0 && check_y < height) {
                        if(gridmap.at<uchar>(check_y, check_x) > 0) {
                            collision = true;
                            break;
                        }
                    }
                }
                if(collision) break;
            }
            if(collision) continue;

            double new_g = current->g_cost + costs[i];  // g_cost 사용
            double h = std::abs(goal_grid.x - nx) + std::abs(goal_grid.y - ny);

            // 기존 노드보다 좋은 경로인지 확인
            if (nodes[nx][ny] == nullptr || new_g < nodes[nx][ny]->g_cost) {
                auto neighbor = std::make_shared<AstarNode>(nx, ny, new_g, h);
                neighbor->parent = current;

                nodes[nx][ny] = neighbor;
                open_set.push(neighbor);
            }
        }
    }

    // 경로 재구성
        std::vector<int> pixel_path;

    if (goal_node) {
        std::vector<cv::Point2f> world_path;
        auto current = goal_node;

        std::cout << "\n🔍 Path reconstruction (BEFORE reverse):" << std::endl;
        int step = 0;

        // 🔧 경로 수집 (목표 → 시작 순서)
        while (current) {
            cv::Point2f world_pos = gridToWorld(current->x, current->y);
            world_path.push_back(world_pos);

            // 디버그 로그
            std::cout << "  Step " << step++ << ": Grid(" << current->x << ", " << current->y
                      << ") → World(" << world_pos.x << ", " << world_pos.y << ")" << std::endl;

            current = current->parent;
        }

        std::cout << "\n🔄 Reversing path..." << std::endl;
        std::reverse(world_path.begin(), world_path.end());

        std::cout << "🔍 Path AFTER reverse:" << std::endl;
        for (size_t i = 0; i < world_path.size(); i++) {
            std::cout << "  " << i << ": (" << world_path[i].x << ", " << world_path[i].y << ")" << std::endl;
        }

        // 🔧 픽셀 변환도 디버깅
        std::cout << "\n🔍 World to Pixel conversion:" << std::endl;
        for (size_t i = 0; i < world_path.size(); i++) {
            const auto& point = world_path[i];
            int pixel_x = static_cast<int>(point.x / pixel_to_meter + 320);
            pixel_x = std::max(0, std::min(639, pixel_x));
            pixel_path.push_back(pixel_x);

            std::cout << "  " << i << ": World(" << point.x << ", " << point.y
                      << ") → Pixel X: " << pixel_x << std::endl;
        }

        // 🔧 방향 검증
        if (pixel_path.size() >= 2) {
            int start_pixel = static_cast<int>(start.x / pixel_to_meter + 320);
            int goal_pixel = static_cast<int>(goal.x / pixel_to_meter + 320);

            std::cout << "\n🔍 Direction validation:" << std::endl;
            std::cout << "  Expected start pixel: " << start_pixel << ", Actual first: " << pixel_path[0] << std::endl;
            std::cout << "  Expected goal pixel: " << goal_pixel << ", Actual last: " << pixel_path.back() << std::endl;

            bool correct_direction = (abs(pixel_path[0] - start_pixel) < abs(pixel_path[0] - goal_pixel));
            std::cout << "  Direction check: " << (correct_direction ? "✅ CORRECT" : "❌ REVERSED") << std::endl;
        }

        std::cout << "📊 Final pixel path: ";
        for (size_t i = 0; i < std::min((size_t)10, pixel_path.size()); i++) {
            std::cout << pixel_path[i] << " ";
        }
        std::cout << std::endl;

    } else {
        std::cout << "❌ No path found" << std::endl;
    }

    std::cout << "==========================================\n" << std::endl;
    return pixel_path;
}


void Astar::updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                      double robot_x, double robot_y, double robot_theta) {
    std::lock_guard<std::mutex> lock(map_mutex);

    if (!scan || scan->ranges.empty()) return;

    gridmap = cv::Mat::zeros(height, width, CV_8UC1);

    for (size_t i = 0; i < scan->ranges.size(); i++) {
        double range = scan->ranges[i];
        if (range < 0.1 || range > 3.0 || std::isinf(range) || std::isnan(range)) continue;

        double angle = scan->angle_min + i * scan->angle_increment + robot_theta;
        double rx = range * std::cos(angle);
        double ry = range * std::sin(angle);

        double c = std::cos(robot_theta), s = std::sin(robot_theta);
        double wx = rx * c - ry * s;
        double wy = rx * s + ry * c;
        double world_x = robot_x + wy;
        double world_y = robot_y + wx;

        cv::Point2i grid_pos = worldToGrid(world_x, world_y);

        if (grid_pos.x >= 0 && grid_pos.x < width && grid_pos.y >= 0 && grid_pos.y < height) {
            cv::circle(gridmap, grid_pos, 8, 255, -1);  // 작은 장애물
        }
    }

    notifyVisualizationUpdate();
}

cv::Point2i Astar::worldToGrid(double world_x, double world_y) const {
    int grid_x = static_cast<int>(world_x / resolution + width/2);
    int grid_y = static_cast<int>(world_y / resolution + height/2);
    grid_x = std::max(0, std::min(width-1, grid_x));
    grid_y = std::max(0, std::min(height-1, grid_y));
    return cv::Point2i(grid_x, grid_y);
}

cv::Point2f Astar::gridToWorld(int grid_x, int grid_y) const {
    double world_x = (grid_x - width/2) * resolution;
    double world_y = (grid_y - height/2) * resolution;
    return cv::Point2f(world_x, world_y);
}


std::vector<int> Astar::worldToPixel(const std::vector<cv::Point2f>& world_path) const {
    std::vector<int> pixel_waypoints;
    for (const auto& point : world_path) {
        int pixel_x = static_cast<int>(point.x / pixel_to_meter + 320);
        pixel_x = std::max(0, std::min(639, pixel_x));
        pixel_waypoints.push_back(pixel_x);
    }
    return pixel_waypoints;
}
/*
std::vector<cv::Point> Astar::worldToPixel(const std::vector<cv::Point2f>& world_path) const {
    std::vector<cv::Point> pixel_waypoints;

    std::cout << "\n🔍 World to Pixel XY conversion:" << std::endl;

    for (size_t i = 0; i < world_path.size(); i++) {
        const auto& point = world_path[i];

        // X, Y 모두 변환
        int pixel_x = static_cast<int>(point.x / pixel_to_meter + 320);
        int pixel_y = static_cast<int>(point.y / (-pixel_to_meter) + 240);

        pixel_x = std::max(0, std::min(639, pixel_x));
        pixel_y = std::max(0, std::min(479, pixel_y));

        pixel_waypoints.push_back(cv::Point(pixel_x, pixel_y));

        if (i < 5 || i >= world_path.size() - 3) {
            std::cout << "  " << i << ": World(" << point.x << ", " << point.y
                      << ") → Pixel(" << pixel_x << ", " << pixel_y << ")" << std::endl;
        } else if (i == 5) {
            std::cout << "  ... (생략) ..." << std::endl;
        }
    }

    return pixel_waypoints;
}*/



cv::Mat Astar::getgridmap() const {
    std::lock_guard<std::mutex> lock(map_mutex);
    return gridmap.clone();
}

cv::Mat Astar::getVisualizationMap() const {
    std::lock_guard<std::mutex> lock(map_mutex);
    cv::Mat vis_map = gridmap.clone();
    cv::Mat color_map;
    cv::cvtColor(vis_map, color_map, cv::COLOR_GRAY2BGR);
    cv::circle(color_map, cv::Point(width/2, height/2), 5, cv::Scalar(255, 0, 0), -1);
    return color_map;
}

cv::Mat Astar::getVisualizationMapWithPath(const std::vector<cv::Point2f>& path) const {
    cv::Mat color_map = getVisualizationMap();
    if (!path.empty()) {
        for (size_t i = 1; i < path.size(); i++) {
            cv::Point2i prev = worldToGrid(path[i-1].x, path[i-1].y);
            cv::Point2i curr = worldToGrid(path[i].x, path[i].y);
            cv::line(color_map, cv::Point(prev.x, prev.y), cv::Point(curr.x, curr.y), cv::Scalar(0, 0, 255), 2);
        }
    }
    return color_map;
}

/*
cv::Mat Astar::getVisualizationMapWithPixelPath(const std::vector<int>& pixel_waypoints,
                                                    int current_index) const {
    std::lock_guard<std::mutex> lock(map_mutex);

    cv::Mat vis_map = gridmap.clone();
    cv::Mat color_map;
    cv::cvtColor(vis_map, color_map, cv::COLOR_GRAY2BGR);

    // 로봇 위치 (중앙)
    cv::Point robot_pos(width/2, height/2);
    cv::circle(color_map, robot_pos, 5, cv::Scalar(255, 0, 0), -1);

    if (pixel_waypoints.empty()) return color_map;

    // 🔧 실제 A* 계산된 월드 좌표 사용 (임의 생성 X)
    std::vector<cv::Point2f> world_path;
    for (const auto& pp : pixel_waypoints) {
        double world_x = (pp.x - 320) * pixel_to_meter;
        double world_y = (pp.y - 240) * (-pixel_to_meter);  // 실제 Y 좌표 역변환
        world_path.push_back(cv::Point2f(world_x, world_y));
    }

    // 전체 경로 그리기
    for (size_t i = 1; i < world_path.size(); i++) {
        cv::Point2i prev = worldToGrid(world_path[i-1].x, world_path[i-1].y);
        cv::Point2i curr = worldToGrid(world_path[i].x, world_path[i].y);
        cv::line(color_map, prev, curr, cv::Scalar(128, 128, 128), 1);
    }

    // 지나온 경로 (초록색)
    if (current_index > 0) {
        for (size_t i = 1; i <= std::min((size_t)current_index, world_path.size()-1); i++) {
            cv::Point2i prev = worldToGrid(world_path[i-1].x, world_path[i-1].y);
            cv::Point2i curr = worldToGrid(world_path[i].x, world_path[i].y);
            cv::line(color_map, prev, curr, cv::Scalar(0, 255, 0), 2);
        }
    }

    // 웨이포인트 표시
    for (size_t i = 0; i < world_path.size(); i++) {
        cv::Point2i curr = worldToGrid(world_path[i].x, world_path[i].y);

        if (i < (size_t)current_index) {
            cv::circle(color_map, curr, 4, cv::Scalar(0, 255, 0), -1);  // 완료 (초록)
        } else if (i == (size_t)current_index) {
            cv::circle(color_map, curr, 8, cv::Scalar(0, 255, 255), -1);  // 현재 (노랑)
        } else {
            cv::circle(color_map, curr, 4, cv::Scalar(0, 0, 255), -1);  // 미래 (빨강)
        }
    }

    // 정보 텍스트
    std::string info = "Waypoint: " + std::to_string(current_index + 1) + "/" + std::to_string(pixel_waypoints.size());
    cv::putText(color_map, info, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

    return color_map;
}*/
cv::Mat Astar::getVisualizationMapWithPixelPath(const std::vector<int>& pixel_waypoints, int current_index) const {
    std::lock_guard<std::mutex> lock(map_mutex);

    // 기본 맵 (장애물 포함)
    cv::Mat vis_map = gridmap.clone();
    cv::Mat color_map;
    cv::cvtColor(vis_map, color_map, cv::COLOR_GRAY2BGR);

    // 로봇 위치 (중앙)
    cv::Point robot_pos(width/2, height/2);
    cv::circle(color_map, robot_pos, 5, cv::Scalar(255, 0, 0), -1);  // 파란색 로봇

    if (pixel_waypoints.empty()) return color_map;

    // 픽셀 웨이포인트를 world 좌표로 변환
    std::vector<cv::Point2f> world_path;
    for (size_t i = 0; i < pixel_waypoints.size(); i++) {
        double world_x = (pixel_waypoints[i] - 320) * pixel_to_meter;
        double world_y = i * 0.30;  // 30cm 간격
        world_path.push_back(cv::Point2f(world_x, world_y));
    }

    // 전체 경로 그리기 (회색 - 기본)
    for (size_t i = 1; i < world_path.size(); i++) {
        cv::Point2i prev = worldToGrid(world_path[i-1].x, world_path[i-1].y);
        cv::Point2i curr = worldToGrid(world_path[i].x, world_path[i].y);
        cv::line(color_map, prev, curr, cv::Scalar(128, 128, 128), 1);  // 회색 전체 경로
    }

    // 지나온 경로 (초록색 - 완료된 부분) - current_index > 0일 때만
    if (current_index > 0) {
        for (size_t i = 1; i <= std::min((size_t)current_index, world_path.size()-1); i++) {
            cv::Point2i prev = worldToGrid(world_path[i-1].x, world_path[i-1].y);
            cv::Point2i curr = worldToGrid(world_path[i].x, world_path[i].y);
            cv::line(color_map, prev, curr, cv::Scalar(0, 255, 0), 2);  // 초록색 완료 경로
        }
    }

    // 웨이포인트 표시
    for (size_t i = 0; i < world_path.size(); i++) {
        cv::Point2i curr = worldToGrid(world_path[i].x, world_path[i].y);

        if (i < (size_t)current_index) {
            // 완료된 웨이포인트 (초록색)
            cv::circle(color_map, curr, 4, cv::Scalar(0, 255, 0), -1);
        } else if (i == (size_t)current_index) {
            // 현재 목표 웨이포인트 (노란색 - 더 잘 보이게)
            cv::circle(color_map, curr, 8, cv::Scalar(0, 255, 255), -1);  // 노란색
            cv::circle(color_map, curr, 10, cv::Scalar(0, 200, 200), 2);
        } else {
            // 미래 웨이포인트 (빨간색)
            cv::circle(color_map, curr, 4, cv::Scalar(0, 0, 255), -1);
        }
    }

    // 정보 텍스트
    std::string info = "Waypoint: " + std::to_string(current_index + 1) + "/" + std::to_string(pixel_waypoints.size());
    cv::putText(color_map, info, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

    // 색상 범례
    std::string legend = "Green: Completed | Yellow: Current | Red: Future";
    cv::putText(color_map, legend, cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);

    // 진행률 표시
    if (!pixel_waypoints.empty()) {
        double progress = (double)current_index / pixel_waypoints.size() * 100.0;
        std::string progress_info = "Progress: " + std::to_string((int)progress) + "%";
        cv::putText(color_map, progress_info, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    }

    return color_map;
}

std::vector<cv::Point2f> Astar::planGlobalPath(cv::Point2f start, cv::Point2f final_goal, double planning_horizon) {
    std::vector<int> pixel_path = planPath(start, final_goal, 1000);

    if(pixel_path.empty()) {
        return {};
    }

    std::vector<cv::Point2f> global_path;

    double waypoint_interval = 0.30;  // 30cm 간격 (큰 스텝)

    for(size_t i = 0; i < pixel_path.size(); i += 5) {  // 🔧 5개마다 하나씩 (더 간격 벌림)
        double world_x = (pixel_path[i] - 320) * pixel_to_meter;
        double world_y = i * waypoint_interval * 0.01;  // y는 인덱스에 비례
        global_path.push_back(cv::Point2f(world_x, world_y));

        if(world_y >= planning_horizon) break;
    }

    std::cout << "📊 Optimized Path:" << std::endl;
    std::cout << "   Original pixels: " << pixel_path.size() << std::endl;
    std::cout << "   Final waypoints: " << global_path.size() << std::endl;
    std::cout << "   Waypoint interval: " << waypoint_interval << "m" << std::endl;

    // 🔍 경로 미리보기 (픽셀)
    std::cout << "   Sampled pixels: ";
    for(size_t i = 0; i < global_path.size() && i < 10; i++) {
        int sample_pixel = static_cast<int>(global_path[i].x / pixel_to_meter + 320);
        std::cout << sample_pixel << " ";
    }
    std::cout << std::endl;

    return global_path;
}

std::vector<int> Astar::planLocalPath(cv::Point2f start, cv::Point2f local_goal, double planning_horizon) {
    std::vector<int> full_path = planPath(start, local_goal, 500);

    if (full_path.empty()) return {};

    // 🔧 로컬 경로도 간소화 (2개마다 하나씩)
    std::vector<int> simplified_path;
    for (size_t i = 0; i < full_path.size(); i += 2) {
        simplified_path.push_back(full_path[i]);
    }

    std::cout << "📊 Local Path: " << full_path.size() << " → " << simplified_path.size() << " waypoints" << std::endl;

    return simplified_path;
}

cv::Point2f Astar::getLocalGoalFromGlobal(const std::vector<cv::Point2f>& global_path, cv::Point2f current_pos, double lookahead) {
    if(global_path.empty()) return cv::Point2f(0.0, 0.5);
    for(const auto& point : global_path) {
        double distance = std::sqrt((point.x - current_pos.x) * (point.x - current_pos.x) + (point.y - current_pos.y) * (point.y - current_pos.y));
        if(distance >= lookahead) return point;
    }
    return global_path.back();
}

void Astar::setLanePoints(const std::vector<cv::Point2f>& left_points, const std::vector<cv::Point2f>& right_points) {
    left_lane_points = left_points;
    right_lane_points = right_points;
    lane_data_available = !left_points.empty() && !right_points.empty();
}

void Astar::clearLanePoints() {
    left_lane_points.clear();
    right_lane_points.clear();
    lane_data_available = false;
}

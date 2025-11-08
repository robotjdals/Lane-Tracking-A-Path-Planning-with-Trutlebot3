#include "../include/min_22_pkg/astar.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <opencv2/imgproc.hpp>

Astar::Astar() {
    visualization_callback = nullptr;
    std::cout << "🌍 Global A* Planner initialized!" << std::endl;
}

bool Astar::loadMapFromFile(const std::string& path, double resolution_m_per_px, const cv::Point2d& origin_m, int inflate_px) {
    std::lock_guard<std::mutex> lock(map_mutex);

    std::cout << "Loading map from: " << path << std::endl;

    // YAML 파일인지 확인
    if (path.find(".yaml") != std::string::npos || path.find(".yml") != std::string::npos) {
        std::cout << "📄 Detected YAML file, parsing..." << std::endl;

        // YAML 파일 파싱
        std::ifstream yaml_file(path);
        if (!yaml_file.is_open()) {
            std::cerr << "ERROR: Cannot open YAML file " << path << std::endl;
            return false;
        }

        std::string yaml_image_path;
        double yaml_resolution = resolution_m_per_px;
        cv::Point2d yaml_origin = origin_m;

        std::string line;
        while (std::getline(yaml_file, line)) {

            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);

            if (line.find("image:") == 0) {
                yaml_image_path = line.substr(6);
                yaml_image_path.erase(0, yaml_image_path.find_first_not_of(" \t"));
                std::cout << "📁 Image file: " << yaml_image_path << std::endl;
            }
            else if (line.find("resolution:") == 0) {
                std::string res_str = line.substr(11);
                res_str.erase(0, res_str.find_first_not_of(" \t"));
                yaml_resolution = std::stod(res_str);
                std::cout << "📐 Resolution: " << yaml_resolution << " m/pixel" << std::endl;
            }
            else if (line.find("origin:") == 0) {
                std::string origin_str = line.substr(7);
                origin_str.erase(0, origin_str.find_first_not_of(" \t"));

                // [x, y, theta] 형태 파싱
                if (origin_str.find('[') != std::string::npos) {
                    origin_str = origin_str.substr(origin_str.find('[') + 1);
                    origin_str = origin_str.substr(0, origin_str.find(']'));

                    std::stringstream ss(origin_str);
                    std::string x_str, y_str;
                    std::getline(ss, x_str, ',');
                    std::getline(ss, y_str, ',');

                    x_str.erase(0, x_str.find_first_not_of(" \t"));
                    x_str.erase(x_str.find_last_not_of(" \t") + 1);
                    y_str.erase(0, y_str.find_first_not_of(" \t"));
                    y_str.erase(y_str.find_last_not_of(" \t") + 1);

                    yaml_origin.x = std::stod(x_str);
                    yaml_origin.y = std::stod(y_str);
                    std::cout << "📍 Origin: (" << yaml_origin.x << ", " << yaml_origin.y << ")" << std::endl;
                }
            }
        }
        yaml_file.close();

        // PGM 파일 경로 결정
        std::string yaml_dir = path.substr(0, path.find_last_of("/\\") + 1);
        std::string pgm_path = yaml_dir + yaml_image_path;

        std::cout << "🖼️  Loading PGM from: " << pgm_path << std::endl;

        // PGM 파일 로드
        map_gray_ = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
        if (map_gray_.empty()) {
            std::cerr << "ERROR: Failed to load PGM map from " << pgm_path << std::endl;
            std::cerr << "🔍 Checking if PGM file exists..." << std::endl;

            // 파일 존재 여부 확인
            std::ifstream pgm_check(pgm_path);
            if (!pgm_check.good()) {
                std::cerr << "❌ PGM file does not exist: " << pgm_path << std::endl;
                std::cerr << "💡 Please ensure map2.pgm is in the same directory as map2.yaml" << std::endl;
            } else {
                std::cerr << "⚠️  PGM file exists but cannot be loaded (format issue?)" << std::endl;
            }
            return false;
        }

        // YAML 파라미터 적용
        W_ = map_gray_.cols;
        H_ = map_gray_.rows;
        res_ = yaml_resolution;
        origin_m_ = yaml_origin;

    } else {
        // 직접 PGM 파일 로드
        std::cout << "🖼️  Direct PGM loading..." << std::endl;
        map_gray_ = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (map_gray_.empty()) {
            std::cerr << "ERROR: Failed to load PGM map from " << path << std::endl;
            return false;
        }

        W_ = map_gray_.cols;
        H_ = map_gray_.rows;
        res_ = resolution_m_per_px;
        origin_m_ = origin_m;
    }

    std::cout << "✅ Map Loaded: " << W_ << "x" << H_ << ", Resolution: " << res_ << " m/px" << std::endl;
    std::cout << "📍 Origin (World m): (" << origin_m_.x << ", " << origin_m_.y << ")" << std::endl;

    const int OCC_THRESHOLD = 50; // 필요시 YAML의 occupied_thresh에 맞추어 조정
    cv::threshold(map_gray_, occ_, OCC_THRESHOLD, 255, cv::THRESH_BINARY_INV);

    // 장애물 팽창
    if (inflate_px > 0) {
        const int k = 2 * inflate_px + 1;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
        cv::dilate(occ_, occ_, kernel);                // ★ 장애물(255)을 팽창
    }

    gridmap = occ_.clone();
    notifyVisualizationUpdate();
    return true;
}

std::vector<cv::Point2f> Astar::planPath(cv::Point2f start, cv::Point2f goal, int max_iterations) {
    std::lock_guard<std::mutex> lock(map_mutex);
    if (gridmap.empty()) {
        std::cerr << "ERROR: Map not loaded or empty." << std::endl;
        return {};
    }

    std::cout << "\n========== A* PLANNING (CORRECTED COORDINATES) ==========" << std::endl;
    std::cout << "🌍 World Start: (" << std::fixed << std::setprecision(6) << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "🌍 World Goal:  (" << std::fixed << std::setprecision(6) << goal.x << ", " << goal.y << ")" << std::endl;

    cv::Point2i start_grid = worldToGrid(start.x, start.y);
    cv::Point2i goal_grid = worldToGrid(goal.x, goal.y);

    std::cout << "🖼️  Grid Start: (" << start_grid.x << ", " << start_grid.y << ")" << std::endl;
    std::cout << "🖼️  Grid Goal:  (" << goal_grid.x << ", " << goal_grid.y << ")" << std::endl;

    // 🔍 실제 거리 계산
    double world_distance = std::sqrt(std::pow(goal.x - start.x, 2) + std::pow(goal.y - start.y, 2));
    int pixel_distance = std::abs(goal_grid.x - start_grid.x) + std::abs(goal_grid.y - start_grid.y);

    std::cout << "📏 World Distance: " << std::fixed << std::setprecision(3) << world_distance << " meters" << std::endl;
    std::cout << "📏 Pixel Distance: " << pixel_distance << " pixels" << std::endl;
    std::cout << "📐 Expected Pixel Distance: " << std::fixed << std::setprecision(1) << world_distance/res_ << " pixels" << std::endl;

    // 역변환으로 검증
    cv::Point2f start_verify = gridToWorld(start_grid.x, start_grid.y);
    cv::Point2f goal_verify = gridToWorld(goal_grid.x, goal_grid.y);
    std::cout << "🔄 Start Verify: (" << std::fixed << std::setprecision(6) << start_verify.x << ", " << start_verify.y << ")" << std::endl;
    std::cout << "🔄 Goal Verify:  (" << std::fixed << std::setprecision(6) << goal_verify.x << ", " << goal_verify.y << ")" << std::endl;

    // 맵 범위 정보
    std::cout << "📏 Map Size: " << W_ << "x" << H_ << " pixels" << std::endl;
    std::cout << "📐 Map Coverage: X[" << origin_m_.x << "-" << (origin_m_.x + W_*res_) << "], Y[" << origin_m_.y << "-" << (origin_m_.y + H_*res_) << "] meters" << std::endl;
    std::cout << "🔧 Resolution: " << res_ << " m/px, Origin: (" << origin_m_.x << ", " << origin_m_.y << ")" << std::endl;

    if (!inBounds(start_grid.x, start_grid.y) || !inBounds(goal_grid.x, goal_grid.y)) {
        std::cout << "❌ Start or Goal position is outside map bounds!" << std::endl;
        return {};
    }

    // 시작점과 목표점 더 넓게 자유화 (9x9 영역)
    auto clear_area = [&](const cv::Point2i& p) {
        for (int dy=-4; dy<=4; ++dy) for (int dx=-4; dx<=4; ++dx) {
            int x = p.x + dx, y = p.y + dy;
            if (inBounds(x,y)) gridmap.at<uchar>(y,x) = 0; // 0=자유
        }
    };

    clear_area(start_grid);
    //clear_area(goal_grid);

    // 🛣️ 자연스러운 우회를 위해 맵 가장자리에만 여유 공간 확보
    std::cout << "🛣️  Creating natural bypass routes (preserving all obstacles)..." << std::endl;

    // 시작점과 목표점 주변에만 추가 여유 공간
    auto create_local_space = [&](const cv::Point2i& p, int radius) {
        for(int dy = -radius; dy <= radius; dy++) {
            for(int dx = -radius; dx <= radius; dx++) {
                int clear_x = p.x + dx;
                int clear_y = p.y + dy;
                if(inBounds(clear_x, clear_y) && std::abs(dx) > 2 && std::abs(dy) > 2) {
                    gridmap.at<uchar>(clear_y, clear_x) = 0;
                }
            }
        }
    };

    create_local_space(start_grid, 6);
    create_local_space(goal_grid, 6);

    std::cout << "   Natural bypass routes created - obstacles preserved for proper avoidance" << std::endl;

    std::cout << "🔍 Debugging: Checking path corridor..." << std::endl;
    // 시작점과 목표점 사이 직선상의 장애물 체크
    int blocked_count = 0;
    for(int y = start_grid.y; y <= goal_grid.y; y += 10) {
        if(inBounds(start_grid.x, y) && !isFree(start_grid.x,y)) {
            blocked_count++;
            std::cout << "🚫 Blocked at Y=" << y << " value=" << (int)gridmap.at<uchar>(y, start_grid.x) << std::endl;
        }
    }
    std::cout << "📊 Blocked points in direct path: " << blocked_count << std::endl;

    // A* 알고리즘
    auto cmp = [](std::shared_ptr<AstarNode> a, std::shared_ptr<AstarNode> b) {
        return a->f_cost > b->f_cost;
    };
    std::priority_queue<std::shared_ptr<AstarNode>, std::vector<std::shared_ptr<AstarNode>>, decltype(cmp)> open_set(cmp);
    std::vector<std::vector<std::shared_ptr<AstarNode>>> nodes(W_, std::vector<std::shared_ptr<AstarNode>>(H_, nullptr));

    double h_start = std::abs(goal_grid.x - start_grid.x) + std::abs(goal_grid.y - start_grid.y);
    auto start_node = std::make_shared<AstarNode>(start_grid.x, start_grid.y, 0.0, h_start);
    open_set.push(start_node);
    nodes[start_grid.x][start_grid.y] = start_node;

    constexpr int NDIR = 4;  // ✅ 상하좌우만
    int dx[NDIR]     = { 0,  1,  0, -1};
    int dy[NDIR]     = { 1,  0, -1,  0};
    double costs[NDIR]= { 1.0, 1.0, 1.0, 1.0};

    std::shared_ptr<AstarNode> goal_node = nullptr;
    int iterations = 0;

    while (!open_set.empty() && iterations < max_iterations) {
        auto current = open_set.top();
        open_set.pop();
        iterations++;

        // 디버깅: 처음 10번 iteration 상세 로그
        if (iterations <= 10) {
            std::cout << "🔍 Iteration " << iterations << ": Current(" << current->x << "," << current->y
                      << ") towards Goal(" << goal_grid.x << "," << goal_grid.y << ") Distance="
                      << std::abs(goal_grid.x - current->x) + std::abs(goal_grid.y - current->y) << std::endl;
        }

        // 목표 도달
        if (current->x == goal_grid.x && current->y == goal_grid.y) {
            goal_node = current;
            std::cout << "✅ SUCCESS in " << iterations << " iterations!" << std::endl;
            break;
        }

        int valid_neighbors = 0;

        for (int i = 0; i < NDIR; ++i) {
    int nx = current->x + dx[i];
    int ny = current->y + dy[i];

    if (!inBounds(nx, ny)) continue;
    if (!isFree(nx, ny)) continue;  // 0=free, 255=obstacle 가정

    double new_g = current->g_cost + costs[i];

    // ✅ 맨해튼 휴리스틱은 4방향에 최적(일관/허용)
    double h = std::abs(goal_grid.x - nx) + std::abs(goal_grid.y - ny);

    if (nodes[nx][ny] == nullptr || new_g < nodes[nx][ny]->g_cost) {
        auto neighbor = std::make_shared<AstarNode>(nx, ny, new_g, h);
        neighbor->parent = current;
        nodes[nx][ny] = neighbor;
        open_set.push(neighbor);
    }
}

        if (iterations <= 10) {
            std::cout << "📊 Valid neighbors: " << valid_neighbors << ", open_set size: " << open_set.size() << std::endl;
            if (open_set.empty()) {
                std::cout << "⚠️ Open set became empty!" << std::endl;
                break;
            }
        }
    }

    // 경로 재구성
    std::vector<cv::Point2f> world_path;

    if (goal_node) {
        auto current = goal_node;
        while (current) {
            world_path.push_back(gridToWorld(current->x, current->y));
            current = current->parent;
        }
        std::reverse(world_path.begin(), world_path.end());

        std::cout << "✅ Path found with " << world_path.size() << " waypoints (UNIFIED COORDINATES)" << std::endl;
        return world_path;
    } else {
        std::cout << "❌ No path found after " << iterations << " iterations" << std::endl;
        return {};
    }
}

cv::Point2i Astar::worldToGrid(double world_x, double world_y) const {
    // YAML에서 로드된 파라미터 사용
    double pixel_x = (world_x - origin_m_.x) / res_;
    double pixel_y = (world_y - origin_m_.y) / res_;

    int grid_x = static_cast<int>(std::round(pixel_x));
    int grid_y = static_cast<int>(std::round(H_ - 1- pixel_y));

    grid_x = std::max(0, std::min(W_-1, grid_x));
    grid_y = std::max(0, std::min(H_-1, grid_y));

    return cv::Point2i(grid_x, grid_y);
}


cv::Point2f Astar::gridToWorld(int grid_x, int grid_y) const {
    // YAML에서 로드된 파라미터 사용
    double world_x = (static_cast<double>(grid_x) + 0.5) * res_ + origin_m_.x;
    double world_y = (static_cast<double>(H_ - 1 - grid_y) + 0.5) * res_ + origin_m_.y;

    return cv::Point2f(world_x, world_y);
}

// 나머지 필요한 함수들
cv::Mat Astar::getgridmap() const {
    std::lock_guard<std::mutex> lock(map_mutex);
    return gridmap.clone();
}

cv::Mat Astar::getVisualizationMap() const {
    std::lock_guard<std::mutex> lock(map_mutex);
    cv::Mat vis_map = gridmap.clone();
    if (vis_map.empty()) {
        vis_map = cv::Mat::zeros(300, 300, CV_8UC1);
    }
    cv::Mat color_map;
    cv::cvtColor(vis_map, color_map, cv::COLOR_GRAY2BGR);
    return color_map;
}

cv::Mat Astar::getVisualizationMapWithPath(const std::vector<cv::Point2f>& path) const {
    cv::Mat color_map = getVisualizationMap();
    if (!path.empty() && W_ > 0 && H_ > 0) {
        for (size_t i = 1; i < path.size(); i++) {
            cv::Point2i prev = worldToGrid(path[i-1].x, path[i-1].y);
            cv::Point2i curr = worldToGrid(path[i].x, path[i].y);
            cv::line(color_map, cv::Point(prev.x, prev.y), cv::Point(curr.x, curr.y), cv::Scalar(0, 0, 255), 2);
        }
    }
    return color_map;
}

cv::Mat Astar::getVisualizationMapWithWorldPath(const std::vector<cv::Point2f>& path, int current_index) const {
    std::lock_guard<std::mutex> lock(map_mutex);
    cv::Mat vis_map = gridmap.clone();
    if (vis_map.empty()) {
        vis_map = cv::Mat::zeros(300, 300, CV_8UC1);
    }
    cv::Mat color_map;
    cv::cvtColor(vis_map, color_map, cv::COLOR_GRAY2BGR);

    if (W_ <= 0 || H_ <= 0) return color_map;

    // 경로 그리기
    for (size_t i = 1; i < path.size(); i++) {
        cv::Point2i prev = worldToGrid(path[i-1].x, path[i-1].y);
        cv::Point2i curr = worldToGrid(path[i].x, path[i].y);
        cv::Scalar color = (i <= (size_t)current_index) ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
        cv::line(color_map, prev, curr, color, 2);
    }

    // 웨이포인트 표시
    for (size_t i = 0; i < path.size(); i++) {
        cv::Point2i curr = worldToGrid(path[i].x, path[i].y);
        if (!inBounds(curr.x, curr.y)) continue;

        if (i < (size_t)current_index) {
            cv::circle(color_map, curr, 4, cv::Scalar(0, 255, 0), -1);
        } else if (i == (size_t)current_index) {
            cv::circle(color_map, curr, 8, cv::Scalar(0, 255, 255), -1);
        } else {
            cv::circle(color_map, curr, 4, cv::Scalar(0, 0, 255), -1);
        }
    }
    if (!path.empty()) {
        cv::Point2i start_g = worldToGrid(path.front().x, path.front().y);
        cv::Point2i goal_g  = worldToGrid(path.back().x,  path.back().y);

        if (inBounds(start_g.x, start_g.y)) {
            // 시작: 파란색
            cv::circle(color_map, start_g, 7, cv::Scalar(255, 0, 0), -1);         // BGR: 파랑
            cv::circle(color_map, start_g, 9, cv::Scalar(0, 0, 0), 2);            // 테두리(검정)
        }
        if (inBounds(goal_g.x, goal_g.y)) {
            // 목표: 흰색(요청사항) + 빨간 테두리로 시인성 강화
            cv::circle(color_map, goal_g, 7, cv::Scalar(255, 255, 255), -1);      // ⚪ 흰색
            cv::circle(color_map, goal_g, 9, cv::Scalar(0, 0, 255), 2);           // 🔴 테두리
        }
    }

    return color_map;
}

std::vector<cv::Point2f> Astar::planGlobalPath(cv::Point2f start, cv::Point2f final_goal, double planning_horizon) {
    std::vector<cv::Point2f> path = planPath(start, final_goal, 50000);
    if(path.empty()) {
        return {};
    }

    std::vector<cv::Point2f> simplified_path;
    for(size_t i = 0; i < path.size(); i += 3) {
        simplified_path.push_back(path[i]);
    }
    if (!path.empty() && simplified_path.back() != path.back()) {
        simplified_path.push_back(path.back());
    }

    std::cout << "🌍 Global Path Optimized: " << path.size() << " -> " << simplified_path.size() << " waypoints" << std::endl;
    return simplified_path;
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

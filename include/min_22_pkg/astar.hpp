#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <cmath>
#include <functional>
#include <mutex>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>

class Astar {
public:
    struct AstarNode {
    int x, y;                              // 그리드 좌표
    double g_cost;                         // 시작점부터의 실제 비용
    double h_cost;                         // 목표점까지의 추정 비용 (휴리스틱)
    double f_cost;                         // g_cost + h_cost (총 비용)
    double detour_score;                   // 🆕 우회 점수 (직선에서 벗어날수록 높음)
    std::shared_ptr<AstarNode> parent;     // 부모 노드 (경로 추적용)

    AstarNode(int x_, int y_, double g_, double h_)
        : x(x_), y(y_), g_cost(g_), h_cost(h_), f_cost(g_ + h_), detour_score(0.0), parent(nullptr) {}
};

    Astar();
    ~Astar() = default;

    // PGM 맵 로드 (글로벌 플래닝용)
    bool loadMapFromFile(const std::string& path, double resolution_m_per_px, const cv::Point2d& origin_m = {0,0}, int inflate_px = 1); // 장애물 팽창 픽셀 (1로 완화)

    std::vector<cv::Point2f> planPath(cv::Point2f start, cv::Point2f goal, int max_iterations = 2000);

    cv::Mat getVisualizationMap() const;
    cv::Mat getVisualizationMapWithPath(const std::vector<cv::Point2f>& path = {}) const;
    cv::Mat getVisualizationMapWithWorldPath(const std::vector<cv::Point2f>& path, int current_index = 0) const;
    cv::Mat getgridmap() const;
    void setVisualizationCallback(std::function<void()> callback) {
        visualization_callback = callback;
    }

    void setLanePoints(const std::vector<cv::Point2f>& left_points,
                       const std::vector<cv::Point2f>& right_points);
    void clearLanePoints();
    std::vector<cv::Point2f> planGlobalPath(cv::Point2f start, cv::Point2f final_goal, double planning_horizon = 2.0);
    cv::Point2i worldToGrid(double world_x, double world_y) const;
    cv::Point2f gridToWorld(int grid_x, int grid_y) const;

    inline bool inBounds(int x, int y) const {
        return (0 <= x && x < W_ && 0 <= y && y < H_);
    }

    inline bool isFree(int x, int y) const {
        // gridmap: 0 = 자유공간, 255 = 장애물
        return inBounds(x, y) && (gridmap.at<uchar>(y, x) == 0);
    }

    inline bool isBlocked(int x, int y) const {
        return !isFree(x, y);
    }


    // 맵 변수들을 public으로 이동 (fallback 맵 설정용)
    cv::Mat gridmap; // 현재 A*가 사용하는 맵
    int W_=0, H_=0;         // 맵 너비/높이
    double res_=0.05;       // m/px (PGM 해상도)
    cv::Point2d origin_m_{0,0}; // 맵좌표의 (0,0)에 해당하는 월드(m)

    // 픽셀-미터 변환 상수 (사용 안 함: PGM 해상도 사용)
    static constexpr double pixel_to_meter = -0.0018;

private:
    mutable std::mutex map_mutex;
    std::function<void()> visualization_callback;

    cv::Mat map_gray_;      // 원본 PGM 그레이(0~255)
    cv::Mat occ_;           // 장애물:255, 자유:0 (binary)

    inline int idx(int x,int y) const { return y*W_ + x; }

    // A* 내부 노드
    struct Node { int x,y; double g,f; int parent_idx; };

    // 차선 정보 (현재 사용 안 함)
    std::vector<cv::Point2f> left_lane_points;
    std::vector<cv::Point2f> right_lane_points;
    bool lane_data_available{false};

    void notifyVisualizationUpdate() {
        if (visualization_callback) {
            visualization_callback();
        }
    }


    size_t getNodeKey(int x, int y) const { return y * W_ + x; } // 맵 크기 W_ 사용

    cv::Mat dist_map_m_;          // 자유공간 각 셀의 최근접 장애물까지 거리 [m]
    double r_robot_m_ = 0.16;     // 로봇 외접원 반경 [m]
    double margin_forbid_ = 0.02; // 금지대역 마진 [m]
    double margin_prefer_ = 0.10; // 선호 여유 [m]
    double w_clear_ = 50.0;       // 여유 부족 패널티 가중치
/*
    static constexpr int dx_[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    static constexpr int dy_[8] = {1, 0, -1, 0, 1, -1, 1, -1};
    static constexpr double move_costs_[8] = {
        1.0, 1.0, 1.0, 1.0,                    // 상하좌우: 1.0
        1.4142135623730951, 1.4142135623730951, // 대각선: √2
        1.4142135623730951, 1.4142135623730951
    };*/
};

#endif

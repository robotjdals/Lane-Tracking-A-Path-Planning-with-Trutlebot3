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
#include <sensor_msgs/msg/laser_scan.hpp>

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

    std::vector<int> planPath(cv::Point2f start, cv::Point2f goal, int max_iterations = 2000);
    void updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                   double robot_x = 0.0, double robot_y = 0.0, double robot_theta = 0.0);

    cv::Mat getVisualizationMap() const;
    cv::Mat getVisualizationMapWithPath(const std::vector<cv::Point2f>& path = {}) const;
    cv::Mat getVisualizationMapWithPixelPath(const std::vector<int>& pixel_waypoints, int current_index = 0) const;
    cv::Mat getgridmap() const;
    void setVisualizationCallback(std::function<void()> callback) {
        visualization_callback = callback;
    }


    void setLanePoints(const std::vector<cv::Point2f>& left_points,
                       const std::vector<cv::Point2f>& right_points);
    void clearLanePoints();
    std::vector<cv::Point2f> planGlobalPath(cv::Point2f start, cv::Point2f final_goal,
                                           double planning_horizon = 2.0);
    std::vector<int> planLocalPath(cv::Point2f start, cv::Point2f local_goal,
                                  double planning_horizon = 0.8);
    cv::Point2f getLocalGoalFromGlobal(const std::vector<cv::Point2f>& global_path,
                                      cv::Point2f current_pos, double lookahead = 0.5);
    cv::Point2i worldToGrid(double world_x, double world_y) const;

    // 픽셀-미터 변환 상수
    static constexpr double pixel_to_meter = -0.0018;

private:
    mutable std::mutex map_mutex;
    std::function<void()> visualization_callback;
    cv::Mat gridmap;

    // 차선 정보
    std::vector<cv::Point2f> left_lane_points;
    std::vector<cv::Point2f> right_lane_points;
    bool lane_data_available{false};

    bool checkDirectPathBlocked(cv::Point2i start, cv::Point2i goal) const;
    double enhancedHeuristic(int x1, int y1, int x2, int y2, bool prefer_detour) const;

    std::vector<std::shared_ptr<AstarNode>> getEnhancedNeighbors(
        std::shared_ptr<AstarNode> current, cv::Point2i goal, bool prefer_detour) const;

    double getEnhancedMoveCost(std::shared_ptr<AstarNode> from,
                              std::shared_ptr<AstarNode> to,
                              cv::Point2i goal, bool prefer_detour) const;

    double calculateDetourScore(std::shared_ptr<AstarNode> current,
                               std::shared_ptr<AstarNode> neighbor,
                               cv::Point2i start, cv::Point2i goal) const;

    void analyzePathQuality(const std::vector<int>& pixel_path) const;

    double euclideanHeuristic(int x1, int y1, int x2, int y2) const;
    double manhattanHeuristic(int x1, int y1, int x2, int y2) const;
    double diagonalHeuristic(int x1, int y1, int x2, int y2) const;
    double getMoveCost(int from_x, int from_y, int to_x, int to_y) const;
    std::vector<std::shared_ptr<AstarNode>> getNeighbors(
        std::shared_ptr<AstarNode> current, cv::Point2i goal) const;
    bool isCollisionFree(int grid_x, int grid_y) const;
    std::vector<cv::Point2f> reconstructPath(std::shared_ptr<AstarNode> goal_node) const;
    void notifyVisualizationUpdate() {
        if (visualization_callback) {
            visualization_callback();
        }
    }

    cv::Point2f gridToWorld(int grid_x, int grid_y) const;
    std::vector<int> worldToPixel(const std::vector<cv::Point2f>& world_path) const;
    size_t getNodeKey(int x, int y) const { return y * width + x; }

    //static constexpr double resolution = 0.003;  // 3mm per pixel
    static constexpr double resolution = 0.003;  // 3mm per pixel
    static constexpr int width = 300;            // 맵 너비
    static constexpr int height = 300;           // 맵 높이

    static constexpr int dx_[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    static constexpr int dy_[8] = {1, 0, -1, 0, 1, -1, 1, -1};
    static constexpr double move_costs_[8] = {
        1.0, 1.0, 1.0, 1.0,                    // 상하좌우: 1.0
        1.4142135623730951, 1.4142135623730951, // 대각선: √2
        1.4142135623730951, 1.4142135623730951
    };
};

#endif

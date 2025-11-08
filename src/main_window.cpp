#include "../include/min_22_pkg/main_window.hpp"
#include "../include/min_22_pkg/driving.hpp"
#include <iostream>
#include <filesystem>
#include <vector>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  driving = new Driving(this);
  a_planner = new Astar();

  driving->setQNode(qnode);
  driving->setMainWindow(this);
  driving->setPlanner(a_planner);

  prev_left_x = 0;
  prev_right_x = 0;
  first_frame = true;

  std::cout << "\n========== MAP LOADING WITH YAML ==========" << std::endl;
  std::cout << "🗺️  Loading map2.pgm with YAML configuration" << std::endl;
  std::cout << "🌍 Gazebo Workspace: X[1.4, 1.8] Y[0.4, 1.6] meters" << std::endl;
  std::cout << "📍 Key Positions (Gazebo):" << std::endl;
  std::cout << "   Start (Blue):  (1.752260, 0.454965)" << std::endl;
  std::cout << "   Goal (Red):    (1.752248, 1.519646)" << std::endl;
  std::cout << "   Obstacle 1:    (1.490000, 0.540000)" << std::endl;
  std::cout << "   Obstacle 2:    (1.740000, 1.000000)" << std::endl;
  std::cout << "   Obstacle 3:    (1.490000, 1.464000)" << std::endl;

  std::string yaml_path = "/home/min/colcon_ws/src/min_22_pkg/map2.yaml";

  std::cout << "Loading YAML from: " << yaml_path << std::endl;

  bool map_loaded = false;
  if (a_planner->loadMapFromFile(yaml_path, 0.0, {0.0, 0.0}, 20)) {  // YAML에서 파라미터 자동 로드
      std::cout << "✅ YAML Map loaded successfully!" << std::endl;
      map_loaded = true;
  }

  if (!map_loaded) {
      std::cout << "❌ YAML Map loading FAILED from all attempted paths!" << std::endl;

      // YAML 기본값으로 fallback 맵 생성
      std::cout << "\n🔧 Creating fallback map with default YAML parameters..." << std::endl;

      // 기본 YAML 파라미터 (map2.yaml 기준)
      const double default_resolution = 0.00506991;
      const cv::Point2d default_origin(1.01712, 0.0240227);
      const int default_width = 170;
      const int default_height = 349;

      cv::Mat test_map = cv::Mat::zeros(default_height, default_width, CV_8UC1);

      // 테두리에 장애물 추가 (맵 경계)
      cv::rectangle(test_map, cv::Rect(0, 0, default_width, 5), cv::Scalar(255), -1);       // 상단
      cv::rectangle(test_map, cv::Rect(0, default_height-5, default_width, 5), cv::Scalar(255), -1);     // 하단
      cv::rectangle(test_map, cv::Rect(0, 0, 5, default_height), cv::Scalar(255), -1);       // 좌측
      cv::rectangle(test_map, cv::Rect(default_width-5, 0, 5, default_height), cv::Scalar(255), -1);     // 우측

      std::cout << "📍 Adding obstacles using default coordinate system:" << std::endl;

      // 기본 파라미터 기반 좌표 변환 함수
      auto worldToGrid = [&](double world_x, double world_y) -> cv::Point2i {
          double pixel_x = (world_x - default_origin.x) / default_resolution;
          double pixel_y = (world_y - default_origin.y) / default_resolution;
          int grid_x = static_cast<int>(std::round(pixel_x));
          int grid_y = static_cast<int>(std::round(pixel_y));
          grid_x = std::max(0, std::min(default_width-1, grid_x));
          grid_y = std::max(0, std::min(default_height-1, grid_y));
          return cv::Point2i(grid_x, grid_y);
      };

      // 가제보상 장애물 포즈를 그리드로 변환하여 배치
      // 첫 번째 벽: (1.490, 0.540)
      cv::Point2i obs1 = worldToGrid(1.490000, 0.540000);
      cv::rectangle(test_map, cv::Rect(obs1.x-10, obs1.y-10, 20, 20), cv::Scalar(255), -1);
      std::cout << "   Obstacle 1: World(1.490, 0.540) → Grid(" << obs1.x << ", " << obs1.y << ")" << std::endl;

      // 두 번째 벽: (1.740, 1.000)
      cv::Point2i obs2 = worldToGrid(1.740000, 1.000000);
      cv::rectangle(test_map, cv::Rect(obs2.x-15, obs2.y-15, 30, 30), cv::Scalar(255), -1);
      std::cout << "   Obstacle 2: World(1.740, 1.000) → Grid(" << obs2.x << ", " << obs2.y << ")" << std::endl;

      // 세 번째 벽: (1.490, 1.464)
      cv::Point2i obs3 = worldToGrid(1.490000, 1.464000);
      cv::rectangle(test_map, cv::Rect(obs3.x-10, obs3.y-10, 20, 20), cv::Scalar(255), -1);
      std::cout << "   Obstacle 3: World(1.490, 1.464) → Grid(" << obs3.x << ", " << obs3.y << ")" << std::endl;

      // A* 플래너에 기본 파라미터로 설정
      a_planner->gridmap = test_map.clone();
      a_planner->W_ = default_width;
      a_planner->H_ = default_height;
      a_planner->res_ = default_resolution;
      a_planner->origin_m_ = default_origin;

      std::cout << "✅ Fallback map created with default coordinate system!" << std::endl;
      std::cout << "   Resolution: " << a_planner->res_ << " m/pixel" << std::endl;
      std::cout << "   Origin: (" << a_planner->origin_m_.x << ", " << a_planner->origin_m_.y << ")" << std::endl;
      std::cout << "   Map coverage: X[" << default_origin.x << " to " << (default_origin.x + default_width * default_resolution) << "], ";
      std::cout << "Y[" << default_origin.y << " to " << (default_origin.y + default_height * default_resolution) << "]" << std::endl;
  } else {
      // 실제 YAML 맵이 로딩된 경우 파라미터 확인
      std::cout << "✅ YAML Map loaded successfully!" << std::endl;
      std::cout << "📏 Loaded map size: " << a_planner->W_ << "x" << a_planner->H_
                << " pixels (" << (a_planner->W_ * a_planner->res_) << "x" << (a_planner->H_ * a_planner->res_) << " meters)" << std::endl;
      std::cout << "📍 Map coverage: X[" << a_planner->origin_m_.x
                << " to " << (a_planner->origin_m_.x + a_planner->W_ * a_planner->res_)
                << "], Y[" << a_planner->origin_m_.y
                << " to " << (a_planner->origin_m_.y + a_planner->H_ * a_planner->res_) << "]" << std::endl;
      std::cout << "📐 Resolution: " << a_planner->res_ << " m/pixel" << std::endl;
      std::cout << "📍 Origin: (" << a_planner->origin_m_.x << ", " << a_planner->origin_m_.y << ")" << std::endl;
  }

  // 맵 시각화 UI 초기에는 숨김
  ui->display_5->setVisible(false);

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(sigRcvImg()), this, SLOT(slotUpdateImg()));
  QObject::connect(this, &MainWindow::waypointsReady, this, &MainWindow::processWaypoints);

}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() {
  if (qnode) {
    qnode->running_ = false;  // 스레드 종료 신호
  }

  delete ui;
}


void MainWindow::slotUpdateImg() {  //UI에 캠화면 출력
  std::lock_guard<std::mutex> lock(qnode->img_mutex);
  clone_mat = qnode->imgRaw->clone();                                           // 원본 이미지 복사
  cv::resize(clone_mat, clone_mat, cv::Size(640, 360), 0, 0, cv::INTER_CUBIC);  // 이미지 크기 조정

  cv::Mat display_img = clone_mat.clone();
  cv::Mat combine_img = clone_mat.clone();

  int left_start, right_start;

  //Find_Binary_img(display1_img);
  //yellow_imgs(display1_img);
  Gaussain_Filter(display_img);
  combine_img = sumImg(white_hsv(display_img), yellow_hsv(display_img));
  perspective_transform(combine_img, Perspective_img);

  QImage RGB_im2((const unsigned char*)(Perspective_img.data), Perspective_img.cols, Perspective_img.rows, QImage::Format_Grayscale8);
  ui->display_3->setPixmap(QPixmap::fromImage(RGB_im2));

  cv::Mat window_img = Perspective_img.clone();

  if(first_frame){
    left_start = left_line(Perspective_img);
    right_start = right_line(Perspective_img);
    first_frame = false;
  }
  else{
    left_start = prev_left_x;
    right_start = prev_right_x;
  }

  prev_left_x = left_start;
  prev_right_x = right_start;


  std::vector<int> waypoints = getWindowSearch(Perspective_img, left_start, right_start);

  current_waypoints = waypoints;
  emit waypointsReady();

  QImage RGB_im1((const unsigned char*)(display_img.data), display_img.cols, display_img.rows, QImage::Format_RGB888);
  ui->label->setPixmap(QPixmap::fromImage(RGB_im1));


  // --- 맵 시각화 및 UI 제어 로직 (상태 기반) ---
  if (a_planner && driving) {
    // PLANNING, PATH_TRACK, AVOIDANCE 상태일 때 맵 표시
    bool show_map = (driving->state == Driving::PLANNING ||
                     driving->state == Driving::PATH_TRACK ||
                     driving->state == Driving::AVOIDANCE);

    if (show_map) {
        // UI 표시
        ui->display_5->setVisible(true);

        cv::Mat grid_map;

        if (driving->state == Driving::PLANNING) {
            // 경로 계획 중일 때
            grid_map = a_planner->getVisualizationMap();
            std::string state_text = "State: PLANNING A* PATH...";
            cv::putText(grid_map, state_text, cv::Point(10, grid_map.rows - 50),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        }
        else if (driving->global_path_ready && !driving->path_m_.empty()) {
            // A* 경로와 함께 시각화
            grid_map = a_planner->getVisualizationMapWithWorldPath(driving->path_m_, driving->wp_idx_);

            // 상태 정보 표시
            std::string state_text = "State: PATH_TRACKING (Progress: " +
                                   std::to_string(driving->wp_idx_) + "/" + std::to_string(driving->path_m_.size()) + ")";
            cv::putText(grid_map, state_text, cv::Point(10, grid_map.rows - 50),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        } else {
            // 기본 맵 표시
            grid_map = a_planner->getVisualizationMap();
            std::string state_text = "State: WAITING...";
            cv::putText(grid_map, state_text, cv::Point(10, grid_map.rows - 50),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        }

        // 현재 로봇 위치를 맵에 표시 - 맵 크기에 비례하게 조정
        if (qnode && qnode->odom_received) {
            cv::Point2i robot_grid = a_planner->worldToGrid(qnode->odom_x, qnode->odom_y);

            // 맵 크기에 따른 마커 크기 자동 조정
            int marker_size = std::max(2, std::min(grid_map.cols, grid_map.rows) / 20);
            int cross_size = marker_size + 5;

            // 맵 경계 내에 있는지 확인
            if (a_planner->inBounds(robot_grid.x, robot_grid.y)) {
                // 스케일링을 고려한 큰 마커 (원본 맵에 크게 그려서 스케일링 후에도 보이도록)
                int marker_size = std::max(8, std::min(grid_map.cols, grid_map.rows) / 15); // 더 큰 마커
                int cross_size = marker_size + 8;

                // 큰 로봇 마커 (스케일링 후에도 명확히 보이도록)
                cv::circle(grid_map, robot_grid, marker_size, cv::Scalar(255, 100, 0), -1);     // 주황색 내부
                cv::circle(grid_map, robot_grid, marker_size + 3, cv::Scalar(0, 255, 255), 3);  // 노란색 테두리
                cv::circle(grid_map, robot_grid, marker_size + 6, cv::Scalar(255, 255, 255), 2); // 흰색 외곽선

                // 굵은 십자가 마커
                cv::line(grid_map, cv::Point(robot_grid.x-cross_size, robot_grid.y), cv::Point(robot_grid.x+cross_size, robot_grid.y),
                         cv::Scalar(0, 0, 255), 4); // 굵은 빨간 수평선
                cv::line(grid_map, cv::Point(robot_grid.x, robot_grid.y-cross_size), cv::Point(robot_grid.x, robot_grid.y+cross_size),
                         cv::Scalar(0, 0, 255), 4); // 굵은 빨간 수직선

                std::cout << "✅ Large robot marker displayed at grid (" << robot_grid.x << ", " << robot_grid.y
                          << ") with size " << marker_size << std::endl;
            } else {
                std::cout << "❌ Robot is outside map bounds!" << std::endl;
            }

            // 텍스트 크기도 맵 크기에 맞게 조정
            double text_scale = std::max(0.3, std::min(0.8, grid_map.cols / 400.0));
            std::string odom_info = "Robot: X=" + std::to_string(qnode->odom_x).substr(0,5) +
                                   " Y=" + std::to_string(qnode->odom_y).substr(0,5);
            cv::putText(grid_map, odom_info, cv::Point(5, grid_map.rows - 10),
                       cv::FONT_HERSHEY_SIMPLEX, text_scale, cv::Scalar(255, 255, 255), 1);
        }

        if (!grid_map.empty()) {
            QImage qimg(grid_map.data, grid_map.cols, grid_map.rows, grid_map.step, QImage::Format_BGR888);
            QPixmap pixmap = QPixmap::fromImage(qimg);

            // UI 크기에 맞게 스케일링 (비율 유지)
            QPixmap scaled_pixmap = pixmap.scaled(ui->display_5->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

            std::cout << "🖼️ Map scaling: Original(" << grid_map.cols << "x" << grid_map.rows
                      << ") → UI(" << scaled_pixmap.width() << "x" << scaled_pixmap.height() << ")" << std::endl;

            ui->display_5->setPixmap(scaled_pixmap);
        }
    } else {
        // PLANNING/PATH_TRACK/AVOIDANCE 상태가 아니면 UI 숨김
        ui->display_5->setVisible(false);
    }
  }
  // --- 맵 시각화 및 UI 제어 로직 끝 ---


  QImage RGB_im3((const unsigned char*)(Perspective_img.data), Perspective_img.cols, Perspective_img.rows, QImage::Format_Grayscale8);
  ui->display_4->setPixmap(QPixmap::fromImage(RGB_im3));
  if (qnode->imgRaw) {
    delete qnode->imgRaw;// 동적 할당된 원본 이미지 메모리 해제
    qnode->imgRaw = nullptr;
  }
  qnode->isreceived = false;  // 이미지 수신 플래그 재설정
}

void MainWindow::processWaypoints() {
  if (driving && !current_waypoints.empty()) {
      driving->go(current_waypoints);  // 직접 호출
  }
}

void MainWindow::perspective_transform(const cv::Mat& input_img, cv::Mat& output_img){
  std::array<cv::Point2f, 4> input_birdseyeview = {cv::Point2f(640 - 520, 360 - 80), cv::Point2f(640 - 120, 360 - 80), cv::Point2f(640 - 10, 360 - 20), cv::Point2f(640 - 630, 360 - 20)};
  //std::array<cv::Point2f, 4> input_birdseyeview = {cv::Point2f(640 - 540, 360 - 80), cv::Point2f(640 - 100, 360 - 80), cv::Point2f(640 - 10, 360 - 30), cv::Point2f(640 - 630, 360 - 30)};
  //std::array<cv::Point2f, 4> input_birdseyeview = {cv::Point2f(640 - 540 , 360 - 80), cv::Point2f(0, 360 - 80), cv::Point2f(640 - 10, 360 - 40), cv::Point2f(640 - 630, 360 - 40)};
  std::array<cv::Point2f, 4> output_birdseyeview = {cv::Point2f(0,0), cv::Point2f(Raw_X,0), cv::Point2f(Raw_X, Raw_Y), cv::Point2f(0,Raw_Y)};

  cv::Mat birdseyeview = cv::getPerspectiveTransform(input_birdseyeview.data(), output_birdseyeview.data());
//*invMatx = getPerspectiveTransform(output_birdseyeview.data(), input_birdseyeview.data());

  cv::warpPerspective(input_img, output_img, birdseyeview, cv::Size(Raw_X, Raw_Y));

  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[0], input_birdseyeview[1], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[1], input_birdseyeview[2], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[2], input_birdseyeview[3], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[3], input_birdseyeview[0], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
}


cv::Mat MainWindow::white_hsv(cv::Mat& img) {
  // 복사 이미지 HSV로 변환
  cv::Mat hsvImg;
  cv::cvtColor(img, hsvImg, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
  cv::erode(hsvImg, hsvImg, mask, cv::Point(-1, -1), 1);

  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Scalar lower(0, 0, 180);
  cv::Scalar upper(180, 30, 255);
  cv::Mat binaryImage;
  cv::inRange(hsvImg, lower, upper, binaryImage);

  //QImage RGB_im3((const unsigned char*)(binaryImage.data), binaryImage.cols, binaryImage.rows, QImage::Format_Grayscale8);
  //ui->display_4->setPixmap(QPixmap::fromImage(RGB_im3));

  return binaryImage;
}

// 이진화
cv::Mat MainWindow::yellow_hsv(cv::Mat& img) {
  // 복사 이미지 HSV로 변환
  cv::Mat hsvImg;
  cv::cvtColor(img, hsvImg, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
  cv::erode(hsvImg, hsvImg, mask, cv::Point(-1, -1), 1);

  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Scalar lower(0, 90, 90);
  cv::Scalar upper(100, 255, 255);
  cv::Mat binaryImage;
  cv::inRange(hsvImg, lower, upper, binaryImage);

  //QImage RGB_im4((const unsigned char*)(binaryImage.data), binaryImage.cols, binaryImage.rows, QImage::Format_Grayscale8);
  //ui->display_5->setPixmap(QPixmap::fromImage(RGB_im4));

  return binaryImage;
}

void MainWindow::Gaussain_Filter(cv::Mat& img){

  cv::Mat blurredImage;
  cv::GaussianBlur(img, blurredImage, cv::Size(7,7), 0); // 가우시안 필터

  QImage binaryQImage(blurredImage.data, blurredImage.cols, blurredImage.rows, blurredImage.step, QImage::Format_RGB888); // 시각화
  ui->test->setPixmap(QPixmap::fromImage(binaryQImage));

}

int MainWindow::left_line(cv::Mat original)
{
  if (original.empty() || original.rows < 4 || original.cols < 4) {
    return 0;
  }

  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::Mat left = original(cv::Range(original.rows* 3 / 4, original.rows), cv::Range(0, original.cols/2));
  cv::Mat left_float;
  left.convertTo(left_float, CV_32F);

  cv::Mat outimg;
  cv::reduce(left_float, outimg, 0, cv::REDUCE_SUM, CV_32F);
  cv::minMaxLoc(outimg, &minVal, &maxVal, &minLoc, &maxLoc);

  return maxLoc.x;
}

int MainWindow::right_line(cv::Mat original)
{
  if (original.empty() || original.rows < 4 || original.cols < 4) {
    return original.cols - 1;
  }

  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::Mat right = original(cv::Range(original.rows * 3 / 4, original.rows), cv::Range(original.cols/2, original.cols));
  cv::Mat right_float;
  right.convertTo(right_float, CV_32F);

  cv::Mat outimg;
  cv::reduce(right_float, outimg, 0, cv::REDUCE_SUM, CV_32F);
  cv::minMaxLoc(outimg, &minVal, &maxVal, &minLoc, &maxLoc);

  return maxLoc.x + original.cols/2 + 20;
}

std::vector<int> MainWindow::getWindowSearch(cv::Mat& searchimg, int& left_x, int& right_x){
  //cv::Rect rect(x, y, width, height);

  int numwindow = 10;
  int window_width = 120;
  int window_height = searchimg.rows / numwindow;
  int margin = 60;


  std::vector<int> left_centers, right_centers, waypoint;

  int current_left_x = left_x;
  int current_right_x = right_x;

  std::vector<cv::Point2f> left_world_points;
  std::vector<cv::Point2f> right_world_points;

  left_world_points.clear();   // 기존 데이터 클리어
  right_world_points.clear();  // 기존 데이터 클리어

  bool left_detected = false;
  bool right_detected = false;
  int left_detection_count = 0;
  int right_detection_count = 0;

  for(int i = 0; i < numwindow; i++){
    int window_y = searchimg.rows - (i + 1) * window_height;
    if(window_y < 0) continue;


    int left_window_x = std::max(0, current_left_x - margin);
    int left_window_width = std::min(window_width, searchimg.cols - left_window_x);
    int left_window_height = std::min(window_height, searchimg.rows - window_y);

    if(left_window_width > 0 && left_window_height > 0){
      cv::Rect leftwindow_rect(left_window_x, window_y, left_window_width, left_window_height);
      cv::rectangle(searchimg, leftwindow_rect, cv::Scalar(255),2);

      cv::Mat Window = searchimg(leftwindow_rect);
      cv::Mat left_lane;
      cv::findNonZero (Window, left_lane);

      if(left_lane.rows > 3){
        int left_sum = 0;
        for(int j =0; j < left_lane.rows; j++){
          cv::Point pt = left_lane.at<cv::Point>(j);
          left_sum += pt.x;
        }
        int new_center = left_window_x + left_sum / left_lane.rows;
        current_left_x = new_center;
        left_centers.push_back(new_center);

        left_detected = true;
        //left_detection_count++;

        double world_x = (new_center - 320) * Driving::pixel_to_meter ;
        // Y축은 전방 거리 (음수)로 변환
        double world_y = -(((numwindow - 1 - i) * window_height) / (double)searchimg.rows * 1.5); // 대략적인 전방 거리 추정 (m)
        left_world_points.push_back(cv::Point2f(world_x, world_y));

        cv::circle(searchimg, cv::Point(new_center, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
      else {
        left_centers.push_back(current_left_x);
        cv::circle(searchimg, cv::Point(current_left_x, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
    }
    else{
      left_centers.push_back(current_left_x);
    }

    int right_window_x = std::max(0, std::min(current_right_x - margin, searchimg.cols - window_width));
    int right_window_width = std::min(window_width, searchimg.cols - right_window_x);
    int right_window_height = std::min(window_height, searchimg.rows - window_y);

    if(right_window_width > 0 && right_window_height >0) {

      cv::Rect rightwindow_rect(right_window_x, window_y, right_window_width, right_window_height);
      cv::rectangle(searchimg, rightwindow_rect, cv::Scalar(255),2);

      cv::Mat Window = searchimg(rightwindow_rect);
      cv::Mat right_lane;
      cv::findNonZero (Window, right_lane);

      if(right_lane.rows > 3) {
        int right_sum = 0;
        for(int j =0; j < right_lane.rows; j++){
          cv::Point pt = right_lane.at<cv::Point>(j);
          right_sum += pt.x;
        }
        int new_center = right_window_x + right_sum / right_lane.rows;
        current_right_x = new_center;
        right_centers.push_back(new_center);

        right_detected = true;
        //right_detection_count++;

        double world_x = (new_center - 320) * Driving::pixel_to_meter;
        // Y축은 전방 거리 (음수)로 변환
        double world_y = -(((numwindow - 1 - i) * window_height) / (double)searchimg.rows * 1.5); // 대략적인 전방 거리 추정 (m)
        right_world_points.push_back(cv::Point2f(world_x, world_y));


        cv::circle(searchimg, cv::Point(new_center, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
      else {
        right_centers.push_back(current_right_x);
        cv::circle(searchimg, cv::Point(current_right_x, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
    }
    else{
      right_centers.push_back(current_right_x);
    }
  }

  if(!left_centers.empty()) {

    int bottom_windows = std::min(3, (int)left_centers.size());
    int sum = 0;
    for(int i = 0; i < bottom_windows; i++) {
        sum += left_centers[i];
    }
    left_x = sum / bottom_windows;

  }

  if(!right_centers.empty()) {

    int bottom_windows = std::min(3, (int)right_centers.size());
    int sum = 0;
    for(int i = 0; i < bottom_windows; i++) {
        sum += right_centers[i];
    }
    right_x = sum / bottom_windows;

  }

  for(int i = 0; i < numwindow; i++){
    int window_y = searchimg.rows - (i + 1) * window_height;

    if(i < left_centers.size() && i < right_centers.size()) {
      int mean = (left_centers[i] + right_centers[i]) / 2;
      waypoint.push_back(mean);

      cv::circle(searchimg, cv::Point(mean, window_y + window_height/2), 4, cv::Scalar(200), -1);
    }
  }

  this -> left_detected = (left_world_points.size() >= numwindow / 2);
  this -> right_detected = (right_world_points.size() >= numwindow / 2);

  if(a_planner) {
    //std::cout << "Left detected: " << this->left_detected
    //          << ", Right detected: " << this->right_detected << std::endl;
    //std::cout << "Left points: " << left_world_points.size()
    //          << ", Right points: " << right_world_points.size() << std::endl;

    if(this->left_detected && this->right_detected &&
       !left_world_points.empty() && !right_world_points.empty()) {
        a_planner->setLanePoints(left_world_points, right_world_points);
        //std::cout << "✅ Lane data sent to A*" << std::endl;
    } else {
        a_planner->clearLanePoints();
        //std::cout << "❌ Lane data cleared - Left:" << this->left_detected
        //          << " Right:" << this->right_detected << std::endl;
    }
}


  return waypoint;

}

cv::Mat MainWindow::sumImg(cv::Mat img1, cv::Mat img2)
{
  cv::Mat imgout;
  cv::bitwise_or(img1, img2, imgout);

  return imgout;
}

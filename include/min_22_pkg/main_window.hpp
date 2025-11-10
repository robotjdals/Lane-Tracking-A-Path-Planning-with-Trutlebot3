#ifndef min_22_pkg_MAIN_WINDOW_H
#define min_22_pkg_MAIN_WINDOW_H

#include <QMainWindow>

#include "QIcon"
#include "driving.hpp"
#include "astar.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

  signals:
    void waypointsReady();

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  Ui::MainWindowDesign* ui;

  QNode* qnode;
  Driving* driving;

  std::vector<int> current_waypoints;

  cv::Mat clone_mat;
  cv::Mat gray_clone;
  int value_hsv[6];

  static constexpr int Raw_X = 640;
  static constexpr int Raw_Y = 360;
  int prev_left_x;
  int prev_right_x;
  bool first_frame;
  bool right_detected = false;
  bool left_detected = false;

  // UI 제어 함수들
  void showMapUI() { if (ui) ui->display_5->setVisible(true); }
  void hideMapUI() { if (ui) ui->display_5->setVisible(false); }

  /*
  bool system_running = false;        // 시스템 동작 상태 (true=동작, false=정지)
  int red_detection_threshold = 1000; // 빨간색 픽셀 임계값
  */

 public Q_SLOTS:
  void slotUpdateImg();
  void processWaypoints();

  void perspective_transform(const cv::Mat& input_img, cv::Mat& output_img);
  void Gaussain_Filter(cv::Mat& img);

  cv::Mat sumImg(cv::Mat img1, cv::Mat img2);
  cv::Mat white_hsv(cv::Mat& img);
  cv::Mat yellow_hsv(cv::Mat& img);
  //cv::Mat red_hsv(cv::Mat& img);
  //bool detectRedSignal();                           // 빨간색 신호 감지
  int left_line(cv::Mat original);
  int right_line(cv::Mat original);
  std::vector<int> getWindowSearch(cv::Mat& searchimg, int& left_x, int& right_x);

 private:
  void closeEvent(QCloseEvent* event);
  cv::Mat Raw_image, Perspective_img;
  Astar* a_planner;

};

#endif

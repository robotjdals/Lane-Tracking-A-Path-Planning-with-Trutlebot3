/**
 * @file /include/min_22_pkg/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2025
 **/

#ifndef min_22_pkg_MAIN_WINDOW_H
#define min_22_pkg_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>

#include "QIcon"
#include "driving.hpp"
#include "astar.hpp"
#include "ui_mainwindow.h"

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */


class MainWindow : public QMainWindow {
  Q_OBJECT

  signals:
    void waypointsReady();

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;
  Driving* driving;

  std::vector<int> current_waypoints;

  cv::Mat clone_mat;   // 원본 이미지 복사
  cv::Mat gray_clone;  // 흑백 이미지의 클론
  int value_hsv[6];    // hsv

  static constexpr int Raw_X = 640;
  static constexpr int Raw_Y = 360;
  int prev_left_x;
  int prev_right_x;
  bool first_frame;
  bool right_detected = false;
  bool left_detected = false;

  /*
  #define HLS_CHANNEL 1
  #define LAB_CHANNEL 2
  #define H_FILTER 0
  #define L_FILTER 1
  #define S_FILTER 2
  #define L_FILTER_ 0
  #define A_FILTER_ 1
  #define B_FILTER_ 2
*/

 public Q_SLOTS:
  void slotUpdateImg();
  //void slotUpdateNewImg();
  void processWaypoints();

  void perspective_transform(const cv::Mat& input_img, cv::Mat& output_img);
  void Gaussain_Filter(cv::Mat& img);
  //cv::Mat HLS_L(cv::Mat output_img);
  //cv::Mat LAB_B(cv::Mat output_img);
  //cv::Mat filterImg(cv::Mat input, int colorspace, int channel);
  cv::Mat sumImg(cv::Mat img1, cv::Mat img2);
  cv::Mat white_hsv(cv::Mat& img);
  cv::Mat yellow_hsv(cv::Mat& img);
  int left_line(cv::Mat original);
  int right_line(cv::Mat original);
  std::vector<int> getWindowSearch(cv::Mat& searchimg, int& left_x, int& right_x);

 private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
  cv::Mat Raw_image, Perspective_img;
  Astar* a_planner;

};

#endif  // hsv_detect_MAIN_WINDOW_H


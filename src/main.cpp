#include "../include/min_22_pkg/main_window.hpp"

#include <QApplication>
#include <iostream>

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}


#ifndef PLANNER_H
#define PLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "Helper.hpp"
#include "OtherVehicle.hpp"

class Planner {
public:
  Planner();

  ~Planner();

  void update(double car_x,
              double car_y,
              double car_s,
              double car_d,
              double car_yaw,
              double car_speed,
              vector<double> &previous_path_x,
              vector<double> &previous_path_y,
              double end_path_s,
              double end_path_d,
              vector<OtherVehicle> &sensor_fusion);

private:
  helper m_helper;

  double m_car_x;
  double m_car_y;
  double m_car_s;
  double m_car_d;
  double m_car_yaw;
  double m_car_speed;
  vector<double> m_previous_path_x;
  vector<double> m_previous_path_y;
  double m_end_path_s;
  double m_end_path_d;
  vector<OtherVehicle> m_sensor_fusion;
  vector<OtherVehicle> m_front_car;
};

#endif //PLANNER_H

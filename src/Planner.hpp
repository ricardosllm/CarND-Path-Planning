#ifndef PLANNER_H
#define PLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "Helper.hpp"
#include "OtherVehicle.hpp"
#include "Map.hpp"
#include "spline.h"
#include "Track.hpp"

#define TIME_INTERVAL 0.02f
#define MPH2MS				0.44704
#define TARGET_SPEED  48.5 // MPH

using namespace std;

class Planner {
public:
  Planner();

  ~Planner();

  enum lane_changing_status {
    CHANGING_LANE,
    KEEPING_LANE
  } m_change_status;

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

  void get_path(vector<double> &next_x_vals,
                vector<double> &next_y_vals);

  void set_map(Map &map);

private:
  helper m_helper;

  Map m_map;

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

  Track m_left,m_center,m_right;
  Track m_change;

  void keep_track(vector<double> &next_x_vals,
                  vector<double> &next_y_vals,
                  float speed,
                  int path_length,
                  Track &lane);

  double set_speed(double desired, double pre_speed);

  double inc2MPH(double inc);

  double MPH2inc(double MPH);

  OtherVehicle get_front_car(double d,double planned_s, double secure_dist);

  vector<OtherVehicle> cars_on_the_side(double d);

  vector<OtherVehicle> cars_on_this_lane(double d,
                                         vector<OtherVehicle> &among_these_cars);

  OtherVehicle closest_car(double s,vector<OtherVehicle> &among_these_cars);

  void setup_lane_changing(Track target, Track curr ,float s_obstacle);

  void change_lane_to(Track lane);

  void best_escape_lane(Track &current_lane, Track &target);
};

#endif //PLANNER_H

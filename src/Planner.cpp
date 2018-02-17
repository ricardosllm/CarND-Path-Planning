#include "Planner.hpp"

Planner::Planner() {
  m_helper = helper();

}

void Planner::update(double car_x,
                         double car_y,
                         double car_s,
                         double car_d,
                         double car_yaw,
                         double car_speed,
                         vector<double> &previous_path_x,
                         vector<double> &previous_path_y,
                         double end_path_s,
                         double end_path_d,
                         vector<OtherVehicle> &sensor_fusion){
  m_car_x = car_x;
  m_car_y = car_y;
  m_car_s = car_s;
  m_car_d = car_d;
  m_car_yaw = car_yaw;
  m_car_speed = car_speed;
  m_previous_path_x = previous_path_x;
  m_previous_path_y = previous_path_y;
  m_end_path_s = end_path_s;
  m_end_path_d = end_path_d;
  m_sensor_fusion = sensor_fusion;
}

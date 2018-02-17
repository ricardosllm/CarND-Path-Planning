#include "Planner.hpp"

Planner::Planner() {
  m_helper = helper();

}

Planner::~Planner() {}

void Planner::set_map(Map &map){
  m_map = map;

  m_lane2_x.set_points(m_map.m_map_waypoints_s,
                       m_helper.operation(m_map.m_map_waypoints_x,
                                          m_map.m_map_waypoints_dx,6));

  m_lane2_y.set_points(m_map.m_map_waypoints_s,
                       m_helper.operation(m_map.m_map_waypoints_y,
                                          m_map.m_map_waypoints_dy,6));
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

void Planner::get_path(vector<double> &next_x_vals,
                       vector<double> &next_y_vals){
  keep_lane(next_x_vals,next_y_vals);
}

void Planner::keep_lane(vector<double> &next_x_vals,
                        vector<double> &next_y_vals){
  double pos_x;
  double pos_y;
  double angle;

  int path_size = fmin( m_previous_path_x.size() , 20);
  double new_start_s;

  for (int i=0; i<path_size; i++) {
      next_x_vals.push_back(m_previous_path_x[i]);
      next_y_vals.push_back(m_previous_path_y[i]);
  }

  if (path_size==0) {
    pos_x = m_car_x;
    pos_y = m_car_y;
    angle = m_helper.deg2rad(m_car_yaw);
    new_start_s = m_car_s;
  } else {
    pos_x = m_previous_path_x[path_size-1];
    pos_y = m_previous_path_y[path_size-1];

    double pos_x2 = m_previous_path_x[path_size-2];
    double pos_y2 = m_previous_path_y[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

    new_start_s = m_end_path_s;
  }

  double dist_inc = set_speed (10); // MPH
  for (int i=0; i< 50-path_size; i++){
    double s_t = new_start_s + dist_inc * i;
    double ne_x = m_lane2_x(s_t);
    double ne_y = m_lane2_y(s_t);
    next_x_vals.push_back(ne_x);
    next_y_vals.push_back(ne_y);
  }
}

float Planner::set_speed(float desired){
  desired *= 0.44704;               // convert mph to m/s
  const float time_interval = 0.02; // second between each point
  float inc = desired * time_interval;
  return inc;
}

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

void Planner::keep_lane(vector<double> &next_x_vals,
                        vector<double> &next_y_vals){
  int not_met_path_size = m_previous_path_x.size();
  int path_length = 50;
  int n_set_points_from_pre_path = min(10, not_met_path_size);
  int idx_first_unmet_set_point_from_pre_path = path_length - not_met_path_size;
  vector<double> planned_s;
  static vector<double> pre_planned_s;

  if (not_met_path_size==0){
    planned_s.push_back( m_car_s );

    double dist_inc=MPH2inc(m_car_speed) ;

    for (int i=0; i<path_length-1 ; i++){
      dist_inc = set_speed (48, inc2MPH(dist_inc));//MPH
      planned_s.push_back(planned_s.back() + dist_inc);
    }
  } else {
    for (int i=0; i<n_set_points_from_pre_path; i++){
      planned_s.push_back(pre_planned_s[idx_first_unmet_set_point_from_pre_path + i]);
    }

    int len = planned_s.size();
    double dist_inc =planned_s[len-1]-planned_s[len-2];

    for (int i=n_set_points_from_pre_path; i<path_length ; i++){
      dist_inc = set_speed (48,inc2MPH(dist_inc));//MPH
      planned_s.push_back(planned_s.back() + dist_inc);
    }
  }

  for (auto it=planned_s.begin() ; it<planned_s.end(); it++){
    double s_p = *it;
    next_x_vals.push_back(m_lane2_x(s_p));
    next_y_vals.push_back(m_lane2_y(s_p));
  }
  pre_planned_s = planned_s;
  // m_tools.twoPlot(m_map.m_map_waypoints_x,
  //                 m_map.m_map_waypoints_y,
  //                 "blue",
  //                 next_x_vals,next_y_vals,
  //                 "red");
}

void Planner::get_path(vector<double> &next_x_vals,
                       vector<double> &next_y_vals){
  keep_lane(next_x_vals, next_y_vals);
}

float Planner::set_speed(float desired, float pre_speed){
  float step = 0.4;
  if (desired - pre_speed < 1.){
    step *= -1;
  } else if(fabs(desired - pre_speed) < 1.){
    step = 0;
  }

  float inc = MPH2inc(pre_speed+step);
  return inc;
}

float Planner::inc2MPH(float inc){
  float mph =inc / TIME_INTERVAL / 0.44704;
  return mph;
}

float Planner::MPH2inc(float MPH){
  float inc =MPH * TIME_INTERVAL *0.44704;
  return inc;
}

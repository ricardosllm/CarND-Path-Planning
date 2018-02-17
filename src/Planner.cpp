#include "Planner.hpp"

Planner::Planner() {
  m_helper = helper();
  m_change_status = Planner::KEEPING_LANE;
}

Planner::~Planner() {}

void Planner::set_map(Map &map){
  m_map = map;

  m_left = Track(m_map.m_map_waypoints_s,
                 m_helper.operation(m_map.m_map_waypoints_x,
                                    m_map.m_map_waypoints_dx,2),
                 m_helper.operation(m_map.m_map_waypoints_y,
                                    m_map.m_map_waypoints_dy,2),
                 1);

  m_center = Track(m_map.m_map_waypoints_s,
                   m_helper.operation(m_map.m_map_waypoints_x,
                                      m_map.m_map_waypoints_dx,6),
                   m_helper.operation(m_map.m_map_waypoints_y,
                                      m_map.m_map_waypoints_dy,6),
                   2);

  m_right = Track(m_map.m_map_waypoints_s,
                  m_helper.operation(m_map.m_map_waypoints_x,
                                     m_map.m_map_waypoints_dx,10),
                  m_helper.operation(m_map.m_map_waypoints_y,
                                     m_map.m_map_waypoints_dy,10),
                  3);

  m_left.priority_to_change_to.push_back(m_center);
  m_center.priority_to_change_to.push_back(m_left);
  m_center.priority_to_change_to.push_back(m_right);
  m_right.priority_to_change_to.push_back(m_center);
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

void Planner::keep_track(vector<double> &next_x_vals,
                         vector<double> &next_y_vals,
                         float speed,
                         int path_length,
                         Track &lane){
  int not_met_path_size = m_previous_path_x.size();
  int n_set_points_from_pre_path = min(10, not_met_path_size);
  int idx_first_unmet_set_point_from_pre_path = path_length - not_met_path_size;
  vector<double> planned_s;
  static vector<double> pre_planned_s;

  if (not_met_path_size==0){
    planned_s.push_back(m_car_s + 0.5);

    double dist_inc=MPH2inc(m_car_speed) ;

    for (int i=0; i<path_length-1 ; i++){
      dist_inc = set_speed (speed, inc2MPH(dist_inc));
      planned_s.push_back(planned_s.back() + dist_inc);
    }
  } else {
    for (int i=0; i<n_set_points_from_pre_path; i++){
      planned_s.push_back(pre_planned_s[idx_first_unmet_set_point_from_pre_path + i]);
    }

    int len = planned_s.size();
    double dist_inc =planned_s[len-1]-planned_s[len-2];

    for (int i=n_set_points_from_pre_path; i<path_length ; i++){
      dist_inc = set_speed (speed, inc2MPH(dist_inc));
      planned_s.push_back(planned_s.back() + dist_inc);
    }
  }

  for (auto it=planned_s.begin() ; it<planned_s.end(); it++){
    double s_p = *it;
    next_x_vals.push_back(lane.m_s_x(s_p));
    next_y_vals.push_back(lane.m_s_y(s_p));
  }
  pre_planned_s = planned_s;
}

void Planner::get_path(vector<double> &next_x_vals,
                       vector<double> &next_y_vals){
  float speed = TARGET_SPEED;
  int path_length = 50;
  float secure_dist = speed * MPH2MS * path_length * TIME_INTERVAL;

  // detect the front car
  OtherVehicle front_car_left   = get_front_car(2.0,m_car_s,secure_dist);
  OtherVehicle front_car_center = get_front_car(6.0,m_car_s,secure_dist);
  OtherVehicle front_car_right  = get_front_car(10.0,m_car_s,secure_dist);

  m_front_car.empty();
  m_front_car.push_back(front_car_left);
  m_front_car.push_back(front_car_center);
  m_front_car.push_back(front_car_right);

  switch (m_change_status){
  case CHANGING_LANE:
    if (m_car_s + secure_dist < m_change.m_s_end){
      keep_track(next_x_vals,
                 next_y_vals,
                 speed,
                 path_length,
                 m_change);
    } else {
      m_change_status = KEEPING_LANE;
    }
    break;
  case KEEPING_LANE:
    if (fabs(m_car_d-6.0)<1.0){ // In second lane


      if (front_car_center.id != 0){ //front_car_center.id != 0
        Track target;
        best_escape_lane(m_center,target);

        if (target.m_id != m_center.m_id){
          setup_lane_changing(target,  // front_car_center.s
                              m_center,
                              front_car_center.s);
        }

        float s_diff = front_car_center.s-m_car_s;
        speed = inc2MPH(s_diff/path_length);
        keep_track(next_x_vals,
                   next_y_vals,
                   speed,
                   path_length,
                   m_center);

      } else { // no car in front
        keep_track(next_x_vals,
                   next_y_vals,
                   speed,
                   path_length,
                   m_center);
      }
    } else if (fabs(m_car_d-2.0)<1.0){ // In left lane
      keep_track(next_x_vals,
                 next_y_vals,
                 speed,
                 path_length,
                 m_left);
    } else if (fabs(m_car_d-10.0)<1.0){ // In right lane
      keep_track(next_x_vals,
                 next_y_vals,
                 speed,
                 path_length,
                 m_right);
    }
    break;
  }
}

double Planner::set_speed(double desired, double pre_speed){
  double step = 0.2;
  if (desired - pre_speed < 1.){
    step *= -1;
  } else if(fabs(desired - pre_speed) < 1.){
    step = 0;
  }

  double inc = MPH2inc(pre_speed + step);
  return inc;
}

double Planner::inc2MPH(double inc){
  double mph = inc / TIME_INTERVAL / MPH2MS;
  return mph;
}

double Planner::MPH2inc(double MPH){
  float inc = MPH * TIME_INTERVAL * MPH2MS;
  return inc;
}

OtherVehicle Planner::get_front_car(double d, double planned_s, double secure_dist){
  OtherVehicle result = OtherVehicle();
  vector<OtherVehicle> CarsInMySide;
  vector<OtherVehicle> CarsInMyLane;

  CarsInMySide = cars_on_the_side(d);
  CarsInMyLane = cars_on_this_lane(d,CarsInMySide);

  OtherVehicle front_car = closest_car(planned_s,CarsInMyLane);

  if (front_car.s - planned_s < secure_dist &&
      front_car.s > planned_s &&
      CarsInMyLane.size()!=0)
    result = front_car;
  return result;
}

vector<OtherVehicle> Planner::cars_on_the_side(double d){
  vector<OtherVehicle> result;

  for (auto it=m_sensor_fusion.begin() ; it<m_sensor_fusion.end() ; it++){
    OtherVehicle car = *it;
    if(car.d * d > 0){
      result.push_back(car);
    }
  }
  return result;
}

vector<OtherVehicle> Planner::cars_on_this_lane(double d,
                                                vector<OtherVehicle> &among_these_cars){
  vector<OtherVehicle> result;

  if (among_these_cars.size()!=0){
    for (auto it=among_these_cars.begin() ; it<among_these_cars.end() ; it++){
      OtherVehicle car = *it;

      if(fabs(car.d - d) < 2.0){
        result.push_back(car);
      }
    }
  }
  return result;
}

OtherVehicle Planner::closest_car(double s,
                                  vector<OtherVehicle> &among_these_cars){
  OtherVehicle result;

  if (among_these_cars.size()!=0){
    double shortest_dist = INFINITY;

    for (auto it=among_these_cars.begin() ; it<among_these_cars.end() ; it++){
      OtherVehicle car = *it;

      if (car.s - s > 0 && car.s-s < shortest_dist ){
        shortest_dist = car.s-s;
        result = car;
      }
    }
  }
  return result;
}

void Planner::change_lane_to(Track lane){
  m_change_status = Planner::CHANGING_LANE;
}

void Planner::setup_lane_changing(Track target, Track curr ,float s_obstacle){
  vector<double> x,y,s_;
  float s = m_car_s;
  float inc =1.0;
  while (s < s_obstacle){
    s += inc;
    double temp_x = curr.m_s_x(s);
    double temp_y = curr.m_s_y(s);
    s_.push_back(s);
    x.push_back(temp_x);
    y.push_back(temp_y);
  }

  s += 40;
  float d = s + 40;

  while (s < d){
    s += inc;
    double temp_x = target.m_s_x(s);
    double temp_y = target.m_s_y(s);
    s_.push_back(s);
    x.push_back(temp_x);
    y.push_back(temp_y);
  }

  m_change = Track(s_, x, y, 0);
  m_change.m_s_end = s;
  m_change.m_s_x.set_points(s_, x);
  m_change.m_s_y.set_points(s_, y);
  m_change_status = CHANGING_LANE;
  m_helper.twoPlot(m_map.m_map_waypoints_x,
                   m_map.m_map_waypoints_y,
                   "blue",
                   x,
                   y,
                   "red");
}

void Planner::best_escape_lane(Track &current_lane, Track &target){
  Track result;
  float dist = INFINITY;

  for (int i=0; i<current_lane.priority_to_change_to.size(); i++){
    int id = current_lane.priority_to_change_to[i].m_id;

    if (m_front_car[id].s < dist){
      dist = m_front_car[id].s;
      result = current_lane.priority_to_change_to[i];
    }
  }
  target = result;
}

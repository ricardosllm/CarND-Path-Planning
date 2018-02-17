#include "Planner.hpp"

Planner::Planner() {
  m_helper = helper();
  m_change_status = Planner::KEEPING_LANE;

  OtherVehicle empty_car;

  for (int i=0; i<3; i ++)
    m_front_car.push_back(empty_car);
}

Planner::~Planner() {}

void Planner::set_map(Map &map, double max_s){
  m_map = map;

  Track left = Track(m_map.m_map_waypoints_s,
                     m_helper.operation(m_map.m_map_waypoints_x,
                                        m_map.m_map_waypoints_dx,2),
                     m_helper.operation(m_map.m_map_waypoints_y,
                                        m_map.m_map_waypoints_dy,2),
                     1, 2.0);

  Track center = Track(m_map.m_map_waypoints_s,
                       m_helper.operation(m_map.m_map_waypoints_x,
                                          m_map.m_map_waypoints_dx,6),
                       m_helper.operation(m_map.m_map_waypoints_y,
                                          m_map.m_map_waypoints_dy,6),
                       2, 6.0);

  Track right = Track(m_map.m_map_waypoints_s,
                      m_helper.operation(m_map.m_map_waypoints_x,
                                         m_map.m_map_waypoints_dx,10),
                      m_helper.operation(m_map.m_map_waypoints_y,
                                         m_map.m_map_waypoints_dy,10),
                      3, 10.0);

  left.priority_to_change_to.push_back(center);
  left.priority_to_change_to.push_back(left);
  center.priority_to_change_to.push_back(left);
  center.priority_to_change_to.push_back(right);
  center.priority_to_change_to.push_back(center);
  right.priority_to_change_to.push_back(center);
  right.priority_to_change_to.push_back(right);
  m_lanes.push_back(left);
  m_lanes.push_back(center);
  m_lanes.push_back(right);
  m_max_s = max_s;
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
  int n_set_points_from_pre_path = min(20, not_met_path_size);
  int idx_first_unmet_set_point_from_pre_path = path_length - not_met_path_size;
  vector<double> planned_s;
  static vector<double> pre_planned_s;

  if (not_met_path_size==0){
    planned_s.push_back(m_car_s + 0.005);

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
    double dist_inc = planned_s[len-1] - planned_s[len-2];

    if (dist_inc <= 0) {
      std::cout << "Reset: "
                << dist_inc
                << ", "
                << "Max_s: " << m_max_s
                << std::endl;
      dist_inc += m_max_s;
    }

    for (int i=n_set_points_from_pre_path; i<path_length ; i++){
      dist_inc = set_speed(speed, inc2MPH(dist_inc));
      double new_s = fmod(planned_s.back() + dist_inc, m_max_s);

      planned_s.push_back(new_s);
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
  float secure_dist = (speed + 20) * MPH2MS * path_length * TIME_INTERVAL;

  Track lane;
  // Get current lane
  for (auto it=m_lanes.begin(); it<m_lanes.end() ; it++){
    lane = *it;
    if (fabs(m_car_d-lane.m_d) < 1.5){ // I Am in this lane
      break;
    }
  }

  if(lane.m_id == 3) {
    speed = 46;
  }

  // detect the space to change lanes
  for (int idx = 0 ; idx < m_front_car.size(); idx++){
    m_front_car[idx].isEmpty = true; //reset it

    if (idx == (lane.m_id - 1)) {
      get_front_car(m_front_car[idx],
                    m_lanes[idx].m_d,
                    m_car_s,
                    secure_dist,
                    0);
    } else {
      get_front_car(m_front_car[idx],
                    m_lanes[idx].m_d,
                    m_car_s,
                    secure_dist + 20.0,
                    -10.0);
    }
  }

  switch (m_change_status){
  case CHANGING_LANE:
    // speed = TARGET_SPEED * 0.80; // Optimal speed for lane changing

    keep_track(next_x_vals, next_y_vals, speed, path_length, m_change);

    if (m_car_s + secure_dist >= m_change.m_s_end){
      m_change_status = KEEPING_LANE;
    }
    break;
  case KEEPING_LANE:
    OtherVehicle car = m_front_car[lane.m_id-1];

    if ( !(car.isEmpty) ){ // If there is a car in front on this line
      Track target;
      best_escape_lane(lane, target, secure_dist);

      float s_diff = car.s-m_car_s;
      speed = inc2MPH(s_diff / path_length);

      // if the target lane is not the same as current lane
      if (target.m_id != lane.m_id){
        setup_lane_changing(target, lane, car.s);
        keep_track(next_x_vals, next_y_vals, speed, path_length, m_change);
        break;

      } else {
        // if the target lane is the same as current lane
        // avoid crashing into the car in front
        speed = fmin(sqrt(car.vx*car.vx + car.vy * car.vy), TARGET_SPEED);
        keep_track(next_x_vals, next_y_vals, speed, path_length, lane);
        break;
      }
    } else { // no car in front
      keep_track(next_x_vals, next_y_vals, speed, path_length, lane);
      break;
    }
    break;
  }
}

double Planner::set_speed(double desired, double pre_speed){
  // double error = desired - pre_speed;
  // double step = 0.005 * error;

  // if (fabs(error) < 0.01){
  //   step = 0;
  // }

  // double inc = MPH2inc(pre_speed + step);
  // return inc;


  double error = desired - pre_speed;

  double sign = (0.0 < error) - (error < 0.0);
  return MPH2inc(sign * 0.224 + pre_speed);
}

double Planner::inc2MPH(double inc){
  double mph = inc / TIME_INTERVAL / MPH2MS;
  return mph;
}

double Planner::MPH2inc(double MPH){
  float inc = MPH * TIME_INTERVAL * MPH2MS;
  return inc;
}

void Planner::get_front_car(OtherVehicle &result1, double d, double planned_s,
                            double secure_dist, float secure_dist_neg){
  OtherVehicle result;

  vector<OtherVehicle> CarsInMySide;
  vector<OtherVehicle> CarsInMyLane;

  CarsInMySide = cars_on_the_side(d);
  CarsInMyLane = cars_on_this_lane(d, CarsInMySide);

  OtherVehicle front_car = closest_car(planned_s, CarsInMyLane, secure_dist_neg);

  if (front_car.s - planned_s < secure_dist &&
      front_car.s - planned_s > secure_dist_neg &&
      CarsInMyLane.size() != 0) {
    result = front_car;
  }

  result1 = result;
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

  if (among_these_cars.size() != 0){
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
                                  vector<OtherVehicle> &among_these_cars,
                                  float secure_dist_neg){
  OtherVehicle result;

  if (among_these_cars.size() != 0){
    double shortest_dist = INFINITY;

    for (auto it=among_these_cars.begin() ; it<among_these_cars.end() ; it++){
      OtherVehicle car = *it;

      if (car.s - s > secure_dist_neg && fabs(car.s-s) < shortest_dist){
				shortest_dist = car.s-s;
				result = car;
			}
		}
	}
	return result;
}

void Planner::setup_lane_changing(Track &target, Track &curr ,float s_obstacle){
  vector<double> x, y, s_;

  float s = m_car_s - 20.0;
  float inc = 1.0;

  while(s < s_obstacle){
    s += inc;
    double temp_x = curr.m_s_x(s);
    double temp_y = curr.m_s_y(s);
    s_.push_back(s);
    x.push_back(temp_x);
    y.push_back(temp_y);
  }

  s += 40;
  float d = s + 40;

  while(s < d){
    s += inc;
    double temp_x = target.m_s_x(s);
    double temp_y = target.m_s_y(s);
    s_.push_back(s);
    x.push_back(temp_x);
    y.push_back(temp_y);
  }

  m_change = Track(s_,x,y,0,0);
  m_change.m_s_end = s;
  m_change.m_s_x.set_points(s_,x);
  m_change.m_s_y.set_points(s_,y);
  m_change_status = CHANGING_LANE;
}

void Planner::best_escape_lane(Track &current_lane, Track &target, float secure_dist){
  Track result;
  float dist = INFINITY;

  for (int i=0; i<current_lane.priority_to_change_to.size(); i++){
    int id = current_lane.priority_to_change_to[i].m_id - 1;
    float s_diff = m_front_car[id].s - m_car_s;

    if (m_front_car[id].isEmpty){
      result = current_lane.priority_to_change_to[i];
      break;
    } else if (s_diff < dist && s_diff > -secure_dist){
      dist = m_front_car[id].s - m_car_s;
      result = current_lane.priority_to_change_to[i];

      if (dist < secure_dist) {
        result = current_lane;
      }
    }
  }
  target = result;
}

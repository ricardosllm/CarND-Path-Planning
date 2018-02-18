#ifndef LANE_H
#define LANE_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

using namespace std;

class Lane {

public:
  Lane() {
    m_id = 0;
    m_d = 0;
  }

  Lane(vector<double> s, vector<double> x, vector<double> y, int id, double d) {
    double temp_end = s.back();
    double delta_s = sqrt(pow(x.front()-x.back() , 2) + pow(y.front()-y.back() , 2));
    s.push_back(s.back() + delta_s);
    s.insert(s.begin(), s.front() - delta_s);

    temp_end = x.back();
    x.push_back(x.front());
    x.insert(x.begin(), temp_end);

    temp_end = y.back();
    y.push_back(y.front());
    y.insert(y.begin(), temp_end);

    m_s_x.set_points(s, x);
    m_s_y.set_points(s, y);
    m_id = id;
    m_s_end = 0;
    m_d = d;
  }

  ~Lane() {}

  tk::spline m_s_x,m_s_y;
  int m_id;
  vector<Lane> priority_to_change_to;
  double m_s_end;
  double m_d;

private:

};

#endif // LANE_H

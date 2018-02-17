#ifndef TRACK_H
#define TRACK_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

using namespace std;

class Track{

public:
  Track(){m_id = 0;m_d = 0;}
  Track(vector<double> s,
        vector<double> x,
        vector<double> y,
        int id,
        double d){
    m_s_x.set_points(s,x);
    m_s_y.set_points(s,y);
    m_id = id;
    m_s_end = 0;
    m_d = d;
  }
  ~Track(){}

  tk::spline m_s_x,m_s_y;
  int m_id;
  vector<Track> priority_to_change_to;
  double m_s_end;
  double m_d;

private:

};

#endif // TRACK_H

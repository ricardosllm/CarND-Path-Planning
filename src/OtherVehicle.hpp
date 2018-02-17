#ifndef OTHER_VECHICLE_H
#define OTHER_VECHICLE_H

class OtherVehicle{
public:
  OtherVehicle(int id,
               float x,
               float y,
               float vx,
               float vy,
               float s,
               float d):id(id)
    ,x(x),y(y),vx(vx),vy(vy),s(s),d(d){}

  OtherVehicle() {
    OtherVehicle(0,0,0,0,0,0,0);
  }

  int id = 0;
  float x;
  float y;
  float vx;
  float vy;
  float s;
  float d;

private:

};

#endif // OTHER_VECHICLE_H

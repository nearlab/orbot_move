#include <math.h>
#include <stdint.h>

#define pi 3.14159265359

class Orbot{
public:
  Orbot(float x_cm=0, float y_cm=0, float length=0.04, float width=0.037, float radius=0.0076);
  void getRotationRates(float* rates,float vx, float vy, float vTheta);
private:
  float wheel_locs[2][4];
  float x_cm, y_cm, length, width, radius;
};





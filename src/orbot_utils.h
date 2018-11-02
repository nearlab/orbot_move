#include <math.h>
#include <stdint.h>

#define pi = 3.14159265359

class Orbot{
public:
  Orbot();
  Orbot(float x_cm, float y_cm, float length, float width, float radius);
  void getRotationRates(float* rates,float vx, float vy, float vTheta);
private:
  float wheel_locs[2][4];
  float x_cm, y_cm, length, width, radius;
};





#include <MatrixMath.h>

#define CONV_D2R M_PI/180

void fk(float* state, float* stateavg, float* L) {
  state[6] = L[0]*sin(stateavg[1]*CONV_D2R)+L[1]*sin(stateavg[0]*CONV_D2R);
  state[7] = -L[0]*cos(stateavg[1]*CONV_D2R)-L[1]*cos(stateavg[0]*CONV_D2R);
}

// row major -> (0,0)=0, (0,1)=1, (1,0)=2, (1,1)=3
void jacobian(float* jacobian) {
  jacobian[0] = L[1]*cos((stateavg[0]-stateavg[1])*CONV_D2R) + L[0]*cos(stateavg[1]*CONV_D2R);
  jacobian[1] = -L[1]*cos((stateavg[0]-stateavg[1])*CONV_D2R);
  jacobian[2] = -L[1]*sin((stateavg[0]-stateavg[1])*CONV_D2R) - L[0]*sin(stateavg[1]*CONV_D2R);
  jacobian[3] = L[1]*sin((stateavg[0]-stateavg[1])*CONV_D2R);
}


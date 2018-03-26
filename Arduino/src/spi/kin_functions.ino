#include <MatrixMath.h>

#define CONV_D2R M_PI/180

void fk(float* state, float* stateavg float* L) {
  state[6] = L[0]*sin(stateavg[1]*CONV_D2R)+L[1]*sin(stateavg[0]*CONV_D2R);
  state[7] = -L[0]*cos(stateavg[1]*CONV_D2R)-L[1]*cos(stateavg[0]*CONV_D2R);
}


#pragma once

#include "EZ-Template/drive/drive.hpp"

struct Pose {
  float x;
  float y;
  float theta;

  Pose(float x_ = 0, float y_ = 0, float theta_ = 0)
      : x(x_), y(y_), theta(theta_) {}
};

struct Particle {
  Pose pose;
  float weight;

  Particle() : pose(), weight(1.0f) {}
  Particle(const Pose &p, float w) : pose(p), weight(w) {}
};

void startMCL(ez::Drive &chassis);
void stopMCL();

void updateMCL(ez::Drive &chassis,
               float north_dist,
               float south_dist,
               float east_dist,
               float west_dist);

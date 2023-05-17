#pragma once

#include "Vector2D.h"

struct KinematicInfo final {
    Vector2D linearVelocity;
    Vector2D linearAcceleration;
    Vector2D displacementVector;
    double angularVelocity;
    double angularAcceleration;
    double rotatedAngle;
};

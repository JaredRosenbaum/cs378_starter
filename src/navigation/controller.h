//========================================================================
/*!
\file    controller.h
\author  Nihar, Jared, Eric, Daniel
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H


namespace controller {

class Controller {
 public:

  // Constructor
  explicit Controller();

  void CalculateValues(double vCurrent, double vMax, double aMax, double distanceTravelled, double goalDist, double results);

  void Acceleration(double vMax, double vCurrent, double aMax, double dt, double distanceLeft, double results);

  void Cruise(double vMax, double vCurrent, double aMax, double dt, double distanceLeft, double results);

  void Deceleration(double vMax, double vCurrent, double aMax, double aCurrent, double dt, double distanceLeft, double goalDist, double results);

 private:

  
};

}  // namespace controller

#endif  // CONTROLLER_H

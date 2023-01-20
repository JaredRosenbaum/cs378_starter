//========================================================================
/*!
\file    controller.h
\author  Nihar, Jared, Eric, Daniel
*/
//========================================================================

#ifndef CONTROLLER_H
#define CONTROLLER_H


namespace navigation {

class Controller {
  public:

  double Run(double vCurrent, double vMax, double aMax, double distanceTravelled, double goalDist, double cp1_distance);

  private:

};

}  // namespace navigation

#endif  // CONTROLLER_H

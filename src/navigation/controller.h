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

  float Run(float vCurrent, float distanceTraveled, float cp1_distance);

  private:

};

}  // namespace navigation

#endif  // CONTROLLER_H
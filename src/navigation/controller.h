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

  float Run(float vCurrent, float distanceTraveled, float cp1_distance, float free_path_length);

  float FreePathLength(std::vector<Eigen::Vector2f> point_cloud_, float cp3_curvature);

  float Clearance(std::vector<Eigen::Vector2f> point_cloud_, float curvature, float free_path_length);

  float DistanceLeft(std::vector<Eigen::Vector2f> point_cloud_, float cp3_curvature, float free_path_length);

  private:

};

}  // namespace navigation

#endif  // CONTROLLER_H

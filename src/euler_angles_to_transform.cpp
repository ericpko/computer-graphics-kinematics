#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Source: Textbook chapter 6 and https://open.gl/transformations
   */

  // Get the radians for Twist-Bend-Twist degrees
  double theta1 = xzx[0] * (M_PI / 180.0);
  double theta2 = xzx[1] * (M_PI / 180.0);
  double theta3 = xzx[2] * (M_PI / 180.0);

  // Create the rotation matricies
  Eigen::Affine3d A, B, C, T;
  A = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX());
  B = Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitZ());
  C = Eigen::AngleAxisd(theta3, Eigen::Vector3d::UnitX());

  T = C * B * A;


  return T;




  // Below is what is actually happening
  // Eigen::Affine3d A, B, C;
  // A.matrix() <<
  //   1, 0, 0, 0,
  //   0, cos(theta1), -sin(theta1), 0,
  //   0, sin(theta1), cos(theta1), 0,
  //   0, 0, 0, 1;

  // B.matrix() <<
  //   cos(theta2), -sin(theta2), 0, 0,
  //   sin(theta2), cos(theta2), 0, 0,
  //   0, 0, 1, 0,
  //   0, 0, 0, 1;

  // C.matrix() <<
  //   1, 0, 0, 0,
  //   0, cos(theta3), -sin(theta3), 0,
  //   0, sin(theta3), cos(theta3), 0,
  //   0, 0, 0, 1;


  // return C * B * A;
  /////////////////////////////////////////////////////////////////////////////
}

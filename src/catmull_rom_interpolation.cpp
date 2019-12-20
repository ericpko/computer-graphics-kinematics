#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>
#include<Eigen/Geometry>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  /**
   * The solution below is based on the textbook pg. 383.
   */
  // Check if there are keyframes
  if (keyframes.size() == 0) {
    return Eigen::Vector3d(0, 0, 0);
  }

  // Loop the query times if t is greater than the largest keyframe time
  t = std::fmod(t, keyframes.back().first);

  // Step 1: Find index i for query time t. This is curve segment i
  int i = 0;
  for (i = 0; i < keyframes.size() - 1; i++) {
    // find t_i < t <= t_i+1
    if (keyframes[i].first < t && t <= keyframes[i+1].first) {
      break;
    }
  }

  // Set up some variables
  Eigen::Vector3d p0, p1, p2, p3;                       // points
  double t0, t1, t2, t3;                                // times
  double tension = 0.0;

  /**
   * Since Catmull-Rom doesn't interpolate the first or last point, we will use
   * cosine interpolation for these two points.
   */

  // Case 1: first point or last point
  if (i == 0 || i == keyframes.size() - 2) {
    // Source: http://paulbourke.net/miscellaneous/interpolation/
    p0 = keyframes[i].second;
    p1 = keyframes[i+1].second;

    t0 = keyframes[i].first;
    t1 = keyframes[i+1].first;

    double u = (t - t0) / (t1 - t0);
    double mu2 = (1 - cos(u * M_PI)) / 2;

    // Interpolate
    return (1 - mu2) * p0 + mu2 * p1;
  }

  // Case 2: some middle point
  /**
   * Find our four points. We will interpolate between p1 and p2.
   * We want:
   * f(0) = p1
   * f(1) = p2
   * f'(0) = (p2 - p0) / (t2 - t0)
   * f'(1) = (p3 - p1) / (t3 - t1)
   */
  p0 = keyframes[i-1].second;
  p1 = keyframes[i].second;
  p2 = keyframes[i+1].second;
  p3 = keyframes[i+2].second;

  t0 = keyframes[i-1].first;
  t1 = keyframes[i].first;
  t2 = keyframes[i+1].first;
  t3 = keyframes[i+2].first;

  // Calculate u --- unit parameter in [0, 1]
  double _u = (t - t1) / (t2 - t1);
  Eigen::RowVector4d u = Eigen::RowVector4d(1, _u, pow(_u, 2), pow(_u, 3));

  // Calculate derivatives/tangents at P1 and P2
  // Source: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull%E2%80%93Rom_spline
  Eigen::Vector3d v1 = (p2 - p0) / (t2 - t0);
  Eigen::Vector3d v2 = (p3 - p1) / (t3 - t1);

  // Calculate constraint matrix. See iPad for calculations
  Eigen::Matrix4d C;
  C <<
    1, 1 - (t2 - t0), 1, 1,
    1, 0, 0, 0,
    1, 1, 1, 1,
    1, (t3 - t1), 2 * (t3 - t1), 3 * (t3 - t1);

  // Calculate the cardinal matrix B and blending functions b(u)
  Eigen::Matrix4d B = C.inverse();
  // Eigen::Vector4d b_of_u = (u * B);

  // Create a matrix for the control points
  Eigen::Matrix<double, 4, 3> p;
  p.row(0) = p0;
  p.row(1) = p1;
  p.row(2) = p2;
  p.row(3) = p3;


  // Interpolate
  return u * B * p;
  /////////////////////////////////////////////////////////////////////////////
}







/**
 * This solution below is based on the wikipedia page:
 * https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull%E2%80%93Rom_spline
 * This solution also works and should be doing the exact same thing as above.
 * It's probably a bit faster as well, but the solution above is more general.
 */
// #include "catmull_rom_interpolation.h"
// #include <Eigen/Dense>
// #include<Eigen/Geometry>

// Eigen::Vector3d catmull_rom_interpolation(
//   const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
//   double t)
// {
//   /////////////////////////////////////////////////////////////////////////////
//   // Check if there are keyframes
//   if (keyframes.size() == 0) {
//     return Eigen::Vector3d(0, 0, 0);
//   }

//   // Loop the query times if t is greater than the largest keyframe time
//   t = std::fmod(t, keyframes.back().first);

//   // Step 1: Find index i for query time t. This is curve segment i
//   int i = 0;
//   for (i = 0; i < keyframes.size() - 1; i++) {
//     // find t_i < t <= t_i+1
//     if (keyframes[i].first < t && t <= keyframes[i+1].first) {
//       break;
//     }
//   }

//   // Set up some variables
//   Eigen::Vector3d p0, p1, p2, p3;                       // points
//   double t0, t1, t2, t3;                                // times
//   double tension = 0.0;

//   /**
//    * Since Catmull-Rom doesn't interpolate the first or last point, we will use
//    * cosine interpolation for these two points.
//    */

//   // Case 1: first point or last point
//   if (i == 0 || i == keyframes.size() - 2) {
//     // Source: http://paulbourke.net/miscellaneous/interpolation/
//     p0 = keyframes[i].second;
//     p1 = keyframes[i+1].second;

//     t0 = keyframes[i].first;
//     t1 = keyframes[i+1].first;

//     double u = (t - t0) / (t1 - t0);
//     double mu2 = (1 - cos(u * M_PI)) / 2;

//     // Interpolate
//     return (1 - mu2) * p0 + mu2 * p1;
//   }

//   // Case 2: some middle point
//   /**
//    * Find our four points. We will interpolate between p1 and p2.
//    * We want:
//    * f(0) = p1
//    * f(1) = p2
//    * f'(0) = (p2 - p0) / (t2 - t0)
//    * f'(1) = (p3 - p1) / (t3 - t1)
//    */
//   p0 = keyframes[i-1].second;
//   p1 = keyframes[i].second;
//   p2 = keyframes[i+1].second;
//   p3 = keyframes[i+2].second;

//   t0 = keyframes[i-1].first;
//   t1 = keyframes[i].first;
//   t2 = keyframes[i+1].first;
//   t3 = keyframes[i+2].first;

//   // Calculate u --- unit parameter in [0, 1]
//   double u = (t - t1) / (t2 - t1);

//   // Calculate derivatives/tangents at P1 and P2
//   // Source: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull%E2%80%93Rom_spline
//   Eigen::Vector3d m1 = (p2 - p0) / (t2 - t0);
//   Eigen::Vector3d m2 = (p3 - p1) / (t3 - t1);

//   // Set up our blending functions
//   double h_00 = 2 * pow(u, 3) - 3 * pow(u, 2) + 1;
//   double h_10 = pow(u, 3) - 2 * pow(u, 2) + u;
//   double h_01 = -2 * pow(u, 3) + 3 * pow(u, 2);
//   double h_11 = pow(u, 3) - pow(u, 2);

//   Eigen::Vector3d interp = h_00 * p1 + h_10 * m1 + h_01 * p2 + h_11 * m2;


//   return interp;
//   /////////////////////////////////////////////////////////////////////////////
// }

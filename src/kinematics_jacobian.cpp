#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  J = Eigen::MatrixXd::Zero(b.size() * 3, skeleton.size() * 3);  // |x| x |a|
  double da = 1.0e-7;

  // Get the pose positions
  Eigen::VectorXd tip_pos = transformed_tips(skeleton, b);
  Skeleton sk_cp = skeleton;


  // Iterate over each bone i
  for (int i = 0; i < skeleton.size(); i++) {
    // Save xzx rotations so we can reset the skeleton
    Eigen::Vector3d xzx = skeleton[i].xzx;

    // Iterate over each angle j
    for (int j = 0; j < 3; j++) {
      // Update the angle j of bone i by a small amount da
      sk_cp[i].xzx(j) += da;

      // Get the new transformed tips after the angle adjustment
      // Numerator of eq. 31 from README.html
      Eigen::VectorXd dx = transformed_tips(sk_cp, b) - tip_pos;

      // Now we iterate down a column of the Jacobian and add dx/da for index ij
      // We're adding a column for bone i and angle j
      for (int k = 0; k < J.rows(); k++) {
        J(k, i * 3 + j) = dx(k)/da;             // dx/da = J_ij
      }
    }

    // Reset xzx rotations after we add entry to Jacobian
    sk_cp[i].xzx = xzx;
  }
  /////////////////////////////////////////////////////////////////////////////
}

#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Set up the list of pose tip positons for the specified bones in b
  Eigen::VectorXd tip_positions = Eigen::VectorXd::Zero(3 * b.size());

  // Get the list of pose transformations
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T;
  forward_kinematics(skeleton, T);

  // Iterate through the given bones indices.
  // b[i] is an index into skeleton of a bone
  for (int i = 0; i < b.size(); i++) {
    // Get the canonical tip vector in homogenous coordinates
    Eigen::Vector4d tip_pos = Eigen::Vector4d(skeleton[b(i)].length, 0, 0, 1);

    // Compute the pose position <x>
    Eigen::Vector4d x = T[b(i)] * skeleton[b(i)].rest_T * tip_pos;
    x /= x(3);         // divide by homogeneous coordinate w

    // Set the tip of bone i's pose position
    tip_positions.segment(i * 3, 3) = x.head(3) / x(3);

    // tip_positions(i * 3 + 0) = x(0);
    // tip_positions(i * 3 + 1) = x(1);
    // tip_positions(i * 3 + 2) = x(2);
  }


  return tip_positions;
  /////////////////////////////////////////////////////////////////////////////
}

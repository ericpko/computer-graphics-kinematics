#include "copy_skeleton_at.h"
Skeleton copy_skeleton_at(
  const Skeleton & skeleton,
  const Eigen::VectorXd & A)
{
  /////////////////////////////////////////////////////////////////////////////
  Skeleton copy = skeleton;
  for (int i = 0; i < skeleton.size(); i++) {
    double twist = A[i * 3 + 0];
    double bend = A[i * 3 + 1];
    double twist2 = A[i * 3 + 2];

    copy[i].xzx = Eigen::Vector3d(twist, bend, twist2);
  }

  return copy;
  /////////////////////////////////////////////////////////////////////////////
}

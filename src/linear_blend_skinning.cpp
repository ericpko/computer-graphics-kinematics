#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  U.resize(V.size(), 3);

  // For each vertex i
  for (int i = 0; i < V.rows(); i++) {
    // Set our new pose position
    Eigen::Vector4d v = Eigen::Vector4d::Zero();

    // For each bone
    for (int j = 0; j < skeleton.size(); j++) {
      // Check the weight index
      if (skeleton[j].weight_index != -1) {
        // Get the weight w_ij of bone j on vertex i
        double w = W(i, skeleton[j].weight_index);
        // Get the rest and pose positions of vertex i
        Eigen::Vector4d rest_pos = Eigen::Vector4d(V(i, 0), V(i, 1), V(i, 2), 1);
        Eigen::Vector4d pose_pos = T[j] * rest_pos;

        // Add to the weighted average for vertex i
        v += pose_pos * w;
      }
    }
    // Divide v by the homogeneous index w and assign deformed mesh positions
    v /= v(3);

    U(i, 0) = v(0);
    U(i, 1) = v(1);
    U(i, 2) = v(2);
  }
  /////////////////////////////////////////////////////////////////////////////
}

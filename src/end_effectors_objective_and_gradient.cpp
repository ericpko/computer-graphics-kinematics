#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>
#include <assert.h>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Function handle that computes the least-squares objective value given a
   * #bones * 3 list of Euler angles.
   * See README.html Inverse kinematics section
   */
  f = [&](const Eigen::VectorXd & A)->double {

    double E = 0.0;
    // Copy the skeleton with joint angles set to those in vector A
    Skeleton skeleton_cp = copy_skeleton_at(skeleton, A);
    // Get the updated transformed pose tip positions (end effectors)
    Eigen::VectorXd tip_pos = transformed_tips(skeleton_cp, b);

    // Iterate over all end effector constaints (each end effector has xyz position)
    for (int i = 0; i < b.size(); i++) {
      // Get the transformed tip position of bone b(i)
      Eigen::Vector3d x = tip_pos.segment(i * 3, 3);
      // Get the desired goal position q
      Eigen::Vector3d q = xb0.segment(i * 3, 3);

      // Compute the energy/cost function/sum of squares
      E += (x - q).squaredNorm();
    }

    return E;
  };


  /**
   * Function handle that computes the least-squares objective gradient
   * given a #bones * 3 list of Euler angles.
   * See README.html projected gradient descent section
   */
  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd {

    // Get the end effector tips (pose position of tips with joint angles from A)
    Skeleton skeleton_cp = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd tip_pos = transformed_tips(skeleton_cp, b);   // size == |x|

    // Compute the kinematic Jacobian matrix
    Eigen::MatrixXd J;
    kinematics_jacobian(skeleton_cp, b, J);

    // NOTE: the gradient result will be size = |a| since J transpose
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(A.size());

    // Iterate over end effector POSITIONS xyz (i.e. |x|)         tip_pos.size()
    for (int i = 0; i < 3 * b.size(); i++) {
      // Since E(x) = ||x-q||^2, then E'(x) = 2(x-q)
      // See https://github.com/alecjacobson/computer-graphics-kinematics/issues/7
      double dEdx = 2 * (tip_pos(i) - xb0(i));
      // Remember, each row of J is a single function w.r.t. different angles for each bone
      Eigen::VectorXd J_i = J.row(i).transpose();

      // Update the gradient
      gradient += J_i * dEdx;                               // J_i.size() == |a|
    }

    return gradient;
  };



  /**
   * Function handle that projects a given set of Euler angles onto the
   * input skeleton's joint angles.
   * See README.html projected gradient descent section.
   */
  proj_z = [&](Eigen::VectorXd & A) {

    // assert(skeleton.size() * 3 == A.size());

    // For each bone
    for (int i = 0; i < skeleton.size(); i++) {
      Bone bone = skeleton[i];

      // For each angle
      for (int j = 0; j < 3; j++) {
        A(i * 3 + j) = std::max(bone.xzx_min(j), std::min(bone.xzx_max(j), A(i * 3 + j)));
      }
    }
  };
  /////////////////////////////////////////////////////////////////////////////
}

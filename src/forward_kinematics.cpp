#include "Skeleton.h"                   // VSCode complains without this here
#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());

  // Define the recursive FK function
  std::function<Eigen::Affine3d(int)> fk_T;
  fk_T = [&fk_T, &skeleton](int index) -> Eigen::Affine3d {
    // Base case:
    if (skeleton[index].parent_index == -1) {
      // Then we're at the root
      return Eigen::Affine3d::Identity();

    } else {
      // Recursively get our parents pose transformation <T_p>
      Eigen::Affine3d T_p = fk_T(skeleton[index].parent_index);

      // Get the canonical-to-rest mapping and relative rotation transformations
      Eigen::Affine3d T_rest = skeleton[index].rest_T;
      Eigen::Affine3d R_i = euler_angles_to_transform(skeleton[index].xzx);

      // Return the rigid pose transformation T_i for bone <i>
      return T_p * T_rest * R_i * T_rest.inverse();
    }
  };

  // Iterate over each bone i to get the rigid pose transformation T_i
  for (int i = 0; i < skeleton.size(); i++) {
    T[i] = fk_T(i);
  }
  /////////////////////////////////////////////////////////////////////////////
}


/**
 * NOTE: We need to use the std::function in this case because we are recursing,
 * otherwise we could just use auto before defining the lambda function. Also,
 * we could use a "c" pointer function declaration instead, so std::function
 * just makes the syntax nicer, but that's basically what it's doing. Also,
 * the list [func1, func2, fk_T, etc.] are functions used in the lambda.
 */

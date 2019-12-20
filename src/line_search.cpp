#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  double sigma = max_step;

  // Get new projected <a> (or z in this case)
  Eigen::VectorXd z_update = z - sigma * dz;
  proj_z(z_update);

  // We found **optimal** step when E(proj(a+sigma* delta a) < E(a)
  while (f(z_update) > f(z)) {
    sigma /= 2.0;                                 // decrease step size by 1/2
    z_update = z - sigma * dz;
    proj_z(z_update);
  }


  return sigma;
  /////////////////////////////////////////////////////////////////////////////
}

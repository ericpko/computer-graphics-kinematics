#include "projected_gradient_descent.h"
#include "line_search.h"

void projected_gradient_descent(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const int max_iters,
  Eigen::VectorXd & z)
{
  /////////////////////////////////////////////////////////////////////////////
  const double max_step = 10000.0;

  for (int i = 0; i < max_iters; i++) {
    // Get the gradient of z
    Eigen::VectorXd grad_z = grad_f(z);

    // Find the best step size
    double sigma = line_search(f, proj_z, z, grad_z, max_step);

    // Update z
    z -= sigma * grad_z;
    proj_z(z);
  }
  /////////////////////////////////////////////////////////////////////////////
}

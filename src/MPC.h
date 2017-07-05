#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"


// Set cte, epsi, and speed
// Both the reference cross track error (REF_CTE) and reference psi error (REF_EPSI) are set to 0.
// The reference speed (REF_V) is set to 20 m/s or ~45 mph.
#define REF_CTE 0.0
#define REF_EPSI 0.0
#define REF_V 20.0 // m/s

// Set weights for the cost function
#define CTE_WEIGHT 2.0
#define EPSI_WEIGHT 20.0
#define V_WEIGHT 1.0
#define DELTA_WEIGHT 1.0
#define A_WEIGHT 21.0
#define DDELTA_WEIGHT 100.0
#define DA_WEIGHT 0.0

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */

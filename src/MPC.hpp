#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/utility/vector.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "utils.hpp"

using namespace std;

const double LF = 2.67;

const size_t N = 9;
const double DT = 0.22;

const double REF_CTE = 0;
const double REF_EPSI = 0;
const double REF_V = mph2mps(80);

const size_t IX_X_START = 0;
const size_t IX_Y_START = IX_X_START + N;
const size_t IX_PSI_START = IX_Y_START + N;
const size_t IX_V_START = IX_PSI_START + N;
const size_t IX_CTE_START = IX_V_START + N;
const size_t IX_EPSI_START = IX_CTE_START + N;
const size_t IX_DELTA_START = IX_EPSI_START + N;
const size_t IX_A_START = IX_DELTA_START + N - 1;

const double LAMBDA_CTE = 2.0;
const double LAMBDA_EPSI = 3.0;
const double LAMBDA_V = 0.1;
const double LAMBDA_DELTA = 150.0;
const double LAMBDA_A = 0.2;
const double LAMBDA_DDELTA = 20.0;
const double LAMBDA_DA = 50.0;


class MPC {
public:
    MPC();

    virtual ~MPC();

    vector<double> x_vals;
    vector<double> y_vals;
    double steer;
    double throttle;
    CppAD::vector<double> last_solution;
    int steps_since_last_solution = 0;


    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */

#include "mpc.hpp"
#include <string>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

const size_t N = 10;
const double dt = 0.1;

// reference velocity
const double ref_v = 40.0; //m/s

// simulation used to figure this out
const double Lf = 2.67;

// cost weights
// const double costs[] = {3000, 3000, 1, 5, 5, 700, 200, 10};
const double costs[] = {1, 1, 1, 1, 1, 0, 1, 1};

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs) {
    this->coeffs = coeffs;
  }

  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

  // fg is a vector containing the cost and constraints
  // vars is a vector containing the variable values (state + actuator)
  void operator()(ADvector& fg, const ADvector& vars) {
    /*
     * Define cost
     * cost is stored in first element of fg
     */
    fg[0] = 0;
    // reference costs
    for(int t=0; t<N; t++) {
      fg[0] += costs[0] * CppAD::pow(vars[cte_start+t], 2); // cte_cost
      fg[0] += costs[1] * CppAD::pow(vars[epsi_start+t], 2); // heading error
      fg[0] += costs[2] * CppAD::pow(vars[v_start+t] - ref_v, 2); // velocity error
    }
    // minimize use of actuators
    for(int t = 0; t < N - 1; t++) {
      fg[0] += costs[3] * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += costs[4] * CppAD::pow(vars[a_start + t], 2);
      fg[0] += costs[5] * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
    }
    // minimize the value gap between sequential actuations
    for(int t=0; t < N - 2; t++) {
      fg[0] += costs[6] * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += costs[7] * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    /*
     * Setup constraints
     */
    // intial constraints (index bumped by 1 due to cost at fg[0])
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // rest of the constraints
    for(int t = 1; t < N; ++t) {
      // grab at t+1 and t states
      CppAD::AD<double> x0 = vars[x_start + t - 1];
      CppAD::AD<double> x1 = vars[x_start + t];
      CppAD::AD<double> y0 = vars[y_start + t - 1];
      CppAD::AD<double> y1 = vars[y_start + t];
      CppAD::AD<double> psi0 = vars[psi_start + t - 1];
      CppAD::AD<double> psi1 = vars[psi_start + t];
      CppAD::AD<double> v0 = vars[v_start + t - 1];
      CppAD::AD<double> v1 = vars[v_start + t];
      CppAD::AD<double> cte0 = vars[cte_start + t - 1];
      CppAD::AD<double> cte1 = vars[cte_start + t];
      CppAD::AD<double> epsi0 = vars[epsi_start + t - 1];
      CppAD::AD<double> epsi1 = vars[epsi_start + t];

      // actuators at time t
      CppAD::AD<double> delta0 = vars[delta_start + t - 1];
      CppAD::AD<double> a0 = vars[a_start + t - 1];

      CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      CppAD::AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // model contraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
        cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
        epsi1 - ((psi0 - psides0) - (v0 / Lf * delta0 * dt));
    }
  }
};

// class FG_eval {
//  public:
//   // Fitted polynomial coefficients
//   Eigen::VectorXd coeffs;
//   FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
//
//
// };

std::vector<double> MPC::solve(const Eigen::VectorXd &x0,
  const Eigen::VectorXd &coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // current state
  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double v = x0[3];
  double cte = x0[4];
  double epsi = x0[5];

  // number of independent variables
  const size_t n_vars = N * 6 + (N-1) * 2; // (N-1) iterations
  // number of constraits
  const size_t n_constraints = N * 6;

  // initial value of independent variables
  // 0 except for initial values
  Dvector vars(n_vars);
  for(int i=0; i<n_vars; ++i) {
    vars[i] = 0.0;
  }
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // lower and upper limits of x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // max limits for non-actuators
  for(int i=0; i< delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // delta limits -25 to 25 degrees
  for(int i=delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // accel limits
  for(int i=a_start; i<n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // lower and upper limits for constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  // all should be 0 except for inital condtiions
  for(int i=0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // solver options
  std::string options;
  options += "Integer print_level 0\n";
  options += "Sparse true         forward\n";
  options += "Sparse true         reverse\n";
  options += "Numeric max_cpu_time          0.1\n";


  // solve the problem
  CppAD::ipopt::solve_result<Dvector> solution; // hold results
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars,
    vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);

  // return solution
  // bool ok = true & (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
  // auto cost = solution.obj_value;
  // std::cout << solution.x[x_start + 1] << "\n" << solution.x[y_start + 1]
  //   << "\n" << solution.x[psi_start + 1] << "\n" << solution.x[v_start + 1]
  //   << "\n" << solution.x[cte_start + 1] << "\n" << solution.x[epsi_start + 1]
  //   << "\n" << solution.x[delta_start] << "\n" << solution.x[a_start] << "\n" << std::endl;
  std::vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  for ( int i = 0; i < N - 2; i++ ) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  return result;
}

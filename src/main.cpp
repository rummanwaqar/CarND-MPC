#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <cmath>

#include "io.hpp"
#include "mpc.hpp"
#include "helpers.hpp"

const int PORT = 4567;

int main(int argc, char** argv) {
  MPC mpc;
  std::cout << "Connecting to simulator" << std::endl;
  SimIO simulator(PORT, [&](std::vector<double> ptsx, std::vector<double> ptsy,
    double px, double py, double psi, double v, double delta, double a) {

    // convert points to local frame
    size_t n_waypoints = ptsx.size();
    auto ptsx_local = Eigen::VectorXd(n_waypoints);
    auto ptsy_local = Eigen::VectorXd(n_waypoints);
    for(size_t i=0; i<n_waypoints; i++) {
      double dX = ptsx[i] - px;
      double dY = ptsy[i] - py;
      ptsx_local(i) = dX * cos(psi) + dY * sin(psi);
      ptsy_local(i) = -dX * sin(psi) + dY * cos(psi);
    }
    // polynomial to fit points - 3rd order
    auto coeffs = polyfit(ptsx_local, ptsy_local, 3);

    // initial state (in car coords)
    const double x0 = 0;
    const double y0 = 0;
    const double psi0 = 0;
    const double cte0 = coeffs[0];
    const double epsi0 = -atan(coeffs[1]); // psi - atan(f'(x))
    // initial state after delay
    const double delay = 0.1; // 100ms
    double x_delay = x0 + (v * cos(psi0) * delay);
    double y_delay = y0 + (v * sin(psi0) * delay);
    double psi_delay = psi0 - (v / mpc.Lf * delta * delay);
    double v_delay = v + a * delay;
    double cte_delay = cte0 + (v * sin(epsi0) * delay);
    double epsi_delay = epsi0 - (v / mpc.Lf * delta * delay);
    Eigen::VectorXd state(6);
    state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;

    auto vars = mpc.solve(state, coeffs);

    SimOutput output{};
    output.steer_value = vars[0];
    output.throttle_value = vars[1];

    // output mpc
    for ( int i = 2; i < vars.size(); i++ ) {
      if ( i % 2 == 0 ) {
        output.mpc_x_vals.push_back(vars[i]);
      } else {
        output.mpc_y_vals.push_back(vars[i]);
      }
    }

    // output reference line
    for(int i=0; i<100; i+=3) {
      output.next_x_vals.push_back(i);
      output.next_y_vals.push_back(polyeval(coeffs, i));
    }
    return output;
  });
  simulator.run();

  return 0;
}

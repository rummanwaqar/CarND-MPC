#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <cmath>

#include "io.hpp"

const int PORT = 4567;

int main(int argc, char** argv) {
  std::cout << "Connecting to simulator" << std::endl;
  SimIO simulator(PORT, [&](std::vector<double> ptsx, std::vector<double> ptsy,
    double px, double py, double psi, double v) {
    
    return SimOutput{};
  });
  simulator.run();

  return 0;
}

#pragma once

#include <iostream>
#include <string>
#include <thread>
#include <tuple>
#include <functional>
#include <uWS/uWS.h>
#include "json.hpp"

struct SimOutput {
  double steer_value; // radians
  double throttle_value; // -1 to 1
  // Display the MPC predicted trajectory
  std::vector<double> mpc_x_vals;
  std::vector<double> mpc_y_vals;
  // Display the waypoints/reference line
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
};

/**
 * callback function definition
 * @return tuple(steering, throttle, mpc_x, mpc_y, next_x, next_y)
 */
typedef std::function< SimOutput(std::vector<double> ptsx, std::vector<double> ptsy,
  double px, double py, double psi, double v) > ProcessCb;

/*
 * Interface to simulator
 */
class SimIO {
public:
  /*
   * Constructor
   * Creates uWebSocket object and defines all event handlers
   * @param port - port number for simulator uWebSocket
   * @param cb callback for processing function
   */
  SimIO(int port, ProcessCb cb);

  /*
   * Destructor
   */
  ~SimIO() = default;

  /*
   * Initializes connection to simulator and blocks it until simulator is closed.
   * event handling for simulator
   */
  void run();

private:
  /*
   * Checks if the SocketIO event has JSON data.
   * If there is data the JSON object in string format will be returned,
   * else the empty string "" will be returned.
   */
  std::string hasData(std::string s);

  // uWS object
  uWS::Hub h_;

  // connection port
  int port_;

  // processing callback
  ProcessCb callbackFunc_;
};

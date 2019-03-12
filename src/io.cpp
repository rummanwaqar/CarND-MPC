#include "io.hpp"
#include "helpers.hpp"

SimIO::SimIO(int port, ProcessCb cb) : port_(port), callbackFunc_(cb) {
  /*
   * Register event handlers for uWS
   */
  h_.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      std::string s = hasData(std::string(data));

      if(s != "") { // data available
        // parse json
        auto j = nlohmann::json::parse(s);
        std::string event = j[0].get<std::string>();

        if(event == "telemetry") {
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = mph2ms(j[1]["speed"]);

          auto output = callbackFunc_(ptsx, ptsy, px, py, psi, v);

          nlohmann::json msgJson;
          // steer value has to be between -1 to 1 for divide by 25 degrees in rads
          msgJson["steering_angle"] = output.steer_value/0.436332;
          msgJson["throttle"] = output.throttle_value;
          // vehicle coods frame, green line
          msgJson["mpc_x"] = output.mpc_x_vals;
          msgJson["mpc_y"] = output.mpc_y_vals;
          // vehicles coods frame, yellow line
          msgJson["next_x"] = output.next_x_vals;
          msgJson["next_y"] = output.next_y_vals;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h_.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h_.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
}

void SimIO::run() {
  // listen and wait for connection
  if (h_.listen(port_)) {
    std::cout << "Listening to port " << port_ << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return;
  }
  // endless loop until application exists
  h_.run();
}

std::string SimIO::hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

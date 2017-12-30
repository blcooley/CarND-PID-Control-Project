#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Reset command copied from
// https://carnd.slack.com/archives/C4Z4GFX0X/p1494312027726915?thread_ts=1494276925.735399&cid=C4Z4GFX0X
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws) {
  // reset
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid;
  
  // PID gains

  /*****************************************************
  // Seed gains and deltas for throttle = 0.2
  double K[3] = { 0.0, 0.0, 0.0 };
  double dK[3] = { 0.025, 0.025, 0.025 };

  // Final gains for throttle = 0.2
  double K[3] = { 0.07975537, 0.0, 0.152628 };
  double dK[3] = { 0.0, 0.0 , 0.0 };
  ******************************************************/
  
  /*****************************************************
  // Seed gains and deltas for throttle = 1
  double K[3] = { 0.036, 1.5e-7, 54.9 };
  double dK[3] = { 0.01, 5.0e-8 , 10.0 };

  // Final gains for throttle = 1
  double K[3] = { 0.057, 1.5e-7, 58.1765 };
  double dK[3] = { 0.0, 0.0 , 0.0 };
  ******************************************************/

  /*****************************************************
  // Seed gains for controlled throttle
  double K[3] = { 0.077606, 2.42501e-7, 42.4277 };
  double dK[3] = { 0.02, 5.0e-8 , 5.0 };
  */
  // Final gains for controlled throttle
  double K[3] = { 0.0797815, 2.26261e-7, 42.0205 };
  double dK[3] = { 0.0, 0.0 , 0.0 };
  /******************************************************/
  
  pid.Init(K[0], K[1], K[2]);

  // This controller attempts to maximize throttle but
  // reduces it when CTE is too high.
  
  PID maxCtePid;
  maxCtePid.Init(0.25, 0.001, 0);
  maxCtePid.shouldDebug = false;
  double cteSetpoint = 1.5;
  
  int iteration = 0;
  int accum_start_iter = 0;
  int accum_end_iter = 6000;
  
  double best_err = 1000000.0;
  double accum_err = 0.0;

  int current_gain = 0;
  bool isTwiddling = false;
  bool isIncreasing = true;

  double tol = 0.4;

  double throttle = 0.3;
  double throttle_max = 0.8;
  
  std::cout << "K0,K1,K2,dK0,dk1,dk2,accum_err" << std::endl;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      //std::cout << "iteration: " << iteration << "accum error: " << accum_err << std::endl;

      if (isTwiddling && (dK[0] + dK[1] + dK[2]) < tol) {
	isTwiddling = false;
      }
      
      if (isTwiddling && iteration == 0) {
	if (isIncreasing) {
	  K[current_gain] += dK[current_gain];
	} else {
	  K[current_gain] -= 2 * dK[current_gain];
	}
	pid.Init(K[0], K[1], K[2]);
	std::cout << K[0] << "," << K[1] << "," << K[2] << ","
		  << dK[0] << "," << dK[1] << "," << dK[2] << ",";
	iteration += 1;
      }


      if (isTwiddling && iteration > accum_end_iter) {
	if (accum_err < best_err) {
	  best_err = accum_err;
	  dK[current_gain] *= 1.1;
	  isIncreasing = true;
	  current_gain = (current_gain + 1) % 3;
	} else if (isIncreasing) {
	  isIncreasing = false;
	} else {
	  K[current_gain] += dK[current_gain];
	  dK[current_gain] *= 0.9;
	  isIncreasing = true;
	  current_gain = (current_gain + 1) % 3;
	}
	std::cout << accum_err << std::endl;
	reset_simulator(ws);
	iteration = 0;
	accum_err = 0.0;
      } else {
	auto s = hasData(std::string(data).substr(0, length));
        if (s != "") {
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            double steer_value;
            /*
            * TODO: Calcuate steering value here, remember the steering value is
            * [-1, 1].
            * NOTE: Feel free to play around with the throttle and speed. Maybe use
            * another PID controller to control the speed!
            */
	    if (isTwiddling && iteration > accum_start_iter) {
	      accum_err += pow(cte, 2);
	      if (accum_err > best_err) {
		//		iteration = accum_end_iter;
	      }
	    }

	    pid.UpdateError(cte);
	    steer_value = pid.TotalError() / -deg2rad(25);
	    if (steer_value > 1.0) { steer_value = 1.0; }
	    if (steer_value < -1.0) { steer_value = -1.0; }

	    maxCtePid.UpdateError(cteSetpoint - abs(cte));
	    throttle += maxCtePid.TotalError();

	    if (throttle > throttle_max) {
	      throttle = throttle_max;
	      maxCtePid.isWoundUpHigh = true;
	    } else {
	      maxCtePid.isWoundUpHigh = false;
	    }

	    if (throttle < -throttle_max) {
	      throttle = -throttle_max;
	      maxCtePid.isWoundUpLow = true;
	    } else {
	      maxCtePid.isWoundUpLow = false;
	    }

	    //	    std::cout << "CTE: " << cte << "   throttle: " << throttle << std::endl;

            // DEBUG
	    //            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

	    iteration += 1;
          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      //    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    //    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    //    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    //    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

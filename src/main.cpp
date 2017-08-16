#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

void transformGlobal2Vehicle(double vehicle_px, double vehicle_py, double vehicle_psi, vector<double> global_x, vector<double> global_y, vector<double> & vehicle_x, vector<double> & vehicle_y){
	// transforms a list of coordinates from global coordinates to vehicle coordinates
	// Inputs:
	// vehicle_px: vehicle x-coordinate in global coordinate system
	// vehicle_py: vehicle y-coordinate in global coordinate system
	// vehicle_psi: vehicle orientation, angle in radians counterclockwise from x-axis of map coordinate system
	// global_x: list of point x-coordinates in global coordinate system
	// global_y: list of point y-coordinates in global coordinate system
	// vehicle_x: list of point x-coordinates in vehicle coordinate system
	// vehicle_y: list of point y-coordinates in vehicle coordinate system

	// setting transformation parameter
	double trans_x = -1 * vehicle_px;
	double trans_y = -1 * vehicle_py;
	double trans_angle = -1 * vehicle_psi;

	// check sizes
	if(global_x.size() != global_y.size()) {
		cout << "error in transformCoordsGlobal2Vehicle: sizes of global_x != global_y" << endl;
		exit(1);
	}
	if(global_x.size() == 0) {
		cout << "error in transformCoordsGlobal2Vehicle: sizes of global_x and global_y are 0" << endl;
		exit(1);
	}

	// transform coordinates
	for(size_t i = 0; i < global_x.size(); i++) {
		vehicle_x.push_back((global_x[i] + trans_x) * cos(trans_angle) - (global_y[i] + trans_y) * sin(trans_angle));
		vehicle_y.push_back((global_x[i] + trans_x) * sin(trans_angle) + (global_y[i] + trans_y) * cos(trans_angle));
	}

    // cout << "--------------" << endl;
    // for(size_t i = 0; i < vehicle_x.size(); i++) {
	   //  cout << i << ":vehicle_ptsx=" << vehicle_x[i] << ", vehicle_ptsy=" << vehicle_y[i] << endl;
    // }
    // cout << "--------------" << endl;

}

// vector<double> transformCoordsVehicle2Map(double vehicle_x, double vehicle_y, double vehicle_theta, double & lm_map_x, double & lm_map_y, double lm_veh_x, double lm_veh_y ){
// 	// transforms a landmark from map coordinates to vehicle coordinates
// 	// Inputs:
// 	// vehicle_x: vehicle x-coordinate in map coordinate system
// 	// vehicle_y: vehicle y-coordinate in map coordinate system
// 	// vehicle_theta: vehicle heading, angle in radians counterclockwise from x-axis of map coordinate system
// 	// lm_map_x: landmark x-coordinate in map coordinate system. this value gets overidden
// 	// lm_map_y: landmark x-coordinate in map coordinate system. this value gets overidden
// 	// lm_veh_x: landmark x-coordinate in vehicle coordinate system. 
// 	// lm_veh_y: landmark x-coordinate in vehicle coordinate system.

// 	// setting transformation parameter
// 	double trans_x = vehicle_x;
// 	double trans_y = vehicle_y;
// 	double trans_angle = vehicle_theta;

// 	// set transformed x, y
// 	lm_map_x = lm_veh_x * cos(trans_angle) - lm_veh_y * sin(trans_angle) + trans_x;
// 	lm_map_y = lm_veh_x * sin(trans_angle) + lm_veh_y * cos(trans_angle) + trans_y;
// }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
//          Eigen::VectorXd ptsx = j[1]["ptsx"];
//          Eigen::VectorXd ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          cout << "--------------" << endl;
          for(size_t i = 0; i < ptsx.size(); i++) {
            cout << i << ":ptsx=" << ptsx[i] << ", ptsy=" << ptsy[i] << endl;
          }

          cout << "px=" << px << endl;
          cout << "py=" << py << endl;
          cout << "psi=" << psi << endl;
          cout << "psi_unity=" << psi_unity << endl;
          cout << "v=" << v << endl;

          cout << "--------------" << endl;


          // convert std::vector to Eigen::VectorXd
          double* ptr_x = &ptsx[0];
          double* ptr_y = &ptsy[0];

          Eigen::Map<Eigen::VectorXd> ptsx_E(ptr_x, ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_E(ptr_y, ptsy.size());

          // calc polyfit coeffs with polynom 3rd degree
          Eigen::VectorXd coeffs = polyfit(ptsx_E, ptsy_E, 3);

          // calc some useful points
          double ref_x0 = px;
          double ref_y0 = polyeval(coeffs, ref_x0);
          double ref_x1 = px+1;
          double ref_y1 = polyeval(coeffs, ref_x1);

          // calc cross_track_error: cte = py - y_value_of_refence_line_with_same_x
          double cte = py - ref_y0;

          // calc orientation_error: epsi = orientation - optimal_orientation_when_vehicle_would_be_on_reference_line
          double ref_psi = atan( (ref_y1 - ref_y0) / (ref_x1 - ref_x0) );
    	  //angle normalization
          while (ref_psi >  2*M_PI) ref_psi -= 2.*M_PI;
          while (ref_psi < 0)      ref_psi += 2.*M_PI;
          cout << "psi=" << psi << " ref_psi=" << ref_psi << endl;


          double epsi = ref_psi -psi;

          // create state
          Eigen::VectorXd state = Eigen::VectorXd(6);
          state << px, py, psi, v, cte, epsi;

          vector<double> result = mpc.Solve(state, coeffs);


          double steer_value;
          double throttle_value;
          // steer_value = steer_throttle[0]  / deg2rad(25);
          // throttle_value = steer_throttle[1];
          steer_value = -result[0] / deg2rad(25);
          throttle_value = result[1];

          // cout << "steer=" << steer_value << "    throttle=" << throttle_value << endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals_global;
          vector<double> mpc_y_vals_global;

          for(size_t i = 2; i < result.size(); i++) {
          	if(i%2 == 0) {
          		mpc_x_vals_global.push_back(result[i]);
          	}
          	else {
          		mpc_y_vals_global.push_back(result[i]);
          	}
          }

          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          transformGlobal2Vehicle(px, py, psi, mpc_x_vals_global, mpc_y_vals_global, mpc_x_vals, mpc_y_vals );

          // for(size_t i=0; i < mpc_x_vals.size(); i++) {
          // 	cout << "trajectory: x=" << mpc_x_vals[i] << ", y=" << mpc_y_vals[i] << endl;
          // }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          vector<double> next_x_vals;
          next_x_vals.reserve(ptsx.size());
          vector<double> next_y_vals;
          next_y_vals.reserve(ptsy.size());
          transformGlobal2Vehicle(px, py, psi, ptsx, ptsy, next_x_vals, next_y_vals );

          // erase the first coordinate to improve visualization
          next_x_vals.erase(next_x_vals.begin());
          next_y_vals.erase(next_y_vals.begin());

          //Display the waypoints/reference line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          // for(size_t i=0; i < next_x_vals.size(); i++) {
          // 	cout << "reference: x=" << next_x_vals[i] << ", y=" << next_y_vals[i] << endl;
          // }


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

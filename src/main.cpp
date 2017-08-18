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

}

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
		//cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];
					double delta = j[1]["steering_angle"];
					double a = j[1]["throttle"];

					// transform the reference line from global to vehicle coordinate system
					vector<double> vehicle_ptsx;
					vector<double> vehicle_ptsy;
					transformGlobal2Vehicle(px, py, psi, ptsx, ptsy, vehicle_ptsx, vehicle_ptsy );

					// convert std::vector to Eigen::VectorXd
					double* ptr_x = &vehicle_ptsx[0];
					double* ptr_y = &vehicle_ptsy[0];

					Eigen::Map<Eigen::VectorXd> vehicle_ptsx_E(ptr_x, vehicle_ptsx.size());
					Eigen::Map<Eigen::VectorXd> vehicle_ptsy_E(ptr_y, vehicle_ptsy.size());

					// calc polyfit coeffs with polynom 3rd degree
					Eigen::VectorXd coeffs = polyfit(vehicle_ptsx_E, vehicle_ptsy_E, 3);

					// calc cross_track_error: cte = distance_from_refernce_line
					// i approximate:
					double cte = polyeval(coeffs, 0);

					// calc orientation_error: epsi = orientation - optimal_orientation_when_vehicle_would_be_on_reference_line
					//double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px, 2));
					// because psi=0 and px=0, this could be simplified to:
					double epsi = atan(coeffs[1]);

					// set Lf
					double Lf = 2.67;

					// set latency
					double dt_latency = 0.1;

					// copy telemetry state...
					double x0 = 0;
					double y0 = 0;
					double psi0 = 0;
					double v0 = v;

					// ... and adjust for latency (the state after dt_latency)
					x0 += v * cos(delta) * dt_latency;
					y0 += v * sin(delta) * dt_latency;
					psi0 += v * delta/Lf * dt_latency;
					v0 += a * dt_latency;
					cte += v * sin(delta) * dt_latency;
					epsi += v * delta / Lf * dt_latency;

					// create state
					Eigen::VectorXd state(6);
					// we need the state also in vehicle coordinate system
					// vehicle_px = 0, vehicle_py=0, vehicle_psi=0 (of course!)
					// state = {px, py, psi, v, cte, epsi}
					state << 0, 0, 0, v, cte, epsi;

					// solve the optimation
					// result = {steering_angle, acceleration, x_coordinates_of_trajectory_in_vehicle_COS, y_coordinates_of_trajectory_in_vehicle_COS}
					vector<double> result = mpc.Solve(state, coeffs);

					double steer_value = -result[0]; // Steering angle is negative in rotated coordinates
					double throttle_value =	result[1];

					json msgJson;
					// NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
					// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle_value;

					//Display the MPC predicted trajectory 
					vector<double> mpc_x_vals;
					vector<double> mpc_y_vals;
					for(size_t i = 2; i < result.size(); i++) {
						if(i%2 == 0) {
							mpc_x_vals.push_back(result[i]);
						}
						else {
							mpc_y_vals.push_back(result[i]);
						}
					}

					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;

					//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Yellow line

					//Display the waypoints/reference line
					msgJson["next_x"] = vehicle_ptsx;
					msgJson["next_y"] = vehicle_ptsy;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING.
					this_thread::sleep_for(chrono::milliseconds(int(dt_latency*1000)));
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

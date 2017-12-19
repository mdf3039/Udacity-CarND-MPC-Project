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
          std::cout<<"Number of waypointsX: "<<ptsx.size()<<std::endl;
          double minX = *std::min_element(ptsx.begin(), ptsx.end());
          double maxX = *std::max_element(ptsx.begin(), ptsx.end());
          std::cout<<"Minimum of waypointsX: "<<minX<<std::endl;
          std::cout<<"Maximum of waypointsX: "<<maxX<<std::endl;
          std::cout<<"Number of waypointsY: "<<ptsy.size()<<std::endl;
          double minY = *std::min_element(ptsy.begin(), ptsy.end());
          double maxY = *std::max_element(ptsy.begin(), ptsy.end());
          std::cout<<"Minimum of waypointsY: "<<minY<<std::endl;
          std::cout<<"Maximum of waypointsY: "<<maxY<<std::endl;
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          //transform the coordinates in ptsx and ptsy using px,py and psi.
          for (size_t i = 0; i < ptsx.size(); i++){
            double point_x = ptsx[i] - px;
            double point_y = ptsy[i] - py;
            ptsx[i] = cos(psi)*point_x + sin(psi)*point_y;
            ptsy[i] = cos(psi)*point_y - sin(psi)*point_x;
          }
          //print out new ptsx and ptsy
          std::cout<<"ptsx: ";
          std::copy(ptsx.begin(), ptsx.end(), std::ostream_iterator<double>(std::cout, ", "));
          std::cout<<" "<<endl;
          std::cout<<"ptsy: ";
          std::copy(ptsy.begin(), ptsy.end(), std::ostream_iterator<double>(std::cout, ", "));
          std::cout<<" "<<endl;
          //px, py is now at the origin and psi is 0
          px = 0;
          py = 0;
          psi = 0;
          double v = j[1]["speed"];
          std::cout<<"Car Position: ("<<px<<","<<py<<")"<<std::endl;
          std::cout<<"Psi: "<<psi<<std::endl;
          //convert ptsx and ptsy into eigen vectors
          Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          Eigen::VectorXd yvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());
          //obtain the coefficients from a fitted polynomial to the points
          auto coeffs = polyfit(xvals,yvals,3);
          //obtain the cte, which is the difference between px and polynomial
          double cte = polyeval(coeffs, px)-py;
          //obtain the orientation error. To do this, the derivative of the polynomial
          //must be found. For a 3rd degree polynomial of coefficients [c0,c1,c2,c3],
          //the derivative will be a polynomial [c1,2c2,3c3].
          Eigen::VectorXd der_coeffs(3);
          der_coeffs << coeffs[1],2*coeffs[2],3*coeffs[3];
          //calculate the orientation error using the
          //desired orientation, psi - arctan(f'(x))
          double oe = psi - atan(polyeval(der_coeffs,px));
          //set up state vector
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, oe;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          int N = 200;
          //set the number of samples used based on velocity
          if (v<10){
            N = 200;
          }
          double steer_value;
          double throttle_value;
          //solve the mpc for the next state
          auto mpc_sol = mpc.Solve(state, coeffs);
          //separate the mpc solution. for all x,y,psi,v,cte,oe,delta,acc values
          std::vector<double> est_x = std::vector<double>(mpc_sol.begin()+1 , mpc_sol.begin()+N);
          std::vector<double> est_y = std::vector<double>(mpc_sol.begin()+N+1 , mpc_sol.begin()+2*N);
          std::vector<double> est_psi = std::vector<double>(mpc_sol.begin()+2*N+1 , mpc_sol.begin()+3*N);
          std::vector<double> est_v = std::vector<double>(mpc_sol.begin()+3*N+1 , mpc_sol.begin()+4*N);
          std::vector<double> est_cte = std::vector<double>(mpc_sol.begin()+4*N+1 , mpc_sol.begin()+5*N);
          std::vector<double> est_oe = std::vector<double>(mpc_sol.begin()+5*N+1 , mpc_sol.begin()+6*N);
          std::vector<double> est_delta = std::vector<double>(mpc_sol.begin()+6*N , mpc_sol.begin()+7*N-1);
          std::vector<double> est_a = std::vector<double>(mpc_sol.begin()+7*N-1 , mpc_sol.end());
          /*std::cout<<"Estimations of X: ";
          std::copy(est_x.begin(), est_x.end(), std::ostream_iterator<double>(std::cout, ", "));
          std::cout<<" "<<endl;*/
          //use the 6th and 7th values as the steer and throttle values
          steer_value = 0.05;//est_delta[0];
          throttle_value = 0.5;//est_a[0];
          std::cout<<"Steer Value: "<<steer_value<<endl;
          std::cout<<"Throttle Value: "<<throttle_value<<endl;
          std::cout<<"Steer Value/25deg: "<<steer_value/deg2rad(25)<<endl;


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals = est_x;
          vector<double> mpc_y_vals = est_y;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = ptsx;
          vector<double> next_y_vals = ptsy;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


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

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

// Time interval to compensate for latency
const double latency = 0.2;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) 
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) 
    {
        return "";
    } 
    else if (b1 != string::npos && b2 != string::npos) 
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;    
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) 
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}












int main() 
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) 
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        
        // Uncomment for debugging purposes:
        //cout << sdata << endl;
        
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') 
        {
            string s = hasData(sdata);
            if (s != "") 
            {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                
                if (event == "telemetry") 
                {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];   // units in rads
                    double v = j[1]["speed"];   // unites in mph

                    // Conversion of speed from mph into m/s
                    v = v*0.44704; 


                    double steer_value = 0; // Values in between [-1, 1]
                    double throttle_value; // Values in between [-1, 1]


                    // Waypoints BEGIN
                    // Coordinate transformation World->Car
                    Eigen::MatrixXd rotation = Eigen::MatrixXd(3, 3);
                    rotation << cos(psi), sin(psi), 0,
                                -sin(psi), cos(psi), 0,
                                0, 0, 1;
                                
                    Eigen::MatrixXd translation = Eigen::MatrixXd(3, 3);
                    translation << 1, 0, -px, 
                                   0, 1, -py,
                                   0, 0, 1;

                    Eigen::VectorXd input(3);
                    Eigen::VectorXd output(3);

                    for (int i=0; i< ptsx.size(); ++i)
                    {
                        input << ptsx[i], ptsy[i], 1;
                        output = rotation * translation * input;
                        ptsx[i] = output(0);
                        ptsy[i] = output(1);
                    }

                    std::vector<double> next_x_vals(6);
                    std::vector<double> next_y_vals(6);

                    next_x_vals = ptsx;
                    next_y_vals = ptsy;
                    // Waypoints END


                    Eigen::VectorXd coeffs = Eigen::VectorXd(4);
                    Eigen::Map<Eigen::VectorXd> next_x_vals_Vector(&ptsx[0], 6);
                    Eigen::Map<Eigen::VectorXd> next_y_vals_Vector(&ptsy[0], 6);

                    // Calculate the coefficients of the fitted polynomial
                    // Fit values to pologon of order 3 
                    coeffs = polyfit(next_x_vals_Vector, next_y_vals_Vector, 3);                    



                    // Calculate cte and epsi here:
                    // Using the nearest points in the Car's coordinate system
                    //double cte = -polyeval(coeffs, 0);
                    double cte = coeffs[0]; // polyeval unnecessary
                    double epsi = -atan(coeffs[1]);
                    cout << "cte: " << cte << "\t" << "epsi: " << epsi << endl;

                    // Calculation of predictive trajectory:
                    Eigen::VectorXd state_current = Eigen::VectorXd(6); // current state (Vehicle coordinate system)
                    Eigen::VectorXd state_future = Eigen::VectorXd(6); // state prediction in 100ms (Vehicle coordinate system)

                    // Pose ist zero since we have transformed everything into the car's coordinate system.
                    // => Will be different if we incorporate a latency of 100ms and have to extraploate these values by means of the bicycle model.
                    /*                
                    state_current << 0, 0, v, 0, cte, epsi;
                    std::vector<double> solution = mpc.Solve(state_current, coeffs);                    
                    */

                    /* 
                        Predict state in 100ms using the bicycle model equations below.
                                           
                        x1 = x0 + v0 * cos(psi0) * dt;
                        y1 = y0 + v0 * sin(psi0) * dt;
                        v1 = v0 + a0 * dt;
                        psi1 = psi0 + v0/Lf * delta * dt;
                        cte1 = f(x0) - y0 + (v0 * sin(epsi0) * dt);  // Error!
                        epsi1 = epsi0 + v0/Lf * delta0 * dt
                        
                        f(y) is the fitted polynomial.
                        Since x0, y0, psi0 are all equal to zero, these equations simplify to the ones given below.
                        Notice that we have also assumed the acceleration a to be zero.
                    */

                    
                    double delta0;
                    double x1, y1, v1, psi1, cte1, epsi1;
                    
                    
                    delta0 = -steer_value;
                    x1 = v * latency;
                    y1 = 0;
                    v1 = v;// neglected the term a*latency in order to approximate further
                    psi1 = v/Lf * delta0 * latency;
                    cte1 = coeffs[0] + v * sin(epsi) * latency;
                    epsi1 = epsi + v/Lf * delta0 * latency;

                    state_future << x1, y1, v1, psi1, cte1, epsi1;
                    std::vector<double> solution = mpc.Solve(state_future, coeffs);
                    


                    steer_value = -solution[0];
                    throttle_value = solution[1];

                    // Predicted points
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    // Not really beautiful, but well...
                    for (int i=0; i<solution.size()/2-1; ++i)
                    {
                        mpc_x_vals.push_back(solution[2*i]);
                        mpc_y_vals.push_back(solution[2*i+1]);
                    }



                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    // Why the heck are input and output units different?!? :D
                    steer_value = steer_value / deg2rad(25);
                    cout << "steer_value=" << steer_value << "\t" << "throttle_value=" << throttle_value << endl;

                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;



                    //Display the MPC predicted trajectory 
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line





                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    

                    // Uncomment to view JSON-payload
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
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } 
            else 
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });


    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) 
    {
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

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) 
    {
        std::cout << "Listening to port " << port << std::endl;
    } 
    else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}

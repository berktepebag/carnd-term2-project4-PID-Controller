#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <array>

// for convenience
using json = nlohmann::json;

using namespace std;

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

bool firstRun = true;
double best_error;
double dp_p, dp_i, dp_d;
int counter = 0;
int counter_limiter = 3000;
double throttle = 0.3;
bool evaluate = false;

array<double, 3> p,dp;

double total_cte;
double best_cte;
double working_good_counter,not_working_good_counter;

double turn = 0;

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  //pid.Init(0.1,0.001,0.9);   
  pid.Init(0.1,0.001,0.8);   

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
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
                   
          if (firstRun)
          {
            best_cte = 1000000;
            firstRun = false;                    
          }     
          //*************************************************************//
          //*****Control the Kp,Ki,Kd and throtthle according to CTE.****//

          //Slow down, car is not stable.
          if (fabs(cte) > 0.5 && speed > 15)
          {
            throttle = 0.1;
            pid.Init(0.1,0.001,0.8);
          }
          //Car is stable enough, increase speed.
          else if(fabs(cte) >= 0.25 && fabs(cte) < 0.5 && speed > 15){
            throttle = 0.3;
            pid.Init(0.075,0.001,0.8);
          }
          //Car is doing fine but bit slow, increase the speed bit more.
          else if(fabs(cte) < 0.25){
            throttle = 1.0;            
          }
          //Especially at low speeds, increase speed fastly.   
          else if(speed < 10){
            throttle = 3.0;
          }
          //*************************************************************//
          //*****Control the Kp,Ki,Kd and throtthle according to CTE.****//

          cout << "counter/counter_limiter: " << counter << " / " << counter_limiter <<endl;

          //Twiddle only for Kp_. After 1 full trip around track (counter ~ 10000), Kp_ settles around 0.1. 
          //Start with short distances and increase as Kp_ changes, until the track is completed.
          //Once the track is completed, Kp_ will be suitable for all the scenarios.
          if (evaluate)
          {     
        
            if (counter < counter_limiter)
            {
              pid.UpdateError(cte);
              steer_value = pid.TotalError();
              counter++;
              total_cte += fabs(cte);
              cout << "turn: " << turn << endl;
              cout << "working good for: " << working_good_counter << endl;
              cout << "not working good for: " << not_working_good_counter << endl;
              cout << "best_cte: " << best_cte << endl;
              cout << "current cte mean: " << (total_cte/counter) << endl;
              cout << "pid.Kp_: " << pid.Kp_ << endl;
            }
            else{
              cout << "*********************************"<<endl;
              cout << "cte mean: " << (total_cte/counter) << endl;
              if ((total_cte/counter) < best_cte)
              {              
                best_cte = total_cte/counter;
                working_good_counter++;    

                //If car is doing better with this Kp_, increase it.
                if (working_good_counter == 2)
                {
                  pid.Kp_ += pid.Kp_* 0.1;    
                  working_good_counter = 0;
                  not_working_good_counter = 0;
                  counter_limiter *= 1.5;                 
                }
              }
              //If car is NOT doing better with this Kp_, decrease it.
              else if(not_working_good_counter == 2){
                pid.Kp_ -= pid.Kp_* 0.12;      
                working_good_counter = 0;
                not_working_good_counter = 0;
                counter_limiter *= 1.2;
              }          
              else{
                not_working_good_counter++;
              }
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              counter = 0;
              total_cte = 0;
              turn++;
            }
          }
          else{
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
          }

          counter++; 

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;         
          //msgJson["throttle"] = 0.3;          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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

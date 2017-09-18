#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <algorithm>

using namespace std;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
	
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
//converting mph to inc
double mphtoinc(double mph) {

    return mph * 0.02 * 0.44704;
}
//converting ms to inc
double mstoinc(double ms) {

    return ms * 0.02;
}
//converting mph to ms
double mph_to_ms(double mph) {
    
    return mph * 0.44704;
}
//converting ms to mph
double ms_to_mph(double ms) {
    
    return ms * 2.23694;
}
//function to constently changing dist_inc to approach given speed limit
double set_speed(double desired, double pre_speed) {
    //takes the difference in current speed to desired speed and increases dist_inc if error is negative and decreases dist_inc if error positive
    double error = desired - pre_speed;
    if(fabs(error) < 0.01){
        error = 0;
    }
    return mphtoinc(pre_speed+error);
}
//same as set_speed just for meters per second
double set_speedms(double desired, double pre_speed) {
    
    double desired_mph = desired * 2.23694;
    double error = desired_mph - pre_speed;
    if(fabs(error) < 0.01){
        error = 0;
    }
    return mphtoinc(pre_speed+error);
}

//creating a Path Planning class to handle actions like Lane Cahnges and Acceleration and Deacceleration
class PathPlanning {
    
private:
    //creating a speed changing status with acceleration and deacceleration
    enum SPEED_CHANGE {ACCELERATE, DEACCELERATE};
    SPEED_CHANGE speedchange;
    //creating a lane changing status with left and right turn
    enum LANE_CHANGE {CHANGE_LEFT, CHANGE_RIGHT};
    LANE_CHANGE	lanechange;
    //progress
    double progress;
    
public:
    PathPlanning();
    ~PathPlanning() {};
    //bool operation to determine each action and void operation to execute actions
    bool IsNoChange();
    //speed change with desired goal inc as input
    bool IsChangeSpeed();
    void SetChangeSpeed(double goal_inc);
    void ChangeSpeed();
    //lane change with desired lane and desired speed (to accelerate back to ca. 50 mph)
    bool IsChangeLane();
    void SetChangeLane(double goal_lane, double carspeed);
    void ChangeLane();
    //all possible actions available
    enum STATUS {CHANGE_SPEED, CHANGE_LANE, NO_CHANGE};
    STATUS status;
    //initializing parameters
    double dist_inc;
    double start_inc;
    double goal_inc;
    double start_lane;
    double goal_lane;
    double carspeed;
    double lane;
    double diff;
    double pi = M_PI;
};

PathPlanning::PathPlanning() {
    
    status = CHANGE_SPEED;
    status = NO_CHANGE;
}
//bool operations to determine which action to execute
bool PathPlanning::IsNoChange() {
    
    return status == NO_CHANGE;
}

bool PathPlanning::IsChangeSpeed() {
    
    return status == CHANGE_SPEED;
}
//executing determined action
void PathPlanning::SetChangeSpeed(double goal_inc) {
    
    status = CHANGE_SPEED;
    this->goal_inc = goal_inc;
    progress = 0.0001;
    start_inc = dist_inc;
    //checking if desired speed is lower or higher than current speed
    if (goal_inc > start_inc) {
        //calculating the differnce in speed
        speedchange = ACCELERATE;
        diff = (goal_inc - start_inc);
    }
    else {
        //calculating the differnce in speed
        speedchange = DEACCELERATE;
        diff = (start_inc - goal_inc);
    }
}
//changing speed function
void PathPlanning::ChangeSpeed() {
    //checking if car should accelerate or deaccelerate or not change. And changing dist_inc smoothly to desired value
    if ((speedchange == ACCELERATE) and (progress < 100.0)) {
        
        progress += 1/(diff*6);
        dist_inc += 0.0015;
    }
    else if ((speedchange == DEACCELERATE) and (progress < 100.0)) {
        
        progress += 1/(diff*6);
        dist_inc -= 0.002;
    }
    else {
        
        status = NO_CHANGE;
    }
}
//bool operations to determine which action to execute
bool PathPlanning::IsChangeLane() {
    
    return status == CHANGE_LANE;
}
//executing determined action
void PathPlanning::SetChangeLane(double goal_lane, double carspeed) {
    
    status = CHANGE_LANE;
    this->goal_lane = goal_lane;
    this->carspeed = carspeed;
    progress = 0.0001;
    start_lane = lane;
    //checking if desired d is lower or higher than current d
    if (goal_lane > start_lane) {
        //calculating the differnce in d
        lanechange = CHANGE_RIGHT;
    }
    else {
        //calculating the differnce in d
        lanechange = CHANGE_LEFT;
    }
}

void PathPlanning::ChangeLane() {
    //checking if car should change right or change left or not change. And steering smoothly to desired lane
    if ((lanechange == CHANGE_RIGHT) and (progress < 1.0) and (lane <= goal_lane)) {

        progress += 0.008;
        lane = start_lane + (2 * (1 - cos(progress * pi)));
        if (dist_inc < set_speed(46.0, carspeed)) {
            dist_inc += 0.001;
        }
    }
    else if ((lanechange == CHANGE_LEFT) and (progress < 1.0) and (lane >= goal_lane)) {
        
        progress += 0.008;
        lane = start_lane - (2 * (1 - cos(progress * pi)));
        if (dist_inc < set_speed(46.0, carspeed)) {
            dist_inc += 0.001;
        }
    }
    else {
        
        status = NO_CHANGE;
    }
}

int main() {
    uWS::Hub h;
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
    vector<double> map_waypoints_angle;
    
    // Waypoint map to read from
    string map_file_ = "./data/highway_map.csv";
    // // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;
    
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    
    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
        map_waypoints_angle.push_back(pi()/2 + atan2(d_y,d_x));
    }
    //adding some extra waypoints to the end for a smoother transition between laps
    for (int i = 0; i < 10; ++i) {
        
        map_waypoints_x.push_back(map_waypoints_x[i]);
        map_waypoints_y.push_back(map_waypoints_y[i]);
        map_waypoints_s.push_back(max_s+map_waypoints_s[i]);
        map_waypoints_dx.push_back(map_waypoints_dx[i]);
        map_waypoints_dy.push_back(map_waypoints_dy[i]);
    }
    //creating splines
    tk::spline sx;
    tk::spline sy;
    tk::spline sdx;
    tk::spline sdy;
    
    sx.set_points(map_waypoints_s, map_waypoints_x);
    sy.set_points(map_waypoints_s, map_waypoints_y);
    sdx.set_points(map_waypoints_s, map_waypoints_dx);
    sdy.set_points(map_waypoints_s, map_waypoints_dy);
    //setting initial parameters for lane position and speed (dist_inc)
    PathPlanning pp;
    pp.dist_inc = 0.00;
    pp.lane = 6.0;
    
    h.onMessage([&pp, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &sx, &sy, &sdx, &sdy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                     uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            
            auto s = hasData(data);
            
            if (s != "") {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    
                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];
                    
                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];
                    
                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    
                    // The max s value before wrapping around the track back to 0
                    double max_s = 6945.554;
                    
                    json msgJson;
                    
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    //evaluating in which lane the car is currently
                    enum LANE_ID { LEFT, MID, RIGHT };
                    LANE_ID car_lane;
                    if (car_d < 3.0) {
                        
                        car_lane = LEFT;
                    }
                    else if ((car_d >= 3.0) and (car_d < 9.0)) {
                        
                        car_lane = MID;
                    }
                    else if (car_d > 9.0) {
                        
                        car_lane = RIGHT;
                    }
                    //creating vectors to sort different information
                    vector<double> leftlane_infront;
                    vector<double> midlane_infront;
                    vector<double> rightlane_infront;
                    vector<double> rightlane_infront_dist;
                    vector<double> midlane_infront_dist;
                    vector<double> leftlane_infront_dist;
                    
                    vector<double> leftlane_behind;
                    vector<double> midlane_behind;
                    vector<double> rightlane_behind;
                    vector<double> leftlane_behind_dist;
                    vector<double> midlane_behind_dist;
                    vector<double> rightlane_behind_dist;
                    //asigning each value to a category and calucaling speed and distance of the other cars with given sensor information
                    for (int i = 0; i < sensor_fusion.size(); ++i) {
                        
                        auto ai = sensor_fusion[i];
                        double ai_id = ai[0];
                        double ai_x = ai[1];
                        double ai_y = ai[2];
                        double ai_vx = ai[3];
                        double ai_vy = ai[4];
                        double ai_s  = ai[5];
                        double ai_d  = ai[6];
                        double ai_v  = sqrt(ai_vx*ai_vx + ai_vy*ai_vy);
                        double ai_distance = distance(car_x, car_y, ai_x, ai_y);
                        
                        //only considering cars within range of 40 meters
                        if (ai_distance < 40.0) {
                            //sorting by lane and if car is infront or behind my car
                            if ((ai_d < 5.0)) {
                                //for left lane
                                if (ai_s > car_s) {
                                    
                                    leftlane_infront.push_back(ai_v);
                                    leftlane_infront_dist.push_back(ai_distance);
                                }
                                else {
                                    
                                    leftlane_behind.push_back(ai_v);
                                    leftlane_behind_dist.push_back(ai_distance);
                                }
                            }
                            else if ((ai_d >= 3.0) and (ai_d < 9.0) ) {
                                //for middle lane
                                if (ai_s > car_s) {
                                    
                                    midlane_infront.push_back(ai_v);
                                    midlane_infront_dist.push_back(ai_distance);
                                }
                                else {
                                    
                                    midlane_behind.push_back(ai_v);
                                    midlane_behind_dist.push_back(ai_distance);
                                }
                            }
                            else if ((ai_d > 7.0)) {
                                //for right lane
                                if (ai_s > car_s) {
                                    
                                    rightlane_infront.push_back(ai_v);
                                    rightlane_infront_dist.push_back(ai_distance);
                                }
                                else {
                                    
                                    rightlane_behind.push_back(ai_v);
                                    rightlane_behind_dist.push_back(ai_distance);
                                }
                            }
                        }
                    }
                    //just to see the progress around the track
                    cout << "Track completed: " << (float)(int)(100*(100*(car_s/max_s)))/100 << "%" << endl;
                    //just to display the car's postion and where other cars are
                    if (car_lane == RIGHT) {
                        
                        cout << endl;
                        cout << "| " << leftlane_infront.size();
                        cout << " | " << midlane_infront.size();
                        cout << " | " << rightlane_infront.size() << " |" << endl;
                        cout << "|   |" << "   " << "|Car|" << endl;
                        cout << "| " << leftlane_behind.size();
                        cout << " | " << midlane_behind.size();
                        cout << " | " << rightlane_behind.size() << " |" << endl << endl;
                    }
                    else if (car_lane == MID) {
                        
                        cout << endl;
                        cout << "| " << leftlane_infront.size();
                        cout << " | " << midlane_infront.size();
                        cout << " | " << rightlane_infront.size() << " |" << endl;
                        cout << "|   |" << "Car" << "|   |" << endl;
                        cout << "| " << leftlane_behind.size();
                        cout << " | " << midlane_behind.size();
                        cout << " | " << rightlane_behind.size() << " |" << endl << endl;
                    }
                    else {
                        cout << endl;
                        cout << "| " << leftlane_infront.size();
                        cout << " | " << midlane_infront.size();
                        cout << " | " << rightlane_infront.size() << " |" << endl;
                        cout << "|Car|" << "   " << "|   |" << endl;
                        cout << "| " << leftlane_behind.size();
                        cout << " | " << midlane_behind.size();
                        cout << " | " << rightlane_behind.size() << " |" << endl << endl;
                    }
                    
                    double pos_x;
                    double pos_y;
                    double pos_s;
                    //creating the trajectory with pervious path points if existent
                    for(int i = 0; i < previous_path_x.size(); i++) {
                        
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    if(previous_path_x.size() == 0) {
                        
                        pos_x = car_x;
                        pos_y = car_y;
                        pos_s = car_s;
                    }
                    else {
                        
                        pos_x = previous_path_x[previous_path_x.size()-1];
                        pos_y = previous_path_y[previous_path_x.size()-1];
                        
                        double pos_x2 = previous_path_x[previous_path_x.size()-2];
                        double pos_y2 = previous_path_y[previous_path_x.size()-2];
                    }
                    //waypoints created with Frenet s and x,y,dx,dy
                    double w_sx, w_sy, w_sdx, w_sdy;
                    
                    for(int i = 0; i < 50-previous_path_x.size(); i++) {
                        //checking if speed has to be changed and if so change it. The same applies for lane changes
                        if (pp.IsChangeSpeed()) {
                            
                            pp.ChangeSpeed();
                        }
                        else if (pp.IsChangeLane()) {
                            
                            pp.ChangeLane();
                        }
                        else {
                            //checking if car is in left lane and assigning 46mph speed target to car if no other car is in front
                            if(car_lane == LEFT) {
                                
                                if (leftlane_infront.size() == 0) {
                                    
                                    if (pp.dist_inc < set_speed(46.0, car_speed)) {
                                        
                                        pp.SetChangeSpeed(set_speed(46.0, car_speed));
                                    }
                                }
                                else {
                                    //if there is a car infront check for other lane/lanes and accelerate back to traget speed if possible
                                    if ((midlane_infront.size() == 0) and (midlane_behind.size() == 0)) {
                                        
                                        pp.SetChangeLane(6.0, car_speed);
                                    }//also change lane if there is no car infront in the target lane plus cars in the target lane have to be 20 meters away and not travel 2m/s faster than own car
                                    else if ((midlane_infront.size() == 0) and (*min_element(midlane_behind_dist.begin(), midlane_behind_dist.end()) > 20.0) and (midlane_behind[std::distance(midlane_behind_dist.begin(),min_element(midlane_behind_dist.begin(), midlane_behind_dist.end()))] - mph_to_ms(car_speed)) < 2.0) {
                                        
                                        pp.SetChangeLane(6.0, car_speed);
                                    }//if there is no way to change lane the car travels at the speed of the car right infront of it
                                    else if (leftlane_infront.size() > 0) {
                                        
                                        double min_dist = *min_element(leftlane_infront_dist.begin(), leftlane_infront_dist.end());
                                        int min_dist_n = std::distance(leftlane_infront_dist.begin(), min_element(leftlane_infront_dist.begin(), leftlane_infront_dist.end()));
                                        //if distance to car infront is to low, speed gets reduces by 1.5 m/s until distance is back to normal
                                        if (min_dist < 20.0) {
                                            
                                            pp.SetChangeSpeed(set_speedms(leftlane_infront[min_dist_n]-1.5, car_speed));
                                        }//if distance is to big, speed get increased until distance is back to normal
                                        else if (min_dist > 40.0) {
                                            
                                            pp.SetChangeSpeed(set_speedms(leftlane_infront[min_dist_n]+1, car_speed));
                                        }//ideal scenario where own car holds perfect distance to car infront
                                        else {
                                            
                                            pp.SetChangeSpeed(set_speedms(leftlane_infront[min_dist_n], car_speed));
                                        }
                                    }
                                }        ////////////  SAME FOR ALL OTHER LANES, SEE COMMENTARY ABOVE ////////////
                            }
                            else if (car_lane == MID) {
                                
                                if (midlane_infront.size() == 0) {
                                    
                                    if (pp.dist_inc < set_speed(46.0, car_speed)) {
                                        
                                        pp.SetChangeSpeed(set_speed(46.0, car_speed));
                                    }
                                }
                                else {
                                    
                                    if ((leftlane_infront.size() == 0) and (leftlane_behind.size() == 0)) {
                                        
                                        pp.SetChangeLane(2.0, car_speed);
                                    }
                                    else if ((leftlane_infront.size() == 0) and (*min_element(leftlane_behind_dist.begin(), leftlane_behind_dist.end()) > 20.0) and (leftlane_behind[std::distance(leftlane_behind_dist.begin(),min_element(leftlane_behind_dist.begin(), leftlane_behind_dist.end()))] - mph_to_ms(car_speed)) < 2.0) {
                                        
                                        pp.SetChangeLane(2.0, car_speed);
                                    }
                                    else if ((rightlane_infront.size() == 0) and (rightlane_behind.size() == 0)) {
                                        
                                        pp.SetChangeLane(10.0, car_speed);
                                    }
                                    else if ((rightlane_infront.size() == 0) and (*min_element(rightlane_behind_dist.begin(), rightlane_behind_dist.end()) > 20.0) and (rightlane_behind[std::distance(rightlane_behind_dist.begin(),min_element(rightlane_behind_dist.begin(), rightlane_behind_dist.end()))] - mph_to_ms(car_speed)) < 2.0) {
                                        
                                        pp.SetChangeLane(10.0, car_speed);
                                    }
                                    else if (midlane_infront.size() > 0) {
                                        
                                        double min_dist = *min_element(midlane_infront_dist.begin(), midlane_infront_dist.end());
                                        int min_dist_n = std::distance(midlane_infront_dist.begin(),min_element(midlane_infront_dist.begin(), midlane_infront_dist.end()));
                        
                                        if (min_dist < 20.0) {
                                            
                                            pp.SetChangeSpeed(set_speedms(midlane_infront[min_dist_n]-1.5, car_speed));
                                        }
                                        else if (min_dist > 40.0) {
                                            
                                            pp.SetChangeSpeed(set_speedms(midlane_infront[min_dist_n]+1, car_speed));
                                        }
                                        else {
                                            
                                            pp.SetChangeSpeed(set_speedms(midlane_infront[min_dist_n], car_speed));
                                        }
                                    }
                                }
                            }
                            else if (car_lane == RIGHT) {
                                
                                if (rightlane_infront.size() == 0) {
                                    
                                    if (pp.dist_inc < set_speed(46.0, car_speed)) {
                                        
                                        pp.SetChangeSpeed(set_speed(46.0, car_speed));
                                    }  
                                }
                                else {
                                    
                                    if ((midlane_infront.size() == 0) and (midlane_behind.size() == 0)) {
                                        
                                        pp.SetChangeLane(6.0, car_speed);
                                    }
                                    else if ((midlane_infront.size() == 0) and (*min_element(midlane_behind_dist.begin(), midlane_behind_dist.end()) > 20.0) and (midlane_behind[std::distance(midlane_behind_dist.begin(),min_element(midlane_behind_dist.begin(), midlane_behind_dist.end()))] - mph_to_ms(car_speed)) < 2.0) {
                                        
                                        pp.SetChangeLane(6.0, car_speed);
                                    }
                                    else if (rightlane_infront.size() > 0) {
                                        
                                        double min_dist = *min_element(rightlane_infront_dist.begin(), rightlane_infront_dist.end());
                                        int min_dist_n = std::distance(rightlane_infront_dist.begin(),min_element(rightlane_infront_dist.begin(), rightlane_infront_dist.end()));
                                    
                                        if (min_dist < 20.0) {
                                            
                                            pp.SetChangeSpeed(set_speedms(rightlane_infront[min_dist_n]-1.5, car_speed));
                                        }
                                        else if (min_dist > 40.0) {
                                            
                                            pp.SetChangeSpeed(set_speedms(rightlane_infront[min_dist_n]+1, car_speed));
                                        }
                                        else {
                                            
                                            pp.SetChangeSpeed(set_speedms(rightlane_infront[min_dist_n], car_speed));
                                        }
                                    }
                                }
                            }
                        }
                        //adding dist_inc to the position in s coordinates
                        pos_s += pp.dist_inc;
                        pos_s = fmod(pos_s, max_s);
                        //calculating the functions with position s as x value
                        w_sx = sx(pos_s);
                        w_sy = sy(pos_s);
                        w_sdx = sdx(pos_s);
                        w_sdy = sdy(pos_s);
                        //getting the current lane from pathplanning
                        double lane = pp.lane;
                        //calculating the final positions for the trajectory with the lane d value, sx, sy, sdx, sdy value
                        pos_x = w_sx + lane * w_sdx;
                        pos_y = w_sy + lane * w_sdy;
                        
                        next_x_vals.push_back(pos_x);
                        next_y_vals.push_back(pos_y);
                    }
                    
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    auto msg = "42[\"control\","+ msgJson.dump()+"]";
                    
                    //this_thread::sleep_for(chrono::milliseconds(1000));
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

















































































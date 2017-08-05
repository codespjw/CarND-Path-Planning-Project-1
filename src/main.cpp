#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cassert>
#include <limits>
#include <algorithm>
#include <random>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

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

/**************************Constants and Types*******************/
const double INF = numeric_limits<double>::infinity();

using Points = vector<double>;

// a trajectory is a smooth mapping from one value -> another
using Trajectory = tk::spline;

// data from sensor fusion module for peer cars
using SensorData = vector<double>; //[id, x, y, vx, vy, s, d]

// Self driving car 
struct SelfDrivingCar {
public:

  SelfDrivingCar(double x, double y,
                 double s, double d,
                 double yaw, double speed):
                 x(x), y(y),
                 s(s), d(d),
                 yaw(yaw), 
                 speed(speed) {};

  double x; // map coordinate
  double y;
  double s; // frenet coordinate
  double d; 
  double yaw; // bearing
  double speed; // speed
};


// other cars on the road
struct PeerCar {
public:
  PeerCar(const SensorData & sensor):
    id(sensor[0]), x(sensor[1]), y(sensor[2]),
    vx(sensor[3]), vy(sensor[4]),
    s(sensor[5]), d(sensor[6]) {}

  int id;
  double x; // map coordinate
  double y; 
  double vx; // velocity
  double vy; 
  double s; // frenet coordinate
  double d; 
};

// road map
class Map {
public:

  Map(const Points & x,
      const Points & y,
      const Points & s,
      const Points & dx,
      const Points & dy):
      x(x), y(y), s(s), dx(dx), dy(dy) {

    auto n = x.size();
    assert (x.size() == n);
    assert (y.size() == n);
    assert (s.size() == n);
    assert (dx.size() == n);
    assert (dy.size() == n);
    
    double HALF_LANE = LANE_WIDTH / 2;
    // initialize gitter - adding gitter for numerical stability
    double lower = -0.5;
    double upper = 0.1;
    uniform_real_distribution<double> unif(lower, upper);
    default_random_engine re(random_device{}()); 
    // Calculate trajectory for each lane based on waypoints
    for (int lane = 0; lane <= RIGHTMOST_LANE; ++lane) {
      vector<double> lane_x;
      vector<double> lane_y;
      vector<double> lane_s;
      for (auto i = 0; i < n; ++i) {
        double jitter = -0.2;//unif(re);
        // map to middle of lane
        lane_x.push_back(x[i] + dx[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
        lane_y.push_back(y[i] + dy[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
        lane_s.push_back(s[i]);
      }
      // this is a quick fix to make it work when car goes back to starting point
      // a more realistic solution will be to fit local models segment by segment
      for (auto i = 0; i < n; ++i) {
        double jitter = -0.2;//unif(re);
        lane_x.push_back(x[i] + dx[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
        lane_y.push_back(y[i] + dy[i] * (HALF_LANE + lane*LANE_WIDTH + jitter));
        lane_s.push_back(s[i] + MAX_S);
      }
      Trajectory s2x; s2x.set_points(lane_s, lane_x);
      Trajectory s2y; s2y.set_points(lane_s, lane_y);
      lane_s2x.push_back(s2x);
      lane_s2y.push_back(s2y);
    }

  }
  virtual ~Map() {}

  // find the index of lane given the d coordinate of a car
  // lane starts with index 0 from the leftmost 
  int find_lane(double car_d) const {
    int lane = floor(car_d / LANE_WIDTH);
    // assert ((lane >= 0) and (lane <= RIGHTMOST_LANE));
    // some cars may have invalid d value at the beginning
    lane = max(0, lane);
    lane = min(RIGHTMOST_LANE, lane);
    return lane;
  }

  // find the index of a peer car immediately before self-driving-car
  // on a certain lane
  // return -1 if there is no car found
  int find_front_car_in_lane(const SelfDrivingCar & sdc, 
                             const vector<PeerCar> & peer_cars,
                             int lane) const {
    int found_car = -1;
    double found_dist = INF;
    for (auto i = 0; i < peer_cars.size(); ++i) {
      const PeerCar & car = peer_cars[i];
      auto car_lane = find_lane(car.d);
      if ( (car_lane == lane) and (car.s >= sdc.s) ) {
        double dist = car.s - sdc.s;
        if (dist < found_dist) {
          found_car = i;
          found_dist = dist;
        }
      }
    }
    return found_car;
  }

  // find the index of a peer car immediately after self-driving-car
  // on a certain lane
  // return -1 if there is no car found
  int find_rear_car_in_lane(const SelfDrivingCar & sdc, 
                            const vector<PeerCar> & peer_cars,
                            int lane) const {
    int found_car = -1;
    double found_dist = INF;
    for (auto i = 0; i < peer_cars.size(); ++i) {
      const PeerCar & car = peer_cars[i];
      auto car_lane = find_lane(car.d);
      if ( (car_lane == lane) and (car.s <= sdc.s) ) {
        double dist = sdc.s - car.s;
        if (dist < found_dist) {
          found_car = i;
          found_dist = dist;
        }
      }
    }
    return found_car;
  }

public:
  Points x; // map coordinates
  Points y;
  Points s; // distance from starting point
  // vector orthogonal to road
  Points dx; 
  Points dy;

  const double LANE_WIDTH = 4; /*meters*/
  const int RIGHTMOST_LANE = 2; /*lane 0, 1, 2*/
  const double MAX_S = 6945.554;

  // mapping from (s) -> (x, y) for each lane
  vector<Trajectory> lane_s2x;
  vector<Trajectory> lane_s2y;
};

/**************************Path Planner**************************/
struct Path {
public:
  Path (const Points xs, const Points ys):
    xs(xs), ys(ys) {}
  Path (const Path & rhs): xs(rhs.xs), ys(rhs.ys) {}
  size_t size() const {
    return xs.size();
  }
public:
  Points xs;
  Points ys;
};
class PathPlanner {
public:
  PathPlanner(const Map & map): map(map) {
    in_lane_change = false;
  }
  virtual ~PathPlanner() {}

  // main access point of path planning
  Path plan(const Path & previous_path,
            const SelfDrivingCar & sdc,
            const vector<PeerCar> &peers) {

    // 1. update previous s path by removing what has been consumed by controller
    auto n_consumed = previous_s_path.size() - previous_path.size();
    previous_s_path.erase(previous_s_path.begin(), previous_s_path.begin() + n_consumed);

    // 2. if in the middle of lane changing, finish it first
    if (in_lane_change) {
      // reserve the last part for next move
      if (previous_path.size() <= PATH_LEN) { 
        // it is finished!
        in_lane_change = false;
      }
      // continue lane changing
      return previous_path; 
    }

    // 3. behavior strategy - keep / change lanes
    // Currently it is implemented as a set of rules
    // considering the speeds of cars on target/current lanes
    // as well as their distances.

    // 3.1 collect road information
    int sdc_lane = map.find_lane(sdc.d);
    vector<double> frontcar_speeds;
    vector<double> frontcar_dists;
    for (auto lane = 0; lane <= map.RIGHTMOST_LANE; ++lane) {
      auto frontcar_idx = map.find_front_car_in_lane(sdc, peers, lane);
      // default value if no front car found
      double frontcar_speed = INF;
      double frontcar_dist = INF; 
      if (frontcar_idx != -1) {
        const PeerCar & frontcar(peers[frontcar_idx]);
        frontcar_speed = sqrt(frontcar.vx*frontcar.vx + frontcar.vy*frontcar.vy);
        frontcar_dist = frontcar.s - sdc.s;
      }
      frontcar_speeds.push_back(frontcar_speed);
      frontcar_dists.push_back(frontcar_dist);
    }
    // 3.2 decide to keep/change lane
    // a. if speed expection not met, consider possibility to change lane
    // b. if a suitable lane is found meeting safety criteria, then change the lane
    //    else stay in lane

    // Consider changing lanes when speed is slow
    if (frontcar_speeds[sdc_lane] <= 0.95 * MAX_SPEED) {

      // pick a target lane - only one lane crossing at a time is allowed 
      int target_lane = sdc_lane;
      if (sdc_lane == 0) target_lane = 1;
      else if (sdc_lane == 2) target_lane = 1;
      else /*sdc_lane == 1*/ {
        // prefer changing from 1 -> 0
        target_lane = 0;
        // unless lane 2 is faster and safe to change to
        if ((frontcar_speeds[2] > frontcar_speeds[0]) and 
              is_safe_to_change_lanes(sdc, peers, 2))
          target_lane = 2; // changing from 1 -> 2
        // or can bypass
        if (frontcar_dists[2] >= frontcar_dists[0] + 100)
          target_lane = 2;
      }
      cout << "Considering changing from " << sdc_lane << " to " << target_lane << endl;
      // check if target lane is better then current lane
      // higher speed on target lane and there is enough room to fit in the target lane
      bool is_target_better = (frontcar_speeds[target_lane] >= frontcar_speeds[sdc_lane] * 1.1);
      is_target_better &= (frontcar_dists[target_lane] + 30 >= frontcar_dists[sdc_lane]);
      // or there is bigger room on the target lane to bypass the current traffic
      is_target_better |= (frontcar_dists[target_lane] >= 100 + frontcar_dists[sdc_lane]);
      if (is_target_better) {
        return change_lane(previous_path, sdc, peers, target_lane - sdc_lane);
      }
    }

    // Keep lanes
    return keep_lane(previous_path, sdc, peers);
  }
private:
  Map map;
private:
  //============== Constants and Parameters =======================
  // time between two plannings
  const double INTERVAL = 0.02;
  // number of points in planned path
  const int PATH_LEN = 40;
  // speed in mph to m/s
  const double MPH2MPS = 0.447;
  // max speed of sdc 
  const double MAX_SPEED = 20; // m/s
  // distance between front cars to maintain
  const double SAFE_CAR_DISTANCE = 15; //meters
  // max steps to use from pervious plan for current plan
  const size_t MAX_PLAN_LOOKBACK = 20;
  //============================ States =============================
  // indicator state whether the car is in the middle of lane changing
  bool in_lane_change;
  // s path from previous planning
  Points previous_s_path;
private:
  //============== Helper functions ===============================

  // car's stay in lane behavior
  Path keep_lane(const Path & previous_path,
    const SelfDrivingCar & sdc,
    const vector<PeerCar> & peers) {
    
    // planning starts with path for s coordinates,
    // path for (x,y) will be derived based on it
    Points s_path;

    // collect information
    int sdc_lane = map.find_lane(sdc.d);
    int frontcar_idx = map.find_front_car_in_lane(sdc, peers, sdc_lane);

    // there are two conditions influencing the behaviors of keep-in-lane

    // whether there is a front car on the lane, it decides the target speed
    // of sdc
    bool is_free_to_go = true;
    if (frontcar_idx != -1) {
      const PeerCar & car(peers[frontcar_idx]);
      double response_time = INTERVAL * PATH_LEN;
      // assume the front car can brake all of sudden
      auto safe_dist = sdc.speed * MPH2MPS * response_time + SAFE_CAR_DISTANCE;
      auto dist = car.s - sdc.s;
      is_free_to_go = (dist >= safe_dist);
    }
    double target_speed = MAX_SPEED;
    if (! is_free_to_go ) {
      const PeerCar & car(peers[frontcar_idx]);
      double carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
      target_speed = carspeed * 0.7;
    }
    // whether there is a previous plan (if not, it means the car just starts)
    // it decides whether to follow previous path for smooth driving
    bool is_previous_plan_available = (previous_path.size() >= 2);
    double last_speed = -1; // last speed from previous plan
    int newplan_start  = -1; // step to start the new plan 
    if (! is_previous_plan_available) {
      s_path.push_back(sdc.s + 0.25); // first move
      last_speed = sdc.speed * MPH2MPS;
      newplan_start = 1;
    } else {
      auto n = previous_s_path.size();
      for (auto i = 0; i < min(MAX_PLAN_LOOKBACK, n); ++i) {
        s_path.push_back(previous_s_path[i]);
      }
      last_speed = (s_path.back() - s_path[s_path.size()-2]) / INTERVAL;
      newplan_start = s_path.size();
    }
    // continue the new plan
    double speed = last_speed;
    for (auto i = newplan_start; i < PATH_LEN; ++i) {
      speed = accelerate(speed, target_speed);
      s_path.push_back(s_path.back() + speed * INTERVAL);
    }

    // convert s path to (x, y) path
    Points x_path;
    Points y_path;
    // trajectory mapping from s -> (x, y)
    const auto & s2x(map.lane_s2x[sdc_lane]);
    const auto & s2y(map.lane_s2y[sdc_lane]);
    for (const auto & s: s_path) {
      x_path.push_back(s2x(s));
      y_path.push_back(s2y(s));
    }

    // remember the last plan
    previous_s_path = s_path;

    // return the path
    return Path(x_path, y_path);
  }

  // car's change lane behavior
  Path change_lane(const Path & previous_path,
    const SelfDrivingCar & sdc,
    const vector<PeerCar> & peers,
    int lane_change) {

    // collect information
    int sdc_lane = map.find_lane(sdc.d);
    int target_lane = sdc_lane + lane_change;
    int target_frontcar_idx = map.find_front_car_in_lane(sdc, peers, target_lane);
    int target_rearcar_idx = map.find_rear_car_in_lane(sdc, peers, target_lane);
    int current_frontcar_idx = map.find_front_car_in_lane(sdc, peers, sdc_lane);

    if (! is_safe_to_change_lanes(sdc, peers, target_lane)) {
      // keep in lane if the dynamic env changes  
      return keep_lane(previous_path, sdc, peers);
    } else {
      cout << "finished changing lanes ..." << endl;
      // build trajectory for lane changing, which is
      // a smooth mapping from s -> (x, y) across lanes

      // current and target lanes trajectory
      const auto & current_s2x{map.lane_s2x[sdc_lane]};
      const auto & current_s2y{map.lane_s2y[sdc_lane]};
      const auto & target_s2x{map.lane_s2x[target_lane]};
      const auto & target_s2y{map.lane_s2y[target_lane]};
      // lane change trajectory
      Trajectory lanechange_s2x;
      Trajectory lanechange_s2y;
      // build up way points for lane change trajectory
      Points lanechange_s;
      Points lanechange_x;
      Points lanechange_y;
      double start_s = sdc.s - 20; // on old lane
      double change_s = sdc.s + 15; // across at this step
      if (target_frontcar_idx != -1) // careful with front car on target
        change_s = min(change_s, peers[target_frontcar_idx].s);
      double end_s = change_s + 100; // to the new lane
      for (auto s = start_s; s <= change_s; ++s) {
        lanechange_s.push_back(s);
        lanechange_x.push_back(current_s2x(s));
        lanechange_y.push_back(current_s2y(s));
      }
      // 50 points for crossing the lane
      for (auto s = change_s + 50; s <= end_s; ++s) {
        lanechange_s.push_back(s);
        lanechange_x.push_back(target_s2x(s));
        lanechange_y.push_back(target_s2y(s)); 
      }
      lanechange_s2x.set_points(lanechange_s, lanechange_x);
      lanechange_s2y.set_points(lanechange_s, lanechange_y);

      // build the plan based on lane change trajectory
      vector<double> s_path;
      // pick the previous path
      bool is_previous_plan_available = (previous_path.size() >= 2);
      double last_speed = -1; // last speed from previous plan
      int newplan_start  = -1; // step to start the new plan 
      if (! is_previous_plan_available) {
        s_path.push_back(sdc.s + 0.25); // first move
        last_speed = sdc.speed * MPH2MPS;
        newplan_start = 1;
      } else {
        auto n = previous_s_path.size();
        for (auto i = 0; i < min(MAX_PLAN_LOOKBACK, n); ++i) {
          s_path.push_back(previous_s_path[i]); 
        }
        last_speed = (s_path.back() - s_path[s_path.size()-2]) / INTERVAL;
        newplan_start = s_path.size();
      }
      // continue to build new path - it is a longer plan
      // than usual because it usually takes longer time 
      // to change lanes
      double target_speed = min(last_speed * 1.02, MAX_SPEED);
      double speed = last_speed;
      for (auto i = newplan_start; i < PATH_LEN * 6; ++i) {
        speed = accelerate(speed, target_speed);
        s_path.push_back(s_path.back() + speed * INTERVAL);
      }

      // convert s path to (x, y) path
      Points x_path;
      Points y_path;
      for (const auto & s: s_path) {
        x_path.push_back(lanechange_s2x(s));
        y_path.push_back(lanechange_s2y(s));
      }

      // remember the last plan
      previous_s_path = s_path;

      // return the path
      in_lane_change = true;
      return Path(x_path, y_path);
    }
  }

  // car's modify speed bevhaivor - maintain acceleration and jerk constraints
  double accelerate(double current_speed, double target_speed) const {
    double diff = target_speed - current_speed;
    double delta = diff * 0.005;
    if (fabs(diff) <= 0.01) delta = 0;
    return current_speed + delta;
  }

  // if it is safe to change lane based on road condition
  bool is_safe_to_change_lanes(const SelfDrivingCar & sdc,
                               const vector<PeerCar> & peers,
                               int target_lane) const {
    // Check the speed and distances of three peer cars wrt SDC
    // namely "front car on current lane", "front and back cars on target lane"

    // collect road information
    int sdc_lane = map.find_lane(sdc.d);
    int target_frontcar_idx = map.find_front_car_in_lane(sdc, peers, target_lane);
    int target_rearcar_idx = map.find_rear_car_in_lane(sdc, peers, target_lane);
    int current_frontcar_idx = map.find_front_car_in_lane(sdc, peers, sdc_lane);

    // lanes must be valid
    bool is_safe_to_change = (target_lane >= 0) and (target_lane <= map.RIGHTMOST_LANE);
    // if the speed too slow, not safe to change lane within limited time
    is_safe_to_change &= (sdc.s >= 100) and (sdc.s <= 6900); // dont bother at beginning or end
    // check front car on target
    if (target_frontcar_idx != -1) {
      const PeerCar & car(peers[target_frontcar_idx]);
      auto carspeed = sqrt(car.vx * car.vx + car.vy + car.vy);
      double response_time = 4; //seconds
      auto safe_distance = fabs(sdc.speed * MPH2MPS - carspeed) * response_time + SAFE_CAR_DISTANCE;
      auto dist = car.s - sdc.s;
      is_safe_to_change &= (dist >= safe_distance);
      if (dist < safe_distance)
        cout << "Safe to change? dist to target_front=" << dist << " safe_dist=" << safe_distance << endl;
    }
    // check rear car on target
    if (target_rearcar_idx != -1) {
      const PeerCar & car(peers[target_rearcar_idx]);
      auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
      double response_time = 4; //seconds
      auto safe_distance = fabs(carspeed - sdc.speed*MPH2MPS) * response_time + SAFE_CAR_DISTANCE;
      auto dist = sdc.s - car.s;
      is_safe_to_change &= (dist >= safe_distance);
      if (dist < safe_distance)
        cout << "Safe to change? dist to target_rear=" << dist << " safe_dist=" << safe_distance << endl;
    }
    // check front car on current lane
    if (current_frontcar_idx != -1) {
      const PeerCar & car(peers[current_frontcar_idx]);
      auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
      double response_time = 3; //seconds
      auto safe_distance = fabs(sdc.speed*MPH2MPS - carspeed) * response_time + SAFE_CAR_DISTANCE;
      auto dist = car.s - sdc.s;
      is_safe_to_change &= (dist >= safe_distance);
      if (dist < safe_distance)
        cout << "Safe to change? dist to current_front=" << dist << " safe_dist=" << safe_distance << endl;
    }

    return is_safe_to_change;
  }
};

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
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
  }

  // construct the path planner
  PathPlanner planner(Map(map_waypoints_x,
                          map_waypoints_y,
                          map_waypoints_s,
                          map_waypoints_dx,
                          map_waypoints_dy));

  h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	json msgJson;

            // Collect information and plan path
            SelfDrivingCar sdc(car_x, car_y, car_s, car_d, car_yaw, car_speed);
            Path previous_path(previous_path_x, previous_path_y);
            vector<PeerCar> peers;
            for (const SensorData & sensor : sensor_fusion) {
              peers.push_back(PeerCar{sensor});
            }
            Path planned_path = planner.plan(previous_path, sdc, peers);
            cout << "car: " << " s=" << sdc.s << " d=" << sdc.d << endl;


          	// Pass the plan to controller
          	msgJson["next_x"] = planned_path.xs;
          	msgJson["next_y"] = planned_path.ys;

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
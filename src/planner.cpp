#include "env.h"
#include "planner.h"
#include "utility.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;

Planner::Planner(const Map & map,
    double target_speed): 
  map(map), TARGET_SPEED(target_speed){
    in_change_lane = false;
}

// update planner states with car and env states
tuple<Path, Path> Planner::plan(const SelfDrivingCar & sdc,
                   const vector<PeerCar> & peers,
                   const vector<double> & previous_x_path,
                   const vector<double> & previous_y_path,
                   const double end_path_s,
                   const double end_path_d)  {
  // // stop at the end
  // if (sdc.s >= 6900) {
  //   return make_tuple(Path(), Path());
  // }


  // House Keeping 
  // set target speed according to lane
  int sdc_lane = map.locate_lane(sdc.d);
  if (sdc_lane == 0) {
    TARGET_SPEED = 20.5; //m/s
  } else if (sdc_lane == 1) {
    TARGET_SPEED = 20; //m/s
  } else {
    TARGET_SPEED = 19.5; //m/s
  }

  // update previous_s_path from the last plan based on size of previous_x_path
  auto n_consumed = previous_s_path.size() - previous_x_path.size();
  previous_s_path.erase(previous_s_path.begin(), previous_s_path.begin() + n_consumed); 

  // check if in the middle of lane changing, if so, finish it
  if (in_change_lane) {
    if (previous_x_path.size() <= 35) {
    // if (is_lane_change_finished) {
      in_change_lane = false;
      cout << "FINISH LANE CHANGE" << endl;
    } 
    return make_tuple(previous_x_path, previous_y_path);
  }

  // Strategy for Self Driving
  // Contributing factors: 
  // speed of front car on each lane
  // distance of front car on each lane

  // collect front car speed infor for lane 0, 1, 2
  vector<double> frontcar_speeds;
  vector<double> frontcar_dists;
  for (int lane = 0; lane <= map.MAX_RIGHT_LANE; ++lane) {
    int frontcar_i = map.locate_front_car_in_lane(sdc, peers, lane);
    double car_speed = numeric_limits<double>::infinity(); // no front car
    double car_dist = numeric_limits<double>::infinity();
    if (frontcar_i != -1) {
      const PeerCar & frontcar(peers[frontcar_i]);
      car_speed = sqrt(frontcar.vx*frontcar.vx + frontcar.vy*frontcar.vy);
      car_dist = frontcar.s - sdc.s;
    }
    frontcar_speeds.push_back(car_speed);
    frontcar_dists.push_back(car_dist);
  }
  
  // DEFAULT behavior - stay in lane
  bool try_to_change_lane = false;
  vector<vector<double>> result;

  if (frontcar_speeds[sdc_lane] <= 0.9 * TARGET_SPEED) {  // consider changing lanes 
    // cout << "CONSIDER CHANGING LANES FOR SPEED" << endl;
    
    
    // pick target lane
    int target_lane = -1;
    if (sdc_lane == 0) {
      target_lane = 1; 
    } else if (sdc_lane == 1) {
      target_lane = 0;
      if ((frontcar_speeds[2] > frontcar_speeds[0]) && is_safe_to_change_lanes(2, sdc, peers))
        target_lane = 2;
    } else /*sdc_lane == 2*/ {
      target_lane = 1;
    }
    // try to change lane
    if ( ( (frontcar_speeds[target_lane] >= frontcar_speeds[sdc_lane]*1.1) &&
            (frontcar_dists[target_lane] +30 >= frontcar_dists[sdc_lane]) ) 
      || (frontcar_dists[target_lane] >= 100 + frontcar_dists[sdc_lane]) ) {
      try_to_change_lane = true;
      cout << "TRY TO CHANGE FROM LANE " << sdc_lane << " LANE " << target_lane << endl;
      result = change_lane(sdc, peers,
                          previous_x_path, previous_y_path,
                          end_path_s, end_path_d, target_lane - sdc_lane);
    }
  } 
  if (! try_to_change_lane) {
    result = keep_lane(sdc, peers,
                      previous_x_path, previous_y_path,
                      end_path_s, end_path_d);
  }
  

  Path x_path = result[0];
  Path y_path = result[1];
  Path s_path = result[2];

  previous_s_path = s_path; // remember the plan

  return make_tuple(x_path, y_path);
}




vector<Path> Planner::keep_lane(const SelfDrivingCar & sdc,
                 const vector<PeerCar> & peers,
                 const vector<double> & previous_x_path,
                 const vector<double> & previous_y_path,
                 const double end_path_s,
                 const double end_path_d) {

  vector<double> s_path;

  int sdc_lane = map.locate_lane(sdc.d);
  int front_car_idx = map.locate_front_car_in_lane(sdc, peers, sdc_lane);
  bool free_to_go = true; // far away from previous car
  if (front_car_idx != -1) { //car ahead on the same lane
    const PeerCar & frontcar = peers[front_car_idx];
    double car_distance = distance(sdc.x, sdc.y, frontcar.x, frontcar.y);
    double safe_buffer = sdc.speed *MPH2MPS * INTERVAL * PATH_LENGTH + SAFE_DISTANCE;
    if (car_distance <= safe_buffer) {
      free_to_go = false;
    }
  } 

  bool previous_plan_available = (previous_s_path.size() > 0);
  double speed = 0;
  int new_start = 1;
  if (!previous_plan_available) {
    s_path.push_back(sdc.s + 0.25); // first move
    speed = sdc.speed * MPH2MPS; // m/s
    new_start = 1;
  } else {
    int n_previous_path = previous_s_path.size();
    for (auto i = 0; i < min(20, n_previous_path); ++i) {
      s_path.push_back(previous_s_path[i]); // from last plan
    }    
    speed = (s_path[s_path.size()-1] - s_path[s_path.size()-2]) / INTERVAL;
    new_start = s_path.size();
  }

  double target_speed = 0;

  if (free_to_go) {
    target_speed = TARGET_SPEED;
  } else /*set according to front car*/ {
    // cout << "CAR AHEAD!!!" << endl;
    const PeerCar & frontcar = peers[front_car_idx];
    double frontcar_speed = sqrt(frontcar.vx*frontcar.vx + 
                            frontcar.vy*frontcar.vy);
    target_speed = frontcar_speed * 0.7;
  }

  // construct new path plan
  for (auto i = new_start; i < PATH_LENGTH; ++i) {
    speed = accelerate(speed, target_speed);
    auto s = s_path.back() + speed * INTERVAL;
    // if (s > 6914.149) s -= 6914.149;
    s_path.push_back(s);
  }

  vector<double> x_path;
  vector<double> y_path;
  auto & s2x = map.lane_s2x[sdc_lane];
  auto & s2y = map.lane_s2y[sdc_lane];
  for (auto s : s_path) {
    // auto xy = getXY(s, sdc.d, map.s, map.x, map.y);
    // auto x = xy[0];
    // auto y = xy[1];
    auto x = s2x(s);
    auto y = s2y(s);

    // cout << "sdc d vs dd:" << sdc.d << " : " << sd[1] << endl; 
    x_path.push_back(x);
    y_path.push_back(y);
  }


  return vector<Path>{x_path, y_path, s_path};
}

double Planner::accelerate(double current_speed, double target_speed) const {
  // cout << "DEBUG: " << "current=" << current_speed << " target=" << target_speed << endl;
  double diff = target_speed - current_speed;
  double acc = diff * 0.005;
  if (fabs(diff) <= 0.01) acc = 0;
  return current_speed + acc;
  // return min(current_speed + acc, target_speed);
}

vector<Path> Planner::change_lane(
  const SelfDrivingCar & sdc,
  const vector<PeerCar> & peers,
  const vector<double> & previous_x_path,
  const vector<double> & previous_y_path,
  const double end_path_s,
  const double end_path_d,
  double lane_change) {
  
  int sdc_lane = map.locate_lane(sdc.d);
  int target_lane = sdc_lane + lane_change;
  int idx_front_car_on_target = map.locate_front_car_in_lane(sdc, peers, target_lane);
  int idx_back_car_on_target = map.locate_back_car_in_lane(sdc, peers, target_lane);
  int idx_front_car_on_current = map.locate_front_car_in_lane(sdc, peers, sdc_lane);



  if (!is_safe_to_change_lanes(target_lane, sdc, peers)) { // already the leftmost lane
    cout << "NOT SAFE to change" << endl;
    return keep_lane(sdc, peers,
                    previous_x_path, previous_y_path,
                    end_path_s, end_path_d);
  } else {
    auto & current_s2x = map.lane_s2x[sdc_lane];
    auto & current_s2y = map.lane_s2y[sdc_lane]; 
    auto & target_s2x = map.lane_s2x[target_lane];
    auto & target_s2y = map.lane_s2y[target_lane]; 
    tk::spline lane_change_s2x;
    tk::spline lane_change_s2y;
    // collect points for lane change trajectory
    vector<double> trajectory_s;
    vector<double> trajectory_x;
    vector<double> trajectory_y;
    double start_s = sdc.s - 20;
    double change_s = sdc.s + 15; //20
    if (idx_front_car_on_target != -1) {
      change_s = min(change_s, peers[idx_front_car_on_target].s);
    }
    double end_s = change_s + 100;
    double s = start_s;
    while (s <= change_s) {
      trajectory_s.push_back(s);
      trajectory_x.push_back(current_s2x(s));
      trajectory_y.push_back(current_s2y(s));
      ++s;
    }
    s += 50; //30
    while (s <= end_s) {
      trajectory_s.push_back(s);
      trajectory_x.push_back(target_s2x(s));
      trajectory_y.push_back(target_s2y(s));
      ++s;
    }
    lane_change_s2x.set_points(trajectory_s, trajectory_x);
    lane_change_s2y.set_points(trajectory_s, trajectory_y);

    // return the planned path
    vector<double> s_path;
    vector<double> x_path;
    vector<double> y_path;

    bool previous_plan_available = (previous_s_path.size() > 0);
    double speed = 0;
    int new_start = 1;
    if (!previous_plan_available) {
      s_path.push_back(sdc.s + 0.25); // first move
      speed = sdc.speed * MPH2MPS; // m/s
      new_start = 1;
    } else {
      int n_previous_path = previous_s_path.size();
      for (auto i = 0; i < min(20, n_previous_path); ++i) {
        s_path.push_back(previous_s_path[i]); // from last plan
      }    
      speed = (s_path[s_path.size()-1] - s_path[s_path.size()-2]) / INTERVAL;
      new_start = s_path.size();
    }
    double target_speed = min(TARGET_SPEED, 20.0);
    for (auto i = new_start; i < PATH_LENGTH*6; ++i) {
      speed = accelerate(speed, target_speed);
      auto s = s_path.back() + speed * INTERVAL;
      // if (s >= 6914.149) s -= 6914.149;
      s_path.push_back(s);
    }

    for (auto s : s_path) {
      auto x = lane_change_s2x(s);
      auto y = lane_change_s2y(s);
      x_path.push_back(x);
      y_path.push_back(y);
    }


    in_change_lane = true;
    return vector<Path>{x_path, y_path, s_path};
  }
}

bool Planner::is_safe_to_change_lanes(int target_lane,
                      const SelfDrivingCar & sdc,
                      const vector<PeerCar> & peers) const {
  int sdc_lane = map.locate_lane(sdc.d);
  int idx_front_car_on_target = map.locate_front_car_in_lane(sdc, peers, target_lane);
  int idx_back_car_on_target = map.locate_back_car_in_lane(sdc, peers, target_lane);
  int idx_front_car_on_current = map.locate_front_car_in_lane(sdc, peers, sdc_lane);
  bool is_safe_to_change = (target_lane >= 0) && (target_lane <= map.MAX_RIGHT_LANE);
  is_safe_to_change = is_safe_to_change && (sdc.speed >= 35/*MPH*/);
  // is_safe_to_change = is_safe_to_change && (fabs(sdc.yaw) <= 0.5);
  const int SAFE_DIST_FOR_LANE_CHANGE = 40;
  if (idx_front_car_on_target != -1) { // check dist with front car on target
    const PeerCar & car(peers[idx_front_car_on_target]);
    double carspeed = sqrt(car.vx*car.vx+car.vy*car.vy);
    double safe_distance = fabs(sdc.speed*MPH2MPS - carspeed) * 4 + SAFE_DISTANCE;
    double dist_with_front = car.s - sdc.s;
    is_safe_to_change = is_safe_to_change && (dist_with_front >= safe_distance);

    // is_safe_to_change = is_safe_to_change && (dist_with_front >= SAFE_DIST_FOR_LANE_CHANGE-15);
    cout << "dist_with_front:" << dist_with_front<< " safe dist: " << safe_distance << endl;
  }
  if (idx_back_car_on_target != -1) { // check dist with back car on target
    const PeerCar & car(peers[idx_back_car_on_target]);
    double carspeed = sqrt(car.vx*car.vx+car.vy*car.vy);
    double safe_distance = fabs(carspeed - sdc.speed*MPH2MPS) * 6 + SAFE_DISTANCE;
    double dist_with_back = sdc.s - car.s;
    is_safe_to_change = is_safe_to_change && (dist_with_back >= safe_distance);
    
    // is_safe_to_change = is_safe_to_change && (dist_with_back >= SAFE_DIST_FOR_LANE_CHANGE-10);
    cout << "dist_with_back:" << dist_with_back << " safe dist: " << safe_distance << endl;
    cout << "carspeed:" << carspeed << " sdc speed:" << sdc.speed << endl;
  }
  if (idx_front_car_on_current != -1) {
    const PeerCar & car(peers[idx_front_car_on_current]);
    double carspeed = sqrt(car.vx*car.vx+car.vy*car.vy);
    double safe_distance = fabs(sdc.speed*MPH2MPS - carspeed) * 3 + SAFE_DISTANCE;
    double dist_with_current_front = car.s - sdc.s;
    is_safe_to_change = is_safe_to_change && (dist_with_current_front >= safe_distance);

    // is_safe_to_change = is_safe_to_change && (dist_with_current_front >= SAFE_DIST_FOR_LANE_CHANGE/4);
    cout << "dist_with_current_front:" << dist_with_current_front << " safe dist: " << safe_distance << endl;
  }

  return is_safe_to_change;
}


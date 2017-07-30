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
}

// update planner states with car and env states
Path Planner::plan(const SelfDrivingCar & sdc,
                   const vector<PeerCar> & peers,
                   const vector<double> & previous_x_path,
                   const vector<double> & previous_y_path,
                   const double end_path_s,
                   const double end_path_d) const {
  

  // return go_straight(sdc, peers, previous_x_path, previous_y_path, end_path_s, end_path_d);

  return keep_lane(sdc, peers, previous_x_path, previous_y_path, end_path_s, end_path_d);
}

Path Planner::go_straight(const SelfDrivingCar & sdc,
                   const vector<PeerCar> & peers,
                   const vector<double> & previous_x_path,
                   const vector<double> & previous_y_path,
                   const double end_path_s,
                   const double end_path_d) const {

  vector<double> x_path;
  vector<double> y_path;
  auto car_x = sdc.x;
  auto car_y = sdc.y;
  auto car_yaw = sdc.yaw;

  double dist_inc = 0.5;
  for(int i = 0; i < 30; i++)
  {
    x_path.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
    y_path.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
  }

  return make_tuple(x_path, y_path);
}

Path Planner::keep_lane(const SelfDrivingCar & sdc,
                 const vector<PeerCar> & peers,
                 const vector<double> & previous_x_path,
                 const vector<double> & previous_y_path,
                 const double end_path_s,
                 const double end_path_d) const {

  vector<double> x_path;
  vector<double> y_path;

  // find the boundary of planned path
  int sdc_lane = map.locate_lane(sdc.d);
  int front_car_idx = map.locate_front_car_in_lane(sdc, peers, sdc_lane);
  bool free_to_go = true;//(front_car_idx == -1);

  double s_start = sdc.s;
  double d = sdc.d;

  if (free_to_go) {
    double sdc_speed = min(sdc.speed * MPH2MPS, TARGET_SPEED);
    double T = PATH_DURATION;
    double speed = min(TARGET_SPEED, sdc_speed + MAX_ACCELERATION * T);
    double acceleration = (speed - sdc_speed) / T;
    double s_end = s_start + sdc_speed * T + .5 * acceleration * T * T;
    cout << "HIHI:" << sdc_speed << "," << speed << "," << acceleration << "," << s_end << endl;
    for (double t = 0; t <= T; t += INTERVAL) {
      double s = s_start + 0.3*t*50;//sdc_speed * t + .5 * acceleration * t * t;
      double x = map.lane_s2x[sdc_lane](s);
      double y = map.lane_s2y[sdc_lane](s);
      x_path.push_back(x);
      y_path.push_back(y);
    } 
  } else /*there is car ahead*/ {
    const PeerCar & frontcar = peers[front_car_idx];
  }


  return make_tuple(x_path, y_path);
}

vector<double> Planner::generate_jmt_coeffs(
                        const vector<double> & start, 
                        const vector<double> & end, 
                        double T) const {
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
          3*T*T, 4*T*T*T,5*T*T*T*T,
          6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> coeffs = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
      coeffs.push_back(C.data()[i]);
  }
  
    return coeffs;
}
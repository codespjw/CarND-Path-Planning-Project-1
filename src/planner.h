// Path Planner and relative classes

#ifndef _PLANNER_H
#define _PLANNER_H

#include <vector>
#include <iostream>
#include <tuple>
#include "env.h"

using namespace std;


// path points in either (xs, ys) coordinates
using Path = tuple<vector<double>, vector<double>>; 

class Planner {
public:

	Planner(const Map & map,
    double target_speed= 17/*meter-per-second*/ );
	virtual ~Planner(){};

  // plan the path for self driving car,
  // return way points in (x, y) map coordinates 
  Path plan(const SelfDrivingCar & sdc,
            const vector<PeerCar> & peers,
            const vector<double> & previous_x_path,
            const vector<double> & previous_y_path,
            const double end_path_s,
            const double end_path_d) const;

public:
  const Map & map;

private:
  // planner update interval
  const double INTERVAL = 0.02; /*seconds*/
  const double SAFE_DISTANCE = 10; /*meters*/
  const double TARGET_SPEED; /*meters per second*/
  const double PATH_DURATION = 1; /*second*/
  const double MAX_ACCELERATION = 8; /*ms2*/
  const double MAX_JERK = 40; /*ms3*/
  const double MPH2MPS = 0.447; /*MPH to meter per second*/

private:
  Path go_straight(const SelfDrivingCar & sdc,
                   const vector<PeerCar> & peers,
                   const vector<double> & previous_x_path,
                   const vector<double> & previous_y_path,
                   const double end_path_s,
                   const double end_path_d) const;
  Path keep_lane(const SelfDrivingCar & sdc,
                 const vector<PeerCar> & peers,
                 const vector<double> & previous_x_path,
                 const vector<double> & previous_y_path,
                 const double end_path_s,
                 const double end_path_d) const;
  // generate jerk minimizging trajectory
  vector<double> /*coefficients*/
  generate_jmt_coeffs(const vector<double> & start, /*[position, speed, acce]*/ 
                      const vector<double> & end,
                      double T /*duration*/) const;
};

#endif
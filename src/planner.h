// Path Planner and relative classes

#ifndef _PLANNER_H
#define _PLANNER_H

#include <vector>
#include <iostream>
#include <tuple>
#include "env.h"

using namespace std;


// path points in either (xs, ys) coordinates
using Path = vector<double>; 

class Planner {
public:

	Planner(const Map & map,
    double target_speed= 20/*meter-per-second*/ );
	virtual ~Planner(){};

  // plan the path for self driving car,
  // return way points in (x, y) map coordinates 
  tuple<Path, Path> plan(const SelfDrivingCar & sdc,
            const vector<PeerCar> & peers,
            const vector<double> & previous_x_path,
            const vector<double> & previous_y_path,
            const double end_path_s,
            const double end_path_d) ;

public:
  const Map & map;

private:
  // planner update interval
  const double INTERVAL = 0.02; /*seconds*/
  const double SAFE_DISTANCE = 15; /*meters*/
  double TARGET_SPEED; /*meters per second*/
  const double PATH_LENGTH = 40; /*points*/
  const double MAX_ACCELERATION = 8; /*ms2*/
  const double MAX_JERK = 40; /*ms3*/
  const double MPH2MPS = 0.447; /*MPH to meter per second*/

  vector<double> previous_s_path;
public:
  int state;
  bool in_change_lane;

private:
  vector<Path> keep_lane(const SelfDrivingCar & sdc,
                 const vector<PeerCar> & peers,
                 const vector<double> & previous_x_path,
                 const vector<double> & previous_y_path,
                 const double end_path_s,
                 const double end_path_d) ;

  vector<Path> change_lane(
    const SelfDrivingCar & sdc,
    const vector<PeerCar> & peers,
    const vector<double> & previous_x_path,
    const vector<double> & previous_y_path,
    const double end_path_s,
    const double end_path_d,
    double lane_change);

  double accelerate(double current_speed, double target_speed) const;
};

#endif
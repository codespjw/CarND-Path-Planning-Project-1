// Environment classes such as map, self-driving-cars and other cars

#ifndef _ENV_H
#define _ENV_H

#include <vector>
#include <iostream>
#include "spline.h"


using namespace std;


// Self driving car status
struct SelfDrivingCar {
public:

  SelfDrivingCar(double x, double y,
                 double s, double d,
                 double yaw, double speed):
                 x(x), y(y),
                 s(s), d(d),
                 yaw(yaw), speed(speed) {};

  double x; // map coordinate
  double y; // map coordinate
  double s; // frenet coordinate
  double d; // frenet coordinate
  double yaw; // bearing
  double speed; // speed
};

// other cars on the road
struct PeerCar {
public:
  PeerCar(int id, double x, double y,
          double vx, double vy,
          double s, double d):
          id(id), x(x), y(y),
          vx(vx), vy(vy),
          s(s), d(d) {}

  int id;
  double x; // map coordinate
  double y; // map coordinate
  double vx; // speed
  double vy; // speed
  double s; // frenet coordinate
  double d; // frenet coordinate
};

// Map information with way points
class Map {
public:

  Map(const vector<double> & waypoint_x,
      const vector<double> & waypoint_y,
      const vector<double> & waypoint_s,
      const vector<double> & waypoint_dx,
      const vector<double> & waypoint_dy);
  virtual ~Map() {};

  // locate the lane index (on the right)
  // of a car based on its frenet d coord.
  int locate_lane(double car_d) const;

  // find the closet car in front of sdc in a lane,
  // return car index in peer_cars (-1 if there is no front car on target lane)
  int locate_front_car_in_lane(const SelfDrivingCar & sdc, 
                              const vector<PeerCar> & peer_cars,
                              int target_lane) const;

public:
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;

  const int LANE_WIDTH = 4; /*meters*/
  const int MAX_RIGHT_LANE = 2; /*lane 0, 1, 2*/

  // spline mapping from s to x,y for each lane 0, 1, 2
  vector<tk::spline> lane_s2x;
  vector<tk::spline> lane_s2y;
};

#endif
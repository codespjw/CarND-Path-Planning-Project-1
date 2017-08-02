
#include "env.h"
#include "utility.h"
#include <limits>
#include <cmath>
#include <random>

using namespace std;

Map::Map(const vector<double> & waypoint_x,
         const vector<double> & waypoint_y,
         const vector<double> & waypoint_s,
         const vector<double> & waypoint_dx,
         const vector<double> & waypoint_dy):
         x(waypoint_x), y(waypoint_y),
         s(waypoint_s), dx(waypoint_dx),
         dy(waypoint_dy) {

  auto n = waypoint_x.size();
  double HALF_LANE = LANE_WIDTH / 2;
  double lower = -0.5;
  double upper = -0.1;
  uniform_real_distribution<double> unif(lower, upper);
  default_random_engine re(random_device{}()); 
  for (int lane = 0; lane <= MAX_RIGHT_LANE; ++lane) {
    vector<double> lane_x;
    vector<double> lane_y;
    vector<double> lane_s;
    for (auto i = 0; i < n; ++i) {
      // jittering a bit because the resolution of waypoints are not good enough for 
      // keeping in lane
      double jitter = unif(re);
      lane_x.push_back(waypoint_x[i] + waypoint_dx[i] * (HALF_LANE+lane*LANE_WIDTH+jitter));
      lane_y.push_back(waypoint_y[i] + waypoint_dy[i] * (HALF_LANE+lane*LANE_WIDTH+jitter));
      lane_s.push_back(waypoint_s[i]);
      // auto xy = getXY(waypoint_s[i], HALF_LANE+lane*LANE_WIDTH, s, x, y);
      // lane_x.push_back(xy[0]);
      // lane_y.push_back(xy[1]);
    }


    // quick fix for inaccrurate spline model of loops
    // a better solution is to use local segment models
    for (auto i = 1; i < n/3; ++i) {
      double jitter = unif(re);
      lane_x.push_back(waypoint_x[i] + waypoint_dx[i] * (HALF_LANE+lane*LANE_WIDTH+jitter));
      lane_y.push_back(waypoint_y[i] + waypoint_dy[i] * (HALF_LANE+lane*LANE_WIDTH+jitter));
      lane_s.push_back(waypoint_s[i]+waypoint_s[n-1]);
    }
    tk::spline s2x; s2x.set_points(lane_s, lane_x);
    tk::spline s2y; s2y.set_points(lane_s, lane_y);
    lane_s2x.push_back(s2x);
    lane_s2y.push_back(s2y);
  }
}

int Map::locate_lane(double car_d) const {
  // return 0 based lane index
  int lane = floor(car_d / LANE_WIDTH);
  // return MAX_RIGHT_LANE when out of lane
  lane = min(MAX_RIGHT_LANE, lane);
  return lane;
}

int Map::locate_front_car_in_lane(const SelfDrivingCar & sdc, 
                              const vector<PeerCar> & peer_cars,
                              int target_lane) const {
  int peer_car_idx = -1; // in case nothing found
  double dist_with_sdc = numeric_limits<double>::infinity();

  for (auto icar = 0; icar < peer_cars.size(); ++icar) {
    const PeerCar & car = peer_cars[icar];
    int car_lane = locate_lane(car.d);
    if ( (car_lane == target_lane) && (car.s > sdc.s)  ) {
      double dist = car.s - sdc.s;
      if (dist < dist_with_sdc) {
        peer_car_idx = icar;
        dist_with_sdc = dist;
      }
    }
  }
  
  return peer_car_idx;
}

int Map::locate_back_car_in_lane(const SelfDrivingCar & sdc, 
                              const vector<PeerCar> & peer_cars,
                              int target_lane) const {
  int peer_car_idx = -1; // in case nothing found
  double dist_with_sdc = numeric_limits<double>::infinity();

  for (auto icar = 0; icar < peer_cars.size(); ++icar) {
    const PeerCar & car = peer_cars[icar];
    int car_lane = locate_lane(car.d);
    if ( (car_lane == target_lane) && (car.s < sdc.s)  ) {
      double dist = sdc.s - car.s;
      if (dist < dist_with_sdc) {
        peer_car_idx = icar;
        dist_with_sdc = dist;
      }
    }
  }
  
  return peer_car_idx;
}
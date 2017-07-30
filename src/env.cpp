
#include "env.h"
#include <limits>
#include <cmath>

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
  auto HALF_LANE = LANE_WIDTH / 2;
  for (int lane = 0; lane <= MAX_RIGHT_LANE; ++lane) {
    vector<double> lane_x;
    vector<double> lane_y;
    for (auto i = 0; i < n; ++i) {
      lane_x.push_back(waypoint_x[i] + waypoint_dx[i] * (HALF_LANE+lane*LANE_WIDTH));
      lane_y.push_back(waypoint_y[i] + waypoint_dy[i] * (HALF_LANE+lane*LANE_WIDTH));
      
    }
    tk::spline s2x; s2x.set_points(waypoint_s, lane_x);
    tk::spline s2y; s2y.set_points(waypoint_s, lane_y);
    lane_s2x.push_back(s2x);
    lane_s2y.push_back(s2y);
  }
}

int Map::locate_lane(double car_d) const {
  // return 0 based lane index
  int lane = floor(static_cast<int>(car_d) / LANE_WIDTH);
  // return -1 when out of lane
  if (lane > MAX_RIGHT_LANE) {
    lane = -1;
  }
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
    if ( (car_lane == target_lane) && (car.s >= sdc.s)  ) {
      double dist = car.s - sdc.s;
      if (dist < dist_with_sdc) {
        peer_car_idx = icar;
        dist_with_sdc = dist;
      }
    }
  }
  
  return peer_car_idx;
}
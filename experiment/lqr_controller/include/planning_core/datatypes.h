//
// Created by yenkn on 2020/10/25.
//

#ifndef SRC_DATATYPES_H
#define SRC_DATATYPES_H

#include <vector>
#include <chrono>
#include <algorithm>
#include <cmath>

using namespace std::chrono;

struct TrajectoryPoint {
  double x, y, theta, velocity, time;
};

typedef std::vector<TrajectoryPoint> Trajectory;

inline double current_timestamp() {
  return ((double)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000);
}

inline Trajectory::const_iterator trajectory_time_lower_bound(const Trajectory &traj, double time) {
  if(time >= traj.back().time) {
    return traj.end() - 1;
  }
  return std::lower_bound(traj.begin(), traj.end(), time, [](const TrajectoryPoint &t, double time) { return t.time < time; });
}

inline size_t trajectory_nearest(const Trajectory &traj, double x, double y) {
  double min_distance = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for(size_t i = 0; i < traj.size(); i++) {
    double distance = hypot(x - traj[i].x, y - traj[i].y);
    if(distance < min_distance) {
      min_distance = distance;
      min_index = i;
    }
  }
  return min_index;
}

struct ChassisCommand {
  double throttle;
  double steering;
};

#endif //SRC_DATATYPES_H

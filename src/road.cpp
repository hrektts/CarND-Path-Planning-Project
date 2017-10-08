#include "road.h"
#include <math.h>
#include "common.h"

Road::Road(int num_lane,
           double lane_width,
           const vector<double> &map_xs,
           const vector<double> &map_ys,
           const vector<double> &map_ss,
           const vector<double> &map_dxs,
           const vector<double> &map_dys) {
    this->num_lane = num_lane;
    this->lane_width = lane_width;
    this->map_xs = map_xs;
    this->map_ys = map_ys;
    this->map_ss = map_ss;
    this->map_dxs = map_dxs;
    this->map_dys = map_dys;
}

Road::~Road() {}

int Road::ClosestWaypoint(double x, double y) {
    double closestLen = 100000;  // large number
    int closestWaypoint = 0;

    for (size_t i = 0; i < map_xs.size(); i++) {
        double map_x = map_xs[i];
        double map_y = map_ys[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

int Road::DToLane(double d) const {
    int lane = static_cast<int>(floor(d / lane_width));
    if (lane >= num_lane) {
        return -1;
    } else {
        return lane;
    }
}

double Road::LaneToD(int lane) const {
    return static_cast<double>(lane) * lane_width + lane_width / 2.;
}

int Road::NextWaypoint(double x, double y, double theta) {
    int closestWaypoint = ClosestWaypoint(x, y);

    double map_x = map_xs[closestWaypoint];
    double map_y = map_ys[closestWaypoint];

    double heading = atan2((map_y-y), (map_x-x));

    double angle = abs(theta-heading);

    if (angle > pi()/4) {
        closestWaypoint++;
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Road::GetFrenet(double x, double y, double theta) {
    int next_wp = NextWaypoint(x, y, theta);

    int prev_wp;
    prev_wp = next_wp-1;
    if (next_wp == 0) {
        prev_wp  = map_xs.size()-1;
    }

    double n_x = map_xs[next_wp]-map_xs[prev_wp];
    double n_y = map_ys[next_wp]-map_ys[prev_wp];
    double x_x = x - map_xs[prev_wp];
    double x_y = y - map_ys[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point
    double center_x = 1000-map_xs[prev_wp];
    double center_y = 2000-map_ys[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(map_xs[i], map_ys[i], map_xs[i+1], map_ys[i+1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Road::GetXY(double s, double d) const {
    int prev_wp = -1;

    while (s > map_ss[prev_wp+1] &&
           (prev_wp < static_cast<int>(map_ss.size()-1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%map_xs.size();

    double heading = atan2((map_ys[wp2]-map_ys[prev_wp]), (map_xs[wp2]-map_xs[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-map_ss[prev_wp]);

    double seg_x = map_xs[prev_wp]+seg_s*cos(heading);
    double seg_y = map_ys[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x, y};
}
